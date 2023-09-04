#include "player.h"

#include "cmsis_device.h"
#include "lib_stm32.h"
#include "logging.h"

#include <array>
#include <limits>

extern usart_logger_t logger;

#define DDS_FREQ 48000 // keep in sync with python generator

const stm32_lib::gpio::gpio_pin_t pin_dac(GPIOA, 4);

typedef uint16_t dds_value_t;
#include "lookup_tables.cc.h"

typedef uint16_t note_id_t;

typedef dds_value_t (*lookup_func_t)(uint32_t);

struct instr_info_t {
	const lookup_func_t lookup_func;
	instr_info_t(lookup_func_t f) : lookup_func(f) {}
};


dds_value_t lookup_sq(uint32_t x)
{
	return (x >> 31) ? std::numeric_limits<dds_value_t>::max() : 0;
}

dds_value_t lookup_sin3(uint32_t x)
{
	return lookup_table_sin3[x >> 29];
}

dds_value_t lookup_sin4(uint32_t x)
{
	return lookup_table_sin4[x >> 28];
}

dds_value_t lookup_sin5(uint32_t x)
{
	return lookup_table_sin5[x >> 27];
}

dds_value_t lookup_sin10(uint32_t x)
{
	return lookup_table_sin10[x >> 22];
}

dds_value_t lookup_sin12(uint32_t x)
{
	return lookup_table_sin12[x >> 20];
}

const std::array<instr_info_t, 6> g_instr_info{
	instr_info_t{&lookup_sq},
	instr_info_t{&lookup_sin3},
	instr_info_t{&lookup_sin4},
	instr_info_t{&lookup_sin5},
	instr_info_t{&lookup_sin10},
	instr_info_t{&lookup_sin12}
};

class voice_t {
	uint32_t phase_acc;
	uint32_t phase_inc;
	lookup_func_t lookup_func;
	note_id_t note_id = 0;
	uint32_t dds_ticks = 0;
	friend size_t find_free_voice();
public:
	voice_t() { reset(); }

	void set(lookup_func_t f, uint32_t inc, uint32_t dt, note_id_t ni)
	{
		phase_acc = 0;
		lookup_func = f;
		phase_inc = inc;
		dds_ticks = (dt ? dt : 1);
		note_id = ni;
	}

	void reset() { set(nullptr, 0, 0, 0); }
	void maybe_reset(note_id_t ni)
	{
		if (is_busy() && note_id == ni) {
			reset();
		}
	}

	bool is_busy() const { return lookup_func != nullptr; }

	dds_value_t tick()
	{
		const auto r = lookup_func(phase_acc);
		phase_acc += phase_inc;
		if (--dds_ticks == 0) {
			reset();
		}
		return r;
	}
};
std::array<voice_t, 8> g_voices;

size_t find_free_voice()
{
	size_t idx = 0;
	for (size_t i = 0; i < g_voices.size(); ++i) {
		if (!g_voices[i].is_busy()) {
			return i;
		}
		if (g_voices[i].note_id < g_voices[idx].note_id) {
			idx = i;
		}
	}
	g_voices[idx].reset();
	return idx;
}


typedef uint32_t queue_item_t;

queue_item_t queue_item_encode(notes::sym_t n, notes::duration_t d, notes::instrument_t instr)
{
	return
		(static_cast<uint32_t>(instr) << 16)
		| (static_cast<uint32_t>(d) << 8)
		| static_cast<uint32_t>(n);
}

void queue_item_decode(queue_item_t item, notes::sym_t& n, notes::duration_t& d, notes::instrument_t& instr)
{
	n = static_cast<notes::sym_t>(item & 0xff);
	d = static_cast<notes::duration_t>((item >> 8) & 0xff);
	instr = static_cast<notes::instrument_t>((item >> 16) & 0xff);
}


void player::enqueue_note(QueueHandle_t queue_handle, notes::sym_t n, notes::duration_t d, notes::instrument_t instr)
{
	const queue_item_t item = queue_item_encode(n, d, instr);
	xQueueSend(queue_handle, &item, 0);
}


struct queue_data_t {
	std::array<queue_item_t, 16> buffer;
	StaticQueue_t q;
} queue_data;

QueueHandle_t player::create_queue()
{
	return xQueueCreateStatic(
		queue_data.buffer.size(),
		sizeof(queue_item_t),
		reinterpret_cast<uint8_t*>(queue_data.buffer.data()),
		&queue_data.q
	);
}


struct play_task_data_t {
	std::array<StackType_t, 128> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;

	QueueHandle_t arg_queue_handle = nullptr;
} play_task_data;


#define TIM_DAC TIM6
extern "C" __attribute__ ((interrupt)) void IntHandler_Timer6()
{
	const auto sr = TIM_DAC->SR;
	if (!(sr & TIM_SR_UIF)) {
		return;
	}
	TIM_DAC->SR = sr & ~TIM_SR_UIF;

	uint32_t value = 0;
	uint8_t voices_count = 0;
	for (auto& v : g_voices) {
		if (v.is_busy()) {
			++voices_count;
			value += uint32_t(v.tick());
		}
	}
	if (voices_count == 0) {
		return;
	}
	value /= voices_count;
	DAC->DHR12R1 = value >> 4; // MSB 16bit -> 12bit
	//DAC->DHR12R1 = value >> 5;
}


#define CLOCK_SPEED configCPU_CLOCK_HZ

#define TIM_DAC TIM6
void dac_init()
{
	// ensure there is no truncation in calculation
	static_assert((CLOCK_SPEED % DDS_FREQ) == 0);
	constexpr uint16_t arr = CLOCK_SPEED / DDS_FREQ - 1;
	static_assert((uint64_t(arr) + 1) * uint64_t(DDS_FREQ) == uint64_t(CLOCK_SPEED));

	TIM_DAC->CR1 = 0;
	TIM_DAC->DIER |= TIM_DIER_UIE;
	TIM_DAC->PSC = 0;
	TIM_DAC->ARR = arr;
	TIM_DAC->CR1 = TIM_CR1_CEN;
	NVIC_SetPriority(TIM6_IRQn, 3); // TODO
	NVIC_EnableIRQ(TIM6_IRQn);

	stm32_lib::gpio::set_mode_output_analog(pin_dac);

	DAC->CR = 0;
	DAC->MCR = (DAC->MCR & ~DAC_MCR_MODE1_Msk) | (0b000 << DAC_MCR_MODE1_Pos); // buffer
	DAC->CR = DAC_CR_EN1_Msk;
}


void stop_note(void* pv, uint32_t ul)
{
	logger.log_async("Player: stopping note\r\n");
	const note_id_t note_id = ul;
	voice_t* const vp = reinterpret_cast<voice_t*>(pv);
	vp->maybe_reset(note_id);
}


void play_note(notes::sym_t n, notes::duration_t d, notes::instrument_t instr, note_id_t note_id)
{
	constexpr auto fade = DDS_FREQ/512;
	const uint32_t dds_ticks = DDS_FREQ*d/64 - fade;

	const auto voice_idx = find_free_voice();
	g_voices[voice_idx].set(g_instr_info[instr].lookup_func, dds_notes_incs[n], dds_ticks, note_id);
}


void play_task_function(void*)
{
	uint32_t note_id = 1;
	for(;;) {
		queue_item_t item;
		if (xQueueReceive(play_task_data.arg_queue_handle, &item, configTICK_RATE_HZ/*1sec*/) == pdTRUE) {
			notes::sym_t n;
			notes::duration_t d;
			notes::instrument_t instr;
			queue_item_decode(item, n, d, instr);
			play_note(n, d, instr, note_id);
			++note_id;
		}
	}
}


void player::create_task(const char* task_name, UBaseType_t prio, QueueHandle_t queue_handle)
{
	dac_init();
	play_task_data.arg_queue_handle = queue_handle;
	xTaskCreateStatic(
		&play_task_function,
		task_name,
		play_task_data.stack.size(),
		nullptr,
		prio,
		play_task_data.stack.data(),
		&play_task_data.task_buffer
	);
}

