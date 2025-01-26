#include "player.h"

#include "cmsis_device.h"
#include "lib_stm32.h"
#include "bsp.h"
#include "freertos_utils.h"
#include "logger_fwd.h"

#include <array>
#include <limits>


#define DDS_FREQ 32000 // keep in sync with python generator

#if defined TARGET_STM32L072
	#define TIM_DAC TIM6
	#define DMA_CHANNEL_DAC DMA1_Channel2
	constexpr stm32_lib::gpio::gpio_pin_t pin_dac(GPIOA_BASE, 4);
#elif defined TARGET_STM32WL55_CPU1
	#define TIM_DAC TIM2
	#define DMA_CHANNEL_DAC DMA1_Channel2
	constexpr stm32_lib::gpio::gpio_pin_t pin_dac(GPIOA_BASE, 10);
#endif

constexpr uint16_t tim_dac_cr1 = 0;
constexpr uint16_t tim_dac_cr1_en = TIM_CR1_CEN;

constexpr uint32_t dac_cr_en =
#ifdef TARGET_STM32L072
	(/*TIM6*/0b000 << DAC_CR_TSEL1_Pos)
#endif
#ifdef TARGET_STM32WL55_CPU1
	(/*TIM2*/0b0010 << DAC_CR_TSEL1_Pos)
#endif
	| DAC_CR_TEN1
	| DAC_CR_EN1
	;

constexpr uint32_t dac_cr_dmaen = dac_cr_en | DAC_CR_DMAEN1;

constexpr uint32_t dma_ccr =
	(0b10 << DMA_CCR_PL_Pos) // priority
	| (0b01 << DMA_CCR_MSIZE_Pos) // 16bit
	| (0b01 << DMA_CCR_PSIZE_Pos)
	| DMA_CCR_MINC
	| DMA_CCR_CIRC
	| DMA_CCR_DIR
	| DMA_CCR_HTIE
	| DMA_CCR_TCIE
	| DMA_CCR_TEIE
	;
constexpr uint32_t dma_ccr_en = dma_ccr | DMA_CCR_EN;

#ifdef TARGET_STM32L072
freertos_utils::pin_toggle_task_t pin_green("pin_toggle_green", bsp::pin_led_green, 1);
#endif
freertos_utils::pin_toggle_task_t pin_blue("pin_toggle_blue", bsp::pin_led_blue, 1);
freertos_utils::pin_toggle_task_t pin_red("pin_toggle_red", bsp::pin_led_red, 1);

typedef uint16_t dds_value_t;
#include "lookup_tables.cc.h"

typedef uint16_t note_id_t;

typedef dds_value_t (*lookup_func_t)(uint32_t);

struct instr_info_t {
	const lookup_func_t lookup_func;
	explicit constexpr instr_info_t(lookup_func_t f) : lookup_func(f) {}
};


enum queue_nf_t : uint8_t {
	ht = 1U << 0,
	tc = 1U << 1,
	te = 1U << 2
};


// use just quarter of sine period to deduce value for any point inside one period
template <typename LookupTable>
dds_value_t lookup_sin_quarter(const LookupTable& t, size_t idx)
{
	constexpr auto sz = std::tuple_size<LookupTable>::value;
	if (idx < sz)
		return t[idx];
	if (idx == sz)
		return 65535; // FIXME
	if (idx < sz*2)
		return t[sz*2 - idx];
	if (idx == sz*2)
		return t[0];
	if (idx < sz*3)
		return t[0] - (t[idx - sz*2] - t[0]) - 1;
	if (idx == sz*3)
		return 0;
	return t[0] - (t[sz*4 - idx] - t[0]) - 1;
}


dds_value_t lookup_sq(uint32_t x)
{
	return (x >> 31) ? std::numeric_limits<dds_value_t>::max() : 0;
}

dds_value_t lookup_sin3(uint32_t x)
{
	return lookup_sin_quarter(lookup_table_sin3, x >> 29);
}

dds_value_t lookup_sin4(uint32_t x)
{
	return lookup_sin_quarter(lookup_table_sin4, x >> 28);
}

dds_value_t lookup_sin5(uint32_t x)
{
	return lookup_sin_quarter(lookup_table_sin5, x >> 27);
}

dds_value_t lookup_sin10(uint32_t x)
{
	return lookup_sin_quarter(lookup_table_sin10, x >> 22);
}

dds_value_t lookup_sin12(uint32_t x)
{
	//return lookup_sin_quarter(lookup_table_sin12, x >> 20);
	return lookup_table_sin12_full[x >> 20];
}

constexpr std::array<instr_info_t, 6> g_instr_info{
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
			// found "older" note, switch to it
			idx = i;
		}
	}
	g_voices[idx].reset();
	return idx;
}


typedef uint32_t queue_item_t;

queue_item_t queue_item_encode(notes::sym_t n, notes::duration_t d, notes::instrument_t instr, queue_nf_t nf)
{
	return
		(static_cast<uint32_t>(nf) << 24)
		| (static_cast<uint32_t>(instr) << 16)
		| (static_cast<uint32_t>(d) << 8)
		| static_cast<uint32_t>(n);
}

void queue_item_decode(queue_item_t item, notes::sym_t& n, notes::duration_t& d, notes::instrument_t& instr, queue_nf_t& nf)
{
	n = static_cast<notes::sym_t>(item & 0xff);
	d = static_cast<notes::duration_t>((item >> 8) & 0xff);
	instr = static_cast<notes::instrument_t>((item >> 16) & 0xff);
	nf = static_cast<queue_nf_t>((item >> 24) & 0xff);
}


struct player_task_data_t {
	std::array<StackType_t, 256> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;

	std::array<queue_item_t, 16> queue_buffer;
	StaticQueue_t queue_q;
	QueueHandle_t queue_handle = nullptr;

	uint32_t next_note_id = 1;
	std::array<uint16_t, 128> dma_buffer{0};
} player_task_data;


void player::enqueue_note(notes::sym_t n, notes::duration_t d, notes::instrument_t instr)
{
	queue_item_t const item = queue_item_encode(n, d, instr, static_cast<queue_nf_t>(0));
	if (xQueueSend(player_task_data.queue_handle, &item, 0) != pdTRUE) {
		logger.log_async("PLAYER: enqueue_note fail\r\n");
		pin_red.pulse_once(configTICK_RATE_HZ/2);
	}
}


#define CLOCK_SPEED configCPU_CLOCK_HZ


constexpr std::pair<uint16_t, uint16_t> calc_psc_arr()
{
	uint16_t psc = 0;
	uint32_t arr = CLOCK_SPEED / DDS_FREQ;
	while (arr >= 65535) {
		++psc;
		arr /= 2;
	}
	return std::make_pair(psc, arr-1);
}


void dac_init()
{
	// ensure there is no truncation in calculation
	static_assert((CLOCK_SPEED % DDS_FREQ) == 0);
	//static_assert((uint64_t(arr) + 1) * uint64_t(DDS_FREQ) == uint64_t(CLOCK_SPEED));
	//static_assert(uint64_t(psc_arr.second-1) * uint64_t(DDS_FREQ) * uint64_t(psc_arr.first+1) == uint64_t(CLOCK_SPEED));

	DAC->CR = 0;
	TIM_DAC->CR1 = 0;
	DMA_CHANNEL_DAC->CCR = 0;

	TIM_DAC->DIER = TIM_DIER_UDE;
	TIM_DAC->CR2 = (0b010 << TIM_CR2_MMS_Pos);
	TIM_DAC->PSC = calc_psc_arr().first;
	TIM_DAC->ARR = calc_psc_arr().second;

#if defined TARGET_STM32L072
	// dma ch2 / dac
	NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3);
	NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
#elif defined TARGET_STM32WL55_CPU1
	// dma/dac
	NVIC_SetPriority(DMA1_Channel2_IRQn, 12);
	NVIC_EnableIRQ(DMA1_Channel2_IRQn);
#endif

	stm32_lib::gpio::set_mode_output_analog(pin_dac);

	//DAC->MCR = (DAC->MCR & ~DAC_MCR_MODE1_Msk) | (0b000 << DAC_MCR_MODE1_Pos); // buffer

#if defined TARGET_STM32L072
	DMA_CHANNEL_DAC->CPAR = reinterpret_cast<uint32_t>(&DAC1->DHR12L1);
#elif defined TARGET_STM32WL55_CPU1
	DMA_CHANNEL_DAC->CPAR = reinterpret_cast<uint32_t>(&DAC->DHR12L1);
#endif
	DMA_CHANNEL_DAC->CMAR = reinterpret_cast<uint32_t>(player_task_data.dma_buffer.data());

#ifdef TARGET_STM32WL55_CPU1
	DMAMUX1_Channel1->CCR = stm32_lib::dma::consts::dmamux_reqid_tx<DAC_BASE>() << DMAMUX_CxCR_DMAREQ_ID_Pos;
#endif
}


void timer_start()
{
	// DMA
	DMA_CHANNEL_DAC->CCR = dma_ccr; // need this?
	DMA_CHANNEL_DAC->CNDTR = player_task_data.dma_buffer.size();
	DMA_CHANNEL_DAC->CCR = dma_ccr_en;

	// DAC
	DAC->CR = dac_cr_en;
	DAC->CR = dac_cr_dmaen;

	// timer
	TIM_DAC->CR1 = tim_dac_cr1;
	TIM_DAC->CNT = 0;
	TIM_DAC->CR1 = tim_dac_cr1_en;

#ifdef TARGET_STM32L072
	pin_green.on();
	logger.log_async(">>> timer_started\r\n");
#endif
}

void timer_stop()
{
	// stop timer
	TIM_DAC->CR1 = tim_dac_cr1;
	while (TIM_DAC->CR1 & TIM_CR1_CEN) {}

	// disable DAC DMA
	DAC->CR = dac_cr_en;
	while (DAC->CR & DAC_CR_DMAEN1) {}

	// disable DMA channel
	{
		// disable interrupts
		constexpr uint32_t ccr1 = dma_ccr_en & ~(DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_TEIE);
		DMA_CHANNEL_DAC->CCR = ccr1;

		// disable channel
		constexpr uint32_t ccr2 = ccr1 & ~DMA_CCR_EN;
		DMA_CHANNEL_DAC->CCR = ccr2;
		while (DMA_CHANNEL_DAC->CCR & DMA_CCR_EN) {}

		// clear interrupt flags
#ifdef TARGET_STM32L072
		DMA1->IFCR = DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2 | DMA_IFCR_CGIF2;
#endif
#ifdef TARGET_STM32WL55_CPU1
		DMA1->IFCR = DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2 | DMA_IFCR_CGIF2;
#endif
	}

#ifdef TARGET_STM32L072
	pin_green.off();
	logger.log_async("<<< - timer_stopped\r\n");
#endif
}


void play_note(notes::sym_t n, notes::duration_t d, notes::instrument_t instr, note_id_t note_id)
{
	constexpr uint32_t fade = DDS_FREQ/1000;
	constexpr uint32_t dds_freq_64 = DDS_FREQ/64;
	static_assert((DDS_FREQ % 64) == 0); // avoid truncation
	const uint32_t dds_ticks = dds_freq_64 * d - fade;

	const auto voice_idx = find_free_voice();
	g_voices[voice_idx].set(g_instr_info[instr].lookup_func, dds_notes_incs[n], dds_ticks, note_id);
}


void player_task_function(void*); // FIXME
void player::create_task(const char* task_name, UBaseType_t prio)
{
	player_task_data.queue_handle = xQueueCreateStatic(
		player_task_data.queue_buffer.size(),
		sizeof(queue_item_t),
		reinterpret_cast<uint8_t*>(player_task_data.queue_buffer.data()),
		&player_task_data.queue_q
	);
	dac_init();
	player_task_data.task_handle = xTaskCreateStatic(
		&player_task_function,
		task_name,
		player_task_data.stack.size(),
		nullptr,
		prio,
		player_task_data.stack.data(),
		&player_task_data.task_buffer
	);
}


namespace dmafill_fsm {
	enum state_t {
		dma_empty, dma_doing_1, dma_doing_2
	};
	const char* state_str(state_t s)
	{
		switch (s) {
			case dma_empty: return "dma_empty\r\n";
			case dma_doing_1: return "dma_doing_1\r\n";
			case dma_doing_2: return "dma_doing_2\r\n";
		}
		return "dma_state_unknown\r\n";
	}

	bool fill_buffer_impl(size_t idx_begin, size_t idx_end)
	{
		for (size_t i=idx_begin; i<idx_end; ++i) {
			uint8_t voices_count = 0;
			uint32_t value = 0;
			for (auto& v : g_voices) {
				if (v.is_busy()) {
					value += uint32_t(v.tick());
					++voices_count;
				}
			}
			if (voices_count == 0) {
				// all voices are silent
				// fill remaining part with previous value (if any) or with middle value
				const dds_value_t fill = (i != idx_begin) ? player_task_data.dma_buffer[i-1] : 32768;
				/*
				for (size_t j=i; j<idx_end; ++j) {
					// TODO
					//player_task_data.dma_buffer[j] = fill;
				}
				*/
				return false;
			}
#ifdef TARGET_STM32L072
			// FIXME
			switch (voices_count) {
				case 1: break;
				case 2: value /= 2; break;
				case 3: value = value*3/8; break; // FIXME
				case 4: value /= 4; break;
			}
#else
			value /= voices_count;
#endif
			value /= 4; // lower volume
			player_task_data.dma_buffer[i] = static_cast<uint16_t>(value);
		}
		return true;
	}

	inline
	bool action_fill_bottom_half()
	{
		return fill_buffer_impl(0, player_task_data.dma_buffer.size()/2);
	}

	inline
	bool action_fill_top_half()
	{
		return fill_buffer_impl(player_task_data.dma_buffer.size()/2, player_task_data.dma_buffer.size());
	}

	inline
	bool action_fill()
	{
		return fill_buffer_impl(0, player_task_data.dma_buffer.size());
	}
}

void player_task_function(void*)
{
	using namespace dmafill_fsm;

#ifdef TARGET_STM32L072
	pin_green.init_pin();
#endif
	pin_blue.init_pin();
	pin_red.init_pin();

	state_t state = dma_empty;
	for(;;) {
		queue_item_t item;
		if (xQueueReceive(player_task_data.queue_handle, &item, portMAX_DELAY) != pdTRUE) {
			// spurious wakeup?
			logger.log_async("PLAYER: spurious\r\n");
			continue;
		}

		//logger.log_async("*** GOT msg\r\n");

		notes::sym_t item_n;
		notes::duration_t item_d;
		notes::instrument_t item_i;
		queue_nf_t item_nf;
		queue_item_decode(item, item_n, item_d, item_i, item_nf);

		if (item_n != notes::sym_t::sym_none) {
			// got note from queue
			// update voices, note will start sounding on next dma transfer
			play_note(item_n, item_d, item_i, player_task_data.next_note_id);
			++player_task_data.next_note_id; // TODO handle wrap
			pin_blue.pulse_once(configTICK_RATE_HZ/50);
			//logger.log_async("*** YES_NOTE\r\n");
		} else {
			//logger.log_async("*** NO_NOTE\r\n");
		}
		if (item_nf & queue_nf_t::te) {
			// TODO reset dma, disable/enable, ...
			timer_stop();
			state = dma_empty;
			logger.log_async("PLAYER error: dma te\r\n");
			continue;
		}

		const bool ht = item_nf & queue_nf_t::ht;
		const bool tc = item_nf & queue_nf_t::tc;
		//logger.log_async(ht ? "*** YES_HT\r\n" : "*** NO_HT\r\n");
		//logger.log_async(tc ? "*** YES_TC\r\n" : "*** NO_TC\r\n");
		if (!ht && !tc) {
			// no dma flags, probably just note added
			if (state == dma_empty) {
				if (action_fill()) {
					timer_start();
					state = dma_doing_1;
				} else {
				}
			} else {
			}
		} else if (state == dma_empty) {
			// ignore
		} else if (ht && !tc && state==dma_doing_1) {
			// transferred first half, refill it
			if (action_fill_bottom_half()) {
				state = dma_doing_2;
			} else {
				timer_stop();
				state = dma_empty;
			}
		} else if (!ht && tc && state==dma_doing_2) {
			// transferred second half, refill it
			if (action_fill_top_half()) {
				state = dma_doing_1;
			} else {
				timer_stop();
				state = dma_empty;
			}
		} else {
			logger.log_async("PLAYER: unsuported flags combination\r\n");
			logger.log_async(ht ? "PLAYER: ht\r\n" : "PLAYER: !ht\r\n");
			logger.log_async(tc ? "PLAYER: tc\r\n" : "PLAYER: !tc\r\n");
			logger.log_async(dmafill_fsm::state_str(state));
			pin_red.pulse_once(configTICK_RATE_HZ/10);
		}
	}
}


template <
	uint32_t IsrCheckHt, uint32_t IfcrClearHt,
	uint32_t IsrCheckTc, uint32_t IfcrClearTc,
	uint32_t IsrCheckTe, uint32_t IfcrClearTe
>
inline
void int_handler_dma_dac_impl(uint32_t isr)
{
	uint8_t events = 0;
	if (isr & IsrCheckHt) {
		DMA1->IFCR = IfcrClearHt;
		events |= queue_nf_t::ht;
	}
	if (isr & IsrCheckTc) {
		DMA1->IFCR = IfcrClearTc;
		events |= queue_nf_t::tc;
	}
	if (isr & IsrCheckTe) {
		DMA1->IFCR = IfcrClearTe;
		events |= queue_nf_t::te;
		logger.log_async_from_isr("DMA error\r\n");
	}
	if (events) {
		BaseType_t yield = pdFALSE;
		queue_item_t const item = queue_item_encode(
			notes::sym_t::sym_none,
			notes::duration_t::dur_zero,
			static_cast<notes::instrument_t>(0),
			static_cast<queue_nf_t>(events)
		);
		xQueueSendFromISR(player_task_data.queue_handle, reinterpret_cast<const void*>(&item), &yield);
		portYIELD_FROM_ISR(yield);
	}
}


#ifdef TARGET_STM32L072
extern "C" __attribute__ ((interrupt)) void IntHandler_DMA1_Channel2_3()
{
	int_handler_dma_dac_impl<
		DMA_ISR_HTIF2, DMA_IFCR_CHTIF2,
		DMA_ISR_TCIF2, DMA_IFCR_CTCIF2,
		DMA_ISR_TEIF2, DMA_IFCR_CTEIF2
	>(DMA1->ISR);
}
#endif

#ifdef TARGET_STM32WL55_CPU1
extern "C" __attribute__ ((interrupt)) void IntHandler_Dma1Ch2()
{
	auto const isr = DMA1->ISR;
	int_handler_dma_dac_impl<
		DMA_ISR_HTIF2, DMA_IFCR_CHTIF2,
		DMA_ISR_TCIF2, DMA_IFCR_CTCIF2,
		DMA_ISR_TEIF2, DMA_IFCR_CTEIF2
	>(DMA1->ISR);
}
#endif

