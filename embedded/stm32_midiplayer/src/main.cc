#include "player.h"

#include "lib_stm32.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 2
#define PRIO_SENDER 1
#define PRIO_LOGGER 3
#define PRIO_PLAYER 4

#if defined TARGET_STM32L432
	const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 3);

	// USART1
	#define USART_LOG USART1
	#define USART_LOG_AF 7
	const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOB, 6);
#endif


#define CLOCK_SPEED configCPU_CLOCK_HZ
#define USART_CON_BAUDRATE 115200


template <size_t StackSize>
using task_stack_t = std::array<StackType_t, StackSize>;

usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_TE;

#ifndef TARGET_STM32F103
	stm32_lib::gpio::set_mode_af_lowspeed_pu(usart_log_pin_tx, USART_LOG_AF);
	usart->BRR = CLOCK_SPEED / USART_CON_BAUDRATE;
#endif

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
}


void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void bus_init()
{
#if defined TARGET_STM32L432
	// flash
	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | FLASH_ACR_LATENCY_2WS;

	// clock
	//while (!(RCC->CR & RCC_CR_MSIRDY)) {} // wait for MSI to be ready
	RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE_Msk) | RCC_CR_MSIRANGE_11;
	RCC->CR |= RCC_CR_MSIRGSEL;

	// GPIOs
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk | RCC_AHB2ENR_GPIOBEN_Msk;
	toggle_bits_10(&RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOARST_Msk | RCC_AHB2RSTR_GPIOBRST_Msk);

	// DAC1 & TIM6
	RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN_Msk | RCC_APB1ENR1_TIM6EN_Msk;
	toggle_bits_10(
		&RCC->APB1RSTR1,
		RCC_APB1RSTR1_DAC1RST_Msk | RCC_APB1RSTR1_TIM6RST_Msk
	);

	// DMA1
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN_Msk;
	toggle_bits_10(
		&RCC->AHB1RSTR,
		RCC_AHB1RSTR_DMA1RST_Msk
	);

	// USART1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
	toggle_bits_10(
		&RCC->APB2RSTR,
		RCC_APB2RSTR_USART1RST_Msk
	);
#endif
}


StaticTask_t xTaskBufferIdle;
task_stack_t<128> idle_task_stack;
extern "C"
void vApplicationGetIdleTaskMemory(StaticTask_t **tcbIdle, StackType_t **stackIdle, uint32_t *stackSizeIdle)
{
	*tcbIdle = &xTaskBufferIdle;
	*stackIdle = idle_task_stack.data();
	*stackSizeIdle = idle_task_stack.size();
}

extern "C"
void vApplicationIdleHook(void)
{
	__WFI();
}


// LED blinking tasks
struct blink_task_data_t {
	const char* const task_name;
	const stm32_lib::gpio::gpio_pin_t pin;
	const TickType_t ticks_on;
	const TickType_t ticks_off;
	blink_task_data_t(const char* tname, const stm32_lib::gpio::gpio_pin_t& p, TickType_t ton, TickType_t toff)
		: task_name(tname), pin(p), ticks_on(ton), ticks_off(toff)
	{}

	task_stack_t<128> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};

struct blink_tasks_t {
	std::array<blink_task_data_t, 1> tasks = {
		blink_task_data_t(
			"blink_green",
			pin_led_green,
			configTICK_RATE_HZ/8,
			configTICK_RATE_HZ*3/8
		)
	};
} blink_tasks;


void blink_task_function(void* arg)
{
	const blink_task_data_t* const args = reinterpret_cast<blink_task_data_t*>(arg);
	for(int i=0;;++i) {
		const bool do_log = (i % 64 == 0);
		if (do_log) {
			logger.log_async("LED -> on\r\n");
		}
		//stm32_lib::gpio::set_state(args->pin, true);
		vTaskDelay(args->ticks_on);
		if (do_log) {
			logger.log_async("LED -> off\r\n");
		}
		//stm32_lib::gpio::set_state(args->pin, false);
		vTaskDelay(args->ticks_off);
	}
}


struct note_t {
	notes::sym_t note;
	notes::duration_t duration;

	note_t() : duration(static_cast<notes::duration_t>(0)) {}
	note_t(notes::sym_t n, notes::duration_t d) : note(n), duration(d) {}
};

using notes_t = std::array<note_t, 5>;

struct beat_t {
	notes::duration_t duration;
	notes_t notes;

	beat_t(notes::duration_t d, const notes_t& nn) : duration(d), notes(nn) {}
};

using namespace notes;
const std::array<beat_t, 67> music{
	// 1
	beat_t{l4, notes_t{note_t{B5f, l4}, note_t{G3, l2d}}},
	beat_t{l4, notes_t{note_t{A5, l4}}},
	beat_t{l4, notes_t{note_t{G5, l4}}},

	// 2
	beat_t{l4, notes_t{note_t{A5, l4}, note_t{F3, l2d}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},

	// 3
	beat_t{l4, notes_t{note_t{G5, l4}, {E3f, l2d}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{B4f, l8}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 4
	beat_t{l4, notes_t{note_t{D3, l4}, note_t{D5, l2d}}},
	beat_t{l8, notes_t{note_t{D4, l8}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},
	beat_t{l8, notes_t{note_t{B3f, l8}}},
	beat_t{l8, notes_t{note_t{A3, l8}}},

	// 5
	beat_t{l4, notes_t{note_t{E5f, l4}, note_t{B3f, l2}, note_t{G3, l2}}},
	beat_t{l8, notes_t{note_t{F5, l8}}},
	beat_t{l8, notes_t{note_t{E5f, l8}}},
	beat_t{l8, notes_t{note_t{D5, l8}, note_t{A3, l4}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 6
	beat_t{l4, notes_t{note_t{D5,l4}, note_t{B3f, l2}}},
	beat_t{l8, notes_t{note_t{E5f,l8}}},
	beat_t{l8, notes_t{note_t{D5,l8}}},
	beat_t{l8, notes_t{note_t{C5,l8}, note_t{G3,l8}}},
	beat_t{l8, notes_t{note_t{B4f,l8}}},

	// 7
	beat_t{l4, notes_t{note_t{C5,l4}, note_t{A3,l4}}},
	beat_t{l8, notes_t{note_t{D5,l8}, note_t{F3s,l4}}},
	beat_t{l8, notes_t{note_t{C5,l8}}},
	beat_t{l8, notes_t{note_t{B4f,l8}, note_t{G3,l4}}},
	beat_t{l8, notes_t{note_t{C5,l8}}},

	// 8
	beat_t{l4, notes_t{note_t{A4,l2d}, note_t{D3, l4}}},
	beat_t{l8, notes_t{note_t{D4,l8}}},
	beat_t{l8, notes_t{note_t{C4,l8}}},
	beat_t{l8, notes_t{note_t{B3f,l8}}},
	beat_t{l8, notes_t{note_t{A3,l8}}},

	// 9
	beat_t{l4, notes_t{note_t{B5f, l4}, note_t{G3,l2d}}},
	beat_t{l4, notes_t{note_t{A5, l4}}},
	beat_t{l4, notes_t{note_t{G5, l4}}},

	// 10
	beat_t{l4, notes_t{note_t{A5, l4}, note_t{F3,l2d}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},

	// 11
	beat_t{l4, notes_t{note_t{G5, l4}, note_t{E3f,l2d}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{B4f, l8}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 12
	beat_t{l4, notes_t{note_t{D5, l2d}, note_t{D3,l4}}},
	beat_t{l8, notes_t{note_t{D4, l8}}},
	beat_t{l8, notes_t{note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{B4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},

	// 13
	beat_t{l4, notes_t{note_t{F5, l4}, note_t{D4,l2}, note_t{B4,l2}}},
	beat_t{l8, notes_t{note_t{G5, l8}}},
	beat_t{l8, notes_t{note_t{F5, l8}}},
	beat_t{l8, notes_t{note_t{E5f, l8}, note_t{G3,l4}}},
	beat_t{l8, notes_t{note_t{D5, l8}}},

	// 14
	beat_t{l4, notes_t{note_t{E5f, l4}, note_t{C4, l4}}},
	beat_t{l8, notes_t{note_t{F5, l8}, note_t{A3, l4}}},
	beat_t{l8, notes_t{note_t{E5f, l8}}},
	beat_t{l8, notes_t{note_t{D5, l8}, note_t{F3,l4}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 15
	beat_t{l4, notes_t{note_t{D5, l4}, note_t{B3f,l4}}},
	beat_t{l4, notes_t{note_t{G5, l4}, note_t{E3f,l4}}},
	beat_t{l4, notes_t{note_t{C5, l4}, note_t{F3,l4}, note_t{A3,l4}}},

	// 16
	beat_t{l4, notes_t{note_t{B4f, l2d}, note_t{F4, l2d}, note_t{D4, l2d}, note_t{B3f,l4}}},
	beat_t{l2, notes_t{note_t{B2f, l2}}}
};

const std::array<notes::sym_t, 8> Cmaj{C4, D4, E4, F4, G4, A4, B4, C5};
const std::array<notes::sym_t, 8> Am{A4, B4, C5, D5, E5, F5, G5, A5};

void str_cpy_3(char* dst, const char* s1, const char* s2, const char* s3)
{
	auto p = stpcpy(dst, s1);
	p = stpcpy(p, s2);
	stpcpy(p, s3);
}


void create_blink_task(blink_task_data_t& args)
{
	char log_buf[64];

	str_cpy_3(log_buf, "Creating blink task \"", args.task_name, "\" ...\r\n");
	logger.log_sync(log_buf);

	args.task_handle = xTaskCreateStatic(
		&blink_task_function,
		args.task_name,
		args.stack.size(),
		reinterpret_cast<void*>(&args),
		PRIO_BLINK,
		args.stack.data(),
		&args.task_buffer
	);

	str_cpy_3(log_buf, "Created blink task \"", args.task_name, "\"\r\n");
	logger.log_sync(log_buf);
}


struct sender_task_data_t {
	std::array<StackType_t, 128> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
} sender_task_data;

void sender_task_function(void*)
{
	vTaskDelay(configTICK_RATE_HZ*3);
#if 0
	for (;;) {
		player::enqueue_note(notes::sym_t::A4, notes::duration_t::l2, notes::instrument_t::sin12);
		vTaskDelay(configTICK_RATE_HZ);
		player::enqueue_note(notes::sym_t::B4, notes::duration_t::l2, notes::instrument_t::sin12);
		vTaskDelay(configTICK_RATE_HZ/2);
	}
#endif

	for (;;) {
#if 0
		for (const auto& n : Cmaj) {
			player::enqueue_note(n, notes::duration_t::l4, notes::instrument_t::sin12);
			vTaskDelay(configTICK_RATE_HZ/4);
		}
		vTaskDelay(configTICK_RATE_HZ);
#endif

		for (const auto& n : Am) {
			player::enqueue_note(n, notes::duration_t::l4, notes::instrument_t::sin12);
			vTaskDelay(configTICK_RATE_HZ/4);
		}
		vTaskDelay(configTICK_RATE_HZ*3);

		for (const auto& beat : music) {
			for (const auto& n : beat.notes) {
				if (n.duration) {
					player::enqueue_note(
						n.note,
						n.duration,
						notes::instrument_t::sin12
					);
				}
			}
			vTaskDelay(configTICK_RATE_HZ/32*beat.duration);
		}
		vTaskDelay(configTICK_RATE_HZ*2);
	}
}

void create_task_sender()
{
	sender_task_data.task_handle = xTaskCreateStatic(
		&sender_task_function,
		"sender",
		sender_task_data.stack.size(),
		nullptr,
		PRIO_SENDER,
		sender_task_data.stack.data(),
		&sender_task_data.task_buffer
	);
}


__attribute__ ((noreturn)) void main()
{
	bus_init();
	stm32_lib::gpio::set_mode_output_lowspeed_pushpull(pin_led_green);

	usart_init(USART_LOG);
	logger.set_usart(USART_LOG);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");

	logger.log_sync("Creating blink tasks...\r\n");
	for (auto& bt : blink_tasks.tasks) {
		stm32_lib::gpio::set_mode_output_lowspeed_pushpull(bt.pin);
		create_blink_task(bt);
	}
	logger.log_sync("Created blink tasks\r\n");

	logger.log_sync("Creating player task...\r\n");
	player::create_task("player", PRIO_PLAYER);
	logger.log_sync("Created player task\r\n");

	logger.log_sync("Creating sender task...\r\n");
	create_task_sender();
	logger.log_sync("Created sender task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

