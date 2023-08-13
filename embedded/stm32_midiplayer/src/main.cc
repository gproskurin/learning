#include "player.h"

#include "lib_stm32.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 1
#define PRIO_SENDER 1
#define PRIO_LOGGER 2
#define PRIO_PLAYER 3

#if defined TARGET_STM32L432
	const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 3);

	// USART1
	#define USART_LOG USART1
	#define USART_LOG_AF 7
	const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOB, 6);

	// TIM15, PWM
	const stm32_lib::gpio::gpio_pin_t pin_pwm(GPIOA, 2);
	#define TIM_PWM_AF 14
	#define TIM_PWM TIM15
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


void tim_pwm_init(TIM_TypeDef* const tim)
{
	tim->CR1 = 0;
	tim->CNT = 0;
	tim->BDTR |= TIM_BDTR_MOE_Msk;
#if defined TARGET_STM32L432
	tim->CCMR1 = (tim->CCMR1 & ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk))
		| (0b00 << TIM_CCMR1_CC1S_Pos) // output channel
		| (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) // 0110 = PWM mode 1
		| TIM_CCMR1_OC1FE // output compare fast
		;
	tim->CCER |= TIM_CCER_CC1E;
#endif
	//tim->CR1 = TIM_CR1_CEN;
}


extern "C" __attribute__ ((interrupt)) void IntHandler_Timer()
{
#if 0
	const uint32_t sr = LED_TIM->SR;
	if (sr & TIM_SR_UIF) {
		// new cycle starts, led on
		LED_TIM->SR = ~TIM_SR_UIF;
		stm32_lib::gpio::set_state(LED_GPIO, LED_PIN, 1);
	} else if (sr & TIM_SR_CC1IF) {
		// compare event triggered, part of cycle finished, led off
		LED_TIM->SR = ~TIM_SR_CC1IF;
		stm32_lib::gpio::set_state(LED_GPIO, LED_PIN, 0);
	}
#endif
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
	// enable GPIOs
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk | RCC_AHB2ENR_GPIOBEN_Msk;
	toggle_bits_10(&RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOARST_Msk | RCC_AHB2RSTR_GPIOBRST_Msk);

	// USART1 & TIM15
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN_Msk | RCC_APB2ENR_USART1EN_Msk;
	toggle_bits_10(
		&RCC->APB2RSTR,
		RCC_APB2RSTR_TIM15RST_Msk | RCC_APB2RSTR_USART1RST_Msk
	);
#endif
}


StaticTask_t xTaskBufferIdle;
using idle_task_stack_t = task_stack_t<128>;
idle_task_stack_t idle_task_stack;
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
			configTICK_RATE_HZ,
			configTICK_RATE_HZ*2
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
		stm32_lib::gpio::set_state(args->pin, true);
		vTaskDelay(args->ticks_on);
		if (do_log) {
			logger.log_async("LED -> off\r\n");
		}
		stm32_lib::gpio::set_state(args->pin, false);
		vTaskDelay(args->ticks_off);
	}
}


struct note_t {
	notes::sym_t note;
	notes::duration_t duration;
};
using namespace notes;
const std::array<note_t, 66> music{
	// 1
	note_t{B5f, l4},
	note_t{A5, l4},
	note_t{G5, l4},

	// 2
	note_t{A5, l4},
	note_t{D5, l4},
	note_t{D5, l4},

	// 3
	note_t{G5, l4},
	note_t{G4, l8},
	note_t{A4, l8},
	note_t{B4f, l8},
	note_t{C5, l8},

	// 4
	note_t{D5, l16},
	note_t{E5f, l16},
	note_t{D5, l16},
	note_t{E5f, l16},
	note_t{D5, notes::duration_t(l2d-(4*l16))},

	// 5
	note_t{E5f, l4},
	note_t{F5, l8},
	note_t{E5f, l8},
	note_t{D5, l8},
	note_t{C5, l8},

	// 6
	note_t{D5,l4},
	note_t{E5f,l8},
	note_t{D5,l8},
	note_t{C5,l8},
	note_t{B4f,l8},

	// 7
	note_t{C5,l4},
	note_t{D5,l8},
	note_t{C5,l8},
	note_t{B4f,l8},
	note_t{C5,l8},

	// 8
	note_t{A4,l16},
	note_t{B4f,l16},
	note_t{A4,l16},
	note_t{B4f,l16},
	note_t{A4,static_cast<notes::duration_t>(l2d - 4*l16)},

	// 9
	note_t{B5f, l4},
	note_t{A5, l4},
	note_t{G5, l4},

	// 10
	note_t{A5, l4},
	note_t{D5, l4},
	note_t{D5, l4},

	// 11
	note_t{G5, l4},
	note_t{G4, l8},
	note_t{A4, l8},
	note_t{B4f, l8},
	note_t{C5, l8},

	// 12
	note_t{D5, l16},
	note_t{E5f, l16},
	note_t{D5, l16},
	note_t{E5f, l16},
	note_t{D5, static_cast<notes::duration_t>(l2d-4*l16)},

	// 13
	note_t{F5, l4},
	note_t{G5, l8},
	note_t{F5, l8},
	note_t{E5f, l8},
	note_t{D5, l8},

	// 14
	note_t{E5f, l4},
	note_t{F5, l8},
	note_t{E5f, l8},
	note_t{D5, l8},
	note_t{C5, l8},

	// 15
	note_t{D5, l4},
	note_t{G5, l4},
	note_t{C5, l4},

	// 16
	note_t{B4f, l2d}
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

	QueueHandle_t arg_player_queue_handle = nullptr;
} sender_task_data;

void sender_task_function(void*)
{
	for (;;) {
		for (const auto& n : Cmaj) {
			player::enqueue_note(sender_task_data.arg_player_queue_handle, n, notes::duration_t::l4);
			vTaskDelay(configTICK_RATE_HZ/4);
		}
		vTaskDelay(configTICK_RATE_HZ);

		for (const auto& n : Am) {
			player::enqueue_note(sender_task_data.arg_player_queue_handle, n, notes::duration_t::l4);
			vTaskDelay(configTICK_RATE_HZ/4);
		}
		vTaskDelay(configTICK_RATE_HZ*3);

		for (const auto& n : music) {
			player::enqueue_note(sender_task_data.arg_player_queue_handle, n.note, n.duration);
			vTaskDelay(configTICK_RATE_HZ/50*n.duration);
		}
		vTaskDelay(configTICK_RATE_HZ*3);
	}
}

void create_task_sender(QueueHandle_t player_queue_handle)
{
	sender_task_data.arg_player_queue_handle = player_queue_handle;
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

	usart_init(USART_LOG);
	logger.set_usart(USART_LOG);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");

	stm32_lib::gpio::set_mode_af_lowspeed_pushpull(pin_pwm, TIM_PWM_AF);
	tim_pwm_init(TIM_PWM);

	logger.log_sync("Creating blink tasks...\r\n");
	for (auto& bt : blink_tasks.tasks) {
		stm32_lib::gpio::set_mode_output_lowspeed_pushpull(bt.pin);
		create_blink_task(bt);
	}
	logger.log_sync("Created blink tasks\r\n");

	logger.log_sync("Creating player queue...\r\n");
	QueueHandle_t player_queue_handle = player::create_queue();
	logger.log_sync("Created player queue\r\n");

	logger.log_sync("Creating player task...\r\n");
	player::create_task("player", PRIO_PLAYER, player_queue_handle);
	logger.log_sync("Created player task\r\n");

	logger.log_sync("Creating sender task...\r\n");
	create_task_sender(player_queue_handle);
	logger.log_sync("Created sender task\r\n");

	//logger.log_sync("Initializing interrupts\r\n");
	//nvic_init_tim();

	//logger.log_sync("Initializing timer\r\n");
	//basic_timer_init(LED_TIM, 2000-1, 1000-1);

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {
		//blink(2);
		//delay(100000);
		//__WFI();
	}
}

