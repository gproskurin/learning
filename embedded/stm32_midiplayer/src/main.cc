#include "lib_stm32.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 1
#define PRIO_LOGGER 2
#define PRIO_PLAY 3

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


void basic_timer_init(TIM_TypeDef* const tim, uint16_t prescaler, uint16_t arr)
{
	tim->PSC = prescaler;
	tim->ARR = arr;
	tim->CCR1 = arr / 5 * 4;
	tim->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE; // enable hardware interrupt
	tim->CR1 = (tim->CR1 & ~(TIM_CR1_UDIS | TIM_CR1_OPM)) | TIM_CR1_CEN;
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


// Play task
struct play_task_data_t {
	play_task_data_t() {}

	task_stack_t<128> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
} play_task_data;


namespace notes {
	enum note_sym_t {
		C0 = 0,
		C0s, D0f = C0s,
		D0,
		D0s, E0f = D0s,
		E0,
		F0,
		F0s, G0f = F0s,
		G0,
		G0s, A0f = G0s,
		A0,
		A0s,B0f = A0s,
		B0,

		C1,
		C1s, D1f = C1s,
		D1,
		D1s, E1f = D1s,
		E1,
		F1,
		F1s, G1f = F1s,
		G1,
		G1s, A1f = G1s,
		A1,
		A1s, B1f = A1s,
		B1,

		C2,
		C2s, D2f = C2s,
		D2,
		D2s, E2f = D2s,
		E2,
		F2,
		F2s, G2f = F2s,
		G2,
		G2s, A2f = G2s,
		A2,
		A2s, B2f = A2s,
		B2,

		C3,
		C3s, D3f = C3s,
		D3,
		D3s, E3f = D3s,
		E3,
		F3,
		F3s, G3f = F3s,
		G3,
		G3s, A3f = G3s,
		A3,
		A3s, B3f = A3s,
		B3,

		C4,
		C4s, D4f = C4s,
		D4,
		D4s, E4f = D4s,
		E4,
		F4,
		F4s, G4f = F4s,
		G4,
		G4s, A4f = G4s,
		A4,
		A4s, B4f = A4s,
		B4,

		C5,
		C5s, D5f = C5s,
		D5,
		D5s, E5f = D5s,
		E5,
		F5,
		F5s, G5f = F5s,
		G5,
		G5s, A5f = G5s,
		A5,
		A5s, B5f = A5s,
		B5,

		C6,
		C6s, D6f = C6s,
		D6,
		D6s, E6f = D6s,
		E6,
		F6,
		F6s, G6f = F6s,
		G6,
		G6s, A6f = G6s,
		A6,
		A6s, B6f = A6s,
		B6,

		C7,
		C7s, D7f = C7s,
		D7,
		D7s, E7f = D7s,
		E7,
		F7,
		F7s, G7f = F7s,
		G7,
		G7s, A7f = G7s,
		A7,
		A7s, B7f = A7s,
		B7,

		C8,
		C8s, D8f = C8s,
		D8,
		D8s, E8f = D8s,
		E8,
		F8,
		F8s, G8f = F8s,
		G8,
		G8s, A8f = G8s,
		A8,
		A8s, B8f = A8s,
		B8
	};
	const std::array<uint32_t, 12*9> freq100 = {
		1635, 1732, 1835, 1945, 2060, 2183, 2312, 2450, 2596, 2750, 2914, 3087,
		3270, 3465, 3671, 3889, 4120, 4365, 4625, 4900, 5191, 5500, 5827, 6174,
		6541, 6930, 7342, 7778, 8241, 8731, 9250, 9800, 10383, 11000, 11654, 12347,
		13081, 13859, 14683, 15556, 16481, 17461, 18500, 19600, 20765, 22000, 23308, 24694,
		26163, 27718, 29366, 31113, 32963, 34923, 36999, 39200, 41530, 44000, 46616, 49388,
		52325, 55437, 58733, 62225, 65925, 69846, 73999, 78399, 83061, 88000, 93233, 98777,
		104650, 110873, 117466, 124451, 131851, 139691, 147998, 156798, 166122, 176000, 186466, 197553,
		209300, 221746, 234932, 248902, 263702, 279383, 295996, 313596, 332244, 352000, 372931, 395107,
		418601, 443492, 469863, 497803, 527404, 558765, 591991, 627193, 664488, 704000, 745862, 790213
	};
	enum duration_t {
		l1 = 64,
		l2 = 32,
		l2d = 48,
		l4 = 16,
		l8 = 8,
		l16 = 4
	};
};
struct note_t {
	size_t note;
	uint8_t duration;
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
	note_t{D5, l2d-(4*l16)},

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
	note_t{A4,l2d - 4*l16},

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
	note_t{D5, l2d-4*l16},

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

void play_task_function(void*)
{
	TIM_PWM->CR1 &= ~(TIM_CR1_CEN);
	const auto fade = configTICK_RATE_HZ/100;
	for(;;) {
		for (const auto& m : music) {
			// on
			const auto note_idx = m.note;
			const uint16_t arr = uint32_t(CLOCK_SPEED)/(notes::freq100[note_idx]/100) - 1;
			TIM_PWM->ARR = arr;
			TIM_PWM->CCR1 = arr / 2 + 1;
			TIM_PWM->CR1 |= TIM_CR1_CEN;
			vTaskDelay(configTICK_RATE_HZ/64 * m.duration - fade);

			// off
			TIM_PWM->CR1 &= ~(TIM_CR1_CEN);
			vTaskDelay(fade);
		}
		vTaskDelay(configTICK_RATE_HZ*3);
	}
}

void create_task_play()
{
	xTaskCreateStatic(
		&play_task_function,
		"play",
		play_task_data.stack.size(),
		NULL, // param
		PRIO_PLAY,
		play_task_data.stack.data(),
		&play_task_data.task_buffer
	);
}

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

	create_task_play();

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

