#include "lib_stm32.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <array>
#include <new>


#define PRIO_BLINK 1
#define PRIO_LOGGER 2

#if defined TARGET_STM32F103
	// PB12
	#define LED_GPIO GPIOB
	#define LED_PIN 12
	#define LED_TIM TIM2
	#define GREEN_LED_GPIO LED_GPIO
	#define GREEN_LED_PIN LED_PIN

	// USART1, tx(PA9)
	#define USART_LOG USART1
	#define USART_LOG_GPIO GPIOA
	#define USART_LOG_PIN_TX 9
#elif defined TARGET_STM32L152
	// PA5
	#define LED_GPIO GPIOA
	#define LED_PIN 5
	#define LED_TIM TIM9
	#define GREEN_LED_GPIO LED_GPIO
	#define GREEN_LED_PIN LED_PIN

	#define PWM_GPIO GPIOB
	#define PWM_PIN 13
	#define PWM_PIN_AF 3

	// USART1, tx(PA9)
	#define USART_LOG USART1
	#define USART_LOG_GPIO GPIOA
	#define USART_LOG_PIN_TX 9
	//#define USART_LOG_PIN_RX 10
#elif defined TARGET_STM32H7A3
	#define GREEN_LED_GPIO GPIOB
	#define GREEN_LED_PIN 0
	#define YELLOW_LED_GPIO GPIOE
	#define YELLOW_LED_PIN 1
	#define RED_LED_GPIO GPIOB
	#define RED_LED_PIN 14

	#define LED_GPIO YELLOW_LED_GPIO
	#define LED_PIN YELLOW_LED_PIN

	#define LED_TIM TIM3

	// USART1, tx(PB6)
	#define USART_LOG USART1
	#define USART_LOG_GPIO GPIOB
	#define USART_LOG_PIN_TX 6
#endif


// FIXME do not hardcode
#if defined (TARGET_STM32F103)
#define CLOCK_SPEED 8000000
#elif defined (TARGET_STM32L152)
//#define CLOCK_SPEED 2097000
#define CLOCK_SPEED 4194000
#elif defined (TARGET_STM32H7A3)
#define CLOCK_SPEED 64000000
#endif
#define USART_CON_BAUDRATE 115200


usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset
#ifdef TARGET_STM32F103
	stm32_lib::gpio::set_mode_af_lowspeed_pu(USART_LOG_GPIO, USART_LOG_PIN_TX);
	const uint32_t div = CLOCK_SPEED / USART_CON_BAUDRATE;
	usart->BRR = ((div / 16) << USART_BRR_DIV_Mantissa_Pos) | ((div % 16) << USART_BRR_DIV_Fraction_Pos);
	usart->CR1 = USART_CR1_TE;
#elif defined TARGET_STM32L152
	stm32_lib::gpio::set_mode_af_lowspeed_pu(USART_LOG_GPIO, USART_LOG_PIN_TX, 7);
	usart->BRR = CLOCK_SPEED / USART_CON_BAUDRATE;
	usart->CR1 = USART_CR1_TE;
#elif defined TARGET_STM32H7A3
	stm32_lib::gpio::set_mode_af_lowspeed_pu(USART_LOG_GPIO, USART_LOG_PIN_TX, 7);
	usart->BRR = CLOCK_SPEED / USART_CON_BAUDRATE;
	usart->CR1 = USART_CR1_FIFOEN | USART_CR1_TE;
#endif
	usart->CR1 |= USART_CR1_UE;
}


void basic_timer_init(TIM_TypeDef* const tim, uint16_t prescaler, uint16_t arr)
{
	tim->PSC = prescaler;
	tim->ARR = arr;
	tim->CCR1 = arr / 5 * 4;
	tim->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE; // enable hardware interrupt
	tim->CR1 = (tim->CR1 & ~(TIM_CR1_UDIS | TIM_CR1_OPM)) | TIM_CR1_CEN;
}


void timer_init_output_pin(TIM_TypeDef* const tim, uint16_t prescaler, uint16_t arr, GPIO_TypeDef* gpio, int reg)
{
	stm32_lib::gpio::set_mode_af_hispeed_pushpull(gpio, reg, PWM_PIN_AF);
	tim->CR1 = 0;
	tim->PSC = prescaler;
	tim->ARR = arr;
	tim->CCR1 = arr / 5 * 4;
	tim->CCMR1 = (tim->CCMR1 & ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk))
		| (0b00 << TIM_CCMR1_CC1S_Pos) // output channel
		| (0b110 << TIM_CCMR1_OC1M_Pos) // PWM mode 1
		| TIM_CCMR1_OC1FE // output compare fast
		;
	tim->CCER |= TIM_CCER_CC1E;
	tim->CR1 |= TIM_CR1_CEN;
}

void delay(int val)
{
	for (volatile int i = 0; i<val; ++i) {
	}
}


extern "C" __attribute__ ((interrupt)) void IntHandler_Timer()
{
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
}


void bus_init()
{
#ifdef TARGET_STM32L152
	stm32_lib::rcc::init_clock();
#endif

#if defined TARGET_STM32F103
	// enable timer and port B
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN_Msk;
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk;

	// USART
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN_Msk;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST_Msk;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST_Msk;

	// reset TIM2
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST_Msk;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST_Msk;

#elif defined TARGET_STM32L152
	// enable GPIOs
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN_Msk | RCC_AHBENR_GPIOBEN_Msk;

	// USART
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST_Msk;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST_Msk;

	// TIM9
	RCC->APB2ENR |= RCC_APB2ENR_TIM9EN_Msk;
	RCC->APB2RSTR |= RCC_APB2RSTR_TIM9RST_Msk;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST_Msk;
#elif defined TARGET_STM32H7A3
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOBEN_Msk | RCC_AHB4ENR_GPIOEEN_Msk;
	RCC->AHB4RSTR |= RCC_AHB4RSTR_GPIOBRST_Msk | RCC_AHB4RSTR_GPIOERST_Msk;
	RCC->AHB4RSTR &= ~(RCC_AHB4RSTR_GPIOBRST_Msk | RCC_AHB4RSTR_GPIOERST_Msk);

	// USART
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
	RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST_Msk;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST_Msk;

	// TIM3
	//RCC->APB1LENR |= RCC_APB1LENR_TIM3EN_Msk;
	//RCC->APB2RSTR |= RCC_APB1LRSTR_TIM9RST_Msk;
	//RCC->APB2RSTR &= ~RCC_APB2RSTR_TIM9RST_Msk;
#endif
}


void nvic_init_tim()
{
	// enable interrupt
#if defined TARGET_STM32F103
	NVIC_SetPriority(TIM2_IRQn, 3);
	NVIC_EnableIRQ(TIM2_IRQn);
#elif defined TARGET_STM32L152
	//NVIC_SetVector(TIM9_IRQn, (uint32_t) &IntHandler_Timer);
	NVIC_SetPriority(TIM9_IRQn, 3);
	NVIC_EnableIRQ(TIM9_IRQn);
#endif
}


void do_blink(GPIO_TypeDef* const gpio, int reg, int n)
{
	while (n-- > 0) {
		stm32_lib::gpio::set_state(gpio, reg, 1);
		delay(500000);
		stm32_lib::gpio::set_state(gpio, reg, 0);
		delay(100000);
	}
}


void blink(int n)
{
	return do_blink(LED_GPIO, LED_PIN, n);
}


StaticTask_t xTaskBufferIdle;
typedef std::array<StackType_t, 128> idle_task_stack_t;
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
typedef std::array<StackType_t, 128> blink_task_stack_t;
struct blink_task_data_t {
	const char* const task_name;
	GPIO_TypeDef* const gpio;
	const int reg;
	const TickType_t ticks_on;
	const TickType_t ticks_off;
	blink_task_data_t(const char* tname, GPIO_TypeDef* gp, int r, TickType_t ton, TickType_t toff)
		: task_name(tname), gpio(gp), reg(r), ticks_on(ton), ticks_off(toff)
	{}

	blink_task_stack_t stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};

struct blink_tasks_t {
#ifdef TARGET_STM32H7A3
	std::array<blink_task_data_t, 3> tasks = {
#else
	std::array<blink_task_data_t, 1> tasks = {
#endif
		blink_task_data_t(
			"blink_green",
			GREEN_LED_GPIO,
			GREEN_LED_PIN,
			configTICK_RATE_HZ/16,
			configTICK_RATE_HZ - configTICK_RATE_HZ/16
		)
#ifdef TARGET_STM32H7A3
		, blink_task_data_t(
			"blink_yellow",
			YELLOW_LED_GPIO,
			YELLOW_LED_PIN,
			configTICK_RATE_HZ/20,
			configTICK_RATE_HZ/20
		)
		, blink_task_data_t(
			"blink_red",
			RED_LED_GPIO,
			RED_LED_PIN,
			configTICK_RATE_HZ/7,
			configTICK_RATE_HZ/13
		)
#endif
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
		stm32_lib::gpio::set_state(args->gpio, args->reg, true);
		vTaskDelay(args->ticks_on);
		if (do_log) {
			logger.log_async("LED -> off\r\n");
		}
		stm32_lib::gpio::set_state(args->gpio, args->reg, false);
		vTaskDelay(args->ticks_off);
	}
}


void create_blink_task(blink_task_data_t& args)
{
	logger.log_sync("Creating blink task...\r\n");
	args.task_handle = xTaskCreateStatic(
		&blink_task_function,
		args.task_name,
		args.stack.size(),
		reinterpret_cast<void*>(&args),
		PRIO_BLINK,
		args.stack.data(),
		&args.task_buffer
	);
	logger.log_sync("Created blink task\r\n");
}


__attribute__ ((noreturn)) void main()
{
	// call constructors of global objects
	new(&logger) usart_logger_t();
	new(&idle_task_stack) idle_task_stack_t();
	new(&blink_tasks) blink_tasks_t();

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

	logger.log_sync("Creating blink tasks...\r\n");
	for (auto& bt : blink_tasks.tasks) {
		stm32_lib::gpio::set_mode_output_lowspeed_pushpull(bt.gpio, bt.reg);
		create_blink_task(bt);
	}
	logger.log_sync("Created blink tasks\r\n");

	logger.log_sync("Initializing interrupts\r\n");
	//nvic_init_tim();

	logger.log_sync("Initializing timer\r\n");
	//basic_timer_init(LED_TIM, 2000-1, 1000-1);

	logger.log_sync("Starting PWM timer...\r\n");
	timer_init_output_pin(LED_TIM, 1, 100-1, PWM_GPIO, PWM_PIN);
	logger.log_sync("Started PWM timer\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {
		//blink(2);
		//delay(100000);
		//__WFI();
	}
}

