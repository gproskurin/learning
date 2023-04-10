#include "lib_stm32.h"
#include "logging.h"
#include "ad5932.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 1
#define PRIO_AD 1
#define PRIO_LOGGER 2

#if defined TARGET_STM32F103
	const stm32_lib::gpio::gpio_pin_t pin_led(GPIOB, 12);

	// USART1, tx(PA9)
	#define USART_LOG USART1
	const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOA, 9);

	// ad5932 spi
	#define AD_SPI SPI1
	const stm32_lib::gpio::gpio_pin_t ad_spi_mosi(GPIOA, 7);
	const stm32_lib::gpio::gpio_pin_t ad_spi_miso(GPIOA, 6);
	const stm32_lib::gpio::gpio_pin_t ad_spi_sck(GPIOA, 5);
	const stm32_lib::gpio::gpio_pin_t ad_spi_ss(GPIOA, 4);

	const stm32_lib::gpio::gpio_pin_t pin_ad_pwm_mclk(GPIOA, 2);
	#define TIM_AD_PWM_MCLK TIM2

#elif defined TARGET_STM32L072
	const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 5);
	const stm32_lib::gpio::gpio_pin_t pin_led_blue(GPIOB, 6);
	const stm32_lib::gpio::gpio_pin_t pin_led_red(GPIOB, 7);
	const stm32_lib::gpio::gpio_pin_t pin_led_green2(GPIOA, 5);

	// USART1, tx(PA9)
	#define USART_LOG USART1
	const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOA, 9);
	//#define USART_LOG_PIN_RX 10
	#define USART_LOG_AF 4

	// ad5932 spi
	#define AD_SPI SPI1
	const stm32_lib::gpio::gpio_pin_t ad_spi_mosi(GPIOA, 7);
	#define AD_SPI_MOSI_AF 0
	const stm32_lib::gpio::gpio_pin_t ad_spi_miso(GPIOA, 6);
	#define AD_SPI_MISO_AF 0
	const stm32_lib::gpio::gpio_pin_t ad_spi_sck(GPIOB, 3);
	#define AD_SPI_SCK_AF 0
	const stm32_lib::gpio::gpio_pin_t ad_spi_ss(GPIOA, 4);
	#define AD_SPI_SS_AF 0

#elif defined TARGET_STM32L432
	const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 3);

	// USART1
	#define USART_LOG USART1
	#define USART_LOG_AF 7
	const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOB, 6);

	// ad5932 spi
	#define AD_SPI SPI1
	const stm32_lib::gpio::gpio_pin_t ad_spi_mosi(GPIOA, 7);
	#define AD_SPI_MOSI_AF 5
	const stm32_lib::gpio::gpio_pin_t ad_spi_miso(GPIOA, 6);
	#define AD_SPI_MISO_AF 5
	const stm32_lib::gpio::gpio_pin_t ad_spi_sck(GPIOA, 5);
	#define AD_SPI_SCK_AF 5
	const stm32_lib::gpio::gpio_pin_t ad_spi_ss(GPIOA, 4);
	#define AD_SPI_SS_AF 5

	const stm32_lib::gpio::gpio_pin_t pin_ad_pwm_mclk(GPIOA, 2);
	#define AD_PWM_MCLK_AF 14
	#define TIM_AD_PWM_MCLK TIM15
	const stm32_lib::gpio::gpio_pin_t pin_ad_ctrl(GPIOA, 3);

#elif defined TARGET_STM32H745_CM4
	//const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 0);
	const stm32_lib::gpio::gpio_pin_t pin_led_yellow(GPIOE, 1);
	const stm32_lib::gpio::gpio_pin_t pin_led_red(GPIOB, 14);

	// USART1, tx(PB6)
	#define USART_LOG USART1
	#define USART_LOG_AF 7
	const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOB, 6);

	// ad5932 spi
	#define AD_SPI SPI3
	const stm32_lib::gpio::gpio_pin_t ad_spi_mosi(GPIOB, 5);
	#define AD_SPI_MOSI_AF 7
	const stm32_lib::gpio::gpio_pin_t ad_spi_miso(GPIOB, 4);
	#define AD_SPI_MISO_AF 6
	const stm32_lib::gpio::gpio_pin_t ad_spi_sck(GPIOB, 3);
	#define AD_SPI_SCK_AF 6
	const stm32_lib::gpio::gpio_pin_t ad_spi_ss(GPIOA, 4);
	#define AD_SPI_SS_AF 6

#elif defined TARGET_STM32H745_CM7
	//const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 0);
	const stm32_lib::gpio::gpio_pin_t pin_led_yellow(GPIOE, 1);
	const stm32_lib::gpio::gpio_pin_t pin_led_red(GPIOB, 14);

	// USART1, tx(PB6)
	#define USART_LOG USART1
	#define USART_LOG_AF 7
	const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOB, 6);

	// ad5932 spi
	#define AD_SPI SPI3
	const stm32_lib::gpio::gpio_pin_t ad_spi_mosi(GPIOB, 5);
	#define AD_SPI_MOSI_AF 7
	const stm32_lib::gpio::gpio_pin_t ad_spi_miso(GPIOB, 4);
	#define AD_SPI_MISO_AF 6
	const stm32_lib::gpio::gpio_pin_t ad_spi_sck(GPIOB, 3);
	#define AD_SPI_SCK_AF 6
	const stm32_lib::gpio::gpio_pin_t ad_spi_ss(GPIOA, 4);
	#define AD_SPI_SS_AF 6
#endif


#define CLOCK_SPEED configCPU_CLOCK_HZ
#define USART_CON_BAUDRATE 115200


template <size_t StackSize>
using task_stack_t = std::array<StackType_t, StackSize>;

usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

#if defined TARGET_STM32H745_CM7 || defined TARGET_STM32H745_CM4
	constexpr uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE;
#else
	constexpr uint32_t cr1 = USART_CR1_TE;
#endif

#ifdef TARGET_STM32F103
	stm32_lib::gpio::set_mode_af_lowspeed_pu(usart_log_pin_tx);
	const uint32_t div = CLOCK_SPEED / USART_CON_BAUDRATE;
	usart->BRR = ((div / 16) << USART_BRR_DIV_Mantissa_Pos) | ((div % 16) << USART_BRR_DIV_Fraction_Pos);
#else
	stm32_lib::gpio::set_mode_af_lowspeed_pu(usart_log_pin_tx, USART_LOG_AF);
	usart->BRR = CLOCK_SPEED / USART_CON_BAUDRATE;
#endif

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
}


void basic_timer_init(TIM_TypeDef* const tim, uint16_t prescaler, uint16_t arr)
{
	tim->PSC = prescaler;
	tim->ARR = arr;
	tim->CCR1 = arr / 5 * 4;
	tim->DIER |= TIM_DIER_UIE | TIM_DIER_CC1IE; // enable hardware interrupt
	tim->CR1 = (tim->CR1 & ~(TIM_CR1_UDIS | TIM_CR1_OPM)) | TIM_CR1_CEN;
}


void ad_pwm_mclk_init(TIM_TypeDef* const tim, uint16_t prescaler, uint16_t arr)
{
	tim->CR1 = 0;
	tim->PSC = prescaler;
	tim->ARR = arr;
	tim->CNT = 0;
	tim->BDTR |= TIM_BDTR_MOE_Msk;
	const uint16_t ccr = arr / 2 + 1;
#ifdef TARGET_STM32F103
	// TIM2_CH3
	tim->CCR3 = ccr;
	tim->CCMR2 = (tim->CCMR2 & ~(TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk))
		| (0b00 << TIM_CCMR2_CC3S_Pos) // output channel
		| (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1) // 0b100 = PWM mode 1
		| TIM_CCMR2_OC3FE // output compare fast
		;
	tim->CCER |= TIM_CCER_CC3E;
#elif defined TARGET_STM32H7A3
	tim->CCR3 = ccr;
	tim->CCMR2 = (tim->CCMR2 & ~(TIM_CCMR2_CC3S_Msk | TIM_CCMR2_OC3M_Msk))
		| (0b00 << TIM_CCMR2_CC3S_Pos) // output channel
		| (0b0110 << TIM_CCMR2_OC3M_Pos) // PWM mode 1
		| TIM_CCMR2_OC3FE // output compare fast
		;
	tim->CCER |= TIM_CCER_CC3E;
#elif defined TARGET_STM32L432
	tim->CCR1 = ccr;
	tim->CCMR1 = (tim->CCMR1 & ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk))
		| (0b00 << TIM_CCMR1_CC1S_Pos) // output channel
		| (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1) // 0110 = PWM mode 1
		| TIM_CCMR1_OC1FE // output compare fast
		;
	tim->CCER |= TIM_CCER_CC1E;
#else
	tim->CCR1 = ccr;
	tim->CCMR1 = (tim->CCMR1 & ~(TIM_CCMR1_CC1S_Msk | TIM_CCMR1_OC1M_Msk))
		| (0b00 << TIM_CCMR1_CC1S_Pos) // output channel
		| (0b110 << TIM_CCMR1_OC1M_Pos) // PWM mode 1
		| TIM_CCMR1_OC1FE // output compare fast
		;
	tim->CCER |= TIM_CCER_CC1E;
#endif
	tim->CR1 = TIM_CR1_CEN;
}


void ad_spi_init()
{
#ifdef TARGET_STM32F103
	stm32_lib::spi::init_pins(ad_spi_mosi, ad_spi_miso, ad_spi_sck, ad_spi_ss);
#else
	stm32_lib::spi::init_pins(
		ad_spi_mosi, AD_SPI_MOSI_AF,
		ad_spi_miso, AD_SPI_MISO_AF,
		ad_spi_sck, AD_SPI_SCK_AF,
		ad_spi_ss, AD_SPI_SS_AF
	);
#endif
	// MODE = 2 (POL = 1, PHASE = 0) TODO check

	//AD_SPI->CR1 = (AD_SPI->CR1 & ~(SPI_CR1_CPHA_Msk)) | SPI_CR1_POL_Msk; // FIXME

#ifdef TARGET_STM32L432
	AD_SPI->CR1 = 0;
	constexpr uint32_t cr1 =
		(0b111 << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR_Msk
		| SPI_CR1_CPOL_Msk;
	AD_SPI->CR1 = cr1;

	AD_SPI->CR2 = (AD_SPI->CR2 & ~(SPI_CR2_DS_Msk))
		| SPI_CR2_SSOE
		| (0b1111 << SPI_CR2_DS_Pos)
		;

	AD_SPI->CR1 = cr1 | SPI_CR1_SPE;

#elif defined TARGET_STM32H7A3
	AD_SPI->CFG1 = (AD_SPI->CFG1 & ~(SPI_CFG1_MBR_Msk | SPI_CFG1_DSIZE_Msk))
		| (0b111 << SPI_CFG1_MBR_Pos) // spi_master_clk/256
		| (0b01111 << SPI_CFG1_DSIZE_Pos) // 16 bits
		;
	AD_SPI->CFG2 = (AD_SPI->CFG2 & ~(SPI_CFG2_CPHA_Msk | SPI_CFG2_MIDI_Msk))
		| SPI_CFG2_CPOL_Msk
		| SPI_CFG2_MASTER_Msk
		| SPI_CFG2_SSOE_Msk
		| (0b1111 << SPI_CFG2_MIDI_Pos) // FIXME delay
		;
	AD_SPI->CR1 |= SPI_CR1_SPE;
#elif defined TARGET_STM32L072
#endif
}


void delay(int val)
{
	for (volatile int i = 0; i<val; ++i) {
	}
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
#ifdef TARGET_STM32L152
	stm32_lib::rcc::init_clock();
#endif

#if defined TARGET_STM32F103
	// GPIOs & USART
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN_Msk | RCC_APB2ENR_IOPBEN_Msk | RCC_APB2ENR_USART1EN_Msk;
	toggle_bits_10(
		&RCC->APB2RSTR,
		RCC_APB2RSTR_IOPARST_Msk | RCC_APB2RSTR_IOPBRST_Msk |RCC_APB2RSTR_USART1RST_Msk
	);

	// TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_TIM2RST_Msk);

#elif defined TARGET_STM32L072
	// enable GPIOs
	RCC->IOPENR |= RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk;

	// USART & SPI1
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk | RCC_APB2ENR_SPI1EN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_USART1RST_Msk | RCC_APB2RSTR_SPI1RST_Msk);

	// TIM2
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_TIM2RST_Msk);

#elif defined TARGET_STM32L432
	// enable GPIOs
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk | RCC_AHB2ENR_GPIOBEN_Msk;
	toggle_bits_10(&RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOARST_Msk | RCC_AHB2RSTR_GPIOBRST_Msk);

	// SPI1 & TIM15 & USART1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk | RCC_APB2ENR_TIM15EN_Msk | RCC_APB2ENR_USART1EN_Msk;
	toggle_bits_10(
		&RCC->APB2RSTR,
		RCC_APB2RSTR_SPI1RST_Msk | RCC_APB2RSTR_TIM15RST_Msk | RCC_APB2RSTR_USART1RST_Msk
	);

#elif defined TARGET_STM32H7A3
	RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN_Msk | RCC_AHB4ENR_GPIOBEN_Msk | RCC_AHB4ENR_GPIOEEN_Msk;
	toggle_bits_10(
		&RCC->AHB4RSTR,
		RCC_AHB4RSTR_GPIOARST_Msk | RCC_AHB4RSTR_GPIOBRST_Msk | RCC_AHB4RSTR_GPIOERST_Msk
	);

	// USART
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_USART1RST_Msk);

	// TIM3 & SPI3
	RCC->APB1LENR |= RCC_APB1LENR_TIM3EN_Msk | RCC_APB1LENR_SPI3EN_Msk;
	toggle_bits_10(&RCC->APB1LRSTR, RCC_APB1LRSTR_TIM3RST_Msk | RCC_APB1LRSTR_SPI3RST_Msk);
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


void do_blink(const stm32_lib::gpio::gpio_pin_t& pin, int n)
{
	while (n-- > 0) {
		stm32_lib::gpio::set_state(pin, 1);
		delay(500000);
		stm32_lib::gpio::set_state(pin, 0);
		delay(100000);
	}
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
#ifdef TARGET_STM32F103
	std::array<blink_task_data_t, 1> tasks = {
		blink_task_data_t(
			"blink",
			pin_led,
			configTICK_RATE_HZ/8,
			configTICK_RATE_HZ/16
		)
	};
#elif defined TARGET_STM32H7A3
	std::array<blink_task_data_t, 2> tasks = {
		blink_task_data_t(
			"blink_yellow",
			pin_led_yellow,
			configTICK_RATE_HZ/20,
			configTICK_RATE_HZ/20
		)
		, blink_task_data_t(
			"blink_red",
			pin_led_red,
			configTICK_RATE_HZ/7,
			configTICK_RATE_HZ/13
		)
	};
#elif defined TARGET_STM32L072
	std::array<blink_task_data_t, 3> tasks = {
		blink_task_data_t(
			"blink_green",
			pin_led_green,
			configTICK_RATE_HZ/2,
			configTICK_RATE_HZ/2
		)
		, blink_task_data_t(
			"blink_blue",
			pin_led_blue,
			configTICK_RATE_HZ/3,
			configTICK_RATE_HZ/3
		)
		, blink_task_data_t(
			"blink_red",
			pin_led_red,
			configTICK_RATE_HZ/32,
			configTICK_RATE_HZ - configTICK_RATE_HZ/32
		)
	};
#else
	std::array<blink_task_data_t, 1> tasks = {
		blink_task_data_t(
			"blink_green",
			pin_led_green,
			configTICK_RATE_HZ/32,
			configTICK_RATE_HZ/4
		)
	};
#endif
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


struct ad_task_data_t {
	const char* const task_name;
	SPI_TypeDef* const spi;
	ad_task_data_t(const char* tname, SPI_TypeDef* s)
		: task_name(tname), spi(s)
	{}

	task_stack_t<128> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};
struct ad_task_t {
	ad_task_data_t task;
	ad_task_t(): task("ad5932", AD_SPI) {}
};
ad_task_t ad_task;

void ad_task_function(void* arg)
{
	const ad_task_data_t* const args = reinterpret_cast<ad_task_data_t*>(arg);
	ad5932_t ad(pin_ad_pwm_mclk, pin_ad_ctrl, AD_SPI, ad_spi_ss);
	ad.start();
	for(int i=0;;++i) {
		logger.log_async("AD task is working...\r\n");
		vTaskDelay(configTICK_RATE_HZ * 16);
	}
}

void create_ad_task(ad_task_data_t& args)
{
	logger.log_sync("Creating AD task...\r\n");
	args.task_handle = xTaskCreateStatic(
		&ad_task_function,
		args.task_name,
		args.stack.size(),
		reinterpret_cast<void*>(&args),
		PRIO_AD,
		args.stack.data(),
		&args.task_buffer
	);
	logger.log_sync("Created AD task\r\n");
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

	logger.log_sync("Initializing ad5932 periph...\r\n");
#ifdef TARGET_STM32F103
	stm32_lib::gpio::set_mode_af_lowspeed_pushpull(pin_ad_pwm_mclk);
#else
	stm32_lib::gpio::set_mode_af_lowspeed_pushpull(pin_ad_pwm_mclk, AD_PWM_MCLK_AF);
#endif
	ad_pwm_mclk_init(TIM_AD_PWM_MCLK, 0, CLOCK_SPEED/200000 - 1); // 200kHz
	stm32_lib::gpio::set_mode_output_lowspeed_pushpull(pin_ad_ctrl);
	stm32_lib::gpio::set_state(pin_ad_ctrl, 0);
	ad_spi_init();
	stm32_lib::gpio::set_state(ad_spi_ss, 1);
	logger.log_sync("Initialized ad5932 periph\r\n");
	create_ad_task(ad_task.task);

	logger.log_sync("Creating blink tasks...\r\n");
	for (auto& bt : blink_tasks.tasks) {
		stm32_lib::gpio::set_mode_output_lowspeed_pushpull(bt.pin);
		create_blink_task(bt);
	}
	logger.log_sync("Created blink tasks\r\n");

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

