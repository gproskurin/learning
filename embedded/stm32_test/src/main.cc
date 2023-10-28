#include "lib_stm32.h"
#include "bsp.h"
#include "freertos_utils.h"
#include "logging.h"
#include "ad5932.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>


#define PRIO_BLINK 1
#define PRIO_AD 1
#define PRIO_LOGGER 2

#if defined TARGET_STM32F103
	// ad5932 spi
	#define AD_SPI SPI1
	constexpr stm32_lib::gpio::gpio_pin_t pin_ad_spi_mosi{GPIOA_BASE, 7};
	constexpr stm32_lib::gpio::gpio_pin_t pin_ad_spi_miso{GPIOA_BASE, 6};
	constexpr stm32_lib::gpio::gpio_pin_t pin_ad_spi_sck{GPIOA_BASE, 5};
	constexpr stm32_lib::gpio::gpio_pin_t pin_ad_spi_ss{GPIOA_BASE, 4};

	constexpr stm32_lib::gpio::gpio_pin_t pin_ad_pwm_mclk{GPIOA_BASE, 2};
	#define TIM_AD_PWM_MCLK TIM2

#elif defined TARGET_STM32L072
	// ad5932 spi
	#define AD_SPI SPI1
	const stm32_lib::gpio::gpio_pin_t pin_ad_spi_mosi{GPIOA_BASE, 7};
	#define AD_SPI_MOSI_AF 0
	const stm32_lib::gpio::gpio_pin_t pin_ad_spi_miso{GPIOA_BASE, 6};
	#define AD_SPI_MISO_AF 0
	const stm32_lib::gpio::gpio_pin_t pin_ad_spi_sck{GPIOB_BASE, 3};
	#define AD_SPI_SCK_AF 0

	const stm32_lib::gpio::gpio_pin_t pin_ad_spi_ss{GPIOA_BASE, 4};
#endif


#define USART_CON_BAUDRATE 115200


usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_TE;

#ifdef TARGET_STM32F103
	stm32_lib::gpio::set_mode_af_lowspeed_pu(usart_stlink_pin_tx);
	const uint32_t div = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;
	usart->BRR = ((div / 16) << USART_BRR_DIV_Mantissa_Pos) | ((div % 16) << USART_BRR_DIV_Fraction_Pos);
#else
	stm32_lib::gpio::set_mode_af_lowspeed_pu(bsp::usart_stlink_pin_tx, USART_STLINK_PIN_TX_AF);
	usart->BRR = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;
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
	stm32_lib::spi::init_pins(pin_ad_spi_mosi, pin_ad_spi_miso, pin_ad_spi_sck);
#else
	stm32_lib::spi::init_pins(
		pin_ad_spi_mosi, AD_SPI_MOSI_AF,
		pin_ad_spi_miso, AD_SPI_MISO_AF,
		pin_ad_spi_sck, AD_SPI_SCK_AF
	);
#endif
	stm32_lib::spi::init_pin_nss(pin_ad_spi_ss);

	// MODE = 2 (POL = 1, PHASE = 0) TODO check
	//AD_SPI->CR1 = (AD_SPI->CR1 & ~(SPI_CR1_CPHA_Msk)) | SPI_CR1_POL_Msk; // FIXME
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
using idle_task_stack_t = freertos_utils::task_stack_t<128>;
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


#ifdef TARGET_STM32F103
freertos_utils::pin_toggle_task_t g_pin_led("blink_led", bsp::pin_led, PRIO_BLINK);
#else
freertos_utils::pin_toggle_task_t g_pin_led("blink_green", bsp::pin_led_green, PRIO_BLINK);
#endif


void str_cpy_3(char* dst, const char* s1, const char* s2, const char* s3)
{
	auto p = stpcpy(dst, s1);
	p = stpcpy(p, s2);
	stpcpy(p, s3);
}


struct ad_task_data_t {
	const char* const task_name;
	SPI_TypeDef* const spi;
	ad_task_data_t(const char* tname, SPI_TypeDef* s)
		: task_name(tname), spi(s)
	{}

	freertos_utils::task_stack_t<128> stack;
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

	g_pin_led.init_pin();
	g_pin_led.pulse_continuous(configTICK_RATE_HZ/10, configTICK_RATE_HZ/4);

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

