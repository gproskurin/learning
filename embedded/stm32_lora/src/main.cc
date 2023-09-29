#include "lib_stm32.h"
#include "freertos_utils.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 1
#define PRIO_LORA 3
#define PRIO_LOGGER 2

const stm32_lib::gpio::gpio_pin_t pin_led_green(GPIOB, 5);
const stm32_lib::gpio::gpio_pin_t pin_led_blue(GPIOB, 6);
const stm32_lib::gpio::gpio_pin_t pin_led_red(GPIOB, 7);
const stm32_lib::gpio::gpio_pin_t pin_led_green2(GPIOA, 5);
const stm32_lib::gpio::gpio_pin_t pin_userbutton(GPIOB, 2);

// USART2 (st-link vcom)
#define USART_LOG USART2
#define USART_LOG_AF 4
const stm32_lib::gpio::gpio_pin_t usart_log_pin_tx(GPIOA, 2);

// SX1276 SPI
#define SPI_SX1276_AF 0
const stm32_lib::gpio::gpio_pin_t pin_sxspi_nss(GPIOA, 15);
const stm32_lib::gpio::gpio_pin_t pin_sxspi_sck(GPIOB, 3);
const stm32_lib::gpio::gpio_pin_t pin_sxspi_miso(GPIOA, 6);
const stm32_lib::gpio::gpio_pin_t pin_sxspi_mosi(GPIOA, 7);

const std::array<stm32_lib::gpio::gpio_pin_t, 4> pins_dio{
	stm32_lib::gpio::gpio_pin_t(GPIOB, 4),
	stm32_lib::gpio::gpio_pin_t(GPIOB, 1),
	stm32_lib::gpio::gpio_pin_t(GPIOB, 0),
	stm32_lib::gpio::gpio_pin_t(GPIOC, 13)
};

const stm32_lib::gpio::gpio_pin_t pin_radio_reset(GPIOC, 0);

const stm32_lib::gpio::gpio_pin_t pin_radio_tcxo_vcc(GPIOA, 12);
const stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_rx(GPIOA, 1);
const stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_boost(GPIOC, 1);
const stm32_lib::gpio::gpio_pin_t pin_radio_ant_sw_tx_rfo(GPIOC, 2);


#define CLOCK_SPEED configCPU_CLOCK_HZ
#define USART_CON_BAUDRATE 115200


template <size_t StackSize>
using task_stack_t = std::array<StackType_t, StackSize>;

usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_TE;

	stm32_lib::gpio::set_mode_af_lowspeed_pu(usart_log_pin_tx, USART_LOG_AF);
	usart->BRR = CLOCK_SPEED / USART_CON_BAUDRATE;

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
}


void delay(int val)
{
	for (volatile int i = 0; i<val; ++i) {
	}
}


void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void bus_init()
{
#if defined TARGET_STM32L072
	// flash
	FLASH->ACR |= FLASH_ACR_LATENCY;

	// clock: switch from MSI to HSI
	// turn on HSI & wait
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)) {}
	// switch to HSI
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0b01 << RCC_CFGR_SW_Pos);
	// switch off MSI & wait
	RCC->CR &= ~RCC_CR_MSION;
	while (RCC->CR & RCC_CR_MSIRDY) {}

	// GPIOs
	RCC->IOPENR = RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk | RCC_IOPENR_IOPCEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_IOPARST | RCC_IOPRSTR_IOPBRST | RCC_IOPRSTR_IOPCRST);

	// USART2 & SPI2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN_Msk | RCC_APB1ENR_SPI2EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_USART2RST_Msk | RCC_APB1RSTR_SPI2RST_Msk);

	// SPI1 & SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk | RCC_APB2ENR_SYSCFGEN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST_Msk | RCC_APB2RSTR_SYSCFGRST_Msk);

	// PB2 interrupt TODO: do not hardcode PB2, use bit masks stuff
	{
		auto const cr = &SYSCFG->EXTICR[pin_userbutton.reg / 4];
		*cr = (*cr & ~(SYSCFG_EXTICR1_EXTI2_Msk)) | SYSCFG_EXTICR1_EXTI2_PB;
	}

	// EXTI
	pin_userbutton.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::no_pupd);
	EXTI->IMR |= (1 << pin_userbutton.reg);
	EXTI->RTSR &= ~(1 << pin_userbutton.reg); // disable rising edge interrupt (button release)
	EXTI->FTSR |= (1 << pin_userbutton.reg); // enable falling edge interrupt (button press)
	NVIC_SetPriority(EXTI2_3_IRQn, 13);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
#endif
}


volatile bool blue_state = false;
extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI23()
{
	if (EXTI->PR & (1 << pin_userbutton.reg)) {
		stm32_lib::gpio::set_state(pin_led_blue, (blue_state = !blue_state));
		EXTI->PR = (1 << pin_userbutton.reg);
	}
}


StaticTask_t xTaskBufferIdle;
task_stack_t<64> idle_task_stack;
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


freertos_utils::pin_toggle_task_t g_pin_green2("blink_green2", pin_led_green2, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", pin_led_red, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green("blink_green", pin_led_green, PRIO_BLINK);


char* print_bits(uint8_t x, char* buf)
{
	for (size_t i=0; i<8; ++i) {
		*buf++ = (x & 0b10000000) ? '1' : '0';
		x <<= 1;
	}
	*buf++ = '\r';
	*buf++ = '\n';
	*buf = 0;
	return buf;
}


void init_radio_pin(const stm32_lib::gpio::gpio_pin_t& pin)
{
	pin.set(
		stm32_lib::gpio::mode_t::output,
		stm32_lib::gpio::otype_t::push_pull,
		stm32_lib::gpio::pupd_t::no_pupd,
		stm32_lib::gpio::speed_t::bits_11
	);
}


void spi_sx_init()
{
	stm32_lib::spi::init_pin_nss(pin_sxspi_nss);
	stm32_lib::spi::init_pins(
		pin_sxspi_mosi, SPI_SX1276_AF,
		pin_sxspi_miso, SPI_SX1276_AF,
		pin_sxspi_sck, SPI_SX1276_AF
	);

	// CPOL=0 CPHA=0, Motorola mode
	SPI1->CR1 = 0;
	constexpr uint32_t cr1 =
		// SPI_CR1_DFF_Msk
		(0b011 << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR_Msk
		| SPI_CR1_SSM_Msk
		;
	SPI1->CR1 = cr1;

	SPI1->CR2 = SPI_CR2_SSOE;

	SPI1->CR1 = cr1 | SPI_CR1_SPE;
}


// LORA task
struct lora_task_data_t {
	const char* const task_name;
	lora_task_data_t(const char* tname) : task_name(tname) {}

	task_stack_t<256> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
} lora_task_data("lora");

void lora_task_function(void* arg)
{
	spi_sx_init();

	init_radio_pin(pin_radio_tcxo_vcc);
	init_radio_pin(pin_radio_ant_sw_rx);
	init_radio_pin(pin_radio_ant_sw_tx_boost);
	init_radio_pin(pin_radio_ant_sw_tx_rfo);

	stm32_lib::gpio::set_state(pin_radio_tcxo_vcc, 1);
	vTaskDelay(configTICK_RATE_HZ/10);

	// init dio
	for (const auto& p : pins_dio) {
		using namespace stm32_lib::gpio;
		using mode_t = stm32_lib::gpio::mode_t;
		p.set(mode_t::input, pupd_t::pd, speed_t::bits_11);
	}

	// reset radio
	{
		using namespace stm32_lib::gpio;
		using mode_t = stm32_lib::gpio::mode_t;
		pin_radio_reset.set(
			stm32_lib::gpio::mode_t::output,
			stm32_lib::gpio::otype_t::push_pull,
			stm32_lib::gpio::pupd_t::no_pupd,
			stm32_lib::gpio::speed_t::bits_11
		);
		pin_radio_reset.set(mode_t::output, otype_t::push_pull, pupd_t::pu);

		stm32_lib::gpio::set_state(pin_radio_reset, 0);
		vTaskDelay(configTICK_RATE_HZ/10);

		pin_radio_reset.set(
			stm32_lib::gpio::mode_t::input,
			stm32_lib::gpio::pupd_t::no_pupd
		);
		vTaskDelay(configTICK_RATE_HZ/10);
	}

	stm32_lib::spi::spi_t spi(SPI1, pin_sxspi_nss);

	bool r = false;
	for (uint16_t reg=0x01; reg<= 0x1d; ++reg) {
		auto x = spi.write<uint16_t>(reg);
		//auto r = spi.write<uint8_t>(0x55);
		if (x)
			r = true;
	}

#if 0
		char buf[16];
		int i=0;
		while(r) {
			buf[i] = (r & 1) + '0';
			r = r >> 1;
			++i;
		}
		buf[i++] = '\r'; buf[i++] = '\n'; buf[i++] = 0;
		logger.log_async(buf);
#endif
		logger.log_async("SRI_WRITE done\r\n");
		if (!r) {
			logger.log_async("SRI_WRITE - zero\r\n");
			g_pin_red.on();
		} else {
			logger.log_async(" *** SRI_WRITE - unknown\r\n");
			g_pin_blue.on();
		}

	const lora_task_data_t* const args = reinterpret_cast<lora_task_data_t*>(arg);
	stm32_lib::gpio::set_mode_output_lowspeed_pushpull(pin_led_blue);
	for(;;) {
		//logger.log_async("LORA keepalive\r\n");
		g_pin_green.pulse_once(configTICK_RATE_HZ/16);
		vTaskDelay(configTICK_RATE_HZ);
	}
}

void create_lora_task(lora_task_data_t& args)
{
	args.task_handle = xTaskCreateStatic(
		&lora_task_function,
		args.task_name,
		args.stack.size(),
		reinterpret_cast<void*>(&args),
		PRIO_LORA,
		args.stack.data(),
		&args.task_buffer
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

	g_pin_green.init_pin();
	g_pin_green2.init_pin();
	g_pin_green2.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/25);
	g_pin_blue.init_pin();
	g_pin_red.init_pin();

	logger.log_sync("Creating LORA task...\r\n");
	create_lora_task(lora_task_data);
	logger.log_sync("Created LORA task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

