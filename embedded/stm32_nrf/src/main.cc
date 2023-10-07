#include "lib_stm32.h"
#include "freertos_utils.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 1
#define PRIO_NRF 3
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


#define SPI_NRF SPI2
#define SPI_NRF_AF 0
// MSB first, CPOL=0, CPHA=0
const stm32_lib::gpio::gpio_pin_t pin_nrfspi_nss(GPIOB, 12);
const stm32_lib::gpio::gpio_pin_t pin_nrfspi_sck(GPIOB, 13);
const stm32_lib::gpio::gpio_pin_t pin_nrfspi_miso(GPIOB, 14);
const stm32_lib::gpio::gpio_pin_t pin_nrfspi_mosi(GPIOB, 15);

//const stm32_lib::gpio::gpio_pin_t pin_nrf_vcc(GPIOA, 0); // TODO
const stm32_lib::gpio::gpio_pin_t pin_nrf_irq(GPIOA, 0); // active low
const stm32_lib::gpio::gpio_pin_t pin_nrf_ce(GPIOA, 4);


#define CLOCK_SPEED configCPU_CLOCK_HZ
#define USART_CON_BAUDRATE 115200


template <size_t StackSize>
using task_stack_t = std::array<StackType_t, StackSize>;

usart_logger_t logger;


// NRF task
struct nrf_task_data_t {
	task_stack_t<256> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};

nrf_task_data_t nrf1_task_data;


enum reg_t : uint8_t {
	CMD_R_REGISTER = 0b00000000,
	CMD_W_REGISTER = 0b00100000,
	CMD_NOP = 0b11111111,

	REG_CONFIG = 0x00,
	REGV_CONFIG_PWR_UP = 0b10,
	REGV_CONFIG_PRIM_RX_PRX = 0b1,
	//REGV_CONFIG_PRIM_RX_PTX = 0

	REG_FIFO_STATUS = 0x17
};


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_TE;

	stm32_lib::gpio::set_mode_af_lowspeed_pu(usart_log_pin_tx, USART_LOG_AF);
	usart->BRR = CLOCK_SPEED / USART_CON_BAUDRATE;

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
	RCC->APB2ENR |=/* RCC_APB2ENR_SPI1EN_Msk |*/ RCC_APB2ENR_SYSCFGEN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, /*RCC_APB2RSTR_SPI1RST_Msk |*/ RCC_APB2RSTR_SYSCFGRST_Msk);

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


extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI23()
{
	if (EXTI->PR & (1 << pin_userbutton.reg)) {
		EXTI->PR = (1 << pin_userbutton.reg);
		{
			BaseType_t yield = pdFALSE;
			xTaskNotifyFromISR(nrf1_task_data.task_handle, 1, eSetBits, &yield);
			portYIELD_FROM_ISR(yield);
		}
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


void spi_nrf_init()
{
	stm32_lib::spi::init_pin_nss(pin_nrfspi_nss);
	stm32_lib::spi::init_pins(
		pin_nrfspi_mosi, SPI_NRF_AF,
		pin_nrfspi_miso, SPI_NRF_AF,
		pin_nrfspi_sck, SPI_NRF_AF
	);

	// CPOL=0 CPHA=0, Motorola mode
	SPI2->CR1 = 0;
	constexpr uint32_t cr1 =
		//SPI_CR1_DFF_Msk
		(0b111 << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR_Msk
		| SPI_CR1_SSM_Msk
		//| SPI_CR1_SSI_Msk
		;
	SPI2->CR1 = cr1;

	SPI_NRF->CR2 = SPI_CR2_SSOE;

	SPI_NRF->CR1 = cr1 | SPI_CR1_SPE;
}


uint8_t spi_nrf_write(
		SPI_TypeDef* const spi,
		uint8_t size,
		uint8_t cmd,
		const uint8_t* mosi_buf,
		uint8_t* miso_buf
)
{
	stm32_lib::gpio::set_state(pin_nrfspi_nss, 0);
	for (volatile int i=0; i<10; ++i) {} // 2ns

	const uint8_t status = stm32_lib::spi::write<uint8_t>(spi, cmd);
	for (uint8_t i=0; i<size; ++i) {
		const uint8_t r = stm32_lib::spi::write<uint8_t>(spi, (mosi_buf ? mosi_buf[i] : 0));
		if (miso_buf) {
			miso_buf[i] = r;
		}
	}

	for (volatile int i=0; i<10; ++i) {} // 2ns
	stm32_lib::gpio::set_state(pin_nrfspi_nss, 1);
	return status;
}


void nrf_st_cf()
{
	uint8_t cf;
	const uint8_t st = spi_nrf_write(SPI_NRF, 1, reg_t::CMD_R_REGISTER | reg_t::REG_CONFIG, nullptr, &cf);

	static char buf_st[8+3];
	logger.log_async("NRF status\r\n");
	print_bits(st, buf_st);
	logger.log_async(buf_st);

	static char buf_cf[8+3];
	logger.log_async("NRF config\r\n");
	print_bits(cf, buf_cf);
	logger.log_async(buf_cf);

	static char buf_fifost[8+3];
	uint8_t fs;
	spi_nrf_write(SPI_NRF, 1, reg_t::CMD_R_REGISTER | reg_t::REG_FIFO_STATUS, nullptr, &fs);
	logger.log_async("NRF fifo_status\r\n");
	print_bits(fs, buf_fifost);
	logger.log_async(buf_fifost);
	vTaskDelay(configTICK_RATE_HZ/10);
}


void nrf_task_function(void*)
{
	logger.log_async("NRF1 task started\r\n");
	pin_nrf_irq.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::pu);
	spi_nrf_init();
	stm32_lib::gpio::set_mode_output_lowspeed_pushpull(pin_nrf_ce);
	vTaskDelay(configTICK_RATE_HZ/10);

	logger.log_async("NRF1 init done\r\n");

	nrf_st_cf();

	// switch to standby-1
	stm32_lib::gpio::set_state(pin_nrf_ce, 1);
	vTaskDelay(configTICK_RATE_HZ/10);

	// pwr up
	{
		logger.log_async("NRF1 pwr_up\r\n");
		stm32_lib::gpio::set_state(pin_nrf_ce, 0); // standby-1
		uint8_t cf;
		spi_nrf_write(SPI_NRF, 1, reg_t::CMD_R_REGISTER | reg_t::REG_CONFIG, nullptr, &cf);
		cf |= reg_t::REGV_CONFIG_PWR_UP;
		spi_nrf_write(SPI_NRF, 1, reg_t::CMD_W_REGISTER | reg_t::REG_CONFIG, &cf, nullptr);

		vTaskDelay(configTICK_RATE_HZ/10);
		nrf_st_cf();

		cf |= REGV_CONFIG_PRIM_RX_PRX;
		spi_nrf_write(SPI_NRF, 1, reg_t::CMD_W_REGISTER | reg_t::REG_CONFIG, &cf, nullptr);
		vTaskDelay(configTICK_RATE_HZ/10);
		nrf_st_cf();
	}


	for(;;) {
		uint32_t events = 0;
		if (xTaskNotifyWait(0, 0xffffffff, &events, configTICK_RATE_HZ * 5 /*5sec*/) == pdTRUE) {
			if (events) {
				g_pin_blue.pulse_once(configTICK_RATE_HZ/2);
			}
		}
		g_pin_green.pulse_once(configTICK_RATE_HZ/16);
		nrf_st_cf();
		logger.log_async("---\r\n");
	}
}

void create_nrf_task(const char* task_name, UBaseType_t prio, nrf_task_data_t& args)
{
	args.task_handle = xTaskCreateStatic(
		&nrf_task_function,
		task_name,
		args.stack.size(),
		nullptr,
		prio,
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

	logger.log_sync("Creating NRF-1 task...\r\n");
	create_nrf_task("nrf1_task", PRIO_NRF, nrf1_task_data);
	logger.log_sync("Created NRF-1 task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

