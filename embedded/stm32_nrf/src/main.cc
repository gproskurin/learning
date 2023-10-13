#include "lib_stm32.h"
#include "bsp.h"
#include "nrf24.h"
#include "freertos_utils.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>


#define PRIO_BLINK 1
#define PRIO_NRF 3
#define PRIO_BUTTONS_POLL 4
#define PRIO_LOGGER 2


#define USART_CON_BAUDRATE 115200


usart_logger_t logger;


const nrf24::hw_conf_t nrf1_conf{
	.spi = SPI1,
	.spi_af = 0,
	.pin_spi_nss{GPIOB_BASE, 9},
	.pin_spi_sck{GPIOA_BASE, 15},
	.pin_spi_miso{GPIOA_BASE, 11},
	.pin_spi_mosi{GPIOA_BASE, 12},
	.pin_irq{GPIOB_BASE, 8},
	.pin_ce{GPIOA_BASE, 4}
};

const nrf24::hw_conf_t nrf2_conf{
	.spi = SPI2,
	.spi_af = 0,
	.pin_spi_nss{GPIOB_BASE, 12},
	.pin_spi_sck{GPIOB_BASE, 13},
	.pin_spi_miso{GPIOB_BASE, 14},
	.pin_spi_mosi{GPIOB_BASE, 15},
	.pin_irq{GPIOA_BASE, 0},
	.pin_ce{GPIOA_BASE, 4}
};


// NRF task
struct nrf_task_data_t {
	freertos_utils::task_stack_t<256> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};

nrf_task_data_t nrf1_task_data;
nrf_task_data_t nrf2_task_data;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE;

	stm32_lib::gpio::set_mode_af_lowspeed_pu(bsp::usart_stlink_pin_tx, USART_STLINK_PIN_TX_AF);
	usart->BRR = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;

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
#if defined TARGET_STM32WB55
	// flash & clock
	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk) | (0b010 << FLASH_ACR_LATENCY_Pos);
	RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE_Msk) | (0b1011 << RCC_CR_MSIRANGE_Pos);

	// GPIOs
	RCC->AHB2ENR |=
		RCC_AHB2ENR_GPIOAEN_Msk
		| RCC_AHB2ENR_GPIOBEN_Msk
		| RCC_AHB2ENR_GPIOCEN_Msk
		| RCC_AHB2ENR_GPIODEN_Msk;
	toggle_bits_10(
		&RCC->AHB2RSTR,
		RCC_AHB2RSTR_GPIOARST
			| RCC_AHB2RSTR_GPIOBRST
			| RCC_AHB2RSTR_GPIOCRST
			| RCC_AHB2RSTR_GPIODRST
	);

	// SPI2
	RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR1, RCC_APB1RSTR1_SPI2RST_Msk);

	// SPI1 & USART1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk | RCC_APB2ENR_USART1EN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST_Msk | RCC_APB2RSTR_USART1RST_Msk);

#if 0
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
#endif
}


#if 0
extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI23()
{
	if (EXTI->PR & (1 << pin_userbutton.reg)) {
		EXTI->PR = (1 << pin_userbutton.reg);
		{
			BaseType_t yield = pdFALSE;
			xTaskNotifyFromISR(nrf2_task_data.task_handle, 1, eSetBits, &yield);
			portYIELD_FROM_ISR(yield);
		}
	}
}
#endif


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<64> idle_task_stack;
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


freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);


void pinpoll_cb1(const stm32_lib::gpio::pin_t& pin, bool new_status)
{
	if (new_status) {
		logger.log_async("CB1: up\r\n");
	} else {
		logger.log_async("CB1: down\r\n");
	}
}


void pinpoll_cb2(const stm32_lib::gpio::pin_t& pin, bool new_status)
{
	if (new_status) {
		logger.log_async("CB2: up\r\n");
	} else {
		logger.log_async("CB2: down\r\n");
	}
}


void pinpoll_cb3(const stm32_lib::gpio::pin_t& pin, bool new_status)
{
	if (new_status) {
		logger.log_async("CB3: up\r\n");
	} else {
		logger.log_async("CB3: down\r\n");
	}
}


void pinpoll_cb_a(const stm32_lib::gpio::pin_t& pin, bool new_status)
{
	if (new_status) {
		logger.log_async("CB_A: up\r\n");
	} else {
		logger.log_async("CB_A: down\r\n");
	}
}


auto pinpoll_task_arg = freertos_utils::pinpoll::make_task_arg(
	freertos_utils::pinpoll::make_pin_info(bsp::pin_userbutton1, &pinpoll_cb1),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_userbutton2, &pinpoll_cb2),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_userbutton3, &pinpoll_cb3),
	freertos_utils::pinpoll::make_pin_info(stm32_lib::gpio::pin_t{GPIOC_BASE,13}, &pinpoll_cb_a)
);


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


void spi_nrf_init(const nrf24::hw_conf_t& cf)
{
	stm32_lib::spi::init_pin_nss(cf.pin_spi_nss);
	stm32_lib::spi::init_pins(
		cf.pin_spi_mosi, cf.spi_af,
		cf.pin_spi_miso, cf.spi_af,
		cf.pin_spi_sck, cf.spi_af
	);

	// CPOL=0 CPHA=0, Motorola mode
	cf.spi->CR1 = 0;
	constexpr uint32_t cr1 =
		(0b111 << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR_Msk
		| SPI_CR1_SSM_Msk
		;
	cf.spi->CR1 = cr1;
	cf.spi->CR2 = SPI_CR2_SSOE;
	cf.spi->CR1 = cr1 | SPI_CR1_SPE;
}


void nrf_st_cf(const nrf24::hw_conf_t& hwc)
{
	uint8_t cf;
	const uint8_t st = nrf24::spi_write(hwc, 1, nrf24::reg_t::CMD_R_REGISTER | nrf24::reg_t::REG_CONFIG, nullptr, &cf);

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
	nrf24::spi_write(hwc, 1, nrf24::reg_t::CMD_R_REGISTER | nrf24::reg_t::REG_FIFO_STATUS, nullptr, &fs);
	logger.log_async("NRF fifo_status\r\n");
	print_bits(fs, buf_fifost);
	logger.log_async(buf_fifost);

	vTaskDelay(configTICK_RATE_HZ/10);
}


void nrf_task_function(void* arg)
{
	{
		for(;;) {
			g_pin_blue.pulse_many(configTICK_RATE_HZ/20, configTICK_RATE_HZ/10, 3);
			logger.log_async("NRF keep-alive\r\n");
			vTaskDelay(configTICK_RATE_HZ*10);
		}
	}

	const nrf24::hw_conf_t* const hwc = reinterpret_cast<const nrf24::hw_conf_t*>(arg);

	logger.log_async("NRF1 task started\r\n");
	hwc->pin_irq.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::pu);
	spi_nrf_init(*hwc);
	stm32_lib::gpio::set_mode_output_lowspeed_pushpull(hwc->pin_ce);
	vTaskDelay(configTICK_RATE_HZ/10);

	logger.log_async("NRF1 init done\r\n");

	nrf_st_cf(*hwc);

	// pwr up, RX
	{
		logger.log_async("NRF1 pwr_up\r\n");
		stm32_lib::gpio::set_state(hwc->pin_ce, 0); // standby-1
		vTaskDelay(configTICK_RATE_HZ/10);

		uint8_t cf;

		nrf24::spi_write(*hwc, 1, nrf24::reg_t::CMD_R_REGISTER | nrf24::reg_t::REG_CONFIG, nullptr, &cf);
		cf |= nrf24::reg_t::REGV_CONFIG_PWR_UP;
		nrf24::spi_write(*hwc, 1, nrf24::reg_t::CMD_W_REGISTER | nrf24::reg_t::REG_CONFIG, &cf, nullptr);
		vTaskDelay(configTICK_RATE_HZ/10);

		// RX
		cf |= nrf24::reg_t::REGV_CONFIG_PRIM_RX_PRX;
		nrf24::spi_write(*hwc, 1, nrf24::reg_t::CMD_W_REGISTER | nrf24::reg_t::REG_CONFIG, &cf, nullptr);
		vTaskDelay(configTICK_RATE_HZ/10);

		stm32_lib::gpio::set_state(hwc->pin_ce, 1);
		vTaskDelay(configTICK_RATE_HZ/10);

		nrf_st_cf(*hwc);
	}


	for(;;) {
		uint32_t events = 0;
		if (xTaskNotifyWait(0, 0xffffffff, &events, configTICK_RATE_HZ * 5 /*5sec*/) == pdTRUE) {
			if (events) {
				g_pin_blue.pulse_once(configTICK_RATE_HZ/2);
			}
		}
		g_pin_green.pulse_once(configTICK_RATE_HZ/16);
		nrf_st_cf(*hwc);
		logger.log_async("---\r\n");
	}
}


void create_nrf_task(const char* task_name, UBaseType_t prio, nrf_task_data_t& task_data, const nrf24::hw_conf_t* const hwc)
{
	task_data.task_handle = xTaskCreateStatic(
		&nrf_task_function,
		task_name,
		task_data.stack.size(),
		const_cast<void*>(reinterpret_cast<const void*>(hwc)),
		prio,
		task_data.stack.data(),
		&task_data.task_buffer
	);
}


__attribute__ ((noreturn)) void main()
{
	bus_init();

	g_pin_blue.init_pin();
	g_pin_green.init_pin();
	g_pin_red.init_pin();
	g_pin_green.pulse_continuous(configTICK_RATE_HZ/10, configTICK_RATE_HZ/4);

	usart_init(USART_STLINK);
	logger.set_usart(USART_STLINK);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");

	logger.log_sync("Creating pinpoll task...\r\n");
	freertos_utils::pinpoll::create_task("pinpoll", PRIO_BUTTONS_POLL, &pinpoll_task_arg);
	logger.log_sync("Created pinpoll task\r\n");

	logger.log_sync("Creating NRF-2 task...\r\n");
	create_nrf_task("nrf2_task", PRIO_NRF, nrf2_task_data, &nrf2_conf);
	logger.log_sync("Created NRF-2 task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

