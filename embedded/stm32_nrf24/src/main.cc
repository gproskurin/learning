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
#define PRIO_NRF1 5
#define PRIO_NRF2 5
#define PRIO_BUTTONS_POLL 2
#define PRIO_LOGGER 3


#define USART_CON_BAUDRATE 115200


usart_logger_t logger;


const nrf24::hw_conf_t nrf1_conf{
	.spi = SPI2,
	.spi_af = 5,
	.pin_spi_nss{GPIOB_BASE, 12},
	.pin_spi_sck{GPIOB_BASE, 13},
	.pin_spi_miso{GPIOB_BASE, 14},
	.pin_spi_mosi{GPIOB_BASE, 15},
	.pin_irq{GPIOA_BASE, 14},
	.pin_ce{GPIOA_BASE, 13}
};

const nrf24::hw_conf_t nrf2_conf{
	.spi = SPI1,
	.spi_af = 5,
	.pin_spi_nss{GPIOA_BASE, 4},
	.pin_spi_sck{GPIOA_BASE, 5},
	.pin_spi_miso{GPIOA_BASE, 6},
	.pin_spi_mosi{GPIOA_BASE, 7},
	.pin_irq{GPIOC_BASE, 2},
	.pin_ce{GPIOC_BASE, 3}
};


// NRF task
using nrf_task_data_t = freertos_utils::task_data_t<1024>;

nrf_task_data_t nrf1_task_data;
nrf_task_data_t nrf2_task_data;


enum nrf_task_nf : uint32_t {
	irq = 1 << 0
};

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

	// nrf1 irq pin: A14
        {
                auto const cr = &SYSCFG->EXTICR[nrf1_conf.pin_irq.reg / 4];
                *cr = (*cr & ~SYSCFG_EXTICR4_EXTI14_Msk) | (/*PA14*/0b000 << SYSCFG_EXTICR4_EXTI14_Pos);
                EXTI->IMR1 |= (1 << nrf1_conf.pin_irq.reg);
                EXTI->RTSR1 &= ~(1 << nrf1_conf.pin_irq.reg);
                EXTI->FTSR1 |= (1 << nrf1_conf.pin_irq.reg);
                NVIC_SetPriority(EXTI15_10_IRQn, 40); // FIXME prio
                NVIC_EnableIRQ(EXTI15_10_IRQn);
        }

	// nrf2 irq pin: C2
        {
                auto const cr = &SYSCFG->EXTICR[nrf2_conf.pin_irq.reg / 4];
                *cr = (*cr & ~SYSCFG_EXTICR1_EXTI2_Msk) | (/*PC2*/0b010 << SYSCFG_EXTICR1_EXTI2_Pos);
                EXTI->IMR1 |= (1 << nrf2_conf.pin_irq.reg);
                EXTI->RTSR1 &= ~(1 << nrf2_conf.pin_irq.reg);
                EXTI->FTSR1 |= (1 << nrf2_conf.pin_irq.reg);
                NVIC_SetPriority(EXTI2_IRQn, 13);
                NVIC_EnableIRQ(EXTI2_IRQn);
        }

#endif
}


extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI10_15()
{
	if (EXTI->PR1 & (1 << nrf1_conf.pin_irq.reg)) {
		EXTI->PR1 = (1 << nrf1_conf.pin_irq.reg);
		BaseType_t yield = pdFALSE;
		xTaskNotifyFromISR(nrf1_task_data.task_handle, nrf_task_nf::irq, eSetBits, &yield);
		portYIELD_FROM_ISR(yield);
	}
}


extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI2()
{
	if (EXTI->PR1 & (1 << nrf2_conf.pin_irq.reg)) {
		EXTI->PR1 = (1 << nrf2_conf.pin_irq.reg);
		BaseType_t yield = pdFALSE;
		xTaskNotifyFromISR(nrf2_task_data.task_handle, nrf_task_nf::irq, eSetBits, &yield);
		portYIELD_FROM_ISR(yield);
	}
}


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<256> idle_task_stack;
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


auto g_pin_blue = freertos_utils::make_pin_toggle_task("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
auto g_pin_green = freertos_utils::make_pin_toggle_task("blink_green", bsp::pin_led_green, PRIO_BLINK);
auto g_pin_red = freertos_utils::make_pin_toggle_task("blink_red", bsp::pin_led_red, PRIO_BLINK);


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


freertos_utils::pinpoll::task_arg_t<stm32_lib::gpio::pin_t, 4> pinpoll_task_arg{
	freertos_utils::pinpoll::make_pin_info(bsp::pin_userbutton1, &pinpoll_cb1),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_userbutton2, &pinpoll_cb2),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_userbutton3, &pinpoll_cb3),
	freertos_utils::pinpoll::make_pin_info(stm32_lib::gpio::pin_t{GPIOC_BASE,13}, &pinpoll_cb_a)
};


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
	cf.spi->CR1 = 0;

	stm32_lib::spi::init_pin_nss(cf.pin_spi_nss);
	stm32_lib::spi::init_pins(
		cf.pin_spi_mosi, cf.spi_af,
		cf.pin_spi_miso, cf.spi_af,
		cf.pin_spi_sck, cf.spi_af
	);

	// CPOL=0 CPHA=0, Motorola mode
	constexpr uint32_t cr1 =
		(0b111 << SPI_CR1_BR_Pos)
		| SPI_CR1_MSTR_Msk
		| SPI_CR1_SSM_Msk
		;
	cf.spi->CR1 = cr1;
	cf.spi->CR2 =
		(0b0111 << SPI_CR2_DS_Pos)
		| SPI_CR2_FRXTH
		| SPI_CR2_SSOE;
	cf.spi->CR1 = cr1 | SPI_CR1_SPE;
}


void nrf_init_common(const nrf24::hw_conf_t& hwc)
{
        {
                auto cf = nrf24::reg_get(hwc, nrf24::reg_t::REG_CONFIG);
                cf =
                        ((cf & ~(
                                //nrf24::reg_t::REGV_CONFIG_EN_CRC
                                nrf24::reg_t::REGV_CONFIG_MASK_RX_DR
                                | nrf24::reg_t::REGV_CONFIG_MASK_TX_DS
                                | nrf24::reg_t::REGV_CONFIG_MASK_MAX_RT
                        ))
                                | nrf24::reg_t::REGV_CONFIG_EN_CRC
                        );
                nrf24::reg_set(hwc, nrf24::reg_t::REG_CONFIG, cf);
                vTaskDelay(1);
        }

	nrf24::reg_set(hwc, nrf24::reg_t::REG_SETUP_AW, nrf24::reg_t::REGV_SETUP_AW_5BYTES);
	vTaskDelay(1);

        {
                const auto sr = nrf24::reg_get(hwc, nrf24::reg_t::REG_SETUP_RETR);
                nrf24::reg_set(hwc, nrf24::reg_t::REG_SETUP_RETR, sr & ~nrf24::reg_t::REGV_SETUP_RETR_ARC_Msk);
                vTaskDelay(1);
        }

	nrf24::reg_set(hwc, nrf24::reg_t::REG_EN_RXADDR, 0);
	vTaskDelay(1);

	nrf24::reg_set(hwc, nrf24::reg_t::REG_EN_AA, 0);
	vTaskDelay(1);

	nrf24::reg_set(hwc, nrf24::reg_t::REG_RX_PW_P0, 32),
	vTaskDelay(1);
	nrf24::reg_set(hwc, nrf24::reg_t::REG_RX_PW_P1, 32),
	vTaskDelay(1);

        {
                const auto rfs = nrf24::reg_get(hwc, nrf24::reg_t::REG_RF_SETUP);
                nrf24::reg_set(
                        hwc,
                        nrf24::reg_t::REG_RF_SETUP,
                        rfs & ~(nrf24::reg_t::REGV_RF_SETUP_DR_LOW | nrf24::reg_t::REGV_RF_SETUP_DR_HIGH) // 00 - 1M
                );
                vTaskDelay(1);
        }

	nrf24::reg_set(hwc, nrf24::reg_t::REG_RF_CH, 55);
	vTaskDelay(1);

	nrf24::cmd0(hwc, nrf24::reg_t::CMD_FLUSH_RX);
	vTaskDelay(1);
	nrf24::cmd0(hwc, nrf24::reg_t::CMD_FLUSH_TX);
	vTaskDelay(1);
}


void nrf_st_cf(const nrf24::hw_conf_t& hwc)
{
	const uint8_t st = nrf24::reg_get(hwc, nrf24::reg_t::REG_STATUS);
	const uint8_t cf = nrf24::reg_get(hwc, nrf24::reg_t::REG_CONFIG);

	static char buf_st[8+3];
	logger.log_async("NRF status\r\n");
	print_bits(st, buf_st);
	logger.log_async(buf_st);

	static char buf_cf[8+3];
	logger.log_async("NRF config\r\n");
	print_bits(cf, buf_cf);
	logger.log_async(buf_cf);

	static char buf_fifost[8+3];
	const uint8_t fs = nrf24::reg_get(hwc, nrf24::reg_t::REG_FIFO_STATUS);
	logger.log_async("NRF fifo_status\r\n");
	print_bits(fs, buf_fifost);
	logger.log_async(buf_fifost);

	vTaskDelay(configTICK_RATE_HZ/10);
}


const std::array<uint8_t, 5> addr0{0xE7,0xE8,0xE9,0xEA,0xEB};

void nrf1_task_function(void* arg)
{
	logger.log_async("NRF1 task started\r\n");
	const nrf24::hw_conf_t* const hwc = reinterpret_cast<const nrf24::hw_conf_t*>(arg);

	hwc->pin_irq.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::pu); // IRQ pin
	hwc->pin_ce.set_mode_output_lowspeed_pushpull();
	hwc->pin_ce.set_state(0); // standby-1

	spi_nrf_init(*hwc);
	vTaskDelay(configTICK_RATE_HZ/10);

        nrf_init_common(*hwc);
	logger.log_async("NRF1 init done\r\n");

	nrf24::reg_set(*hwc, nrf24::reg_t::REG_EN_RXADDR, 0b10); // TODO -> defs
	vTaskDelay(1);

        nrf24::spi_write(
                *hwc,
                addr0.size(),
                nrf24::reg_t::CMD_W_REGISTER | nrf24::reg_t::REG_RX_ADDR_P1,
                addr0.data(),
                nullptr
        );
        vTaskDelay(1);

	logger.log_async("NRF1 powering up\r\n");
	{
                // set RX mode, power up
		const auto cf = nrf24::reg_get(*hwc, nrf24::reg_t::REG_CONFIG);
		nrf24::reg_set(
                        *hwc,
                        nrf24::reg_t::REG_CONFIG,
                        cf | nrf24::reg_t::REGV_CONFIG_PWR_UP | nrf24::reg_t::REGV_CONFIG_PRIM_RX_PRX
                );
		vTaskDelay(configTICK_RATE_HZ/10);
	}

	stm32_lib::gpio::set_state(hwc->pin_ce, 1);
        vTaskDelay(configTICK_RATE_HZ/10);
	nrf_st_cf(*hwc);

	logger.log_async("NRF1 loop\r\n");
	for(;;) {
		logger.log_async("--- NRF1 start wait\r\n");
		uint32_t events = 0;
		if (xTaskNotifyWait(0, 0xffffffff, &events, configTICK_RATE_HZ * 1) == pdTRUE) {
			if (events & nrf_task_nf::irq) {
				logger.log_async("NRF1: IRQ\r\n");
			} else if (events) {
				logger.log_async("NRF1: unknown notification\r\n");
			}
		}
		logger.log_async("--- NRF1 wakeup\r\n");
		g_pin_blue.pulse_once(configTICK_RATE_HZ/16);
		nrf_st_cf(*hwc);
                {
                        const uint8_t st = nrf24::reg_get(*hwc, nrf24::reg_t::REG_STATUS);
                        if (st & nrf24::reg_t::REGV_STATUS_RX_DR) {
                                logger.log_async("NRF-1: RX_DR\r\n");
                                bool rx_fifo_empty = false;
                                while (!rx_fifo_empty) {
                                        std::array<uint8_t, 32> buf;
                                        const auto st1 = nrf24::spi_write(
                                                *hwc,
                                                buf.size(),
                                                nrf24::reg_t::CMD_R_RX_PAYLOAD,
                                                nullptr,
                                                buf.data()
                                        );
                                        nrf24::reg_set(
                                                *hwc,
                                                nrf24::reg_t::REG_STATUS,
                                                ~nrf24::reg_t::REGV_STATUS_RX_DR // write 1 to clear this bit
                                        );
                                        logger.log_async("*** NRF1 data\r\n");
                                        logger.log_async(reinterpret_cast<const char*>(buf.data()));
                                        vTaskDelay(configTICK_RATE_HZ/4);
                                        const uint8_t fs = nrf24::reg_get(*hwc, nrf24::reg_t::REG_FIFO_STATUS);
                                        if (fs & nrf24::reg_t::REGV_FIFO_STATUS_RX_EMPTY) {
                                                rx_fifo_empty = true;
                                                logger.log_async("^^^ NRF-1 RX_FIFO_EMPTY\r\n");
                                                nrf_st_cf(*hwc);
                                        }
                                }
                        } else {
                                logger.log_async("... NRF1 no data\r\n");
                        }
                }
		logger.log_async("--- goto wait\r\n");
	}
}


void create_nrf1_task(const char* task_name, UBaseType_t prio, nrf_task_data_t& task_data, const nrf24::hw_conf_t* const hwc)
{
	task_data.task_handle = xTaskCreateStatic(
		&nrf1_task_function,
		task_name,
		task_data.stack.size(),
		const_cast<void*>(reinterpret_cast<const void*>(hwc)),
		prio,
		task_data.stack.data(),
		&task_data.task_buffer
	);
}


void nrf2_task_function(void* arg)
{
	vTaskDelay(configTICK_RATE_HZ*5);
	logger.log_async("NRF2 task started\r\n");
	const nrf24::hw_conf_t* const hwc = reinterpret_cast<const nrf24::hw_conf_t*>(arg);

	hwc->pin_irq.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::pu); // IRQ pin
	hwc->pin_ce.set_mode_output_lowspeed_pushpull();
	hwc->pin_ce.set_state(0); // standby-1

	spi_nrf_init(*hwc);
	vTaskDelay(configTICK_RATE_HZ/10);

        nrf_init_common(*hwc);

	{
                // activate TX mode
		uint8_t cf = nrf24::reg_get(*hwc, nrf24::reg_t::REG_CONFIG);
		nrf24::reg_set(*hwc, nrf24::reg_t::REG_CONFIG, cf & ~nrf24::reg_t::REGV_CONFIG_PRIM_RX_PRX);
		vTaskDelay(1);
	}

        nrf24::spi_write(
                *hwc,
                addr0.size(),
                nrf24::reg_t::CMD_W_REGISTER | nrf24::reg_t::REG_TX_ADDR,
                addr0.data(),
                nullptr
        );
        vTaskDelay(1);

        {
                const auto rfs = nrf24::reg_get(*hwc, nrf24::reg_t::REG_RF_SETUP);
                nrf24::reg_set(
                        *hwc,
                        nrf24::reg_t::REG_RF_SETUP,
                        (rfs & ~nrf24::reg_t::REGV_RF_SETUP_RF_PWR_Msk) | nrf24::reg_t::REGV_RF_SETUP_RF_PWR_00
                );
                vTaskDelay(1);
        }

	logger.log_async("NRF2 powering up\r\n");
	{
		const auto cf = nrf24::reg_get(*hwc, nrf24::reg_t::REG_CONFIG);
		nrf24::reg_set(*hwc, nrf24::reg_t::REG_CONFIG, cf | nrf24::reg_t::REGV_CONFIG_PWR_UP);
		vTaskDelay(configTICK_RATE_HZ/10);
	}

	nrf_st_cf(*hwc);

	logger.log_async("NRF2 loop\r\n");
	for(;;) {
		logger.log_async("NRF-2: sending\r\n");
		const std::array<uint8_t, 32> tdata{'H','e','l','l','o','\r','\n',0};
		nrf24::spi_write(*hwc, tdata.size(), nrf24::reg_t::CMD_W_TX_PAYLOAD, tdata.data(), nullptr);

		// pulse CE
		stm32_lib::gpio::set_state(hwc->pin_ce, 1);
		vTaskDelay(10);
		stm32_lib::gpio::set_state(hwc->pin_ce, 0);
		vTaskDelay(configTICK_RATE_HZ/10);
                nrf_st_cf(*hwc);

		vTaskDelay(configTICK_RATE_HZ*10);
	}
}


void create_nrf2_task(const char* task_name, UBaseType_t prio, nrf_task_data_t& task_data, const nrf24::hw_conf_t* const hwc)
{
	task_data.task_handle = xTaskCreateStatic(
		&nrf2_task_function,
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
	g_pin_green.pulse_continuous(configTICK_RATE_HZ/20, configTICK_RATE_HZ/10);
	g_pin_red.pulse_once(configTICK_RATE_HZ/10);
	g_pin_blue.pulse_once(configTICK_RATE_HZ/10);

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

	logger.log_sync("Creating NRF-1 task...\r\n");
	create_nrf1_task("nrf1_task", PRIO_NRF1, nrf1_task_data, &nrf1_conf);
	logger.log_sync("Created NRF-1 task\r\n");

	logger.log_sync("Creating NRF-2 task...\r\n");
	create_nrf2_task("nrf2_task", PRIO_NRF2, nrf2_task_data, &nrf2_conf);
	logger.log_sync("Created NRF-2 task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

