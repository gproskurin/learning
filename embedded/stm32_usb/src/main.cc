#include "cmsis_device.h"

#include "logger_fwd.h"

#include "bsp.h"
#include "freertos_utils.h"
#include "lib_stm32.h"
#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>


#define PRIO_BLINK 1
#define PRIO_USB_PERIPH 3
//#define PRIO_USB_HOST 3
#define PRIO_LOGGER 2

//#define USBDEV_HOST USB1_OTG_HS

logging::logger_t<log_dev_t> logger("logger", PRIO_LOGGER);

void log_sync(const char* s)
{
	stm32_lib::usart::send(USART_STLINK, s);
}

char prn_halfbyte(uint8_t x)
{
	return ((x <= 9) ? '0' : ('A'-10)) + x;
}

char* printf_byte(uint8_t x, char* buf)
{
	buf[0] = prn_halfbyte(x >> 4);
	buf[1] = prn_halfbyte(x & 0b1111);
	return buf+2;
}

char* printf_mem(const uint8_t* const mem, char* buf, size_t n)
{
	for (size_t i=0; i<n; ++i) {
		buf = printf_byte(mem[i], buf);
		if (i != n-1)
			*buf++ = ' ';
	}
	return buf;
}

char* log_nl0(char* buf)
{
	*buf++ = '\r';
	*buf++ = '\n';
	*buf++ = 0;
	return buf;
}


freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);


static
void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


static
void init_periph()
{
#ifdef TARGET_STM32L072
	// flash
	FLASH->ACR |= FLASH_ACR_LATENCY;

	// clock: switch from MSI to HSI
	// turn on HSI & wait
	RCC->CR |= RCC_CR_HSION | RCC_CR_PLLON;
	{
		constexpr auto wait = RCC_CR_HSIRDY | RCC_CR_PLLRDY;
		while ((RCC->CR & wait) != wait) {}
	}
	// switch to HSI
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0b01 << RCC_CFGR_SW_Pos);
	// switch off MSI & wait
	RCC->CR &= ~RCC_CR_MSION;
	while (RCC->CR & RCC_CR_MSIRDY) {}

	// enable HSI48 for USB
	RCC->APB2ENR = RCC_APB2ENR_SYSCFGEN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_SYSCFGRST_Msk);
	RCC->CRRCR |= RCC_CRRCR_HSI48ON;
	while (!(RCC->CRRCR | RCC_CRRCR_HSI48RDY)) {}
	SYSCFG->CFGR3 = SYSCFG_CFGR3_ENREF_HSI48;
	RCC->CCIPR = RCC_CCIPR_HSI48SEL;

	// GPIOs
	RCC->IOPENR = RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk | RCC_IOPENR_IOPCEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_IOPARST | RCC_IOPRSTR_IOPBRST | RCC_IOPRSTR_IOPCRST);

	// USART2 & USB & CRS
	RCC->APB1ENR = RCC_APB1ENR_USART2EN_Msk | RCC_APB1ENR_USBEN_Msk | RCC_APB1ENR_CRSEN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_USART2RST_Msk | RCC_APB1RSTR_USBRST_Msk | RCC_APB1RSTR_CRSRST_Msk);

	// DMA
	RCC->AHBENR = RCC_AHBENR_DMAEN;
	toggle_bits_10(&RCC->AHBRSTR, RCC_AHBRSTR_DMARST_Msk);

	DMA1_CSELR->CSELR =
		(0b0100 << DMA_CSELR_C4S_Pos); // USART2_TX
	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 18);
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);
#endif

#ifdef TARGET_STM32H747_CM4
	RCC->AHB4ENR =
		RCC_AHB4ENR_GPIOAEN_Msk
		| RCC_AHB4ENR_GPIOBEN_Msk
		| RCC_AHB4ENR_GPIODEN_Msk
		| RCC_AHB4ENR_GPIOGEN_Msk
		| RCC_AHB4ENR_GPIOHEN_Msk
		| RCC_AHB4ENR_GPIOIEN_Msk
		| RCC_AHB4ENR_GPIOJEN_Msk
		| RCC_AHB4ENR_GPIOKEN_Msk
		;
	toggle_bits_10(
		&RCC->AHB4RSTR,
		RCC_AHB4RSTR_GPIOARST_Msk
			| RCC_AHB4RSTR_GPIOBRST_Msk
			| RCC_AHB4RSTR_GPIODRST_Msk
			| RCC_AHB4RSTR_GPIOGRST_Msk
			| RCC_AHB4RSTR_GPIOHRST_Msk
			| RCC_AHB4RSTR_GPIOIRST_Msk
			| RCC_AHB4RSTR_GPIOJRST_Msk
			| RCC_AHB4RSTR_GPIOKRST_Msk
	);

	// USART
	RCC->APB1LENR = RCC_APB1LENR_USART3EN_Msk;
	toggle_bits_10(&RCC->APB1LRSTR, RCC_APB1LRSTR_USART3RST_Msk);

	// DMA & USB
	RCC->AHB1ENR = RCC_AHB1ENR_DMA1EN | RCC_AHB1ENR_USB1OTGHSEN;
	toggle_bits_10(&RCC->AHB1RSTR, RCC_AHB1RSTR_DMA1RST | RCC_AHB1RSTR_USB1OTGHSRST);

	// DMA irq
	NVIC_SetPriority(DMA1_Stream1_IRQn, 12);
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);
#endif
}

//#ifdef TARGET_STM32H747_CM7
template <typename Pin>
void init_pin_usb(const Pin& pin)
{
	#if 1
	pin.set(
		stm32_lib::gpio::mode_t::input,
		stm32_lib::gpio::speed_t::bits_11,
		stm32_lib::gpio::pupd_t::no_pupd
		/*
		stm32_lib::gpio::mode_t::af
		stm32_lib::gpio::af_t(USB_AF)
		*/
	);
	#endif
}
//#endif


struct usb_state_t {
	uint8_t usb_addr;
	bool usb_addr_set;
	uint8_t lastRequest;


	usb_state_t() { reset(); }
	void reset()
	{
		usb_addr = 0;
		usb_addr_set = false;
		lastRequest = 255;
	}
} usb_state;

template <uint16_t Size>
constexpr uint16_t calc_count_rx()
{
	static_assert((Size & 1) == 0);
	if (Size < 64) {
		return (Size / 2) << 10;
	}
	static_assert((Size & 31) == 0);
	return (1 << 15) | (((Size / 32) - 1) << 10);
}


struct usb_btable_entry_t {
	uint16_t addr_tx;
	uint16_t count_tx;
	uint16_t addr_rx;
	uint16_t count_rx;
};

// TODO packed, alignment
struct usb_pma_t {
	std::array<usb_btable_entry_t, 1> btable;
	std::array<char, 64> ep0_tx;
	std::array<char, 64> ep0_rx;

	usb_pma_t()
	{
		// ep0
		btable[0].addr_rx = offsetof(usb_pma_t, ep0_rx);
		constexpr auto c = calc_count_rx<64>();
		static_assert(c == 0b1000010000000000);
		btable[0].count_rx = c;

		// ep1 / keyboard
		//usb_pma->btable[1].addr_tx = offsetof(usb_pma_t, ep1_tx);
		//usb_pma->btable[1].count_tx = usb_pma->ep1_tx.size();
	}
};
constexpr size_t usb_pma_size = 1024;
static_assert(sizeof(usb_pma_t) <= usb_pma_size);

// 8-byte aligned
static_assert((offsetof(usb_pma_t, btable) & 0b111) == 0);
// 2-byte aligned
static_assert((offsetof(usb_pma_t, ep0_rx) & 0b1) == 0);
static_assert((offsetof(usb_pma_t, ep0_tx) & 0b1) == 0);

usb_pma_t* const usb_pma = reinterpret_cast<usb_pma_t*>(USB_PMAADDR);


void usb_set_ep(volatile uint16_t *ep, uint16_t value, uint16_t mask)
{
	constexpr uint16_t toggle = 0b0111000001110000;
	constexpr uint16_t rc_w0 = 0b1000000010000000;
	constexpr uint16_t rw = 0b0000011100001111;
	auto const v = *ep;

	auto const wr0 = rc_w0 & (~mask | value);
	auto const wr1 = (mask & toggle) & (v ^ value);
	auto const wr2 = rw & ((v & ~mask) | value);

	*ep = wr0 | wr1 | wr2;
}

void usb_set_ep_tx(volatile uint16_t *ep, uint16_t value)
{
	usb_set_ep(ep, value, USB_EP_TX_VALID /* mask */);
}

void usb_set_ep_rx(volatile uint16_t *ep, uint16_t value)
{
	usb_set_ep(ep, value, USB_EP_RX_VALID /* mask */);
}


// run from interrupt context
inline
void usb_reset()
{
	usb_state.reset();
	memset(usb_pma, 0, usb_pma_size);
	USB->BTABLE = offsetof(usb_pma_t, btable);

	new(usb_pma) usb_pma_t(); // call ctor


	USB->EP0R = USB_EP_CONTROL | /* USB_EP_CTR_TX | USB_EP_CTR_RX |*/ USB_EP_RX_VALID | USB_EP_TX_NAK | 0 /*EA*/;
	//USB->EP1R = USB_EP_INTERRUPT | USB_EP_CTR_TX | USB_EP_CTR_RX | USB_EP_RX_NAK | USB_EP_TX_NAK | 1 /*EA*/;

	USB->BCDR = USB_BCDR_DPPU;

	// enable
	USB->DADDR = USB_DADDR_EF;
}


void init_usb()
{
	init_pin_usb(bsp::pin_usb_dm);
	init_pin_usb(bsp::pin_usb_dp);

	constexpr uint16_t cntr =
		USB_CNTR_CTRM
		| USB_CNTR_PMAOVRM
		| USB_CNTR_ERRM
		| USB_CNTR_WKUPM
		| USB_CNTR_SUSPM
		| USB_CNTR_RESETM
		//| USB_CNTR_SOFM
		//| USB_CNTR_ESOFM
		| USB_CNTR_L1REQM
		| USB_CNTR_L1RESUME
		;

	logger.log_async("USB-PERIPH: initializing USB...\r\n");
#ifdef TARGET_STM32L072
	USB->CNTR = USB_CNTR_FRES; // clear PWDN bit
	vTaskDelay(1);
	USB->ISTR = 0;

	NVIC_SetPriority(USB_IRQn, 38); // TODO: configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY ?
	NVIC_EnableIRQ(USB_IRQn);

	memset(usb_pma, 0, usb_pma_size);
	USB->BTABLE = offsetof(usb_pma_t, btable);

	// clear FRES bit, "usb reset" interrupt will be triggered to complete initialization
	USB->CNTR = cntr;
#endif

#ifdef TARGET_STM32H747_CM7
	NVIC_SetPriority(OTG_HS_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	//USBDEV_HOST->
#endif
	logger.log_async("USB-PERIPH: initialized USB\r\n");
}


#ifdef TARGET_STM32L072
extern "C" __attribute__ ((interrupt)) void IntHandler_DMA1_Channel4_5_6_7()
{
	auto const isr = DMA1->ISR;

	{
		uint32_t ev_log = 0;
		if (isr & DMA_ISR_TCIF4) {
			DMA1->IFCR = DMA_IFCR_CTCIF4;
			ev_log |= stm32_lib::dma::dma_result_t::tc;
		}
		if (isr & DMA_ISR_TEIF4) {
			DMA1->IFCR = DMA_IFCR_CTEIF4;
			ev_log |= stm32_lib::dma::dma_result_t::te;
		}
		if (ev_log) {
			logger.notify_from_isr(ev_log);
		}
	}
}
#endif

uint8_t usb_setaddr = 0;
#ifdef TARGET_STM32H747_CM4
extern "C" __attribute__ ((interrupt)) void IntHandler_Dma1S1()
{
	auto const isr = DMA1->LISR;
	uint32_t events = 0;
	if (isr & DMA_LISR_TCIF1) {
		DMA1->LIFCR = DMA_LIFCR_CTCIF1;
		events |= stm32_lib::dma::dma_result_t::tc;
	}
	if (isr & DMA_LISR_TEIF1) {
		DMA1->LIFCR = DMA_LIFCR_CTEIF1;
		events |= stm32_lib::dma::dma_result_t::te;
	}
	if (isr & DMA_LISR_DMEIF1) {
		DMA1->LIFCR = DMA_LIFCR_CDMEIF1;
		events |= stm32_lib::dma::dma_result_t::te;
	}
	if (events) {
		logger.notify_from_isr(events);
	}
}
#endif


extern "C" __attribute__ ((interrupt)) void IntHandler_USB()
{
	auto const istr = USB->ISTR;
	logger.log_async_from_isr("IRQ-USB: begin\r\n");

	if ((istr & USB_ISTR_EP_ID)!=0) {
		logger.log_async_from_isr("IRQ-USB: ep!=0\r\n");
	}

	if (!usb_state.usb_addr_set && usb_state.usb_addr!=0/*&& usb_state.lastRequest==5*/ /* && (USB->EP0R & USB_EP_CTR_TX)*/) {
		logger.log_async_from_isr("IRQ-USB: time to set addr\r\n");
		USB->DADDR = USB_DADDR_EF | usb_state.usb_addr;
		usb_state.usb_addr_set = true;
		//usb_set_ep_rx(&USB->EP0R, USB_EP_RX_VALID);
		return;
	}

	if (
		(istr & USB_ISTR_EP_ID) == 0
		&& (istr & USB_ISTR_DIR /*rx*/)
		&& (istr & USB_ISTR_CTR)
		&& (
			(USB->EP0R & (USB_EP_SETUP | USB_EP_CTR_RX))
			== (USB_EP_SETUP | USB_EP_CTR_RX)
		)
		//&& (usb_pma->btable[0].count_rx == 8)
	) {
		logger.log_async_from_isr("IRQ-USB: EP0 setup\r\n");
		if (1) {
			logger.log_async_from_isr("IRQ-USB: size...\r\n");
			static char buf[16];
			uint16_t const sz = usb_pma->btable[0].count_rx;
			auto b = printf_mem((const uint8_t*)&sz, buf, 2);
			log_nl0(b);
			logger.log_async_from_isr(buf);
		}

		if (1) {
			logger.log_async_from_isr("IRQ-USB: data[64]...\r\n");
			constexpr size_t sz = 64;
			static char buf[sz*3 + 2];
			auto b = printf_mem((const uint8_t*)usb_pma->ep0_rx.data(), buf, sz);
			log_nl0(b);
			logger.log_async_from_isr(buf);
		}

		auto const rxbuf = usb_pma->ep0_rx.data();
		auto const bmRequestType = rxbuf[0];
		auto const bRequest = rxbuf[1];
		usb_state.lastRequest = bRequest;

		if (bmRequestType==0 && bRequest==5) {
			// set_address
			if (usb_state.usb_addr_set) {
				logger.log_async_from_isr("IRQ-USB: EP0 setup -> SET_ADDR: error: already set\r\n");
				usb_pma->btable[0].count_tx = 0;
				usb_set_ep_tx(&USB->EP0R, USB_EP_TX_STALL);
			} else if (rxbuf[3] != 0 || (rxbuf[2] & ~USB_DADDR_ADD) != 0) {
				logger.log_async_from_isr("IRQ-USB: EP0 setup -> SET_ADDR: error: invalid addr from host\r\n");
				usb_pma->btable[0].count_tx = 0;
				usb_set_ep_tx(&USB->EP0R, USB_EP_TX_STALL);
			} else {
				logger.log_async_from_isr("IRQ-USB: EP0 setup -> SET_ADDR: scheduling\r\n");
				usb_state.usb_addr = rxbuf[2];
				usb_pma->btable[0].count_tx = 0;
				usb_set_ep_tx(&USB->EP0R, USB_EP_TX_VALID);
			}
			//usb_set_ep(&USB->EP0R, 0, USB_EP_CTR_RX); // clear CTR_RX
			//usb_pma->btable[0].count_rx = calc_count_rx<64>();
		} else {
			logger.log_async_from_isr("IRQ-USB: EP0 setup -> UNSUPPORTED\r\n");
		}

	}
	if (istr & USB_ISTR_ERR) {
		USB->ISTR = ~USB_ISTR_ERR;
		logger.log_async_from_isr("IRQ-USB: ERR\r\n");
	}
	if (istr & USB_ISTR_CTR) {
		logger.log_async_from_isr("IRQ-USB: CTR\r\n");
	}
	if (istr & USB_ISTR_PMAOVR) {
		USB->ISTR = ~USB_ISTR_PMAOVR;
		logger.log_async_from_isr("IRQ-USB: PMAOVR\r\n");
	}
	if (istr & USB_ISTR_WKUP) {
		USB->ISTR = ~USB_ISTR_WKUP;
		USB->CNTR &= ~(USB_CNTR_FSUSP | USB_CNTR_LPMODE);
		logger.log_async_from_isr("IRQ-USB: WKUP\r\n");
	}
	if (istr & USB_ISTR_SUSP) {
		USB->ISTR = ~USB_ISTR_SUSP;
		logger.log_async_from_isr("IRQ-USB: SUSP\r\n");
		USB->CNTR |= USB_CNTR_FSUSP;
	}
	//if (istr & USB_ISTR_SOF) {
	//	USB->ISTR = ~USB_ISTR_SOF;
	//	logger.log_async_from_isr("IRQ-USB: SOF\r\n");
	//}
	//if (istr & USB_ISTR_ESOF) {
	//	USB->ISTR = ~USB_ISTR_ESOF;
	//	logger.log_async_from_isr("IRQ-USB: ESOF\r\n");
	//}
	if (istr & USB_ISTR_L1REQ) {
		USB->ISTR = ~USB_ISTR_L1REQ;
		logger.log_async_from_isr("IRQ-USB: L1REQ\r\n");
	}
	if (istr & USB_ISTR_RESET) {
		USB->ISTR = ~USB_ISTR_RESET;
		logger.log_async_from_isr("IRQ-USB: RESET\r\n");
		usb_reset();
	}
}


freertos_utils::task_data_t<128> task_data_usb_periph;

void task_function_usb_periph(void*)
{
	logger.log_async("USB-PERIPH: task started\r\n");
	init_usb();
	vTaskDelay(configTICK_RATE_HZ);

	#if 0
	if (0) {
		// send some data via ep1/tx
		uint16_t* const p = reinterpret_cast<uint16_t*>(usb_pma->ep1_tx.data());
		*p = 0xAA55;
		usb_pma->btable[1].count_tx = 2;

		usb_set_ep_tx(&USB->EP1R, USB_EP_TX_VALID);
	}
	#endif

	for (;;) {
		vTaskDelay(configTICK_RATE_HZ * 5);
		logger.log_async("USB-periph task keep-alive\r\n");
	}
}


#ifdef TARGET_STM32H747_CM4
freertos_utils::task_data_t<128> task_data_usb_host;

void task_function_usb_host(void*)
{
	logger.log_async("USB-host task started\r\n");

	for (;;) {
		vTaskDelay(configTICK_RATE_HZ * 10);
		logger.log_async("USB-host task keep-alive\r\n");
	}
}
#endif

void create_task_usb_periph()
{
	task_data_usb_periph.task_handle = xTaskCreateStatic(
		&task_function_usb_periph,
		"usb_periph",
		task_data_usb_periph.stack.size(),
		nullptr,
		PRIO_USB_PERIPH,
		task_data_usb_periph.stack.data(),
		&task_data_usb_periph.task_buffer
	);
}


#ifdef TARGET_STM32H747_CM4
void create_task_usb_host()
{
	task_data_usb_host.task_handle = xTaskCreateStatic(
		&task_function_usb_host,
		"usb_host",
		task_data_usb_host.stack.size(),
		nullptr,
		PRIO_USB_HOST,
		task_data_usb_host.stack.data(),
		&task_data_usb_host.task_buffer
	);
}
#endif


__attribute__ ((noreturn)) void main()
{
	init_periph();
	stm32_lib::usart::init_logger_uart<configCPU_CLOCK_HZ>(
		USART_STLINK,
		bsp::usart_stlink_pin_tx,
		USART_STLINK_PIN_TX_AF
	);

	log_sync("\r\nLogger initialized (sync)\r\n");

	g_pin_green.init_pin();
	//g_pin_red.init_pin();

	g_pin_green.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/50*4);
	g_pin_red.pulse_continuous(configTICK_RATE_HZ/2, configTICK_RATE_HZ/2);

	create_task_usb_periph();
#ifdef TARGET_STM32H747_CM4
	create_task_usb_host();
#endif

	log_sync("Starting FreeRTOS scheduler\r\n");
	logger.init();
	vTaskStartScheduler();

	for (;;) {}
}

