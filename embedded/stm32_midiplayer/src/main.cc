#include "player.h"
#include "logger_fwd.h"

#include "lib_stm32.h"
#include "bsp.h"
#include "freertos_utils.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>


#define PRIO_BLINK 1
#define PRIO_SENDER 2
#define PRIO_LOGGER 3
#define PRIO_PLAYER 4

/*
Resources:

stm32l072
logger: DMA1 Channel4, USART2
dac: DMA1 Channel2

stm32wl55
logger: DMAMUX1 Channel0, LPUART1
dac: DMMAUX1 Channel1

*/


logging::logger_t<log_dev_t> logger("logger", PRIO_LOGGER);

void log_sync(const char* s)
{
	stm32_lib::usart::send(USART_STLINK, s);
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
#elif defined TARGET_STM32WL55_CPU1
	// flash
	FLASH->ACR &= (FLASH_ACR_DCEN | FLASH_ACR_ICEN); // disable caches
	FLASH->ACR |= (FLASH_ACR_DCRST | FLASH_ACR_ICRST); // reset caches
	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY_Msk)
		| (0b010 << FLASH_ACR_LATENCY_Pos) // 2WS
		| (FLASH_ACR_DCEN | FLASH_ACR_ICEN) // enable caches
		| FLASH_ACR_PRFTEN
		;

	while (!(RCC->CR & RCC_CR_MSIRDY)) {}
	RCC->CR = (RCC->CR & ~RCC_CR_MSIRANGE_Msk)
		| RCC_CR_MSIRGSEL
		| (0b1011 << RCC_CR_MSIRANGE_Pos)
		;
#endif

#if defined TARGET_STM32L072
	// GPIOs
	RCC->IOPENR = RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_IOPARST | RCC_IOPRSTR_IOPBRST);

	// DAC1 & TIM6 & USART2
	RCC->APB1ENR |= RCC_APB1ENR_DACEN_Msk | RCC_APB1ENR_TIM6EN_Msk | RCC_APB1ENR_USART2EN_Msk;
	toggle_bits_10(
		&RCC->APB1RSTR,
		RCC_APB1RSTR_DACRST_Msk | RCC_APB1RSTR_TIM6RST_Msk | RCC_APB1RSTR_USART2RST_Msk
	);

	// DMA
	RCC->AHBENR |= RCC_AHBENR_DMAEN_Msk;
	toggle_bits_10(&RCC->AHBRSTR, RCC_AHBRSTR_DMARST_Msk);

	// AHB1: DMA1 & DMAMUX1
	RCC->AHB1ENR |= RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_DMA1EN /*| RCC_AHB1ENR_DMA2EN*/;
	toggle_bits_10(&RCC->AHB1RSTR, RCC_AHB1RSTR_DMA1RST_Msk /*| RCC_AHB1RSTR_DMA2RST_Msk*/ | RCC_AHB1RSTR_DMAMUX1RST_Msk);

	// AHB2: GPIOs
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN_Msk | RCC_AHB2ENR_GPIOBEN_Msk | RCC_AHB2ENR_GPIOCEN_Msk;
	toggle_bits_10(
		&RCC->AHB2RSTR,
		RCC_AHB2RSTR_GPIOARST_Msk | RCC_AHB2RSTR_GPIOBRST_Msk | RCC_AHB2RSTR_GPIOCRST_Msk
	);

	// AHB3
	//RCC->AHB3ENR |= RCC_AHB3ENR_FLASHEN;

	// APB1: LPUART1
	RCC->APB1ENR2 |= RCC_APB1ENR2_LPUART1EN;
	toggle_bits_10(&RCC->APB1RSTR2, RCC_APB1RSTR2_LPUART1RST_Msk);
	RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_LPUART1SEL_Msk))
		| (0b01/*SYSCLK*/ << RCC_CCIPR_LPUART1SEL_Pos)
		;


	NVIC_SetPriority(DMA1_Channel1_IRQn, 11);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#elif defined TARGET_STM32WL55_CPU1
	// GPIOs
	RCC->AHB2ENR = RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
	toggle_bits_10(&RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOARST | RCC_AHB2RSTR_GPIOBRST);
#endif
}


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<128> idle_task_stack;
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


#ifdef TARGET_STM32L072
freertos_utils::pin_toggle_task_t g_pin_green2("blink_green2", bsp::pin_led_green2, PRIO_BLINK);
#endif


struct note_t {
	notes::sym_t note;
	notes::duration_t duration;

	note_t() : duration(static_cast<notes::duration_t>(0)) {}
	note_t(notes::sym_t n, notes::duration_t d) : note(n), duration(d) {}
};

using notes_t = std::array<note_t, 5>;

struct beat_t {
	notes::duration_t duration;
	notes_t notes;

	beat_t(notes::duration_t d, const notes_t& nn) : duration(d), notes(nn) {}
};

using namespace notes;
const std::array<beat_t, 67> music{
	// 1
	beat_t{l4, notes_t{note_t{B5f, l4}, note_t{G3, l2d}}},
	beat_t{l4, notes_t{note_t{A5, l4}}},
	beat_t{l4, notes_t{note_t{G5, l4}}},

	// 2
	beat_t{l4, notes_t{note_t{A5, l4}, note_t{F3, l2d}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},

	// 3
	beat_t{l4, notes_t{note_t{G5, l4}, {E3f, l2d}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{B4f, l8}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 4
	beat_t{l4, notes_t{note_t{D3, l4}, note_t{D5, l2d}}},
	beat_t{l8, notes_t{note_t{D4, l8}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},
	beat_t{l8, notes_t{note_t{B3f, l8}}},
	beat_t{l8, notes_t{note_t{A3, l8}}},

	// 5
	beat_t{l4, notes_t{note_t{E5f, l4}, note_t{B3f, l2}, note_t{G3, l2}}},
	beat_t{l8, notes_t{note_t{F5, l8}}},
	beat_t{l8, notes_t{note_t{E5f, l8}}},
	beat_t{l8, notes_t{note_t{D5, l8}, note_t{A3, l4}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 6
	beat_t{l4, notes_t{note_t{D5,l4}, note_t{B3f, l2}}},
	beat_t{l8, notes_t{note_t{E5f,l8}}},
	beat_t{l8, notes_t{note_t{D5,l8}}},
	beat_t{l8, notes_t{note_t{C5,l8}, note_t{G3,l8}}},
	beat_t{l8, notes_t{note_t{B4f,l8}}},

	// 7
	beat_t{l4, notes_t{note_t{C5,l4}, note_t{A3,l4}}},
	beat_t{l8, notes_t{note_t{D5,l8}, note_t{F3s,l4}}},
	beat_t{l8, notes_t{note_t{C5,l8}}},
	beat_t{l8, notes_t{note_t{B4f,l8}, note_t{G3,l4}}},
	beat_t{l8, notes_t{note_t{C5,l8}}},

	// 8
	beat_t{l4, notes_t{note_t{A4,l2d}, note_t{D3, l4}}},
	beat_t{l8, notes_t{note_t{D4,l8}}},
	beat_t{l8, notes_t{note_t{C4,l8}}},
	beat_t{l8, notes_t{note_t{B3f,l8}}},
	beat_t{l8, notes_t{note_t{A3,l8}}},

	// 9
	beat_t{l4, notes_t{note_t{B5f, l4}, note_t{G3,l2d}}},
	beat_t{l4, notes_t{note_t{A5, l4}}},
	beat_t{l4, notes_t{note_t{G5, l4}}},

	// 10
	beat_t{l4, notes_t{note_t{A5, l4}, note_t{F3,l2d}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},

	// 11
	beat_t{l4, notes_t{note_t{G5, l4}, note_t{E3f,l2d}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{B4f, l8}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 12
	beat_t{l4, notes_t{note_t{D5, l2d}, note_t{D3,l4}}},
	beat_t{l8, notes_t{note_t{D4, l8}}},
	beat_t{l8, notes_t{note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{B4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},

	// 13
	beat_t{l4, notes_t{note_t{F5, l4}, note_t{D4,l2}, note_t{B4,l2}}},
	beat_t{l8, notes_t{note_t{G5, l8}}},
	beat_t{l8, notes_t{note_t{F5, l8}}},
	beat_t{l8, notes_t{note_t{E5f, l8}, note_t{G3,l4}}},
	beat_t{l8, notes_t{note_t{D5, l8}}},

	// 14
	beat_t{l4, notes_t{note_t{E5f, l4}, note_t{C4, l4}}},
	beat_t{l8, notes_t{note_t{F5, l8}, note_t{A3, l4}}},
	beat_t{l8, notes_t{note_t{E5f, l8}}},
	beat_t{l8, notes_t{note_t{D5, l8}, note_t{F3,l4}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 15
	beat_t{l4, notes_t{note_t{D5, l4}, note_t{B3f,l4}}},
	beat_t{l4, notes_t{note_t{G5, l4}, note_t{E3f,l4}}},
	beat_t{l4, notes_t{note_t{C5, l4}, note_t{F3,l4}, note_t{A3,l4}}},

	// 16
	beat_t{l4, notes_t{note_t{B4f, l2d}, note_t{F4, l2d}, note_t{D4, l2d}, note_t{B3f,l4}}},
	beat_t{l2, notes_t{note_t{B2f, l2}}}
};

const std::array<notes::sym_t, 8> Cmaj{C4, D4, E4, F4, G4, A4, B4, C5};
const std::array<notes::sym_t, 8> Am{A4, B4, C5, D5, E5, F5, G5, A5};

void str_cpy_3(char* dst, const char* s1, const char* s2, const char* s3)
{
	auto p = stpcpy(dst, s1);
	p = stpcpy(p, s2);
	stpcpy(p, s3);
}


struct sender_task_data_t {
	std::array<StackType_t, 128> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
} sender_task_data;

void sender_task_function(void*)
{
	vTaskDelay(configTICK_RATE_HZ);

	player::enqueue_note(notes::sym_t::A4, notes::duration_t::l8, notes::instrument_t::sin12);
	logger.log_async("Enqueued 1 note\r\n");
	vTaskDelay(configTICK_RATE_HZ*3);
	logger.log_async("Enqueueing next note...\r\n");
	player::enqueue_note(notes::sym_t::A4, notes::duration_t::l8, notes::instrument_t::sin12);
	logger.log_async("Enqueued next note\r\n");

	//for(;;) {
	//	vTaskDelay(configTICK_RATE_HZ);
	//}
#if 0
	for (;;) {
		player::enqueue_note(notes::sym_t::A4, notes::duration_t::l2, notes::instrument_t::sin12);
		vTaskDelay(configTICK_RATE_HZ);
		player::enqueue_note(notes::sym_t::B4, notes::duration_t::l2, notes::instrument_t::sin12);
		vTaskDelay(configTICK_RATE_HZ/2);
	}
#endif

	for (;;) {
#if 0
		for (const auto& n : Cmaj) {
			player::enqueue_note(n, notes::duration_t::l4, notes::instrument_t::sin12);
			vTaskDelay(configTICK_RATE_HZ/4);
		}
		vTaskDelay(configTICK_RATE_HZ);
#endif

		for (const auto& n : Am) {
			player::enqueue_note(n, notes::duration_t::l4, notes::instrument_t::sin12);
			vTaskDelay(configTICK_RATE_HZ/4);
		}
		vTaskDelay(configTICK_RATE_HZ*3);

		for (const auto& beat : music) {
			for (const auto& n : beat.notes) {
				if (n.duration) {
					player::enqueue_note(
						n.note,
						n.duration,
						notes::instrument_t::sin12
					);
				}
			}
			vTaskDelay(configTICK_RATE_HZ/32*beat.duration);
		}
		vTaskDelay(configTICK_RATE_HZ*2);
	}
}

void create_task_sender()
{
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


#if defined TARGET_STM32L072
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

#elif defined TARGET_STM32WL55_CPU1
extern "C" __attribute__ ((interrupt)) void IntHandler_Dma1Ch1()
{
	auto const isr = DMA1->ISR;
	uint32_t events = 0;
	if (isr & DMA_ISR_TCIF1) {
		DMA1->IFCR = DMA_IFCR_CTCIF1;
		events |= stm32_lib::dma::dma_result_t::tc;
	}
	if (isr & DMA_ISR_TEIF1) {
		DMA1->IFCR = DMA_IFCR_CTEIF1;
		events |= stm32_lib::dma::dma_result_t::te;
	}
	if (events) {
		logger.notify_from_isr(events);
	}
}
#endif


__attribute__ ((noreturn)) void main()
{
	bus_init();
	stm32_lib::usart::init_logger_uart<configCPU_CLOCK_HZ>(
		USART_STLINK,
		bsp::usart_stlink_pin_tx,
		USART_STLINK_PIN_TX_AF
	);

	log_sync("\r\nLogger initialized (sync)\r\n");

#ifdef TARGET_STM32L072
	log_sync("Initializing blink task pin...\r\n");
	g_pin_green2.init_pin();
	g_pin_green2.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/25);
	log_sync("Initialized blink task pin\r\n");
#endif

	log_sync("Creating player task...\r\n");
	player::create_task("player", PRIO_PLAYER);
	log_sync("Created player task\r\n");

	log_sync("Creating sender task...\r\n");
	create_task_sender();
	log_sync("Created sender task\r\n");

	log_sync("Starting FreeRTOS scheduler\r\n");
	logger.init();
	vTaskStartScheduler();

	log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

