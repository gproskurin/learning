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
logger: DMAMUX1 Channel0 -> DMA1 channel 1, LPUART1
dac: DMAMUX1 Channel1 -> DMA1 channel 2

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
	DMA1_CSELR->CSELR =
		(0b1001/*TIM6&DAC1*/ << DMA_CSELR_C2S_Pos)
		| (0b0100/*USART2_TX*/ << DMA_CSELR_C4S_Pos)
		;

	// dma ch4 / logger
	NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 11);
	NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

#elif defined TARGET_STM32WL55_CPU1
	// GPIOs
	RCC->AHB2ENR = RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN;
	toggle_bits_10(&RCC->AHB2RSTR, RCC_AHB2RSTR_GPIOARST | RCC_AHB2RSTR_GPIOBRST);

	// DMA
	RCC->AHB1ENR = RCC_AHB1ENR_DMAMUX1EN | RCC_AHB1ENR_DMA1EN;
	toggle_bits_10(&RCC->AHB1RSTR, RCC_AHB1RSTR_DMA1RST_Msk | RCC_AHB1RSTR_DMAMUX1RST_Msk);

	// DAC & TIM2
	RCC->APB1ENR1 = RCC_APB1ENR1_DACEN | RCC_APB1ENR1_TIM2EN;
	toggle_bits_10(&RCC->AHB1RSTR, RCC_APB1RSTR1_DACRST_Msk | RCC_APB1RSTR1_TIM2RST_Msk);

	// LPUART1
	RCC->APB1ENR2 = RCC_APB1ENR2_LPUART1EN;
	toggle_bits_10(&RCC->APB1RSTR2, RCC_APB1RSTR2_LPUART1RST_Msk);
	RCC->CCIPR = (RCC->CCIPR & ~(RCC_CCIPR_LPUART1SEL_Msk))
		| (0b01 << RCC_CCIPR_LPUART1SEL_Pos)
		;

	// dma ch1 / logger
	NVIC_SetPriority(DMA1_Channel1_IRQn, 11);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
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
#ifdef TARGET_STM32WL55_CPU1
freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);
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
const std::array<beat_t, 67> music_menuet_g_minor{
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

const std::array<beat_t, 194> music_k545{
	// 1
	beat_t{l8, notes_t{note_t{C5, l2}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{E5, l4}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{G5, l4}, note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},

	// 2
	beat_t{l8, notes_t{note_t{B4, l4d}, note_t{D4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{F4, l8}}},
	beat_t{l16, notes_t{note_t{G4, l8}, note_t{C5, l16}}},
	beat_t{l16, notes_t{note_t{D5, l16}}},

	beat_t{l8, notes_t{note_t{C5, l4}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},

	// 3
	beat_t{l8, notes_t{note_t{A5, l2}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{F4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{G5, l4}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{C6, l4}, note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},

	// 4
	beat_t{l8, notes_t{note_t{G5,l4}, note_t{B3, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{F5,l8}, note_t{D4, l8}}},
	beat_t{l16, notes_t{note_t{E5,l16}, note_t{G4, l8}}},
	beat_t{l16, notes_t{note_t{F5, l16}}},
	beat_t{l8, notes_t{note_t{E5,l4}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},

	// 5
	beat_t{l8, notes_t{note_t{A4,l8}, note_t{F4, l4}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},

	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},

	beat_t{l16, notes_t{note_t{A5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},

	beat_t{l16, notes_t{note_t{D5,l16}, note_t{C4,l4s}, note_t{F3,l4s}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},

	// 6
	beat_t{l8, notes_t{note_t{G4,l8}, note_t{C4,l4s}, note_t{E3,l4s}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},

	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},

	beat_t{l16, notes_t{note_t{G5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},

	beat_t{l16, notes_t{note_t{C5,l16}, note_t{C4,l4s}, note_t{E3,l4s}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},

	// 7
	beat_t{l8, notes_t{note_t{F4,l8}, note_t{C4,l4s}, note_t{D3,l4s}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},

	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},

	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},

	beat_t{l16, notes_t{note_t{B4,l16}, note_t{B3,l4s}, note_t{D3,l4s}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},
	beat_t{l16, notes_t{note_t{F4,l16}}},

	// 8
	beat_t{l8, notes_t{note_t{E4,l8}, note_t{C4,l4s}, note_t{C3,l4s}}},
	beat_t{l16, notes_t{note_t{F4,l16}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},

	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},

	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},

	beat_t{l16, notes_t{note_t{A4,l16}, note_t{C3,l4s}, note_t{E3,l4s}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},
	beat_t{l16, notes_t{note_t{F4,l16}}},
	beat_t{l16, notes_t{note_t{E4,l16}}},

	// 9
	beat_t{l8, notes_t{note_t{D4,l8}, note_t{F3,l1}, note_t{A3,l1}}},
	beat_t{l16, notes_t{note_t{E4,l16}}},
	beat_t{l16, notes_t{note_t{F4,l16}}},

	beat_t{l16, notes_t{note_t{G4,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5s,l16}}},

	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},

	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},

	// 10
	beat_t{l16, notes_t{note_t{A5,l16}, note_t{F3,l4d}}},
	beat_t{l16, notes_t{note_t{B5,l16}}},
	beat_t{l16, notes_t{note_t{C6,l16}}},
	beat_t{l16, notes_t{note_t{B5,l16}}},

	beat_t{l16, notes_t{note_t{A5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}, note_t{G3,l8}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},

	beat_t{l16, notes_t{note_t{F5,l16}, note_t{A3,l4d}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},
	beat_t{l16, notes_t{note_t{A5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},

	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}, note_t{F3s,l8}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},

	// 11
	beat_t{l16, notes_t{note_t{G2,l16}, note_t{B4,l4}}},
	beat_t{l16, notes_t{note_t{B2,l16}}},
	beat_t{l16, notes_t{note_t{D3,l16}, note_t{G5,l4}}},
	beat_t{l16, notes_t{note_t{G3,l16}}},

	beat_t{l16, notes_t{note_t{G2,l16}, note_t{E5,l4s}}},
	beat_t{l16, notes_t{note_t{C3,l16}}},
	beat_t{l16, notes_t{note_t{E3,l16}, note_t{C5,l4s}}},
	beat_t{l16, notes_t{note_t{G3,l16}}},

	beat_t{l16, notes_t{note_t{G2,l16}, note_t{D5,l4}}},
	beat_t{l16, notes_t{note_t{B2,l16}}},
	beat_t{l16, notes_t{note_t{D3,l16}, note_t{G5,l4}}},
	beat_t{l16, notes_t{note_t{G3,l16}}},

	beat_t{l16, notes_t{note_t{G2,l16}, note_t{E5,l4s}}},
	beat_t{l16, notes_t{note_t{C3,l16}}},
	beat_t{l16, notes_t{note_t{E3,l16}, note_t{C5,l4s}}},
	beat_t{l16, notes_t{note_t{G3,l16}}},

	// 12
	beat_t{l4, notes_t{note_t{D5,l4s}, note_t{G2,l4s}}},
	beat_t{l4, notes_t{note_t{G5,l4s}, note_t{D5,l4s}, note_t{B4,l4s}, note_t{G3,l4s}}},
	beat_t{l4, notes_t{note_t{G4,l4s}, note_t{G2,l4s}}},
	beat_t{l4, notes_t{}},

	// 13
	beat_t{l16, notes_t{note_t{C4s,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4s,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4s,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4s,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},

	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},

	// 14
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{D6,l8}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{B5,l8}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{G5,l4d}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{A5,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}, note_t{B5,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{A5,l8}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{G5,l8}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},

	// 15
	beat_t{l32, notes_t{note_t{C4,l16}, note_t{A5,l32}}},
	beat_t{l32, notes_t{note_t{G5,static_cast<duration_t>(static_cast<uint8_t>(l8d)-static_cast<uint8_t>(l32))}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}, note_t{F5s,l16s}}},
	beat_t{l16, notes_t{note_t{A3,l16}, note_t{F5s,l4}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{A3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
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

#if 0
	player::enqueue_note(notes::sym_t::A4, notes::duration_t::l8, notes::instrument_t::sin12);
	logger.log_async("Enqueued 1 note\r\n");
	vTaskDelay(configTICK_RATE_HZ*3);
	logger.log_async("Enqueueing next note...\r\n");
	player::enqueue_note(notes::sym_t::A4, notes::duration_t::l8, notes::instrument_t::sin12);
	logger.log_async("Enqueued next note\r\n");
#endif

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

		/*
		for (const auto& n : Am) {
			player::enqueue_note(n, notes::duration_t::l4, notes::instrument_t::sin12);
			vTaskDelay(configTICK_RATE_HZ/4);
		}
		vTaskDelay(configTICK_RATE_HZ*3);
		*/

		for (const auto& beat : music_k545) {
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


template <uint32_t IsrCheckTc, uint32_t IfcrClearTc, uint32_t IsrCheckTe, uint32_t IfcrClearTe>
inline
void int_handler_logger_impl(uint32_t isr)
{
	uint32_t ev_log = 0;
	if (isr & IsrCheckTc) {
		DMA1->IFCR = IfcrClearTc;
		ev_log |= stm32_lib::dma::dma_result_t::tc;
	}
	if (isr & IsrCheckTe) {
		DMA1->IFCR = IfcrClearTe;
		ev_log |= stm32_lib::dma::dma_result_t::te;
	}
	if (ev_log) {
		logger.notify_from_isr(ev_log);
	}
}

#if defined TARGET_STM32L072
extern "C" __attribute__ ((interrupt)) void IntHandler_DMA1_Channel4_5_6_7()
{
	int_handler_logger_impl<DMA_ISR_TCIF4, DMA_IFCR_CTCIF4, DMA_ISR_TEIF4, DMA_IFCR_CTEIF4>(DMA1->ISR);
}
#elif defined TARGET_STM32WL55_CPU1
extern "C" __attribute__ ((interrupt)) void IntHandler_Dma1Ch1()
{
	int_handler_logger_impl<DMA_ISR_TCIF1, DMA_IFCR_CTCIF1, DMA_ISR_TEIF1, DMA_IFCR_CTEIF1>(DMA1->ISR);
}
#endif


__attribute__ ((noreturn)) void main()
{
	bus_init();
#ifdef TARGET_STM32L072
	stm32_lib::usart::init_logger_uart<configCPU_CLOCK_HZ>(
		USART_STLINK,
		bsp::usart_stlink_pin_tx,
		USART_STLINK_PIN_TX_AF
	);
#endif
#ifdef TARGET_STM32WL55_CPU1
	stm32_lib::usart::init_logger_lpuart<configCPU_CLOCK_HZ>(
		USART_STLINK,
		bsp::usart_stlink_pin_tx,
		USART_STLINK_PIN_TX_AF
	);
#endif

	log_sync("\r\nLogger initialized (sync)\r\n");

#ifdef TARGET_STM32L072
	g_pin_green2.init_pin();
	g_pin_green2.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/25);
#endif
#ifdef TARGET_STM32WL55_CPU1
	g_pin_green.init_pin();
	g_pin_green.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/25);
#endif

	player::create_task("player", PRIO_PLAYER);
	create_task_sender();

	log_sync("Starting FreeRTOS scheduler\r\n");
	logger.init();
	vTaskStartScheduler();

	log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

