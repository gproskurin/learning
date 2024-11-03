#ifndef _my_logger_fwd_h_included_
#define _my_logger_fwd_h_included_

#include "logging.h"

#if defined TARGET_STM32L072
using log_dev_t = stm32_lib::dma::dev_usart_dma_t<USART2_BASE, DMA1_Channel4_BASE>;
#elif defined TARGET_STM32WL55_CPU1
using log_dev_t = stm32_lib::dma::dev_usart_dmamux_t<LPUART1_BASE, DMAMUX1_Channel0_BASE>;
#endif
extern logging::logger_t<log_dev_t> logger;

#endif

