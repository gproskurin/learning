#ifndef _my_logger_fwd_h_included_
#define _my_logger_fwd_h_included_

#include "logging.h"

using log_dev_t = stm32_lib::dma::dev_usart_dma_t<USART2_BASE, DMA1_Channel4_BASE>;
extern logging::logger_t<log_dev_t> logger;

#endif

