#include "logging.h"


namespace logging {
namespace impl {


template <typename DmaParams>
void usart_logger_t<DmaParams>::task_function(void* arg)
{
	auto* const lp = reinterpret_cast<usart_logger_t*>(arg);
	for (;;) {
		queue_item_t item;
		if (xQueueReceive(lp->queue_handle_, reinterpret_cast<void*>(&item), portMAX_DELAY) == pdTRUE) {
#ifdef LOGGING_DMA
			lp->log_dma(item);

			// wait for DMA to complete/error
			uint32_t events = 0;
			while (
				(xTaskNotifyWait(0, 0xffffffff, &events, portMAX_DELAY) != pdTRUE)
				|| ((events & (dma_te | dma_tc)) == 0)
			)
			{}
			// TODO handle TE

			DmaParams::dma_channel()->CCR = DmaParams::dma_channel_ccr;
#else
			lp->log_sync(item);
#endif
		}
	}
}
}

namespace impl {
template <typename DmaParams>
void usart_logger_t<DmaParams>::log_sync(const char* s) const
{
#ifdef TARGET_NRF52DK
	if (!*s) {
		return;
	}

	usart_->TASKS_STARTTX = 1;

	// tx_byte
	usart_->EVENTS_TXDRDY = 0;
	usart_->TXD = *s;
	++s;
#endif

	while (*s) {
#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7 || defined TARGET_STM32WB55 || defined TARGET_STM32WL55 || defined TARGET_STM32G030 || defined TARGET_STM32G031
		while (! (usart_->ISR & USART_ISR_TXE_TXFNF)) {}
		usart_->TDR = *s;
#elif defined TARGET_STM32L072
		while (! (usart_->ISR & USART_ISR_TXE)) {}
		usart_->TDR = *s;
#elif defined TARGET_NRF52DK
		while (! (usart_->EVENTS_TXDRDY)) {}
		usart_->EVENTS_TXDRDY = 0;
		usart_->TXD = *s;
#else
		while (! (usart_->SR & USART_SR_TXE)) {}
		usart_->DR = *s;
#endif
		++s;
	}

#ifdef TARGET_NRF52DK
	while (! (usart_->EVENTS_TXDRDY)) {}
	usart_->TASKS_STOPTX = 1;
#endif
}


} // namespace impl
} // namespace logging


template
void logging::impl::usart_logger_t<logging::dma_params_t>::task_function(void* arg);

template
void logging::impl::usart_logger_t<logging::dma_params_t>::log_sync(const char* s) const;

