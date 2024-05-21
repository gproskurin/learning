#include "logging.h"


void usart_logger_t::task_function(void* arg)
{
	usart_logger_t* const lp = reinterpret_cast<usart_logger_t*>(arg);
	for (;;) {
		queue_item_t item;
		if (xQueueReceive(lp->queue_handle_, reinterpret_cast<void*>(&item), portMAX_DELAY) == pdTRUE) {
#ifdef TARGET_STM32L072
			lp->log_dma(item);

			// wait for DMA to complete/error
			uint32_t events = 0;
			while (
				(xTaskNotifyWait(0, 0xffffffff, &events, portMAX_DELAY) != pdTRUE)
				&& ((events & (dma_te | dma_tc)) == 0)
			)
			{}
			// TODO handle TE

			lp->dma_channel_->CCR = dma_channel_ccr_;
#else
			lp->log_sync(item);
#endif
		}
	}
}


void usart_logger_t::log_sync(const char* s) const
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

