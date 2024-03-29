#include "logging.h"

#include "task.h"
#include "queue.h"


void usart_logger_t::task_function(void* arg)
{
	usart_logger_t* const lp = reinterpret_cast<usart_logger_t*>(arg);
	for (;;) {
		const char* buf; // receive one pointer
		if (xQueueReceive(lp->queue_handle_, reinterpret_cast<void*>(&buf), portMAX_DELAY) == pdTRUE) {
			lp->log_sync(buf);
		}
	}
}


void usart_logger_t::init_queue()
{
	queue_handle_ = xQueueCreateStatic(
		queue_storage_.size(),
		sizeof(queue_storage_[0]),
		reinterpret_cast<uint8_t*>(queue_storage_.data()),
		&queue_
	);
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
#if defined TARGET_STM32H745_CM4 || defined TARGET_STM32H745_CM7 || defined TARGET_STM32WB55 || defined TARGET_STM32G030 || defined TARGET_STM32G031
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


void usart_logger_t::create_task(const char* task_name, UBaseType_t prio)
{
	task_handle_ = xTaskCreateStatic(
		&task_function,
		task_name,
		task_stack_.size(),
		reinterpret_cast<void*>(this), // param
		prio,
		task_stack_.data(),
		&task_buffer_
	);
}

