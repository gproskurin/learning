#include "logging.h"

#include "task.h"
#include "queue.h"


namespace {

void usart_tx(USART_TypeDef* const usart, const char* s)
{
#if 0
	while (*s) {
#ifdef TARGET_STM32H7A3
		while (! (usart->ISR & USART_ISR_TXE_TXFNF)) {}
		usart->TDR = *s;
#elif defined TARGET_STM32L072 || defined TARGET_STM32L432
		while (! (usart->ISR & USART_ISR_TXE)) {}
		usart->TDR = *s;
#else
		while (! (usart->SR & USART_SR_TXE)) {}
		usart->DR = *s;
#endif
		++s;
	}
#endif
}

} // namespace


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


void usart_logger_t::log_sync(const char* str) const
{
	usart_tx(usart_, str);
}


void usart_logger_t::log_async(const char* static_str)
{
	xQueueSend(queue_handle_, &static_str, 0);
	// TODO check error, queue overflow?
}


void usart_logger_t::log_async_from_isr(const char* static_str)
{
	// copy pointer itself to queue
	xQueueSendFromISR(queue_handle_, reinterpret_cast<void*>(&static_str), NULL);
	// TODO check error, queue overflow?
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
