#include "logging.h"

#include "task.h"
#include "queue.h"


namespace {

void usart_tx(USART_TypeDef* const usart, const char* s)
{
	while (*s) {
		while (! (usart->SR & USART_SR_TXE)) {
		}
		usart->DR = *s;
		++s;
	}
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
		queue_len_,
		sizeof(queue_storage_[0]),
		reinterpret_cast<uint8_t*>(queue_storage_),
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
		task_stack_len_,
		reinterpret_cast<void*>(this), // param
		prio,
		task_stack_,
		&task_buffer_
	);
}

