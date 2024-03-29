#ifndef __logging_h_included__
#define __logging_h_included__

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "cmsis_device.h"

#include <array>


class usart_logger_t {
#ifdef TARGET_NRF52DK
	using uart_t = NRF_UART_Type;
#else
	using uart_t = USART_TypeDef;
#endif
	uart_t* usart_ = nullptr;

	// FreeRTOS queue
	std::array<const char*, 128> queue_storage_;
	StaticQueue_t queue_;
	QueueHandle_t queue_handle_ = nullptr;

	// FreeRTOS task
	TaskHandle_t task_handle_ = nullptr;
	std::array<StackType_t, 128> task_stack_;
	StaticTask_t task_buffer_;

	static void task_function(void*);

public:
	// initialization
	void set_usart(uart_t* usart) { usart_ = usart; }
	void init_queue();
	void create_task(const char* task_name, UBaseType_t prio);

	// API
	void log_sync(const char* str) const;
	// TODO check error, queue overflow?
	void log_async(const char* static_str) { xQueueSend(queue_handle_, &static_str, 0); }
	void log_async_from_isr(const char* static_str) { xQueueSendFromISR(queue_handle_, reinterpret_cast<void*>(&static_str), NULL); }
};


#endif

