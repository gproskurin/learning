#ifndef __logging_h_included__
#define __logging_h_included__

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "cmsis_device.h"


class usart_logger_t {
	USART_TypeDef* usart_ = nullptr;

	// FreeRTOS queue
	static constexpr size_t queue_len_ = 128;
	const char *queue_storage_[queue_len_];
	StaticQueue_t queue_;
	QueueHandle_t queue_handle_ = nullptr;

	// FreeRTOS task
	TaskHandle_t task_handle_ = nullptr;
	static constexpr size_t task_stack_len_ = 128;
	StackType_t task_stack_[task_stack_len_];
	StaticTask_t task_buffer_;

	static void task_function(void*);

public:
	// initialization
	void set_usart(USART_TypeDef* usart) { usart_ = usart; }
	void init_queue();
	void create_task(const char* task_name, UBaseType_t prio);

	// API
	void log_sync(const char* str) const;
	void log_async(const char* static_str);
	void log_async_from_isr(const char* static_str);
};


#endif

