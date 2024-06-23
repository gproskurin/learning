#ifndef __logging_h_included__
#define __logging_h_included__

#include "lib_stm32.h"
#include "freertos_utils.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "cmsis_device.h"

#include <string.h> // strlen

#include <array>


namespace logging {


template <typename dev_t>
class logger_t {
	using queue_item_t = const char*;

	// FreeRTOS queue
	std::array<queue_item_t, 128> queue_storage_;
	StaticQueue_t queue_;
	QueueHandle_t const queue_handle_;

	// FreeRTOS task
	std::array<StackType_t, 128> task_stack_;
	StaticTask_t task_buffer_;
	TaskHandle_t const task_handle_;

public:
	logger_t(const char* task_name, UBaseType_t prio)
		: queue_handle_(xQueueCreateStatic(
			queue_storage_.size(),
			sizeof(queue_storage_[0]),
			reinterpret_cast<uint8_t*>(queue_storage_.data()),
			&queue_
		))
		, task_handle_(xTaskCreateStatic(
			&task_function,
			task_name,
			task_stack_.size(),
			reinterpret_cast<void*>(this), // param
			prio,
			task_stack_.data(),
			&task_buffer_
		))
	{
	}

	// API

	// TODO check error, queue overflow?
	void log_async(const char* str)
	{
		xQueueSend(queue_handle_, &str, 0);
	}

	void log_async_from_isr(const char* str)
	{
		xQueueSendFromISR(queue_handle_, &str, nullptr);
	}

	void init()
	{
		dev_t::init();
	}

	void notify_from_isr(uint32_t events)
	{
		xTaskNotifyFromISR(task_handle_, events, eSetBits, nullptr);
	}

private:
	static void task_function(void* arg)
	{
		auto* const lp = reinterpret_cast<logger_t<dev_t>*>(arg);
		for (;;) {
			queue_item_t item;
			if (xQueueReceive(lp->queue_handle_, reinterpret_cast<void*>(&item), portMAX_DELAY) == pdTRUE) {
				auto const size = strlen(item);
				dev_t::start(size, reinterpret_cast<const uint8_t*>(item));

				// wait for DMA to complete/error
				constexpr uint32_t wait_mask = stm32_lib::dma::dma_result_t::te | stm32_lib::dma::dma_result_t::tc;
				auto const _events = freertos_utils::notify_wait_any(wait_mask);
				// TODO handle TE

				dev_t::stop();
			}
		}
	}
};


} // namespace logging

#endif

