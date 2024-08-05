#ifndef __logging_h_included__
#define __logging_h_included__

#if defined(TARGET_NRF52DK) || defined(TARGET_NRF5340DK_APP)
#include "lib_nrf5.h"
#else
#include "lib_stm32.h"
#endif

#include "freertos_utils.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "cmsis_device.h"

#include <string.h> // strlen

#include <array>


namespace logging {


#if defined(TARGET_NRF52DK) || defined (TARGET_NRF5340DK_APP)
constexpr uint32_t max_dma_size = 255;

template <uint32_t UarteBase>
struct uarte_dev_t {
	static void init() {}
	static void start(size_t size, const uint8_t* data)
	{
		nrf5_lib::uart::impl::uarte_send_start(uarte(), data, size);
	}
	static void stop() {}
//private:
	static NRF_UARTE_Type* uarte() { return reinterpret_cast<NRF_UARTE_Type*>(UarteBase); }
};
#endif


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

#if defined(TARGET_NRF52DK) || defined(TARGET_NRF5340DK_APP)
	std::array<uint8_t, 64> dma_copy_;
#endif

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
	static void wait()
	{
#if defined(TARGET_NRF52DK) || defined(TARGET_NRF5340DK_APP)
		constexpr uint32_t wait_mask = 1; // FIXME
#else
		// wait for DMA to complete/error
		constexpr uint32_t wait_mask = stm32_lib::dma::dma_result_t::te | stm32_lib::dma::dma_result_t::tc;
#endif
		auto const _events = freertos_utils::notify_wait_any(wait_mask);
		// TODO handle TE
	}

	static void task_function(void* arg)
	{
		auto* const lp = reinterpret_cast<logger_t<dev_t>*>(arg);
		for (;;) {
#if defined(TARGET_NRF52DK) || defined(TARGET_NRF5340DK_APP)
			auto item = lp->queue_receive();
			// nrf can do DMA from RAM only
			if (nrf5_lib::can_dma(item)) {
				auto const len = strlen(item);
				if (len <= max_dma_size) {
					dev_t::start(len, reinterpret_cast<const uint8_t*>(item));
					wait();
					dev_t::stop();
					continue;
				}
			}

			// not RAM, copy to RAM and do DMA from copy (in chunks)
			for (;;) {
				auto const chunk_size = nrf5_lib::chunk0(
					lp->dma_copy_.data(),
					item,
					lp->dma_copy_.size()
				);
				if (chunk_size == 0) {
					break;
				}

				dev_t::start(chunk_size, lp->dma_copy_.data());

				static_assert(sizeof(*item) == 1);
				item += chunk_size;

				//vTaskDelay(configTICK_RATE_HZ);
				wait();

				// FIXME hack to wait until TX is "fully" completed before starting new transfer
				vTaskDelay(1);

				dev_t::stop();
			}
#else
			// stm32 can DMA from any type of memory
			auto const item = lp->queue_receive();
			dev_t::start(strlen(item), reinterpret_cast<const uint8_t*>(item));
			wait();
			dev_t::stop();
#endif
		}
	}

	queue_item_t queue_receive()
	{
		queue_item_t item;
		while (xQueueReceive(queue_handle_, reinterpret_cast<void*>(&item), portMAX_DELAY) != pdTRUE)
		{
		}
		return item;
	}
};


} // namespace logging

#endif

