#ifndef __logging_h_included__
#define __logging_h_included__

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "cmsis_device.h"

#ifdef TARGET_STM32L072
#include <string.h> // strlen
#endif

#include <array>


class usart_logger_t {
#ifdef TARGET_NRF52DK
	using uart_t = NRF_UART_Type;
#else
	using uart_t = USART_TypeDef;
#endif
	uart_t* const usart_;

#ifdef TARGET_STM32L072
	DMA_Channel_TypeDef* const dma_channel_ = DMA1_Channel4;
#endif

	using queue_item_t = const char*;

	// FreeRTOS queue
	std::array<queue_item_t, 128> queue_storage_;
	StaticQueue_t queue_;
	QueueHandle_t const queue_handle_;

	// FreeRTOS task
	std::array<StackType_t, 128> task_stack_;
	StaticTask_t task_buffer_;
	public: // FIXME
	TaskHandle_t const task_handle_;
	private:

	static void task_function(void*);

public:
#ifdef TARGET_STM32L072
	enum events_t : uint32_t {
		dma_te = (1 << 0),
		dma_tc = (1 << 1),
	};
#endif

	usart_logger_t(uart_t* u, const char* task_name, UBaseType_t prio)
		: usart_(u)
		, queue_handle_(xQueueCreateStatic(
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

#ifdef TARGET_STM32L072
	void init_dma()
	{
		dma_channel_->CCR = 0;
		dma_channel_->CCR = dma_channel_ccr_;
		dma_channel_->CPAR = reinterpret_cast<uint32_t>(&usart_->TDR);
	}
#endif

	// API
	void log_sync(const char* str) const;

	// TODO check error, queue overflow?
	void log_async(const char* str)
	{
		xQueueSend(queue_handle_, &str, 0);
	}

	void log_async_from_isr(const char* str)
	{
		xQueueSendFromISR(queue_handle_, &str, nullptr);
	}

#ifdef TARGET_STM32L072
private:
	static constexpr uint32_t dma_channel_ccr_ = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TEIE | DMA_CCR_TCIE;

	void log_dma(const char* str)
	{
		static_assert(sizeof(str) == sizeof(uint32_t));
		dma_channel_->CMAR = reinterpret_cast<uint32_t>(str);
		dma_channel_->CNDTR = strlen(str);
		dma_channel_->CCR = dma_channel_ccr_ | DMA_CCR_EN;
	}
#endif
};


#endif

