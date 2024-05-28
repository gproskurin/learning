#ifndef __logging_h_included__
#define __logging_h_included__

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "cmsis_device.h"

#include <string.h> // strlen

#include <array>


namespace logging {


#ifdef TARGET_NRF52DK
using uart_t = NRF_UART_Type;
#else
using uart_t = USART_TypeDef;
#endif


struct base_dma_params_t {
	static constexpr uint32_t dma_channel_ccr = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TEIE | DMA_CCR_TCIE;
	static constexpr uint32_t dma_channel_ccr_en = dma_channel_ccr | DMA_CCR_EN;
};

#ifdef TARGET_STM32L072
#define LOGGING_DMA
struct dma_params_t : public base_dma_params_t {
	static DMA_Channel_TypeDef* dma_channel() { return DMA1_Channel4; }
	static void init_dma(uart_t* uart)
	{
		dma_channel()->CCR = 0;
		dma_channel()->CCR = dma_channel_ccr;
		dma_channel()->CPAR = reinterpret_cast<uint32_t>(&uart->TDR);
	}
	static constexpr uint32_t isr_flag_te = DMA_ISR_TEIF4;
	static constexpr uint32_t isr_flag_tc = DMA_ISR_TCIF4;
	static constexpr uint32_t ifcr_flags_clear = DMA_IFCR_CGIF4 | DMA_IFCR_CTEIF4 | DMA_IFCR_CTCIF4;
};
#elif defined TARGET_STM32WL55
#define LOGGING_DMA
struct dma_params_t : public base_dma_params_t {
	static DMA_Channel_TypeDef* dma_channel() { return DMA1_Channel1; }
	static DMAMUX_Channel_TypeDef* dmamux_channel() { return DMAMUX1_Channel0; }
	static void init_dma(uart_t* uart)
	{
		dma_channel()->CCR = 0;
		dma_channel()->CCR = dma_channel_ccr;
		dma_channel()->CPAR = reinterpret_cast<uint32_t>(&uart->TDR);
		dmamux_channel()->CCR = (22/*LPUART1_TX*/ << DMAMUX_CxCR_DMAREQ_ID_Pos);
	}
	static constexpr uint32_t isr_flag_te = DMA_ISR_TEIF1;
	static constexpr uint32_t isr_flag_tc = DMA_ISR_TCIF1;
	static constexpr uint32_t ifcr_flags_clear = DMA_IFCR_CGIF1 | DMA_IFCR_CTEIF1 | DMA_IFCR_CTCIF1;
};
#else
struct dma_params_t {};
#endif


namespace impl {


template <typename DmaParams>
class usart_logger_t {
	uart_t* const usart_;

	using queue_item_t = const char*;

	// FreeRTOS queue
	std::array<queue_item_t, 128> queue_storage_;
	StaticQueue_t queue_;
	QueueHandle_t const queue_handle_;

	// FreeRTOS task
	std::array<StackType_t, 128> task_stack_;
	StaticTask_t task_buffer_;
	TaskHandle_t const task_handle_;

	static void task_function(void*);

public:
#ifdef LOGGING_DMA
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

	// API
	void log_sync(const char* str) const;

	// TODO check error, queue overflow?
	void log_async(const char* str)
	{
		xQueueSend(queue_handle_, &str, 0);
	}

	void log_async_from_isr(const char* str, BaseType_t* const woken)
	{
		xQueueSendFromISR(queue_handle_, &str, woken);
	}

#ifdef LOGGING_DMA
	void init_dma()
	{
		DmaParams::init_dma(usart_);
	}

	void handle_dma_irq()
	{
		uint32_t events = 0;

		auto const isr = DMA1->ISR;
		if (isr & DmaParams::isr_flag_te) {
			events |= events_t::dma_te;
		}
		if (isr & DmaParams::isr_flag_tc) {
			events |= events_t::dma_tc;
		}

		if (events) {
			DMA1->IFCR = DmaParams::ifcr_flags_clear;
			xTaskNotifyFromISR(task_handle_, events, eSetBits, nullptr);
		}
	}

private:

	void log_dma(const char* str)
	{
		static_assert(sizeof(str) == sizeof(uint32_t));
		DmaParams::dma_channel()->CMAR = reinterpret_cast<uint32_t>(str);
		DmaParams::dma_channel()->CNDTR = strlen(str);
		DmaParams::dma_channel()->CCR = DmaParams::dma_channel_ccr_en;
	}
#endif
};


} // namespace impl


} // namespace logging

using usart_logger_t = logging::impl::usart_logger_t<logging::dma_params_t>;

#endif

