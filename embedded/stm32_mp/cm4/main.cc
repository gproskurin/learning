#include "lib_stm32.h"
#include "bsp.h"
#include "logging.h"
#include "freertos_utils.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>


#define PRIO_BLINK 1
#define PRIO_PINPOLL 2
#define PRIO_TEST1 2
#define PRIO_LOGGER 3


using log_dev_t = stm32_lib::dma::dev_usart_dmamux_t<USART3_BASE, DMAMUX1_Channel1_BASE>;
logging::logger_t<log_dev_t> logger("logger_cm4", PRIO_LOGGER);

void log_sync(const char* s)
{
	stm32_lib::usart::send(USART_STLINK, s);
}


freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);


void pinpoll_cb_joysel(const stm32_lib::gpio::pin_t&, bool new_status)
{
	if (new_status) {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 1);
		logger.log_async("PINPOLL: joysel UP\r\n");
	} else {
		logger.log_async("PINPOLL: joysel DOWN\r\n");
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 2);
	}
}

void pinpoll_cb_joyup(const stm32_lib::gpio::pin_t&, bool new_status)
{
	if (new_status) {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 3);
		logger.log_async("PINPOLL: joyup UP\r\n");
	} else {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 4);
		logger.log_async("PINPOLL: joyup DOWN\r\n");
	}
}

void pinpoll_cb_joydown(const stm32_lib::gpio::pin_t&, bool new_status)
{
	if (new_status) {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 5);
		logger.log_async("PINPOLL: joydown UP\r\n");
	} else {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 6);
		logger.log_async("PINPOLL: joydown DOWN\r\n");
	}
}

void pinpoll_cb_joyleft(const stm32_lib::gpio::pin_t&, bool new_status)
{
	if (new_status) {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 7);
		logger.log_async("PINPOLL: joyleft UP\r\n");
	} else {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 8);
		logger.log_async("PINPOLL: joyleft DOWN\r\n");
	}
}

void pinpoll_cb_joyright(const stm32_lib::gpio::pin_t&, bool new_status)
{
	if (new_status) {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 9);
		logger.log_async("PINPOLL: joyright UP\r\n");
	} else {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/2, 10);
		logger.log_async("PINPOLL: joyright DOWN\r\n");
	}
}

void pinpoll_cb_btn(const stm32_lib::gpio::pin_t&, bool new_status)
{
	if (new_status) {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/3, configTICK_RATE_HZ/4, 2);
		logger.log_async("PINPOLL: button UP\r\n");
	} else {
		g_pin_blue.pulse_many(configTICK_RATE_HZ/3, configTICK_RATE_HZ/4, 1);
		logger.log_async("PINPOLL: button DOWN\r\n");
	}
}


freertos_utils::pinpoll::task_arg_t<stm32_lib::gpio::pin_t, 6> pinpoll_task_arg{
	freertos_utils::pinpoll::make_pin_info(bsp::pin_joy_sel, &pinpoll_cb_joysel),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_joy_left, &pinpoll_cb_joyleft),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_joy_right, &pinpoll_cb_joyright),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_joy_up, &pinpoll_cb_joyup),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_joy_down, &pinpoll_cb_joydown),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_btn, &pinpoll_cb_btn)
};


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<128> idle_task_stack;
extern "C"
void vApplicationGetIdleTaskMemory(StaticTask_t **tcbIdle, StackType_t **stackIdle, uint32_t *stackSizeIdle)
{
	*tcbIdle = &xTaskBufferIdle;
	*stackIdle = idle_task_stack.data();
	*stackSizeIdle = idle_task_stack.size();
}

extern "C"
void vApplicationIdleHook(void)
{
	__WFI();
}


inline
void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void periph_init_hsem()
{
	RCC->AHB4ENR |= RCC_AHB4ENR_HSEMEN_Msk;
	//toggle_bits_10(&RCC->AHB4RSTR, RCC_AHB4RSTR_HSEMRST_Msk);
}


freertos_utils::task_data_t<1024> task_data_test1;

void task_function_test1(void*)
{
	logger.log_async("CM4: TEST1 task started\r\n");
	for(;;) {
		vTaskDelay(configTICK_RATE_HZ*10);
		logger.log_async("CM4: TEST1 task keep-alive\r\n");
	}
}

void create_task_test1()
{
	task_data_test1.task_handle = xTaskCreateStatic(
		&task_function_test1,
		"test1",
		task_data_test1.stack.size(),
		nullptr,
		PRIO_TEST1,
		task_data_test1.stack.data(),
		&task_data_test1.task_buffer
	);
}


extern "C" __attribute__ ((interrupt)) void IntHandler_Dma1S1()
{
	auto const isr = DMA1->LISR;
	uint32_t events = 0;
	if (isr & DMA_LISR_TCIF1) {
		DMA1->LIFCR = DMA_LIFCR_CTCIF1;
		events |= stm32_lib::dma::dma_result_t::tc;
	}
	if (isr & DMA_LISR_TEIF1) {
		DMA1->LIFCR = DMA_LIFCR_CTEIF1;
		events |= stm32_lib::dma::dma_result_t::te;
	}
	if (isr & DMA_LISR_DMEIF1) {
		DMA1->LIFCR = DMA_LIFCR_CDMEIF1;
		events |= stm32_lib::dma::dma_result_t::te;
	}
	if (events) {
		logger.notify_from_isr(events);
	}
}


template <typename Pin>
void blink(const Pin& pin, int n)
{
	while (n-- > 0) {
		pin.set_state(0);
		for (volatile int i=0; i<500000; ++i) {}
		pin.set_state(1);
		for (volatile int i=0; i<500000; ++i) {}
	}
}


__attribute__ ((noreturn)) void main()
{
	for (volatile int i=0; i<1000000; ++i) {}

	//const auto& pin = bsp::pin_led_red;

	//pin.set_state(1);
	//pin.set_mode_output_lowspeed_pushpull();

	//blink(pin, 1);
#if 0
	periph_init_hsem();
	HSEM_COMMON->IER |= (1 << 0); // enable interrupt for sem0
	// goto STOP mode and wait for CM7 to init periph and wakeup us
	//__WFE(); // clear pending events, HAL_PWREx_ClearPendingEvent()

	PWR->CR1 &= ~PWR_CR1_LPDS;
	PWR->CPU2CR &= ~PWR_CPU2CR_PDDS_D2_Msk; // keep STOP mode when in DEEPSLEEP
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__ISB();
	//blink(pin, 1);
	__WFE(); // sleep

	pin.set_state(1);
	pin.set_mode_output_lowspeed_pushpull();
	blink(pin, 1);

	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
	blink(pin, 1);

	HSEM_COMMON->ICR = (1 << 0); // clear interrupt status flag for sem0
#endif

	NVIC_SetPriority(DMA1_Stream1_IRQn, 12);
	NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	g_pin_blue.init_pin();
	g_pin_red.init_pin();
	//g_pin_blue.pulse_continuous(configTICK_RATE_HZ/20, configTICK_RATE_HZ/5);
	g_pin_red.pulse_continuous(configTICK_RATE_HZ, configTICK_RATE_HZ);

	log_sync("\r\nCM4: USART initialized (sync)\r\n");

	create_task_test1();

	// pinpoll task
	bsp::pin_joy_sel.set(stm32_lib::gpio::pupd_t::pu, stm32_lib::gpio::mode_t::input);
	bsp::pin_joy_down.set(stm32_lib::gpio::pupd_t::pu, stm32_lib::gpio::mode_t::input);
	bsp::pin_joy_left.set(stm32_lib::gpio::pupd_t::pu, stm32_lib::gpio::mode_t::input);
	bsp::pin_joy_right.set(stm32_lib::gpio::pupd_t::pu, stm32_lib::gpio::mode_t::input);
	bsp::pin_joy_up.set(stm32_lib::gpio::pupd_t::pu, stm32_lib::gpio::mode_t::input);
	bsp::pin_btn.set(stm32_lib::gpio::pupd_t::pd, stm32_lib::gpio::mode_t::input);
	freertos_utils::pinpoll::create_task("pinpoll", PRIO_PINPOLL, &pinpoll_task_arg);

	log_sync("CM4: Starting FreeRTOS scheduler\r\n");
	logger.init();
	vTaskStartScheduler();

	for (;;) {}
}

