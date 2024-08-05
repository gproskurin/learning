#include "cmsis_device.h"

#include "FreeRTOS.h"
#include "task.h"

#include "freertos_utils.h"
#include "logging.h"
#include "lib_nrf5.h"
#include "bsp.h"


#include <stdint.h>
#include <string.h>

#define PRIO_BLINK 1
#define PRIO_NRF52 2
#define PRIO_PINPOLL 3
#define PRIO_LOGGER 4


#define LOG_UART UARTE_VCOM

logging::logger_t<logging::uarte_dev_t<NRF_UARTE0_BASE>> logger("logger", PRIO_LOGGER);


void log_sync(const char* s)
{
	nrf5_lib::uart::uarte_send(LOG_UART, s);
}


auto g_pin_led1 = freertos_utils::make_pin_toggle_task("blink_led1", bsp::pin_led_1, PRIO_BLINK);
auto g_pin_led2 = freertos_utils::make_pin_toggle_task("blink_led2", bsp::pin_led_2, PRIO_BLINK);
auto g_pin_led3 = freertos_utils::make_pin_toggle_task("blink_led3", bsp::pin_led_3, PRIO_BLINK);
auto g_pin_led4 = freertos_utils::make_pin_toggle_task("blink_led4", bsp::pin_led_4, PRIO_BLINK);


template <typename BlinkTask>
void handle_btn(BlinkTask& t, bool s)
{
	if (s) {
		// release
		t.pulse_once(configTICK_RATE_HZ/10);
	} else {
		// press
		t.pulse_many(configTICK_RATE_HZ/10, configTICK_RATE_HZ/10, 3);
	}
}

using btn_t = nrf5_lib::gpio::pin_t;

void btn_cb_1(const btn_t&, bool s) { handle_btn(g_pin_led1, s); }
void btn_cb_2(const btn_t&, bool s) { handle_btn(g_pin_led2, s); }
void btn_cb_3(const btn_t&, bool s) { handle_btn(g_pin_led3, s); }
void btn_cb_4(const btn_t&, bool s) { handle_btn(g_pin_led4, s); }

freertos_utils::pinpoll::task_arg_t<btn_t, 4> pinpoll_task_args{
	freertos_utils::pinpoll::make_pin_info(bsp::pin_button_1, btn_cb_1),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_button_2, btn_cb_2),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_button_3, btn_cb_3),
	freertos_utils::pinpoll::make_pin_info(bsp::pin_button_4, btn_cb_4)
};


StaticTask_t xTaskBufferIdle;
freertos_utils::task_stack_t<64> idle_task_stack;
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
	//__WFI();
}


void nrf52_task_function(void* arg)
{
	const char* const str = "NRF-52 task keep-alive from RAM\r\n";
	static std::array<char, 64> ram_buf;
	memcpy(ram_buf.data(), str, strlen(str)+1); // copy including terminating zero

	logger.log_async("NRF-52 task started\r\n");
	for (;;) {
		vTaskDelay(configTICK_RATE_HZ*10);
		logger.log_async("NRF-52 task keep-alive from FLASH\r\n");
		logger.log_async(ram_buf.data());
	}
}


freertos_utils::task_data_t<128> nrf52_task_data;

void create_nrf52_task(const char* task_name, UBaseType_t prio)
{
	nrf52_task_data.task_handle = xTaskCreateStatic(
		&nrf52_task_function,
		task_name,
		nrf52_task_data.stack.size(),
		nullptr,
		prio,
		nrf52_task_data.stack.data(),
		&nrf52_task_data.task_buffer
	);
}


extern "C" __attribute__ ((interrupt)) void IntHandler_Uart0()
{
	if (NRF_UARTE0->EVENTS_ENDTX) {
		NRF_UARTE0->EVENTS_ENDTX = 0;
		logger.notify_from_isr(0b11); // FIXME
	}
}


__attribute__ ((noreturn)) void main()
{
	nrf5_lib::uart::uarte_init(LOG_UART, bsp::pin_vcom_txd);
	LOG_UART->INTEN = 0;
	log_sync("\r\nLogger initialized\r\n");

	g_pin_led1.init_pin();
	g_pin_led2.init_pin();
	g_pin_led3.init_pin();
	g_pin_led4.init_pin();
	g_pin_led4.pulse_continuous(configTICK_RATE_HZ/10, configTICK_RATE_HZ/5);

	log_sync("Creating NRF-52 task...\r\n");
	create_nrf52_task("nrf52_task", PRIO_NRF52);
	log_sync("Created NRF-52 task\r\n");

	log_sync("Creating PINPOLL task...\r\n");
	freertos_utils::pinpoll::create_task("pinpoll", PRIO_PINPOLL, &pinpoll_task_args);
	log_sync("Created PINPOLL task\r\n");

	log_sync("Starting FreeRTOS scheduler\r\n");

	LOG_UART->INTEN = UARTE_INTEN_ENDTX_Msk;
	logger.init();
	NVIC_EnableIRQ(UARTE0_UART0_IRQn);
	vTaskStartScheduler();

	log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

