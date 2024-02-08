#include "lib_stm32.h"
#include "bsp.h"
#include "logging.h"
#include "freertos_utils.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>


#define PRIO_BLINK 1
#define PRIO_LOGGER 2


#define USART_CON_BAUDRATE 115200


usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE;

	stm32_lib::gpio::set_mode_af_lowspeed_pu(bsp::usart_stlink_pin_tx, USART_STLINK_PIN_TX_AF);
	usart->BRR = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
}


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


freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red_otg("blink_red_otg", bsp::pin_led_red_otg_overcurrent, PRIO_BLINK);


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

        const auto& pin = bsp::pin_led_red;

        //pin.set_state(1);
        //pin.set_mode_output_lowspeed_pushpull();

        //blink(pin, 1);
#if 1
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

	g_pin_red.init_pin();
	g_pin_red.pulse_continuous(configTICK_RATE_HZ/10, configTICK_RATE_HZ);

	g_pin_red_otg.init_pin();
	g_pin_red_otg.pulse_continuous(configTICK_RATE_HZ/10, configTICK_RATE_HZ);

	usart_init(USART_STLINK);
	logger.set_usart(USART_STLINK);
	logger.log_sync("\r\nCM4: Logger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	for (;;) {}
}

