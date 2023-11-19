#include "lib_stm32.h"
#include "freertos_utils.h"
//#include "logging.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#include <algorithm>
#include <array>


#define PRIO_BLINK 1
#define PRIO_M 2
//#define PRIO_LOGGER 3

#define TIM_MOTOR TIM1


struct pin_info_t {
	uint8_t reg;
	bool inverted;
	constexpr pin_info_t(uint8_t r, bool i) : reg(r), inverted(i) {}
};

constexpr pin_info_t m1_hi(2, false);
constexpr pin_info_t m1_lo(3, false);
constexpr pin_info_t m2_hi(4, false);
constexpr pin_info_t m2_lo(5, false);
constexpr pin_info_t m3_hi(6, false);
constexpr pin_info_t m3_lo(7, false);

using pin_group_t = std::array<pin_info_t, 2>;
constexpr std::array<pin_group_t, 3> pin_groups{
	pin_group_t{m1_hi, m2_lo},
	pin_group_t{m2_hi, m3_lo},
	pin_group_t{m3_hi, m1_lo}
};

template <size_t Size>
constexpr std::array<uint32_t, Size*2> gen_bsrr_actions(const std::array<pin_group_t, Size>& pg)
{
	std::array<uint32_t, Size*2> result{};
	for(size_t i=0; i<pg.size(); ++i) {
		uint32_t set = 0, reset = 0;
		for (const auto& pin : pg[i]) {
			if (pin.inverted) {
				reset |= 1 << pin.reg;
			} else {
				set |= 1 << pin.reg;
			}
		}
		result[i*2] = (reset << 16) | set; // raising edge
		result[i*2+1] = (set << 16) | reset; // falling edge
	}
	return result;
}


template <size_t Size>
constexpr std::array<uint32_t, Size> rev_bsrr_actions(const std::array<uint32_t, Size>& x)
{
	static_assert(Size >= 4);
	std::array<uint32_t,Size> r(x);
	std::swap(r[0], r[2]);
	std::swap(r[1], r[3]);
	return r;
}


// used by DMA
const std::array<uint32_t, 6> bsrr_actions = gen_bsrr_actions(pin_groups);
const std::array<uint32_t, 6> bsrr_actions_rev = rev_bsrr_actions(bsrr_actions);

static
void init_pin_motor(const pin_info_t& pi)
{
	stm32_lib::gpio::pin_t pin(GPIOA_BASE, pi.reg); // FIXME
	pin.set_state(pi.inverted); // lo for non-inverted, hi for inverted
	pin.set(
		stm32_lib::gpio::otype_t::push_pull,
		stm32_lib::gpio::mode_t::output,
		stm32_lib::gpio::speed_t::bits_00,
		stm32_lib::gpio::pupd_t::no_pupd
	);
}


constexpr stm32_lib::gpio::pin_t pin_led_keepalive{GPIOA_BASE, 8};

#if 0
#define USART_CON_BAUDRATE 115200
#define USART_LOG USART1
#define USART_LOG_TX_AF 1
constexpr stm32_lib::gpio::pin_t pin_usart_tx{GPIOA_BASE, 9};


usart_logger_t logger;


static
void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_FIFOEN | USART_CR1_TE;

#ifdef TARGET_STM32F103
	stm32_lib::gpio::set_mode_af_lowspeed_pu(usart_stlink_pin_tx);
	const uint32_t div = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;
	usart->BRR = ((div / 16) << USART_BRR_DIV_Mantissa_Pos) | ((div % 16) << USART_BRR_DIV_Fraction_Pos);
#else
	stm32_lib::gpio::set_mode_af_lowspeed_pu(pin_usart_tx, USART_LOG_TX_AF);
	usart->BRR = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;
#endif

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
}
#endif


static
void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


static
void periph_init()
{
#if defined TARGET_STM32G031
	// GPIOs
	RCC->IOPENR = RCC_IOPENR_GPIOAEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_GPIOARST);

	// USART1 & SYSCFG & TIM1
	RCC->APBENR2 = /* RCC_APBENR2_USART1EN_Msk | RCC_APBENR2_SYSCFGEN_Msk |*/ RCC_APBENR2_TIM1EN_Msk;
	toggle_bits_10(&RCC->APBRSTR2, /* RCC_APBRSTR2_USART1RST_Msk | RCC_APBRSTR2_SYSCFGRST_Msk |*/ RCC_APBRSTR2_TIM1RST_Msk);

	// enable PA9 in place of PA11
	//SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_RMP;
#endif
}


StaticTask_t xTaskBufferIdle;
using idle_task_stack_t = freertos_utils::task_stack_t<64>;
idle_task_stack_t idle_task_stack;
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


void timer_motor_init()
{
	constexpr uint16_t arr = 60000; // 0..10..20, 20..30..40, 40..50..60
	TIM_MOTOR->CR1 = 0;
	TIM_MOTOR->CR2 = 0;
	TIM_MOTOR->PSC = 16;
	TIM_MOTOR->ARR = arr;
	TIM_MOTOR->CNT = 0;
	TIM_MOTOR->DMAR = reinterpret_cast<uint32_t>(&GPIOA->BSRR);

	// channel1
	TIM_MOTOR->CCR1 = arr / 6 - 10;
}

void timer_motor_start()
{
	TIM_MOTOR->CR1 |= TIM_CR1_CEN;
}


#ifdef TARGET_STM32G031
freertos_utils::pin_toggle_task_t g_pin_led_keepalive("blink_keepalive", pin_led_keepalive, PRIO_BLINK);
#endif


void task_function_m(void*)
{
	//logger.log_async("M task started\r\n");
	g_pin_led_keepalive.pulse_continuous(configTICK_RATE_HZ/10, configTICK_RATE_HZ/5);

	init_pin_motor(m1_hi);
	init_pin_motor(m1_lo);
	init_pin_motor(m2_hi);
	init_pin_motor(m2_lo);
	init_pin_motor(m3_hi);
	init_pin_motor(m3_lo);

	const uint32_t* const ap = bsrr_actions.data();
	//const uint32_t* const ap = bsrr_actions_rev.data();

	static_assert(bsrr_actions.size() > 0);
	static_assert((bsrr_actions.size() & 1) == 0);
	size_t i = 0;
	constexpr auto tm_lo = configTICK_RATE_HZ/2;
	constexpr auto tm_hi = configTICK_RATE_HZ/5;

	vTaskDelay(configTICK_RATE_HZ*3);
	for(;;) {
		vTaskDelay(tm_lo);

		GPIOA->BSRR = ap[i++]; // raise edge
		vTaskDelay(tm_hi);
		GPIOA->BSRR = ap[i]; // falling edge

		vTaskDelay(tm_lo);

		if (i == bsrr_actions.size() - 1) {
			i = 0;
		} else {
			++i;
		}
	}
}


freertos_utils::task_data_t<128> task_data_m;
void create_task_m()
{
	task_data_m.task_handle = xTaskCreateStatic(
		&task_function_m,
		"M",
		task_data_m.stack.size(),
		nullptr,
		PRIO_M,
		task_data_m.stack.data(),
		&task_data_m.task_buffer
	);
}



__attribute__ ((noreturn)) void main()
{
	periph_init();

#if 0
	usart_init(USART_LOG);
	logger.set_usart(USART_LOG);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");
#endif

	g_pin_led_keepalive.init_pin();

	//logger.log_sync("Creating task M...\r\n");
	create_task_m();
	//logger.log_sync("Created task M\r\n");

	//logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	//logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

