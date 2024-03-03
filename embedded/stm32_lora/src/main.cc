#include "lib_stm32.h"
#include "bsp.h"
#include "freertos_utils.h"
#include "logging.h"

#include "lora.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <string.h>
#include <array>

#include "ugui.h"

#define PRIO_BLINK 1
#define PRIO_DISPLAY 2
#define PRIO_LORA_EMB 5
#define PRIO_LORA_EXT 5
#define PRIO_LOGGER 8 // FIXME


#define USART_CON_BAUDRATE 115200

namespace sx1276 {
	struct hwconf_t;
	extern const hwconf_t hwc;
};

usart_logger_t logger;


void usart_init(USART_TypeDef* const usart)
{
	usart->CR1 = 0; // ensure UE flag is reset

	constexpr uint32_t cr1 = USART_CR1_TE;

	stm32_lib::gpio::set_mode_af_lowspeed_pu(bsp::usart_stlink_pin_tx, USART_STLINK_PIN_TX_AF);
	usart->BRR = configCPU_CLOCK_HZ / USART_CON_BAUDRATE;

	usart->CR1 = cr1;
	usart->CR1 = cr1 | USART_CR1_UE;
}


void toggle_bits_10(volatile uint32_t* const ptr, const uint32_t mask)
{
	const auto val = *ptr;
	*ptr = val | mask; // set mask bits to 1
	*ptr = val & ~mask; // reset mask bits to 0
}


void bus_init()
{
#if defined TARGET_STM32L072
	// flash
	FLASH->ACR |= FLASH_ACR_LATENCY;

	// clock: switch from MSI to HSI
	// turn on HSI & wait
	RCC->CR |= RCC_CR_HSION;
	while (!(RCC->CR & RCC_CR_HSIRDY)) {}
	// switch to HSI
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | (0b01 << RCC_CFGR_SW_Pos);
	// switch off MSI & wait
	RCC->CR &= ~RCC_CR_MSION;
	while (RCC->CR & RCC_CR_MSIRDY) {}

	// GPIOs
	RCC->IOPENR = RCC_IOPENR_IOPAEN_Msk | RCC_IOPENR_IOPBEN_Msk | RCC_IOPENR_IOPCEN_Msk;
	toggle_bits_10(&RCC->IOPRSTR, RCC_IOPRSTR_IOPARST | RCC_IOPRSTR_IOPBRST | RCC_IOPRSTR_IOPCRST);

	// USART2 & SPI2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN_Msk | RCC_APB1ENR_I2C1EN_Msk | RCC_APB1ENR_SPI2EN_Msk;
	toggle_bits_10(&RCC->APB1RSTR, RCC_APB1RSTR_USART2RST_Msk | RCC_APB1RSTR_I2C1RST_Msk | RCC_APB1RSTR_SPI2RST_Msk);

	// SPI1 & SYSCFG
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN_Msk | RCC_APB2ENR_SYSCFGEN_Msk;
	toggle_bits_10(&RCC->APB2RSTR, RCC_APB2RSTR_SPI1RST_Msk | RCC_APB2RSTR_SYSCFGRST_Msk);

#if 0
	// PB2 interrupt TODO: do not hardcode PB2, use bit masks stuff
	{
		auto const cr = &SYSCFG->EXTICR[pin_userbutton.reg / 4];
		*cr = (*cr & ~(SYSCFG_EXTICR1_EXTI2_Msk)) | SYSCFG_EXTICR1_EXTI2_PB;
	}

	// EXTI
	pin_userbutton.set(stm32_lib::gpio::mode_t::input, stm32_lib::gpio::pupd_t::no_pupd);
	EXTI->IMR |= (1 << pin_userbutton.reg);
	EXTI->RTSR &= ~(1 << pin_userbutton.reg); // disable rising edge interrupt (button release)
	EXTI->FTSR |= (1 << pin_userbutton.reg); // enable falling edge interrupt (button press)
	NVIC_SetPriority(EXTI2_3_IRQn, 13);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
#endif
#endif
}


//volatile bool blue_state = false;
extern "C" __attribute__ ((interrupt)) void IntHandler_EXTI23()
{
	if (EXTI->PR & (1 << bsp::pin_userbutton.reg)) {
		//stm32_lib::gpio::set_state(pin_led_blue, (blue_state = !blue_state));
		EXTI->PR = (1 << bsp::pin_userbutton.reg);
	}
}


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
	__WFI();
}


lora::task_data_t task_data_lora_emb;
lora::task_data_t task_data_lora_ext;


freertos_utils::pin_toggle_task_t g_pin_green2("blink_green2", bsp::pin_led_green2, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_blue("blink_blue", bsp::pin_led_blue, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_red("blink_red", bsp::pin_led_red, PRIO_BLINK);
freertos_utils::pin_toggle_task_t g_pin_green("blink_green", bsp::pin_led_green, PRIO_BLINK);


#define i2c_display I2C1
constexpr stm32_lib::gpio::pin_t pin_scl(GPIOB_BASE, 8);
constexpr stm32_lib::gpio::pin_t pin_sda(GPIOB_BASE, 9);
constexpr uint8_t i2c_af = 4;
constexpr uint8_t i2c_addr = 0x3C;


void init_display()
{
	stm32_lib::i2c::init_pins(pin_scl, i2c_af, pin_sda, i2c_af);
	i2c_display->CR1 = 0;
	i2c_display->CR2 = 0;
	i2c_display->TIMINGR = (i2c_display->TIMINGR & ~(I2C_TIMINGR_PRESC_Msk))
		| (0b0010 << I2C_TIMINGR_PRESC_Pos);
	i2c_display->CR1 = I2C_CR1_PE;
	logger.log_async("DISPLAY init done\r\n");
}


freertos_utils::task_data_t<1024> task_data_display;


template <size_t DataSize>
void i2c_write_data(const uint8_t* const data)
{
	static_assert(DataSize < 255);
	std::array<uint8_t, DataSize+1> buf;
	buf[0] = 0b01000000; // data
	for (size_t d = 0; d < DataSize; ++d) {
		buf[1+d] = data[d];
	}
	stm32_lib::i2c::write(i2c_display, i2c_addr, buf.data(), buf.size());
}

void i2c_write_cmd0(uint8_t cmd)
{
	const std::array<uint8_t, 2> buf{0, cmd};
	stm32_lib::i2c::write(i2c_display, i2c_addr, buf.data(), buf.size());
}

constexpr uint8_t W = 128;
constexpr uint8_t H = 32;
static std::array<uint8_t, W*H/8> buffer{0};
UG_GUI ugui;


inline
void ugui_pset(int16_t x, int16_t y, uint32_t color)
{
	const auto page = y / 8;
	const auto page_seg = x % W;
	const auto seg_bit = y % 8;
	const auto idx = page * W + page_seg;
	if (color) {
		buffer[idx] |= (1 << seg_bit);
	} else {
		buffer[idx] &= ~(1 << seg_bit);
	}

}


void flush_buffer()
{
	for (uint8_t i=0; i<H/8; ++i) {
		i2c_write_cmd0(0xB0 + i);
		i2c_write_cmd0(0x00 + 0); // offset
		i2c_write_cmd0(0x10 + 0); // offset
		i2c_write_data<W>(&buffer[W*i]);
	}
}


void task_function_display(void*)
{
	vTaskDelay(configTICK_RATE_HZ);
	logger.log_async("DISPLAY task started\r\n");
	init_display();

	constexpr std::array<uint8_t, 6> init_cmds{
		0xAE, // display off
		0xDA, 0b00000010, // COM pins configuration: sequential, disable COM left/right remap
		0x8D, 0x14, // enable charge pump
		0xAF // display on
	};
	for (const auto cmd : init_cmds) {
		i2c_write_cmd0(cmd);
	}

	flush_buffer();

	UG_Init(&ugui, ugui_pset, W, H);
	UG_SelectGUI(&ugui);
	UG_FontSelect(&FONT_8X14);

	UG_FillScreen(0);
	UG_PutString(1, 2, "123456789012345678901234567890");
	UG_Update();
	flush_buffer();

	for(;;) {
		vTaskDelay(configTICK_RATE_HZ);
	}


	logger.log_async("DISPLAY loop\r\n");
	for (;;) {
		for(uint8_t hl=0; hl<H; ++hl) {
			const auto page_hl = hl/8;
			for (uint8_t page=0; page<H/8; ++page) {
				for (size_t w=0; w<W; ++w) {
					buffer[page*W + w] = ((page == page_hl) ? (1 << (hl % 8)): 0);
				}
			}
			flush_buffer();
			//vTaskDelay(configTICK_RATE_HZ/16);
			vTaskDelay(1);
		}

		for(uint8_t wl=0; wl<W; ++wl) {
			for (auto& x : buffer) {
				x = 0;
			}
			for (uint8_t page=0; page<H/8; ++page) {
				buffer[page*W + wl] = 0xFF;
			}
			flush_buffer();
			vTaskDelay(1);
		}

		vTaskDelay(configTICK_RATE_HZ*5);
	}
}

void create_task_display()
{
	task_data_display.task_handle = xTaskCreateStatic(
		&task_function_display,
		"DISPLAY",
		task_data_display.stack.size(),
		nullptr,
		PRIO_DISPLAY,
		task_data_display.stack.data(),
		&task_data_display.task_buffer
	);
}


__attribute__ ((noreturn)) void main()
{
	bus_init();

	usart_init(USART_STLINK);
	logger.set_usart(USART_STLINK);
	logger.log_sync("\r\nLogger initialized (sync)\r\n");

	logger.log_sync("Creating logger queue...\r\n");
	logger.init_queue();
	logger.log_sync("Created logger queue\r\n");

	logger.log_sync("Creating logger task...\r\n");
	logger.create_task("logger", PRIO_LOGGER);
	logger.log_sync("Created logger task\r\n");

	g_pin_green.init_pin();
	g_pin_green2.init_pin();
	g_pin_green2.pulse_continuous(configTICK_RATE_HZ/50, configTICK_RATE_HZ/25);
	g_pin_blue.init_pin();
	g_pin_red.init_pin();

	logger.log_sync("Creating LORA_EMB task...\r\n");
	lora::create_task_emb("lora_emb", PRIO_LORA_EMB, task_data_lora_emb, &sx1276::hwc_emb);
	logger.log_sync("Created LORA_EMB task\r\n");

	logger.log_sync("Creating LORA_EXT task...\r\n");
	lora::create_task_ext("lora_ext", PRIO_LORA_EXT, task_data_lora_ext, &sx1276::hwc_ext);
	logger.log_sync("Created LORA_EXT task\r\n");

	logger.log_sync("Creating DISPLAY task...\r\n");
	create_task_display();
	logger.log_sync("Created DISPLAY task\r\n");

	logger.log_sync("Starting FreeRTOS scheduler\r\n");
	vTaskStartScheduler();

	logger.log_sync("Error in FreeRTOS scheduler\r\n");
	for (;;) {}
}

