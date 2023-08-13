#include "player.h"

#include "cmsis_device.h"

#include <array>


const std::array<uint32_t, 12*9> freq100 = {
	1635, 1732, 1835, 1945, 2060, 2183, 2312, 2450, 2596, 2750, 2914, 3087,
	3270, 3465, 3671, 3889, 4120, 4365, 4625, 4900, 5191, 5500, 5827, 6174,
	6541, 6930, 7342, 7778, 8241, 8731, 9250, 9800, 10383, 11000, 11654, 12347,
	13081, 13859, 14683, 15556, 16481, 17461, 18500, 19600, 20765, 22000, 23308, 24694,
	26163, 27718, 29366, 31113, 32963, 34923, 36999, 39200, 41530, 44000, 46616, 49388,
	52325, 55437, 58733, 62225, 65925, 69846, 73999, 78399, 83061, 88000, 93233, 98777,
	104650, 110873, 117466, 124451, 131851, 139691, 147998, 156798, 166122, 176000, 186466, 197553,
	209300, 221746, 234932, 248902, 263702, 279383, 295996, 313596, 332244, 352000, 372931, 395107,
	418601, 443492, 469863, 497803, 527404, 558765, 591991, 627193, 664488, 704000, 745862, 790213
};


typedef uint32_t queue_item_t;

queue_item_t queue_item_encode(notes::sym_t n, notes::duration_t d)
{
	return static_cast<uint32_t>(n) | (static_cast<uint32_t>(d) << 8);
}

void queue_item_decode(queue_item_t item, notes::sym_t& n, notes::duration_t& d)
{
	n = static_cast<notes::sym_t>(item & 0xff);
	d = static_cast<notes::duration_t>(item >> 8);
}


void player::enqueue_note(QueueHandle_t queue_handle, notes::sym_t n, notes::duration_t d)
{
	const queue_item_t item = queue_item_encode(n, d);
	xQueueSend(queue_handle, &item, 0);
}


struct queue_data_t {
	std::array<queue_item_t, 16> buffer;
	StaticQueue_t q;
} queue_data;

QueueHandle_t player::create_queue()
{
	return xQueueCreateStatic(
		queue_data.buffer.size(),
		sizeof(queue_item_t),
		reinterpret_cast<uint8_t*>(queue_data.buffer.data()),
		&queue_data.q
	);
}


struct play_task_data_t {
	std::array<StackType_t, 128> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
} play_task_data;

struct task_args_t {
	QueueHandle_t queue_handle;
} task_args;


#define TIM_PWM TIM15
#define CLOCK_SPEED configCPU_CLOCK_HZ
void play_note(notes::sym_t n, notes::duration_t d)
{
	const auto fade = configTICK_RATE_HZ/256;
	const uint16_t arr = uint32_t(CLOCK_SPEED)/(freq100[n]/100) - 1;
	TIM_PWM->ARR = arr;
	TIM_PWM->CCR1 = arr / 2 + 1;
	TIM_PWM->CR1 = TIM_CR1_CEN;
	vTaskDelay(configTICK_RATE_HZ/64*d - fade);
	TIM_PWM->CR1 = 0;
	vTaskDelay(fade);
}


void play_task_function(void* arg)
{
	task_args_t* const args = static_cast<task_args_t*>(arg);
	for(;;) {
		queue_item_t item;
		if (xQueueReceive(args->queue_handle, &item, configTICK_RATE_HZ/*1sec*/) == pdTRUE) {
			notes::sym_t n;
			notes::duration_t d;
			queue_item_decode(item, n, d);
			play_note(n, d);
		}
	}
}


void player::create_task(const char* task_name, UBaseType_t prio, QueueHandle_t queue_handle)
{
	task_args.queue_handle = queue_handle;
	xTaskCreateStatic(
		&play_task_function,
		task_name,
		play_task_data.stack.size(),
		&task_args,
		prio,
		play_task_data.stack.data(),
		&play_task_data.task_buffer
	);
}

