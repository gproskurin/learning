#ifndef _my_lora_h_included_
#define _my_lora_h_included_

#include "freertos_utils.h"
#include "sx1276.h"

#include "FreeRTOS.h"


namespace lora {


struct task_data_t {
	freertos_utils::task_stack_t<256> stack;
	TaskHandle_t task_handle = nullptr;
	StaticTask_t task_buffer;
};

void create_task_ext(const char* task_name, UBaseType_t prio, task_data_t&, const sx1276::hwconf_t*);
void create_task_emb(const char* task_name, UBaseType_t prio, task_data_t&, const sx1276::hwconf_t*);


} // namespace

#endif

