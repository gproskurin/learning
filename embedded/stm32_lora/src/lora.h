#ifndef _my_lora_h_included_
#define _my_lora_h_included_

#include "sx1276.h"

#include "FreeRTOS.h"


namespace lora {


void create_task(const char* task_name, UBaseType_t prio);


} // namespace

#endif

