#ifndef _my_lora_h_included_
#define _my_lora_h_included_

#include "FreeRTOS.h"


namespace sx1276 { struct hwconf_t; };


namespace lora {


void create_task(const char* task_name, UBaseType_t prio, const sx1276::hwconf_t*);


} // namespace


#endif

