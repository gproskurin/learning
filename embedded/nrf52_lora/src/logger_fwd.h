#ifndef _my_logger_fwd_h_included_
#define _my_logger_fwd_h_included_

#include "logging.h"

using log_dev_t = logging::uarte_dev_t<NRF_UARTE0_BASE>;
extern logging::logger_t<log_dev_t> logger;

#endif

