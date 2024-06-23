//#include "logging.h"
#include "utils.h"


char prn_halfbyte(uint8_t x)
{
	return ((x <= 9) ? '0' : ('A'-10)) + x;
}

char* printf_byte(uint8_t x, char* buf)
{
	buf[0] = prn_halfbyte(x >> 4);
	buf[1] = prn_halfbyte(x & 0b1111);
	return buf+2;
}

#if 0
void log_async_1(uint8_t x, char* const buf0)
{
	auto buf = buf0;
	buf = printf_byte(x, buf);
	*buf++ = '\r';
	*buf++ = '\n';
	*buf = 0;
	logger.log_async(buf0);
}
#endif

