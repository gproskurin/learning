#ifndef __my_common_utils_h_included__
#define __my_common_utils_h_included__

#include <stddef.h>
#include <stdint.h>


namespace mailbox {


static_assert(sizeof(uintptr_t) == sizeof(uint32_t));

enum mb_type : uint32_t {
	// some magic numbers
	buffer = 0x1234EEFF,
	string0 = 0xEEFF3427
};

template <mb_type Mt>
struct mailbox_t;

template <>
struct mailbox_t<buffer> {
	const mb_type type = buffer;
	size_t size;
	void* data;
};

template <>
struct mailbox_t<string0> {
	const mb_type type = string0;
	const char* s;
};


inline
void* get_mb_ptr(uint32_t addr)
{
	return *reinterpret_cast<void**>(addr);
}


inline
mb_type get_mb_type(const void* mb_ptr)
{
	return *reinterpret_cast<const mb_type*>(mb_ptr);
}


template <mb_type Mt>
mailbox_t<Mt>* mb_cast(void* mb_ptr)
{
	return reinterpret_cast<mailbox_t<Mt>*>(mb_ptr);
}


template <mb_type Mt>
void set(uint32_t addr, mailbox_t<Mt>* const mp)
{
	// NOTE: make sure shared mailbox memory is flushed before "have new data" interrupt is received
	*reinterpret_cast<mailbox_t<Mt>**>(addr) = mp;
}


} // mailbox


void log_async_1(uint8_t x, char* const buf);


#endif

