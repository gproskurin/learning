#ifndef __my_common_utils_h_included__
#define __my_common_utils_h_included__

#include <stddef.h>
#include <stdint.h>
#include <array>


namespace mailbox {

// NOTE:
// These structures are shared between CM0 and CM4 cores. Beware of potential alignment issues.

static_assert(sizeof(uintptr_t) == sizeof(uint32_t));

enum mb_type : uint32_t {
	// some magic numbers
	buffer = 0x1234EEFF,
	string0 = 0xEEFF3427,
	lora_packet = 0x547EF7BA
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

template <>
struct mailbox_t<lora_packet> {
	const mb_type type = lora_packet;

	std::array<uint8_t, 256> data;
	uint8_t data_len;

	int8_t rssi_pkt;
	int8_t snr_pkt;
	int8_t signal_rssi_pkt;
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

char* printf_byte(uint8_t x, char* buf);


#endif

