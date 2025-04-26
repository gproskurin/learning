#ifndef _my_player_h_included_
#define _my_player_h_included_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

namespace notes {

	enum sym_t : uint8_t {
		sym_none = 0,

		C0,
		C0s, D0f = C0s,
		D0,
		D0s, E0f = D0s,
		E0,
		F0,
		F0s, G0f = F0s,
		G0,
		G0s, A0f = G0s,
		A0,
		A0s,B0f = A0s,
		B0,

		C1,
		C1s, D1f = C1s,
		D1,
		D1s, E1f = D1s,
		E1,
		F1,
		F1s, G1f = F1s,
		G1,
		G1s, A1f = G1s,
		A1,
		A1s, B1f = A1s,
		B1,

		C2,
		C2s, D2f = C2s,
		D2,
		D2s, E2f = D2s,
		E2,
		F2,
		F2s, G2f = F2s,
		G2,
		G2s, A2f = G2s,
		A2,
		A2s, B2f = A2s,
		B2,

		C3,
		C3s, D3f = C3s,
		D3,
		D3s, E3f = D3s,
		E3,
		F3,
		F3s, G3f = F3s,
		G3,
		G3s, A3f = G3s,
		A3,
		A3s, B3f = A3s,
		B3,

		C4,
		C4s, D4f = C4s,
		D4,
		D4s, E4f = D4s,
		E4,
		F4,
		F4s, G4f = F4s,
		G4,
		G4s, A4f = G4s,
		A4,
		A4s, B4f = A4s,
		B4,

		C5,
		C5s, D5f = C5s,
		D5,
		D5s, E5f = D5s,
		E5,
		F5,
		F5s, G5f = F5s,
		G5,
		G5s, A5f = G5s,
		A5,
		A5s, B5f = A5s,
		B5,

		C6,
		C6s, D6f = C6s,
		D6,
		D6s, E6f = D6s,
		E6,
		F6,
		F6s, G6f = F6s,
		G6,
		G6s, A6f = G6s,
		A6,
		A6s, B6f = A6s,
		B6,

		C7,
		C7s, D7f = C7s,
		D7,
		D7s, E7f = D7s,
		E7,
		F7,
		F7s, G7f = F7s,
		G7,
		G7s, A7f = G7s,
		A7,
		A7s, B7f = A7s,
		B7,

		C8,
		C8s, D8f = C8s,
		D8,
		D8s, E8f = D8s,
		E8,
		F8,
		F8s, G8f = F8s,
		G8,
		G8s, A8f = G8s,
		A8,
		A8s, B8f = A8s,
		B8
	};
	static_assert(sym_t::B8 <= 255);

	enum duration_t : uint8_t {
		dur_zero = 0,
		l1 = 64,
		l2 = l1/2,
		l2d = l2 + l2/2,
		l4 = l2/2,
		l4d = l4 + l4/2,
		l4s = l4/4*3, //staccato
		l8 = l4/2,
		l8d = l8 + l8/2,
		l16 = l8/2,
		l16s = l16/4*3,
		l32 = l16/2,
	};
	// avoid truncation
	static_assert(duration_t::l32 * 32 == duration_t::l1);
	static_assert(duration_t::l16s * 4 / 3 == duration_t::l16);

	enum instrument_t : uint8_t {
		sq, sin3, sin4, sin5, sin10, sin12
	};

} // namespace notes


namespace player {

constexpr uint32_t dds_freq = 32000; // keep in sync with python generator

inline
uint32_t dds_ticks(notes::duration_t d)
{
	constexpr uint32_t tempo_l4_per_minute = 60;
	constexpr uint32_t fade = dds_freq/1000;
	constexpr uint32_t mul = dds_freq * tempo_l4_per_minute / notes::l4 / 60;
	auto const r = mul * d;
	return (r <= fade) ? r : (r - fade);
}

void create_task(const char* task_name, UBaseType_t prio);
void enqueue_note(notes::sym_t, notes::duration_t, notes::instrument_t);

}


#endif

