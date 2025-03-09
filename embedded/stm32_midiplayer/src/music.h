#ifndef _my_music_h_included_
#define _my_music_h_included_

#include <array>

const std::array<beat_t, 67> music_menuet_g_minor{
	// 1
	beat_t{l4, notes_t{note_t{B5f, l4}, note_t{G3, l2d}}},
	beat_t{l4, notes_t{note_t{A5, l4}}},
	beat_t{l4, notes_t{note_t{G5, l4}}},

	// 2
	beat_t{l4, notes_t{note_t{A5, l4}, note_t{F3, l2d}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},

	// 3
	beat_t{l4, notes_t{note_t{G5, l4}, {E3f, l2d}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{B4f, l8}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 4
	beat_t{l4, notes_t{note_t{D3, l4}, note_t{D5, l2d}}},
	beat_t{l8, notes_t{note_t{D4, l8}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},
	beat_t{l8, notes_t{note_t{B3f, l8}}},
	beat_t{l8, notes_t{note_t{A3, l8}}},

	// 5
	beat_t{l4, notes_t{note_t{E5f, l4}, note_t{B3f, l2}, note_t{G3, l2}}},
	beat_t{l8, notes_t{note_t{F5, l8}}},
	beat_t{l8, notes_t{note_t{E5f, l8}}},
	beat_t{l8, notes_t{note_t{D5, l8}, note_t{A3, l4}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 6
	beat_t{l4, notes_t{note_t{D5,l4}, note_t{B3f, l2}}},
	beat_t{l8, notes_t{note_t{E5f,l8}}},
	beat_t{l8, notes_t{note_t{D5,l8}}},
	beat_t{l8, notes_t{note_t{C5,l8}, note_t{G3,l8}}},
	beat_t{l8, notes_t{note_t{B4f,l8}}},

	// 7
	beat_t{l4, notes_t{note_t{C5,l4}, note_t{A3,l4}}},
	beat_t{l8, notes_t{note_t{D5,l8}, note_t{F3s,l4}}},
	beat_t{l8, notes_t{note_t{C5,l8}}},
	beat_t{l8, notes_t{note_t{B4f,l8}, note_t{G3,l4}}},
	beat_t{l8, notes_t{note_t{C5,l8}}},

	// 8
	beat_t{l4, notes_t{note_t{A4,l2d}, note_t{D3, l4}}},
	beat_t{l8, notes_t{note_t{D4,l8}}},
	beat_t{l8, notes_t{note_t{C4,l8}}},
	beat_t{l8, notes_t{note_t{B3f,l8}}},
	beat_t{l8, notes_t{note_t{A3,l8}}},

	// 9
	beat_t{l4, notes_t{note_t{B5f, l4}, note_t{G3,l2d}}},
	beat_t{l4, notes_t{note_t{A5, l4}}},
	beat_t{l4, notes_t{note_t{G5, l4}}},

	// 10
	beat_t{l4, notes_t{note_t{A5, l4}, note_t{F3,l2d}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},
	beat_t{l4, notes_t{note_t{D5, l4}}},

	// 11
	beat_t{l4, notes_t{note_t{G5, l4}, note_t{E3f,l2d}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{B4f, l8}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 12
	beat_t{l4, notes_t{note_t{D5, l2d}, note_t{D3,l4}}},
	beat_t{l8, notes_t{note_t{D4, l8}}},
	beat_t{l8, notes_t{note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{B4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},

	// 13
	beat_t{l4, notes_t{note_t{F5, l4}, note_t{D4,l2}, note_t{B4,l2}}},
	beat_t{l8, notes_t{note_t{G5, l8}}},
	beat_t{l8, notes_t{note_t{F5, l8}}},
	beat_t{l8, notes_t{note_t{E5f, l8}, note_t{G3,l4}}},
	beat_t{l8, notes_t{note_t{D5, l8}}},

	// 14
	beat_t{l4, notes_t{note_t{E5f, l4}, note_t{C4, l4}}},
	beat_t{l8, notes_t{note_t{F5, l8}, note_t{A3, l4}}},
	beat_t{l8, notes_t{note_t{E5f, l8}}},
	beat_t{l8, notes_t{note_t{D5, l8}, note_t{F3,l4}}},
	beat_t{l8, notes_t{note_t{C5, l8}}},

	// 15
	beat_t{l4, notes_t{note_t{D5, l4}, note_t{B3f,l4}}},
	beat_t{l4, notes_t{note_t{G5, l4}, note_t{E3f,l4}}},
	beat_t{l4, notes_t{note_t{C5, l4}, note_t{F3,l4}, note_t{A3,l4}}},

	// 16
	beat_t{l4, notes_t{note_t{B4f, l2d}, note_t{F4, l2d}, note_t{D4, l2d}, note_t{B3f,l4}}},
	beat_t{l2, notes_t{note_t{B2f, l2}}}
};

const std::array<beat_t, 194> music_k545{
	// 1
	beat_t{l8, notes_t{note_t{C5, l2}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{E5, l4}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{G5, l4}, note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},

	// 2
	beat_t{l8, notes_t{note_t{B4, l4d}, note_t{D4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{F4, l8}}},
	beat_t{l16, notes_t{note_t{G4, l8}, note_t{C5, l16}}},
	beat_t{l16, notes_t{note_t{D5, l16}}},

	beat_t{l8, notes_t{note_t{C5, l4}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},

	// 3
	beat_t{l8, notes_t{note_t{A5, l2}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{F4, l8}}},
	beat_t{l8, notes_t{note_t{A4, l8}}},
	beat_t{l8, notes_t{note_t{G5, l4}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{C6, l4}, note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},

	// 4
	beat_t{l8, notes_t{note_t{G5,l4}, note_t{B3, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{F5,l8}, note_t{D4, l8}}},
	beat_t{l16, notes_t{note_t{E5,l16}, note_t{G4, l8}}},
	beat_t{l16, notes_t{note_t{F5, l16}}},
	beat_t{l8, notes_t{note_t{E5,l4}, note_t{C4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},
	beat_t{l8, notes_t{note_t{E4, l8}}},
	beat_t{l8, notes_t{note_t{G4, l8}}},

	// 5
	beat_t{l8, notes_t{note_t{A4,l8}, note_t{F4, l4}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},

	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},

	beat_t{l16, notes_t{note_t{A5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},

	beat_t{l16, notes_t{note_t{D5,l16}, note_t{C4,l4s}, note_t{F3,l4s}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},

	// 6
	beat_t{l8, notes_t{note_t{G4,l8}, note_t{C4,l4s}, note_t{E3,l4s}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},

	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},

	beat_t{l16, notes_t{note_t{G5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},

	beat_t{l16, notes_t{note_t{C5,l16}, note_t{C4,l4s}, note_t{E3,l4s}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},

	// 7
	beat_t{l8, notes_t{note_t{F4,l8}, note_t{C4,l4s}, note_t{D3,l4s}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},

	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},

	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},

	beat_t{l16, notes_t{note_t{B4,l16}, note_t{B3,l4s}, note_t{D3,l4s}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},
	beat_t{l16, notes_t{note_t{F4,l16}}},

	// 8
	beat_t{l8, notes_t{note_t{E4,l8}, note_t{C4,l4s}, note_t{C3,l4s}}},
	beat_t{l16, notes_t{note_t{F4,l16}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},

	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},

	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},

	beat_t{l16, notes_t{note_t{A4,l16}, note_t{C3,l4s}, note_t{E3,l4s}}},
	beat_t{l16, notes_t{note_t{G4,l16}}},
	beat_t{l16, notes_t{note_t{F4,l16}}},
	beat_t{l16, notes_t{note_t{E4,l16}}},

	// 9
	beat_t{l8, notes_t{note_t{D4,l8}, note_t{F3,l1}, note_t{A3,l1}}},
	beat_t{l16, notes_t{note_t{E4,l16}}},
	beat_t{l16, notes_t{note_t{F4,l16}}},

	beat_t{l16, notes_t{note_t{G4,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5s,l16}}},

	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{A4,l16}}},
	beat_t{l16, notes_t{note_t{B4,l16}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},

	beat_t{l16, notes_t{note_t{D5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},

	// 10
	beat_t{l16, notes_t{note_t{A5,l16}, note_t{F3,l4d}}},
	beat_t{l16, notes_t{note_t{B5,l16}}},
	beat_t{l16, notes_t{note_t{C6,l16}}},
	beat_t{l16, notes_t{note_t{B5,l16}}},

	beat_t{l16, notes_t{note_t{A5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},
	beat_t{l16, notes_t{note_t{F5,l16}, note_t{G3,l8}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},

	beat_t{l16, notes_t{note_t{F5,l16}, note_t{A3,l4d}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},
	beat_t{l16, notes_t{note_t{A5,l16}}},
	beat_t{l16, notes_t{note_t{G5,l16}}},

	beat_t{l16, notes_t{note_t{F5,l16}}},
	beat_t{l16, notes_t{note_t{E5,l16}}},
	beat_t{l16, notes_t{note_t{D5,l16}, note_t{F3s,l8}}},
	beat_t{l16, notes_t{note_t{C5,l16}}},

	// 11
	beat_t{l16, notes_t{note_t{G2,l16}, note_t{B4,l4}}},
	beat_t{l16, notes_t{note_t{B2,l16}}},
	beat_t{l16, notes_t{note_t{D3,l16}, note_t{G5,l4}}},
	beat_t{l16, notes_t{note_t{G3,l16}}},

	beat_t{l16, notes_t{note_t{G2,l16}, note_t{E5,l4s}}},
	beat_t{l16, notes_t{note_t{C3,l16}}},
	beat_t{l16, notes_t{note_t{E3,l16}, note_t{C5,l4s}}},
	beat_t{l16, notes_t{note_t{G3,l16}}},

	beat_t{l16, notes_t{note_t{G2,l16}, note_t{D5,l4}}},
	beat_t{l16, notes_t{note_t{B2,l16}}},
	beat_t{l16, notes_t{note_t{D3,l16}, note_t{G5,l4}}},
	beat_t{l16, notes_t{note_t{G3,l16}}},

	beat_t{l16, notes_t{note_t{G2,l16}, note_t{E5,l4s}}},
	beat_t{l16, notes_t{note_t{C3,l16}}},
	beat_t{l16, notes_t{note_t{E3,l16}, note_t{C5,l4s}}},
	beat_t{l16, notes_t{note_t{G3,l16}}},

	// 12
	beat_t{l4, notes_t{note_t{D5,l4s}, note_t{G2,l4s}}},
	beat_t{l4, notes_t{note_t{G5,l4s}, note_t{D5,l4s}, note_t{B4,l4s}, note_t{G3,l4s}}},
	beat_t{l4, notes_t{note_t{G4,l4s}, note_t{G2,l4s}}},
	beat_t{l4, notes_t{}},

	// 13
	beat_t{l16, notes_t{note_t{C4s,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4s,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4s,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4s,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},

	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},

	// 14
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{D6,l8}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{B5,l8}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{G5,l4d}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{A5,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}, note_t{B5,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{A5,l8}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}, note_t{G5,l8}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},

	// 15
	beat_t{l32, notes_t{note_t{C4,l16}, note_t{A5,l32}}},
	beat_t{l32, notes_t{note_t{G5,static_cast<duration_t>(static_cast<uint8_t>(l8d)-static_cast<uint8_t>(l32))}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}, note_t{F5s,l16s}}},
	beat_t{l16, notes_t{note_t{A3,l16}, note_t{F5s,l4}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{B3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{C4,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
	beat_t{l16, notes_t{note_t{A3,l16}}},
	beat_t{l16, notes_t{note_t{D4,l16}}},
};

const std::array<notes::sym_t, 8> Cmaj{C4, D4, E4, F4, G4, A4, B4, C5};
const std::array<notes::sym_t, 8> Am{A4, B4, C5, D5, E5, F5, G5, A5};

#endif

