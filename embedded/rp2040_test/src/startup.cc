#include "hardware/regs/addressmap.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/resets.h"

#include "lib_rp2040.h"
#include "bsp.h"


__attribute__((noreturn)) void my_main()
{
	// unreset io_bank0
	resets_hw->reset = ~RESETS_RESET_IO_BANK0_BITS;
	while (! (resets_hw->reset_done & RESETS_RESET_DONE_IO_BANK0_BITS) ) {}

	bsp::pin_led.set(
		rp2040_lib::gpio::af_t(5), // function = sio
		rp2040_lib::gpio::mode_t::output
	);

	for(;;) {
		bsp::pin_led.set_state(1);
		for (volatile unsigned i=0; i<10000; ++i) {}

		bsp::pin_led.set_state(0);
		for (volatile unsigned i=0; i<200000; ++i) {}
	}
}


__attribute__((naked, noreturn)) void _reset()
{
	my_main();
}


extern "C" void _estack();
__attribute__((section(".vectors"))) void (*tab[2])(void) =
{
	_estack,
	_reset,
};

