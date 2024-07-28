#include "hardware/regs/addressmap.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/resets.h"
#include "hardware/structs/sio.h"


#define LED 25


__attribute__((noreturn)) void my_main()
{
	// unreset io_bank0
	((resets_hw_t*)(RESETS_BASE + REG_ALIAS_CLR_BITS))->reset = RESETS_RESET_IO_BANK0_BITS; // TODO better?

	// wait for unreset done
	while (! (resets_hw->reset_done & RESETS_RESET_DONE_IO_BANK0_BITS) ) {}

	iobank0_hw->io[LED].ctrl = 5; // function = sio
	sio_hw->gpio_oe_set = 1 << LED;

        for(;;) {
		// LED on
		sio_hw->gpio_set = 1 << LED;
		for (volatile unsigned i=0; i<10000; ++i) {}

		// LED off
		sio_hw->gpio_clr = 1 << LED;
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

