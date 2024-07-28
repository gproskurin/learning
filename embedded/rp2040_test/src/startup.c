#include <stdint.h>

extern void _estack(void);

#define PUT32(addr, value) do { *(volatile uint32_t*)(addr) = (uint32_t)value; } while(0)
#define GET32(addr) (*(volatile uint32_t*)(addr))

#define REGSHIFT_XOR 0x1000
#define REGSHIFT_SET 0x2000
#define REGSHIFT_CLEAR 0x3000

#define REG_RESETS_BASE (0x4000c000UL)
#define REG_IO_BANK0_BASE (0x40014000UL)
#define REG_SIO_BASE (0xd0000000UL)

#define REG_RESETS_DONE (REG_RESETS_BASE + 8)

#define REG_RESETS_IO_BANK0_Msk 0x20

#define GPIO_OE_ADDR (REG_SIO_BASE + 0x20)
#define GPIO_OE_SET_ADDR (REG_SIO_BASE + 0x24)

#define GPIO_CTRL_ADDR(r) (REG_IO_BANK0_BASE + (8*r) + 4)

#define GPIO_OUT_SET_ADDR (REG_SIO_BASE + 0x14)
#define GPIO_OUT_CLR_ADDR (REG_SIO_BASE + 0x18)
#define GPIO_OUT_XOR_ADDR (REG_SIO_BASE + 0x1c)

#define LED 25

__attribute__((/*naked,*/ noreturn)) void _reset(void)
{
	// unreset io_bank0
	PUT32(REG_RESETS_BASE + REGSHIFT_CLEAR, REG_RESETS_IO_BANK0_Msk);

	// wait for unreset done
	while (! (GET32(REG_RESETS_DONE) & REG_RESETS_IO_BANK0_Msk)) {}

	PUT32(GPIO_CTRL_ADDR(LED), 5); // set function5

	PUT32(GPIO_OE_SET_ADDR, (1 << LED)); // OE

        for(;;) {
		// LED on
		PUT32(GPIO_OUT_SET_ADDR, 1 << LED);
		for (volatile unsigned i=0; i<10000; ++i) {}

		// LED off
		PUT32(GPIO_OUT_CLR_ADDR, 1 << LED);
		for (volatile unsigned i=0; i<200000; ++i) {}
        }
}


__attribute__((section(".vectors"))) void (*tab[2])(void) =
{
	_estack,
	_reset,
};

