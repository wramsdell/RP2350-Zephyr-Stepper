#include <stdint.h>

#include "regs.h"

extern uint32_t __bss_start__;
extern uint32_t __bss_end__;

void core1_main(void);

static void fpu_enable(void)
{
	/* Each core's CPACR is private; core0/Zephyr enables its own via
	 * runtime_init_per_core_enable_coprocessors() at boot, which never
	 * touches core1. The motion profile math needs the FPU (sqrtf), so
	 * core1 must enable it before core1_main() runs. */
	REG(M33_CPACR) |= M33_CPACR_CP10_BITS;
	__asm volatile("dsb");
	__asm volatile("isb");
}

void core1_reset_handler(void)
{
	for (uint32_t *p = &__bss_start__; p < &__bss_end__; p++) {
		*p = 0;
	}

	fpu_enable();
	core1_main();

	while (1) {
	}
}
