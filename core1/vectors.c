#include <stdint.h>

extern uint32_t __core1_stack_top;
void core1_reset_handler(void);

static void core1_default_handler(void)
{
	while (1) {
	}
}

/*
 * Minimal Cortex-M33 vector table. Only entries [0] (initial SP) and [1]
 * (reset handler) are consumed by the core1 launch handshake, but the rest
 * of the core exception table is filled in with a spin loop so a stray
 * fault doesn't jump into garbage memory.
 */
__attribute__((section(".vectors"), used))
static const void *const vector_table[16] = {
	(void *)&__core1_stack_top, /* [0]  initial SP */
	(void *)core1_reset_handler, /* [1]  Reset */
	(void *)core1_default_handler, /* [2]  NMI */
	(void *)core1_default_handler, /* [3]  HardFault */
	(void *)core1_default_handler, /* [4]  MemManage */
	(void *)core1_default_handler, /* [5]  BusFault */
	(void *)core1_default_handler, /* [6]  UsageFault */
	(void *)core1_default_handler, /* [7]  SecureFault */
	0, /* [8]  reserved */
	0, /* [9]  reserved */
	0, /* [10] reserved */
	(void *)core1_default_handler, /* [11] SVCall */
	(void *)core1_default_handler, /* [12] DebugMon */
	0, /* [13] reserved */
	(void *)core1_default_handler, /* [14] PendSV */
	(void *)core1_default_handler, /* [15] SysTick */
};
