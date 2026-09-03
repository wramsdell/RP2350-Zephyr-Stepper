#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include <zephyr/sys/util.h>

#include <hardware/structs/sio.h>
#include <hardware/structs/psm.h>
#include <hardware/address_mapped.h>

#include "core1_launch.h"

/* Must match core1/linker.ld CORE1_RAM ORIGIN, and the &sram0 carve-out in
 * the app's devicetree overlay. */
#define CORE1_RAM_BASE ((uint32_t *)0x20062000u)

extern const uint8_t core1_blob[];
extern const size_t core1_blob_size;

/*
 * Direct reimplementation of the RP2350 bootrom's core1 launch protocol
 * (see pico_multicore's multicore_reset_core1()/multicore_launch_core1_raw()
 * for the reference this was checked against) against the raw SIO/PSM
 * registers, without pulling pico_multicore into the Zephyr build.
 */
static void core1_reset(void)
{
	io_rw_32 *frce_off = &psm_hw->frce_off;

	/*
	 * hw_set_alias()/hw_clear_alias() rely on typeof(), a GNU extension
	 * not available under Zephyr's strict -std=c17; use the _untyped
	 * variants directly instead (same pattern multicore_reset_core1()
	 * itself falls back to under __STRICT_ANSI__).
	 */
	*(io_rw_32 *)hw_set_alias_untyped(frce_off) = PSM_FRCE_OFF_PROC1_BITS;
	while (!(*frce_off & PSM_FRCE_OFF_PROC1_BITS)) {
	}

	*(io_rw_32 *)hw_clear_alias_untyped(frce_off) = PSM_FRCE_OFF_PROC1_BITS;

	/* Core1 drains its own FIFO on reset then pushes a 0 to confirm. */
	while (!(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS)) {
	}
	(void)sio_hw->fifo_rd;
}

int core1_launch(void)
{
	memcpy(CORE1_RAM_BASE, core1_blob, core1_blob_size);

	uint32_t vector_table = (uint32_t)CORE1_RAM_BASE;
	uint32_t sp = CORE1_RAM_BASE[0];
	uint32_t entry = CORE1_RAM_BASE[1];

	core1_reset();

	const uint32_t cmd_sequence[] = {0, 0, 1, vector_table, sp, entry};
	uint32_t seq = 0;

	while (seq < ARRAY_SIZE(cmd_sequence)) {
		uint32_t cmd = cmd_sequence[seq];

		if (cmd == 0) {
			while (sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS) {
				(void)sio_hw->fifo_rd;
			}
			__asm volatile("sev");
		}

		while (!(sio_hw->fifo_st & SIO_FIFO_ST_RDY_BITS)) {
		}
		sio_hw->fifo_wr = cmd;
		__asm volatile("sev");

		while (!(sio_hw->fifo_st & SIO_FIFO_ST_VLD_BITS)) {
		}
		uint32_t resp = sio_hw->fifo_rd;

		seq = (cmd == resp) ? seq + 1 : 0;
	}

	return 0;
}
