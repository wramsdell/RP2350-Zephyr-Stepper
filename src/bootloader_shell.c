#include <zephyr/shell/shell.h>
#include <zephyr/sys/sys_io.h>

#include <pico/bootrom.h>

/*
 * Reboots straight into the RP2350's USB mass-storage UF2 bootloader
 * (RPI-RP2), the same mode BOOTSEL does at power-on - so reflashing over
 * USB no longer needs physically holding the button. rom_reset_usb_boot()
 * is a genuine RP2350 bootrom ROM call (modules/hal_rpi_pico's
 * pico_bootrom, already vendored and built as part of this project); it
 * never returns.
 */
static int cmd_bootloader(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Rebooting into USB bootloader (RPI-RP2)...");

	rom_reset_usb_boot(0, 0);

	return 0;
}

SHELL_CMD_REGISTER(bootloader, NULL,
		    "Reboot into the RP2350 USB mass-storage bootloader (RPI-RP2) for reflashing",
		    cmd_bootloader);

/*
 * Plain reset back into the currently flashed application - not the
 * bootloader. Zephyr's own sys_reboot()/`kernel reboot` isn't usable here:
 * it needs an SoC-specific sys_arch_reboot() implementation, and none
 * exists for RP2350 in this Zephyr version (CONFIG_REBOOT=y fails to
 * link).
 *
 * The Pico SDK's own watchdog_reboot() would be the idiomatic way to do
 * this, but its home library (hal_rpi_pico's hardware_watchdog, a
 * pico_add_subdirectory() CMake target like pico_bootrom above) isn't
 * actually linked into this Zephyr build unless something else already
 * depends on it - unlike pico_bootrom, nothing here does, and there's no
 * clean way to opt in from an app's own CMakeLists.txt (a
 * target_link_libraries(app PRIVATE hardware_watchdog) resolves to
 * "cannot find -lhardware_watchdog": the CMake target exists during
 * configure, but Zephyr's own build glue doesn't expose it as something
 * an app can link against directly). So this replicates the delay_ms=0
 * path of watchdog_reboot(0, 0, 0) directly via raw register access
 * instead - the same style core1/regs.h and drivers/eth_lan9250/ already
 * use elsewhere in this project, and genuinely simpler here too: register
 * offsets/bits cross-checked against
 * hal_rpi_pico/src/rp2_common/hardware_watchdog/watchdog.c's
 * _watchdog_enable() (the real implementation watchdog_reboot() calls
 * into for this exact delay_ms=0 case).
 */
#define WATCHDOG_BASE			0x400d8000u
#define WATCHDOG_CTRL_OFFSET		0x00u
#define WATCHDOG_CTRL_TRIGGER_BITS	0x80000000u /* self-clearing: write 1 to reset now */
#define WATCHDOG_CTRL_ENABLE_BITS	0x40000000u
#define WATCHDOG_SCRATCH4_OFFSET	0x1cu

#define PSM_BASE			0x40018000u
#define PSM_WDSEL_OFFSET		0x08u
#define PSM_WDSEL_BITS			0x01ffffffu
#define PSM_WDSEL_ROSC_BITS		0x00000004u
#define PSM_WDSEL_XOSC_BITS		0x00000008u

static int cmd_reset(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Resetting...");

	/* Disable watchdog before reconfiguring it */
	sys_clear_bits(WATCHDOG_BASE + WATCHDOG_CTRL_OFFSET, WATCHDOG_CTRL_ENABLE_BITS);

	/* Reset everything the watchdog can reset except the oscillators */
	sys_set_bits(PSM_BASE + PSM_WDSEL_OFFSET,
		     PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));

	/* pc=0: reboot through the normal flash boot path, not a jump to a
	 * custom vector and not the USB bootloader.
	 */
	sys_write32(0, WATCHDOG_BASE + WATCHDOG_SCRATCH4_OFFSET);

	/* Self-clearing trigger bit - fires the reset immediately */
	sys_set_bits(WATCHDOG_BASE + WATCHDOG_CTRL_OFFSET, WATCHDOG_CTRL_TRIGGER_BITS);

	for (;;) {
		/* Should never get here - reset fires above */
	}

	return 0;
}

SHELL_CMD_REGISTER(reset, NULL, "Reset the board", cmd_reset);
