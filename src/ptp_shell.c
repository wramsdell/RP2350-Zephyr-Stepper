#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/device.h>

#include "../drivers/eth_lan9250/eth_lan9250_ptp.h"

/*
 * Phase 1 validation command for the LAN9250's 1588 PTP hardware clock
 * (see drivers/eth_lan9250/eth_lan9250.c's lan9250_1588_init() and
 * THEORY_OF_OPERATION.md's "IEEE 1588 / PTP" section) - reads the clock
 * twice, ~200ms apart, so the reported delta can be sanity-checked
 * against real elapsed time by eye.
 */
static const struct device *const lan9250_dev = DEVICE_DT_GET(DT_NODELABEL(lan9250));

static int cmd_ptp_clock(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t sec1, ns1, sec2, ns2;
	int64_t delta_ns;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(lan9250_dev)) {
		shell_error(sh, "lan9250 device not ready");
		return -ENODEV;
	}

	ret = lan9250_ptp_clock_read(lan9250_dev, &sec1, &ns1, NULL);
	if (ret < 0) {
		shell_error(sh, "1588 clock read failed (%d)", ret);
		return ret;
	}

	shell_print(sh, "1588 clock: %u.%09u", sec1, ns1);

	k_msleep(200);

	ret = lan9250_ptp_clock_read(lan9250_dev, &sec2, &ns2, NULL);
	if (ret < 0) {
		shell_error(sh, "1588 clock read failed (%d)", ret);
		return ret;
	}

	shell_print(sh, "1588 clock: %u.%09u (~200ms later)", sec2, ns2);

	delta_ns = ((int64_t)sec2 - (int64_t)sec1) * 1000000000LL +
		   ((int64_t)ns2 - (int64_t)ns1);

	shell_print(sh, "Delta: %lld ns (expect roughly +200000000)", delta_ns);

	return 0;
}

/*
 * Phase 2 validation command: arm the 1PPS output on GPIO1 (pin 46 -
 * this board's LED1/GPIO1/TDI/MNGT1, unused by the RJ45 LEDs, only a
 * 10k pull-down for the MNGT1 strap). See
 * lan9250_1588_pps_enable()'s definition for the hardware reasoning
 * (push/pull output, 100ns pulse mode, auto-repeating Clock Target).
 */
static int cmd_ptp_pps(const struct shell *sh, size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(lan9250_dev)) {
		shell_error(sh, "lan9250 device not ready");
		return -ENODEV;
	}

	ret = lan9250_1588_pps_enable(lan9250_dev);
	if (ret < 0) {
		shell_error(sh, "Failed to arm 1PPS output (%d)", ret);
		return ret;
	}

	shell_print(sh, "1PPS output armed on GPIO1 (pin 46) - 100ns pulse, active high, "
			"once per second");

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ptp,
	SHELL_CMD(clock, NULL,
		  "Read the LAN9250's 1588 PTP hardware clock twice, ~200ms apart",
		  cmd_ptp_clock),
	SHELL_CMD(pps, NULL,
		  "Arm a 1PPS output on GPIO1 (pin 46) for scope observation",
		  cmd_ptp_pps),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ptp, &sub_ptp, "IEEE 1588 PTP hardware clock", NULL);
