/*
 * ptp_clock_driver_api glue for the LAN9250's IEEE 1588 hardware clock -
 * Phase 3 of this project's PTP work, see THEORY_OF_OPERATION.md's "IEEE
 * 1588 / PTP" section.
 *
 * Not devicetree-backed: there's no separate physical PTP clock block to
 * bind to, just this project's single LAN9250 instance's own 1588 unit
 * (already brought up by eth_lan9250.c's lan9250_1588_init()). Follows
 * the same pattern Zephyr's own drivers/ethernet/eth_stm32_hal_ptp.c uses
 * for an internal-to-the-MAC PTP clock: a plain DEVICE_DEFINE() with no
 * devicetree node, whose init function stashes a pointer to itself into
 * the Ethernet driver's own runtime data (lan9250_runtime::ptp_clock) so
 * eth_lan9250.c's .get_ptp_clock callback has something to return.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/ptp_clock.h>
#include <zephyr/net/ptp_time.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/net/ethernet.h>

#include "eth_lan9250_priv.h"
#include "eth_lan9250_ptp.h"

static const struct device *const lan9250_dev = DEVICE_DT_GET(DT_NODELABEL(lan9250));

static int lan9250_ptp_clock_api_set(const struct device *dev, struct net_ptp_time *tm)
{
	ARG_UNUSED(dev);

	/* The LAN9250's hardware seconds counter is 32 bits, not the 48
	 * IEEE 1588-2008 specifies (see THEORY_OF_OPERATION.md - the
	 * datasheet notes 32 bits is sufficient for the 136-year rollover
	 * this gives, and one-step Sync egress insertion separately
	 * maintains the upper 16 bits via TX_ONE_STEP_SYNC_SEC). Truncate
	 * rather than reject: a full 48-bit second value won't be seen in
	 * practice for a very long time.
	 */
	return lan9250_ptp_clock_set(lan9250_dev, (uint32_t)tm->second, tm->nanosecond);
}

static int lan9250_ptp_clock_api_get(const struct device *dev, struct net_ptp_time *tm)
{
	uint32_t sec, ns;
	int ret;

	ARG_UNUSED(dev);

	ret = lan9250_ptp_clock_read(lan9250_dev, &sec, &ns, NULL);
	if (ret < 0) {
		return ret;
	}

	tm->second = sec;
	tm->nanosecond = ns;

	return 0;
}

static int lan9250_ptp_clock_api_adjust(const struct device *dev, int increment)
{
	ARG_UNUSED(dev);

	return lan9250_ptp_clock_adjust(lan9250_dev, (int32_t)increment);
}

static int lan9250_ptp_clock_api_rate_adjust(const struct device *dev, double ratio)
{
	ARG_UNUSED(dev);

	return lan9250_ptp_clock_rate_adjust(lan9250_dev, ratio);
}

static DEVICE_API(ptp_clock, lan9250_ptp_clock_api) = {
	.set = lan9250_ptp_clock_api_set,
	.get = lan9250_ptp_clock_api_get,
	.adjust = lan9250_ptp_clock_api_adjust,
	.rate_adjust = lan9250_ptp_clock_api_rate_adjust,
};

static int lan9250_ptp_clock_init(const struct device *port)
{
	struct lan9250_runtime *eth_data;

	if (!device_is_ready(lan9250_dev)) {
		return -ENODEV;
	}

	eth_data = lan9250_dev->data;
	eth_data->ptp_clock = port;

	return 0;
}

/* No devicetree node to size a priority/instance count from - this
 * project has exactly one LAN9250, so this is a plain singleton. Priority
 * must run after CONFIG_ETH_INIT_PRIORITY=60 (lan9250_dev->data must
 * already be the real, initialized runtime struct - it is, from link
 * time, since it's a plain static struct, but ordering after the
 * Ethernet driver's own init keeps this consistent with when the 1588
 * hardware itself is actually enabled). Hardcoded as a plain literal,
 * not an arithmetic expression on CONFIG_ETH_INIT_PRIORITY - the
 * priority is used in preprocessor token-pasting to build the init-level
 * section name, which requires a literal token, not an expression; using
 * one broke the link ("Undefined initialization levels used").
 */
DEVICE_DEFINE(lan9250_ptp_clock, PTP_CLOCK_NAME, lan9250_ptp_clock_init, NULL, NULL, NULL,
	     POST_KERNEL, 61, &lan9250_ptp_clock_api);
