#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/ptp.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include "../drivers/eth_lan9250/eth_lan9250_ptp.h"
#include "ptp_shell.h"

LOG_MODULE_REGISTER(ptp_shell, LOG_LEVEL_INF);

int ptp_pps_arm_when_synced(const struct device *lan9250_dev, int max_wait_ms)
{
	struct net_if *iface;
	bool is_time_transmitter = false, is_synced = false;
	int64_t deadline;
	int ret;

	if (!device_is_ready(lan9250_dev)) {
		LOG_ERR("lan9250 device not ready, cannot arm 1PPS output");
		return -ENODEV;
	}

	iface = net_if_lookup_by_dev(lan9250_dev);
	deadline = k_uptime_get() + max_wait_ms;

	while (k_uptime_get() < deadline) {
		if (iface &&
		    ptp_get_port_sync_state(iface, &is_time_transmitter, &is_synced) == 0 &&
		    (is_time_transmitter || is_synced)) {
			break;
		}
		k_sleep(K_MSEC(500));
	}

	if (!is_time_transmitter && !is_synced) {
		LOG_WRN("Arming 1PPS output without confirmed PTP sync "
			"(is_time_transmitter=%d is_synced=%d after %d ms) - alignment "
			"to any peer is not guaranteed",
			is_time_transmitter, is_synced, max_wait_ms);
	}

	ret = lan9250_1588_pps_enable(lan9250_dev);
	if (ret < 0) {
		LOG_ERR("Failed to arm 1PPS output (%d)", ret);
	}

	return ret;
}

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
	struct net_if *iface;
	bool is_time_transmitter = false, is_synced = false;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(lan9250_dev)) {
		shell_error(sh, "lan9250 device not ready");
		return -ENODEV;
	}

	iface = net_if_lookup_by_dev(lan9250_dev);
	if (iface &&
	    ptp_get_port_sync_state(iface, &is_time_transmitter, &is_synced) == 0 &&
	    !is_time_transmitter && !is_synced) {
		shell_print(sh, "Waiting up to 30s for PTP sync before arming "
				"(is_time_transmitter=%d is_synced=%d)...",
			    is_time_transmitter, is_synced);
	}

	ret = ptp_pps_arm_when_synced(lan9250_dev, 30000);
	if (ret < 0) {
		shell_error(sh, "Failed to arm 1PPS output (%d)", ret);
		return ret;
	}

	shell_print(sh, "1PPS output armed on GPIO1 (pin 46) - 100ns pulse, active high, "
			"once per second");

	return 0;
}

/*
 * Phase 4 TX validation command: builds and sends a minimal, real Layer-2
 * PTP Sync frame (EtherType 0x88F7, IEEE 1588-2008 common header) through
 * the normal net_if TX path - the same path Zephyr's own LLDP sender uses
 * for its raw Ethernet frames (subsys/net/l2/ethernet/lldp/lldp.c) - so it
 * actually reaches lan9250_tx() and exercises
 * lan9250_1588_tx_timestamp_check() the same way a genuine outbound PTP
 * message would, no shortcuts into driver internals. Confirmed working:
 * see THEORY_OF_OPERATION.md's "Hardware TX packet timestamping" section.
 *
 * Destined to the well-known Layer-2 (non-peer-delay) PTP multicast
 * address so a real capture on a mirrored switch port - the same setup
 * used for RX validation, see THEORY_OF_OPERATION.md's "Hardware RX
 * packet timestamping" section - can confirm this frame actually left the
 * wire. Watch the console (CONFIG_ETHERNET_LOG_LEVEL_DBG=y, already set
 * in prj.conf) for lan9250_1588_tx_timestamp_check()'s own LOG_DBG output
 * to see the hardware egress capture get matched.
 *
 * Only messageType (byte 0), messageLength (bytes 2-3), and sequenceId
 * (bytes 30-31) of the 32-byte PTP common header matter here - the rest
 * is left zeroed. messageType 0 (Sync) is one of the four types
 * LAN9250_1588_PTP_MESSAGE_EN_DEFAULT enables for hardware timestamping
 * (eth_lan9250_priv.h) - matches what RX-side validation used against
 * real ptp4l traffic, for consistency. messageLength must be a real,
 * non-zero value (44, matching a genuine Sync message's length, even
 * though this frame's payload only actually contains the first 32 bytes)
 * - the LAN9250 silently never records an egress timestamp for a frame
 * whose messageLength is 0, a requirement not listed among the
 * datasheet's documented TX egress-recording gating conditions (section
 * 14.2.2.3: messageType enable, versionPTP, domain, alt-master,
 * FCS/checksum) but real nonetheless.
 *
 * UDP/IPv4-framed PTP (dst port 319, 224.0.1.129) was also tried and did
 * NOT produce a capture even with messageLength fixed - left as a known,
 * unresolved gap; this command validates the L2 path only.
 */
static int cmd_ptp_txtest(const struct shell *sh, size_t argc, char **argv)
{
	static const struct net_eth_addr l2_ptp_mcast = {
		{ 0x01, 0x1b, 0x19, 0x00, 0x00, 0x00 }
	};
	static uint16_t seq_id;
	struct net_if *iface;
	struct net_pkt *pkt;
	uint8_t ptp_payload[32] = {0};

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	if (!device_is_ready(lan9250_dev)) {
		shell_error(sh, "lan9250 device not ready");
		return -ENODEV;
	}

	iface = net_if_lookup_by_dev(lan9250_dev);
	if (!iface) {
		shell_error(sh, "No net_if bound to lan9250 device");
		return -ENODEV;
	}

	ptp_payload[0] = 0x00; /* messageType = Sync (transportSpecific nibble = 0) */
	ptp_payload[1] = 0x02; /* versionPTP = 2, matching TX_TIMESTAMP_CONFIG's default */
	sys_put_be16(44, &ptp_payload[2]); /* messageLength - see comment above */
	sys_put_be16(seq_id, &ptp_payload[30]);

	pkt = net_pkt_alloc_with_buffer(iface, sizeof(ptp_payload), NET_AF_UNSPEC, 0,
					K_MSEC(100));
	if (!pkt) {
		shell_error(sh, "Failed to allocate net_pkt");
		return -ENOMEM;
	}

	if (net_pkt_write(pkt, ptp_payload, sizeof(ptp_payload)) < 0) {
		net_pkt_unref(pkt);
		shell_error(sh, "Failed to write PTP payload");
		return -EIO;
	}

	net_pkt_set_ll_proto_type(pkt, NET_ETH_PTYPE_PTP);
	(void)net_linkaddr_copy(net_pkt_lladdr_src(pkt), net_if_get_link_addr(iface));
	(void)net_linkaddr_set(net_pkt_lladdr_dst(pkt), l2_ptp_mcast.addr,
			       sizeof(l2_ptp_mcast.addr));

	shell_print(sh, "Sending L2 PTP Sync frame (seq=%u) to "
			"%02x:%02x:%02x:%02x:%02x:%02x",
		    seq_id, l2_ptp_mcast.addr[0], l2_ptp_mcast.addr[1],
		    l2_ptp_mcast.addr[2], l2_ptp_mcast.addr[3], l2_ptp_mcast.addr[4],
		    l2_ptp_mcast.addr[5]);

	if (net_if_try_send_data(iface, pkt, K_MSEC(100)) == NET_DROP) {
		net_pkt_unref(pkt);
		shell_error(sh, "Send failed");
		return -EIO;
	}

	shell_print(sh, "Sent - watch the console for "
			"lan9250_1588_tx_timestamp_check()'s output");

	seq_id++;

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_ptp,
	SHELL_CMD(clock, NULL,
		  "Read the LAN9250's 1588 PTP hardware clock twice, ~200ms apart",
		  cmd_ptp_clock),
	SHELL_CMD(pps, NULL,
		  "Arm a 1PPS output on GPIO1 (pin 46) for scope observation",
		  cmd_ptp_pps),
	SHELL_CMD(txtest, NULL,
		  "Send a real L2 PTP Sync frame to test hardware TX timestamping",
		  cmd_ptp_txtest),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ptp, &sub_ptp, "IEEE 1588 PTP hardware clock", NULL);
