#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/igmp.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(eth_id, LOG_LEVEL_INF);

#include "eth_id.h"

void force_multicast_rejoin(struct net_if *iface, const char *addr_str)
{
	struct net_in_addr addr;

	/*
	 * A subsystem's own SYS_INIT-/boot-time IGMP join can run long
	 * before the LAN9250's PHY link finishes negotiation (~2s
	 * post-boot). eth_lan9250.c's lan9250_tx() writes the IGMP report
	 * straight to the LAN9250's TX FIFO over SPI and returns success
	 * purely based on that SPI transaction completing - it never checks
	 * net_if_is_carrier_ok() - so with no physical link yet, the report
	 * is silently discarded by the hardware while net_ipv4_igmp_join()
	 * still sees ret == 0 and marks the group "joined" (confirmed via
	 * the `net ipv4` shell command).
	 *
	 * That false-positive "joined" state then defeats both of Zephyr's
	 * built-in recovery paths for this exact case: net_if.c's
	 * rejoin_ipv4_mcast_groups() (run when the interface later goes
	 * operationally up for real) and any subsystem's own
	 * NET_EVENT_IF_UP handler both skip any group whose
	 * net_if_ipv4_maddr_is_joined() is already true - so no genuine
	 * report is ever retried.
	 *
	 * Worse, net_ipv4_igmp_join()'s net_if_ipv4_maddr_add() refcounts
	 * the group entry on *every* join call regardless of is_joined - and
	 * e.g. mdns_responder joins twice on its own (the SYS_INIT boot
	 * join, then again from its NET_EVENT_IF_UP handler once carrier
	 * really comes up), leaving a refcount of 2. net_ipv4_igmp_leave()
	 * only decrements that count; net_if_ipv4_maddr_rm() treats "count
	 * still > 0" as "still in use" and returns without ever clearing
	 * is_joined or sending a Leave report. So a single leave() call here
	 * isn't enough - it silently no-ops, and the following join() then
	 * also no-ops (is_joined is still true). Draining the refcount to
	 * zero first (leave() in a loop until the address is actually gone,
	 * i.e. -ENOENT) is what actually clears is_joined so the final
	 * join() below performs a real send.
	 */
	net_addr_pton(NET_AF_INET, addr_str, &addr);

	while (net_ipv4_igmp_leave(iface, &addr) == 0) {
		/* keep draining refcount until the group is fully removed */
	}

	int ret = net_ipv4_igmp_join(iface, &addr, NULL);

	if (ret < 0) {
		LOG_WRN("Forced multicast rejoin of %s failed (%d)", addr_str, ret);
	} else {
		LOG_INF("Forced multicast rejoin of %s", addr_str);
	}
}

void ptp_multicast_rejoin(struct net_if *iface)
{
	/* General/event PTP messages (Sync, Announce, Delay_Req, ...). */
	force_multicast_rejoin(iface, "224.0.1.129");

	/* Peer-delay mechanism messages (Pdelay_Req/Resp). */
	force_multicast_rejoin(iface, "224.0.0.107");
}

#define MULTICAST_MEMBERSHIP_REFRESH_INTERVAL K_SECONDS(30)

static struct net_if *multicast_refresh_iface;
static struct k_work_delayable multicast_refresh_work;

static void multicast_membership_refresh_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Real IGMP membership reports, sent periodically rather than only
	 * once at boot - this network's switch was confirmed (via a live
	 * capture) to send zero IGMP query traffic at all, meaning nothing
	 * ever prompts a renewed report on its own. Without one, an IGMP-
	 * snooping switch with no active querier has no way to distinguish
	 * "still a member" from "left silently", and relies purely on an
	 * internal aging timeout with nothing to refresh it - matching the
	 * intermittent, worsening-over-time multicast reception observed
	 * between boards once three of them were competing for the switch's
	 * forwarding table. net_ipv4_igmp_join() can't be reused for this -
	 * it's a deliberate no-op once already joined - so this uses the new
	 * net_ipv4_igmp_resend_reports() instead (see its comment in the
	 * Zephyr tree, subsys/net/ip/igmp.c).
	 */
	(void)net_ipv4_igmp_resend_reports(multicast_refresh_iface);

	k_work_reschedule(&multicast_refresh_work, MULTICAST_MEMBERSHIP_REFRESH_INTERVAL);
}

void multicast_membership_refresh_start(struct net_if *iface)
{
	if (multicast_refresh_iface != NULL) {
		/* Already running - only one interface's worth needed. */
		return;
	}

	multicast_refresh_iface = iface;
	k_work_init_delayable(&multicast_refresh_work, multicast_membership_refresh_handler);
	k_work_reschedule(&multicast_refresh_work, MULTICAST_MEMBERSHIP_REFRESH_INTERVAL);
}
