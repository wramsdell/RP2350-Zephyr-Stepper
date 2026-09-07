#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_dhcpv4_client_sample, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/linker/sections.h>
#include <errno.h>
#include <stdio.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_context.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/device.h>
#include <zephyr/net/phy.h>

#include "core1_launch.h"
#include "stepper_shell.h"
#include "mdns_service.h"
#include "eth_id.h"
#include "ptp_shell.h"

#define DHCP_OPTION_NTP (42)

static uint8_t ntp_server[4];

static struct net_mgmt_event_callback mgmt_cb;

static struct net_dhcpv4_option_callback dhcp_cb;

static void start_dhcpv4_client(struct net_if *iface, void *user_data)
{
	ARG_UNUSED(user_data);

	LOG_INF("Start on %s: index=%d", net_if_get_device(iface)->name,
		net_if_get_by_iface(iface));
	net_dhcpv4_start(iface);
}

static void handler(struct net_mgmt_event_callback *cb,
		    uint64_t mgmt_event,
		    struct net_if *iface)
{
	int i = 0;

	if (mgmt_event == NET_EVENT_IF_UP) {
		/* Restart DHCP fresh on every link-up transition, not just at
		 * boot. Confirmed via a real link-flap event (a cold-spray
		 * disturbance test briefly perturbing the LAN9250's PHY): the
		 * board came back with its interface operationally up but no
		 * valid IPv4 address at all (net_ctx: "src addr is
		 * unspecified", every PTP send failing with
		 * "ptp_transport: Failed to send message") and never
		 * recovered on its own - the DHCP client's own state machine
		 * doesn't appear to notice a link bounce and re-acquire on
		 * its own. net_dhcpv4_stop() before net_dhcpv4_start() forces
		 * a genuinely fresh acquisition rather than trusting whatever
		 * internal state (e.g. still considering itself BOUND) the
		 * client was left in - calling stop() on a client that was
		 * never started, or already stopped, is a safe no-op, so
		 * this is fine to also run unconditionally on the very first
		 * boot-time link-up alongside main()'s own initial
		 * start_dhcpv4_client() call.
		 */
		net_dhcpv4_stop(iface);
		net_dhcpv4_start(iface);
		return;
	}

	if (mgmt_event != NET_EVENT_IPV4_ADDR_ADD) {
		return;
	}

	/* A bound IPv4 address means DHCP just completed a real Ethernet
	 * round trip, so the link is confirmed up - unlike at boot, when
	 * mDNS's own early multicast join can silently fail in hardware
	 * before the PHY has linked. See mdns_force_multicast_rejoin(). */
	mdns_force_multicast_rejoin(iface);

	/* ptp_multicast_rejoin() is deliberately NOT called here anymore. It
	 * was based on the false assumption that CONFIG_PTP has no boot-time
	 * multicast join of its own - it does (transport_join_multicast(),
	 * confirmed to run right after link-up and succeed). Draining and
	 * rejoining the group out from under the PTP library's own
	 * already-good membership was corrupting it, which is why role
	 * negotiation (BMCA) never converged between boards: both stayed
	 * self-elected masters, each barely receiving the other's
	 * Announce/Sync traffic despite it being confirmed present on the
	 * wire. */

	/* This network's switch sends zero IGMP query traffic (confirmed via
	 * a live capture) - no active querier, so nothing else ever prompts
	 * a renewed membership report. Without periodic refreshing, the
	 * switch's IGMP-snooping forwarding entries can silently age out,
	 * which is why multi-board PTP reception was observed to work for a
	 * while then intermittently drop, worsening as more boards competed
	 * for the switch's table. See multicast_membership_refresh_start()'s
	 * comment (eth_id.c/h) for the full story. */
	multicast_membership_refresh_start(iface);

	for (i = 0; i < NET_IF_MAX_IPV4_ADDR; i++) {
		char buf[NET_IPV4_ADDR_LEN];

		if (iface->config.ip.ipv4->unicast[i].ipv4.addr_type !=
							NET_ADDR_DHCP) {
			continue;
		}

		LOG_INF("   Address[%d]: %s", net_if_get_by_iface(iface),
			net_addr_ntop(NET_AF_INET,
			    &iface->config.ip.ipv4->unicast[i].ipv4.address.in_addr,
						  buf, sizeof(buf)));
		LOG_INF("    Subnet[%d]: %s", net_if_get_by_iface(iface),
			net_addr_ntop(NET_AF_INET,
				       &iface->config.ip.ipv4->unicast[i].netmask,
				       buf, sizeof(buf)));
		LOG_INF("    Router[%d]: %s", net_if_get_by_iface(iface),
			net_addr_ntop(NET_AF_INET,
						 &iface->config.ip.ipv4->gw,
						 buf, sizeof(buf)));
		LOG_INF("Lease time[%d]: %u seconds", net_if_get_by_iface(iface),
			iface->config.dhcpv4.lease_time);
	}
}

static void option_handler(struct net_dhcpv4_option_callback *cb,
			   size_t length,
			   enum net_dhcpv4_msg_type msg_type,
			   struct net_if *iface)
{
	char buf[NET_IPV4_ADDR_LEN];

	LOG_INF("DHCP Option %d: %s", cb->option,
		net_addr_ntop(NET_AF_INET, cb->data, buf, sizeof(buf)));
}

struct phy_data {
    const struct device *dev;
    struct phy_link_state state;
    // ... other fields we don't care about
};

int main(void)
{
	LOG_INF("Run dhcpv4 client");

	/* The unique MAC address itself is now set much earlier, inside
	 * eth_lan9250.c's lan9250_init() - see
	 * lan9250_load_unique_mac_address()'s comment for why that had to
	 * move out of main() (gPTP's clockIdentity, computed during kernel
	 * boot before main() runs, needs it that early). This just reads
	 * the now-already-unique link address back out to derive the
	 * hostname suffix.
	 */
	set_unique_hostname();

	if (core1_launch()) {
		LOG_ERR("Failed to launch core1");
	}

	if (stepper_mbox_init()) {
		LOG_ERR("Failed to init core1 mailbox");
	}

	net_mgmt_init_event_callback(&mgmt_cb, handler,
				     NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IF_UP);
	net_mgmt_add_event_callback(&mgmt_cb);

	net_dhcpv4_init_option_callback(&dhcp_cb, option_handler,
					DHCP_OPTION_NTP, ntp_server,
					sizeof(ntp_server));

	net_dhcpv4_add_option_callback(&dhcp_cb);

	net_if_foreach(start_dhcpv4_client, NULL);

	/* Arm the 1PPS scope-observation output at boot instead of requiring
	 * the `ptp pps` shell command after every reboot/reflash - see
	 * lan9250_1588_pps_enable()'s own comment for the hardware details.
	 * Two separate waits, for two separate reasons:
	 *
	 * 1. A fixed hardware settle delay: arming this immediately at the
	 *    top of main() (~30ms after kernel boot) reliably fails silently
	 *    - the LAN9250's own internal reset/auto-load sequence apparently
	 *    hasn't fully settled that early, even though device_is_ready()
	 *    and the SPI writes themselves both report success.
	 *
	 * 2. ptp_pps_arm_when_synced() then additionally waits for PTP to
	 *    consider this port's clock trustworthy before actually arming -
	 *    otherwise the Clock Target gets seeded from whatever the local
	 *    clock happens to read mid-negotiation, silently baking in
	 *    however far off that was as a fixed offset between this board's
	 *    pulses and its peer's (see ptp_get_port_sync_state()'s doc
	 *    comment, zephyr/net/ptp.h). Falls back to arming anyway past its
	 *    own timeout, for a standalone board with no peer to sync to.
	 */
	k_sleep(K_SECONDS(15));
	{
		const struct device *const lan9250_dev = DEVICE_DT_GET(DT_NODELABEL(lan9250));

		ptp_pps_arm_when_synced(lan9250_dev, 60000);
	}

	while(1)
	{
//		printf("Hello World! %s\n", CONFIG_BOARD_TARGET);
		k_sleep(K_MSEC(1000));
	}


	return 0;
}