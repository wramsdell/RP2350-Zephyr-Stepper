#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/igmp.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(eth_id, LOG_LEVEL_INF);

#include "eth_id.h"

void set_unique_mac_address(void)
{
	struct net_if *iface = net_if_get_default();
	uint8_t hw_id[8];
	struct net_eth_addr mac = {0};
	ssize_t len;

	if (!iface) {
		LOG_WRN("No default interface; keeping the devicetree MAC");
		return;
	}

	len = hwinfo_get_device_id(hw_id, sizeof(hw_id));
	if (len < (ssize_t)sizeof(mac.addr)) {
		LOG_WRN("hwinfo_get_device_id() returned %d bytes; keeping the devicetree MAC",
			(int)len);
		return;
	}

	/* Use the last 6 of the 8 ID bytes (RP2350's flash unique ID, or
	 * DEVICE_ID+WAFER_ID - either way, stable and unique per chip; see
	 * hwinfo_rpi_pico.c). */
	memcpy(mac.addr, &hw_id[len - (ssize_t)sizeof(mac.addr)], sizeof(mac.addr));

	/* Locally administered, unicast - the same convention Zephyr's own
	 * net_eth_mac_load()/NET_ETH_MAC_RANDOM path uses for a generated
	 * (non-IEEE-assigned) MAC address. */
	mac.addr[0] &= ~0x01;
	mac.addr[0] |= 0x02;

	/*
	 * Deliberately NOT using net_mgmt(NET_REQUEST_ETHERNET_SET_MAC_ADDRESS,
	 * ...): that handler (subsys/net/l2/ethernet/ethernet_mgmt.c) refuses
	 * with -EACCES unless the interface is administratively down first
	 * (net_if_is_admin_up()) - and Zephyr brings interfaces up on their
	 * own well before main() runs, so satisfying that would require a
	 * net_if_down()/net_if_up() cycle here. That's actively harmful:
	 * net_if_down() unconditionally calls leave_mcast_all() and
	 * leave_ipv4_mcast_all(), stripping every multicast group membership
	 * the interface holds - including mDNS's 224.0.0.251 join, made once
	 * at its own early boot-time init. net_if_up()'s IGMP
	 * re-initialization does NOT restore application-level multicast
	 * subscriptions, and mDNS never re-joins on a later "interface up"
	 * event - so that cycle would silently and permanently kill all
	 * multicast reception (mDNS included) for the rest of the boot, while
	 * unicast traffic (DHCP, ARP, TCP/telnet) keeps working fine, making
	 * it a very quiet failure mode. (Discovered exactly this way: mDNS
	 * hostname resolution worked before this MAC-override code existed,
	 * and went silent - zero incoming multicast packets, even from
	 * unrelated devices' background chatter - once it started calling
	 * net_if_down()/up().)
	 *
	 * Calling the driver's set_config() and net_if_set_link_addr()
	 * directly - the same two calls ethernet_mgmt.c's handler makes
	 * internally - achieves the identical end result without ever
	 * touching admin state: set_config() reprograms the LAN9250's
	 * hardware RX address filter (skip this and the chip keeps silently
	 * dropping inbound unicast frames addressed to the new MAC);
	 * net_if_set_link_addr() updates the stack's view (what
	 * ARP/DHCP/set_unique_hostname() actually see). Neither call has any
	 * admin-up precondition of its own - that check only exists in
	 * ethernet_mgmt.c's wrapper.
	 */
	const struct device *dev = net_if_get_device(iface);
	const struct ethernet_api *api = dev->api;

	if (!api || !api->set_config) {
		LOG_WRN("Driver has no set_config(); keeping the devicetree MAC");
		return;
	}

	struct ethernet_config config = {0};

	memcpy(&config.mac_address, &mac, sizeof(config.mac_address));

	int ret = api->set_config(dev, ETHERNET_CONFIG_TYPE_MAC_ADDRESS, &config);

	if (ret) {
		LOG_WRN("Failed to set MAC address from hwinfo (%d); keeping the devicetree MAC",
			ret);
		return;
	}

	ret = net_if_set_link_addr(iface, mac.addr, sizeof(mac.addr), NET_LINK_ETHERNET);
	if (ret) {
		LOG_WRN("net_if_set_link_addr() failed (%d)", ret);
		return;
	}

	LOG_INF("MAC address set from hwinfo: %02x:%02x:%02x:%02x:%02x:%02x", mac.addr[0],
		mac.addr[1], mac.addr[2], mac.addr[3], mac.addr[4], mac.addr[5]);
}

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
