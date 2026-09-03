#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/ethernet_mgmt.h>
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

void enable_multicast_rx(void)
{
	struct net_if *iface = net_if_get_default();

	if (!iface) {
		LOG_WRN("No default interface; multicast RX not enabled");
		return;
	}

	/*
	 * drivers/ethernet/eth_lan9250.c's lan9250_configure() programs
	 * HMAC_CR with only PADSTR|TXEN|RXEN|FDPX - neither MCPAS (pass all
	 * multicast) nor a populated+enabled hash filter (HPFILT). Per the
	 * LAN9250 datasheet's HMAC_CR description, with both of those clear
	 * the chip's own RX address filter silently drops every frame whose
	 * destination MAC isn't an exact match for our unicast address or
	 * the broadcast address - including all multicast traffic (mDNS,
	 * IGMP queries, everything on 01:00:5e:xx:xx:xx). This is a hardware
	 * filter, entirely independent of net_if's IGMP/multicast-group
	 * bookkeeping (net_ipv4_igmp_join(), rejoin_ipv4_mcast_groups(),
	 * etc.) - joining a group at the IP layer changes nothing here, so
	 * no amount of IGMP-side fixing can make multicast RX work on its
	 * own.
	 *
	 * A one-line patch to the vendored driver (adding MCPAS to that
	 * write) was tried and works, but was reverted in favor of this
	 * app-level approach: patching drivers/ethernet/eth_lan9250.c lives
	 * outside this repo, in the west-managed ~/zephyrproject/zephyr
	 * checkout, so it isn't tracked by git here and would silently
	 * vanish on a `west update`. Full promiscuous mode
	 * (ETHERNET_CONFIG_TYPE_PROMISC_MODE) is the only RX-filter toggle
	 * this driver actually implements - there's no
	 * ETHERNET_CONFIG_TYPE_FILTER handler, and the driver doesn't
	 * advertise ETHERNET_HW_FILTERING, so the generic per-group hardware
	 * filter hook (ethernet.c's ethernet_mcast_monitor_cb()) never even
	 * calls into this driver - so this is the only lever available
	 * without touching the driver. The IP/UDP layers above still
	 * correctly filter out traffic nothing is listening for, so this
	 * doesn't affect what reaches application sockets, just what the
	 * hardware hands up to be filtered.
	 *
	 * This does mean every frame on the LAN gets pulled over the
	 * LAN9250's SPI bus and examined, not just ones addressed to us -
	 * on a busy LAN this exhausted the default net_pkt RX buffer pool
	 * badly enough to break basic reachability (ARP failures,
	 * "Could not allocate rx buffer" in the log). Fixed by sizing
	 * CONFIG_NET_PKT_RX_COUNT/CONFIG_NET_BUF_RX_COUNT generously in
	 * prj.conf rather than trying to reduce traffic volume - see the
	 * comment there.
	 *
	 * net_eth_promisc_mode() -> NET_REQUEST_ETHERNET_SET_PROMISC_MODE
	 * has no admin-up precondition (unlike NET_REQUEST_ETHERNET_SET_MAC_
	 * ADDRESS - see the comment in set_unique_mac_address(), above), so
	 * this is safe to call here without any net_if_down()/up() cycle.
	 */
	int ret = net_eth_promisc_mode(iface, true);

	if (ret) {
		LOG_WRN("Failed to enable promiscuous mode for multicast RX (%d)", ret);
	} else {
		LOG_INF("Promiscuous mode enabled (required for multicast RX on this driver)");
	}
}
