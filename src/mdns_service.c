#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/hostname.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/igmp.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mdns_service, LOG_LEVEL_INF);

#include "mdns_service.h"

/*
 * "<CONFIG_NET_HOSTNAME><2 hex bytes>\0" - e.g. "stepper0203". Shared by
 * both the mDNS hostname (set_unique_hostname(), below) and the DNS-SD
 * service instance name registered at the bottom of this file, so they
 * can't drift out of sync. Starts as just CONFIG_NET_HOSTNAME so there's a
 * sane fallback if set_unique_hostname() is never called or the interface
 * has no link address yet.
 */
static char stepper_id[sizeof(CONFIG_NET_HOSTNAME) - 1 + 4 + 1] = CONFIG_NET_HOSTNAME;

void set_unique_hostname(void)
{
	struct net_if *iface = net_if_get_default();
	struct net_linkaddr *link_addr;

	if (!iface) {
		return;
	}

	link_addr = net_if_get_link_addr(iface);
	if (!link_addr || link_addr->len < 2) {
		return;
	}

	snprintk(stepper_id, sizeof(stepper_id), "%s%02x%02x", CONFIG_NET_HOSTNAME,
		 link_addr->addr[link_addr->len - 2], link_addr->addr[link_addr->len - 1]);

	net_hostname_set(stepper_id, strlen(stepper_id));
}

/*
 * Advertises the network shell (CONFIG_SHELL_BACKEND_TELNET, a fixed,
 * well-known port) via DNS-SD, so it's discoverable as `<stepper_id>.local`
 * without knowing the DHCP-assigned IP - e.g. `avahi-browse -r _telnet._tcp`
 * or `dns-sd -B _telnet._tcp` should find it, and `telnet <stepper_id>.local`
 * connects to the same shell the USB CDC-ACM console provides.
 *
 * The .instance field is stepper_id's *address*, not a copy - a plain
 * global array's address is a valid link-time constant even though its
 * *contents* are only filled in later by set_unique_hostname(), and dns_sd.c
 * reads .instance fresh (strlen() etc.) on every query response rather than
 * caching it, so this picks up the real, MAC-suffixed name once
 * set_unique_hostname() has run.
 *
 * A fixed port means this can be a plain file-scope declaration (no socket
 * setup needed here - the telnet shell backend already owns its own
 * listening socket internally); see
 * zephyr/samples/net/mdns_responder/src/service.c for the ephemeral-port
 * variant, which isn't needed here.
 */
DNS_SD_REGISTER_TCP_SERVICE(mdns_telnet_shell, stepper_id, "_telnet", "local", DNS_SD_EMPTY_TXT,
			     CONFIG_SHELL_TELNET_PORT);

void mdns_force_multicast_rejoin(struct net_if *iface)
{
	struct net_in_addr mdns_addr;

	/*
	 * mdns_responder's own SYS_INIT-time join for 224.0.0.251
	 * (subsys/net/lib/dns/mdns_responder.c) runs long before the
	 * LAN9250's PHY link finishes negotiation (~2s post-boot).
	 * eth_lan9250.c's lan9250_tx() writes the IGMP report straight to
	 * the LAN9250's TX FIFO over SPI and returns success purely based
	 * on that SPI transaction completing - it never checks
	 * net_if_is_carrier_ok() - so with no physical link yet, the
	 * report is silently discarded by the hardware while
	 * net_ipv4_igmp_join() still sees ret == 0 and marks the group
	 * "joined" (confirmed via the `net ipv4` shell command).
	 *
	 * That false-positive "joined" state then defeats both of
	 * Zephyr's built-in recovery paths for this exact case: net_if.c's
	 * rejoin_ipv4_mcast_groups() (run when the interface later goes
	 * operationally up for real) and mdns_responder's own
	 * NET_EVENT_IF_UP handler both skip any group whose
	 * net_if_ipv4_maddr_is_joined() is already true - so no genuine
	 * report is ever retried.
	 *
	 * Worse, net_ipv4_igmp_join()'s net_if_ipv4_maddr_add() refcounts
	 * the group entry on *every* join call regardless of is_joined -
	 * and mdns_responder joins twice on its own (the SYS_INIT boot
	 * join, then again from its NET_EVENT_IF_UP handler once carrier
	 * really comes up), leaving a refcount of 2. net_ipv4_igmp_leave()
	 * only decrements that count; net_if_ipv4_maddr_rm() treats
	 * "count still > 0" as "still in use" and returns without ever
	 * clearing is_joined or sending a Leave report. So a single leave()
	 * call here isn't enough - it silently no-ops, and the following
	 * join() then also no-ops (is_joined is still true). Draining the
	 * refcount to zero first (leave() in a loop until the address is
	 * actually gone, i.e. -ENOENT) is what actually clears is_joined so
	 * the final join() below performs a real send.
	 */
	net_addr_pton(NET_AF_INET, "224.0.0.251", &mdns_addr);

	while (net_ipv4_igmp_leave(iface, &mdns_addr) == 0) {
		/* keep draining refcount until the group is fully removed */
	}

	int ret = net_ipv4_igmp_join(iface, &mdns_addr, NULL);

	if (ret < 0) {
		LOG_WRN("Forced mDNS multicast rejoin failed (%d)", ret);
	} else {
		LOG_INF("Forced mDNS multicast rejoin of 224.0.0.251");
	}
}
