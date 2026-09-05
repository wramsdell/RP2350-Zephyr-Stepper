#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/net/dns_sd.h>
#include <zephyr/net/hostname.h>
#include <zephyr/net/net_if.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mdns_service, LOG_LEVEL_INF);

#include "mdns_service.h"
#include "eth_id.h"

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
	/*
	 * mdns_responder's own SYS_INIT-time join for 224.0.0.251
	 * (subsys/net/lib/dns/mdns_responder.c) can race the LAN9250's PHY
	 * link negotiation and end up falsely marked "joined" with no real
	 * report ever sent - see the comment on force_multicast_rejoin()
	 * (eth_id.c) for the full explanation of why a plain leave+join
	 * isn't enough and a refcount-draining loop is required.
	 */
	force_multicast_rejoin(iface, "224.0.0.251");
}
