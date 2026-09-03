#ifndef MDNS_SERVICE_H
#define MDNS_SERVICE_H

#include <zephyr/net/net_if.h>

/* Sets both the mDNS hostname and the DNS-SD service instance name to
 * "<CONFIG_NET_HOSTNAME><last 2 MAC bytes, hex>", e.g. "stepper0203" for a
 * MAC ending in 02:03. Call once, early in main(), after the network
 * interface (and thus its link address) exists.
 */
void set_unique_hostname(void);

/* Forces a real IGMP leave+rejoin of the mDNS multicast group
 * (224.0.0.251) on iface. Call once a working link is independently
 * confirmed (e.g. a bound DHCP lease) - see the comment on this function's
 * definition for why this is necessary on top of mDNS's own join.
 */
void mdns_force_multicast_rejoin(struct net_if *iface);

#endif /* MDNS_SERVICE_H */
