#ifndef ETH_ID_H
#define ETH_ID_H

#include <zephyr/net/net_if.h>

/* Forces a real IGMP leave+rejoin of the IPv4 multicast group at addr_str
 * (dotted-decimal) on iface. Call once a working link is independently
 * confirmed (e.g. a bound DHCP lease) - see the comment on this function's
 * definition for why this is necessary on top of any subsystem's own boot-
 * time join. Used by mdns_force_multicast_rejoin() (mdns_service.h) for
 * 224.0.0.251, and by ptp_multicast_rejoin() below for PTP's groups.
 */
void force_multicast_rejoin(struct net_if *iface, const char *addr_str);

/* Forces IGMP joins for PTP-over-UDP's multicast destination addresses:
 * 224.0.1.129 (general/event messages - Sync, Announce, Delay_Req, etc.)
 * and 224.0.0.107 (peer-delay mechanism messages). Without these, every
 * inbound PTP packet is silently dropped by net_ipv4_input()'s "mcast not
 * for me" gate before it ever reaches the driver's RX processing, even
 * though the LAN9250's hardware RX filter (RX_PARSE_CONFIG's RX_ADD1_EN/
 * RX_ADD5_EN, both on by default) already recognizes both addresses at the
 * hardware level - same class of issue as mDNS's 224.0.0.251, just never
 * joined in the first place since nothing subscribes to PTP traffic on its
 * own like mdns_responder does. Call once a working link is independently
 * confirmed, same timing as mdns_force_multicast_rejoin().
 */
void ptp_multicast_rejoin(struct net_if *iface);

/* Starts a periodic (30s) IGMP membership-report refresh for iface, for
 * the lifetime of the application. Necessary because this network's
 * switch was confirmed to send zero IGMP query traffic - with no active
 * querier, nothing else ever prompts a renewed report, and a switch's
 * IGMP-snooping forwarding entry can silently age out with no signal to
 * the host that it happened. See the definition's comment for the full
 * story. Call once, after a working link is confirmed (same timing as
 * the other multicast-join calls above); safe to call more than once,
 * only the first call has any effect.
 */
void multicast_membership_refresh_start(struct net_if *iface);

#endif /* ETH_ID_H */
