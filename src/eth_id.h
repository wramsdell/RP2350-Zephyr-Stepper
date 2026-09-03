#ifndef ETH_ID_H
#define ETH_ID_H

/* Overrides the LAN9250's MAC address (hardcoded/static in the devicetree
 * overlay) with one derived from the RP2350's real per-chip unique ID via
 * hwinfo, so each physical board gets its own stable, distinct MAC - and by
 * extension, via set_unique_hostname() (mdns_service.h), its own mDNS
 * hostname. Call once, early in main(), before DHCP starts.
 */
void set_unique_mac_address(void);

/* Puts the LAN9250 into promiscuous mode so it actually receives multicast
 * frames (mDNS, IGMP queries, etc.) - see the comment on this function's
 * definition for why this driver requires that. Call once, early in
 * main(), alongside set_unique_mac_address().
 */
void enable_multicast_rx(void);

#endif /* ETH_ID_H */
