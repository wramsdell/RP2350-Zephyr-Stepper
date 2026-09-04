#ifndef ETH_LAN9250_PTP_H
#define ETH_LAN9250_PTP_H

#include <zephyr/device.h>
#include <stdint.h>

/*
 * Public API for the LAN9250's IEEE 1588 (PTP) hardware clock - see
 * drivers/eth_lan9250/eth_lan9250.c's lan9250_1588_init() and
 * THEORY_OF_OPERATION.md's "IEEE 1588 / PTP" section for the phased plan
 * this is part of.
 */

/* Snapshots the live 1588 clock and reads it back. sec/ns are required;
 * subns may be NULL if the extra sub-nanosecond precision isn't needed.
 * Returns 0 on success, a negative errno from the underlying SPI
 * transaction on failure.
 */
int lan9250_ptp_clock_read(const struct device *dev, uint32_t *sec, uint32_t *ns,
			   uint32_t *subns);

/* Arms a self-sustaining 1PPS output (100ns pulse, active high) on
 * GPIO1. Idempotent - safe to call again to re-arm/re-align. See the
 * comment on this function's definition for the hardware-specific
 * reasoning (push/pull vs. open-drain, channel/pin choice).
 */
int lan9250_1588_pps_enable(const struct device *dev);

#endif /* ETH_LAN9250_PTP_H */
