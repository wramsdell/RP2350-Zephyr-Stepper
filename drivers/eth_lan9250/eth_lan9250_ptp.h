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

/* Directly sets the clock to sec.ns (LOAD, not a step - the clock jumps
 * straight to this value).
 */
int lan9250_ptp_clock_set(const struct device *dev, uint32_t sec, uint32_t ns);

/* Applies a one-time step of increment_ns nanoseconds (either sign) to
 * the clock. See the comment on this function's definition for why this
 * is a software read-modify-write rather than the hardware's single-tick
 * step mechanism.
 */
int lan9250_ptp_clock_adjust(const struct device *dev, int32_t increment_ns);

/* Applies a permanent rate trim, as a ratio relative to nominal (1.0 =
 * unadjusted, >1.0 = faster, <1.0 = slower) - matches
 * ptp_clock_driver_api.rate_adjust()'s convention directly.
 */
int lan9250_ptp_clock_rate_adjust(const struct device *dev, double ratio);

/* Arms a self-sustaining 1PPS output (100ns pulse, active high) on
 * GPIO1. Idempotent - safe to call again to re-arm/re-align. See the
 * comment on this function's definition for the hardware-specific
 * reasoning (push/pull vs. open-drain, channel/pin choice).
 */
int lan9250_1588_pps_enable(const struct device *dev);

/* Reads and clears the hardware RX_DROP counter (see the comment on
 * LAN9250_RX_DROP) - the number of frames the chip's own MIL FIFO lost
 * before the driver's RX_FIFO_INF/RX_STATUS_FIFO path ever saw them.
 */
int lan9250_rx_drop_get(const struct device *dev, uint32_t *rx_drop);

#endif /* ETH_LAN9250_PTP_H */
