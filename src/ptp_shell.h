#ifndef PTP_SHELL_H_
#define PTP_SHELL_H_

#include <zephyr/device.h>
#include <zephyr/kernel.h>

/*
 * Arms the LAN9250's 1PPS scope-observation output, but only once gPTP
 * considers this port's clock trustworthy (elected master, or slave with
 * as_capable true) - see gptp_get_port_sync_state()'s doc comment for why
 * that's the right proxy for "safe to treat this clock as synchronized".
 * Blocks the calling thread while polling, up to max_wait_ms; if that
 * elapses without convergence, arms anyway and logs a warning, rather than
 * blocking forever on a board with no peer to sync to.
 */
int ptp_pps_arm_when_synced(const struct device *lan9250_dev, int max_wait_ms);

#endif /* PTP_SHELL_H_ */
