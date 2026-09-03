#ifndef CORE1_LAUNCH_H
#define CORE1_LAUNCH_H

/* Copies the embedded core1 blob into its reserved RAM region and launches
 * it via the RP2350 bootrom SIO FIFO handshake. Returns 0 on success.
 */
int core1_launch(void);

#endif /* CORE1_LAUNCH_H */
