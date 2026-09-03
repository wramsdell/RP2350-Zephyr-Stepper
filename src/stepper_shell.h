#ifndef STEPPER_SHELL_H
#define STEPPER_SHELL_H

/* Sets up the mbox device and callback used by the `stepper` shell commands
 * to talk to core1. Returns 0 on success.
 */
int stepper_mbox_init(void);

#endif /* STEPPER_SHELL_H */
