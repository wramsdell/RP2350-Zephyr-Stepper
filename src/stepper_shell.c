#include <stdlib.h>
#include <errno.h>
#include <math.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/mbox.h>
#include <zephyr/shell/shell.h>

#include "../core1/mailbox_proto.h"
#include "stepper_shell.h"

/* Calculated assuming 6mm pitch leadscrew, 1600 steps per rev - same
 * constant Stepper-Control.c uses for its `movein` command. */
#define STEPS_PER_INCH 6773.3f

static const struct device *mbox_dev;
static struct k_sem reply_sem;
static volatile uint32_t last_reply;

static void mbox_cb(const struct device *dev, mbox_channel_id_t channel_id,
		     void *user_data, struct mbox_msg *msg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(channel_id);
	ARG_UNUSED(user_data);

	last_reply = *(const uint32_t *)msg->data;
	k_sem_give(&reply_sem);
}

int stepper_mbox_init(void)
{
	mbox_dev = DEVICE_DT_GET(DT_NODELABEL(mbox));
	if (!device_is_ready(mbox_dev)) {
		return -ENODEV;
	}

	k_sem_init(&reply_sem, 0, 1);

	int ret = mbox_register_callback(mbox_dev, 0, mbox_cb, NULL);

	if (ret) {
		return ret;
	}

	return mbox_set_enabled(mbox_dev, 0, true);
}

static int mbox_send_word(uint32_t word)
{
	struct mbox_msg msg = {
		.data = &word,
		.size = sizeof(word),
	};

	return mbox_send(mbox_dev, 0, &msg);
}

static int mbox_wait_reply(uint32_t *reply_out)
{
	if (k_sem_take(&reply_sem, K_MSEC(1000)) != 0) {
		return -ETIMEDOUT;
	}

	*reply_out = last_reply;
	return 0;
}

/* opcode-only request, single-word reply - CMD_STOP and each CMD_STATUS
 * field query (which additionally sends a field-index payload word, see
 * status_query() below).
 */
static int cmd_roundtrip0(uint32_t opcode, uint32_t *reply_out)
{
	int ret = mbox_send_word(opcode);

	if (ret) {
		return ret;
	}
	return mbox_wait_reply(reply_out);
}

/* opcode + one payload word request, single-word reply - CMD_SPEED/ACCEL/
 * DECEL/MOVE, and CMD_STATUS (payload = field index).
 */
static int cmd_roundtrip1(uint32_t opcode, uint32_t payload, uint32_t *reply_out)
{
	int ret = mbox_send_word(opcode);

	if (ret) {
		return ret;
	}
	ret = mbox_send_word(payload);
	if (ret) {
		return ret;
	}
	return mbox_wait_reply(reply_out);
}

static int status_query(const struct shell *sh, uint32_t field, uint32_t *reply_out)
{
	if (cmd_roundtrip1(CMD_STATUS, field, reply_out)) {
		shell_error(sh, "core1 did not respond");
		return -1;
	}
	return 0;
}

static int cmd_stepper_speed(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	float hz = strtof(argv[1], NULL);
	uint32_t reply;

	if (cmd_roundtrip1(CMD_SPEED, mbox_pack_float(hz), &reply)) {
		shell_error(sh, "core1 did not respond");
		return -1;
	}

	shell_print(sh, "max speed set to %.0f steps/sec", (double)mbox_unpack_float(reply));
	return 0;
}

static int cmd_stepper_accel(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	float v = strtof(argv[1], NULL);
	uint32_t reply;

	if (cmd_roundtrip1(CMD_ACCEL, mbox_pack_float(v), &reply)) {
		shell_error(sh, "core1 did not respond");
		return -1;
	}

	shell_print(sh, "accel set to %.0f steps/sec^2", (double)mbox_unpack_float(reply));
	return 0;
}

static int cmd_stepper_decel(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	float v = strtof(argv[1], NULL);
	uint32_t reply;

	if (cmd_roundtrip1(CMD_DECEL, mbox_pack_float(v), &reply)) {
		shell_error(sh, "core1 did not respond");
		return -1;
	}

	shell_print(sh, "decel set to %.0f steps/sec^2", (double)mbox_unpack_float(reply));
	return 0;
}

static int move_steps(const struct shell *sh, int32_t steps)
{
	uint32_t reply;

	if (steps == 0) {
		shell_error(sh, "step count cannot be zero");
		return -1;
	}

	if (cmd_roundtrip1(CMD_MOVE, (uint32_t)steps, &reply)) {
		shell_error(sh, "core1 did not respond");
		return -1;
	}
	if (reply == STEPPER_ACK_BUSY) {
		shell_error(sh, "already moving; use 'stepper stop' first");
		return -1;
	}

	shell_print(sh, "moving %d steps %s...", steps, steps > 0 ? "forward" : "reverse");
	return 0;
}

static int cmd_stepper_move(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	return move_steps(sh, (int32_t)strtol(argv[1], NULL, 10));
}

static int cmd_stepper_movein(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);

	float inches = strtof(argv[1], NULL);
	int32_t steps = (int32_t)lroundf(inches * STEPS_PER_INCH);

	return move_steps(sh, steps);
}

static int cmd_stepper_stop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint32_t reply;

	if (cmd_roundtrip0(CMD_STOP, &reply)) {
		shell_error(sh, "core1 did not respond");
		return -1;
	}

	shell_print(sh, "stop requested - will halt after current step.");
	return 0;
}

static int cmd_stepper_status(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint32_t state, max_speed, accel, decel, speed, remaining;

	if (status_query(sh, STATUS_STATE, &state) ||
	    status_query(sh, STATUS_MAX_SPEED, &max_speed) ||
	    status_query(sh, STATUS_ACCEL, &accel) ||
	    status_query(sh, STATUS_DECEL, &decel) ||
	    status_query(sh, STATUS_SPEED, &speed) ||
	    status_query(sh, STATUS_REMAINING, &remaining)) {
		return -1;
	}

	shell_print(sh, "  max speed : %.0f steps/sec", (double)mbox_unpack_float(max_speed));
	shell_print(sh, "  accel     : %.0f steps/sec^2", (double)mbox_unpack_float(accel));
	shell_print(sh, "  decel     : %.0f steps/sec^2", (double)mbox_unpack_float(decel));
	if (state) {
		shell_print(sh, "  state     : moving - %d steps remaining, current %.0f steps/sec",
			    (int32_t)remaining, (double)mbox_unpack_float(speed));
	} else {
		shell_print(sh, "  state     : idle");
	}
	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_stepper,
	SHELL_CMD_ARG(speed, NULL, "Set max step rate (steps/sec)", cmd_stepper_speed, 2, 0),
	SHELL_CMD_ARG(accel, NULL, "Set acceleration (steps/sec^2)", cmd_stepper_accel, 2, 0),
	SHELL_CMD_ARG(decel, NULL, "Set deceleration (steps/sec^2)", cmd_stepper_decel, 2, 0),
	SHELL_CMD_ARG(move, NULL, "Move N steps (+forward/-reverse)", cmd_stepper_move, 2, 0),
	SHELL_CMD_ARG(movein, NULL, "Move N inches (+forward/-reverse)", cmd_stepper_movein, 2, 0),
	SHELL_CMD_ARG(stop, NULL, "Abort the current move", cmd_stepper_stop, 1, 0),
	SHELL_CMD_ARG(status, NULL, "Show settings and motion state", cmd_stepper_status, 1, 0),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(stepper, &sub_stepper, "Core1 stepper motor control", NULL);
