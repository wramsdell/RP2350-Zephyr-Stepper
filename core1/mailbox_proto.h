/*
 * Wire protocol for the core0 <-> core1 SIO FIFO mailbox. Shared verbatim by
 * both the Zephyr (core0) side and the bare-metal (core1) side.
 *
 * Core0's mailbox driver (drivers/mbox/mbox_rpi_pico.c) has an ISR that
 * reads exactly one FIFO word, delivers it to the registered callback, then
 * calls fifo_drain() - which silently discards anything else already
 * queued in the RX FIFO. That means core1 must never push more than one
 * reply word per request without an intervening core0-initiated request:
 * a burst of words from core1 would have all but the first one dropped.
 * (This only affects core1 -> core0 traffic; core0 -> core1 sends don't
 * go through that ISR/drain path at all, so multi-word *requests* are
 * fine.)
 *
 * So every command here is exactly one request -> one reply, word for
 * word, even CMD_STATUS: each status field is its own request/reply pair
 * (STEPPER_STATUS_FIELD_COUNT of them), keyed by a field index in the
 * second request word, rather than one request returning a multi-word
 * burst.
 *
 * Floats are packed as raw IEEE-754 bits (see mbox_pack_float()/
 * mbox_unpack_float() below) - no separate encoding scheme.
 */
#ifndef CORE1_MAILBOX_PROTO_H
#define CORE1_MAILBOX_PROTO_H

#include <stdint.h>

enum stepper_cmd {
	CMD_SPEED = 0,  /* req: [opcode, float hz]    reply: [float accepted_hz] */
	CMD_ACCEL = 1,  /* req: [opcode, float v]     reply: [float accepted_v]  */
	CMD_DECEL = 2,  /* req: [opcode, float v]     reply: [float accepted_v]  */
	CMD_MOVE = 3,   /* req: [opcode, int32 steps] reply: [uint32 ack]       */
	CMD_STOP = 4,   /* req: [opcode]              reply: [uint32 ack]      */
	CMD_STATUS = 5, /* req: [opcode, uint32 field (see enum stepper_status_field)]
			  * reply: [uint32 or packed-float value for that field]
			  */
};

/* CMD_MOVE / CMD_STOP ack values. */
#define STEPPER_ACK_OK   0u
#define STEPPER_ACK_BUSY 1u /* CMD_MOVE while already moving */

/* CMD_STATUS field indices - one mailbox round trip per field.
 * STATUS_STATE reply is 0=idle, 1=moving; STATUS_REMAINING reply is an
 * int32 packed as raw bits; the rest are packed floats. */
enum stepper_status_field {
	STATUS_STATE = 0,
	STATUS_MAX_SPEED = 1,
	STATUS_ACCEL = 2,
	STATUS_DECEL = 3,
	STATUS_SPEED = 4,
	STATUS_REMAINING = 5,
	STEPPER_STATUS_FIELD_COUNT = 6,
};

static inline uint32_t mbox_pack_float(float f)
{
	union {
		float f;
		uint32_t u;
	} v = {.f = f};
	return v.u;
}

static inline float mbox_unpack_float(uint32_t u)
{
	union {
		float f;
		uint32_t u;
	} v = {.u = u};
	return v.f;
}

#endif /* CORE1_MAILBOX_PROTO_H */
