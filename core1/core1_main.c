#include <stdint.h>

#include "regs.h"
#include "mailbox_proto.h"

/*
 * Bare-metal core1: open-loop trapezoidal stepper motion via PIO-generated
 * STEP pulses, controlled by core0 over the SIO FIFO mailbox.
 *
 * Ported from /home/ward/src/Stepper-Control (Stepper-Control.c +
 * stepper.pio) - same v^2-kinematics motion profile, same PIO protocol
 * (one FIFO push per step, SM stalls at "pull block" when idle), same pin
 * assignment. See THEORY_OF_OPERATION.md for the full design writeup.
 */

#define PIN_ENABLE 13u /* active low */
#define PIN_DIR    14u /* high = forward */
#define PIN_STEP   15u /* PIO0 side-set output */

#define SIO_FUNCSEL 5u
#define PIO0_FUNCSEL 6u

/* This board's Zephyr build runs clk_sys at 150MHz (confirmed via
 * CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC), not the Pico SDK's 125MHz default
 * the reference project assumed. PIO's clock divider is left at its 1:1
 * reset value, so this is the frequency the half-period formula must use. */
#define PIO_F_SYS 150000000.0f

#define MIN_SPEED_HZ 50.0f /* floor: below this we halt rather than slow further */

/* EMMS-ST-28 torque-speed curve is voltage-dependent (see Stepper-Control.c) */
#define ABS_MAX_SPEED 160000.0f /* steps/sec, assumes 48V DC supply */
#define ABS_MAX_ACCEL 2666666.0f /* steps/sec^2, 10 m/s^2 slide limit */

#define DEF_MAX_SPEED 160000.0f
#define DEF_ACCEL     1000000.0f
#define DEF_DECEL     1000000.0f

/* Assembled stepper.pio program (wrap_target=0, wrap=5, 1-bit side-set) -
 * taken verbatim from Stepper-Control's generated build/stepper.pio.h. */
static const uint16_t stepper_program[] = {
	0x80a0, 0xa047, 0xb022, 0x1043, 0xa022, 0x0045,
};

typedef enum { IDLE, MOVING } motion_state_t;

static struct {
	motion_state_t state;
	float max_speed; /* steps/sec, user-settable */
	float accel;     /* steps/sec^2 */
	float decel;     /* steps/sec^2 */
	float speed;     /* current instantaneous speed, steps/sec */
	int32_t remaining; /* steps left to push into the PIO FIFO */
	uint8_t stop_req;  /* set by CMD_STOP to abort the current move */
} M = {
	.state = IDLE,
	.max_speed = DEF_MAX_SPEED,
	.accel = DEF_ACCEL,
	.decel = DEF_DECEL,
};

/* ── GPIO ─────────────────────────────────────────────────────────────── */

static void gpio_pad_init(uint32_t pin, uint32_t funcsel)
{
	/* Clear the RP2350 pad isolation latch (set by default after reset)
	 * and select the pin's function - see the LED bring-up comment in
	 * the original multicore project's core1_main.c for why the ISO bit
	 * matters. */
	REG_CLR(PADS_BANK0_GPIO(pin), PADS_BANK0_GPIO0_ISO_BITS);
	REG(IO_BANK0_GPIO_CTRL(pin)) = funcsel << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
}

static void enable_dir_init(void)
{
	gpio_pad_init(PIN_ENABLE, SIO_FUNCSEL);
	REG(SIO_GPIO_OE_SET) = (1u << PIN_ENABLE);
	REG(SIO_GPIO_OUT_SET) = (1u << PIN_ENABLE); /* active low: start disabled */

	gpio_pad_init(PIN_DIR, SIO_FUNCSEL);
	REG(SIO_GPIO_OE_SET) = (1u << PIN_DIR);
	REG(SIO_GPIO_OUT_SET) = (1u << PIN_DIR); /* forward default */
}

static void timer0_init(void)
{
	REG_CLR(RESETS_RESET, RESETS_RESET_TIMER0_BITS);
	while (!(REG(RESETS_RESET_DONE) & RESETS_RESET_TIMER0_BITS)) {
	}
}

static inline uint32_t now_us(void)
{
	return REG(TIMER0_TIMERAWL);
}

static void delay_us(uint32_t us)
{
	uint32_t start = now_us();

	while ((uint32_t)(now_us() - start) < us) {
	}
}

/* ── PIO: STEP pulse generation ──────────────────────────────────────────
 *
 * core1 has no Pico SDK PIO C-SDK available (same reasoning as regs.h's
 * comment on avoiding the hardware/structs headers), so this programs PIO0 SM0
 * directly against raw registers. Every step here is cross-checked against
 * the Pico SDK's own hardware_pio implementation
 * (pio_gpio_init/pio_sm_set_consecutive_pindirs/pio_sm_init), not guessed.
 */

static void pio_step_init(void)
{
	REG_CLR(RESETS_RESET, RESETS_RESET_PIO0_BITS);
	while (!(REG(RESETS_RESET_DONE) & RESETS_RESET_PIO0_BITS)) {
	}

	gpio_pad_init(PIN_STEP, PIO0_FUNCSEL);

	for (uint32_t i = 0; i < 6u; i++) {
		REG(PIO0_INSTR_MEM0 + 4u * i) = stepper_program[i];
	}

	/* WRAP_TOP=5, WRAP_BOTTOM=0; SIDE_EN/SIDE_PINDIR left at their
	 * reset-default 0 (side-set is always applied, not optional, and
	 * doesn't control pin direction) - matches
	 * stepper_program_get_default_config()'s
	 * sm_config_set_sideset(&c, 1, false, false). EXECCTRL's reset value
	 * is 0x0001f000, so this direct write is safe (no other field needs
	 * preserving). */
	REG(PIO0_SM0_EXECCTRL) = (5u << 12) | (0u << 7);

	/*
	 * Pin direction: PIO-routed pins get their output-enable from the
	 * PIO block itself, not SIO_GPIO_OE, so it's set by *executing* a
	 * "SET PINDIRS" instruction on the state machine - this replicates
	 * pio_sm_set_consecutive_pindirs() exactly (pio.c in the Pico SDK)
	 * for our single pin, single count case:
	 *   1. Point PINCTRL's SET_BASE/SET_COUNT at our pin.
	 *   2. Inject "SET PINDIRS, 0x1f" via SM0_INSTR - this executes
	 *      immediately, out of band from the program counter.
	 *   3. Reprogram PINCTRL to its real, final value (SIDESET_BASE/
	 *      SIDESET_COUNT) for actual program execution.
	 * 0xe09f = pio_encode_set(pio_pindirs, 0x1f) =
	 *          0xe000 | (4 << 5) | 0x1f
	 */
	REG(PIO0_SM0_PINCTRL) = (1u << 26) | (PIN_STEP << 5);
	REG(PIO0_SM0_INSTR) = 0xe09f;
	REG(PIO0_SM0_PINCTRL) = (1u << 29) | (PIN_STEP << 10);

	/* SM0_CLKDIV and SM0_SHIFTCTRL are left at their power-on reset
	 * values (0x00010000 = divisor 1.0 = full clk_sys; 0x000c0000 = no
	 * autopull/autopush) - both already match what this program needs. */

	REG(PIO0_CTRL) = 0x1u; /* enable SM0; stalls at "pull block" until fed */
}

static inline uint32_t hz_to_half_period(float hz)
{
	float half_period = PIO_F_SYS / (2.0f * hz) - 3.0f;

	return half_period > 0.0f ? (uint32_t)half_period : 0u;
}

static inline int pio_tx_full(void)
{
	return (REG(PIO0_FSTAT) & (1u << 16)) != 0;
}

static inline void pio_push_step(float hz)
{
	REG(PIO0_TXF0) = hz_to_half_period(hz);
}

/* ── Motion profile (ported from Stepper-Control.c's next_speed()/
 * feed_step()/motion_start() - exact v^2-kinematics, unchanged) ───────── */

/*
 * -ffreestanding implies -fno-builtin, so __builtin_sqrtf() doesn't lower
 * to the M33 FPU's single vsqrt.f32 instruction the way it would in a
 * normal (hosted) build - it instead emits a call to the sqrtf() library
 * function, which doesn't exist here (no libm linked). Emit the FPU
 * instruction directly instead; "t" is GCC's ARM VFP single-precision
 * register constraint (s0-s31).
 */
static inline float hw_sqrtf(float x)
{
	float result;

	__asm volatile("vsqrt.f32 %0, %1" : "=t"(result) : "t"(x));
	return result;
}

static float next_speed(void)
{
	float v = M.speed;
	float vmax = M.max_speed;
	float a = M.accel;
	float d = M.decel;
	float vmin = MIN_SPEED_HZ;

	/* Steps required to decelerate from v to vmin: (v^2-vmin^2)/(2*d) */
	float brake_steps = (v * v - vmin * vmin) / (2.0f * d);

	if (M.remaining <= (int32_t)(brake_steps + 1.0f)) {
		float v2 = v * v - 2.0f * d;

		return v2 > (vmin * vmin) ? hw_sqrtf(v2) : vmin;
	}
	if (v < vmax) {
		float v2 = v * v + 2.0f * a;

		return v2 < (vmax * vmax) ? hw_sqrtf(v2) : vmax;
	}
	return vmax;
}

static int feed_step(void)
{
	if (M.state != MOVING) {
		return 0;
	}

	if (M.stop_req || M.remaining <= 0) {
		M.state = IDLE;
		M.speed = 0.0f;
		M.stop_req = 0;
		return 0;
	}

	M.speed = next_speed();
	pio_push_step(M.speed);
	M.remaining--;
	return 1;
}

static void motion_start(int32_t steps)
{
	if (steps == 0) {
		return;
	}

	int forward = (steps > 0);

	if (forward) {
		REG(SIO_GPIO_OUT_SET) = (1u << PIN_DIR);
	} else {
		REG(SIO_GPIO_OUT_CLR) = (1u << PIN_DIR);
	}
	delay_us(5); /* direction setup time for driver */

	M.remaining = forward ? steps : -steps;
	M.speed = MIN_SPEED_HZ;
	M.stop_req = 0;
	M.state = MOVING;
}

/* ── Mailbox command handling ────────────────────────────────────────────
 *
 * Polled once per main-loop iteration (interleaved with keeping the PIO
 * FIFO fed during a move). Each command is a fixed word count agreed with
 * core0 in mailbox_proto.h; the opcode word is only ever read when FIFO
 * data is actually pending, but any expected follow-up word(s) are read
 * with a short blocking wait since core0 always sends them back-to-back.
 */

static uint32_t fifo_read_blocking(void)
{
	while (!(REG(SIO_FIFO_ST) & SIO_FIFO_ST_VLD_BITS)) {
	}
	return REG(SIO_FIFO_RD);
}

static void fifo_write_blocking(uint32_t word)
{
	while (!(REG(SIO_FIFO_ST) & SIO_FIFO_ST_RDY_BITS)) {
	}
	REG(SIO_FIFO_WR) = word;
}

static void handle_command(void)
{
	if (!(REG(SIO_FIFO_ST) & SIO_FIFO_ST_VLD_BITS)) {
		return;
	}

	uint32_t opcode = REG(SIO_FIFO_RD);

	switch (opcode) {
	case CMD_SPEED: {
		float v = mbox_unpack_float(fifo_read_blocking());

		if (v < MIN_SPEED_HZ) {
			v = MIN_SPEED_HZ;
		} else if (v > ABS_MAX_SPEED) {
			v = ABS_MAX_SPEED;
		}
		M.max_speed = v;
		fifo_write_blocking(mbox_pack_float(M.max_speed));
		break;
	}
	case CMD_ACCEL: {
		float v = mbox_unpack_float(fifo_read_blocking());

		if (v <= 0.0f || v > ABS_MAX_ACCEL) {
			v = DEF_ACCEL;
		}
		M.accel = v;
		fifo_write_blocking(mbox_pack_float(M.accel));
		break;
	}
	case CMD_DECEL: {
		float v = mbox_unpack_float(fifo_read_blocking());

		if (v <= 0.0f || v > ABS_MAX_ACCEL) {
			v = DEF_DECEL;
		}
		M.decel = v;
		fifo_write_blocking(mbox_pack_float(M.decel));
		break;
	}
	case CMD_MOVE: {
		int32_t steps = (int32_t)fifo_read_blocking();
		uint32_t ack = STEPPER_ACK_OK;

		if (M.state == MOVING) {
			ack = STEPPER_ACK_BUSY;
		} else {
			motion_start(steps);
		}
		fifo_write_blocking(ack);
		break;
	}
	case CMD_STOP: {
		if (M.state == MOVING) {
			M.stop_req = 1;
		}
		fifo_write_blocking(STEPPER_ACK_OK);
		break;
	}
	case CMD_STATUS: {
		uint32_t field = fifo_read_blocking();
		uint32_t reply = 0;

		switch (field) {
		case STATUS_STATE:
			reply = (M.state == MOVING) ? 1u : 0u;
			break;
		case STATUS_MAX_SPEED:
			reply = mbox_pack_float(M.max_speed);
			break;
		case STATUS_ACCEL:
			reply = mbox_pack_float(M.accel);
			break;
		case STATUS_DECEL:
			reply = mbox_pack_float(M.decel);
			break;
		case STATUS_SPEED:
			reply = mbox_pack_float(M.speed);
			break;
		case STATUS_REMAINING:
			reply = (uint32_t)M.remaining;
			break;
		default:
			break;
		}
		fifo_write_blocking(reply);
		break;
	}
	default:
		break;
	}
}

void core1_main(void)
{
	enable_dir_init();
	timer0_init();
	pio_step_init();

	delay_us(10000);
	REG(SIO_GPIO_OUT_CLR) = (1u << PIN_ENABLE); /* enable the driver */

	while (1) {
		if (M.state == MOVING) {
			while (!pio_tx_full()) {
				if (!feed_step()) {
					break;
				}
			}
		}
		handle_command();
	}
}
