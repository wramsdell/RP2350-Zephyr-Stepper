# Theory of Operation: Core1 bare-metal stepper motor control

This document describes how Core0 (Zephyr) and Core1 (bare-metal) coexist on
the RP2350 in this project: the source layout, the memory partition, how
Core1's firmware gets built and loaded, how Core1 drives the stepper motor,
and the wire protocol the two cores use to talk to each other.

This project forks [RP2350-Zephyr-Multicore](https://github.com/wramsdell/RP2350-Zephyr-Multicore),
which established the multicore plumbing (memory partition, launch
handshake, mailbox mechanism, blob-embedding build) using a trivial LED
blink demo on Core1. Everything in that plumbing is unchanged here; only
Core1's actual workload, and the mailbox protocol that controls it, are new
- ported from the bare-metal Pico SDK reference project at
`/home/ward/src/Stepper-Control` (`Stepper-Control.c` + `stepper.pio`).

## Source structure

```
CMakeLists.txt                 Top-level app build; wires core1's build in
cmake/core1.cmake              Cross-compiles + embeds the core1 image
cmake/pioasm.cmake             Builds pioasm as a native host tool (see below)
prj.conf                       Core0 (Zephyr) Kconfig
rpi_pico2_rp2350a_m33.overlay  Devicetree: SRAM split, mbox, LAN9250, USB console

src/                           Core0 (Zephyr) application sources
  main.c                       Calls core1_launch() + stepper_mbox_init(), then
                                the existing DHCPv4 client demo
  core1_launch.h / .c          PSM/SIO launch handshake that starts core1
                                (unchanged from the multicore project - generic)
  core1_blob.c                 const uint8_t[] holding core1's compiled image
                                (via #include <core1_blob.bin.inc>, generated
                                at build time - see "Build & load" below)
  stepper_shell.c / .h         mbox client + the `stepper ...` shell commands

core1/                         Bare-metal core1 image sources (NOT Zephyr)
  linker.ld                    Flat linker script; must match the SRAM
                                carve-out in the devicetree overlay
                                (unchanged from the multicore project)
  vectors.c                    16-entry Cortex-M33 vector table
                                (unchanged from the multicore project)
  start.c                      Reset handler: zero .bss, enable the FPU,
                                call core1_main()
  core1_main.c                 GPIO/PIO bring-up, trapezoidal motion state
                                machine, mailbox command handling
  stepper.pio                  PIO assembly source for STEP pulse generation -
                                real, editable source, not pre-assembled bytes
                                (see "PIO: STEP pulse generation" below)
  regs.h                       Minimal raw register definitions (see below
                                for why this doesn't use the Pico SDK's
                                hardware/structs headers) - extended from the
                                multicore project with PIO0 + CPACR/FPU regs
  mailbox_proto.h               Wire protocol shared by both cores' code
```

Everything under `core1/` is compiled completely separately from the Zephyr
application — no Zephyr headers, no RTOS, no C library beyond a couple of
freestanding-safe standard headers (`<stdint.h>`). It is cross-compiled by
`cmake/core1.cmake` using the same Zephyr SDK toolchain, but as a bare
freestanding ELF/binary, and only its *output bytes* end up inside the
Zephyr (Core0) image, via `src/core1_blob.c`.

### Why `core1/regs.h` doesn't use the Pico SDK's `hardware/structs/*.h`

The obvious approach would be to include the vendored Pico SDK's
`hardware/structs/sio.h` etc., like the Core0-side code does. That doesn't
work for Core1's build: those headers pull in `hardware/address_mapped.h`,
which includes `pico.h`, which in this Zephyr checkout chains into
Zephyr-only shim headers
(`zephyr/modules/hal_rpi_pico/pico/config_autogen.h` →
`<zephyr/toolchain.h>` and friends) that simply don't exist outside a normal
Zephyr compilation unit.

The plain `hardware/regs/*.h` headers, by contrast, are pure, dependency-free
macro files (register offsets and bitmasks only, `#include`ing nothing but
each other). `core1/regs.h` includes those directly (now also
`hardware/regs/pio.h`) and defines a handful of `REG()`/`REG_SET()`/
`REG_CLR()` helpers on top, giving Core1 register access without dragging in
any Zephyr or Pico-SDK-runtime machinery. Every offset/bitmask used was
cross-checked against the real generated headers at
`modules/hal/rpi_pico/src/rp2350/hardware_regs/include/hardware/regs/`, and
the PIO register *sequence* (not just individual offsets) was cross-checked
against the Pico SDK's own tested `hardware_pio` implementation - see
"PIO: STEP pulse generation" below.

Core0's code (`src/core1_launch.c`) *does* use the full `hardware/structs/*.h`
headers, because it's a normal part of the Zephyr build and already has the
whole Zephyr include environment available — no issue there.

## Memory partition

Unchanged from the multicore project. Zephyr models RP2350's 520KB of SRAM
as a single flat `sram0` devicetree node (`0x20000000`-`0x20082000`), with
no notion of the chip's individual SRAM banks. `CONFIG_SRAM_SIZE`/
`CONFIG_SRAM_BASE_ADDRESS` are Kconfig defaults derived directly from that
node's `reg` property, and both the ARM linker script and the RP2350 MPU
region builder consume those Kconfig values directly. So the devicetree
overlay simply shrinks `&sram0`'s `reg`:

```dts
&sram0 {
    reg = <0x20000000 DT_SIZE_K(392)>;
};
```

This confines Zephyr's linker layout *and* its MPU SRAM region to the bottom
392KB (`0x20000000`-`0x20062000`), leaving the top 128KB
(`0x20062000`-`0x20082000`, two 64KB banks) completely outside anything
Zephyr's build system or runtime memory protection touches. `core1/linker.ld`
targets exactly that region:

```
MEMORY { CORE1_RAM (rwx) : ORIGIN = 0x20062000, LENGTH = 128K }
```

within which the two 64KB banks are used as:
- Bank at `0x20062000`: vector table + `.text`/`.rodata`/`.data`
- Bank at `0x20072000`: `.bss` + stack (stack grows down from the top of
  SRAM, `__core1_stack_top`)

Neither core's Zephyr devicetree/Kconfig ever references GP13/GP14/GP15 (the
stepper driver pins) — Core1 owns them exclusively via its own per-core SIO
block and PIO0, entirely independent of Zephyr's GPIO driver.

## Build & load: how Core1's code gets onto the chip

Unchanged from the multicore project. There is exactly one build
(`west build`) and one flash (one `zephyr.uf2`). Core1's firmware is
embedded inside Core0's image and copied into place by Core0 at runtime:

1. **Assemble** (`cmake/pioasm.cmake` + the `core1.cmake` step that uses it):
   `core1/stepper.pio` is assembled into a generated `stepper.pio.h` by
   `pioasm` - see "PIO: STEP pulse generation" below for how `pioasm` itself
   gets built and invoked.
2. **Cross-compile** (`cmake/core1.cmake`): the three `core1/*.c` sources are
   compiled and linked against `core1/linker.ld` with the same
   `arm-zephyr-eabi-gcc`/`objcopy` Core0 uses (resolved automatically by
   `find_package(Zephyr REQUIRED)`), but with `-ffreestanding -nostdlib
   -nostartfiles` and none of Zephyr's normal compile flags — producing
   `core1.elf`, then `core1.bin` (a raw flat binary; link address == load
   address, so no relocation is needed).
3. **Embed**: Zephyr's own `generate_inc_file_for_target()` CMake helper
   turns `core1.bin` into a generated `core1_blob.bin.inc` — a plain
   comma-separated byte list. `src/core1_blob.c` `#include`s it into a
   `const uint8_t core1_blob[]` array, which becomes part of the ordinary
   Core0 Zephyr binary and thus part of `zephyr.uf2`.
4. **Load at boot** (`core1_launch()` in `src/core1_launch.c`, called near
   the top of Core0's `main()`): `memcpy()`s `core1_blob` verbatim into
   `0x20062000`, reads the initial stack pointer and entry point directly
   out of the copied vector table's first two words, then performs the
   RP2350 bootrom's documented core1 launch handshake (see the multicore
   project's own THEORY_OF_OPERATION.md for the exact PSM/SIO FIFO sequence
   - unchanged here).

## PIO: STEP pulse generation

`core1/stepper.pio` is real, editable PIO assembly source - not
pre-assembled bytes copied from another project. Getting from that source
to instruction words core1 can load into `PIO0_INSTR_MEM0` needs `pioasm`,
the Pico SDK's own PIO assembler - a completely separate tool from
`arm-zephyr-eabi-gcc` (PIO has its own small, distinct instruction set;
generic C compilers have no concept of it). `pioasm`'s source is vendored
in `hal_rpi_pico` (`modules/hal/rpi_pico/tools/pioasm`) - it ships with the
Pico SDK - but it's a code generator that must run *on the build machine*,
not be cross-compiled for the RP2350. So `cmake/pioasm.cmake` builds it as
a wholly separate, native-host CMake sub-build via `ExternalProject_Add`,
deliberately not forwarding this project's ARM cross-compiler settings, so
its own fresh `cmake` invocation picks up the host's default `cc`/`c++` the
same way any ordinary native build would.

`cmake/core1.cmake` then runs the resulting `pioasm` executable against
`core1/stepper.pio` (`pioasm -o c-sdk stepper.pio stepper.pio.h`) as a
build step ordered before compiling `core1_main.c`, which
`#include "stepper.pio.h"`s the result and uses its generated
`stepper_program_instructions[]`/`stepper_wrap_target`/`stepper_wrap`
directly - so editing `stepper.pio` and rebuilding (`west build`, no
`--pristine` needed) regenerates and relinks everything downstream
automatically, the same as editing any other source file.

`stepper.pio` deliberately has no `% c-sdk { ... %}` block (the mechanism
`.pio` files normally use to also emit Pico-SDK-dependent C-SDK helper
functions like `stepper_program_init()`): that block's contents get copied
into the generated header *unconditionally*, `#include "hardware/clocks.h"`
and all, which would reintroduce the same `pico.h` chain-into-Zephyr-shims
problem `regs.h`'s header comment describes above. Compiling with
`-DPICO_NO_HARDWARE=1` (set in `core1.cmake`) additionally makes the
generated header skip its own `#include "hardware/pio.h"` and the `struct
pio_program`/`stepper_program_get_default_config()` it would otherwise
define - both guarded by `#if !PICO_NO_HARDWARE` in `pioasm`'s own
`c-sdk` output template - leaving just the plain, dependency-free
instruction array and `wrap`/`wrap_target` `#define`s this freestanding
build actually needs. Core1 hand-rolls the rest of the SM setup (pin
config, `EXECCTRL`, pin-direction handshake) against raw registers itself -
see the init sequence below.

One push to the PIO TX FIFO produces exactly one STEP pulse, at a period
encoded by the pushed word (a half-period in PIO clock cycles). The state
machine blocks at `pull block` when the FIFO is empty, so it stops cleanly
with zero extra pulses the instant the CPU stops feeding it - no explicit
stop/drain logic needed, and no risk of extra steps at the end of a move.

The init sequence, in order (every step cross-checked against the Pico
SDK's own `hardware_pio` implementation, not guessed):

1. Release `RESETS_RESET_PIO0` (same clear-then-wait-for-`reset_done`
   pattern used for `TIMER0`).
2. Configure GP15's pad (clear the ISO latch, same as every other GPIO
   here) with `FUNCSEL=6` (PIO0).
3. Load the instruction words (`stepper_program_instructions[]`, from the
   generated header - `STEPPER_PROGRAM_LENGTH` of them) into
   `PIO0_INSTR_MEM0` onward.
4. Write `SM0_EXECCTRL` to set `WRAP_TOP`/`WRAP_BOTTOM` from the generated
   `stepper_wrap`/`stepper_wrap_target` (a direct write is safe here -
   `EXECCTRL`'s power-on reset value is `0x0001f000`, and every other
   field's reset default of 0 is what this program needs anyway). Because
   these come from pioasm's output rather than being typed in by hand,
   editing `stepper.pio`'s `.wrap`/`.wrap_target` can never silently drift
   out of sync with what core1_main.c programs into the SM.
5. **Pin direction** - the one non-obvious step. PIO-routed pins get their
   output-enable from the PIO block itself, not `SIO_GPIO_OE`, so it has to
   be set by *executing* a `SET PINDIRS` instruction on the state machine
   (replicating the Pico SDK's `pio_sm_set_consecutive_pindirs()`): point
   `PINCTRL`'s `SET_BASE`/`SET_COUNT` at GP15, inject the encoded
   `SET PINDIRS, 0x1f` instruction directly via `SM0_INSTR` (executes
   immediately, out of band from the program counter), then reprogram
   `PINCTRL` to its real, final value (`SIDESET_BASE`/`SIDESET_COUNT`) for
   actual program execution.
6. `SM0_CLKDIV` and `SM0_SHIFTCTRL` are left at their power-on reset values
   (divisor 1.0 = full `clk_sys`; no autopull/autopush) - both already
   match what this program needs.
7. Enable SM0 via `PIO0_CTRL`.

Half-period formula (`stepper.pio`'s C-SDK block, adapted): 
`half_period = f_pio / (2 * step_freq_hz) - 3`. This board's Zephyr build
runs `clk_sys` (and thus, PIO's default 1:1 clock, `f_pio`) at **150MHz**
(`CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC`), not the reference project's
Pico-SDK-default 125MHz - `core1_main.c` uses `150000000.0f` accordingly.

GP13 (Enable, active-low) and GP14 (Dir) are plain SIO GPIO outputs, set up
the same way the multicore project's LED demo drove GP25.

## Motion profile

Ported directly from `Stepper-Control.c`'s `next_speed()`/`feed_step()`/
`motion_start()` - the same exact v²-kinematics, unchanged:

```
accel step:  v_new = sqrt(v² + 2a)       (one step of displacement)
decel step:  v_new = sqrt(v² - 2d)
cruise:      v_new = max_speed
```

The decel trigger fires when `steps_remaining ≤ (v² - v_min²) / (2·decel)` -
exact, not approximated, so the motor reaches `MIN_SPEED_HZ` on the last
step with no overshoot. Short moves that can't reach `max_speed` form a
triangle profile automatically (the decel trigger fires while still
accelerating).

`core1_main()`'s main loop keeps the 4-entry PIO TX FIFO topped up whenever
a move is active (`feed_step()` computes and pushes the next step's speed
each time there's FIFO space), and polls for mailbox commands once per
iteration regardless.

### Floating point: `sqrtf()` needs the FPU explicitly enabled - and a direct instruction, not the builtin

Two build-level details worth calling out, since the LED-only multicore
project never touched floats:

- Each core's FPU-access register (`CPACR`, at the standard ARMv8-M address
  `0xE000ED88`) is private per core. Core0/Zephyr enables its own via the
  Pico SDK's `runtime_init_per_core_enable_coprocessors()`, which never
  touches Core1. `core1/start.c` now does the Core1 equivalent (`CPACR |=
  0x300000`, full CP10/FPU access) before calling `core1_main()`.
  `cmake/core1.cmake` also switched Core1's compile flags from
  `-mfloat-abi=soft` to `-mfpu=fpv5-sp-d16 -mfloat-abi=hard` (the same FPU
  variant Zephyr's own build uses for this SoC's Cortex-M33, per
  `cmake/gcc-m-fpu.cmake`).
- `-ffreestanding` (needed for Core1's standalone build) implies
  `-fno-builtin` in GCC, so `__builtin_sqrtf()` does *not* lower to the
  FPU's single `vsqrt.f32` instruction the way it normally would - it emits
  an actual call to the `sqrtf()` library function instead, which doesn't
  exist here (no libm linked, deliberately, to keep Core1 fully
  freestanding). `core1_main.c`'s `hw_sqrtf()` sidesteps this with inline
  assembly (`vsqrt.f32` via GCC's `"t"` VFP single-precision register
  constraint) instead of relying on the builtin.

Both fixes mean `sqrtf` costs exactly one hardware instruction per call, no
library linked, keeping Core1 genuinely freestanding.

## Inter-core communication: the mailbox

Core0 and Core1 exchange 32-bit words over the RP2350's SIO inter-core
FIFOs — separate hardware FIFOs per direction, but each core addresses "its"
TX/RX side via the same register names (`fifo_wr`/`fifo_rd`/`fifo_st`), so
the hardware handles the crossover.

- **Core0 side**: uses Zephyr's existing `mbox` driver for this SoC
  (`drivers/mbox/mbox_rpi_pico.c`), enabled via `&mbox { status = "okay"; }`
  in the overlay and `CONFIG_MBOX=y` in `prj.conf`. `mbox_send()` writes
  `*(uint32_t *)msg->data` straight to `sio_hw->fifo_wr`; the RX ISR reads
  `sio_hw->fifo_rd` and hands it to a registered callback.
- **Core1 side**: being bare-metal, it pokes the identical underlying
  `SIO_FIFO_WR`/`SIO_FIFO_RD`/`SIO_FIFO_ST` registers directly (via
  `core1/regs.h`), polling `SIO_FIFO_ST_VLD_BITS` once per main-loop
  iteration.

### A real constraint this protocol had to work around

`mbox_rpi_pico`'s ISR (`rpi_pico_mbox_isr()`) reads **exactly one** FIFO
word, delivers it to the registered callback, then calls `fifo_drain()` -
which silently discards anything else already queued in the RX FIFO. It was
written for a strict one-word-request/one-word-reply protocol (which is all
the LED blink demo ever needed) and will **silently drop data** if Core1
ever pushes more than one reply word in a row without an intervening
Core0-initiated request in between.

This only affects Core1→Core0 traffic - `mbox_send()` (Core0→Core1) writes
directly to the FIFO with no ISR/drain path involved, so multi-word
*requests* from Core0 are fine. But it ruled out an initial design where a
single `status` query would get a 6-word burst reply (state, max_speed,
accel, decel, speed, remaining) - up to 5 of those 6 words would have been
silently lost. The protocol below reflects the fix: `CMD_STATUS` is one
request/reply pair *per field*, not one request for all of them.

### Message format (`core1/mailbox_proto.h`)

Every command is a fixed sequence of raw 32-bit words, zero framing beyond
the opcode - matching the underlying driver's zero-framing raw-word
semantics. Floats are packed as raw IEEE-754 bits
(`mbox_pack_float()`/`mbox_unpack_float()`).

| Opcode | Request words | Reply words |
|---|---|---|
| `CMD_SPEED` | `[opcode, float hz]` | `[float accepted_hz]` |
| `CMD_ACCEL` | `[opcode, float v]` | `[float accepted_v]` |
| `CMD_DECEL` | `[opcode, float v]` | `[float accepted_v]` |
| `CMD_MOVE` | `[opcode, int32 steps]` | `[uint32 ack]` (`STEPPER_ACK_OK`/`_BUSY`) |
| `CMD_STOP` | `[opcode]` | `[uint32 ack]` |
| `CMD_STATUS` | `[opcode, uint32 field]` | `[uint32 or packed-float value]` |

`CMD_STATUS`'s `field` is one of `STATUS_STATE`/`_MAX_SPEED`/`_ACCEL`/
`_DECEL`/`_SPEED`/`_REMAINING` (`enum stepper_status_field`). `stepper
status`'s shell handler does six separate round trips, one per field -
exactly the same one-request/one-reply pattern as every other command, just
called in a loop.

`CMD_MOVE` is fire-and-forget from Core0's point of view: the ack means
"accepted" (or "rejected, already moving"), not "move complete" - Core1
keeps running the trapezoidal profile independently in the background, and
`stepper status` is how you observe progress.

Every command's round trip is driven the same way from `stepper_shell.c`:
send the request word(s), then block on a semaphore (given by the mbox RX
callback) with a 1-second timeout, so all `stepper ...` shell commands feel
like ordinary synchronous commands even though the actual reply arrives
asynchronously via IRQ.

Sequencing note: `core1_launch()` runs before `stepper_mbox_init()` enables
the mbox IRQ, so there's no need to save/restore the SIO FIFO IRQ enable
state around the launch handshake.
