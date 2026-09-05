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
Kconfig                        App-root Kconfig; sources drivers/eth_lan9250/Kconfig
cmake/core1.cmake              Cross-compiles + embeds the core1 image
cmake/pioasm.cmake             Builds pioasm as a native host tool (see below)
prj.conf                       Core0 (Zephyr) Kconfig
rpi_pico2_rp2350a_m33.overlay  Devicetree: SRAM split, mbox, LAN9250, USB console

drivers/eth_lan9250/           Project-owned fork of Zephyr's LAN9250 driver
                                (see "Forked LAN9250 driver" below)
  eth_lan9250_ptp.h            Public API: LAN9250 1588 clock + PPS (see
                                "IEEE 1588 / PTP" below)
  ptp_clock_lan9250.c           Zephyr ptp_clock device wrapping the 1588 clock
dts/bindings/ethernet/         Devicetree binding matching the forked driver's
                                renamed compatible string

src/                           Core0 (Zephyr) application sources
  main.c                       Calls core1_launch() + stepper_mbox_init(), then
                                the existing DHCPv4 client demo
  core1_launch.h / .c          PSM/SIO launch handshake that starts core1
                                (unchanged from the multicore project - generic)
  core1_blob.c                 const uint8_t[] holding core1's compiled image
                                (via #include <core1_blob.bin.inc>, generated
                                at build time - see "Build & load" below)
  stepper_shell.c / .h         mbox client + the `stepper ...` shell commands
  eth_id.c / .h                Per-board unique MAC from hwinfo (see
                                "Network discovery" below)
  mdns_service.c / .h          mDNS hostname + DNS-SD advertisement (see
                                "Network discovery" below)
  bootloader_shell.c           `bootloader`/`reset` shell commands (no
                                physical BOOTSEL button needed to reflash)
  ptp_shell.c                  `ptp clock`/`ptp pps` shell commands (see
                                "IEEE 1588 / PTP" below)

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

## Network discovery: per-board unique ID, mDNS hostname + DNS-SD service (Core0 only)

Unrelated to the core0/core1 split above, but worth documenting: Core0
answers to `stepperXXXX.local` over mDNS (`XXXX` = the last 2 bytes, in hex,
of the board's real MAC address) and advertises its network shell via
DNS-SD, so it's reachable without knowing its DHCP-assigned IP - and
distinguishable from any other board running the same firmware.

- **Unique MAC first** (`src/eth_id.c`, `set_unique_mac_address()`, called
  before anything else in `main()`): the LAN9250's `local-mac-address` in
  the devicetree overlay is a fixed placeholder (`00:00:00:01:02:03`) -
  every board would otherwise get the *identical* hostname. `hwinfo_get_device_id()`
  (`CONFIG_HWINFO=y`) returns the RP2350's real per-chip unique ID (its
  flash's 64-bit unique RUID, read via `hardware/flash.h`'s
  `flash_get_unique_id()` - see `hwinfo_rpi_pico.c`); the last 6 of its 8
  bytes become the new MAC, with the locally-administered/unicast bits set
  the same way Zephyr's own `net_eth_mac_load()`/`NET_ETH_MAC_RANDOM` path
  does for a generated (non-IEEE-assigned) address. Applying it takes two
  calls, both required: the driver's `set_config()` (`ETHERNET_CONFIG_TYPE_MAC_ADDRESS`)
  reprograms the LAN9250's *hardware* RX address filter - skip this and the
  chip keeps silently dropping inbound unicast frames addressed to the new
  MAC - and `net_if_set_link_addr()` updates the *stack's* view (what
  ARP/DHCP/the hostname derivation below actually see). These are called
  directly rather than through `net_mgmt(NET_REQUEST_ETHERNET_SET_MAC_ADDRESS, ...)`:
  that wrapper (`ethernet_mgmt.c`) refuses unless the interface is
  administratively down first, and Zephyr brings interfaces up on their own
  well before `main()` runs. Satisfying that would mean a
  `net_if_down()`/`net_if_up()` cycle here - which is actively harmful:
  `net_if_down()` unconditionally strips every multicast group membership
  the interface holds (including mDNS's `224.0.0.251` join), and
  `net_if_up()` never restores them. Calling the driver directly gets the
  identical end result without ever touching admin state.
- **Multicast RX needs a driver fix** - see "Forked LAN9250 driver" below.
  Without it, the LAN9250's own RX filter silently drops every
  multicast-destination frame in hardware (mDNS, IGMP, everything addressed
  to `01:00:5e:xx:xx:xx`), completely independent of anything at the
  IP/IGMP layer - confirmed by elevating `net_ipv4`/`net_conn` to debug
  logging: broadcast and unicast frames reached `net_ipv4_input()` and were
  processed normally, but not one multicast-destined frame ever arrived
  there, even with constant mDNS/IGMP traffic on the wire.
- `CONFIG_NET_HOSTNAME_ENABLE=y` + `CONFIG_NET_HOSTNAME="stepper"` +
  `CONFIG_NET_HOSTNAME_DYNAMIC=y` give the device a base name that can be
  changed at runtime; `set_unique_hostname()` (`src/mdns_service.c`, called
  right after `set_unique_mac_address()`) reads the *now-real* MAC via
  `net_if_get_link_addr()` and appends its last 2 bytes as hex, then calls
  `net_hostname_set()`. `CONFIG_NET_HOSTNAME_DYNAMIC` also raises
  `NET_HOSTNAME_MAX_LEN`'s default from just `sizeof(CONFIG_NET_HOSTNAME)-1`
  to 63, which is what actually leaves room to append those 4 hex digits -
  without it `net_hostname_set()` would simply reject the longer string.
  `CONFIG_MDNS_RESPONDER=y` makes the device answer mDNS queries for
  `stepperXXXX.local` on `224.0.0.251` (IPv6 is disabled in this project, so
  only the IPv4 multicast group is used); `mdns_responder.c` calls
  `net_hostname_get()` fresh on every query rather than caching a
  compile-time copy, so the runtime-set name is what's actually advertised.
- `CONFIG_SHELL_BACKEND_TELNET=y` adds a second, independent shell backend
  (Zephyr supports multiple simultaneous shell instances) alongside the
  existing USB CDC-ACM one - same `stepper ...`/`net ...` commands, reachable
  over TCP port 23 (`CONFIG_SHELL_TELNET_PORT`, the standard telnet port)
  instead of USB.
- `CONFIG_DNS_SD=y` + `CONFIG_MDNS_RESPONDER_DNS_SD=y` enable DNS-SD
  (RFC 6763) service advertisement. `src/mdns_service.c` registers the
  telnet shell as a discoverable service with a single file-scope
  `DNS_SD_REGISTER_TCP_SERVICE(...)` declaration - no server socket code of
  its own is needed there, since the telnet shell backend already owns its
  own listening socket; the macro just adds a static record (placed in an
  iterable linker section the mDNS responder scans, the same general
  mechanism used elsewhere in Zephyr for `SHELL_CMD_REGISTER` etc.) that
  answers PTR/SRV/TXT queries for `_telnet._tcp.local`, discoverable via
  e.g. `avahi-browse -r _telnet._tcp` or `dns-sd -B _telnet._tcp`. Its
  `.instance` field is a pointer to the *same* `stepper_id` buffer
  `set_unique_hostname()` writes - a plain global array's address is a
  valid link-time constant even though its contents are only filled in
  later, and `dns_sd.c` calls `strlen(inst->instance)` fresh on every
  response rather than caching it, so the DNS-SD instance name and the
  mDNS hostname can never drift out of sync with each other.

Reference: `zephyr/samples/net/mdns_responder/src/service.c` demonstrates
the same macro family, including the ephemeral-port variant
(`DNS_SD_REGISTER_SERVICE`) for services that don't have a fixed,
well-known port the way telnet does.

### Forcing a real IGMP join (`src/eth_id.c`, `force_multicast_rejoin()`)

Originally added as `mdns_service.c`'s `mdns_force_multicast_rejoin()`;
generalized into `eth_id.c` (already the home for other "network interface
setup" concerns like MAC address assignment) once phase 4's PTP work
(below) needed the identical join-timing/refcount-draining logic for its
own multicast groups (`224.0.1.129`, `224.0.0.107`) via
`ptp_multicast_rejoin()`. `mdns_force_multicast_rejoin()` is now a thin
wrapper calling `force_multicast_rejoin(iface, "224.0.0.251")`.

`mdns_responder`'s own boot-time IGMP join for `224.0.0.251` runs at
`SYS_INIT` priority 96 - long before the LAN9250's PHY finishes link
negotiation (~2s post-boot). `drivers/ethernet/eth_lan9250.c`'s
`lan9250_tx()` writes frames straight to the LAN9250's TX FIFO over SPI and
reports success purely from that SPI transaction completing - it never
checks `net_if_is_carrier_ok()` - so with no physical link yet, that first
membership report is silently discarded by the hardware while
`net_ipv4_igmp_join()` still sees `ret == 0` and marks the group "joined"
(confirmed via the `net ipv4` shell command). That false-positive "joined"
state then defeats both of Zephyr's built-in recovery paths for exactly
this case: `net_if.c`'s `rejoin_ipv4_mcast_groups()` (run when the
interface later goes operationally up for real) and `mdns_responder`'s own
`NET_EVENT_IF_UP` handler both skip any group already marked joined, so no
genuine report is ever retried.

Worse, `net_ipv4_igmp_join()` refcounts the group entry on every call
regardless of join state, and `mdns_responder` joins twice on its own (the
boot-time join, then again from its `NET_EVENT_IF_UP` handler) - leaving a
refcount of 2. A single `net_ipv4_igmp_leave()` call only decrements that;
`net_if_ipv4_maddr_rm()` treats "count still > 0" as "still in use" and
returns without ever clearing the joined flag or sending a Leave. So
`force_multicast_rejoin()` drains the refcount to zero in a loop
(`net_ipv4_igmp_leave()` until the address is actually gone) before
rejoining - only then does the final `net_ipv4_igmp_join()` perform a real
send. `mdns_force_multicast_rejoin()` and `ptp_multicast_rejoin()` are both
called back-to-back from `main.c`'s existing `NET_EVENT_IPV4_ADDR_ADD`
handler: a bound DHCP lease is independent proof the link genuinely works
(it required real Ethernet round trips), unlike at boot. PTP's groups have
no boot-time join of their own to race in the first place (nothing
subscribes to them until phase 4 needs to), so for PTP this is really just
"join at the right time", not a "rejoin" - but it needs the identical
timing and the same refcount-safe path, so reusing the helper as-is (rather
than writing a separate plain-join function) avoids two subtly different
copies of logic that's already been debugged once.

### Forked LAN9250 driver (`drivers/eth_lan9250/`)

**Two approaches to multicast RX were tried before landing here.**

The first was app-level: put the whole interface into full promiscuous
mode (`net_eth_promisc_mode()`), since that's the only RX-filter toggle
Zephyr's upstream `eth_lan9250.c` driver actually implements
(`ETHERNET_CONFIG_TYPE_PROMISC_MODE` - there's no per-group hardware
filter support, and the driver doesn't advertise `ETHERNET_HW_FILTERING`,
so the generic multicast-join hardware-filter hook in `ethernet.c` never
even calls into it). This worked initially, but on this LAN's real traffic
volume - constant mDNS chatter from several devices, Chromecast/Google
Home discovery, SSDP, etc. - it caused two escalating problems:

1. Every frame on the LAN (not just multicast ones) raised an RX
   interrupt and got pulled over the LAN9250's slow 10MHz SPI bus, which
   exhausted the default `net_pkt` RX buffer pool under sustained bursts
   (`Could not allocate rx buffer` in the log). Sizing the pool generously
   in `prj.conf` (`CONFIG_NET_PKT_RX_COUNT`/`CONFIG_NET_BUF_RX_COUNT`,
   still in place today - see below) fixed *that* symptom.
2. But it recurred anyway under sustained (not just bursty) traffic, and
   `net stats` told a different story the second time: `IPv4 recv` stayed
   flat while `IP vhlerr`/`protoer` (header-version and protocol-field
   validation errors) climbed and `Processing err` sat in the thousands -
   i.e. frames were arriving *corrupted*, not just too numerous to buffer.
   ARP resolution failed intermittently as a result (`ping`/`telnet`
   directly to the board's IP reported "Destination Host Unreachable" from
   the querying machine's own kernel - a local ARP-resolution failure, and
   `arp -n` showed `(incomplete)`). The likely mechanism: promiscuous mode
   raises an RX interrupt for *every* frame on the LAN, and the driver's
   single RX thread (`lan9250_thread()`, shared between PHY-link
   interrupts and FIFO draining) doing a new frame's multi-step SPI FIFO
   read while a previous one is still mid-flight can desync the LAN9250's
   FIFO byte stream - no amount of software-side buffer sizing fixes
   corrupted frame *data*.

That pointed at reducing traffic at the hardware source instead of trying
to out-buffer it: `HMAC_CR`'s `MCPAS` (pass-all-multicast) bit. The
driver's own comment right above its `HMAC_CR` init write already claims
"Pass all multicast frames" / "Hash filtering disabled" - but the code
never actually sets `MCPAS`, so that comment describes intent, not what
the register write does. Setting it (multicast only, not every frame on
the LAN) cuts the interrupt rate at the source rather than trying to
survive it in software.

The straightforward way to apply that is a one-line patch to Zephyr's
vendored driver - but that file lives in the west-managed
`~/zephyrproject/zephyr` checkout, outside this repo, untracked by git,
and liable to silently vanish on a `west update`. That's a real cost for
a single MCPAS bit; it stops being one once real driver-level work is
needed. And there's a second reason to own this driver anyway: this
project may eventually want the LAN9250's hardware IEEE 1588/PTP
timestamping unit (an extensive hardware block - ~80 pages of the
datasheet, Section 14.0), and Zephyr's driver doesn't just omit PTP
support, it actively disables the 1588 clock and timestamp unit at init
(`lan9250_configure()`'s `PMT_CTRL` write includes `1588_DIS |
1588_TSU_DIS`). Implementing that would mean substantial changes to this
driver regardless - real PTP support isn't tracked here yet, but when it
happens, having a git-tracked, project-owned copy to build on beats
carrying an ever-growing untracked patch against the west tree.

So `drivers/eth_lan9250/` is a full fork of Zephyr's
`drivers/ethernet/eth_lan9250.c`/`eth_lan9250_priv.h`, kept as a
minimal-diff copy (same structure, same register-level logic) with:
- `MCPAS` added to the `HMAC_CR` init write (the actual fix).
- `DT_DRV_COMPAT` renamed from `microchip_lan9250` to `rp2350zs_lan9250`,
  matching a new devicetree binding
  (`dts/bindings/ethernet/rp2350zs,lan9250.yaml`, itself a renamed copy of
  the upstream binding) and the overlay's `compatible = "rp2350zs,lan9250"`
  (`rpi_pico2_rp2350a_m33.overlay`). This - not disabling the upstream
  driver - is what keeps the two from colliding: with no devicetree node
  using `compatible = "microchip,lan9250"` any more, Zephyr's upstream
  driver (still present, untouched, in the west tree) simply never binds
  to anything and isn't compiled in.
- The three driver-specific Kconfig options renamed to
  `CONFIG_RP2350ZS_ETH_LAN9250_*` (`drivers/eth_lan9250/Kconfig`, a
  similarly renamed copy of upstream's `Kconfig.lan9250`), avoiding any
  ambiguity with the (inert) upstream symbols of the same shape.

Wired into the build like any other app source
(`target_sources(app PRIVATE drivers/eth_lan9250/eth_lan9250.c ...)` in
`CMakeLists.txt`) plus an app-root `Kconfig` file
(`rsource "drivers/eth_lan9250/Kconfig"` then `source "Kconfig.zephyr"` -
Zephyr auto-detects a `Kconfig` file at the application root; see the
Kconfig section of `doc/develop/application/index.rst` in the Zephyr
tree). No `ZEPHYR_EXTRA_MODULES`/west module machinery needed for a
single driver used by one app.

Owning the binding also unlocked two follow-on fixes that the upstream
`microchip,lan9250.yaml` binding couldn't accommodate:

- **SPI clock raised from 10MHz to 25MHz**
  (`rpi_pico2_rp2350a_m33.overlay`'s `spi-max-frequency`). Datasheet TABLE
  10-3 "SPI/SQI Timing Values" caps `f_sck` at 80MHz for writes/Dual/Quad
  SIO instructions but only 30MHz for single-line Read instructions (Note
  3) - and this driver uses single-line reads exclusively (RX FIFO pulls,
  register polling), so 30MHz is the real ceiling. 25MHz leaves some
  margin below that for this hand-wired setup. (An RP2350 SPI0 quirk
  worth noting: `&spi0`'s own `clock-frequency` devicetree property looks
  like it might cap this, but it's actually inert - `spi_pl022.c`, the
  driver backing this SoC's SPI controller, queries the real peripheral
  clock at runtime via `clock_control_get_rate()` and never reads that
  property at all; only the LAN9250 child node's `spi-max-frequency`
  matters.)
- **Real hardware reset on every boot, not just power-on**
  (`reset-gpios` added to `dts/bindings/ethernet/rp2350zs,lan9250.yaml`,
  then wired to GP22 in the overlay). `eth_lan9250.c`'s `lan9250_init()`
  already had a complete, correctly-timed reset-pulse implementation
  (`config->reset.port != NULL` gate, driving GP22 low for 250us then
  waiting 20ms - matching LAN9250 datasheet Section 19.6.3's t_rstia/t_cfg
  timing) - it just never ran, because the upstream binding this project
  forked never declared `reset-gpios` as a valid property, so the overlay
  could only leave it commented out (see the overlay's git history) with
  no way to enable it. The symptom without it: reflashing over USB resets
  the RP2350 but not the LAN9250 itself (a warm MCU reset isn't a power
  cycle for a separate chip on the board), so the LAN9250 could retain
  state from its previous boot and Zephyr's own init sequence could fail
  against it - observed as `eth_id: net_if_set_link_addr() failed (-1)`
  on boot, requiring a manual power cycle to clear. Adding the property to
  this project's own binding and pointing it at the already-wired GP22
  (previously dead code, a commented-out `gpio-hog` block that only ever
  released reset once at cold power-on) activates the driver's existing
  logic with no driver-code changes at all.

### Sizing the buffer pools (`prj.conf`)

Even with multicast-only RX (not full promiscuous mode), this LAN is
genuinely chatty across many multicast groups - mDNS from several devices,
Chromecast/Google Home discovery, SSDP, etc., not just our own
`224.0.0.251` traffic - with responses commonly running 400-650+ bytes and
arriving in bursts of several back-to-back. Combined with the LAN9250's
slow 10MHz SPI bus (each frame needs a full synchronous SPI read before
the next can start), two buffer pools are sized generously in `prj.conf`
as cheap insurance against burst traffic, given RAM has plenty of headroom
(~25% used of 392KB):

- The generic `net_pkt` RX pool (`CONFIG_NET_PKT_RX_COUNT=32`,
  `CONFIG_NET_BUF_RX_COUNT=128` vs. defaults of 14/36 at 128 bytes each,
  ~4.6KB total) - originally bumped to fix `Could not allocate rx buffer`
  under the promiscuous-mode approach above; kept since the traffic
  volume, while reduced, is still real.
- `mdns_responder.c`'s own separate, dedicated buffer pool
  (`mdns_msg_pool`, distinct from the pool above) for every packet it
  parses on `224.0.0.251:5353`. It defaults to just
  `DNS_RESOLVER_MIN_BUF(2) + CONFIG_MDNS_RESOLVER_ADDITIONAL_BUF_CTR(0)` =
  2 buffers of `MDNS_RESOLVER_BUF_SIZE(512)` bytes each. Since the
  responder has to parse every multicast packet it receives to check
  relevance (not just ones meant for this device), a query burst - e.g.
  `avahi-browse -a -r`, which makes every other device on the LAN answer
  about every service type nearly simultaneously - exhausted that 2-buffer
  pool almost instantly, confirmed via `net stats`: the DNS drop counter
  spiked sharply during exactly this kind of burst while DNS recv only
  rose modestly (packets arriving and being dropped specifically at this
  layer, not just the generic IP layer). Fixed with
  `CONFIG_MDNS_RESOLVER_ADDITIONAL_BUF_CTR=14`.

## IEEE 1588 / PTP (Core0 only)

The LAN9250 has a genuinely serious hardware IEEE 1588-2008 (PTP)
implementation - datasheet Section 14.0, ~80 pages - well beyond a simple
free-running counter: a tunable 32-bit-seconds/30-bit-nanoseconds clock
with load/step/rate-adjust primitives, automatic hardware RX/TX packet
timestamping (with optional on-the-fly one-step timestamp insertion), and
a clock-event comparator block that can drive a GPIO pin directly from
hardware. Zephyr's upstream driver not only omits all of this, it actively
disables the 1588 clock and timestamp unit at init (`lan9250_configure()`'s
`PMT_CTRL` write used to include `PMT_CTRL_1588_DIS | PMT_CTRL_1588_TSU_DIS`
- see `drivers/eth_lan9250/eth_lan9250.c`'s header comment). Implementing
this is being done in phases, each independently testable before moving
on:

1. **Bring up the raw 1588 clock** (`lan9250_1588_init()`, `ptp clock`
   shell command) - enable the 1588 unit and its timestamp unit, load the
   clock to a known value (0), confirm it free-runs at a sane rate by
   reading it twice a known interval apart. No Zephyr API integration
   yet.
2. **1PPS output** (`lan9250_1588_pps_enable()`, `ptp pps` shell command)
   - a hardware-generated, self-sustaining pulse-per-second signal,
   directly oscilloscope-observable, validating the whole clock + GPIO
   event chain independently of any network protocol work. Done, see
   below.
3. **Zephyr `ptp_clock` driver integration**
   (`drivers/eth_lan9250/ptp_clock_lan9250.c`) - wire `CLOCK_SEC/NS`,
   `CLOCK_STEP_ADJ`, and `CLOCK_RATE_ADJ` into Zephyr's standard
   `ptp_clock` device abstraction (`.get`/`.set`/`.adjust`/
   `.rate_adjust`), exposed via the Ethernet driver's `.get_ptp_clock()`,
   so the clock becomes usable by ordinary Zephyr networking APIs
   independent of any PTP protocol work. Done, see below.
4. **Hardware RX/TX packet timestamping** - enable the PTP Timestamp
   block's ingress/egress recording for Sync/Delay_Req/PDelay messages,
   wire captured timestamps into `net_pkt`'s timestamp fields (the
   mechanism Zephyr's gPTP subsystem expects). Both sides done and
   confirmed - RX against real `ptp4l` L2 traffic, TX via the `ptp txtest`
   shell command - see below. TX is confirmed for L2-framed PTP only;
   UDP/IPv4-framed PTP TX was tried and did not produce a capture, left as
   a known, unresolved gap.
5. **Full network PTP sync** (not yet done, stretch goal) - either
   Zephyr's built-in gPTP (802.1AS) subsystem against the `ptp_clock`
   driver from phase 3, or a minimal hand-rolled ordinary-clock PTP
   client (Sync/Delay_Req exchange + servo) if gPTP doesn't fit. Milestone:
   the 1PPS output from phase 2 tracking/locking to an external reference
   instead of free-running.

### Register access

All the registers phases 1-2 need (`1588_CMD_CTL`, `1588_GENERAL_CONFIG`,
`1588_CLOCK_SEC/NS/SUBNS`, `1588_CLOCK_RATE_ADJ`, the Clock Target/Reload
register pairs, `GPIO_CFG`, `LED_CFG`) are plain directly-addressed system
registers - "Bank: na" in the datasheet's Table 14-1 - reached exactly the
same way as `PMT_CTRL` etc. elsewhere in this driver
(`lan9250_read_sys_reg()`/`lan9250_write_sys_reg()`), no MAC-CSR
indirection or bank-select needed. Per-port RX/TX timestamp config (Banks
0-2) and per-GPIO capture registers (Bank 3), needed starting at phase 4,
are reached via `1588_BANK_PORT_GPIO_SEL` and aren't used yet.

One correction worth flagging for future reference: `1588_GENERAL_CONFIG`'s
`RELOAD_ADD_A`/`RELOAD_ADD_B` bits were initially guessed at bits
11/10 (`0x400`/`0x800`) by pattern-matching neighboring fields' scale
without checking the actual bit table - checking the datasheet page
directly (Section 14.8.2, page 323) before writing any code caught this:
they're actually bits 0/1 (`0x1`/`0x2`). Left as a reminder that this
particular datasheet's field layouts don't always follow an obvious
pattern from one page to the next - verify against the actual bit table,
not adjacent fields' scale.

### 1PPS output (`lan9250_1588_pps_enable()`, `ptp pps`)

Uses 1588 Clock Event Channel A with the Clock Target's Reload/Add
register pair set to exactly `{1s, 0ns}` in *increment* mode
(`GENERAL_CONFIG`'s `RELOAD_ADD_A = 0`, confusingly the opposite of what
the name suggests - 0 means "increment the Clock Target by the Reload/Add
value on every compare event", 1 means "reload it" - a one-shot preload,
not what a repeating signal needs). Once armed, the LAN9250's own
comparator advances the Clock Target by 1 second and re-fires
indefinitely, entirely in hardware - no CPU involvement per pulse, so
jitter is bounded only by the 1588 clock's own accuracy, not by any
software re-arming loop.

Output pin is GPIO1 (this board's pin 46, `LED1/GPIO1/TDI/MNGT1`) - unused
by the RJ45's LEDs (those are on GPIO0/GPIO2), confirmed only a 10k
pull-down for the `MNGT1` boot strap. That pull-down is also why the GPIO
is configured as a **push/pull** output rather than open-drain
(`GPIO_CFG`'s `GPIOBUF[1] = 1`): the datasheet's open-drain-plus-1588-event
behavior only ever drives the pin low or leaves it floating, and with a
pull-*down* (not pull-up) present, "floating" would just read low on a
scope too - never producing an observable high pulse. Clock Event Channel
A is set to "100ns pulse" mode rather than "toggle": that's the actual PPS
convention (a sharp edge marking each second boundary), not a 0.5Hz square
wave.

Validated on a real board: `ptp pps` armed the output, and a scope
confirmed a clean pulse train at very close to exactly 1 second intervals
(measured ~4.56ppm fast on one board - well inside the crystal's own
±40ppm datasheet tolerance, and easily correctable later via
`CLOCK_RATE_ADJ` - bit 31 = direction, 0=slower/9ns increments,
1=faster/11ns increments; bits 29:0 = adjustment value in units of 2⁻³²ns
added to the 32-bit `CLOCK_SUBNS` accumulator every 10ns reference tick,
nudging that tick's nanoseconds increment by ±1ns each time the
accumulator overflows - not applied here since the as-shipped accuracy was
already considered acceptable).

### Zephyr `ptp_clock` driver integration (`drivers/eth_lan9250/ptp_clock_lan9250.c`)

Zephyr's `ptp_clock_driver_api` (`.set`/`.get`/`.adjust`/`.rate_adjust`) is
implemented by a **separate, non-devicetree-backed `struct device`**, not
by the LAN9250 Ethernet device itself - a `struct device` has exactly one
`.api` vtable, and the LAN9250's is already `ethernet_api`. This mirrors
Zephyr's own `drivers/ethernet/eth_stm32_hal_ptp.c`, which has the
identical problem (an internal-to-the-MAC PTP clock, no separate physical
block to bind a devicetree node to): a plain `DEVICE_DEFINE()` with no
devicetree node at all, whose init function reaches into the Ethernet
driver's own runtime data and stashes a pointer to itself there
(`lan9250_runtime::ptp_clock`), so `eth_lan9250.c`'s `.get_ptp_clock`
callback has something to return. Register-level operations
(`lan9250_ptp_clock_set/adjust/rate_adjust()`) live in `eth_lan9250.c`
itself, next to `lan9250_ptp_clock_read()` from phase 1, since they need
the driver's private `lan9250_read_sys_reg()`/`lan9250_write_sys_reg()`;
`ptp_clock_lan9250.c` is just the thin API-shape adapter.

One real pitfall hit here: `DEVICE_DEFINE()`'s init-priority argument is
used in preprocessor token-pasting to build the init-level linker section
name, so it must be a literal integer token - `CONFIG_ETH_INIT_PRIORITY + 1`
(intended to just mean "sometime after the Ethernet driver's own init")
compiles fine but fails at *link* time with a cryptic `Undefined
initialization levels used` error. Fixed by hardcoding the literal `61`
(one past `CONFIG_ETH_INIT_PRIORITY=60`) with a comment explaining why -
same category of "looks like it should work, only breaks visibly at a
much later build step" issue as the `RELOAD_ADD_A/B` bit-position mistake
above.

`.adjust()` (a signed nanosecond step, any magnitude) is implemented as a
plain software read-modify-write (`CLOCK_READ`, adjust in software,
`CLOCK_LOAD`) rather than the hardware's single-tick `CLOCK_STEP_ADJ`
mechanism: that mechanism only supports subtraction via its
seconds-portion step (the nanoseconds-portion step is addition-only, per
the datasheet), which would need explicit nanosecond/second-borrow
composition to support arbitrary signed steps correctly. The
read-modify-write approach is trivially correct for any `increment_ns`
value, at the cost of the sub-tick timing precision the hardware
mechanism would otherwise offer - an acceptable trade for the step sizes
a PTP servo actually uses, which are dwarfed by normal SPI transaction
latency anyway.

`CONFIG_PTP_CLOCK_SHELL=y` gives a generic `ptp_clock get/set/adj/freq/
selftest <device>` shell interface for free (device name is
`PTP_CLOCK_NAME`, i.e. `"PTP_CLOCK"`) - used for testing `.set`/
`.adjust`/`.rate_adjust` instead of hand-writing equivalents ourselves.
Note a small upstream (Zephyr) documentation bug encountered while
testing: `ptp_clock adj <device> <value>`'s help text says `<seconds>`,
but the value is actually nanoseconds (it calls `ptp_clock_adjust()`
directly, whose own doc comment says nanoseconds) - not this project's
bug, just worth knowing when testing.

### Hardware RX packet timestamping (phase 4, `lan9250_1588_rx_timestamp_check()`)

Getting a real hardware ingress timestamp attached to an actual inbound
PTP frame required three independent bugs to be found and fixed, none of
which were visible from reading the datasheet alone - each only showed up
against genuine `ptp4l` traffic on the wire.

**Multicast join capacity.** PTP-over-UDP needs two more IPv4 multicast
joins (`224.0.1.129` general/event, `224.0.0.107` peer-delay) on top of
IGMP's own reserved all-systems address and mDNS's `224.0.0.251`.
`CONFIG_NET_IF_MCAST_IPV4_ADDR_COUNT` defaults to only 2, leaving a single
free slot already spoken for - both PTP joins failed with `-ENOMEM`
(`net ipv4` showed only 2 of the 4 expected groups; console log confirmed
`-12`). Fixed by raising the count to 5 in `prj.conf`, and by extracting
mDNS's existing forced-rejoin helper into `force_multicast_rejoin()`
(`eth_id.c`) so PTP's `ptp_multicast_rejoin()` could reuse the identical
join-timing and refcount-draining logic rather than duplicating it - see
"Forcing a real IGMP join" above.

**1588 register configuration ordering.** Several `1588_GENERAL_CONFIG`,
`1588_RX_TIMESTAMP_CONFIG`, and `1588_TX_TIMESTAMP_CONFIG` bit-field
descriptions explicitly state the host must not change them while
`1588_CMD_CTL`'s `1588_ENABLE` bit is set - but `lan9250_1588_init()`
enabled the unit *before* phase 1's `lan9250_1588_timestamping_init()`
configured them. Fixed by extracting a separate `lan9250_1588_enable()`
called last, after both init functions complete, and by switching the two
timestamp-config writes from a blind whole-register overwrite to
read-modify-write (the original blindly zeroed `RX_PTP_VERSION`/
`TX_PTP_VERSION` from their reset default of `2h` as a side effect, even
though those particular bits aren't among the ones the datasheet calls out
as enable-gated).

**The real remaining bug: capture/drain ordering.** The 1588 unit's
ingress-timestamp capture (`RX_INGRESS_SEC/NS` + `RX_MSG_HEADER`, up to 4
events buffered in `CAP_INFO`'s `RX_TS_CNT`) happens at wire speed,
independent of this driver's frame reads - which are comparatively slow
and strictly serialized one at a time over a 10MHz SPI bus. Checking
`CAP_INFO` once, immediately after reading a frame, assumed "the newest
queued event" and "the frame just read" were the same thing; against real
traffic that assumption failed constantly - confirmed via a real
mirrored-switch-port Wireshark capture that genuine PTP frames were
reaching the driver and parsing correctly, while the hardware capture for
a given Sync frame would routinely surface one or more *other* frames'
worth of processing later, correlated against the wrong, by-then-stale
packet. A bounded busy-wait retry (tested up to 1ms, 10x the first
attempt) made no measurable difference to the match rate, ruling out a
fixed short pipeline delay as the cause - the two paths can be out of
step by more than a second under real traffic.

Closed instead with a symmetric pending-match design in
`lan9250_runtime`: `rx_pending` (packets read off the wire whose matching
hardware event hasn't shown up yet - kept alive past normal delivery via
an extra `net_pkt_ref()`) and `rx_unclaimed` (hardware events that showed
up before their own frame had even been read out of the FIFO - the
reverse case, confirmed to happen because physical reception and this
driver's SPI drain really can complete out of order relative to each
other). Both lists are bounded to `LAN9250_1588_RX_PENDING_MAX` (4,
matching the hardware's own capture depth) and self-expire after
`LAN9250_1588_RX_PENDING_TIMEOUT_MS` (2s) so a genuinely lost counterpart
can't hold a `net_pkt` reference or a slot forever. Every call now checks,
in order: does `rx_unclaimed` already have this frame's event; does a
freshly drained `CAP_INFO` event match this frame directly; does it match
something in `rx_pending`; only if none of those hit does the current
frame get added to `rx_pending` for a later call to find.

Verified against a real `ptp4l` grandmaster (`network_transport L2`,
`time_stamping software`, this project's own `ptp4l` skill) with a
concurrent Wireshark capture on a mirrored switch port confirming the
traffic actually on the wire: 23/23 Sync frames correctly timestamped in
one run (19 matched immediately, 4 via the `rx_unclaimed` path), zero
drops, zero expirations, zero list evictions.

### Hardware TX packet timestamping (phase 4, `lan9250_1588_tx_timestamp_check()`)

TX timestamping still uses the original bounded-retry polling design
(`LAN9250_1588_TX_TS_POLL_ATTEMPTS`/`_DELAY`, 5 attempts × 1ms) rather
than RX's `rx_pending`/`rx_unclaimed` redesign - it hasn't needed it in
practice, since TX traffic here is driven by an explicit shell command
(`ptp txtest`) rather than a continuous real-world stream, so the
capture/drain-ordering race RX had to solve for essentially never comes
up. Confirmed against real transmitted frames (`ptp txtest`, `src/ptp_shell.c`)
with a concurrent Wireshark capture on a mirrored switch port confirming
the traffic actually left the wire.

Getting a real hardware TX egress timestamp required finding one thing
the datasheet doesn't document: **`messageLength` (PTP common header
bytes 2-3) must be a real, non-zero value.** A test frame with messageType
and versionPTP both correct but messageLength left at 0 transmits
perfectly fine and is recognized by this driver's own parser, but the
LAN9250 silently never records an egress timestamp for it - `CAP_INFO`'s
`TX_TS_CNT` stays at exactly `0x0`, indefinitely, no matter how long you
wait or how many frames you send. This isn't in the datasheet's own list
of TX egress-recording gating conditions (section 14.2.2.3: messageType
enable, versionPTP match, domain match, alt-master, FCS/checksum) - it's
a real requirement the datasheet just doesn't mention. Setting
messageLength to 44 (correct for a real Sync message, even though the
test frame's payload only actually contains the first 32 bytes) fixed it
immediately and reproducibly (5/5 sends captured correctly in the
confirming run).

Ruling this in took an extensive process of elimination first: config
registers all read back exactly as documented (`CMD_CTL`, `GENERAL_CONFIG`,
`TX_TIMESTAMP_CONFIG`'s message-type-enable and version-match fields,
`TX_PARSE_CONFIG`'s L2/address enables) with nothing amiss; the frame was
confirmed reaching the wire correctly via a mirrored-port capture; no
on-the-fly one-step timestamp insertion was silently happening instead
(checked the actual wire bytes where that would land - untouched);
forcing `TX_PTP_FCS_DIS` and proactively clearing `1588_INT_STS`'s
`TX_TS_INT` both made no difference; five independent real-world LAN9250
driver implementations (Zephyr upstream, NuttX, CycloneTCP, Microchip's
own official reference driver, and mainline Linux's `smsc911x`) were
checked and none implement TX hardware timestamping at all, so no
external reference existed to compare against.

One self-inflicted wrinkle along the way: an earlier diagnostic added to
test a "stale/stuck buffer" theory proactively wrote `1588_INT_STS`'s
`TX_TS_INT` bit to clear it before checking `CAP_INFO` - but per the
datasheet, writing that bit is exactly the action that decrements
`CAP_INFO`'s `TX_TS_CNT`. That diagnostic was silently consuming the one
real capture event before the actual check ever saw it, on every single
test run after it was added, until the ordering bug was caught (the
tell: `INT_STS` read back with `TX_TS_INT` genuinely `SET` for the first
time, immediately followed by `CAP_INFO` reading `0` anyway).

UDP/IPv4-framed PTP TX (dst port 319, 224.0.1.129) was also tried, with
the same `messageLength` fix applied - and did not produce a capture.
Wireshark confirmed the synthetic UDP/IPv4 frame was well-formed at every
layer that matters (valid IPv4 header/checksum, valid UDP header, correctly
dissected as `eth:ethertype:ip:udp:ptp`) and this driver's own parser
recognized it correctly, so this is a genuine, separate, unresolved gap -
left as a known limitation. TX timestamping is confirmed working for
L2-framed PTP only; `ptp txtest` sends L2 frames exclusively.
