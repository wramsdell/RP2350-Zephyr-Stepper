/*
 * Minimal, self-contained RP2350 register access for the bare-metal core1
 * image.
 *
 * Deliberately does NOT include the hardware/structs headers or
 * hardware/address_mapped.h from the vendored Pico SDK: those pull in
 * pico.h, which in this Zephyr checkout drags in Zephyr-only shim headers
 * (zephyr/modules/hal_rpi_pico/pico/config_autogen.h -> zephyr/toolchain.h
 * etc.) that don't exist in a freestanding, non-Zephyr build. The plain
 * hardware/regs headers are pure, dependency-free macro files (no #include
 * beyond each other), so this uses those directly.
 *
 * Field offsets/bit values below are cross-checked against the real
 * generated headers at:
 *   modules/hal/rpi_pico/src/rp2350/hardware_regs/include/hardware/regs/
 */
#ifndef CORE1_REGS_H
#define CORE1_REGS_H

#include <stdint.h>

#include <hardware/regs/addressmap.h>
#include <hardware/regs/sio.h>
#include <hardware/regs/psm.h>
#include <hardware/regs/pads_bank0.h>
#include <hardware/regs/io_bank0.h>
#include <hardware/regs/resets.h>
#include <hardware/regs/timer.h>
#include <hardware/regs/pio.h>

#define REG(addr) (*(volatile uint32_t *)(addr))

/* REG_ALIAS_{XOR,SET,CLR}_BITS (atomic register access aliases) come from
 * hardware/regs/addressmap.h, included above. */
#define REG_SET(addr, mask) (REG((addr) + REG_ALIAS_SET_BITS) = (mask))
#define REG_CLR(addr, mask) (REG((addr) + REG_ALIAS_CLR_BITS) = (mask))

#define SIO_GPIO_OUT     (SIO_BASE + SIO_GPIO_OUT_OFFSET)
#define SIO_GPIO_OUT_SET (SIO_BASE + SIO_GPIO_OUT_SET_OFFSET)
#define SIO_GPIO_OUT_CLR (SIO_BASE + SIO_GPIO_OUT_CLR_OFFSET)
#define SIO_GPIO_OUT_XOR (SIO_BASE + SIO_GPIO_OUT_XOR_OFFSET)
#define SIO_GPIO_OE_SET  (SIO_BASE + SIO_GPIO_OE_SET_OFFSET)
#define SIO_FIFO_ST      (SIO_BASE + SIO_FIFO_ST_OFFSET)
#define SIO_FIFO_WR      (SIO_BASE + SIO_FIFO_WR_OFFSET)
#define SIO_FIFO_RD      (SIO_BASE + SIO_FIFO_RD_OFFSET)

#define PSM_FRCE_OFF (PSM_BASE + PSM_FRCE_OFF_OFFSET)

#define PADS_BANK0_GPIO(n) (PADS_BANK0_BASE + PADS_BANK0_GPIO0_OFFSET + 4u * (n))
#define IO_BANK0_GPIO_CTRL(n) (IO_BANK0_BASE + IO_BANK0_GPIO0_CTRL_OFFSET + 8u * (n))

#define RESETS_RESET      (RESETS_BASE + RESETS_RESET_OFFSET)
#define RESETS_RESET_DONE (RESETS_BASE + RESETS_RESET_DONE_OFFSET)

#define TIMER0_TIMERAWL (TIMER0_BASE + TIMER_TIMERAWL_OFFSET)

/*
 * PIO0 - used to generate STEP pulses. Only the handful of registers/values
 * the stepper program needs; see core1_main.c for the full explanation of
 * this register sequence (it's cross-checked against the Pico SDK's own
 * hardware_pio implementation, not guessed).
 */
#define PIO0_CTRL       (PIO0_BASE + PIO_CTRL_OFFSET)
#define PIO0_FSTAT      (PIO0_BASE + PIO_FSTAT_OFFSET)
#define PIO0_TXF0       (PIO0_BASE + PIO_TXF0_OFFSET)
#define PIO0_INSTR_MEM0 (PIO0_BASE + PIO_INSTR_MEM0_OFFSET)
#define PIO0_SM0_EXECCTRL  (PIO0_BASE + PIO_SM0_EXECCTRL_OFFSET)
#define PIO0_SM0_INSTR     (PIO0_BASE + PIO_SM0_INSTR_OFFSET)
#define PIO0_SM0_PINCTRL   (PIO0_BASE + PIO_SM0_PINCTRL_OFFSET)

/*
 * M33_CPACR (0xE000ED88) is the standard ARMv8-M private-peripheral-bus
 * Coprocessor Access Control Register (RP2350's own M33_CPACR_OFFSET
 * register definition resolves to this same, architecturally standard,
 * address - it isn't an RP2350-specific register). Each core's copy is
 * private, so core1 must enable its own FPU access before using any
 * floating point code; core0/Zephyr does the equivalent via
 * runtime_init_per_core_enable_coprocessors() in the Pico SDK.
 */
#define M33_CPACR 0xE000ED88u
#define M33_CPACR_CP10_BITS 0x00300000u /* full access, CP10 (FPU) */

#endif /* CORE1_REGS_H */
