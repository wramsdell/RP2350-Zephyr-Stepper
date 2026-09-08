/* LAN9250 Stand-alone Ethernet Controller with SPI
 *
 * Copyright (c) 2024 Mario Paja
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

#ifndef _LAN9250_
#define _LAN9250_

#define LAN9250_DEFAULT_NUMOF_RETRIES 3U
#define LAN9250_PHY_TIMEOUT           2000
#define LAN9250_MAC_TIMEOUT           2000
#define LAN9250_RESET_TIMEOUT         5000

#define LAN9250_ALIGN(v) (((v) + 3) & (~3))

/* SPI instructions */
#define LAN9250_SPI_INSTR_WRITE 0x02
#define LAN9250_SPI_INSTR_READ  0x03

/* TX command 'A' format */
#define LAN9250_TX_CMD_A_INT_ON_COMP     0x80000000
#define LAN9250_TX_CMD_A_BUFFER_ALIGN_4B 0x00000000
#define LAN9250_TX_CMD_A_START_OFFSET_0B 0x00000000
#define LAN9250_TX_CMD_A_FIRST_SEG       0x00002000
#define LAN9250_TX_CMD_A_LAST_SEG        0x00001000

/* TX command 'B' format */
#define LAN9250_TX_CMD_B_PACKET_TAG 0xFFFF0000

/* RX status format */
#define LAN9250_RX_STS_PACKET_LEN 0x3FFF0000

/* LAN9250 System registers */
#define LAN9250_RX_DATA_FIFO   0x0000
#define LAN9250_TX_DATA_FIFO   0x0020
#define LAN9250_RX_STATUS_FIFO 0x0040
#define LAN9250_TX_STATUS_FIFO 0x0048
#define LAN9250_ID_REV         0x0050
#define LAN9250_IRQ_CFG        0x0054
#define LAN9250_INT_STS        0x0058
#define LAN9250_INT_EN         0x005C
#define LAN9250_BYTE_TEST      0x0064
#define LAN9250_FIFO_INT       0x0068
#define LAN9250_RX_CFG         0x006C
#define LAN9250_TX_CFG         0x0070
#define LAN9250_HW_CFG         0x0074
#define LAN9250_RX_FIFO_INF    0x007C
#define LAN9250_TX_FIFO_INF    0x0080
#define LAN9250_PMT_CTRL       0x0084
#define LAN9250_MAC_CSR_CMD    0x00A4
#define LAN9250_MAC_CSR_DATA   0x00A8
#define LAN9250_AFC_CFG        0x00AC
#define LAN9250_RESET_CTL      0x01F8
#define LAN9250_LED_CFG        0x01BC
#define LAN9250_GPIO_CFG       0x01E0
#define LAN9250_GPIO_DATA_DIR  0x01E4
#define LAN9250_GPIO_INT_STS_EN 0x01E8

/* LED_CFG bits (datasheet Section 16.4.1) - LED_EN(x): 0 = pin x is a
 * plain GPIO, 1 = pin x is an LED output. x is 0/1/2 for GPIO0/1/2.
 */
#define LAN9250_LED_CFG_LED_EN(x) (1u << (x))

/* GPIO_CFG bits (datasheet Section 16.4.2), one bit per GPIO pin unless
 * noted. x is 0/1/2 for GPIO0/1/2.
 */
#define LAN9250_GPIO_CFG_1588_GPIO_CH_SEL(x) (1u << (24 + (x))) /* 0=ch A, 1=ch B */
#define LAN9250_GPIO_CFG_GPIO_POL(x)         (1u << (16 + (x))) /* 1=active high */
#define LAN9250_GPIO_CFG_1588_GPIO_OE(x)     (1u << (8 + (x)))  /* 1=1588 event drives pin */
#define LAN9250_GPIO_CFG_GPIOBUF(x)          (1u << (x))        /* 1=push/pull, 0=open-drain */

/*
 * LAN9250 IEEE 1588 (PTP) registers - datasheet Section 14.8, "1588
 * Registers". These are directly-addressed system registers ("Bank: na"
 * in the datasheet's Table 14-1), reached the same way as PMT_CTRL etc.
 * above (lan9250_read_sys_reg()/lan9250_write_sys_reg()) - no MAC-CSR
 * indirection and no bank-select needed for this subset. Per-port
 * RX/TX timestamp config (Banks 0-2) and per-GPIO capture registers
 * (Bank 3), reached via 1588_BANK_PORT_GPIO_SEL, are not yet used here.
 */
#define LAN9250_1588_CMD_CTL             0x0100
#define LAN9250_1588_GENERAL_CONFIG      0x0104
#define LAN9250_1588_INT_STS             0x0108
#define LAN9250_1588_INT_EN              0x010C
#define LAN9250_1588_CLOCK_SEC           0x0110
#define LAN9250_1588_CLOCK_NS            0x0114
#define LAN9250_1588_CLOCK_SUBNS         0x0118
#define LAN9250_1588_CLOCK_RATE_ADJ      0x011C
#define LAN9250_1588_CLOCK_TEMP_RATE_ADJ 0x0120
#define LAN9250_1588_CLOCK_TEMP_RATE_DURATION 0x0124
#define LAN9250_1588_CLOCK_STEP_ADJ      0x0128
/* Clock Target/Reload-Add register pairs: x=A at 0x12C.., x=B at 0x13C.. */
#define LAN9250_1588_CLOCK_TARGET_SEC(x)        (0x012C + 0x10 * (x))
#define LAN9250_1588_CLOCK_TARGET_NS(x)         (0x0130 + 0x10 * (x))
#define LAN9250_1588_CLOCK_TARGET_RELOAD_SEC(x) (0x0134 + 0x10 * (x))
#define LAN9250_1588_CLOCK_TARGET_RELOAD_NS(x)  (0x0138 + 0x10 * (x))
/* x: 0 = channel A, 1 = channel B (datasheet uses "A"/"B" - offsets above
 * are literal, do not use this macro for the target/reload registers). */

/* 1588_CLOCK_RATE_ADJ bits (permanent rate trim). Bits 29:0 are added to
 * the 32-bit CLOCK_SUBNS accumulator every 10ns reference tick; each time
 * it overflows, that tick's nanoseconds increment is nudged by +-1ns
 * instead of the normal 10ns.
 */
#define LAN9250_1588_CLOCK_RATE_ADJ_DIR        0x80000000 /* 0=slower(9ns), 1=faster(11ns) */
#define LAN9250_1588_CLOCK_RATE_ADJ_VALUE_MASK 0x3FFFFFFF

/* 1588_CLOCK_STEP_ADJ bits (one-time step). DIR applies to both the
 * seconds- and nanoseconds-portion step commands in 1588_CMD_CTL, but
 * per the datasheet only addition (DIR=1) is supported for the
 * nanoseconds portion - subtraction (DIR=0) is only meaningful for the
 * seconds-portion step. For the ns-portion step, VALUE replaces (not
 * adds to) that one tick's normal ~10ns increment - to get an exact net
 * step of +X ns, write VALUE = X + 10. For the seconds-portion step,
 * only the low 4 bits of VALUE are used (max +-15s per step).
 */
#define LAN9250_1588_CLOCK_STEP_ADJ_DIR        0x80000000 /* 0=subtracted, 1=added */
#define LAN9250_1588_CLOCK_STEP_ADJ_VALUE_MASK 0x3FFFFFFF

/* 1588_INT_STS bits (also 1588_INT_EN, same bit positions) */
#define LAN9250_1588_INT_STS_TX_TS_INT 0x00001000 /* bit 12 */
#define LAN9250_1588_INT_STS_RX_TS_INT 0x00000100 /* bit 8 */

/*
 * 1588 Port/GPIO banked registers (datasheet Section 14.8.18 onward).
 * Port and GPIO registers share one address window (0x158-0x18C);
 * 1588_BANK_PORT_GPIO_SEL's BANK_SEL[2:0] selects which bank is currently
 * mapped there. Must be written before accessing any register below, and
 * accessing a "Port General"/"Port RX"/"Port TX" register requires having
 * selected that bank first - the offsets themselves are reused across
 * banks.
 */
#define LAN9250_1588_BANK_PORT_GPIO_SEL 0x0154
#define LAN9250_1588_BANK_SEL_PORT_GENERAL 0
#define LAN9250_1588_BANK_SEL_PORT_RX      1
#define LAN9250_1588_BANK_SEL_PORT_TX      2
#define LAN9250_1588_BANK_SEL_GPIOS        3

/* Bank 0 ("Port General") */
#define LAN9250_1588_LATENCY      0x0158
#define LAN9250_1588_ASYM_PEERDLY 0x015C
#define LAN9250_1588_CAP_INFO     0x0160
#define LAN9250_1588_CAP_INFO_TX_TS_CNT_MASK 0x00000070 /* bits 6:4 */
#define LAN9250_1588_CAP_INFO_TX_TS_CNT_SHIFT 4
#define LAN9250_1588_CAP_INFO_RX_TS_CNT_MASK 0x00000007 /* bits 2:0 */

/* Bank 1 ("Port RX") */
#define LAN9250_1588_RX_PARSE_CONFIG      0x0158
#define LAN9250_1588_RX_TIMESTAMP_CONFIG  0x015C
#define LAN9250_1588_RX_TIMESTAMP_CONFIG_MESSAGE_EN_MASK 0x0000FFFF
#define LAN9250_1588_RX_TS_INSERT_CONFIG  0x0160
#define LAN9250_1588_RX_FILTER_CONFIG     0x0168
#define LAN9250_1588_RX_INGRESS_SEC       0x016C
#define LAN9250_1588_RX_INGRESS_NS        0x0170
#define LAN9250_1588_RX_MSG_HEADER        0x0174
#define LAN9250_1588_MSG_HEADER_MSG_TYPE_MASK 0x000F0000 /* bits 19:16 */
#define LAN9250_1588_MSG_HEADER_MSG_TYPE_SHIFT 16
#define LAN9250_1588_MSG_HEADER_SEQ_ID_MASK    0x0000FFFF /* bits 15:0 */

/* Bank 2 ("Port TX") */
#define LAN9250_1588_TX_PARSE_CONFIG      0x0158
#define LAN9250_1588_TX_TIMESTAMP_CONFIG  0x015C
#define LAN9250_1588_TX_TIMESTAMP_CONFIG_MESSAGE_EN_MASK 0x0000FFFF
#define LAN9250_1588_TX_EGRESS_SEC        0x016C
#define LAN9250_1588_TX_EGRESS_NS         0x0170
#define LAN9250_1588_TX_MSG_HEADER        0x0174
/* TX_MSG_HEADER shares RX_MSG_HEADER's bit layout (MSG_HEADER_* above). */

/* PTP messageType values needed for RX/TX message-type-enable masks
 * (IEEE 1588-2008 Table 19) - only the four types this driver enables
 * timestamping for.
 */
#define LAN9250_1588_PTP_MSGTYPE_SYNC        0x0
#define LAN9250_1588_PTP_MSGTYPE_DELAY_REQ   0x1
#define LAN9250_1588_PTP_MSGTYPE_PDELAY_REQ  0x2
#define LAN9250_1588_PTP_MSGTYPE_PDELAY_RESP 0x3
#define LAN9250_1588_PTP_MESSAGE_EN_DEFAULT \
	(BIT(LAN9250_1588_PTP_MSGTYPE_SYNC) | BIT(LAN9250_1588_PTP_MSGTYPE_DELAY_REQ) | \
	 BIT(LAN9250_1588_PTP_MSGTYPE_PDELAY_REQ) | BIT(LAN9250_1588_PTP_MSGTYPE_PDELAY_RESP))

/* 1588_CMD_CTL bits */
#define LAN9250_1588_CMD_CTL_CLOCK_TARGET_READ 0x00002000
#define LAN9250_1588_CMD_CTL_CLOCK_TEMP_RATE   0x00000080
#define LAN9250_1588_CMD_CTL_CLOCK_STEP_NS     0x00000040
#define LAN9250_1588_CMD_CTL_CLOCK_STEP_SEC    0x00000020
#define LAN9250_1588_CMD_CTL_CLOCK_LOAD        0x00000010
#define LAN9250_1588_CMD_CTL_CLOCK_READ        0x00000008
#define LAN9250_1588_CMD_CTL_ENABLE            0x00000004
#define LAN9250_1588_CMD_CTL_DISABLE           0x00000002
#define LAN9250_1588_CMD_CTL_RESET             0x00000001

/* 1588_GENERAL_CONFIG bits.
 *
 * RELOAD_ADD_A/B polarity per the datasheet is the opposite of what the
 * names suggest at a glance: 0 = increment the Clock Target by the
 * Reload/Add registers on every compare event (auto-repeating - what a
 * free-running PPS output needs), 1 = reload the Clock Target from the
 * Reload/Add registers on the next event (a one-shot pre-load).
 */
#define LAN9250_1588_GENERAL_CONFIG_TSU_ENABLE      0x00010000
#define LAN9250_1588_GENERAL_CONFIG_RELOAD_ADD_B    0x00000002
#define LAN9250_1588_GENERAL_CONFIG_RELOAD_ADD_A    0x00000001
#define LAN9250_1588_GENERAL_CONFIG_CLOCK_EVENT_B_MASK 0x00000030
#define LAN9250_1588_GENERAL_CONFIG_CLOCK_EVENT_B_SHIFT 4
#define LAN9250_1588_GENERAL_CONFIG_CLOCK_EVENT_A_MASK 0x0000000C
#define LAN9250_1588_GENERAL_CONFIG_CLOCK_EVENT_A_SHIFT 2
#define LAN9250_1588_CLOCK_EVENT_MODE_100NS_PULSE 0x0
#define LAN9250_1588_CLOCK_EVENT_MODE_TOGGLE      0x1
#define LAN9250_1588_CLOCK_EVENT_MODE_INT_BIT     0x2

/* LAN9250 Host MAC registers (datasheet DS00001913C, Table 11-17, page
 * 191 - indirect addresses, reached via MAC_CSR_CMD/MAC_CSR_DATA).
 */
#define LAN9250_HMAC_CR       0x01
#define LAN9250_HMAC_ADDRH    0x02
#define LAN9250_HMAC_ADDRL    0x03
#define LAN9250_HMAC_HASHH    0x04
#define LAN9250_HMAC_HASHL    0x05
#define LAN9250_HMAC_MII_ACC  0x06
#define LAN9250_HMAC_MII_DATA 0x07

/* LAN9250 PHY registers */
#define LAN9250_PHY_BASIC_CONTROL            0x00
#define LAN9250_PHY_AN_ADV                   0x04
#define LAN9250_PHY_SPECIAL_MODES            0x12
#define LAN9250_PHY_SPECIAL_CONTROL_STAT_IND 0x1B
#define LAN9250_PHY_INTERRUPT_SOURCE         0x1D
#define LAN9250_PHY_INTERRUPT_MASK           0x1E
#define LAN9250_PHY_SPECIAL_CONTROL_STATUS   0x1F

/* Interrupt Configuration register */
#define LAN9250_IRQ_CFG_INT_DEAS       0xFF000000
#define LAN9250_IRQ_CFG_INT_DEAS_10US  0x01000000
#define LAN9250_IRQ_CFG_INT_DEAS_100US 0x0A000000
#define LAN9250_IRQ_CFG_INT_DEAS_1MS   0x64000000
#define LAN9250_IRQ_CFG_INT_DEAS_CLR   0x00004000
#define LAN9250_IRQ_CFG_INT_DEAS_STS   0x00002000
#define LAN9250_IRQ_CFG_IRQ_INT        0x00001000
#define LAN9250_IRQ_CFG_IRQ_EN         0x00000100
#define LAN9250_IRQ_CFG_IRQ_POL        0x00000010
#define LAN9250_IRQ_CFG_IRQ_POL_LOW    0x00000000
#define LAN9250_IRQ_CFG_IRQ_POL_HIGH   0x00000010
#define LAN9250_IRQ_CFG_IRQ_CLK_SELECT 0x00000002
#define LAN9250_IRQ_CFG_IRQ_TYPE       0x00000001
#define LAN9250_IRQ_CFG_IRQ_TYPE_OD    0x00000000
#define LAN9250_IRQ_CFG_IRQ_TYPE_PP    0x00000001

/* INTERRUPT STATUS REGISTER (INT_STS) */
#define LAN9250_INT_STS_SW_INT     0x80000000
#define LAN9250_INT_STS_READY      0x40000000
#define LAN9250_INT_STS_1588_EVNT  0x20000000
#define LAN9250_INT_STS_PHY_INT    0x04000000
#define LAN9250_INT_STS_TXSTOP_INT 0x02000000
#define LAN9250_INT_STS_RXSTOP_INT 0x01000000
#define LAN9250_INT_STS_RXDFH_INT  0x00800000
#define LAN9250_INT_STS_TX_IOC     0x00200000
#define LAN9250_INT_STS_RXD_INT    0x00100000
#define LAN9250_INT_STS_GPT_INT    0x00080000
#define LAN9250_INT_STS_PME_INT    0x00020000
#define LAN9250_INT_STS_TXSO       0x00010000
#define LAN9250_INT_STS_RWT        0x00008000
#define LAN9250_INT_STS_RXE        0x00004000
#define LAN9250_INT_STS_TXE        0x00002000
#define LAN9250_INT_STS_GPIO       0x00001000
#define LAN9250_INT_STS_TDFO       0x00000400
#define LAN9250_INT_STS_TDFA       0x00000200
#define LAN9250_INT_STS_TSFF       0x00000100
#define LAN9250_INT_STS_TSFL       0x00000080
#define LAN9250_INT_STS_RXDF_INT   0x00000040
#define LAN9250_INT_STS_RSFF       0x00000010
#define LAN9250_INT_STS_RSFL       0x00000008

/* INTERRUPT ENABLE REGISTER (INT_EN) */
#define LAN9250_INT_EN_SW_INT_EN     0x80000000
#define LAN9250_INT_EN_READY_EN      0x40000000
#define LAN9250_INT_EN_1588_EVNT_EN  0x20000000
#define LAN9250_INT_EN_PHY_INT_EN    0x04000000
#define LAN9250_INT_EN_TXSTOP_INT_EN 0x02000000
#define LAN9250_INT_EN_RXSTOP_INT_EN 0x01000000
#define LAN9250_INT_EN_RXDFH_INT_EN  0x00800000
#define LAN9250_INT_EN_TIOC_INT_EN   0x00200000
#define LAN9250_INT_EN_RXD_INT_EN    0x00100000
#define LAN9250_INT_EN_GPT_INT_EN    0x00080000
#define LAN9250_INT_EN_PME_INT_EN    0x00020000
#define LAN9250_INT_EN_TXSO_EN       0x00010000
#define LAN9250_INT_EN_RWT_INT_EN    0x00008000
#define LAN9250_INT_EN_RXE_INT_EN    0x00004000
#define LAN9250_INT_EN_TXE_INT_EN    0x00002000
#define LAN9250_INT_EN_GPIO_EN       0x00001000
#define LAN9250_INT_EN_TDFO_EN       0x00000400
#define LAN9250_INT_EN_TDFA_EN       0x00000200
#define LAN9250_INT_EN_TSFF_EN       0x00000100
#define LAN9250_INT_EN_TSFL_EN       0x00000080
#define LAN9250_INT_EN_RXDF_INT_EN   0x00000040
#define LAN9250_INT_EN_RSFF_EN       0x00000010
#define LAN9250_INT_EN_RSFL_EN       0x00000008

/* Byte Order Test register */
#define LAN9250_BYTE_TEST_DEFAULT 0x87654321
#define BOTR_MASK                 0xffffffff

/* FIFO Level Interrupt register */
#define LAN9250_FIFO_INT_TX_DATA_AVAILABLE_LEVEL 0xFF000000
#define LAN9250_FIFO_INT_TX_STATUS_LEVEL         0x00FF0000
#define LAN9250_FIFO_INT_RX_STATUS_LEVEL         0x000000FF

/* TRANSMIT CONFIGURATION REGISTER (TX_CFG) */
#define LAN9250_TX_CFG_TXS_DUMP 0x00008000
#define LAN9250_TX_CFG_TXD_DUMP 0x00004000
#define LAN9250_TX_CFG_TXSAO    0x00000004
#define LAN9250_TX_CFG_TX_ON    0x00000002
#define LAN9250_TX_CFG_STOP_TX  0x00000001

/* HARDWARE CONFIGURATION REGISTER (HW_CFG) */
#define LAN9250_HW_CFG_DEVICE_READY         0x08000000
#define LAN9250_HW_CFG_AMDIX_EN_STRAP_STATE 0x02000000
#define LAN9250_HW_CFG_MBO                  0x00100000
#define LAN9250_HW_CFG_TX_FIF_SZ            0x000F0000
#define LAN9250_HW_CFG_TX_FIF_SZ_2KB        0x00020000
#define LAN9250_HW_CFG_TX_FIF_SZ_3KB        0x00030000
#define LAN9250_HW_CFG_TX_FIF_SZ_4KB        0x00040000
#define LAN9250_HW_CFG_TX_FIF_SZ_5KB        0x00050000
#define LAN9250_HW_CFG_TX_FIF_SZ_6KB        0x00060000
#define LAN9250_HW_CFG_TX_FIF_SZ_7KB        0x00070000
#define LAN9250_HW_CFG_TX_FIF_SZ_8KB        0x00080000
#define LAN9250_HW_CFG_TX_FIF_SZ_9KB        0x00090000
#define LAN9250_HW_CFG_TX_FIF_SZ_10KB       0x000A0000
#define LAN9250_HW_CFG_TX_FIF_SZ_11KB       0x000B0000
#define LAN9250_HW_CFG_TX_FIF_SZ_12KB       0x000C0000
#define LAN9250_HW_CFG_TX_FIF_SZ_13KB       0x000D0000
#define LAN9250_HW_CFG_TX_FIF_SZ_14KB       0x000E0000

/* RX FIFO Information register */
#define LAN9250_RX_FIFO_INF_RXSUSED 0x00FF0000
#define LAN9250_RX_FIFO_INF_RXDUSED 0x0000FFFF

/* TX FIFO Information register */
#define LAN9250_TX_FIFO_INF_TXSUSED 0x00FF0000
#define LAN9250_TX_FIFO_INF_TXFREE  0x0000FFFF

/* Power Management Control Register (PMT_CTRL) */
#define LAN9250_PMT_CTRL_PM_MODE           0xE0000000
#define LAN9250_PMT_CTRL_PM_SLEEP_EN       0x10000000
#define LAN9250_PMT_CTRL_PM_WAKE           0x08000000
#define LAN9250_PMT_CTRL_LED_DIS           0x04000000
#define LAN9250_PMT_CTRL_1588_DIS          0x02000000
#define LAN9250_PMT_CTRL_1588_TSU_DIS      0x00400000
#define LAN9250_PMT_CTRL_HMAC_DIS          0x00080000
#define LAN9250_PMT_CTRL_HMAC_SYS_ONLY_DIS 0x00040000
#define LAN9250_PMT_CTRL_ED_STS            0x00010000
#define LAN9250_PMT_CTRL_ED_EN             0x00004000
#define LAN9250_PMT_CTRL_WOL_EN            0x00000200
#define LAN9250_PMT_CTRL_PME_TYPE          0x00000040
#define LAN9250_PMT_CTRL_WOL_STS           0x00000020
#define LAN9250_PMT_CTRL_PME_IND           0x00000008
#define LAN9250_PMT_CTRL_PME_POL           0x00000004
#define LAN9250_PMT_CTRL_PME_EN            0x00000002
#define LAN9250_PMT_CTRL_READY             0x00000001

/* HOST MAC CSR INTERFACE COMMAND REGISTER (MAC_CSR_CMD) */
#define LAN9250_MAC_CSR_CMD_BUSY  0x80000000
#define LAN9250_MAC_CSR_CMD_WRITE 0x00000000
#define LAN9250_MAC_CSR_CMD_READ  0x40000000
#define LAN9250_MAC_CSR_CMD_ADDR  0x000000FF

/* Reset Control Register (RESET_CTL) */
#define LAN9250_RESET_CTL_HMAC_RST    0x00000020
#define LAN9250_RESET_CTL_PHY_RST     0x00000002
#define LAN9250_RESET_CTL_DIGITAL_RST 0x00000001

/* HOST MAC CONTROL REGISTER (HMAC_CR) */
#define LAN9250_HMAC_CR_RXALL           0x80000000
#define LAN9250_HMAC_CR_HMAC_EEE_ENABLE 0x02000000
#define LAN9250_HMAC_CR_RCVOWN          0x00800000
#define LAN9250_HMAC_CR_LOOPBK          0x00200000
#define LAN9250_HMAC_CR_FDPX            0x00100000
#define LAN9250_HMAC_CR_MCPAS           0x00080000
#define LAN9250_HMAC_CR_PRMS            0x00040000
#define LAN9250_HMAC_CR_INVFILT         0x00020000
#define LAN9250_HMAC_CR_PASSBAD         0x00010000
#define LAN9250_HMAC_CR_HO              0x00008000
#define LAN9250_HMAC_CR_HPFILT          0x00002000
#define LAN9250_HMAC_CR_BCAST           0x00000800
#define LAN9250_HMAC_CR_DISRTY          0x00000400
#define LAN9250_HMAC_CR_PADSTR          0x00000100
#define LAN9250_HMAC_CR_BOLMT           0x000000C0
#define LAN9250_HMAC_CR_BOLMT_10_BITS   0x00000000
#define LAN9250_HMAC_CR_BOLMT_8_BITS    0x00000040
#define LAN9250_HMAC_CR_BOLMT_4_BITS    0x00000080
#define LAN9250_HMAC_CR_BOLMT_1_BIT     0x000000C0
#define LAN9250_HMAC_CR_DFCHK           0x00000020
#define LAN9250_HMAC_CR_TXEN            0x00000008
#define LAN9250_HMAC_CR_RXEN            0x00000004

/* HOST MAC MII ACCESS REGISTER (HMAC_MII_ACC) */
#define LAN9250_HMAC_MII_ACC_PHY_ADDR         0x0000F800
#define LAN9250_HMAC_MII_ACC_PHY_ADDR_DEFAULT 0x00000800
#define LAN9250_HMAC_MII_ACC_MIIRINDA         0x000007C0
#define LAN9250_HMAC_MII_ACC_MIIW_R           0x00000002
#define LAN9250_HMAC_MII_ACC_MIIBZY           0x00000001

/* PHY Basic Control Register (PHY_BASIC_CONTROL) */
#define LAN9250_PHY_BASIC_CONTROL_PHY_SRST          0x8000
#define LAN9250_PHY_BASIC_CONTROL_PHY_LOOPBACK      0x4000
#define LAN9250_PHY_BASIC_CONTROL_PHY_SPEED_SEL_LSB 0x2000
#define LAN9250_PHY_BASIC_CONTROL_PHY_AN            0x1000
#define LAN9250_PHY_BASIC_CONTROL_PHY_PWR_DWN       0x0800
#define LAN9250_PHY_BASIC_CONTROL_PHY_RST_AN        0x0200
#define LAN9250_PHY_BASIC_CONTROL_PHY_DUPLEX        0x0100
#define LAN9250_PHY_BASIC_CONTROL_PHY_COL_TEST      0x0080

/* PHY Auto-Negotiation Advertisement Register (PHY_AN_ADV) */
#define LAN9250_PHY_AN_ADV_NEXT_PAGE          0x8000
#define LAN9250_PHY_AN_ADV_REMOTE_FAULT       0x2000
#define LAN9250_PHY_AN_ADV_EXTENDED_NEXT_PAGE 0x1000
#define LAN9250_PHY_AN_ADV_ASYM_PAUSE         0x0800
#define LAN9250_PHY_AN_ADV_SYM_PAUSE          0x0400
#define LAN9250_PHY_AN_ADV_100BTX_FD          0x0100
#define LAN9250_PHY_AN_ADV_100BTX_HD          0x0080
#define LAN9250_PHY_AN_ADV_10BT_FD            0x0040
#define LAN9250_PHY_AN_ADV_10BT_HD            0x0020
#define LAN9250_PHY_AN_ADV_SELECTOR           0x001F
#define LAN9250_PHY_AN_ADV_SELECTOR_DEFAULT   0x0001

/* PHY Mode Control/Status Register (PHY_MODE_CONTROL_STATUS) */
#define LAN9250_PHY_MODE_CONTROL_STATUS_EDPWRDOWN 0x2000
#define LAN9250_PHY_MODE_CONTROL_STATUS_ALTINT    0x0040
#define LAN9250_PHY_MODE_CONTROL_STATUS_ENERGYON  0x0002

/* PHY Special Control/Status Indication Register (PHY_SPECIAL_CONTROL_STAT_IND) */
#define LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_AMDIXCTRL  0x8000
#define LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_AMDIXEN    0x4000
#define LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_AMDIXSTATE 0x2000
#define LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_SQEOFF     0x0800
#define LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_FEFI_EN    0x0020
#define LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_XPOL       0x0010

/* PHY Interrupt Source Flags Register (PHY_INTERRUPT_SOURCE) */
#define LAN9250_PHY_INTERRUPT_SOURCE_LINK_UP               0x0200
#define LAN9250_PHY_INTERRUPT_SOURCE_ENERGYON              0x0080
#define LAN9250_PHY_INTERRUPT_SOURCE_AN_COMPLETE           0x0040
#define LAN9250_PHY_INTERRUPT_SOURCE_REMOTE_FAULT          0x0020
#define LAN9250_PHY_INTERRUPT_SOURCE_LINK_DOWN             0x0010
#define LAN9250_PHY_INTERRUPT_SOURCE_AN_LP_ACK             0x0008
#define LAN9250_PHY_INTERRUPT_SOURCE_PARALLEL_DETECT_FAULT 0x0004
#define LAN9250_PHY_INTERRUPT_SOURCE_AN_PAGE_RECEIVED      0x0002

/* PHY Interrupt Mask Register (PHY_INTERRUPT_MASK) */
#define LAN9250_PHY_INTERRUPT_MASK_LINK_UP               0x0200
#define LAN9250_PHY_INTERRUPT_MASK_ENERGYON              0x0080
#define LAN9250_PHY_INTERRUPT_MASK_AN_COMPLETE           0x0040
#define LAN9250_PHY_INTERRUPT_MASK_REMOTE_FAULT          0x0020
#define LAN9250_PHY_INTERRUPT_MASK_LINK_DOWN             0x0010
#define LAN9250_PHY_INTERRUPT_MASK_AN_LP_ACK             0x0008
#define LAN9250_PHY_INTERRUPT_MASK_PARALLEL_DETECT_FAULT 0x0004
#define LAN9250_PHY_INTERRUPT_MASK_AN_PAGE_RECEIVED      0x0002

/* Chip ID and Revision register */
#define LAN9250_ID_REV_CHIP_ID         0xFFFF0000
#define LAN9250_ID_REV_CHIP_ID_DEFAULT 0x92500000
#define LAN9250_ID_REV_CHIP_REV        0x0000FFFF

struct lan9250_config {
	struct spi_dt_spec spi;
	struct gpio_dt_spec interrupt;
	struct gpio_dt_spec reset;
	uint8_t full_duplex;
	struct net_eth_mac_config mac_cfg;
};

/* Originally sized to match the LAN9250's own RX timestamp capture buffer
 * depth (CAP_INFO's RX_TS_CNT field, up to 4) - see the comment on
 * lan9250_1588_rx_timestamp_check(). That's the hardware's own physical
 * buffer depth, not a ceiling on how many *software-side* entries can be
 * outstanding at once: rx_pending/rx_unclaimed track frames and events
 * still waiting to be matched across an asynchronous, sometimes
 * multi-second gap, and every PTP frame - not just HW-timestamp-eligible
 * ones - drains one CAP_INFO event per check (see that comment for why).
 * Real gPTP traffic interleaves far more message types (Sync, Follow_Up,
 * Pdelay_Req/Resp, Pdelay_Resp_Follow_Up, Announce) at a much higher
 * combined rate than this project's earlier, single-message-type RX
 * validation against real ptp4l traffic - confirmed via a live two-board
 * gPTP bring-up that plenty of Pdelay_Resp_Follow_Up frames (never
 * HW-timestamped themselves) were draining CAP_INFO events belonging to
 * other, real Pdelay_Req/Resp frames, only for genuine rx_pending/
 * rx_unclaimed entries to then expire unmatched under the resulting
 * pressure on just 4 slots each - starving gPTP's Peer Delay measurement
 * of RX timestamps often enough to prevent its neighborRateRatio from
 * ever going valid, which in turn (see gptp_update_local_port_clock() in
 * Zephyr's own gptp_mi.c) silently skips every ongoing clock correction:
 * the observed symptom was two boards converging roughly on connection
 * and then drifting apart with no visible ongoing correction. Widened
 * well past the hardware's own 4-deep buffer to give real, busy gPTP
 * traffic enough headroom; RAM cost is negligible (a pointer + a few
 * small fields per slot).
 */
#define LAN9250_1588_RX_PENDING_MAX 16

struct lan9250_1588_rx_pending {
	struct net_pkt *pkt; /* NULL = free slot */
	uint8_t msg_type;
	uint16_t seq_id;
	int64_t deadline; /* k_uptime_get(), give up/unref past this */
};

/* The mirror image of lan9250_1588_rx_pending: a hardware timestamp
 * captured for a frame this driver hasn't read out of the SPI RX FIFO yet.
 * See the comment on lan9250_1588_rx_timestamp_check() - physical
 * reception (wire-speed) and this driver's SPI drain (comparatively slow,
 * serialized per frame) can complete out of order, so a capture can be
 * ready before its own frame's net_pkt even exists yet.
 */
struct lan9250_1588_rx_unclaimed {
	bool valid;
	uint8_t msg_type;
	uint16_t seq_id;
	uint32_t sec;
	uint32_t ns;
	int64_t deadline;
};

struct lan9250_runtime {
	struct net_if *iface;

	K_KERNEL_STACK_MEMBER(thread_stack,
			      CONFIG_RP2350ZS_ETH_LAN9250_RX_THREAD_STACK_SIZE);
	struct k_thread thread;

	uint8_t mac_address[6];
	struct gpio_callback gpio_cb;
	struct k_sem tx_rx_sem;
	struct k_sem int_sem;
	uint8_t buf[NET_ETH_MAX_FRAME_SIZE];

	/* Set once, by the separate ptp_clock device's own init function
	 * (drivers/eth_lan9250/ptp_clock_lan9250.c) - not devicetree-backed,
	 * so it can't reach this device by phandle, and instead stashes a
	 * pointer to itself here (mirroring Zephyr's own
	 * drivers/ethernet/eth_stm32_hal_ptp.c) so lan9250_get_ptp_clock()
	 * (this driver's ethernet_api .get_ptp_clock callback) has
	 * something to return.
	 */
	const struct device *ptp_clock;

	/* Guards 1588_BANK_PORT_GPIO_SEL + whatever banked register access
	 * follows it (see eth_lan9250_priv.h's "1588 Port/GPIO banked
	 * registers" comment) as one atomic sequence. Needed because
	 * lan9250_rx() (this driver's own RX thread) and lan9250_tx()
	 * (called from whatever thread the network stack's TX path runs
	 * on) can run concurrently, and both need to select a bank before
	 * reading their own RX/TX timestamp registers - without this, one
	 * side's bank selection could be stomped by the other's mid-access.
	 * Distinct from tx_rx_sem above, which is a narrower RX-drain/
	 * TX-start handoff, not a general critical-section lock.
	 */
	struct k_mutex bank_lock;

	/* Packets awaiting a hardware RX timestamp that hadn't landed in
	 * CAP_INFO yet by the time this driver finished reading them off the
	 * wire - see the comment on lan9250_1588_rx_timestamp_check() (the
	 * "pending" array) for why this exists and how entries are matched/
	 * expired. Guarded by bank_lock, same as the registers this
	 * correlates against.
	 */
	struct lan9250_1588_rx_pending rx_pending[LAN9250_1588_RX_PENDING_MAX];

	/* Hardware timestamps captured for a frame not yet read out of the
	 * SPI RX FIFO - see lan9250_1588_rx_unclaimed. Guarded by bank_lock.
	 */
	struct lan9250_1588_rx_unclaimed rx_unclaimed[LAN9250_1588_RX_PENDING_MAX];

	/* Per-bit reference count for the 64-bit multicast hash filter
	 * (HMAC_HASHH/HMAC_HASHL - see lan9250_mcast_hash_bit() and
	 * lan9250_set_config()'s ETHERNET_CONFIG_TYPE_FILTER case). More
	 * than one joined multicast group can hash to the same bit (a
	 * 6-bit index has only 64 possible values), so a bit can only be
	 * cleared in hardware once every group that mapped to it has been
	 * left, not on the first leave. Guarded by hash_lock.
	 */
	uint8_t mcast_hash_refcount[64];
	struct k_mutex hash_lock;
};

#endif /*_LAN9250_*/
