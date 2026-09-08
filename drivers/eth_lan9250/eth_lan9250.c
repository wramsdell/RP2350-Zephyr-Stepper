/* LAN9250 Stand-alone Ethernet Controller with SPI
 *
 * Copyright (c) 2024 Mario Paja
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Forked from Zephyr's upstream drivers/ethernet/eth_lan9250.c into this
 * project (project-owned copy, tracked by git here instead of living as an
 * untracked patch to the west-managed checkout) so HMAC_CR's MCPAS bit can
 * be set at init - see the "Forked LAN9250 driver" section in
 * THEORY_OF_OPERATION.md for the full story: without it, this driver's own
 * upstream comment already claims "Pass all multicast frames" right above
 * the HMAC_CR write, but the code never actually sets that bit, so the
 * LAN9250's hardware RX filter silently drops all multicast traffic
 * (mDNS, IGMP) regardless of anything at the IP/IGMP layer. The app-level
 * alternative (CONFIG_NET_PROMISCUOUS_MODE) was tried first and reverted:
 * it raises an RX interrupt for every frame on the LAN rather than just
 * multicast ones, and under sustained heavy traffic that rate corrupted
 * incoming frames badly enough to break ARP (net_stats' IP header/protocol
 * error counters climbed steadily, and ARP resolution failed
 * intermittently) - a problem no amount of buffer-pool tuning fixed, since
 * it's the frame data itself arriving corrupted, not a capacity issue.
 *
 * Also the home of this project's IEEE 1588 (PTP) hardware clock work -
 * see lan9250_1588_init()/lan9250_ptp_clock_read() below, and
 * THEORY_OF_OPERATION.md's "IEEE 1588 / PTP" section for the phased plan.
 */
#define DT_DRV_COMPAT rp2350zs_lan9250

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/sys/byteorder.h>
#include <ethernet/eth_stats.h>

#include "eth_lan9250_ptp.h"

#include "eth_lan9250_priv.h"

LOG_MODULE_REGISTER(eth_lan9250, CONFIG_ETHERNET_LOG_LEVEL);

static int lan9250_write_sys_reg(const struct device *dev, uint16_t address, uint32_t data)
{
	const struct lan9250_config *config = dev->config;
	uint8_t cmd[1] = {LAN9250_SPI_INSTR_WRITE};
	uint8_t addr[2];
	uint8_t instr[4];
	struct spi_buf tx_buf[3];
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 3};

	sys_put_be16(address, addr);
	sys_put_le32(data, instr);

	tx_buf[0].buf = &cmd;
	tx_buf[0].len = ARRAY_SIZE(cmd);
	tx_buf[1].buf = addr;
	tx_buf[1].len = ARRAY_SIZE(addr);
	tx_buf[2].buf = instr;
	tx_buf[2].len = ARRAY_SIZE(instr);

	return spi_write_dt(&config->spi, &tx);
}

static int lan9250_read_sys_reg(const struct device *dev, uint16_t address, uint32_t *value)
{
	const struct lan9250_config *config = dev->config;
	uint8_t cmd[1] = {LAN9250_SPI_INSTR_READ};
	uint8_t addr[2];
	struct spi_buf tx_buf[3];
	struct spi_buf rx_buf[3];
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 3};
	const struct spi_buf_set rx = {.buffers = rx_buf, .count = 3};

	sys_put_be16(address, addr);

	tx_buf[0].buf = &cmd;
	tx_buf[0].len = ARRAY_SIZE(cmd);
	tx_buf[1].buf = addr;
	tx_buf[1].len = ARRAY_SIZE(addr);
	tx_buf[2].buf = NULL;
	tx_buf[2].len = sizeof(uint32_t);

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;
	rx_buf[1].buf = NULL;
	rx_buf[1].len = 2;
	rx_buf[2].buf = value;
	rx_buf[2].len = sizeof(uint32_t);

	return spi_transceive_dt(&config->spi, &tx, &rx);
}

static int lan9250_wait_ready(const struct device *dev, uint16_t address, uint32_t mask,
			      uint32_t expected, uint32_t m_second)
{
	int ret;
	uint32_t tmp;
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(m_second));

	while (true) {
		ret = lan9250_read_sys_reg(dev, address, &tmp);
		if ((ret == 0) && ((tmp & mask) == expected)) {
			return 0;
		}
		if (sys_timepoint_expired(end)) {
			return -EIO;
		}
		k_busy_wait(USEC_PER_MSEC * 1U);
	}
}

static int lan9250_read_mac_reg(const struct device *dev, uint8_t address, uint32_t *value)
{
	uint32_t tmp;
	int ret;

	/* Wait for MAC to be ready and send writing register command and data */
	ret = lan9250_wait_ready(dev, LAN9250_MAC_CSR_CMD, LAN9250_MAC_CSR_CMD_BUSY, 0,
				 LAN9250_MAC_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_MAC_CSR_CMD,
				    address | LAN9250_MAC_CSR_CMD_BUSY | LAN9250_MAC_CSR_CMD_READ);
	if (ret < 0) {
		return ret;
	}

	/* Wait for MAC to be ready and send writing register command and data */
	ret = lan9250_wait_ready(dev, LAN9250_MAC_CSR_CMD, LAN9250_MAC_CSR_CMD_BUSY, 0,
				 LAN9250_MAC_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_read_sys_reg(dev, LAN9250_MAC_CSR_DATA, &tmp);
	if (ret < 0) {
		return ret;
	}

	*value = tmp;

	return 0;
}

static int lan9250_write_mac_reg(const struct device *dev, uint8_t address, uint32_t data)
{
	int ret;
	/* Wait for MAC to be ready and send writing register command and data */
	ret = lan9250_wait_ready(dev, LAN9250_MAC_CSR_CMD, LAN9250_MAC_CSR_CMD_BUSY, 0,
				 LAN9250_MAC_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_MAC_CSR_DATA, data);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_MAC_CSR_CMD, address | LAN9250_MAC_CSR_CMD_BUSY);
	if (ret < 0) {
		return ret;
	}

	/* Wait until writing MAC is done */
	return lan9250_wait_ready(dev, LAN9250_MAC_CSR_CMD, LAN9250_MAC_CSR_CMD_BUSY, 0,
				  LAN9250_MAC_TIMEOUT);
}

static int lan9250_wait_mac_ready(const struct device *dev, uint8_t address, uint32_t mask,
				  uint32_t expected, uint32_t m_second)
{
	int ret;
	uint32_t tmp;
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(m_second));

	while (true) {
		ret = lan9250_read_mac_reg(dev, address, &tmp);
		if ((ret == 0) && ((tmp & mask) == expected)) {
			return 0;
		}
		if (sys_timepoint_expired(end)) {
			return -EIO;
		}
		k_msleep(1);
	}
}

static int lan9250_read_phy_reg(const struct device *dev, uint8_t address, uint16_t *value)
{
	uint32_t tmp;
	int ret;

	/* Wait PHY to be ready and send reading register command */
	ret = lan9250_wait_mac_ready(dev, LAN9250_HMAC_MII_ACC, LAN9250_HMAC_MII_ACC_MIIBZY, 0,
				     LAN9250_PHY_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	/* Reference: Microchip Ethernet LAN9250
	 * https://github.com/microchip-pic-avr-solutions/ethernet-lan9250/
	 *
	 * Datasheet:
	 * https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/00001913A.pdf
	 *
	 * 12.2.18 PHY REGISTERS
	 * The PHY registers are indirectly accessed through the Host MAC MII Access Register
	 * (HMAC_MII_ACC) and Host MAC MII Data Register (HMAC_MII_DATA).
	 *
	 * Write 32bit value to the indirect MAC registers
	 * Where phy_add = 0b00001 & index = address
	 * Data = ((phy_add & 0x1F) << 11) | ((index & 0x1F) << 6)
	 */
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_MII_ACC, (1 << 11) | ((address & 0x1F) << 6));
	if (ret < 0) {
		return ret;
	}

	/* Wait PHY to be ready and send reading register command */
	ret = lan9250_wait_mac_ready(dev, LAN9250_HMAC_MII_ACC, LAN9250_HMAC_MII_ACC_MIIBZY, 0,
				     LAN9250_PHY_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	/* Read 32bit value from the indirect MAC registers */
	ret = lan9250_read_mac_reg(dev, LAN9250_HMAC_MII_DATA, &tmp);
	if (ret < 0) {
		return ret;
	}

	*value = tmp;

	return 0;
}

static int lan9250_write_phy_reg(const struct device *dev, uint8_t address, uint16_t data)
{
	int ret;
	/* Wait PHY to be ready and send reading register command */
	ret = lan9250_wait_mac_ready(dev, LAN9250_HMAC_MII_ACC, LAN9250_HMAC_MII_ACC_MIIBZY, 0,
				     LAN9250_PHY_TIMEOUT);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_MII_DATA, data);
	if (ret < 0) {
		return ret;
	}

	/* Reference: Microchip Ethernet LAN9250
	 * https://github.com/microchip-pic-avr-solutions/ethernet-lan9250/
	 *
	 * Datasheet:
	 * https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/00001913A.pdf
	 *
	 * 12.2.18 PHY REGISTERS
	 * The PHY registers are indirectly accessed through the Host MAC MII Access Register
	 * (HMAC_MII_ACC) and Host MAC MII Data Register (HMAC_MII_DATA).
	 *
	 * Write 32bit value to the indirect MAC registers
	 * Where phy_add = 0b00001 & index = address
	 * Data = ((phy_add & 0x1F) << 11) | ((index & 0x1F)<< 6) | MIIWnR
	 */
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_MII_ACC,
				    (1 << 11) | ((address & 0x1F) << 6) |
					    LAN9250_HMAC_MII_ACC_MIIW_R);
	if (ret < 0) {
		return ret;
	}

	/* Wait PHY to be ready and send reading register command */
	return lan9250_wait_mac_ready(dev, LAN9250_HMAC_MII_ACC, LAN9250_HMAC_MII_ACC_MIIBZY, 0,
				      LAN9250_PHY_TIMEOUT);
}

/*
 * Overrides the devicetree-configured (fixed, identical-on-every-board)
 * MAC address in ctx->mac_address with one derived from the RP2350's own
 * per-chip unique ID, if hwinfo is available - same derivation
 * src/eth_id.c's set_unique_mac_address() used to do from main(), moved
 * here to run early enough for phase 5's gPTP support.
 *
 * This must happen before lan9250_set_macaddr() (immediately below, in
 * lan9250_init()) programs the hardware's own RX address filter, and
 * before this driver's net_if is registered at all - not just before
 * main() runs. Zephyr's gPTP subsystem computes its own clockIdentity
 * (an EUI-64 derived from the interface's link address) exactly once, in
 * net_gptp_init(), called from net_post_init() via
 * SYS_INIT(net_init, POST_KERNEL, CONFIG_NET_INIT_PRIO) - which runs
 * during kernel boot, before main() is ever called. A MAC override from
 * main() (the original approach) is applied correctly for everything
 * else (ARP, DHCP, hostname) but is already too late for gPTP: every
 * board ends up computing the identical clockIdentity from the
 * devicetree overlay's placeholder MAC (00:00:00:01:02:03 here), since
 * that's still what the interface's link address was at the moment
 * net_gptp_init() ran. Confirmed via a live two-board test: every
 * captured Pdelay_Resp's ClockIdentity read back as the fixed
 * 0x000000fffe010203 regardless of which board sent it, and the
 * Best Master Clock Algorithm never progressed past ROLE_SELECTION
 * because - from either board's perspective - every peer response
 * appeared to carry its own local Clock Identity.
 */
static void lan9250_load_unique_mac_address(struct lan9250_runtime *ctx)
{
	uint8_t hw_id[8];
	ssize_t len;

	len = hwinfo_get_device_id(hw_id, sizeof(hw_id));
	if (len < (ssize_t)sizeof(ctx->mac_address)) {
		LOG_WRN("hwinfo_get_device_id() returned %d bytes; keeping the devicetree MAC",
			(int)len);
		return;
	}

	/* Use the last 6 of the (up to 8) ID bytes - RP2350's flash unique
	 * ID, or DEVICE_ID+WAFER_ID depending on variant, either way stable
	 * and unique per chip (see hwinfo_rpi_pico.c).
	 */
	memcpy(ctx->mac_address, &hw_id[len - (ssize_t)sizeof(ctx->mac_address)],
	       sizeof(ctx->mac_address));

	/* Locally administered, unicast - the same convention Zephyr's own
	 * net_eth_mac_load()/NET_ETH_MAC_RANDOM path uses for a generated
	 * (non-IEEE-assigned) MAC address.
	 */
	ctx->mac_address[0] &= ~0x01;
	ctx->mac_address[0] |= 0x02;
}

static int lan9250_set_macaddr(const struct device *dev)
{
	struct lan9250_runtime *ctx = dev->data;
	int ret;

	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_ADDRL,
				    ctx->mac_address[0] | (ctx->mac_address[1] << 8) |
					    (ctx->mac_address[2] << 16) |
					    (ctx->mac_address[3] << 24));
	if (ret < 0) {
		return ret;
	}

	return lan9250_write_mac_reg(dev, LAN9250_HMAC_ADDRH,
				     ctx->mac_address[4] | (ctx->mac_address[5] << 8));
}

static int lan9250_hw_cfg_check(const struct device *dev)
{
	uint32_t tmp;
	int ret;

	do {
		ret = lan9250_read_sys_reg(dev, LAN9250_HW_CFG, &tmp);
		if (ret < 0) {
			return ret;
		}
		k_busy_wait(USEC_PER_MSEC * 1U);
	} while ((tmp & LAN9250_HW_CFG_DEVICE_READY) == 0);

	return 0;
}

static int lan9250_sw_reset(const struct device *dev)
{
	int ret;

	ret = lan9250_write_sys_reg(dev, LAN9250_RESET_CTL,
				    LAN9250_RESET_CTL_HMAC_RST | LAN9250_RESET_CTL_PHY_RST |
					    LAN9250_RESET_CTL_DIGITAL_RST);
	if (ret < 0) {
		return ret;
	}

	/* Wait until LAN9250 SPI bus is ready */
	return lan9250_wait_ready(dev, LAN9250_BYTE_TEST, BOTR_MASK, LAN9250_BYTE_TEST_DEFAULT,
				  LAN9250_RESET_TIMEOUT);
}

static int lan9250_configure(const struct device *dev)
{
	uint32_t tmp;
	int ret;

	ret = lan9250_hw_cfg_check(dev);
	if (ret < 0) {
		return ret;
	}

	/* Read LAN9250 hardware ID */
	ret = lan9250_read_sys_reg(dev, LAN9250_ID_REV, &tmp);
	if (ret < 0) {
		return ret;
	}

	if ((tmp & LAN9250_ID_REV_CHIP_ID) != LAN9250_ID_REV_CHIP_ID_DEFAULT) {
		LOG_ERR("ERROR: Bad Rev ID: %08x\n", tmp);
		return -ENODEV;
	}

	/* Configure TX FIFO size mode to be 8:
	 *
	 *   - TX data FIFO size:   7680
	 *   - RX data FIFO size:   7680
	 *   - TX status FIFO size: 512
	 *   - RX status FIFO size: 512
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_HW_CFG,
				    LAN9250_HW_CFG_MBO | LAN9250_HW_CFG_TX_FIF_SZ_8KB);
	if (ret < 0) {
		return ret;
	}

	/* Configure MAC automatic flow control:
	 *
	 *  Reference: Microchip Ethernet LAN9250
	 *  https://github.com/microchip-pic-avr-solutions/ethernet-lan9250/
	 *  LAN_Regwrite32(AFC_CFG, 0x006E3741);
	 *
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_AFC_CFG, 0x006e3741);
	if (ret < 0) {
		return ret;
	}

	/* Configure interrupt:
	 *
	 *   - Interrupt De-assertion interval: 100
	 *   - Interrupt output to pin
	 *   - Interrupt pin active output low
	 *   - Interrupt pin push-pull driver
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_IRQ_CFG,
				    LAN9250_IRQ_CFG_INT_DEAS_100US | LAN9250_IRQ_CFG_IRQ_EN |
					    LAN9250_IRQ_CFG_IRQ_TYPE_PP);
	if (ret < 0) {
		return ret;
	}

	/* Configure interrupt trigger source, please refer to macro
	 * LAN9250_INT_SOURCE.
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_INT_EN,
				    LAN9250_INT_EN_PHY_INT_EN | LAN9250_INT_EN_RSFL_EN);
	if (ret < 0) {
		return ret;
	}

	/* Disable TX data FIFO available interrupt */
	ret = lan9250_write_sys_reg(dev, LAN9250_FIFO_INT,
				    LAN9250_FIFO_INT_TX_DATA_AVAILABLE_LEVEL |
					    LAN9250_FIFO_INT_TX_STATUS_LEVEL);
	if (ret < 0) {
		return ret;
	}

	/* Configure RX:
	 *
	 *   - RX DMA counter: Ethernet maximum packet size
	 *   - RX data offset: 4, so that need read dummy before reading data
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_RX_CFG, 0x06000000 | 0x00000400);
	if (ret < 0) {
		return ret;
	}

	/* Configure remote power management:
	 *
	 *   - Auto wakeup
	 *   - Energy-detect
	 *   - Wake on
	 *   - Clear wakeon
	 *
	 * Deliberately NOT setting PMT_CTRL_1588_DIS/PMT_CTRL_1588_TSU_DIS
	 * (upstream Zephyr's driver sets both unconditionally) - this
	 * project uses the 1588 PTP hardware clock (see lan9250_1588_init()
	 * below), which needs the 1588 clock and timestamp unit left
	 * running.
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_PMT_CTRL,
				    LAN9250_PMT_CTRL_PM_WAKE |
					    LAN9250_PMT_CTRL_WOL_EN | LAN9250_PMT_CTRL_WOL_STS);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHY basic control:
	 *
	 *   - Auto-Negotiation for 10/100 Mbits and Half/Full Duplex
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_BASIC_CONTROL,
				    LAN9250_PHY_BASIC_CONTROL_PHY_AN |
					    LAN9250_PHY_BASIC_CONTROL_PHY_SPEED_SEL_LSB |
					    LAN9250_PHY_BASIC_CONTROL_PHY_DUPLEX);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHY auto-negotiation advertisement capability:
	 *
	 *   - Asymmetric pause
	 *   - Symmetric pause
	 *   - 100Base-X half/full duplex
	 *   - 10Base-X half/full duplex
	 *   - Select IEEE802.3
	 */
	ret = lan9250_write_phy_reg(
		dev, LAN9250_PHY_AN_ADV,
		LAN9250_PHY_AN_ADV_ASYM_PAUSE | LAN9250_PHY_AN_ADV_SYM_PAUSE |
			LAN9250_PHY_AN_ADV_100BTX_HD | LAN9250_PHY_AN_ADV_100BTX_FD |
			LAN9250_PHY_AN_ADV_10BT_HD | LAN9250_PHY_AN_ADV_10BT_FD |
			LAN9250_PHY_AN_ADV_SELECTOR_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHY special mode:
	 *
	 *   - PHY mode = 111b, enable all capable and auto-nagotiation
	 *   - PHY address = 1, default value is fixed to 1 by manufacturer
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_SPECIAL_MODES, 0x00E0 | 1);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHY special control or status indication:
	 *
	 *   - Port auto-MDIX determined by bits 14 and 13
	 *   - Auto-MDIX
	 *   - Disable SQE tests
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_SPECIAL_CONTROL_STAT_IND,
				    LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_AMDIXCTRL |
					    LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_AMDIXEN |
					    LAN9250_PHY_SPECIAL_CONTROL_STAT_IND_SQEOFF);
	if (ret < 0) {
		return ret;
	}

	/* Configure PHY interrupt source:
	 *
	 *   - Link up
	 *   - Link down
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_INTERRUPT_MASK,
				    LAN9250_PHY_INTERRUPT_SOURCE_LINK_UP |
					    LAN9250_PHY_INTERRUPT_SOURCE_LINK_DOWN);
	if (ret < 0) {
		return ret;
	}

	/* Configure special control or status:
	 *
	 *   - Fixed to write 0000010b to reserved filed
	 */
	ret = lan9250_write_phy_reg(dev, LAN9250_PHY_SPECIAL_CONTROL_STATUS,
				    LAN9250_PHY_MODE_CONTROL_STATUS_ALTINT);
	if (ret < 0) {
		return ret;
	}

	/* Clear interrupt status */
	ret = lan9250_write_sys_reg(dev, LAN9250_INT_STS, 0xFFFFFFFFU);
	if (ret < 0) {
		return ret;
	}

	/* Configure HMAC control:
	 *
	 *   - Automatically strip the pad field on incoming packets
	 *   - Full duplex
	 *   - TX enable
	 *   - RX enable
	 *   - Hash Perfect filtering mode: unicast perfect-filtered against
	 *     our own MAC (HMAC_ADDRH/L, HO left clear), multicast
	 *     hash-filtered against HMAC_HASHH/L (HPFILT set, MCPAS clear)
	 *   - Promiscuous disabled
	 *
	 * This project originally ran with MCPAS (Pass All Multicast) set
	 * instead - upstream Zephyr's driver has a comment claiming this
	 * mode but never actually set the bit, so every multicast frame
	 * (mDNS, IGMP) was silently dropped by hardware regardless of the
	 * IP/IGMP layer; this project's own fork fixed that by actually
	 * setting MCPAS. MCPAS was a blunt instrument, though: per the
	 * datasheet (DS00001913C, Section 11.15.4, "HMAC_HASHH"), MCPAS
	 * overrides the hash table entirely and accepts every multicast
	 * frame on the wire unconditionally, regardless of whether this
	 * interface has actually joined that group. On a network with other
	 * multicast traffic (any device's own mDNS, other multicast
	 * chatter), that meant every one of those irrelevant frames still
	 * had to be pulled across this driver's SPI link and processed by
	 * the RP2350 - real, wasted SPI/RX-thread bandwidth on frames
	 * nothing here ever wanted, on a link whose throughput is bounded by
	 * SPI transaction rate, not wire speed. Hash Perfect mode (verified
	 * against Table 11-1, page 141) rejects multicast frames at the
	 * MAC's own address-check logic - before they ever reach the RX
	 * FIFO the host has to drain - unless their destination hashes to a
	 * bit this interface has actually asked for via a real IGMP/MLD
	 * join. HMAC_HASHH/HMAC_HASHL start at 0 here (reject all
	 * multicast) and get populated dynamically as groups are joined -
	 * see lan9250_get_capabilities()'s ETHERNET_HW_FILTERING and
	 * lan9250_set_config()'s ETHERNET_CONFIG_TYPE_FILTER case.
	 */
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_CR,
				    LAN9250_HMAC_CR_PADSTR | LAN9250_HMAC_CR_TXEN |
					    LAN9250_HMAC_CR_RXEN | LAN9250_HMAC_CR_FDPX |
					    LAN9250_HMAC_CR_HPFILT);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_HASHL, 0);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_HASHH, 0);
	if (ret < 0) {
		return ret;
	}

	/* Configure TX:
	 *
	 *   - TX enable
	 */
	return lan9250_write_sys_reg(dev, LAN9250_TX_CFG, LAN9250_TX_CFG_TX_ON);
}

static int lan9250_write_buf(const struct device *dev, uint8_t *data_buffer, uint16_t buf_len)
{
	const struct lan9250_config *config = dev->config;
	uint8_t cmd[1] = {LAN9250_SPI_INSTR_WRITE};
	uint8_t instr[2] = {(LAN9250_TX_DATA_FIFO >> 8) & 0xFF, (LAN9250_TX_DATA_FIFO & 0xFF)};
	struct spi_buf tx_buf[3];
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 3};

	tx_buf[0].buf = &cmd;
	tx_buf[0].len = ARRAY_SIZE(cmd);
	tx_buf[1].buf = &instr;
	tx_buf[1].len = ARRAY_SIZE(instr);
	tx_buf[2].buf = data_buffer;
	tx_buf[2].len = buf_len;

	return spi_transceive_dt(&config->spi, &tx, NULL);
}

static int lan9250_read_buf(const struct device *dev, uint8_t *data_buffer, uint16_t buf_len)
{
	const struct lan9250_config *config = dev->config;
	uint8_t cmd[1] = {LAN9250_SPI_INSTR_READ};
	uint8_t instr[2] = {(LAN9250_RX_DATA_FIFO >> 8) & 0xFF, (LAN9250_RX_DATA_FIFO & 0xFF)};
	struct spi_buf tx_buf[3];
	struct spi_buf rx_buf[3];
	const struct spi_buf_set tx = {.buffers = tx_buf, .count = 3};
	const struct spi_buf_set rx = {.buffers = rx_buf, .count = 3};

	tx_buf[0].buf = &cmd;
	tx_buf[0].len = ARRAY_SIZE(cmd);
	tx_buf[1].buf = &instr;
	tx_buf[1].len = ARRAY_SIZE(instr);
	tx_buf[2].buf = NULL;
	tx_buf[2].len = buf_len;

	rx_buf[0].buf = NULL;
	rx_buf[0].len = 1;
	rx_buf[1].buf = NULL;
	rx_buf[1].len = 2;
	rx_buf[2].buf = data_buffer;
	rx_buf[2].len = buf_len;

	return spi_transceive_dt(&config->spi, &tx, &rx);
}

/* Defined near lan9250_init() below (Phase 4, IEEE 1588 RX/TX packet
 * timestamping) - forward-declared here since lan9250_rx()/lan9250_tx()
 * are defined earlier in this file than the rest of the 1588 code.
 */
static void lan9250_1588_rx_timestamp_check(const struct device *dev, struct net_pkt *pkt,
					    size_t pkt_len);
static void lan9250_1588_tx_timestamp_check(const struct device *dev, struct net_pkt *pkt,
					    const uint8_t *frame, size_t len);
static void lan9250_1588_rx_pending_expire(struct lan9250_runtime *context, int64_t now);

static int lan9250_rx(const struct device *dev)
{
	struct lan9250_runtime *ctx = dev->data;
	const uint16_t buf_rx_size = CONFIG_NET_BUF_DATA_SIZE;
	struct net_pkt *pkt;
	struct net_buf *pkt_buf;
	uint16_t pkt_len;
	uint16_t frame_len;
	uint8_t pktcnt;
	uint32_t tmp;
	int ret;

	/* Check valid packet count */
	ret = lan9250_read_sys_reg(dev, LAN9250_RX_FIFO_INF, &tmp);
	if (ret < 0) {
		return ret;
	}
	pktcnt = (tmp & 0x00ff0000) >> 16;

	/* Check packet length */
	ret = lan9250_read_sys_reg(dev, LAN9250_RX_STATUS_FIFO, &tmp);
	if (ret < 0) {
		return ret;
	}
	pkt_len = (tmp & LAN9250_RX_STS_PACKET_LEN) >> 16;

	if (pktcnt == 0 || pkt_len == 0) {
		return 0;
	}

	/* Read dummy data - this drains the RX_CFG RXDOFF=4 start-of-packet
	 * offset configured in lan9250_init(), NOT an FCS/CRC field.
	 * LAN9250_RX_STS_PACKET_LEN already reports the exact Ethernet
	 * frame length (no FCS, no RXDOFF) - confirmed empirically against
	 * real captured frame lengths (e.g. a Follow_Up message's raw
	 * status length reads exactly 86, matching its wire frame.len
	 * exactly). This used to be followed by a "pkt_len -= 4" that
	 * wrongly assumed the reported length included a 4-byte FCS,
	 * silently truncating the last 4 bytes of every received frame and
	 * leaving them stuck in the FIFO to desync every subsequent
	 * frame's read (manifesting as garbage/misaligned EtherTypes like
	 * 0xc0a8 - the raw bytes of "192.168" from a shifted IPv4 header).
	 */
	ret = lan9250_read_sys_reg(dev, LAN9250_RX_DATA_FIFO, &tmp);
	if (ret < 0) {
		return ret;
	}
	frame_len = pkt_len;

	/* RX_CFG's RX_EA field (bits 31:30) is left at its default 00b (4-
	 * byte end alignment): the chip pads the last transfer of
	 * (RXDOFF + frame) up to a DWORD boundary. RXDOFF is already a
	 * whole DWORD, so the needed pad is purely a function of the
	 * frame's own length remainder - drained below, after the frame's
	 * real bytes, instead of the old fixed 4-byte read that only
	 * happened to be correct when frame_len was itself a multiple of
	 * 4.
	 */
	uint16_t end_pad = (4 - (frame_len % 4)) % 4;

	if (pkt_len > NET_ETH_MAX_FRAME_SIZE) {
		LOG_ERR("Maximum frame length exceeded, it should be: %d", NET_ETH_MAX_FRAME_SIZE);
		eth_stats_update_errors_rx(ctx->iface);
	}

	/* Get the frame from the buffer */
	pkt = net_pkt_rx_alloc_with_buffer(ctx->iface, pkt_len, NET_AF_UNSPEC, 0,
					   K_MSEC(CONFIG_RP2350ZS_ETH_LAN9250_BUF_ALLOC_TIMEOUT));
	if (!pkt) {
		LOG_ERR("%s: Could not allocate rx buffer", dev->name);
		eth_stats_update_errors_rx(ctx->iface);

		/* Sweep rx_pending/rx_unclaimed for timed-out entries even on
		 * this failure path - lan9250_1588_rx_pending_expire() is
		 * otherwise only reached from lan9250_1588_rx_timestamp_check(),
		 * which itself only runs *after* a successful allocation above.
		 * If the RX pkt pool is genuinely exhausted (e.g. by
		 * LAN9250_1588_RX_PENDING_MAX held refs awaiting a hardware
		 * timestamp match that never arrives), that's a real deadlock:
		 * no allocation can succeed until stale entries are freed, but
		 * the only code path that frees them requires an allocation to
		 * have already succeeded. Running the sweep here breaks that
		 * cycle regardless of whether this particular frame's own
		 * allocation happened to succeed.
		 */
		k_mutex_lock(&ctx->bank_lock, K_FOREVER);
		lan9250_1588_rx_pending_expire(ctx, k_uptime_get());
		k_mutex_unlock(&ctx->bank_lock);

		/* Must still drain this frame's pkt_len bytes (plus the same
		 * trailing dummy word the success path reads below) out of
		 * LAN9250_RX_DATA_FIFO even though it's being dropped - the
		 * chip's own RX FIFO read pointer only advances as bytes are
		 * clocked out over SPI, so returning here without reading
		 * them leaves this frame's payload sitting in the FIFO. The
		 * next call would then misinterpret those leftover bytes as
		 * a new packet's RX_FIFO_INF/RX_STATUS_FIFO header, corrupting
		 * every subsequent frame (Sync, Follow_Up, Announce, ...)
		 * until the chip is reset - silently starving PTP of any
		 * further valid traffic while its local clock free-runs
		 * uncorrected. lan9250_read_buf() already treats a NULL
		 * buffer as "discard", matching how the cmd/instr echo bytes
		 * above are handled.
		 *
		 * Chunked at buf_rx_size, same as the success path below -
		 * every other call site in this driver only ever asks the SPI
		 * layer for a transfer that size or smaller (bounded by
		 * CONFIG_NET_BUF_DATA_SIZE), so a single one-shot transfer of
		 * up to a full ~1514-byte frame here would be the first time
		 * this driver ever asked the RP2350's SPI/DMA path for a
		 * transfer that large - not proven safe, and not worth risking
		 * on a rarely-exercised error path.
		 */
		while (pkt_len > 0) {
			uint16_t chunk = pkt_len > buf_rx_size ? buf_rx_size : pkt_len;

			if (lan9250_read_buf(dev, NULL, chunk) < 0) {
				break;
			}
			pkt_len -= chunk;
		}
		if (end_pad > 0) {
			(void)lan9250_read_buf(dev, NULL, end_pad);
		}
		return 0;
	}

	pkt_buf = pkt->buffer;

	do {
		uint8_t *data_ptr = pkt_buf->data;
		uint16_t data_len;

		if (pkt_len > buf_rx_size) {
			data_len = buf_rx_size;
		} else {
			data_len = pkt_len;
		}
		pkt_len -= data_len;

		ret = lan9250_read_buf(dev, data_ptr, data_len);
		if (ret < 0) {
			return ret;
		}
		net_buf_add(pkt_buf, data_len);
		pkt_buf = pkt_buf->frags;
	} while (pkt_len > 0);

	if (end_pad > 0) {
		ret = lan9250_read_buf(dev, NULL, end_pad);
		if (ret < 0) {
			return ret;
		}
	}

	lan9250_1588_rx_timestamp_check(dev, pkt, frame_len);

	/* Feed buffer frame to IP stack */
	if (net_recv_data(ctx->iface, pkt) < 0) {
		net_pkt_unref(pkt);
	}

	k_sem_give(&ctx->tx_rx_sem);

	return 0;
}

static int lan9250_tx(const struct device *dev, struct net_pkt *pkt)
{
	struct lan9250_runtime *ctx = dev->data;
	size_t len = net_pkt_get_len(pkt);
	uint32_t regval;
	uint16_t free_size;
	uint8_t status_size;
	uint32_t tmp;
	int ret;

	ret = lan9250_read_sys_reg(dev, LAN9250_TX_FIFO_INF, &regval);
	if (ret < 0) {
		return ret;
	}

	status_size = (regval & LAN9250_TX_FIFO_INF_TXSUSED) >> 16;
	free_size = regval & LAN9250_TX_FIFO_INF_TXFREE;

	k_sem_take(&ctx->tx_rx_sem, K_FOREVER);

	/* TX command 'A' */
	ret = lan9250_write_sys_reg(
		dev, LAN9250_TX_DATA_FIFO,
		LAN9250_TX_CMD_A_INT_ON_COMP | LAN9250_TX_CMD_A_BUFFER_ALIGN_4B |
			LAN9250_TX_CMD_A_START_OFFSET_0B | LAN9250_TX_CMD_A_FIRST_SEG |
			LAN9250_TX_CMD_A_LAST_SEG | len);
	if (ret < 0) {
		return ret;
	}

	/* TX command 'B' */
	ret = lan9250_write_sys_reg(dev, LAN9250_TX_DATA_FIFO, LAN9250_TX_CMD_B_PACKET_TAG | len);
	if (ret < 0) {
		return ret;
	}

	if (net_pkt_read(pkt, ctx->buf, len)) {
		return -EIO;
	}

	ret = lan9250_write_buf(dev, ctx->buf, LAN9250_ALIGN(len));
	if (ret < 0) {
		return ret;
	}

	for (int i = 0; i < status_size; i++) {
		ret = lan9250_read_sys_reg(dev, LAN9250_TX_STATUS_FIFO, &tmp);
		if (ret < 0) {
			return ret;
		}
	}

	lan9250_1588_tx_timestamp_check(dev, pkt, ctx->buf, len);

	k_sem_give(&ctx->tx_rx_sem);

	return 0;
}

static void lan9250_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct lan9250_runtime *context = CONTAINER_OF(cb, struct lan9250_runtime, gpio_cb);

	k_sem_give(&context->int_sem);
}

static void lan9250_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	const struct device *dev = p1;
	struct lan9250_runtime *context = dev->data;
	uint32_t int_sts;
	uint16_t tmp = 0;
	uint32_t ier;

	while (true) {
		k_sem_take(&context->int_sem, K_FOREVER);

		/* Save interrupt enable register value */
		lan9250_read_sys_reg(dev, LAN9250_INT_EN, &ier);

		/* Disable interrupts to release the interrupt line */
		lan9250_write_sys_reg(dev, LAN9250_INT_EN, 0);

		/* Read interrupt status register */
		lan9250_read_sys_reg(dev, LAN9250_INT_STS, &int_sts);

		if ((int_sts & LAN9250_INT_STS_PHY_INT) != 0) {

			/* Read PHY interrupt source register */
			lan9250_read_phy_reg(dev, LAN9250_PHY_INTERRUPT_SOURCE, &tmp);
			if (tmp & LAN9250_PHY_INTERRUPT_SOURCE_LINK_UP) {
				uint16_t special_stat = 0, special_ind = 0;

				lan9250_read_phy_reg(dev, LAN9250_PHY_SPECIAL_CONTROL_STATUS,
						     &special_stat);
				lan9250_read_phy_reg(dev, LAN9250_PHY_SPECIAL_CONTROL_STAT_IND,
						     &special_ind);
				LOG_DBG("LINK UP: special_control_status=0x%04x "
					"special_control_stat_ind=0x%04x",
					special_stat, special_ind);
				net_eth_carrier_on(context->iface);
			} else if (tmp & LAN9250_PHY_INTERRUPT_SOURCE_LINK_DOWN) {
				LOG_DBG("LINK DOWN");
				net_eth_carrier_off(context->iface);
			}
		}

		if ((int_sts & LAN9250_INT_STS_RSFL) != 0) {
			lan9250_write_sys_reg(dev, LAN9250_INT_STS, LAN9250_INT_STS_RSFL);
			lan9250_rx(dev);
		}

		/* Re-enable interrupts */
		lan9250_write_sys_reg(dev, LAN9250_INT_EN, ier);
	}
}

static enum ethernet_hw_caps lan9250_get_capabilities(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* ETHERNET_PTP is a hard gate, not just informational: Zephyr's own
	 * net_eth_get_ptp_clock() (subsys/net/l2/ethernet/ethernet.c) returns
	 * NULL - regardless of whether .get_ptp_clock is implemented - unless
	 * this bit is set here. gPTP (phase 5) calls exactly that function to
	 * reach lan9250_get_ptp_clock(), so this must be advertised for gPTP
	 * to work at all, even though phase 3 already wired up
	 * .get_ptp_clock itself.
	 */
	/* ETHERNET_HW_FILTERING: makes Zephyr's own ethernet_mcast_monitor_cb()
	 * (subsys/net/l2/ethernet/ethernet.c) call this driver's set_config()
	 * with ETHERNET_CONFIG_TYPE_FILTER on every multicast group join/
	 * leave, which is how the LAN9250's 64-bit hardware hash filter gets
	 * populated - see lan9250_set_config()'s ETHERNET_CONFIG_TYPE_FILTER
	 * case and lan9250_init()'s Hash Perfect mode setup.
	 */
	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE | ETHERNET_PTP |
	       ETHERNET_HW_FILTERING
#if defined(CONFIG_NET_PROMISCUOUS_MODE)
		| ETHERNET_PROMISC_MODE
#endif
	;
}

static void lan9250_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct lan9250_runtime *context = dev->data;

	net_if_set_link_addr(iface, context->mac_address, sizeof(context->mac_address),
			     NET_LINK_ETHERNET);
	context->iface = iface;
	ethernet_init(iface);

	net_if_carrier_off(iface);

	k_thread_create(&context->thread, context->thread_stack,
			CONFIG_RP2350ZS_ETH_LAN9250_RX_THREAD_STACK_SIZE,
			lan9250_thread, (void *)dev, NULL, NULL,
			K_PRIO_COOP(CONFIG_RP2350ZS_ETH_LAN9250_RX_THREAD_PRIO), 0, K_NO_WAIT);
}

/*
 * Standard Ethernet multicast-hash CRC (the same algorithm used by
 * essentially every 10/100 MAC with a 64-bit hash filter, and by the
 * Linux kernel's own ether_crc() helper for identical purposes) - CRC-32
 * over the 6 destination-address bytes, reflected polynomial 0xEDB88320,
 * initial value all-ones. Confirmed against the LAN9250 datasheet
 * (DS00001913C, Section 11.4.2, "Hash Only Filtering", page 142): "the
 * destination address in the incoming frame is passed through the CRC
 * logic and the upper 6-bits of the CRC register are used to index the
 * contents of the hash table. The most significant bit determines the
 * register to be used (HMAC_HASHH or HMAC_HASHL), while the other five
 * bits determine the bit within the register."
 */
static uint32_t lan9250_ether_crc(const uint8_t addr[6])
{
	uint32_t crc = 0xffffffffUL;

	for (int byte = 0; byte < 6; byte++) {
		uint8_t data = addr[byte];

		for (int bit = 0; bit < 8; bit++) {
			crc = (crc >> 1) ^ (((crc ^ data) & 1) ? 0xEDB88320UL : 0);
			data >>= 1;
		}
	}

	return crc;
}

/* Returns the 0-63 hash table bit index for a multicast MAC address, per
 * the algorithm in lan9250_ether_crc()'s comment: the upper 6 bits of the
 * CRC register (bits 31:26).
 */
static uint8_t lan9250_mcast_hash_bit(const uint8_t addr[6])
{
	return (uint8_t)(lan9250_ether_crc(addr) >> 26);
}

/* Recomputes and writes HMAC_HASHH/HMAC_HASHL from the current
 * mcast_hash_refcount[] state. Caller must hold ctx->hash_lock.
 */
static int lan9250_mcast_hash_write(const struct device *dev)
{
	struct lan9250_runtime *ctx = dev->data;
	uint32_t hashh = 0, hashl = 0;
	int ret;

	for (int bit = 0; bit < 64; bit++) {
		if (ctx->mcast_hash_refcount[bit] == 0) {
			continue;
		}

		if (bit < 32) {
			hashl |= BIT(bit);
		} else {
			hashh |= BIT(bit - 32);
		}
	}

	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_HASHL, hashl);
	if (ret < 0) {
		return ret;
	}

	return lan9250_write_mac_reg(dev, LAN9250_HMAC_HASHH, hashh);
}

static int lan9250_set_config(const struct device *dev, enum ethernet_config_type type,
			      const struct ethernet_config *config)
{
	struct lan9250_runtime *ctx = dev->data;
	int ret;

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(ctx->mac_address, config->mac_address.addr,
		       sizeof(ctx->mac_address));
		ret = lan9250_set_macaddr(dev);
		if (ret < 0) {
			LOG_ERR("Set mac address failed");
			return ret;
		}

		LOG_INF("%s MAC set to %02x:%02x:%02x:%02x:%02x:%02x",
			dev->name,
			ctx->mac_address[0], ctx->mac_address[1],
			ctx->mac_address[2], ctx->mac_address[3],
			ctx->mac_address[4], ctx->mac_address[5]);

		return 0;
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		if (IS_ENABLED(CONFIG_NET_PROMISCUOUS_MODE)) {
			uint32_t reg;

			ret = lan9250_read_mac_reg(dev, LAN9250_HMAC_CR, &reg);
			if (ret < 0) {
				return ret;
			}

			/* See Table 11-1 from the LAN9250 data sheet */
			if (config->promisc_mode) {
				if ((reg & LAN9250_HMAC_CR_PRMS) != 0) {
					return -EALREADY;
				}

				reg &= ~LAN9250_HMAC_CR_MCPAS;
				reg |= LAN9250_HMAC_CR_PRMS;
				reg &= ~LAN9250_HMAC_CR_HO;
			} else {
				if ((reg & LAN9250_HMAC_CR_PRMS) == 0) {
					return -EALREADY;
				}

				/* Restore Hash Perfect mode (see
				 * lan9250_init()'s comment) rather than the
				 * old Pass-All-Multicast behavior - this
				 * project no longer runs with MCPAS set.
				 */
				reg &= ~LAN9250_HMAC_CR_MCPAS;
				reg |= LAN9250_HMAC_CR_HPFILT;
				reg &= ~LAN9250_HMAC_CR_PRMS;
				reg &= ~LAN9250_HMAC_CR_HO;
			}

			return lan9250_write_mac_reg(dev, LAN9250_HMAC_CR, reg);
		}

		break;
	case ETHERNET_CONFIG_TYPE_FILTER:
		/* Only multicast destination-address filtering is
		 * implemented (source-address filtering isn't a LAN9250
		 * hardware feature at all, per the datasheet's Address
		 * Filtering chapter). Zephyr's own ethernet L2 layer
		 * (ethernet_mcast_monitor_cb(), subsys/net/l2/ethernet/
		 * ethernet.c) calls this for every IPv4/IPv6 multicast
		 * group join/leave on this interface, since
		 * ETHERNET_HW_FILTERING is advertised below - no separate
		 * monitoring needed on this driver's part.
		 */
		if (config->filter.type != ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS) {
			return -ENOTSUP;
		}

		if (!(config->filter.mac_address.addr[0] & 0x01)) {
			/* Not actually a multicast address - shouldn't
			 * happen given the caller, but the hash filter only
			 * ever applies to multicast per the datasheet, so
			 * there's nothing meaningful to do with a unicast
			 * address here.
			 */
			return -ENOTSUP;
		}

		{
			struct lan9250_runtime *ctx = dev->data;
			uint8_t bit = lan9250_mcast_hash_bit(config->filter.mac_address.addr);

			k_mutex_lock(&ctx->hash_lock, K_FOREVER);

			if (config->filter.set) {
				ctx->mcast_hash_refcount[bit]++;
			} else if (ctx->mcast_hash_refcount[bit] > 0) {
				/* Only decrement, never write the register,
				 * if another still-joined group also hashes
				 * to this same bit (see mcast_hash_refcount's
				 * comment in eth_lan9250_priv.h) - checked via
				 * the refcount reaching zero below, not by
				 * skipping the decrement itself.
				 */
				ctx->mcast_hash_refcount[bit]--;
			}

			ret = lan9250_mcast_hash_write(dev);
			k_mutex_unlock(&ctx->hash_lock);
		}

		return ret;
	default:
		break;
	}

	return -ENOTSUP;
}

int lan9250_rx_drop_get(const struct device *dev, uint32_t *rx_drop)
{
	return lan9250_read_sys_reg(dev, LAN9250_RX_DROP, rx_drop);
}

/*
 * Returns the separate ptp_clock device wrapping this driver's 1588
 * clock, once drivers/eth_lan9250/ptp_clock_lan9250.c's own init has run
 * and stashed itself into context->ptp_clock (see the comment on that
 * field in eth_lan9250_priv.h). NULL before that (or if PTP support
 * isn't built in), matching this callback's documented contract.
 */
static const struct device *lan9250_get_ptp_clock(const struct device *dev)
{
	struct lan9250_runtime *context = dev->data;

	return context->ptp_clock;
}

#if defined(CONFIG_NET_STATISTICS_ETHERNET)
static struct net_stats_eth *lan9250_get_stats(const struct device *dev)
{
	struct lan9250_runtime *ctx = dev->data;

	return &ctx->stats;
}
#endif

static const struct ethernet_api api_funcs = {
	.iface_api.init = lan9250_iface_init,
	.get_capabilities = lan9250_get_capabilities,
	.set_config = lan9250_set_config,
	.send = lan9250_tx,
	.get_ptp_clock = lan9250_get_ptp_clock,
#if defined(CONFIG_NET_STATISTICS_ETHERNET)
	.get_stats = lan9250_get_stats,
#endif
};

/*
 * IEEE 1588 (PTP) hardware clock - Phase 1 bring-up (see
 * THEORY_OF_OPERATION.md's "IEEE 1588 / PTP" section for the project's
 * phased plan, and the LAN9250 datasheet Section 14.0 for the hardware
 * this is built on). This phase only enables the clock and gets it
 * readable; no Zephyr ptp_clock integration, GPIO event output, or
 * network timestamping yet.
 */
/*
 * Turns the 1588 unit on. Deliberately the LAST 1588-related init step
 * this driver performs (called from lan9250_init(), after
 * lan9250_1588_init() and lan9250_1588_timestamping_init() have both
 * finished configuring it): the datasheet marks GENERAL_CONFIG's
 * TSU_ENABLE bit and most of RX_TIMESTAMP_CONFIG/TX_TIMESTAMP_CONFIG's
 * fields (domain match, alternate-master, checksum/FCS bypass, PTP
 * version) "must not change while 1588_ENABLE is set" - so enabling
 * first and configuring after, as an earlier version of this driver did,
 * left those writes touching a live/enabled unit and their effect
 * undefined. CMD_CTL's bits are all write-1-to-trigger, self-clearing
 * (writing 0 to any other bit is a no-op), so this single-bit write is
 * safe and doesn't disturb anything else in the register.
 */
static int lan9250_1588_enable(const struct device *dev)
{
	return lan9250_write_sys_reg(dev, LAN9250_1588_CMD_CTL, LAN9250_1588_CMD_CTL_ENABLE);
}

static int lan9250_1588_init(const struct device *dev)
{
	int ret;
	uint32_t general_config;

	/* The timestamp unit (RX/TX timestamping) defaults to enabled
	 * (reset value 1b), but set it explicitly rather than relying on
	 * that - it depends on the board's 1588_enable_strap configuration,
	 * which this project doesn't control. Must happen before
	 * lan9250_1588_enable() - see its comment.
	 */
	ret = lan9250_read_sys_reg(dev, LAN9250_1588_GENERAL_CONFIG, &general_config);
	if (ret < 0) {
		return ret;
	}

	general_config |= LAN9250_1588_GENERAL_CONFIG_TSU_ENABLE;

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_GENERAL_CONFIG, general_config);
	if (ret < 0) {
		return ret;
	}

	/* Load the clock to a known value (0) so early reads are
	 * meaningful before any real time source (network PTP sync, once
	 * implemented) has set it. CLOCK_LOAD carries no "must not change
	 * while enabled" restriction, so its ordering relative to
	 * lan9250_1588_enable() doesn't matter - done here for simplicity.
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CLOCK_SEC, 0);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CLOCK_NS, 0);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CMD_CTL, LAN9250_1588_CMD_CTL_CLOCK_LOAD);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("1588 PTP clock enabled");

	return 0;
}

int lan9250_ptp_clock_read(const struct device *dev, uint32_t *sec, uint32_t *ns,
			   uint32_t *subns)
{
	int ret;

	/* Snapshot the live, free-running clock into CLOCK_SEC/NS/SUBNS so
	 * they can be read back consistently (the clock keeps ticking
	 * between the three register reads below otherwise).
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CMD_CTL, LAN9250_1588_CMD_CTL_CLOCK_READ);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_read_sys_reg(dev, LAN9250_1588_CLOCK_SEC, sec);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_read_sys_reg(dev, LAN9250_1588_CLOCK_NS, ns);
	if (ret < 0) {
		return ret;
	}

	if (subns != NULL) {
		ret = lan9250_read_sys_reg(dev, LAN9250_1588_CLOCK_SUBNS, subns);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

/*
 * Phase 3: register-level primitives backing this driver's ptp_clock
 * device (drivers/eth_lan9250/ptp_clock_lan9250.c) - see
 * THEORY_OF_OPERATION.md's "IEEE 1588 / PTP" section.
 */

int lan9250_ptp_clock_set(const struct device *dev, uint32_t sec, uint32_t ns)
{
	int ret;

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CLOCK_SEC, sec);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CLOCK_NS, ns);
	if (ret < 0) {
		return ret;
	}

	return lan9250_write_sys_reg(dev, LAN9250_1588_CMD_CTL, LAN9250_1588_CMD_CTL_CLOCK_LOAD);
}

/*
 * Applies a one-time step of increment_ns nanoseconds (either sign, any
 * magnitude) to the clock. Implemented as a plain read-modify-write
 * (CLOCK_READ, adjust in software, CLOCK_LOAD) rather than the hardware's
 * single-tick CLOCK_STEP_ADJ/CMD_CTL mechanism: that mechanism only
 * supports subtraction via its seconds-portion step (addition-only for
 * the nanoseconds portion, per the datasheet), which would need
 * nanosecond/second-borrow composition to support arbitrary signed
 * nanosecond steps correctly. The read-modify-write approach is trivially
 * correct for any increment_ns value at the cost of the sub-tick timing
 * precision the hardware mechanism would otherwise offer - an acceptable
 * trade for the step sizes a PTP servo actually uses (dwarfed by normal
 * SPI transaction latency anyway).
 */
int lan9250_ptp_clock_adjust(const struct device *dev, int32_t increment_ns)
{
	int ret;
	uint32_t sec, ns;
	int64_t total_ns;

	ret = lan9250_ptp_clock_read(dev, &sec, &ns, NULL);
	if (ret < 0) {
		return ret;
	}

	total_ns = (int64_t)ns + increment_ns;
	sec += (uint32_t)(total_ns / 1000000000LL);
	total_ns %= 1000000000LL;

	if (total_ns < 0) {
		total_ns += 1000000000LL;
		sec -= 1;
	}

	return lan9250_ptp_clock_set(dev, sec, (uint32_t)total_ns);
}

/*
 * Applies a permanent rate trim, as a ratio relative to nominal (1.0 =
 * unadjusted, >1.0 = faster, <1.0 = slower) - the same convention
 * Zephyr's ptp_clock_driver_api.rate_adjust() uses, so this doubles as
 * its direct implementation. See eth_lan9250_priv.h's
 * LAN9250_1588_CLOCK_RATE_ADJ_* comment for the register's units.
 */
int lan9250_ptp_clock_rate_adjust(const struct device *dev, double ratio)
{
	double ppb = (ratio - 1.0) * 1.0e9;
	uint64_t magnitude;
	uint32_t rate_adj;

	magnitude = (uint64_t)(llround((ppb < 0 ? -ppb : ppb) * 42.94967296));

	if (magnitude > LAN9250_1588_CLOCK_RATE_ADJ_VALUE_MASK) {
		return -ERANGE;
	}

	rate_adj = (uint32_t)magnitude;
	if (ppb >= 0) {
		rate_adj |= LAN9250_1588_CLOCK_RATE_ADJ_DIR;
	}

	return lan9250_write_sys_reg(dev, LAN9250_1588_CLOCK_RATE_ADJ, rate_adj);
}

/*
 * Phase 2: arm a 1PPS output on GPIO1 (this board's pin 46, LED1/GPIO1/
 * TDI/MNGT1 - not connected to the RJ45's LEDs, just a 10k pull-down for
 * the MNGT1 boot strap, so it's free to use). Uses 1588 Clock Event
 * Channel A with a Clock Target Reload/Add of exactly 1 second in
 * increment ("auto-repeat") mode: once armed, the hardware itself
 * advances the Clock Target by 1s on every compare event and re-fires,
 * indefinitely, with zero CPU involvement per pulse - jitter is bounded
 * by the 1588 clock's own accuracy, not any software timing loop.
 *
 * Configured as a push/pull output, not open-drain: this board's pull
 * resistor on GPIO1 is a pull-DOWN, and the datasheet's open-drain-plus-
 * 1588-event behavior only ever drives the pin low or leaves it
 * floating - with a pull-down and no pull-up, that would never actually
 * read high on a scope. Push/pull drives a genuine, clean pulse instead.
 *
 * Uses "100ns pulse" event mode (not "toggle"): that's the actual PPS
 * convention - a sharp edge marking each second boundary - rather than a
 * 0.5Hz square wave.
 */
int lan9250_1588_pps_enable(const struct device *dev)
{
	int ret;
	uint32_t led_cfg, gpio_cfg, general_config, sec, ns;

	/* Make sure GPIO1 is in GPIO mode, not LED mode */
	ret = lan9250_read_sys_reg(dev, LAN9250_LED_CFG, &led_cfg);
	if (ret < 0) {
		return ret;
	}

	led_cfg &= ~LAN9250_LED_CFG_LED_EN(1);

	ret = lan9250_write_sys_reg(dev, LAN9250_LED_CFG, led_cfg);
	if (ret < 0) {
		return ret;
	}

	/* GPIO1: push/pull, active-high, 1588 channel A, 1588 output
	 * enabled (this also overrides GPIO_DATA_DIR's direction bit for
	 * GPIO1, per the datasheet - GPIOBUF is not overridden, hence
	 * setting it explicitly above).
	 */
	ret = lan9250_read_sys_reg(dev, LAN9250_GPIO_CFG, &gpio_cfg);
	if (ret < 0) {
		return ret;
	}

	gpio_cfg |= LAN9250_GPIO_CFG_GPIOBUF(1);
	gpio_cfg |= LAN9250_GPIO_CFG_GPIO_POL(1);
	gpio_cfg &= ~LAN9250_GPIO_CFG_1588_GPIO_CH_SEL(1);
	gpio_cfg |= LAN9250_GPIO_CFG_1588_GPIO_OE(1);

	ret = lan9250_write_sys_reg(dev, LAN9250_GPIO_CFG, gpio_cfg);
	if (ret < 0) {
		return ret;
	}

	/* Clock Event Channel A: 100ns pulse per compare event, increment
	 * (auto-repeat) mode rather than one-shot reload.
	 */
	ret = lan9250_read_sys_reg(dev, LAN9250_1588_GENERAL_CONFIG, &general_config);
	if (ret < 0) {
		return ret;
	}

	general_config &= ~LAN9250_1588_GENERAL_CONFIG_CLOCK_EVENT_A_MASK;
	general_config |= (LAN9250_1588_CLOCK_EVENT_MODE_100NS_PULSE
			    << LAN9250_1588_GENERAL_CONFIG_CLOCK_EVENT_A_SHIFT);
	general_config &= ~LAN9250_1588_GENERAL_CONFIG_RELOAD_ADD_A;

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_GENERAL_CONFIG, general_config);
	if (ret < 0) {
		return ret;
	}

	/* Reload/Add A = exactly 1 second: each compare event advances the
	 * Clock Target by 1s, so the pulse train self-sustains forever once
	 * armed.
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CLOCK_TARGET_RELOAD_SEC(0), 1);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CLOCK_TARGET_RELOAD_NS(0), 0);
	if (ret < 0) {
		return ret;
	}

	/* Seed the initial Clock Target a couple of seconds in the future
	 * (comfortably past this function's own remaining SPI transactions)
	 * and aligned to a whole-second boundary, so the pulse train lines
	 * up with the clock's own seconds tick from the very first pulse.
	 */
	ret = lan9250_ptp_clock_read(dev, &sec, &ns, NULL);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CLOCK_TARGET_SEC(0), sec + 2);
	if (ret < 0) {
		return ret;
	}

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CLOCK_TARGET_NS(0), 0);
	if (ret < 0) {
		return ret;
	}

	LOG_INF("1588 PPS output armed on GPIO1 (first pulse at clock second %u)", sec + 2);

	return 0;
}

/*
 * Phase 4: hardware RX/TX packet timestamping - see
 * THEORY_OF_OPERATION.md's "IEEE 1588 / PTP" section. Caller must hold
 * context->bank_lock.
 */
static int lan9250_1588_bank_select(const struct device *dev, uint32_t bank)
{
	return lan9250_write_sys_reg(dev, LAN9250_1588_BANK_PORT_GPIO_SEL, bank);
}

/*
 * Enables hardware timestamping of Sync/Delay_Req/Pdelay_Req/Pdelay_Resp
 * messages on both RX and TX. RX/TX *parsing* (detecting a frame is PTP
 * at all - RX_PARSE_CONFIG/TX_PARSE_CONFIG) is left at its reset default,
 * which already enables IPv4/IPv6/Layer2 detection with the standard PTP
 * multicast MAC/IP addresses - only *which message types get their
 * ingress/egress time recorded* (RX_TIMESTAMP_CONFIG/TX_TIMESTAMP_CONFIG)
 * needs an explicit write, since that defaults to all-disabled.
 *
 * Must run before lan9250_1588_enable() - see its comment. Read-modify-
 * write rather than a blind overwrite of the whole register: bits 19:16
 * (PTP version, reset default 2 = "v2 only") and others above the
 * message-enable field carry their own "must not change while
 * 1588_ENABLE is set" restriction, so a blind write that happened to run
 * after enable (as an earlier version of this driver did) wasn't just
 * risking those fields specifically - preserving them here is simply
 * correct regardless.
 */
static int lan9250_1588_timestamping_init(const struct device *dev)
{
	struct lan9250_runtime *context = dev->data;
	uint32_t rx_ts_config, tx_ts_config;
	int ret;

	k_mutex_lock(&context->bank_lock, K_FOREVER);

	ret = lan9250_1588_bank_select(dev, LAN9250_1588_BANK_SEL_PORT_RX);
	if (ret < 0) {
		goto out;
	}

	ret = lan9250_read_sys_reg(dev, LAN9250_1588_RX_TIMESTAMP_CONFIG, &rx_ts_config);
	if (ret < 0) {
		goto out;
	}

	rx_ts_config = (rx_ts_config & ~LAN9250_1588_RX_TIMESTAMP_CONFIG_MESSAGE_EN_MASK) |
		       LAN9250_1588_PTP_MESSAGE_EN_DEFAULT;

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_RX_TIMESTAMP_CONFIG, rx_ts_config);
	if (ret < 0) {
		goto out;
	}

	ret = lan9250_1588_bank_select(dev, LAN9250_1588_BANK_SEL_PORT_TX);
	if (ret < 0) {
		goto out;
	}

	ret = lan9250_read_sys_reg(dev, LAN9250_1588_TX_TIMESTAMP_CONFIG, &tx_ts_config);
	if (ret < 0) {
		goto out;
	}

	tx_ts_config = (tx_ts_config & ~LAN9250_1588_TX_TIMESTAMP_CONFIG_MESSAGE_EN_MASK) |
		       LAN9250_1588_PTP_MESSAGE_EN_DEFAULT;

	ret = lan9250_write_sys_reg(dev, LAN9250_1588_TX_TIMESTAMP_CONFIG, tx_ts_config);

out:
	k_mutex_unlock(&context->bank_lock);

	return ret;
}

/*
 * Parses just enough of a raw Ethernet frame to identify it as a PTP
 * message and extract messageType + sequenceId, for correlating against
 * the LAN9250's own captured RX_MSG_HEADER/TX_MSG_HEADER (see the
 * comment on lan9250_1588_rx_timestamp_check() below for why this
 * correlation is necessary at all).
 *
 * Recognizes: raw Ethernet-II PTP (EtherType 0x88F7) and UDP/IPv4 PTP
 * (destination port 319 "event" or 320 "general", the standard PTP
 * ports). Deliberately does NOT handle VLAN-tagged frames, IPv6, or SNAP
 * encapsulation - a real-world limitation worth revisiting if testing
 * turns up PTP traffic in one of those forms, but out of scope for this
 * phase's bring-up. IPv4 header length is read from the IHL field rather
 * than assumed to be the common no-options 20 bytes, since getting this
 * wrong would silently misalign every field read after it.
 *
 * frame/len: the raw frame bytes (this driver already has these
 * available as a flat buffer on both RX, via a stack copy below, and TX,
 * via lan9250_runtime::buf - no net_buf fragment walking needed either
 * way). Returns true and fills *msg_type and *seq_id if recognized as
 * PTP, false otherwise (including if the frame is simply too short for
 * the fields this function needs).
 */
static bool lan9250_ptp_parse_header(const uint8_t *frame, size_t len, uint8_t *msg_type,
				     uint16_t *seq_id)
{
	size_t ptp_off;
	uint16_t ethertype;

	if (len < 14) {
		return false;
	}

	ethertype = sys_get_be16(&frame[12]);

	if (ethertype == 0x88F7) {
		ptp_off = 14;
	} else if (ethertype == 0x0800) {
		uint8_t ihl;
		size_t udp_off;

		if (len < 15) {
			return false;
		}

		ihl = (frame[14] & 0x0F) * 4;
		if (ihl < 20 || len < 14 + ihl + 8) {
			return false;
		}

		if (frame[14 + 9] != 0x11 /* IPPROTO_UDP */) {
			return false;
		}

		udp_off = 14 + ihl;
		if (sys_get_be16(&frame[udp_off + 2]) != 319 &&
		    sys_get_be16(&frame[udp_off + 2]) != 320) {
			return false;
		}

		ptp_off = udp_off + 8;
	} else {
		return false;
	}

	/* Common PTP header (IEEE 1588-2008 Table 18/19): need through
	 * byte 31 (sequenceId) at minimum.
	 */
	if (len < ptp_off + 32) {
		return false;
	}

	*msg_type = frame[ptp_off] & 0x0F;
	*seq_id = sys_get_be16(&frame[ptp_off + 30]);

	return true;
}

#define LAN9250_1588_RX_PENDING_TIMEOUT_MS 2000

static bool lan9250_1588_msg_type_is_timestamped(uint8_t msg_type)
{
	return (BIT(msg_type) & LAN9250_1588_PTP_MESSAGE_EN_DEFAULT) != 0;
}

/* Drops any rx_pending/rx_unclaimed entries whose deadline has passed - a
 * counterpart that never showed up (genuinely lost, or missed its window
 * in the 4-deep hardware buffer under very bursty PTP traffic). Called
 * with bank_lock held.
 */
/* Counts how RX timestamp matching actually resolves in practice, to
 * distinguish LAN9250_1588_RX_PENDING_MAX's 2-second-per-entry timeout
 * being exercised as a rare fallback (as designed) from it becoming a
 * routine, structural source of RX buffer pool pressure under real multi-
 * board traffic. Printed periodically (every ~10s) rather than per-packet,
 * to avoid recreating the log-volume-induced RX backpressure bug fixed
 * earlier in this project's history.
 */
static uint32_t diag_matched_immediate;
static uint32_t diag_matched_pending;
static uint32_t diag_queued_pending;
static uint32_t diag_expired_unmatched;
static int64_t diag_last_summary_ms;

static void diag_rx_match_summary(void)
{
	int64_t now = k_uptime_get();

	if (now - diag_last_summary_ms < 10000) {
		return;
	}
	diag_last_summary_ms = now;

	LOG_INF("DIAG rx_match: immediate=%u pending_match=%u queued=%u expired=%u",
		diag_matched_immediate, diag_matched_pending, diag_queued_pending,
		diag_expired_unmatched);
}

static void lan9250_1588_rx_pending_expire(struct lan9250_runtime *context, int64_t now)
{
	for (size_t i = 0; i < LAN9250_1588_RX_PENDING_MAX; i++) {
		struct lan9250_1588_rx_pending *p = &context->rx_pending[i];

		if (p->pkt != NULL && now >= p->deadline) {
			LOG_DBG("1588 RX: pending entry expired unmatched (type=%u seq=%u)",
				p->msg_type, p->seq_id);
			diag_expired_unmatched++;
			net_pkt_unref(p->pkt);
			p->pkt = NULL;
		}
	}

	for (size_t i = 0; i < LAN9250_1588_RX_PENDING_MAX; i++) {
		struct lan9250_1588_rx_unclaimed *u = &context->rx_unclaimed[i];

		if (u->valid && now >= u->deadline) {
			LOG_DBG("1588 RX: unclaimed event expired unmatched (type=%u seq=%u)",
				u->msg_type, u->seq_id);
			u->valid = false;
		}
	}

	diag_rx_match_summary();
}

/*
 * Checks whether the LAN9250 just captured a new RX PTP timestamp and, if
 * it matches a frame this driver has already read off the wire, attaches
 * it (net_pkt_set_timestamp()).
 *
 * The hardware's ingress-timestamp capture (RX_INGRESS_SEC/NS +
 * RX_MSG_HEADER, up to LAN9250_1588_RX_PENDING_MAX events buffered in
 * CAP_INFO's RX_TS_CNT) does not become visible in CAP_INFO at the same
 * time the frame's data lands in the RX FIFO. Confirmed against real PTP
 * traffic (phase 4 bring-up): a bounded busy-wait retry (even up to 1ms,
 * 10x the first attempt) made no measurable difference to the match rate
 * - so this isn't a fixed short hardware pipeline delay polling can close.
 * It can lag by more than a second, i.e. by several *other* PTP frames'
 * worth of processing - "the newest queued RX timestamp" and "the PTP
 * message this function just received" are frequently different events.
 *
 * So instead of polling: every eligible (see
 * lan9250_1588_msg_type_is_timestamped()) frame's net_pkt is ref'd and
 * stashed in context->rx_pending before this function returns, unless a
 * matching hardware event was already available and got attached
 * directly. Every call - including for message types the hardware never
 * timestamps (Follow_Up, Announce, ...) - also drains one CAP_INFO event
 * if one is ready, and checks it against both the current frame and every
 * still-pending one, so a delayed event gets matched (and its packet's
 * timestamp attached, however late) no matter which later frame's
 * processing happens to be running when the hardware finally makes it
 * visible.
 *
 * The reverse can also happen: physical reception (wire-speed) and this
 * driver's SPI drain (comparatively slow, serialized one frame at a time)
 * run independently and can complete out of order, so a frame's hardware
 * capture has sometimes already shown up in CAP_INFO - during an *earlier*
 * frame's check - before this driver has even read that frame's own data
 * out of the FIFO yet. context->rx_unclaimed is the mirror of rx_pending
 * for exactly this: a captured event with nothing (yet) to attach it to.
 * Every call checks rx_unclaimed for the current frame first, before ever
 * touching CAP_INFO.
 *
 * Entries nobody ever claims - on either side - expire after
 * LAN9250_1588_RX_PENDING_TIMEOUT_MS so a lost/dropped counterpart can't
 * hold a net_pkt ref (rx_pending) or a slot (rx_unclaimed) forever.
 *
 * This still verifies messageType and sequenceId before attaching a
 * timestamp, rather than assuming a queue-order correspondence - cheap
 * insurance against a subtle mismatch that would otherwise be silent and
 * hard to debug. It does not check the 12-bit sourcePortIdentity CRC
 * RX_MSG_HEADER also provides (would need replicating the hardware's
 * CRC-12 algorithm in software) - a known gap if this ever needs to be
 * airtight against genuinely adversarial or very bursty multi-sender PTP
 * traffic.
 *
 * Any queued RX timestamp event found here is always acknowledged
 * (RX_TS_INT written to clear, which advances the hardware to the next
 * buffered event) whether or not it matched anything, so a mismatched/
 * stale event can never block future ones from being read.
 */
static void lan9250_1588_rx_timestamp_check(const struct device *dev, struct net_pkt *pkt,
					    size_t pkt_len)
{
	struct lan9250_runtime *context = dev->data;
	/* Must cover the worst-case UDP/IPv4 PTP header offset
	 * lan9250_ptp_parse_header() needs: 14 (Ethernet) + up to 60 (IPv4
	 * with max IHL/options) + 8 (UDP) + 32 (PTP header through
	 * sequenceId) = 114 bytes. 64 was sized for the raw L2 EtherType-
	 * 0x88F7 case only (14 + 32 = 46) from this project's gPTP-era
	 * bring-up, and silently truncated every UDP/IPv4-framed PTP message
	 * (all of CONFIG_PTP's traffic) below the length
	 * lan9250_ptp_parse_header() needs, so it always returned false and
	 * this function returned before ever attempting a timestamp match -
	 * the RX hardware timestamp was silently never attached to any
	 * Sync/Delay_Req/Pdelay_Req/Pdelay_Resp message.
	 */
	uint8_t header_buf[128];
	size_t header_len = MIN(pkt_len, sizeof(header_buf));
	uint8_t pkt_msg_type;
	uint16_t pkt_seq_id;
	uint32_t cap_info, msg_header, sec, ns;
	uint8_t hw_msg_type;
	uint16_t hw_seq_id;

	net_pkt_cursor_init(pkt);

	if (net_pkt_read(pkt, header_buf, header_len) < 0) {
		LOG_DBG("1588 RX: net_pkt_read failed");
		return;
	}

	net_pkt_cursor_init(pkt);

	if (!lan9250_ptp_parse_header(header_buf, header_len, &pkt_msg_type, &pkt_seq_id)) {
		return;
	}

	k_mutex_lock(&context->bank_lock, K_FOREVER);

	lan9250_1588_rx_pending_expire(context, k_uptime_get());

	bool attached = false;

	/* Check for an already-captured hardware event for *this* frame
	 * before even touching CAP_INFO - physical reception (wire-speed)
	 * and this driver's SPI drain (slow, serialized per frame) can
	 * complete out of order, so the capture may have arrived, and been
	 * stashed here by an earlier call, before this frame's own net_pkt
	 * even existed. See lan9250_1588_rx_unclaimed.
	 */
	for (size_t i = 0; i < LAN9250_1588_RX_PENDING_MAX; i++) {
		struct lan9250_1588_rx_unclaimed *u = &context->rx_unclaimed[i];

		if (u->valid && u->msg_type == pkt_msg_type && u->seq_id == pkt_seq_id) {
			struct net_ptp_time ts = {.second = u->sec, .nanosecond = u->ns};

			net_pkt_set_timestamp(pkt, &ts);
			LOG_DBG("1588 RX timestamp: %u.%09u (type=%u seq=%u) - "
				"matched an event captured before this frame was read",
				u->sec, u->ns, u->msg_type, u->seq_id);
			u->valid = false;
			attached = true;
			diag_matched_immediate++;
			break;
		}
	}

	if (attached) {
		goto queue;
	}

	if (lan9250_1588_bank_select(dev, LAN9250_1588_BANK_SEL_PORT_GENERAL) < 0) {
		LOG_DBG("1588 RX: bank select (general) failed");
		goto queue;
	}

	if (lan9250_read_sys_reg(dev, LAN9250_1588_CAP_INFO, &cap_info) < 0) {
		LOG_DBG("1588 RX: CAP_INFO read failed");
		goto queue;
	}

	if ((cap_info & LAN9250_1588_CAP_INFO_RX_TS_CNT_MASK) == 0) {
		goto queue;
	}

	if (lan9250_1588_bank_select(dev, LAN9250_1588_BANK_SEL_PORT_RX) < 0) {
		goto queue;
	}

	if (lan9250_read_sys_reg(dev, LAN9250_1588_RX_MSG_HEADER, &msg_header) < 0 ||
	    lan9250_read_sys_reg(dev, LAN9250_1588_RX_INGRESS_SEC, &sec) < 0 ||
	    lan9250_read_sys_reg(dev, LAN9250_1588_RX_INGRESS_NS, &ns) < 0) {
		goto queue;
	}

	hw_msg_type = (msg_header & LAN9250_1588_MSG_HEADER_MSG_TYPE_MASK) >>
		      LAN9250_1588_MSG_HEADER_MSG_TYPE_SHIFT;
	hw_seq_id = msg_header & LAN9250_1588_MSG_HEADER_SEQ_ID_MASK;

	/* Always acknowledge/drain now that MSG_HEADER/INGRESS_SEC/NS have
	 * been read - INT_STS is "na" bank, no bank switch needed. Done
	 * before the matching below so a mismatched/stale event can never
	 * block the next one from being read.
	 */
	(void)lan9250_write_sys_reg(dev, LAN9250_1588_INT_STS, LAN9250_1588_INT_STS_RX_TS_INT);

	if (hw_msg_type == pkt_msg_type && hw_seq_id == pkt_seq_id) {
		struct net_ptp_time ts = {.second = sec, .nanosecond = ns};

		net_pkt_set_timestamp(pkt, &ts);
		LOG_DBG("1588 RX timestamp: %u.%09u (type=%u seq=%u)", sec, ns, hw_msg_type,
			hw_seq_id);
		attached = true;
		diag_matched_immediate++;
	} else {
		bool matched_pending = false;

		for (size_t i = 0; i < LAN9250_1588_RX_PENDING_MAX; i++) {
			struct lan9250_1588_rx_pending *p = &context->rx_pending[i];

			if (p->pkt != NULL && p->msg_type == hw_msg_type &&
			    p->seq_id == hw_seq_id) {
				struct net_ptp_time ts = {.second = sec, .nanosecond = ns};

				net_pkt_set_timestamp(p->pkt, &ts);
				LOG_DBG("1588 RX timestamp: %u.%09u (type=%u seq=%u) - "
					"matched a pending earlier frame, not this one",
					sec, ns, hw_msg_type, hw_seq_id);
				net_pkt_unref(p->pkt);
				p->pkt = NULL;
				matched_pending = true;
				diag_matched_pending++;
				break;
			}
		}

		if (!matched_pending) {
			/* Doesn't match anything we know about yet - stash it
			 * rather than dropping it, in case it's for a frame
			 * that hasn't been read out of the RX FIFO yet (see
			 * lan9250_1588_rx_unclaimed and the check at the top
			 * of this function).
			 */
			struct lan9250_1588_rx_unclaimed *slot = NULL;

			for (size_t i = 0; i < LAN9250_1588_RX_PENDING_MAX; i++) {
				if (!context->rx_unclaimed[i].valid) {
					slot = &context->rx_unclaimed[i];
					break;
				}
			}

			if (slot == NULL) {
				slot = &context->rx_unclaimed[0];
				for (size_t i = 1; i < LAN9250_1588_RX_PENDING_MAX; i++) {
					if (context->rx_unclaimed[i].deadline < slot->deadline) {
						slot = &context->rx_unclaimed[i];
					}
				}
				LOG_DBG("1588 RX: unclaimed list full, evicting oldest "
					"(type=%u seq=%u)", slot->msg_type, slot->seq_id);
			}

			slot->valid = true;
			slot->msg_type = hw_msg_type;
			slot->seq_id = hw_seq_id;
			slot->sec = sec;
			slot->ns = ns;
			slot->deadline = k_uptime_get() + LAN9250_1588_RX_PENDING_TIMEOUT_MS;

			LOG_DBG("1588 RX timestamp event didn't match this or any pending "
				"packet (hw type=%u seq=%u, pkt type=%u seq=%u) - stashed as "
				"unclaimed",
				hw_msg_type, hw_seq_id, pkt_msg_type, pkt_seq_id);
		}
	}

queue:
	if (!attached && lan9250_1588_msg_type_is_timestamped(pkt_msg_type)) {
		struct lan9250_1588_rx_pending *slot = NULL;

		for (size_t i = 0; i < LAN9250_1588_RX_PENDING_MAX; i++) {
			if (context->rx_pending[i].pkt == NULL) {
				slot = &context->rx_pending[i];
				break;
			}
		}

		if (slot == NULL) {
			/* All slots full - evict the oldest (soonest deadline)
			 * to make room rather than silently giving up on this
			 * frame's chance at a timestamp. Shouldn't happen in
			 * practice: LAN9250_1588_RX_PENDING_MAX slots and a
			 * LAN9250_1588_RX_PENDING_TIMEOUT_MS timeout give far
			 * more headroom than this message type's real traffic
			 * rate needs.
			 */
			slot = &context->rx_pending[0];
			for (size_t i = 1; i < LAN9250_1588_RX_PENDING_MAX; i++) {
				if (context->rx_pending[i].deadline < slot->deadline) {
					slot = &context->rx_pending[i];
				}
			}
			LOG_WRN("1588 RX: pending list full, evicting oldest (type=%u seq=%u)",
				slot->msg_type, slot->seq_id);
			net_pkt_unref(slot->pkt);
		}

		net_pkt_ref(pkt);
		slot->pkt = pkt;
		slot->msg_type = pkt_msg_type;
		slot->seq_id = pkt_seq_id;
		slot->deadline = k_uptime_get() + LAN9250_1588_RX_PENDING_TIMEOUT_MS;
		diag_queued_pending++;

		/* DIAGNOSTIC (phase 5 gPTP bring-up): shows exactly what
		 * every rx_pending entry is waiting for, to cross-reference
		 * against later "stashed as unclaimed"/"expired unmatched"
		 * lines and see whether the expected hw event ever actually
		 * shows up with a mismatched type/seq, or never shows up at
		 * all. Remove once the Pdelay reliability issue is resolved.
		 */
		LOG_DBG("1588 RX: added to pending (type=%u seq=%u)", pkt_msg_type, pkt_seq_id);
	}

	k_mutex_unlock(&context->bank_lock);
}

/*
 * TX counterpart of lan9250_1588_rx_timestamp_check() above - same
 * correlation strategy and same caveats, called from lan9250_tx() after
 * it has queued the frame into the LAN9250's TX FIFO.
 *
 * Unlike the RX side, this can't simply check CAP_INFO once: this
 * driver's TX completion isn't interrupt-driven at all (lan9250_thread()
 * only services PHY link and RX FIFO level interrupts - see that
 * function above), and the status-FIFO drain loop at the top of
 * lan9250_tx() drains whatever backlog was *already* pending from prior
 * sends, read before the current frame is even written - it doesn't
 * confirm *this* frame's own completion. Physical transmission (and so
 * the 1588 egress capture) genuinely may not have happened yet by the
 * time lan9250_write_buf() returns. So this retries a bounded number of
 * times with a short delay between attempts rather than checking once -
 * cheap and bounded (worst case is a full one -1518-byte frame's wire
 * time, ~1.2ms even at 10Mbps), and avoids redesigning this driver's TX
 * completion signaling into something interrupt-driven, which is out of
 * scope for this phase.
 *
 * frame/len here is lan9250_tx()'s own already-serialized copy of the
 * outgoing frame (context->buf), not pkt itself - simpler than re-reading
 * from pkt a second time, and exactly the bytes that went out the wire.
 *
 * Bring-up note (phase 4 TX, confirmed working against real L2-framed PTP
 * traffic - see THEORY_OF_OPERATION.md's "Hardware TX packet
 * timestamping" section): CAP_INFO's TX_TS_CNT never incremented for any
 * outbound PTP frame whose common header left messageLength (bytes 2-3)
 * at 0, no matter what else was tried. The datasheet's own "Transmit
 * Message Egress Time Recording" section (14.2.2.3) doesn't list
 * messageLength among its documented gating conditions (only messageType
 * enable, versionPTP, domain, alt-master, FCS/checksum) - so this
 * requirement is real but apparently undocumented. Any code constructing
 * a PTP frame for this driver to transmit must set a real messageLength.
 */
#define LAN9250_1588_TX_TS_POLL_ATTEMPTS 5
#define LAN9250_1588_TX_TS_POLL_DELAY    K_MSEC(1)

static void lan9250_1588_tx_timestamp_check(const struct device *dev, struct net_pkt *pkt,
					    const uint8_t *frame, size_t len)
{
	struct lan9250_runtime *context = dev->data;
	uint8_t pkt_msg_type;
	uint16_t pkt_seq_id;
	uint32_t cap_info, msg_header, sec, ns;
	uint8_t hw_msg_type;
	uint16_t hw_seq_id;
	int attempt;

	if (!lan9250_ptp_parse_header(frame, len, &pkt_msg_type, &pkt_seq_id)) {
		return;
	}

	k_mutex_lock(&context->bank_lock, K_FOREVER);

	if (lan9250_1588_bank_select(dev, LAN9250_1588_BANK_SEL_PORT_GENERAL) < 0) {
		LOG_DBG("1588 TX: bank select (general) failed");
		goto out;
	}

	for (attempt = 0; attempt < LAN9250_1588_TX_TS_POLL_ATTEMPTS; attempt++) {
		if (lan9250_read_sys_reg(dev, LAN9250_1588_CAP_INFO, &cap_info) < 0) {
			LOG_DBG("1588 TX: CAP_INFO read failed");
			goto out;
		}

		if ((cap_info & LAN9250_1588_CAP_INFO_TX_TS_CNT_MASK) != 0) {
			break;
		}

		k_sleep(LAN9250_1588_TX_TS_POLL_DELAY);
	}

	if ((cap_info & LAN9250_1588_CAP_INFO_TX_TS_CNT_MASK) == 0) {
		goto out;
	}

	if (lan9250_1588_bank_select(dev, LAN9250_1588_BANK_SEL_PORT_TX) < 0) {
		LOG_DBG("1588 TX: bank select (TX) failed");
		goto out;
	}

	if (lan9250_read_sys_reg(dev, LAN9250_1588_TX_MSG_HEADER, &msg_header) < 0 ||
	    lan9250_read_sys_reg(dev, LAN9250_1588_TX_EGRESS_SEC, &sec) < 0 ||
	    lan9250_read_sys_reg(dev, LAN9250_1588_TX_EGRESS_NS, &ns) < 0) {
		LOG_DBG("1588 TX: MSG_HEADER/EGRESS_SEC/NS read failed");
		goto out;
	}

	hw_msg_type = (msg_header & LAN9250_1588_MSG_HEADER_MSG_TYPE_MASK) >>
		      LAN9250_1588_MSG_HEADER_MSG_TYPE_SHIFT;
	hw_seq_id = msg_header & LAN9250_1588_MSG_HEADER_SEQ_ID_MASK;

	if (hw_msg_type == pkt_msg_type && hw_seq_id == pkt_seq_id) {
		struct net_ptp_time ts = {.second = sec, .nanosecond = ns};

		net_pkt_set_timestamp(pkt, &ts);
		/* net_pkt_set_timestamp() alone only updates pkt's own
		 * metadata - it does not notify anyone waiting on this
		 * specific pkt via net_if_register_timestamp_cb(). gPTP's
		 * Pdelay_Resp handling (gptp_handle_pdelay_req(),
		 * subsys/net/l2/ethernet/gptp/gptp_messages.c) registers
		 * exactly such a callback on its own reply packet, to know
		 * when its real TX egress time is available so it can send
		 * the matching Pdelay_Resp_Follow_Up message - without this
		 * call, that callback never fires and Pdelay_Resp_Follow_Up
		 * is never sent (found via a live two-board gPTP bring-up:
		 * Pdelay_Req/Resp exchanged fine, but every reply-side board
		 * logged "Multiple pdelay requests" the moment a second
		 * request arrived while the first's still-registered,
		 * never-fired callback sat stale).
		 */
#if defined(CONFIG_NET_PKT_TIMESTAMP_THREAD)
		net_if_add_tx_timestamp(pkt);
#endif
		LOG_DBG("1588 TX timestamp: %u.%09u (type=%u seq=%u)", sec, ns, hw_msg_type,
			hw_seq_id);
	} else {
		LOG_DBG("1588 TX timestamp event didn't match transmitted packet "
			"(hw type=%u seq=%u, pkt type=%u seq=%u) - dropping stale event",
			hw_msg_type, hw_seq_id, pkt_msg_type, pkt_seq_id);
	}

	(void)lan9250_write_sys_reg(dev, LAN9250_1588_INT_STS, LAN9250_1588_INT_STS_TX_TS_INT);

out:
	k_mutex_unlock(&context->bank_lock);
}

static int lan9250_init(const struct device *dev)
{
	int ret;
	const struct lan9250_config *config = dev->config;
	struct lan9250_runtime *context = dev->data;

	/* SPI config */
	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI master port %s not ready", config->spi.bus->name);
		return -EINVAL;
	}

	/* Initialize GPIO */
	if (!gpio_is_ready_dt(&config->interrupt)) {
		LOG_ERR("GPIO port %s not ready", config->interrupt.port->name);
		return -EINVAL;
	}

	ret = gpio_pin_configure_dt(&config->interrupt, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Unable to configure GPIO pin %u", config->interrupt.pin);
		return ret;
	}

	gpio_init_callback(&(context->gpio_cb), lan9250_gpio_callback,
			   BIT(config->interrupt.pin));
	ret = gpio_add_callback(config->interrupt.port, &context->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Unable to add GPIO callback %u", config->interrupt.pin);
		return ret;
	}

	ret = gpio_pin_interrupt_configure_dt(&config->interrupt,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Unable to enable GPIO INT %u", config->interrupt.pin);
		return ret;
	}

	if (config->reset.port != NULL) {
		if (!gpio_is_ready_dt(&config->reset)) {
			LOG_ERR("GPIO port %s not ready", config->reset.port->name);
			return -EINVAL;
		}

		ret = gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			LOG_ERR("Unable to configure GPIO pin %u", config->reset.pin);
			return ret;
		}

		/* See Section 19.6.3 from the LAN9250 Data Sheet
		 *
		 * trstia is 200 microseconds min (use 250 us)
		 * tcfg is 15 milliseconds min (use 20 ms for after reset)
		 */
		gpio_pin_set_dt(&config->reset, 1);
		k_usleep(250);
		gpio_pin_set_dt(&config->reset, 0);
		k_msleep(20);
	}

	/* Reset and wait for ready on the LAN9250 SPI device */
	ret = lan9250_sw_reset(dev);
	if (ret < 0) {
		LOG_ERR("Reset failed");
		return ret;
	}
	ret = lan9250_configure(dev);
	if (ret < 0) {
		LOG_ERR("Configuration failed");
		return ret;
	}

	(void)net_eth_mac_load(&config->mac_cfg, context->mac_address);
	lan9250_load_unique_mac_address(context);
	ret = lan9250_set_macaddr(dev);
	if (ret < 0) {
		LOG_ERR("Set mac address failed");
		return ret;
	}

	ret = lan9250_1588_init(dev);
	if (ret < 0) {
		LOG_ERR("1588 PTP clock init failed");
		return ret;
	}

	ret = lan9250_1588_timestamping_init(dev);
	if (ret < 0) {
		LOG_ERR("1588 timestamping init failed");
		return ret;
	}

	ret = lan9250_1588_enable(dev);
	if (ret < 0) {
		LOG_ERR("1588 enable failed");
		return ret;
	}

	LOG_INF("LAN9250 Initialized");

	return 0;
}

#define LAN9250_DEFINE(inst)                                                                       \
	static struct lan9250_runtime lan9250_##inst##_runtime = {                                 \
		.tx_rx_sem = Z_SEM_INITIALIZER(lan9250_##inst##_runtime.tx_rx_sem, 1, UINT_MAX),   \
		.int_sem = Z_SEM_INITIALIZER(lan9250_##inst##_runtime.int_sem, 0, UINT_MAX),       \
		.bank_lock = Z_MUTEX_INITIALIZER(lan9250_##inst##_runtime.bank_lock),              \
		.hash_lock = Z_MUTEX_INITIALIZER(lan9250_##inst##_runtime.hash_lock),              \
	};                                                                                         \
                                                                                                   \
	static const struct lan9250_config lan9250_##inst##_config = {                             \
		.spi = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8)),                                \
		.interrupt = GPIO_DT_SPEC_INST_GET(inst, int_gpios),                               \
		.reset = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                         \
		.mac_cfg = NET_ETH_MAC_DT_INST_CONFIG_INIT(inst),                                  \
	};                                                                                         \
                                                                                                   \
	ETH_NET_DEVICE_DT_INST_DEFINE(inst, lan9250_init, NULL, &lan9250_##inst##_runtime,         \
				      &lan9250_##inst##_config, CONFIG_ETH_INIT_PRIORITY,          \
				      &api_funcs, NET_ETH_MTU);
DT_INST_FOREACH_STATUS_OKAY(LAN9250_DEFINE);
