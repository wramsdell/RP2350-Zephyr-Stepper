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
#include <zephyr/drivers/spi.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/ethernet.h>
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
	 *   - Pass all multicast frames
	 *   - Hash filtering disabled
	 *   - Promiscuous disabled
	 *
	 * Upstream Zephyr's driver has this exact comment but never actually
	 * sets MCPAS, so "pass all multicast frames" was never true - the
	 * hardware RX filter silently dropped every multicast frame (mDNS,
	 * IGMP) regardless of anything at the IP/IGMP layer. Fixed here; see
	 * the file header comment and THEORY_OF_OPERATION.md.
	 */
	ret = lan9250_write_mac_reg(dev, LAN9250_HMAC_CR,
				    LAN9250_HMAC_CR_PADSTR | LAN9250_HMAC_CR_TXEN |
					    LAN9250_HMAC_CR_RXEN | LAN9250_HMAC_CR_FDPX |
					    LAN9250_HMAC_CR_MCPAS);
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

static int lan9250_rx(const struct device *dev)
{
	struct lan9250_runtime *ctx = dev->data;
	const uint16_t buf_rx_size = CONFIG_NET_BUF_DATA_SIZE;
	struct net_pkt *pkt;
	struct net_buf *pkt_buf;
	uint16_t pkt_len;
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

	/* Read dummy  data */
	ret = lan9250_read_sys_reg(dev, LAN9250_RX_DATA_FIFO, &tmp);
	if (ret < 0) {
		return ret;
	}
	pkt_len -= 4;

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

	ret = lan9250_read_sys_reg(dev, LAN9250_RX_DATA_FIFO, &tmp);
	if (ret < 0) {
		return ret;
	}

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
				LOG_DBG("LINK UP");
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

	return ETHERNET_LINK_10BASE | ETHERNET_LINK_100BASE
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

				reg |= LAN9250_HMAC_CR_MCPAS;
				reg &= ~LAN9250_HMAC_CR_PRMS;
				reg &= ~LAN9250_HMAC_CR_HO;
			}

			return lan9250_write_mac_reg(dev, LAN9250_HMAC_CR, reg);
		}

		break;
	default:
		break;
	}

	return -ENOTSUP;
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

static const struct ethernet_api api_funcs = {
	.iface_api.init = lan9250_iface_init,
	.get_capabilities = lan9250_get_capabilities,
	.set_config = lan9250_set_config,
	.send = lan9250_tx,
	.get_ptp_clock = lan9250_get_ptp_clock,
};

/*
 * IEEE 1588 (PTP) hardware clock - Phase 1 bring-up (see
 * THEORY_OF_OPERATION.md's "IEEE 1588 / PTP" section for the project's
 * phased plan, and the LAN9250 datasheet Section 14.0 for the hardware
 * this is built on). This phase only enables the clock and gets it
 * readable; no Zephyr ptp_clock integration, GPIO event output, or
 * network timestamping yet.
 */
static int lan9250_1588_init(const struct device *dev)
{
	int ret;
	uint32_t general_config;

	/* Enable the 1588 unit. CMD_CTL's bits are all write-1-to-trigger,
	 * self-clearing (writing 0 to any other bit is defined as a no-op),
	 * so a single-bit write here is safe and doesn't disturb anything
	 * else in the register.
	 */
	ret = lan9250_write_sys_reg(dev, LAN9250_1588_CMD_CTL, LAN9250_1588_CMD_CTL_ENABLE);
	if (ret < 0) {
		return ret;
	}

	/* The timestamp unit (RX/TX timestamping, not yet used by this
	 * phase) defaults to enabled (reset value 1b), but set it
	 * explicitly rather than relying on that - it depends on the
	 * board's 1588_enable_strap configuration, which this project
	 * doesn't control.
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
	 * implemented) has set it.
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

	LOG_INF("LAN9250 Initialized");

	return 0;
}

#define LAN9250_DEFINE(inst)                                                                       \
	static struct lan9250_runtime lan9250_##inst##_runtime = {                                 \
		.tx_rx_sem = Z_SEM_INITIALIZER(lan9250_##inst##_runtime.tx_rx_sem, 1, UINT_MAX),   \
		.int_sem = Z_SEM_INITIALIZER(lan9250_##inst##_runtime.int_sem, 0, UINT_MAX),       \
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
