/*	$NetBSD$	*/

/*
 * Copyright (c) 2025 Hiroki Mori
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY GENETEC CORPORATION ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL GENETEC CORPORATION
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef	_ARM_MINDSPEED_M83XXX_PCIE_H_
#define	_ARM_MINDSPEED_M83XXX_PCIE_H_

struct m83pcie_ih;

struct m83pcie_softc {
	device_t sc_dev;

	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	bus_space_handle_t sc_indirect_ioh;
	bus_space_handle_t sc_phy_ioh;
	bus_dma_tag_t sc_dmat;

	paddr_t sc_root_addr;
	size_t sc_root_size;

	struct arm32_pci_chipset sc_pc;

	TAILQ_HEAD(, m83pcie_ih) sc_intrs;

	void *sc_ih;
	kmutex_t sc_lock;
	u_int sc_intrgen;

	void *sc_cookie;
	void (* sc_pci_netbsd_configure)(void *);

	int is_endpoint;
};

struct m83pcie_ih {
	int (*ih_handler)(void *);
	void *ih_arg;
	int ih_ipl;
	TAILQ_ENTRY(m83pcie_ih) ih_entry;
};

int m83pcie_intr(void *);
void m83pcie_attach_common(struct m83pcie_softc *);

#define PCIE_SUBSYS_CTRL		0x00	/* PCIE_CTL */
#define PCIE_SUBSYS_CFG			0x04	/* PCIE_CFG */

#define PCIE_INT_RAW_STATUS		0x08	/* PCIE_IRQ */

#define PCIE_INT_ROUTING_CPU_ENABLE	0x0C	/* PCIE_IE0 */
#define PCIE_INT_ROUTING_CPU_DISABLE	0x10	/* PCIE_ID0 */
#define PCIE_INT_ROUTING_CPU_MASK	0x14	/* PCIE_IM0 */

#define PCIE_INT_ROUTING_MSI1_ENABLE	0x18	/* PCIE_IE1 */
#define PCIE_INT_ROUTING_MSI1_DISABLE	0x1C	/* PCIE_ID1 */
#define PCIE_INT_ROUTING_MSI1_MASK	0x20	/* PCIE_IM1 */

#define PCIE_INT_ROUTING_MSI2_ENABLE	0x24	/* PCIE_IE2 */
#define PCIE_INT_ROUTING_MSI2_DISABLE	0x28	/* PCIE_ID2 */
#define PCIE_INT_ROUTING_MSI2_MASK	0x2C	/* PCIE_IM2 */

#define PCIE_INT_ROUTING_MSI3_ENABLE	0x30	/* PCIE_IE3 */
#define PCIE_INT_ROUTING_MSI3_DISABLE	0x34	/* PCIE_ID3 */
#define PCIE_INT_ROUTING_MSI3_MASK	0x38	/* PCIE_IM3 */

#define PCIE_INT_ROUTING_MSI4_ENABLE	0x3C	/* PCIE_IE3 */
#define PCIE_INT_ROUTING_MSI4_DISABLE	0x40	/* PCIE_ID4 */
#define PCIE_INT_ROUTING_MSI4_MASK	0x44	/* PCIE_IM4 */

#define PCIE_INT_TEST			0x48	/* PCIE_ITS */

#define PCIE_UPST_INT_RAW_STATUS	0x4C	/* US_IRQ */
#define PCIE_UPST_INT_ENABLE		0x50	/* US_IE */
#define PCIE_UPST_INT_DISABLE		0x54	/* US_ID */
#define PCIE_UPST_INT_MASK		0x58	/* US_IM */
#define PCIE_UPST_INT_TEST		0x5C	/* US_ITS */

#define PCIE_MSI_CH0_ADDR		0x60	/* MSI_CH0 */
#define PCIE_MSI_CH1_ADDR		0x64	/* MSI_CH1 */
#define PCIE_MSI_CH2_ADDR		0x68	/* MSI_CH2 */
#define PCIE_MSI_CH3_ADDR		0x6C	/* MSI_CH3 */

#define PCIE_POWER_MANAGEMENT_STATUS	0x70	/* PM_STS */

#define PCIE_BAR0_ADDR_OFFSET		0x74	/* BAR0_LA */
#define PCIE_BAR1_ADDR_OFFSET		0x78	/* BAR1_LA */
#define PCIE_BAR2_ADDR_OFFSET		0x7C	/* BAR2_LA */

#define PCIE_BAR0_SIZE			0x80	/* BAR0_MSK */
#define PCIE_BAR1_SIZE			0x84	/* BAR1_MSK */
#define PCIE_BAR2_SIZE			0x88	/* BAR2_MSK */

#define PCIE_DMA_CTRL_CFG		0x8C	/* DMA_CTL */

#define PCIE_CH0_DMA_CTRL_CFG		0x90	/* CH0_CTL */

#define PCIE_CH0_DMA_INT_RAW_STATUS	0x94	/* CH0_IRQ */
#define PCIE_CH0_DMA_INT_ENABLE		0x98	/* CH0_IE */
#define PCIE_CH0_DMA_INT_DISABLE	0x9C	/* CH0_ID */
#define PCIE_CH0_DMA_INT_MASK		0xA0	/* CH0_IM */
#define PCIE_CH0_DMA_INT_TEST		0xA4	/* CH0_ITS */

#define PCIE_CH0_MSG_STATUS		0xA8	/* CH0_MSG */
#define PCIE_CH0_DESC_QUEUE_ADDR	0xAC	/* CH0_QAD */
#define PCIE_CH0_DESC_QUEUE_SIZE	0xB0	/* CH0_QSZ */
#define PCIE_CH0_DESC_QUEUE_NEW		0xB4	/* CH0_QNW */
#define PCIE_CH0_DESC_QUEUE_COUNT	0xB8	/* CH0_QCN */
#define PCIE_CH0_DESC_QUEUE_CURRENT	0xBC	/* CH0_QPT */

#define PCIE_CH1_DMA_CTRL_CFG		0xC0	/* CH1_CTL */

#define PCIE_CH1_DMA_INT_RAW_STATUS	0xC4	/* CH1_IRQ */
#define PCIE_CH1_DMA_INT_ENABLE		0xC8	/* CH1_IE */
#define PCIE_CH1_DMA_INT_DISABLE	0xCC	/* CH1_ID */
#define PCIE_CH1_DMA_INT_MASK		0xD0	/* CH1_IM */
#define PCIE_CH1_DMA_INT_TEST		0xD4	/* CH1_ITS */

#define PCIE_PEX_CFG1			0xD8	/* PEX_CFG1 */
#define PCIE_PEX_CFG2			0xDC	/* PEX_CFG2 */
#define PCIE_PEX_CFG3			0xE0	/* PEX_CFG3 */

#define PCIE_PEX_ERROR_STATUS		0xE4	/* PEX_ERR */
#define PCIE_PEX_ERROR_MASK		0xE8	/* PEX_MSK */

#define PCIE_BIST_CTRL_STATUS		0xEC	/* PCIE_BIST */

#define PCIE_MEM_SENSE_AMP_ADJUST	0xF0	/* PCIE_SNS */

/* PEX configuration registers */
#define PCIE_PEX_IP_BASEADDR		0x1000

#define PCIE_PEX_IP_COMMAND_STATUS	(PCIE_PEX_IP_BASEADDR + 0x04)
#define PCIE_PEX_IP_BAR0		(PCIE_PEX_IP_BASEADDR + 0x10)
#define PCIE_PEX_IP_BAR1		(PCIE_PEX_IP_BASEADDR + 0x14)
#define PCIE_PEX_IP_BAR2		(PCIE_PEX_IP_BASEADDR + 0x18)
#define PCIE_PEX_IP_BAR3		(PCIE_PEX_IP_BASEADDR + 0x1C)
#define PCIE_PEX_IP_BAR4		(PCIE_PEX_IP_BASEADDR + 0x20)
#define PCIE_PEX_IP_BAR5		(PCIE_PEX_IP_BASEADDR + 0x24)


/* Indirect command registers */
#define PCIE_CH0_DMA_REMOTE_ADDR_LOW	0x00	/* PCIE_CH0_ALO */
#define PCIE_CH0_DMA_REMOTE_ADDR_HIGH	0x04	/* PCIE_CH0_AHI */
#define PCIE_CH0_DMA_LOCAL_ADDR		0x08	/* PCIE_CH0_LA */
#define PCIE_CH0_DMA_START		0x0C	/* PCIE_CH0_STR */
#define PCIE_CH0_DMA_SIMPLE_WRITE	0x10	/* PCIE_CH0_SWD */
#define PCIE_CH0_DMA_SIMPLE_READ	0x14	/* PCIE_CH0_SRD */
#define PCIE_CH0_DMA_STATUS		0x18	/* PCIE_CH0_STS */

#define PCIE_CH1_DMA_REMOTE_ADDR_LOW	0x20	/* PCIE_CH1_ALO */
#define PCIE_CH1_DMA_REMOTE_ADDR_HIGH	0x24	/* PCIE_CH1_AHI */
#define PCIE_CH1_DMA_LOCAL_ADDR		0x28	/* PCIE_CH1_LA */
#define PCIE_CH1_DMA_START		0x2C	/* PCIE_CH1_STR */
#define PCIE_CH1_DMA_SIMPLE_WRITE	0x30	/* PCIE_CH1_SWD */
#define PCIE_CH1_DMA_SIMPLE_READ	0x34	/* PCIE_CH1_SRD */
#define PCIE_CH1_DMA_STATUS		0x38	/* PCIE_CH1_STS */


/* PCIe PHY registers */
#define PCIE_PHY_PARALLEL_CR_CTRL_PORT_ADDR	0x00	/* CR_ADDR */
#define PCIE_PHY_PARALLEL_CR_CTRL_PORT_DATA	0x04	/* CR_DATA */
#define PCIE_PHY_POWER_GOOD_STATUS		0x08	/* PG_STS */
#define PCIE_PHY_MPLL_CTRL			0x0C	/* MPLL_CTL */
#define PCIE_PHY_TEST_CTRL			0x10	/* TEST_CTL */
#define PCIE_PHY_TRANSMIT_LEVEL_CTRL		0x14	/* TX_LVL_CTL */
#define PCIE_PHY_LANE0_TX_CTRL			0x18	/* TX0_CTL */
#define PCIE_PHY_LANE1_TX_CTRL			0x1C	/* TX1_CTL */
#define PCIE_PHY_LOS_LEVEL_CTRL			0x20	/* LOS_LVL_CTL */
#define PCIE_PHY_LANE0_RX_CTRL			0x24	/* RX0_CTL */
#define PCIE_PHY_LANE1_RX_CTRL			0x28	/* RX1_CTL */
#define PCIE_PHY_TECHNOLOGY_CTRL		0x2C	/* TECH_CTL */
#define PCIE_PHY_RESISTOR_TUNE_CTRL		0x30	/* RTUNE_CTL */
#define PCIE_PHY_PCS_STATUS			0x34	/* PCS_STS */
#define PCIE_PHY_PCS_CTRL			0x38	/* PCS_CTL */

/***** Masks *****/

/* PCIE_CTL bits */
#define PCIE_CTL_ENDPOINT_MODE		(1 << 0)
#define PCIE_CTL_RW1C			(1 << 1)
#define PCIE_CTL_CFG_READY		(1 << 2)

/* PCIE_IRQ bits */
#define PCIE_IRQ_POWER_MANAGEMENT	(1 << 0)
#define PCIE_IRQ_TL_ERROR		(1 << 1)
#define PCIE_IRQ_PEX_REJECT		(1 << 2)
#define PCIE_IRQ_TX_REQUEST_ERROR	(1 << 3)
#define PCIE_IRQ_DMA_CH0		(1 << 6)
#define PCIE_IRQ_DMA_CH1		(1 << 7)
#define PCIE_IRQ_VC0_LINK_UP		(1 << 8)
#define PCIE_IRQ_VC0_LINK_DOWN		(1 << 9)

#define PCIE_IRQ_ALL			(PCIE_IRQ_POWER_MANAGEMENT | PCIE_IRQ_TL_ERROR | PCIE_IRQ_PEX_REJECT | \
					PCIE_IRQ_TX_REQUEST_ERROR | PCIE_IRQ_DMA_CH0 | PCIE_IRQ_DMA_CH1 | \
					PCIE_IRQ_VC0_LINK_UP | PCIE_IRQ_VC0_LINK_DOWN)


/* US_IRQ bits */
#define US_IRQ_MSI0			(1 << 0)
#define US_IRQ_MSI1			(1 << 1)
#define US_IRQ_MSI2			(1 << 2)
#define US_IRQ_MSI3			(1 << 3)
#define US_IRQ_INTA			(1 << 4)
#define US_IRQ_INTB			(1 << 5)
#define US_IRQ_INTC			(1 << 6)
#define US_IRQ_INTD			(1 << 7)

#define US_IRQ_ALL			(US_IRQ_MSI1 | US_IRQ_MSI2 | US_IRQ_MSI3 | US_IRQ_MSI4 | \
					US_IRQ_INTA | US_IRQ_INTB | US_IRQ_INTC | US_IRQ_INTD)

/* DMA_CTL bits */
#define PCIE_DMA_CTL_REQUESTER_ENABLE	(1 << 0)
#define PCIE_DMA_CTL_COMPLETER_ENABLE	(1 << 1)
#define PCIE_DMA_CTL_AHB64BIT_OVERWRITE	(1 << 7)
#define PCIE_DMA_CTL_AHB_EARLY_BURST_TERMINATION	(1 << 8)

/* BARn_MSK bits */
#define BAR_MSK_PREFETCHABLE		(1 << 4)

/* PEX_CFG1 bits */
#define PCIE_PEX_CFG1_BAR_MATCH_ENABLE	(1U << 31)

/* PEX_ERR bits */
#define PCIE_PEX_ERR_LTSSM_STATE(v)	(((v) >> 24) & 0xff)

/* PCIE_CHx_STR bits */ 
#define PCIE_CH_DW_LENTGH(v)		(((v) & 0x3f) << 22)
#define PCIE_CH_STR_START		(1 << 20)
#define PCIE_CH_STR_LAST_ADDR_BE(v)	(((v) & 0xf) << 16)
#define PCIE_CH_STR_FIRST_ADDR_BE(v)	(((v) & 0xf) << 12)
#define PCIE_CH_STR_ATTR(v)		(((v) & 0x2) << 10)
#define PCIE_CH_STR_TYPE(v)		(((v) & 0xf) << 5)
#define PCIE_CH_STR_CLASS(v)		(((v) & 0x7) << 1)
#define TYPE_MEM32		0x0
#define TYPE_MEM64		0x1
#define TYPE_MSG		0x2
#define TYPE_CFG0		0x3
#define TYPE_CFG1		0x4
#define PCIE_CH_STR_DIR_WRITE		(1 << 4)
#define PCIE_CH_STR_SIMPLE_MODE		(1 << 0)

/* PCIE_CHx_STS bits */
#define PCIE_CH_STS_DMA_REQ_STATUS_GET(v)	(((v) >> 11) & 0x7)
#define PCIE_CH_STS_SIMPLE_REQ_STATUS_GET(v)	(((v) >> 1) & 0x7)
#define PCIE_CH_STS_SIMPLE_REQ_BUSY		(1 << 0)

/* CHx_CTL bits */
#define PCIE_CH_CTL_BULK_MODE_ENABLE		(1 << 12)
#define PCIE_CH_CTL_WEIGHT(v)			(((v) & 0xF) << 8)
#define PCIE_CH_CTL_CMD_QUEUE_THRESHLD(v)	(((v) & 0x7) << 1)
#define PCIE_CH_CTL_CMD_QUEUE_FLUSH		(1 << 0)

/* TECH_CTL bits */
#define PCIE_TECH_CTL_FAST_TECH		(1 << 2)
#define PCIE_TECH_CTL_VP_IS_1P2		(1 << 1)
#define PCIE_TECH_CTL_VPH_IS_3P3	(1 << 0)

#ifndef __ASSEMBLY__

struct pcie_desc {
	uint32_t ctrl_status;
	uint32_t local_addr;
	uint32_t remote_addr_low;
	uint32_t remote_addr_high;
};
#endif

#define PCIE_DESC_HOST_OWNED			(1 << 0)
#define PCIE_DESC_PKT_TRAFFIC_CLASS(v)		(((v) & 0x7) << 1)
#define PCIE_DESC_WRITE				(1 << 4)
#define PCIE_DESC_64BIT				(1 << 5)
#define PCIE_DESC_PKT_ATTRIBUTES(v)		(((v) & 0x7) << 10)
#define PCIE_DESC_ERROR				(1 << 12)
#define PCIE_DESC_STATUS_GET(v)			(((v) >> 13) & 0x7)
#define PCIE_DESC_LENGTH(v)			(((v) & 0xFFF) << 20)

#endif	/* _ARM_MINDSPEED_M83XXX_PCIE_H_ */
