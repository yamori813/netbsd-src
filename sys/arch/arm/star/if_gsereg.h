/*	$NetBSD$	*/

/*-
 * Copyright (c) 2009 SHIMIZU Ryo <ryo@nerv.org>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _IF_GSEREG_H_
#define _IF_GSEREG_H_

#include <sys/endian.h>

#define GSE_PHY_CTRL_REG		0x0000	/* PHY Control Register 0 */
# define GSE_PHY_CTRL_RW_OK			0x00008000
# define GSE_PHY_CTRL_RD_CMD			0x00004000
# define GSE_PHY_CTRL_WT_CMD			0x00002000
# define GSE_PHY_CTRL_PHY_REG(reg)		(((reg) & 0x1f) << 8)
# define GSE_PHY_CTRL_PHY_ADDR(addr)		((addr) & 3)
#define GSE_SW_CFG_REG			0x0004	/* Switch Configuration Register */
# define GSE_SW_CFG_NIC_MODE			0x40000000
# define GSE_SW_CFG_SKIP_L2_PORT1		0x20000000
# define GSE_SW_CFG_SKIP_L2_PORT0		0x10000000
# define GSE_SW_CFG_FIREWALL_MODE		0x01000000
# define GSE_SW_CFG_HNAT_EN			0x00800000
# define GSE_SW_CFG_IVL				0x00400000
# define GSE_SW_CFG_CRC_STRIP			0x00200000
# define GSE_SW_CFG_COL_MODE(n)			(((n) & 3) << 19)
# define GSE_SW_CFG_COL_MODE_MASK		GSE_SW_CFG_COL_MODE(3)
# define GSE_SW_CFG_REV_MC_FILTER		0x00040000
# define GSE_SW_CFG_BP_MODE(n)			(((n) & 3) << 16)
# define GSE_SW_CFG_BP_JAM_NO(n)		(((n) & 15) << 12)
# define GSE_SW_CFG_BKOFF_MODE(n)		(((n) & 7) << 8)
# define GSE_SW_CFG_HASH_ALG(n)			(((n) & 3) << 6)
# define GSE_SW_CFG_MAX_LEN(n)			(((n) & 3) << 4)
# define GSE_SW_CFG_AGE_TIME(n)			((n) & 15)
#define GSE_MAC_PORT0_CFG_REG		0x0008	/* MAC Port0 Configuration */
#define GSE_MAC_PORT1_CFG_REG		0x000c	/* MAC Port1 Configuration */
# define GSE_MAC_PORT_CFG_BCS_BC_PKT_EN		0x80000000
# define GSE_MAC_PORT_CFG_BCS_MC_PKT_EN		0x40000000
# define GSE_MAC_PORT_CFG_BCS_UN_PKT_EN		0x20000000
# define GSE_MAC_PORT_CFG_DIS_UC_PAUSE		0x10000000
# define GSE_MAC_PORT_CFG_BC_PKT_DIS		0x08000000
# define GSE_MAC_PORT_CFG_MC_PKT_DIS		0x04000000
# define GSE_MAC_PORT_CFG_UN_PKT_DIS		0x02000000
# define GSE_MAC_PORT_CFG_INGRESS_CHECK		0x01000000
# define GSE_MAC_PORT_CFG_SA_SECURED		0x00800000
# define GSE_MAC_PORT_CFG_AGE_EN		0x00400000
# define GSE_MAC_PORT_CFG_BLOCK_MODE		0x00200000
# define GSE_MAC_PORT_CFG_BLOCKING_STATE	0x00100000
# define GSE_MAC_PORT_CFG_LEARN_DIS		0x00080000
# define GSE_MAC_PORT_CFG_PORT_DIS		0x00040000
# define GSE_MAC_PORT_CFG_BP_EN			0x00020000
# define GSE_MAC_PORT_CFG_RGMII_PHY		0x00008000
# define GSE_MAC_PORT_CFG_REV_MII_RGMII		0x00004000
# define GSE_MAC_PORT_CFG_TXC_CHECK_EN		0x00002000
# define GSE_MAC_PORT_CFG_FORCE_FC_TX		0x00001000
# define GSE_MAC_PORT_CFG_FORCE_FC_RX		0x00000800
# define GSE_MAC_PORT_CFG_FORCE_DUPLEX		0x00000400
# define GSE_MAC_PORT_CFG_FORCE_SPEED(n)	(((n) & 3) << 8)
# define GSE_MAC_PORT_CFG_FORCE_SPEED_MASK(n)	GSE_MAC_PORT_CFG_FORCE_SPEED(3)
# define GSE_MAC_PORT_CFG_AN_EN			0x00000080
# define GSE_MAC_PORT_CFG_FC_TX_ST		0x00000040
# define GSE_MAC_PORT_CFG_FC_RX_ST		0x00000020
# define GSE_MAC_PORT_CFG_DUPLEX_ST		0x00000010
# define GSE_MAC_PORT_CFG_SPEED_ST_MASK		0x0000000c
# define GSE_MAC_PORT_CFG_SPEED_ST_10M		0x00000000
# define GSE_MAC_PORT_CFG_SPEED_ST_100M		0x00000004
# define GSE_MAC_PORT_CFG_SPEED_ST_1000N	0x00000008
# define GSE_MAC_PORT_CFG_TXC_ST		0x00000002
# define GSE_MAC_PORT_CFG_LINK_ST		0x00000001
#define GSE_CPU_PORT_CFG_REG		0x0010	/* CPU Port Configuration */
# define GSE_CPU_PORT_CFG_OFFSET_2B		0x80000000
# define GSE_CPU_PORT_CFG_INGRESS_CHECK		0x01000000
# define GSE_CPU_PORT_CFG_SA_SECURED		0x00800000
# define GSE_CPU_PORT_CFG_AGE_EN		0x00400000
# define GSE_CPU_PORT_CFG_LEARN_DIS		0x00080000
# define GSE_CPU_PORT_CFG_PORT_DIS		0x00040000
#define GSE_PRI_CTRL_REG		0x0014	/* Priority Control */
#define GSE_UDP_PRI_REG			0x0018	/* UDP Priority Register */
#define GSE_IP_TOS0_PRI_REG		0x001c	/* IP TOS 0-7 Priority */
#define GSE_IP_TOS1_PRI_REG		0x0020	/* IP TOS 8-15 Priority */
#define GSE_IP_TOS2_PRI_REG		0x0024	/* IP TOS 16-23 Priority */
#define GSE_IP_TOS3_PRI_REG		0x0028	/* IP TOS 24-31 Priority */
#define GSE_IP_TOS4_PRI_REG		0x002c	/* IP TOS 32-39 Priority */
#define GSE_IP_TOS5_PRI_REG		0x0030	/* IP TOS 40-47 Priority */
#define GSE_IP_TOS6_PRI_REG		0x0034	/* IP TOS 48-55 Priority */
#define GSE_IP_TOS7_PRI_REG		0x0038	/* IP TOS 56-63 Priority */
#define GSE_SCHED_CTRL_REG		0x003c	/* Scheduling Control */
#define GSE_RATE_LIMIT_CTRL_REG		0x0040	/* Rate Limit Control */
#define GSE_FLOW_CTRL_GLOBAL_THR_REG	0x0044	/* Flow Control Global Threshold */
#define GSE_FLOW_CTRL_PORT_THR_REG	0x0048	/* Flow Control Port Threshold */
#define GSE_SMART_FLOW_CTRL_REG		0x004c	/* Smart Flow Control */
#define GSE_ARL_TABLE_CTRL0_REG		0x0050	/* ARL Table Access Control0 */
# define GSE_ARL_TABLE_CTRL0_WT_CMD		0x00000008
# define GSE_ARL_TABLE_CTRL0_LKUP_CMD		0x00000004
# define GSE_ARL_TABLE_CTRL0_SRCH_AGAIN_CMD	0x00000002
# define GSE_ARL_TABLE_CTRL0_SRCH_START_CMD	0x00000001
#define GSE_ARL_TABLE_CTRL1_REG		0x0054	/* ARL Table Access Control1 */
# define GSE_ARL_TABLE_CTRL1_MAC1(a1)		(((a1) & 0xff) << 24)
# define GSE_ARL_TABLE_CTRL1_MAC0(a0)		(((a0) & 0xff) << 16)
# define GSE_ARL_TABLE_CTRL1_PORTMAP_MASK	0x00003800
# define GSE_ARL_TABLE_CTRL1_PORTMAP_SHIFT	11
# define GSE_ARL_TABLE_CTRL1_PORTMAP(p)		(((p) & 7) << GSE_ARL_TABLE_CTRL1_PORTMAP_SHIFT)
# define GSE_ARL_TABLE_CTRL1_AGE(n)		(((n) & 7) << 8)
# define GSE_ARL_TABLE_CTRL1_AGE_MASK		GSE_ARL_TABLE_CTRL1_AGE(7)
# define GSE_ARL_TABLE_CTRL1_AGE_SHIFT		7
# define GSE_ARL_TABLE_CTRL1_VLAN_ID(id)	(((id) & 7) << 5)
# define GSE_ARL_TABLE_CTRL1_VLAN_ID_MASK	GSE_ARL_TABLE_CTRL1_VLAN_ID(7)
# define GSE_ARL_TABLE_CTRL1_VLAN_ID_SHIFT	5
# define GSE_ARL_TABLE_CTRL1_VLAN_MAC		0x00000010
# define GSE_ARL_TABLE_CTRL1_FILTER		0x00000008
# define GSE_ARL_TABLE_CTRL1_LKUP_SEARHC_MATCH	0x00000004
# define GSE_ARL_TABLE_CTRL1_TABLE_END		0x00000002
# define GSE_ARL_TABLE_CTRL1_CMD_COMPLETE	0x00000001
#define GSE_ARL_TABLE_CTRL2_REG		0x0058	/* ARL Table Access Control2 */
# define GSE_ARL_TABLE_CTRL2_MAC5(a5)		(((a5) & 0xff) << 24)
# define GSE_ARL_TABLE_CTRL2_MAC4(a4)		(((a4) & 0xff) << 16)
# define GSE_ARL_TABLE_CTRL2_MAC3(a3)		(((a3) & 0xff) << 8)
# define GSE_ARL_TABLE_CTRL2_MAC2(a2)		(((a2) & 0xff))
#define GSE_PORT_VID_REG		0x005c	/* Port VID */
#define GSE_VLAN_GID01			0x0060	/* VLAN Group ID 0-1 */
#define GSE_VLAN_GID23			0x0064	/* VLAN Group ID 2-3 */
#define GSE_VLAN_GID45			0x0068	/* VLAN Group ID 4-5 */
#define GSE_VLAN_GID67			0x006c	/* VLAN Group ID 6-7 */
#define GSE_VLAN_PORT_MAP_REG		0x0070	/* VLAN Port Map */
#define GSE_VLAN_TAG_PORT_TAG_REG	0x0074	/* VLAN Tag Port Map */
#define GSE_SESSION_ID01_REG		0x0078	/* Session ID 0-1 */
#define GSE_SESSION_ID23_REG		0x007c	/* Session ID 2-3 */
#define GSE_SESSION_ID45_REG		0x0080	/* Session ID 4-5 */
#define GSE_SESSION_ID67_REG		0x0084	/* Session ID 6-7 */
#define GSE_INT_STATUS_REG		0x0088	/* Interrupt Status */
# define GEC_INT_PORT1_INGRESS			0x80000000
# define GEC_INT_PORT1_LOCAL			0x40000000
# define GEC_INT_PORT1_RMC_PAUSE		0x20000000
# define GEC_INT_PORT1_NO_DEST			0x10000000
# define GEC_INT_PORT1_JAMMED			0x08000000
# define GEC_INT_PORT1_RX_ERR			0x04000000
# define GEC_INT_PORT1_BCSTORM			0x02000000
# define GEC_INT_PORT1_NOFREELINK		0x01000000
# define GEC_INT_PORT0_INGRESS			0x00800000
# define GEC_INT_PORT0_LOCAL			0x00400000
# define GEC_INT_PORT0_RMC_PAUSE		0x00200000
# define GEC_INT_PORT0_NO_DEST			0x00100000
# define GEC_INT_PORT0_JAMMED			0x00080000
# define GEC_INT_PORT0_RX_ERR			0x00040000
# define GEC_INT_PORT0_BCSTORM			0x00020000
# define GEC_INT_PORT0_NOFREELINK		0x00010000
# define GEC_INT_CPU_UNKNOWN_VLAN		0x00001000
# define GEC_INT_PORT1_UNKNOWN_VLAN		0x00000800
# define GEC_INT_PORT0_UNKNOWN_VLAN		0x00000400
# define GEC_INT_INTRUDER1			0x00000100
# define GEC_INT_INTRUDER0			0x00000080
# define GEC_INT_PORT_STATUS_CHG		0x00000040
# define GEC_INT_BUFFER_FULL			0x00000020
# define GEC_INT_GLOBAL_Q_FULL			0x00000010
# define GEC_INT_HNAT_Q_FULL			0x00000008
# define GEC_INT_CPU_Q_FULL			0x00000004
# define GEC_INT_PORT1_Q_FULL			0x00000002
# define GEC_INT_PORT0_Q_FULL			0x00000001
#define GSE_INT_MASK_REG		0x008c	/* Interrupt Mask */
#define GSE_AUTOPOLL_PHYADDR_REG	0x0090	/* Auto-Polling PHY Address */
#define GSE_TEST0_REG			0x0094	/* Clock Skew Setting */
# define GSE_TEST0_PORT1_TX_SKEW(n)		(((n) & 3) << 30)
# define GSE_TEST0_PORT1_TX_SKEW_MASK		GSE_TEST0_PORT1_TX_SKEW(3)
# define GSE_TEST0_PORT1_RX_SKEW(n)		(((n) & 3) << 28)
# define GSE_TEST0_PORT1_RX_SKEW_MASK		GSE_TEST0_PORT1_RX_SKEW(3)
# define GSE_TEST0_PORT0_TX_SKEW(n)		(((n) & 3) << 26)
# define GSE_TEST0_PORT0_TX_SKEW_MASK		GSE_TEST0_PORT0_TX_SKEW(3)
# define GSE_TEST0_PORT0_RX_SKEW(n)		(((n) & 3) << 24)
# define GSE_TEST0_PORT0_RX_SKEW_MASK		GSE_TEST0_PORT0_RX_SKEW(3)


#define GSE_TEST1_REG			0x0098	/* Queue Status and PHY Address */
# define GSE_TEST1_PHY_ADDR(addr)		((((addr) >> 1) & 0xf) << 16)
# define GSE_TEST1_PHY_ADDR_MASK		GSE_TEST1_PHY_ADDR(0x0f)
#define GSE_TX_DMA_CTRL_REG		0x0100	/* To-Switch TS_DMS Control */
# define GSE_TX_DMA_CTRL_TX_EN			0x00000001
#define GSE_RX_DMA_CTRL_REG		0x0104	/* From-Switch FS_DMA Control */
# define GSE_RX_DMA_CTRL_RX_EN			0x00000001
#define GSE_TX_DPTR_REG			0x0108	/* TS_Descrptor Starting Address */
#define GSE_RX_DPTR_REG			0x010c	/* FS_Descrptor Starting Address */
#define GSE_TX_BASE_ADDR_REG		0x0110	/* TS Descriptor Base Address */
#define GSE_RX_BASE_ADDR_REG		0x0114	/* FS Descriptor Base Address */
#define GSE_DLY_INT_CFG_REG		0x0118	/* Delayed Interrupt Configuration */
# define GSE_DLY_INT_CFG_DELAY_INT_EN		0x00010000
# define GSE_DLY_INT_CFG_MAX_INT_CNT(n) 	(((n) & 0xff) << 8)
# define GSE_DLY_INT_CFG_MAX_PEND_TIME(n)	((n) & 0xff)

#define GSE_REG_SIZE			0x29c




struct gse_txdesc {
	uint32_t tx_sdp;
	uint32_t tx_ctrl;
#define GSE_TXDESC_CTRL_COWN		0x80000000
#define GSE_TXDESC_CTRL_EOR		0x40000000
#define GSE_TXDESC_CTRL_FS		0x20000000
#define GSE_TXDESC_CTRL_LS		0x10000000
#define GSE_TXDESC_CTRL_INT		0x08000000
#define GSE_TXDESC_CTRL_FP		0x04000000
#define GSE_TXDESC_CTRL_PRI(n)		(((n) & 7) << 23)
#define GSE_TXDESC_CTRL_FR		0x00400000
#define GSE_TXDESC_CTRL_PMAP_CPU	0x00200000
#define GSE_TXDESC_CTRL_PMAP_PORT1	0x00100000
#define GSE_TXDESC_CTRL_PMAP_PORT0	0x00080000
#define GSE_TXDESC_CTRL_ICO		0x00040000
#define GSE_TXDESC_CTRL_UCO		0x00020000
#define GSE_TXDESC_CTRL_TCO		0x00010000
#define GSE_TXDESC_CTRL_SDL(len)	((len) & 0xffff)
	uint32_t tx_vlan;
#define GSE_TXDESC_VLAN_INSS		0x00000080
#define GSE_TXDESC_VLAN_SIDP(pri)	(((pri) & 7) << 4)
#define GSE_TXDESC_VLAN_INSV		0x00000008
#define GSE_TXDESC_VLAN_VID(vid)	((vid) & 7)
	uint32_t tx_rsvd;
} __packed;

struct gse_rxdesc {
	uint32_t rx_sdp;
	uint32_t rx_ctrl;
#define GSE_RXDESC_CTRL_COWN		0x80000000
#define GSE_RXDESC_CTRL_EOR		0x40000000
#define GSE_RXDESC_CTRL_FS		0x20000000
#define GSE_RXDESC_CTRL_LS		0x10000000
#define GSE_RXDESC_CTRL_SP_MASK		0x0c000000
#define GSE_RXDESC_CTRL_SP_PORT0	0x00000000
#define GSE_RXDESC_CTRL_SP_PORT1	0x04000000
#define GSE_RXDESC_CTRL_SP_CPU		0x08000000
#define GSE_RXDESC_CTRL_HR_MASK		0x03f00000
#define GSE_RXDESC_CTRL_HR(n)		(((n) & 63) << 20)
#define GSE_RXDESC_CTRL_PROT_MASK	0x000c0000
#define GSE_RXDESC_CTRL_PROT_IP 	0x00000000
#define GSE_RXDESC_CTRL_PROT_UDP	0x00040000
#define GSE_RXDESC_CTRL_PROT_TCP	0x00080000
#define GSE_RXDESC_CTRL_PROT_OTHERS	0x000c0000
#define GSE_RXDESC_CTRL_IPF		0x00020000
#define GSE_RXDESC_CTRL_L4F		0x00010000
#define GSE_RXDESC_CTRL_SDL(len)	((len) & 0xffff)
#define GSE_RXDESC_CTRL_SDL_MASK	0x0000ffff
	uint32_t rx_rsvd1;
	uint32_t rx_rsvd2;
} __packed;

struct gse_arl {
	int garl_filter;	/* packet will be dropped when set */
	int garl_mymac;		/* my mac address */
	int garl_vlangid;	/* VLAN GID0-7 */
	int garl_age;		/* aging of the entry */
#define GARL_AGE_INVALID	0
#define GARL_AGE_NEW		1
#define GARL_AGE_OLD		6
#define GARL_AGE_STATIC		7
	int garl_port;
#define GARL_PORT_0	0x01
#define GARL_PORT_1	0x02
#define GARL_PORT_CPU	0x04
	uint8_t garl_mac[6];
};

#endif /* _IF_GSEREG_H_ */
