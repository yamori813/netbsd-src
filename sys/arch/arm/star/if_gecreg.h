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

#ifndef _IF_GECREG_H_
#define _IF_GECREG_H_

#include <sys/endian.h>

#define GEC_PHY_CTRL0_REG	0x0000	/* PHY Control Register 0 */
# define GEC_PHY_CTRL0_RW_OK		0x00008000
# define GEC_PHY_CTRL0_RD_CMD		0x00004000
# define GEC_PHY_CTRL0_WT_CMD		0x00002000
# define GEC_PHY_CTRL0_PHY_REG(reg)	(((reg) & 0x1f) << 8)
# define GEC_PHY_CTRL0_PHY_ADDR(addr)	((addr) & 0x1f)
#define GEC_PHY_CTRL1_REG	0x0004	/* PHY Control Register 1 */
# define GEC_PHY_CTRL1_AUTO_POLL_DIS	0x80000000
# define GEC_PHY_CTRL1_PHY_ADDR_AUTO(a)	((a) << 24)
# define GEC_PHY_CTRL1_INTERNAL_PHY_SEL	0x00040000
# define GEC_PHY_CTRL1_RGMII_PHY	0x00020000
# define GEC_PHY_CTRL1_REV_MII_RGMII	0x00010000
# define GEC_PHY_CTRL1_TXC_CHECK_EN	0x00004000
# define GEC_PHY_CTRL1_FORCE_FC_TX	0x00002000
# define GEC_PHY_CTRL1_FORCE_FC_RX	0x00001000
# define GEC_PHY_CTRL1_FORCE_DUPLEX	0x00000800
# define GEC_PHY_CTRL1_FORCE_SPEED(n)	(((n) & 3) << 9)
# define GEC_PHY_CTRL1_FORCE_SPEED_MASK	GEC_PHY_CTRL1_FORCE_SPEED(3)
# define GEC_PHY_CTRL1_AN_EN		0x00000100
# define GEC_PHY_CTRL1_MI_DIS		0x00000080
# define GEC_PHY_CTRL1_FC_TX_ST		0x00000040
# define GEC_PHY_CTRL1_FC_RX_ST		0x00000020
# define GEC_PHY_CTRL1_DUPLEX_ST	0x00000010
# define GEC_PHY_CTRL1_SPEED_ST		0x0000000c
# define GEC_PHY_CTRL1_TXC_ST		0x00000002
# define GEC_PHY_CTRL1_LINK_ST		0x00000001
#define GEC_MAC_CFG_REG		0x0008	/* MAC Configuration */
# define GEC_MAC_CFG_NIC_PD		0x80000000
# define GEC_MAC_CFG_WoL		0x40000000
# define GEC_MAC_CFG_NIC_PD_READY	0x20000000
# define GEC_MAC_CFG_TX_CKS_EN		0x04000000
# define GEC_MAC_CFG_RX_CKS_EN		0x02000000
# define GEC_MAC_CFG_ACPT_CKS_ERR	0x01000000
# define GEC_MAC_CFG_IST_EN		0x00800000
# define GEC_MAC_CFG_VLAN_STRIP		0x00400000
# define GEC_MAC_CFG_ACPT_CRC_ERR	0x00200000
# define GEC_MAC_CFG_CRC_STRIP		0x00100000
# define GEC_MAC_CFG_ACPT_LONG_PKT	0x00040000
# define GEC_MAC_CFG_MAX_LEN(n)		(((n) & 3) << 16)
# define GEC_MAC_CFG_IPG(n)		(((n) & 0x1f) << 10)
# define GEC_MAC_CFG_DO_NO_SKIP		0x00000200
# define GEC_MAC_CFG_FAST_RETRY		0x00000100
#define GEC_FC_CFG_REG		0x000C	/* Flow Control Configuration */
# define GEC_FC_CFG_SEND_PAUSE_TH(n)	(((n) & 0xfff) << 16)
# define GEC_FC_CFG_UC_PAUSE_DIS	0x00000100
# define GEC_FC_CFG_BP_ENABLE		0x00000080
# define GEC_FC_CFG_MAX_BP_COL_EN	0x00000020
# define GEC_FC_CFG_MAX_BP_COL_CNT(n)	((n) & 0x1f)
#define GEC_ARL_CFG_REG		0x0010	/* ARL Configuration */
# define GEC_ARL_CFG_MISC_MODE		0x00000010
# define GEC_ARL_CFG_MY_MAC_ONLY	0x00000008
# define GEC_ARL_CFG_CPU_LEARN_DIS	0x00000004
# define GEC_ARL_CFG_REV_MC_FILTER	0x00000002
# define GEC_ARL_CFG_HASH_ALG		0x00000001
#define GEC_MY_MAC_H_REG	0x0014	/* My MAC High Byte */
#define GEC_MY_MAC_L_REG	0x0018	/* My MAC Low Byte */
#define GEC_HASH_CTRL_REG	0x001C	/* Hash Table Control */
# define GEC_HASH_CTRL_BIST_DONE	0x00020000
# define GEC_HASH_CTRL_BIST_OK		0x00010000
# define GEC_HASH_CTRL_CMD_START	0x00004000
# define GEC_HASH_CTRL_CMD_READ		0x00000000
# define GEC_HASH_CTRL_CMD_WRITE	0x00002000
# define GEC_HASH_CTRL_HASHBIT(n)	(((n) & 1) << 12)
# define GEC_HASH_CTRL_HASHBIT_MASK	GEC_HASH_CTRL_HASHBIT(1)
# define GEC_HASH_CTRL_HASHADDR(n)	((n) & 0x1ff)
# define GEC_HASH_CTRL_HASHBITSIZE	512
#define GEC_VLAN_CTRL_REG	0x0020	/* My VLAN ID Control */
#define GEC_VLAN_ID_0_1_REG	0x0024	/* My VLAN ID 0 ? 1 */
#define GEC_VLAN_ID_2_3_REG	0x0028	/* My VLAN ID 2 ? 3 */
#define GEC_DMA_CFG_REG		0x0030	/* DMA Configuration */
# define GEC_DMA_CFG_RX_OFFSET_2B_DIS	0x00010000
# define GEC_DMA_CFG_TX_POLL_PERIOD(n)	((n) << 6)
# define GEC_DMA_CFG_TX_POLL_EN		0x00000020
# define GEC_DMA_CFG_TX_SUSPEND		0x00000010
# define GEC_DMA_CFG_RX_POLL_PERIOD(n)	((n) << 2)
# define GEC_DMA_CFG_RX_POLL_EN		0x00000002
# define GEC_DMA_CFG_RX_SUSPEND		0x00000001
#define GEC_TX_DMA_CTRL_REG	0x0034	/* TX_DMA Control */
# define GEC_TX_DMA_CTRL_TX_EN		0x00000001
#define GEC_RX_DMA_CTRL_REG	0x0038	/* RX_DMA Control */
# define GEC_RX_DMA_CTRL_RX_EN		0x00000001
#define GEC_TX_DPTR_REG		0x003C	/* TX Descriptor Pointer */
#define GEC_RX_DPTR_REG		0x0040	/* RX Descriptor Pointer */
#define GEC_TX_BASE_ADDR_REG	0x0044	/* TX Descriptor Base Address */
#define GEC_RX_BASE_ADDR_REG	0x0048	/* RX Descriptor Base Address */
#define GEC_DLY_INT_CFG_REG	0x004C	/* Delayed Interrupt Configuration */
# define GEC_DLY_INT_CFG_MAX_PEND_TIME(n)	((n) & 0xff)
# define GEC_DLY_INT_CFG_MAX_INT_CNT(n)		(((n) & 0xff) << 8)
# define GEC_DLY_INT_CFG_DELAY_INT_EN		0x00010000
#define GEC_INT_STATUS_REG	0x0050	/* Interrupt Status */
#define GEC_INT_MASK_REG	0x0054	/* Interrupt Mask */
# define GEC_INT_MAGIC_PKT_REC		0x00000010
# define GEC_INT_MIB_COUNT_TH		0x00000008
# define GEC_INT_PORT_STATUS_CHG	0x00000004
# define GEC_INT_RX_FIFO_FULL		0x00000002
# define GEC_INT_TX_FIFO_UNDERRUN	0x00000001
#define GEC_TEST0_REG		0x0058	/* Test 0 (Clock Skew Setting) */
#define GEC_TEST1_REG		0x005C	/* Test 1 (Queue Status) */
#define GEC_EXTEND_CFG_REG	0x0060	/* Extended Configuration Register */
#define GEC_C_RXOKPKT_REG	0x0100	/* RX OK Packet Counter */
#define GEC_C_RXOKBYTE_REG	0x0104	/* RX OK Byte Counter */
#define GEC_C_RXRUNT_REG	0x0108	/* RX Runt Packet Counter */
#define GEC_C_RXLONG_REG	0x010C	/* RX Over Size Packet Counter */
#define GEC_C_RXDROP_REG	0x0110	/* RX No Buffer Drop Packet Counter */
#define GEC_C_RXCRC_REG		0x0114	/* RX CRC Error Packet Counter */
#define GEC_C_RXARLDROP_REG	0x0118	/* RX ARL Drop Packet Counter */
#define GEC_C_RXVLANDROP_REG	0x011C	/* My VLAN ID Mismatch Drop Counter */
#define GEC_C_RXCSERR_REG	0x0120	/* RX Check Sum Error Packet Counter */
#define GEC_C_RXPAUSE_REG	0x0124	/* RX Pause Frame Packet Counter */
#define GEC_C_TXOKPKT_REG	0x0128	/* TX OK Packet Counter */
#define GEC_C_TXOKBYTE_REG	0x012C	/* TX OK Byte Counter */
#define GEC_C_TXPAUSECOL_REG	0x0130	/* TX Collision Counter/Pause Frame Counter */
#define GEC_REG_SIZE		0x0134


struct gec_txdesc {
	uint32_t tx_sdp;
	uint32_t tx_ctrl;
#define GEC_TXDESC_CTRL_COWN		0x80000000
#define GEC_TXDESC_CTRL_EOR		0x40000000
#define GEC_TXDESC_CTRL_FS		0x20000000
#define GEC_TXDESC_CTRL_LS		0x10000000
#define GEC_TXDESC_CTRL_INT		0x08000000
#define GEC_TXDESC_CTRL_INSV		0x04000000
#define GEC_TXDESC_CTRL_ICO		0x02000000
#define GEC_TXDESC_CTRL_UCO		0x01000000
#define GEC_TXDESC_CTRL_TCO		0x00800000
#define GEC_TXDESC_CTRL_SDL(len)	((len) & 0xffff)
	uint32_t tx_vlan;
#define GEC_TXDESC_VLAN_EPID(epid)	(((epid) & 0xffff) << 16)
#define GEC_TXDESC_VLAN_PRI(pri)	(((pri) & 7) << 13)
#define GEC_TXDESC_VLAN_CFI		0x00001000
#define GEC_TXDESC_VLAN_VID(vid)	((vid) & 0x0fff)
	uint32_t tx_rsvd;
} __packed;

struct gec_rxdesc {
	uint32_t rx_sdp;
	uint32_t rx_ctrl;
#define GEC_RXDESC_CTRL_COWN		0x80000000
#define GEC_RXDESC_CTRL_EOR		0x40000000
#define GEC_RXDESC_CTRL_FS		0x20000000
#define GEC_RXDESC_CTRL_LS		0x10000000
#define GEC_RXDESC_CTRL_OSIZE		0x02000000
#define GEC_RXDESC_CTRL_CRCE		0x01000000
#define GEC_RXDESC_CTRL_RMC		0x00800000
#define GEC_RXDESC_CTRL_HHIT		0x00400000
#define GEC_RXDESC_CTRL_MYMAC		0x00200000
#define GEC_RXDESC_CTRL_VTEC		0x00100000
#define GEC_RXDESC_CTRL_PROT_MASK	0x000c0000
#define GEC_RXDESC_CTRL_PROT_IP		0x00000000
#define GEC_RXDESC_CTRL_PROT_UDP	0x00040000
#define GEC_RXDESC_CTRL_PROT_TCP	0x00080000
#define GEC_RXDESC_CTRL_PROT_OTHERS	0x000c0000
#define GEC_RXDESC_CTRL_IPF		0x00020000
#define GEC_RXDESC_CTRL_L4F		0x00010000
#define GEC_RXDESC_CTRL_SDL(len)	((len) & 0xffff)
#define GEC_RXDESC_CTRL_SDL_MASK	0x0000ffff
	uint32_t rx_vlan;
#define GEC_RXDESC_VLAN_EPID(epid)	(((epid) & 0xffff) << 16)
#define GEC_RXDESC_VLAN_PRI(pri)	(((pri) & 7) << 13)
#define GEC_RXDESC_VLAN_CFI		0x00001000
#define GEC_RXDESC_VLAN_VID(vid)	((vid) & 0x0fff)
	uint32_t rx_rsvd;
} __packed;

#endif /* _IF_GECREG_H_ */
