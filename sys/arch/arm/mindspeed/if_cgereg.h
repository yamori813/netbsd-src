/*	$NetBSD$	*/

/*
 * Copyright (c) 2022 Hiroki Mori
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
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __IF_CGEREG_H__
#define	__IF_CGEREG_H__

struct tRXdesc {
	uint32_t	rx_data;
	uint32_t	rx_status;
	uint32_t	rx_extstatus;
	uint32_t	pad;
};

//gemac rx controls
// Wrap flag - marks last descriptor in a queue when set
// goes to the status word (offset 0x4)
#define GEMRX_WRAP              (1<<28)
// Ownership flag - when 0 gem can use the descriptor
// goes to the extended status word (offset 0x8)
#define GEMRX_OWN               (1<<15)

// gemac rx status

#define RX_STA_BCAST            (1UL<<31)
#define RX_STA_MCAST            (1<<30)
#define RX_STA_UM               (1<<29)
#define RX_MAC_MATCH_FLAG       (0x4<<25)
#define RX_MAC_MATCH_NUM_MASK   (0x3<<25)
#define RX_MAC_MATCH_POS        25
#define RX_INT                  (1<<24)
#define RX_IPSEC_OUT            (1<<23)
#define RX_IPSEC_IN             (1<<22)
#define RX_STA_VLAN             (1<<21)
#define RX_STA_VLAN_802p        (1<<20)
#define RX_STA_VLAN_PRI_MASK    (7<<17)
#define RX_STA_VLAN_PRI_POS     17
#define RX_STA_VLAN_CFI         (1<<16)
#define RX_STA_SOF              (1<<15)
#define RX_STA_EOF              (1<<14)
#define RX_STA_PACKET           (RX_STA_SOF|RX_STA_EOF)
#define RX_STA_CRCERR           (1<<13)
#define RX_STA_LEN_MASK         0xfff
#define RX_STA_LEN_POS          0
#define RX_CHECK_ERROR          RX_STA_CRCERR

// gemac rx extended status(word2)

#define RX_STA_L4OFF_MASK       (0xff<<24)
#define RX_STA_L4OFF_POS        24
#define RX_STA_L3OFF_MASK       (0xff<<16)
#define RX_STA_L3OFF_POS        16

#define RX_STA_L3_CKSUM         (1<<11)
#define RX_STA_L3_GOOD          (1<<12)
#define RX_STA_L4_CKSUM         (1<<13)
#define RX_STA_L4_GOOD          (1<<14)

#define RX_STA_TCP              (1<<9)
#define RX_STA_UDP              (1<<8)
#define RX_STA_IPV6             (1<<7)
#define RX_STA_IPV4             (1<<6)
#define RX_STA_PPPOE            (1<<5)
#define RX_STA_WILLHANDLE       ( RX_STA_IPV6 | RX_STA_IPV4)
#define RX_STA_QinQ             (1<<4)
#define RX_STA_TYPEID_MATCH_FLAG (0x8 << 0)
#define RX_STA_TYPEID           (0x7 << 0)
#define RX_STA_TYPEID_POS       0

struct tTXdesc {
	uint32_t	tx_data;
	union {
		uint32_t	tx_ctl;
		uint32_t	tx_status;
	};
};

#define GEMTX_USED_MASK         (1UL<<31)
#define GEMTX_WRAP              (1UL<<30)
#define GEMTX_IE                (1UL<<29)
#define GEMTX_L4_CSUM           (1UL<<26)
#define GEMTX_L3_CSUM           (1UL<<25)
#define GEMTX_FCS               (1UL<<24)
#define GEMTX_OFFSET_MASK       0xff
#define GEMTX_OFFSET_SHIFT      16
#define GEMTX_LAST              (1UL<<15)
#define GEMTX_POOLB             (1UL<<14)
#define GEMTX_BUFRET            (1UL<<13)
#define GEMTX_LENGTH_MASK       0x1fff
#define GEMTX_LENGTH_SHIFT      0
#define GEMTX_LENGTH_MAX        0x1fff

#define	CGE_DMASIZE(len)	((len)  & ((1 << 11)-1))		
#define	CGE_PKTSIZE(len)	((len & 0xffff0000) >> 16)

#define	CGE_RX_RING_CNT		256
#define	CGE_TX_RING_CNT		256
#define	CGE_TX_RING_SIZE	sizeof(struct tTXdesc) * CGE_TX_RING_CNT
#define	CGE_RX_RING_SIZE	sizeof(struct tRXdesc) * CGE_RX_RING_CNT
#define	CGE_RING_ALIGN		sizeof(struct tRXdesc)
#define	CGE_RX_ALIGN		sizeof(uint32_t)
#define	CGE_MAXFRAGS		8
#define	CGE_TX_INTR_THRESH	8

#define	CGE_MIN_FRAMELEN	(ETHER_MIN_LEN - ETHER_CRC_LEN)

#define	CGE_TX_RING_ADDR(sc, i)	\
    ((sc)->cge_rdata.cge_tx_ring_paddr + sizeof(struct tTXdesc) * (i))
#define	CGE_RX_RING_ADDR(sc, i)	\
    ((sc)->cge_rdata.cge_rx_ring_paddr + sizeof(struct tRXdesc) * (i))
#define	CGE_INC(x,y)		(x) = (((x) + 1) % y)

struct cge_ring_data {
	bus_dmamap_t		tx_dm[CGE_TX_RING_CNT];
	struct mbuf		*tx_mb[CGE_TX_RING_CNT];
	bus_dmamap_t		rx_dm[CGE_RX_RING_CNT];
	struct mbuf		*rx_mb[CGE_RX_RING_CNT];
};

struct cge_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	bus_size_t		sc_bss;
	bus_dma_tag_t		sc_bdt;
	bus_space_handle_t	sc_bsh_txdescs;
	bus_space_handle_t	sc_bsh_rxdescs;
	bus_addr_t		sc_txdescs_pa;
	bus_addr_t		sc_rxdescs_pa;
	struct ethercom		sc_ec;
	void			*sc_txpad;
	bus_dmamap_t		sc_txpad_dm;
#define sc_txpad_pa sc_txpad_dm->dm_segs[0].ds_addr
	uint8_t			sc_enaddr[ETHER_ADDR_LEN];
	bool			sc_attached;
	struct cge_ring_data	*sc_rdp;
	struct mii_data		sc_mii;
	volatile u_int		sc_txnext;
	volatile u_int		sc_txhead;
	volatile u_int		sc_rxhead;

	struct tRXdesc		*sc_rxdesc_ring;
	bus_dmamap_t		sc_rxdesc_dmamap;

	struct tTXdesc		*sc_txdesc_ring;
	bus_dmamap_t		sc_txdesc_dmamap;
	bool			sc_txbusy;

	kmutex_t		mtx;
};

#define CGE_LOCK(sc)		mutex_enter(&(sc)->mtx)
#define CGE_UNLOCK(sc)		mutex_exit(&(sc)->mtx)

/*
 * register space access macros
 */
#define	CSR_WRITE_4(sc, reg, val)	\
	bus_space_write_4(sc->cge_btag, sc->cge_bhandle, reg, val)

#define	CSR_READ_4(sc, reg)		\
	bus_space_read_4(sc->cge_btag, sc->cge_bhandle, reg)


#define GEM_IP                                          0xE000

#define GEM_NET_CONTROL         (0x00)
#define GEM_NET_CONFIG          (0x04)
#define GEM_NET_STATUS          (0x08)
#define GEM_USER_IO             (0x0C)
#define GEM_DMA_CONFIG          (0x10)
#define GEM_TX_STATUS           (0x14)
#define GEM_RX_QPTR             (0x18)
#define GEM_RX_OFFSET           (0x1C)
#define GEM_RX_STATUS           (0x20)
#define GEM_IRQ_STATUS          (0x24)
#define GEM_IRQ_ENABLE          (0x28)
#define GEM_IRQ_DISABLE         (0x2C)
#define GEM_IRQ_MASK            (0x30)
#define GEM_PHY_MAN             (0x34)
#define GEM_RX_PAUSE_TIME       (0x38)
#define GEM_TX_PAUSE_QUANT      (0x3C)

#define GEM_HASH_BOT            (0x80)
#define GEM_HASH_TOP            (0x84)
#define GEM_LADDR1_BOT          (0x88)
#define GEM_LADDR1_TOP          (0x8C)
#define GEM_LADDR2_BOT          (0x90)
#define GEM_LADDR2_TOP          (0x94)
#define GEM_LADDR3_BOT          (0x98)
#define GEM_LADDR3_TOP          (0x9C)
#define GEM_LADDR4_BOT          (0xA0)
#define GEM_LADDR4_TOP          (0xA4)
#define GEM_ID_CHECK1           (0xA8)
#define GEM_ID_CHECK2           (0xAC)
#define GEM_ID_CHECK3           (0xB0)
#define GEM_ID_CHECK4           (0xB4)
#define GEM_REV_ID              (0xFC)

/* Bit positions for network control register */
#define GEM_READ_SNAP       (1<<14)     /* Read snapshot register */
#define GEM_TAKE_SNAP       (1<<13)     /* Take a snapshot */
#define GEM_TX_0Q_PAUSE     (1<<12)     /* Transmit zero quantum pause frame */
#define GEM_TX_PAUSE        (1<<11)     /* Transmit pause frame */
#define GEM_TX_HALT         (1<<10)     /* Halt transmission after curr frame */
#define GEM_TX_START        (1<<9)      /* Start tx (tx_go) */
#define GEM_STATS_WR_EN     (1<<7)      /* Enable writing to stat registers */
#define GEM_STATS_INC       (1<<6)      /* Increment statistic registers */
#define GEM_STATS_CLR       (1<<5)      /* Clear statistic registers */
#define GEM_MDIO_EN         (1<<4)      /* Enable MDIO port */
#define GEM_TX_EN           (1<<3)      /* Enable transmit circuits */
#define GEM_RX_EN           (1<<2)      /* Enable receive circuits */
#define GEM_LB_MAC          (1<<1)      /* Perform local loopback at MAC */
#define GEM_LB_PHY          (1<<0)      /* Perform ext loopback through PHY */

/* Bit positions for network configuration register */
#define GEM_RX_BAD_PREAMBLE (1<<29)     /* Receive frames with bad preamble */
#define GEM_CKSUM_OFFLOAD   (1<<24)     /* Enable Checksum Engine*/
#define GEM_RX_NO_PAUSE     (1<<23)     /* Do not copy pause frames to memory */
#define GEM_ENABLE_L4_DROP  (1<<22)     /* Drop packets if error in L4 check
sum */
#define GEM_ENABLE_L4_CKSUM (1<<21)     /* Enable L4 checksum check */
#define GEM_MDC_DIV_MASK    (0x7 << 18) /* PCLK divisor for MDC */
#define GEM_RX_NO_FCS       (1<<17)     /* Discard FCS from received frames. */
#define GEM_RX_LEN_CHK      (1<<16)     /* Receive length check. */
#define GEM_ENABLE_L3_DROP  (1<<15)     /* Drop packets if error in L3 checksum */
#define GEM_ENABLE_L3_CKSUM (1<<14)     /* Enable L3 checksum check */
#define GEM_RX_PAUSE_EN     (1<<13)     /* Enable pause reception */
#define GEM_RETRY_TEST      (1<<12)     /* Retry test for speeding up debug */
#define GEM_PCS_SEL         (1<<11)     /* Select PCS */
#define GEM_GIG_MODE        (1<<10)     /* Gigabit mode enable */
#define GEM_EAM_EN          (1<<9)      /* External address match enable */
#define GEM_FRAME_1536      (1<<8)      /* Enable 1536 byte frames reception */
#define GEM_UNICAST_EN      (1<<7)      /* Receive unicast hash frames */
#define GEM_MULTICAST_EN    (1<<6)      /* Receive multicast hash frames */
#define GEM_NO_BROADCAST    (1<<5)      /* Do not receive broadcast frames */
#define GEM_COPY_ALL        (1<<4)      /* Copy all frames */
#define GEM_RX_JUMBO        (1<<3)      /* Allow jumbo frame reception */
#define GEM_VLAN_ONLY       (1<<2)      /* Receive only VLAN frames */
#define GEM_FULL_DUPLEX     (1<<1)      /* Enable full duplex */
#define GEM_SPEED_100       (1<<0)      /* Set to 100Mb mode */

/* Bit positions for network status register */
#define GEM_PHY_IDLE        (1<<2)      /* PHY management is idle */
#define GEM_MDIO_IN         (1<<1)      /* Status of mdio_in pin */
#define GEM_LINK_STATUS     (1<<0)      /* Status of link pin */

/* Bit positions for interrupts */
#define GEM_IRQ_PCS_AN      (1<<16)     /* PCS autonegotiation complete */
#define GEM_IRQ_EXT_INT     (1<<15)     /* External interrupt pin triggered */
#define GEM_IRQ_PAUSE_TX    (1<<14)     /* Pause frame transmitted */
#define GEM_IRQ_PAUSE_0     (1<<13)     /* Pause time has reached zero */
#define GEM_IRQ_PAUSE_RX    (1<<12)     /* Pause frame received */
#define GEM_IRQ_HRESP       (1<<11)     /* hresp not ok */
#define GEM_IRQ_RX_ORUN     (1<<10)     /* Receive overrun occurred */
#define GEM_IRQ_PCS_LINK    (1<<9)      /* Status of PCS link changed */
#define GEM_IRQ_TX_DONE     (1<<7)      /* Frame transmitted ok */
#define GEM_IRQ_TX_ERROR    (1<<6)      /* Transmit err occurred or no buffers*/
#define GEM_IRQ_RETRY_EXC   (1<<5)      /* Retry limit exceeded */
#define GEM_IRQ_TX_URUN     (1<<4)      /* Transmit underrun occurred */
#define GEM_IRQ_TX_USED     (1<<3)      /* Tx buffer used bit read */
#define GEM_IRQ_RX_USED     (1<<2)      /* Rx buffer used bit read */
#define GEM_IRQ_RX_DONE     (1<<1)      /* Frame received ok */
#define GEM_IRQ_MAN_DONE    (1<<0)      /* PHY management operation complete */
#define GEM_IRQ_ALL         (0xFFFFFFFF)/* Everything! */

/* Bit positions for dma configuration register */
#define GEM_TX_CSUM_OFFLOAD (1 << 11)
#define GEM_RX_SW_ALLOC     (1<<25)
#define GEM_TX_SW_ALLOC     (1<<26)

/* Rx Admittance Control block */

#define GEM_ADM_BLOCK                           0x4000

#define ADM_STATUS                      0x0
#define ADM_PKTDQ                       0x4
#define ADM_CNFG                        0x8
#define ADM_CONTROL                     0xC
#define ADM_QUEUEDEPTH                  0x80
#define ADM_QFULLTHR                    0x88
#define ADM_QDROPMAXTHR                 0x8c
#define ADM_QDROPMINTHR                 0x90
#define ADM_DECAYTIMER                  0x98
#define ADM_BATCHINTRPKTCNT             0x100
#define ADM_BATCHINTRPKTTHRES           0x104
#define ADM_BATCHINTRTIMER              0x108
#define ADM_BATCHINTRTIMERINIT          0x10C
#define ADM_BATCHINTRSTAT               0x110

/* Tx Scheduling and Shaping block */

#define GEM_SCH_BLOCK                           0x8000

#define SCH_STATUS                      0x00
#define SCH_CONTROL                     0x04
#define SCH_PACKET_QUEUED               0x08
#define SCH_PORT_BYTE_COUNTER           0x0C
#define SCH_PORT_PACKET_COUNTER         0x10
#define SCH_PACKET_OVERHEAD             0x14
#define SCH_HW_FAULT_STATUS             0x18
#define SCH_HW_FAULT_MASK               0x1C
#define SCH_PORT_SHAPER_QBYTES          0x40
#define SCH_PORT_SHAPER_QPACKETS        0x44
#define SCH_PORT_SHAPER_RATE            0x50
#define SCH_PORT_SHAPER_MAx_CREDIT      0x54
#define SCH_PORT_SHAPER_CREDIT          0x58
#define SCH_PORT_SHAPER_CONTROL         0x5C
#define SCH_GROUP_SHAPER_QBYTES         0x60
#define SCH_GROUP_SHAPER_QPACKETS       0x64
#define SCH_GROUP_SHAPER_RATE           0x70
#define SCH_GROUP_SHAPER_MAx_CREDIT     0x74
#define SCH_GROUP_SHAPER_CREDIT         0x78
#define SCH_GROUP_SHAPER_CONTROL        0x7C
#define SCH_IOB_BASE                    0x80

// scheduler Queus
#define SCH_BYTES_Q0                    0x80
#define SCH_PACKETS_Q0                  0x84
#define SCH_IDLE_Q0                     0x88
#define SCH_BYTES_Q1                    0xa0
#define SCH_PACKETS_Q1                  0xa4
#define SCH_IDLE_Q1                     0xa8
#define SCH_BYTES_Q2                    0xc0
#define SCH_PACKETS_Q2                  0xc4
#define SCH_IDLE_Q2                     0xc8
#define SCH_BYTES_Q3                    0xe0
#define SCH_PACKETS_Q3                  0xe4
#define SCH_IDLE_Q3                     0xe8
#define SCH_BYTES_Q4                    0x100
#define SCH_PACKETS_Q4                  0x104
#define SCH_IDLE_Q4                     0x108
#define SCH_BYTES_Q5                    0x120
#define SCH_PACKETS_Q5                  0x124
#define SCH_IDLE_Q5                     0x128
#define SCH_BYTES_Q6                    0x140
#define SCH_PACKETS_Q6                  0x144
#define SCH_IDLE_Q6                     0x148
#define SCH_BYTES_Q7                    0x160
#define SCH_PACKETS_Q7                  0x164
#define SCH_IDLE_Q7                     0x168

/* on GEM_IP */

#define GEM_QUEUE_BASE0         (0x300)
#define GEM_QUEUE_BASE1         (0x304)
#define GEM_QUEUE_BASE2         (0x308)
#define GEM_QUEUE_BASE3         (0x30C)
#define GEM_QUEUE_BASE4         (0x310)
#define GEM_QUEUE_BASE5         (0x314)
#define GEM_QUEUE_BASE6         (0x318)
#define GEM_QUEUE_BASE7         (0x31C)

#define GEM_ID_CHECK5           (0x320)
#define GEM_ID_CHECK6           (0x324)
#define GEM_ID_CHECK7           (0x328)
#define GEM_ID_CHECK8           (0x32C)

#define GEM_CFG                                         0xF000

#define GEM_CONF_MODE_SEL_PIN                           (0 << 0)
#define GEM_CONF_MODE_SEL_GEM                           (1 << 0)
#define GEM_CONF_MODE_GEM_MASK                          (7 << 1)
#define GEM_CONF_MODE_GEM_RGMII                         (0 << 1)
#define GEM_CONF_MODE_GEM_RMII                          (1 << 1)
#define GEM_CONF_MODE_GEM_MII                           (2 << 1)
#define GEM_CONF_MODE_GEM_GMII                          (3 << 1)
#define GEM_CONF_MODE_PIN_MASK                          (7 << 4)
#define GEM_CONF_MODE_PIN_RGMII                         (0 << 4)
#define GEM_CONF_MODE_PIN_RMII                          (1 << 4)
#define GEM_CONF_MODE_PIN_MII                           (2 << 4)
#define GEM_CONF_MODE_PIN_GMII                          (3 << 4)
#define GEM_CONF_DUPLEX_SEL_PHY                         (0 << 8)
#define GEM_CONF_DUPLEX_SEL_GEM                         (1 << 8)
#define GEM_CONF_DUPLEX_GEM_HALF                        (0 << 9)
#define GEM_CONF_DUPLEX_GEM_FULL                        (1 << 9)
#define GEM_CONF_DUPLEX_PHY_HALF                        (0 << 10)
#define GEM_CONF_DUPLEX_PHY_FULL                        (1 << 10)
#define GEM_CONF_SPEED_SEL_PHY                          (0 << 11)
#define GEM_CONF_SPEED_SEL_GEM                          (1 << 11)
#define GEM_CONF_SPEED_MASK                             (3 << 12)
#define GEM_CONF_SPEED_GEM_10M                          (0 << 12)
#define GEM_CONF_SPEED_GEM_100M                         (1 << 12)
#define GEM_CONF_SPEED_GEM_1G                           (2 << 12)
#define GEM_CONF_SPEED_PHY_10M                          (0 << 14)
#define GEM_CONF_SPEED_PHY_100M                         (1 << 14)
#define GEM_CONF_SPEED_PHY_1G                           (2 << 14)
#define GEM_CONF_PHY_LINK_DOWN                          (0 << 16)
#define GEM_CONF_PHY_LINK_UP                            (1 << 16)
#define GEM_CONF_GEM_LOOPBACK                           (1 << 17)

#define GEM_TX_CTRL					0xF004
#define GEM_TX_COLL					0xF008
#define GEM_RX_CTRL					0xF010
#define GEM_RX_STAT_PKTSIZE				0xF014
#define GEM_RX_STAT_FIFODEPTH				0xF018
#define GEM_RX_STAT_FIFODATA				0xF01C

#define GEM_TXCTRL_DMAIF_EN				(1 << 0)
#define GEM_TXCTRL_CRC_EN				(1 << 1)
#define GEM_TXCTRL_RETR_EN				(1 << 2)
#define GEM_TXCTRL_TX_STATE				0xf0000


#define GEM_RXCTRL_DMAIF_EN				(1 << 0)
#define GEM_RXCTRL_RX_STATE				0xf0000


// Host fifo control bits
#define ARM_FIFO_RXDREQWE				(1 << 2)
#define ARM_FIFO_TXDREQRE				(1 << 3)
#define ARM_FIFO_TXFF_RES				(1 << 12)
#define ARM_FIFO_RXFF_RES				(1 << 13)
#define ARM_FIFO_RXCP_INH				(1 << 15)



#endif /* __IF_CGEREG_H__ */
