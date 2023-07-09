/*	$NetBSD$	*/

/*
 * Copyright (c) 2023 Hiroki Mori
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

#ifndef __IF_PGEREG_H__
#define	__IF_PGEREG_H__

#include <net/if.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/bpf.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

typedef struct bufDesc {
	uint32_t ctrl;
	uint32_t status;
	uint32_t data;
	struct bufDesc *next;
} bufDesc_t;

typedef struct hif_header_s {
	uint8_t port_no;	/* Carries input port no for host rx packets
			 	   and output port no for tx pkts */
	uint8_t reserved0;
        uint32_t reserved2;
} __attribute__((packed)) hif_header_t;


#define	PGE_RX_RING_CNT		64
#define	PGE_TX_RING_CNT		64
#define	PGE_TX_RING_SIZE	sizeof(struct bufDesc) * CGE_TX_RING_CNT
#define	PGE_RX_RING_SIZE	sizeof(struct bufDesc) * CGE_RX_RING_CNT

#define	PGE_MIN_FRAMELEN	(ETHER_MIN_LEN - ETHER_CRC_LEN)

#define	PFE_DDR_SIZE		0xc0

#define	PGE_TX_RING_ADDR(sc, i)	\
    ((sc)->pge_rdata.pge_tx_ring_paddr + sizeof(struct tTXdesc) * (i))
#define	PGE_RX_RING_ADDR(sc, i)	\
    ((sc)->pge_rdata.pge_rx_ring_paddr + sizeof(struct tRXdesc) * (i))
#define	PGE_INC(x,y)		(x) = (((x) + 1) % y)

struct pge_ring_data {
	bus_dmamap_t		tx_dm[PGE_TX_RING_CNT];
	struct mbuf		*tx_mb[PGE_TX_RING_CNT];
	bus_dmamap_t		rx_dm[PGE_RX_RING_CNT];
	struct mbuf		*rx_mb[PGE_RX_RING_CNT];
};

struct pge_softc {
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
	bus_dma_segment_t	sc_ddrsegs[1];
	bus_size_t		sc_ddrsize;
	bus_dmamap_t		sc_ddrmap;
	void			*sc_ddr;
#define sc_ddr_pa sc_ddrmap->dm_segs[0].ds_addr
	uint8_t			sc_enaddr[ETHER_ADDR_LEN];
	bool			sc_attached;
	struct pge_ring_data	*sc_rdp;
	struct mii_data		sc_mii;
	volatile u_int		sc_txnext;
	volatile u_int		sc_rxhead;
	void			*sc_ih;

	bufDesc_t		*sc_rxdesc_ring;
	bus_dmamap_t		sc_rxdesc_dmamap;

	bufDesc_t		*sc_txdesc_ring;
	bus_dmamap_t		sc_txdesc_dmamap;
	bool			sc_txbusy;

	kmutex_t		mtx;

	callout_t		sc_tick_ch;
};

#define PGE_LOCK(sc)		mutex_enter(&(sc)->mtx)
#define PGE_UNLOCK(sc)		mutex_exit(&(sc)->mtx)

/*
 * register space access macros
 */

static inline uint32_t
pge_read_4(struct pge_softc * const sc, bus_size_t const offset)
{
	return bus_space_read_4(sc->sc_bst, sc->sc_bsh, offset);
}

static inline void
pge_write_4(struct pge_softc * const sc, bus_size_t const offset,
    uint32_t const value)
{
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, offset, value);
}


/*
 * pfe
 */

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

#define CONFIG_UTIL_PE_DISABLED

#endif /* __IF_PGEREG_H__ */
