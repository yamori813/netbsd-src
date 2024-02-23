/*-
 * Copyright (c) 2024 Hiroki Mori
 * Copyright (c) 2012 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Northstar AMAC Ethernet driver based on if_cge.c */

#define _ARM32_BUS_DMA_PRIVATE
#define GMAC_PRIVATE

#include "locators.h"
#include "opt_broadcom.h"

#include <sys/cdefs.h>

__KERNEL_RCSID(1, "$NetBSD$");

#include <sys/param.h>
#include <sys/atomic.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/ioctl.h>
#include <sys/intr.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/socket.h>
#include <sys/systm.h>
#include <sys/workqueue.h>

#include <net/if.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/if_dl.h>
#include <net/bpf.h>

#include <dev/mii/miivar.h>

#include <arm/locore.h>

#include <arm/broadcom/bcm53xx_reg.h>
#include <arm/broadcom/bcm53xx_var.h>

#define AMAC_TX_RING_CNT	64
#define AMAC_RX_RING_CNT	64
#define AMAC_TXFRAGS		16

static int amac_ccb_match(device_t, cfdata_t, void *);
static int amac_ccb_detach(device_t, int);
static void amac_ccb_attach(device_t, device_t, void *);

struct tTXdesc {
	uint32_t	txdb_flags;
	uint32_t	txdb_buflen;
	uint32_t	txdb_addrlo;
	uint32_t	txdb_addrhi;
};

struct tRXdesc {
	uint32_t	rxdb_flags;
	uint32_t	rxdb_buflen;
	uint32_t	rxdb_addrlo;
	uint32_t	rxdb_addrhi;
};

struct amac_ring_data {
	bus_dmamap_t		tx_dm[AMAC_TX_RING_CNT];
	struct mbuf		*tx_mb[AMAC_TX_RING_CNT];
	bus_dmamap_t		rx_dm[AMAC_RX_RING_CNT];
	struct mbuf		*rx_mb[AMAC_RX_RING_CNT];
};

struct amac_softc {
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
	struct amac_ring_data	*sc_rdp;
	struct mii_data		sc_mii;
	volatile u_int		sc_txnext;
	volatile u_int		sc_rxhead;
	void			*sc_ih;
	int			sc_rcvoffset;

	struct tRXdesc		*sc_rxdesc_ring;
	bus_dmamap_t		sc_rxdesc_dmamap;

	struct tTXdesc		*sc_txdesc_ring;
	bus_dmamap_t		sc_txdesc_dmamap;
	bool			sc_txbusy;

	kmutex_t		mtx;
};

static void amac_start(struct ifnet *);
static int amac_ioctl(struct ifnet *, u_long, void *);
static void amac_watchdog(struct ifnet *);
static int amac_init(struct ifnet *);
static void amac_stop(struct ifnet *, int);

static int amac_intr(void *);
static int amac_rxintr(void *);
static int amac_txintr(void *);

#if 0
static int amac_mii_readreg(device_t, int, int, uint16_t *);
static int amac_mii_writereg(device_t, int, int, uint16_t);
static void amac_mii_statchg(struct ifnet *);
#endif

static int amac_new_rxbuf(struct amac_softc * const, const u_int);
//static void amac_tick(void *);

static int amac_alloc_dma(struct amac_softc *, size_t, void **, bus_dmamap_t *);

#define AMAC_MIN_FRAMELEN	(ETHER_MIN_LEN - ETHER_CRC_LEN)

#define AMAC_LOCK(sc)	mutex_enter(&(sc)->mtx)
#define AMAC_UNLOCK(sc)	mutex_exit(&(sc)->mtx)

#define TXDESC_NEXT(x)	amac_txdesc_adjust((x), 1)
#define TXDESC_PREV(x)	amac_txdesc_adjust((x), -1)

#define RXDESC_NEXT(x)	amac_rxdesc_adjust((x), 1)
#define RXDESC_PREV(x)	amac_rxdesc_adjust((x), -1)

static inline u_int
amac_txdesc_adjust(u_int x, int y)
{
	int res = x + y + AMAC_TX_RING_CNT;
	return res % AMAC_TX_RING_CNT;
}

static inline u_int
amac_rxdesc_adjust(u_int x, int y)
{
	int res = x + y + AMAC_RX_RING_CNT;
	return res % AMAC_RX_RING_CNT;
}


static inline uint32_t
amac_read_4(struct amac_softc *sc, bus_size_t o)
{
	return bus_space_read_4(sc->sc_bst, sc->sc_bsh, o);
}

static inline void
amac_write_4(struct amac_softc *sc, bus_size_t o, uint32_t v)
{
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, o, v);
}

CFATTACH_DECL_NEW(amac_ccb, sizeof(struct amac_softc),
	amac_ccb_match, amac_ccb_attach, amac_ccb_detach, NULL);

static int
amac_ccb_match(device_t parent, cfdata_t cf, void *aux)
{
	struct bcmccb_attach_args * const ccbaa = aux;
	const struct bcm_locators * const loc = &ccbaa->ccbaa_loc;

	if (strcmp(cf->cf_name, loc->loc_name))
		return 0;

#ifdef DIAGNOSTIC
	const int port = cf->cf_loc[BCMCCBCF_PORT];
	KASSERT(port == BCMCCBCF_PORT_DEFAULT || port == loc->loc_port);
#endif

	return 1;
}

static int
amac_ccb_detach(device_t self, int flags)
{
	return 1;
}

static void
amac_ccb_attach(device_t parent, device_t self, void *aux)
{
	struct amac_softc * const sc = device_private(self);
	struct bcmccb_attach_args * const ccbaa = aux;
	const struct bcm_locators * const loc = &ccbaa->ccbaa_loc;
	int i, error;
	struct ethercom * const ec = &sc->sc_ec;
	struct ifnet * const ifp = &ec->ec_if;

	sc->sc_dev = self;
	sc->sc_bst = ccbaa->ccbaa_ccb_bst;
	sc->sc_bdt = ccbaa->ccbaa_dmat;
	bus_space_subregion(sc->sc_bst, ccbaa->ccbaa_ccb_bsh,
	    loc->loc_offset, loc->loc_size, &sc->sc_bsh);

	mutex_init(&sc->mtx, MUTEX_DEFAULT, IPL_NET);

	sc->sc_ih = intr_establish(loc->loc_intrs[0], IPL_NET, IST_LEVEL,
	    amac_intr, sc);

	sc->sc_enaddr[0] = 0x00;
	sc->sc_enaddr[1] = 0x01;
	sc->sc_enaddr[2] = 0x02;
	sc->sc_enaddr[3] = 0x03;
	sc->sc_enaddr[4] = 0x04;
	sc->sc_enaddr[5] = 0x05 + device_unit(self);

	sc->sc_rdp = kmem_alloc(sizeof(*sc->sc_rdp), KM_SLEEP);

	for (i = 0; i < AMAC_TX_RING_CNT; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES,
		    AMAC_TXFRAGS, MCLBYTES, 0, 0,
		    &sc->sc_rdp->tx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create tx DMA map: %d\n", error);
		}
		sc->sc_rdp->tx_mb[i] = NULL;
	}

	for (i = 0; i < AMAC_RX_RING_CNT; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES, 1,
		    MCLBYTES, 0, 0, &sc->sc_rdp->rx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create rx DMA map: %d\n", error);
		}
		sc->sc_rdp->rx_mb[i] = NULL;
	}

	if (amac_alloc_dma(sc, sizeof(struct tRXdesc) * AMAC_RX_RING_CNT,
	    (void **)&(sc->sc_rxdesc_ring), &(sc->sc_rxdesc_dmamap)) != 0)
		return;

	if (amac_alloc_dma(sc, sizeof(struct tTXdesc) * AMAC_TX_RING_CNT,
	    (void **)&(sc->sc_txdesc_ring), &(sc->sc_txdesc_dmamap)) != 0)
		return;

	memset(sc->sc_rxdesc_ring, 0, sizeof(struct tRXdesc) * AMAC_RX_RING_CNT);

	for (i = 0; i < AMAC_TX_RING_CNT; i++) {
		sc->sc_txdesc_ring[i].txdb_addrhi = 0;
		sc->sc_txdesc_ring[i].txdb_addrlo = 0;
		sc->sc_txdesc_ring[i].txdb_flags = 0;
		if (i == AMAC_TX_RING_CNT - 1)
			sc->sc_txdesc_ring[i - 1].txdb_flags |= TXDB_FLAG_EF;
		bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
		    sizeof(struct tTXdesc) * i,
		    sizeof(struct tTXdesc),
		    BUS_DMASYNC_PREWRITE);
	}

	sc->sc_txpad = kmem_zalloc(ETHER_MIN_LEN, KM_SLEEP);
	bus_dmamap_create(sc->sc_bdt, ETHER_MIN_LEN, 1, ETHER_MIN_LEN, 0,
	    BUS_DMA_WAITOK, &sc->sc_txpad_dm);
	bus_dmamap_load(sc->sc_bdt, sc->sc_txpad_dm, sc->sc_txpad,
	    ETHER_MIN_LEN, NULL, BUS_DMA_WAITOK | BUS_DMA_WRITE);
	bus_dmamap_sync(sc->sc_bdt, sc->sc_txpad_dm, 0, ETHER_MIN_LEN,
	    BUS_DMASYNC_PREWRITE);
	amac_write_4(sc, GMAC_INTMASK, 0);    // disable interrupts

	aprint_naive("\n");
	aprint_normal(": Gigabit Ethernet Controller\n");

	strlcpy(ifp->if_xname, device_xname(sc->sc_dev), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_capabilities = 0;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_start = amac_start;
	ifp->if_ioctl = amac_ioctl;
	ifp->if_init = amac_init;
	ifp->if_stop = amac_stop;
	ifp->if_watchdog = amac_watchdog;
	IFQ_SET_READY(&ifp->if_snd);

	amac_stop(ifp, 0);

#if 0
	mii->mii_ifp = ifp;
	mii->mii_readreg = amac_mii_readreg;
	mii->mii_writereg = amac_mii_writereg;
	mii->mii_statchg = amac_mii_statchg;

	sc->sc_ec.ec_mii = mii;
	ifmedia_init(&mii->mii_media, 0, ether_mediachange, ether_mediastatus);

	if (device_unit(self) == 0) {
		mii_attach(self, mii, 0xffffffff, 0, MII_OFFSET_ANY, 0);
	}
#endif

	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr);

	/* The attach is successful. */
	sc->sc_attached = true;
}

static void
amac_start(struct ifnet *ifp)
{
	struct amac_softc * const sc = ifp->if_softc;
	struct amac_ring_data * const rdp = sc->sc_rdp;
	struct mbuf *m;
	bus_dmamap_t dm;
	u_int seg;
	u_int txfree;
	int txstart = -1;
	int error;
	bool pad;
	u_int mlen;
	u_int len;
	int i;

	AMAC_LOCK(sc);

	if (__predict_false((ifp->if_flags & IFF_RUNNING) == 0)) {
		device_printf(sc->sc_dev, "if not run\n");
		return;
	}
	if (__predict_false(sc->sc_txbusy)) {
		device_printf(sc->sc_dev, "txbusy\n");
		return;
	}

	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    0, sizeof(struct tTXdesc) * AMAC_TX_RING_CNT,
	    BUS_DMASYNC_PREREAD);
	txfree = 0;
	for (i = 0;i < AMAC_TX_RING_CNT; ++i) {
//		if(sc->sc_txdesc_ring[i].tx_ctl & GEMTX_USED_MASK)
			++txfree;
	}

	while (txfree > 0) {
		len = 0;
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

		dm = rdp->tx_dm[sc->sc_txnext];
		error = bus_dmamap_load_mbuf(sc->sc_bdt, dm, m,
		    BUS_DMA_WRITE | BUS_DMA_NOWAIT);
		if (error == EFBIG) {
			device_printf(sc->sc_dev, "won't fit\n");
			IFQ_DEQUEUE(&ifp->if_snd, m);
			m_freem(m);
			if_statinc(ifp, if_oerrors);
			continue;
		} else if (error != 0) {
			device_printf(sc->sc_dev, "error\n");
			break;
		}
		if (dm->dm_nsegs + 1 >= txfree) {
			sc->sc_txbusy = true;
			bus_dmamap_unload(sc->sc_bdt, dm);
			break;
		}

		mlen = m_length(m);
		pad = mlen < AMAC_MIN_FRAMELEN;

		KASSERT(rdp->tx_mb[sc->sc_txnext] == NULL);
		rdp->tx_mb[sc->sc_txnext] = m;
		IFQ_DEQUEUE(&ifp->if_snd, m);

		bus_dmamap_sync(sc->sc_bdt, dm, 0, dm->dm_mapsize,
		    BUS_DMASYNC_PREWRITE);

//		if (txstart == -1)
			txstart = sc->sc_txnext;
		for (seg = 0; seg < dm->dm_nsegs; seg++) {
			sc->sc_txdesc_ring[sc->sc_txnext].txdb_addrlo =
			    dm->dm_segs[seg].ds_addr;
			sc->sc_txdesc_ring[sc->sc_txnext].txdb_buflen =
			    dm->dm_segs[seg].ds_len;
			sc->sc_txdesc_ring[sc->sc_txnext].txdb_flags = 0;
			len += dm->dm_segs[seg].ds_len;
			if (seg == 0)
				sc->sc_txdesc_ring[sc->sc_txnext].txdb_flags |=
				    TXDB_FLAG_SF;
			if (!pad && (seg == dm->dm_nsegs - 1))
				sc->sc_txdesc_ring[sc->sc_txnext].txdb_flags |=
				    (TXDB_FLAG_EF | TXDB_FLAG_IC);
			if (sc->sc_txnext == AMAC_TX_RING_CNT - 1)
				sc->sc_txdesc_ring[sc->sc_txnext].txdb_flags |=
				    TXDB_FLAG_ET;
			txfree--;
			bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
			    sizeof(struct tTXdesc) * sc->sc_txnext,
			    sizeof(struct tTXdesc), BUS_DMASYNC_PREWRITE);
			sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
		}
		if (pad) {
			sc->sc_txdesc_ring[sc->sc_txnext].txdb_addrlo =
			    sc->sc_txpad_pa;
			sc->sc_txdesc_ring[sc->sc_txnext].txdb_buflen =
			    AMAC_MIN_FRAMELEN - mlen;
			len += AMAC_MIN_FRAMELEN - mlen;
			sc->sc_txdesc_ring[sc->sc_txnext].txdb_flags =
			    (TXDB_FLAG_EF | TXDB_FLAG_IC);
			if (sc->sc_txnext == AMAC_TX_RING_CNT - 1)
				sc->sc_txdesc_ring[sc->sc_txnext].txdb_flags |=
				    TXDB_FLAG_ET;
			txfree--;
			bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
			    sizeof(struct tTXdesc) * sc->sc_txnext,
			    sizeof(struct tTXdesc), BUS_DMASYNC_PREWRITE);
			sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
		}
//		amac_write_4(sc, GEM_SCH_BLOCK + SCH_PACKET_QUEUED, len);
		bpf_mtap(ifp, m, BPF_D_OUT);
	}
	amac_write_4(sc, GMAC_XMTPTR, sc->sc_txnext * 16);

	if (txstart >= 0) {
		ifp->if_timer = 300;	/* not immediate interrupt */
	}

	AMAC_UNLOCK(sc);
}

static int
amac_ioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	struct amac_softc * const sc = ifp->if_softc;
	const int s = splnet();
	int error = 0;
	int reg;

	switch (cmd) {
	case SIOCSIFFLAGS:
		if ((ifp->if_flags & (IFF_PROMISC | IFF_ALLMULTI)) != 0) {
			reg = amac_read_4(sc, UNIMAC_COMMAND_CONFIG);
			reg |= PROMISC_EN;
			amac_write_4(sc, UNIMAC_COMMAND_CONFIG, reg);
		} else {
			reg = amac_read_4(sc, UNIMAC_COMMAND_CONFIG);
			reg &= ~PROMISC_EN;
			amac_write_4(sc, UNIMAC_COMMAND_CONFIG, reg);
		}
		break;
	default:
		error = ether_ioctl(ifp, cmd, data);
		if (error == ENETRESET) {
			error = 0;
		}
		break;
	}

	splx(s);

	return error;
}

static void
amac_watchdog(struct ifnet *ifp)
{
	struct amac_softc *sc = ifp->if_softc;

	device_printf(sc->sc_dev, "device timeout %d\n", sc->sc_txnext);

	if_statinc(ifp, if_oerrors);
#if 0
	ifp->if_flags |= IFF_RUNNING;
	amac_init(ifp);
	amac_start(ifp);
#endif
}

#if 0
static int
amac_mii_readreg(device_t dev, int phy, int reg, uint16_t *val)
{
	struct amac_softc * const sc = device_private(dev);
	int result, wdata;

	wdata = 0x60020000;
	wdata |= ((phy << 23) | (reg << 18));
	amac_write_4(sc, GEM_IP + GEM_PHY_MAN, wdata);
	while(!(amac_read_4(sc, GEM_IP + GEM_NET_STATUS) & GEM_PHY_IDLE))
		;
	result = amac_read_4(sc, GEM_IP + GEM_PHY_MAN);

	*val = (uint16_t)result;
	return 0;
}

static int
amac_mii_writereg(device_t dev, int phy, int reg, uint16_t val)
{
	struct amac_softc * const sc = device_private(dev);
	int wdata;

	wdata = 0x50020000;
	wdata |= ((phy << 23) | (reg << 18) | val);
	amac_write_4(sc, GEM_IP + GEM_PHY_MAN, wdata);
	while(!(amac_read_4(sc, GEM_IP + GEM_NET_STATUS) & GEM_PHY_IDLE))
		;
	return 0;
}

static void
amac_mii_statchg(struct ifnet *ifp)
{
	return;
}
#endif

static int
amac_new_rxbuf(struct amac_softc * const sc, const u_int i)
{
	struct amac_ring_data * const rdp = sc->sc_rdp;
//	const u_int h = RXDESC_PREV(i);
//	struct amac_cpdma_bd bd;
//	uint32_t * const dw = bd.word;
	struct mbuf *m;
	int error = ENOBUFS;

	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m == NULL) {
		goto reuse;
	}

	MCLGET(m, M_DONTWAIT);
	if ((m->m_flags & M_EXT) == 0) {
		m_freem(m);
		goto reuse;
	}

	/* We have a new buffer, prepare it for the ring. */

	if (rdp->rx_mb[i] != NULL)
		bus_dmamap_unload(sc->sc_bdt, rdp->rx_dm[i]);

	m->m_len = m->m_pkthdr.len = MCLBYTES;

	rdp->rx_mb[i] = m;

	error = bus_dmamap_load_mbuf(sc->sc_bdt, rdp->rx_dm[i], rdp->rx_mb[i],
	    BUS_DMA_READ | BUS_DMA_NOWAIT);
	if (error) {
		device_printf(sc->sc_dev, "can't load rx DMA map %d: %d\n",
		    i, error);
	}

	bus_dmamap_sync(sc->sc_bdt, rdp->rx_dm[i],
	    0, rdp->rx_dm[i]->dm_mapsize, BUS_DMASYNC_PREREAD);

	error = 0;

reuse:
	sc->sc_rxdesc_ring[i].rxdb_addrlo = rdp->rx_dm[i]->dm_segs[0].ds_addr;
	sc->sc_rxdesc_ring[i].rxdb_buflen = MCLBYTES;
	sc->sc_rxdesc_ring[i].rxdb_flags = RXDB_FLAG_IC;

	if (i == AMAC_RX_RING_CNT - 1)
		sc->sc_rxdesc_ring[i].rxdb_flags |= RXDB_FLAG_ET;

	bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
	    sizeof(struct tRXdesc) * i, sizeof(struct tRXdesc),
            BUS_DMASYNC_PREWRITE);

	return error;
}

static uint64_t
amac_macaddr_create(const uint8_t *enaddr)
{
	return (enaddr[3] << 0)			// UNIMAC_MAC_0
	    |  (enaddr[2] << 8)			// UNIMAC_MAC_0
	    |  (enaddr[1] << 16)		// UNIMAC_MAC_0
	    |  ((uint64_t)enaddr[0] << 24)	// UNIMAC_MAC_0
	    |  ((uint64_t)enaddr[5] << 32)	// UNIMAC_MAC_1
	    |  ((uint64_t)enaddr[4] << 40);	// UNIMAC_MAC_1
}

static int
amac_init(struct ifnet *ifp)
{
	struct amac_softc * const sc = ifp->if_softc;
	int i;
	uint32_t reg;
	uint64_t mac;
	uint32_t intmask;
	paddr_t paddr;

	amac_stop(ifp, 0);

	intmask = DESCPROTOERR | DATAERR | DESCERR;

	sc->sc_txnext = 0;

	/* Init circular RX list. */
	for (i = 0; i < AMAC_RX_RING_CNT; i++) {
		amac_new_rxbuf(sc, i);
	}
	sc->sc_rxhead = 0;

	/* 5. Load RCVADDR_LO with new pointer */
	paddr = sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr;
	amac_write_4(sc, GMAC_RCVADDR_LOW, paddr);
	sc->sc_rcvoffset = max_linkhdr + 4 - sizeof(struct ether_header);
	if (sc->sc_rcvoffset <= 4)
		sc->sc_rcvoffset += 4;
	amac_write_4(sc, GMAC_RCVCONTROL,
	    __SHIFTIN(sc->sc_rcvoffset, RCVCTL_RCVOFFSET) |
	    RCVCTL_PARITY_DIS |
	    RCVCTL_OFLOW_CONTINUE |
	    __SHIFTIN(3, RCVCTL_BURSTLEN));

        /* 6. Load XMTADDR_LO with new pointer */
	paddr = sc->sc_txdesc_dmamap->dm_segs[0].ds_addr;
	amac_write_4(sc, GMAC_XMTADDR_LOW, paddr);
	amac_write_4(sc, GMAC_XMTCONTROL, XMTCTL_DMA_ACT_INDEX |
	    XMTCTL_PARITY_DIS |
	    __SHIFTIN(3, XMTCTL_BURSTLEN));

	/* 7. Setup other UNIMAC registers */
	amac_write_4(sc, UNIMAC_FRAME_LEN, uimax(ifp->if_mtu + 32, MCLBYTES));
	mac = amac_macaddr_create(sc->sc_enaddr);
	amac_write_4(sc, UNIMAC_MAC_0, (uint32_t)(mac >> 0));
	amac_write_4(sc, UNIMAC_MAC_1, (uint32_t)(mac >> 32));
	amac_write_4(sc, UNIMAC_COMMAND_CONFIG, NO_LENGTH_CHECK | PAUSE_IGNORE |
            __SHIFTIN(ETH_SPEED_1000, ETH_SPEED) | RX_ENA | TX_ENA);

	reg = amac_read_4(sc, GMAC_DEVCONTROL);
	reg |= RGMII_LINK_STATUS_SEL | NWAY_AUTO_POLL_EN | TXARB_STRICT_MODE;
	reg &= ~FLOW_CTRL_MODE;
	reg &= ~MIB_RD_RESET_EN;
	reg &= ~RXQ_OVERFLOW_CTRL_SEL;
	reg &= ~CPU_FLOW_CTRL_ON;
	amac_write_4(sc, GMAC_DEVCONTROL, reg);

	/* Setup lazy receive (at most 1ms). */
	const struct cpu_softc * const cpu = curcpu()->ci_softc;
	reg =  __SHIFTIN(4, INTRCVLAZY_FRAMECOUNT) |
	    __SHIFTIN(cpu->cpu_clk.clk_apb / 1000, INTRCVLAZY_TIMEOUT);
	amac_write_4(sc, GMAC_INTRCVLAZY, reg);

	 /* 11. Enable transmit queues in TQUEUE, and ensure that the transmit
	 scheduling mode is correctly set in TCTRL. */
	intmask |= XMTINT_0 | XMTUF;
	amac_write_4(sc, GMAC_XMTCONTROL,
	    amac_read_4(sc, GMAC_XMTCONTROL) | XMTCTL_ENABLE);

	/* 12. Enable receive queues in RQUEUE, */
	intmask |= RCVINT | RCVDESCUF | RCVFIFOOF;
	amac_write_4(sc, GMAC_RCVCONTROL,
	    amac_read_4(sc, GMAC_RCVCONTROL) | RCVCTL_ENABLE);

//	amac_write_4(sc, GMAC_RCVPTR, 16 * amac_rxdesc_adjust(0, 16));
	amac_write_4(sc, GMAC_RCVPTR, 16 * AMAC_RX_RING_CNT);

	amac_write_4(sc, GMAC_INTMASK, intmask);

	ifp->if_flags |= IFF_RUNNING;

	return 0;
}

static void
amac_stop(struct ifnet *ifp, int disable)
{
	struct amac_softc * const sc = ifp->if_softc;
	struct amac_ring_data * const rdp = sc->sc_rdp;
	u_int i;
//	int reg;

	aprint_debug_dev(sc->sc_dev, "%s: ifp %p disable %d\n", __func__,
	    ifp, disable);

	if ((ifp->if_flags & IFF_RUNNING) == 0)
		return;

	amac_write_4(sc, UNIMAC_COMMAND_CONFIG,
	     amac_read_4(sc, UNIMAC_COMMAND_CONFIG) | SW_RESET);

	/* Disable Rx processing */
	amac_write_4(sc, GMAC_RCVCONTROL,
	    amac_read_4(sc, GMAC_RCVCONTROL) & ~RCVCTL_ENABLE);

	/* Disable Tx processing */
	amac_write_4(sc, GMAC_XMTCONTROL,
	    amac_read_4(sc, GMAC_XMTCONTROL) & ~XMTCTL_ENABLE);

	/* Disable all interrupts */
	amac_write_4(sc, GMAC_INTMASK, 0);

	amac_write_4(sc, UNIMAC_COMMAND_CONFIG,
	     amac_read_4(sc, UNIMAC_COMMAND_CONFIG) & ~SW_RESET);
#if 0
//	callout_stop(&sc->sc_tick_ch);
	mii_down(&sc->sc_mii);

	amac_write_4(sc, GEM_IP + GEM_IRQ_ENABLE, 0);

	reg = amac_read_4(sc, GEM_IP + GEM_NET_CONTROL);
	reg &= ~(GEM_TX_EN | GEM_RX_EN);
	amac_write_4(sc, GEM_IP + GEM_NET_CONTROL, reg);

	/* Release any queued transmit buffers. */
	for (i = 0; i < AMAC_TX_RING_CNT; i++) {
		if (rdp->tx_mb[i] != NULL) {
			bus_dmamap_unload(sc->sc_bdt, rdp->tx_dm[i]);
			m_freem(rdp->tx_mb[i]);
			rdp->tx_mb[i] = NULL;
		}
	}
#endif

	ifp->if_flags &= ~IFF_RUNNING;
	ifp->if_timer = 0;
	sc->sc_txbusy = false;

	if (!disable)
		return;

	for (i = 0; i < AMAC_RX_RING_CNT; i++) {
		bus_dmamap_unload(sc->sc_bdt, rdp->rx_dm[i]);
		m_freem(rdp->rx_mb[i]);
		rdp->rx_mb[i] = NULL;
	}
}

static int
amac_intr(void *arg)
{
	struct amac_softc * const sc = arg;
	int reg;

	for (;;) {
		reg = amac_read_4(sc, GMAC_INTSTATUS);
		amac_write_4(sc, GMAC_INTSTATUS, reg);
		if (reg == 0)
			break;

		if (reg & RCVINT)
			amac_rxintr(arg);
		if (reg & XMTINT_0)
			amac_txintr(arg);
		if (reg & ~(RCVINT | XMTINT_0))
			aprint_error_dev(sc->sc_dev, "intr error %x\n", reg);
	}

	return 1;
}

#if 0
static void
amac_tick(void *arg)
{
	struct amac_softc * const sc = arg;
	struct mii_data * const mii = &sc->sc_mii;
	const int s = splnet();

	mii_tick(mii);

	splx(s);

	callout_schedule(&sc->sc_tick_ch, hz);
}
#endif

static int
amac_rxintr(void *arg)
{
	struct amac_softc * const sc = arg;
	u_int i, cur, act;
	int rxsts, length;
	struct amac_ring_data * const rdp = sc->sc_rdp;
	struct mbuf *m;
	struct ifnet *ifp = &sc->sc_ec.ec_if;

	uint32_t rcvsts0 = amac_read_4(sc, GMAC_RCVSTATUS0);
	uint32_t rcvsts1 = amac_read_4(sc, GMAC_RCVSTATUS1);
//	i = sc->sc_rxhead;
	cur = __SHIFTOUT(rcvsts0, RCV_CURRDSCR);
	act = __SHIFTOUT(rcvsts1, RCV_ACTIVEDSCR);
	i = RXDESC_PREV(act);
	i = RXDESC_PREV(cur);
	i = sc->sc_rxhead;
/*
	if (sc->sc_rxhead != RXDESC_PREV(cur)) {
		aprint_error_dev(sc->sc_dev, "RX Error %d %d %d\n", sc->sc_rxhead,
		    cur, act);
	}
*/
//printf("MORIRXSTS %d, %d, ", i, cur);
	for (i = sc->sc_rxhead; i != cur; i = RXDESC_NEXT(i)) {
		bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
		    sizeof(struct tRXdesc) * i, sizeof(struct tRXdesc),
		    BUS_DMASYNC_PREREAD);
		m = rdp->rx_mb[i];
		bus_dmamap_sync(sc->sc_bdt, rdp->rx_dm[i],
		    0, rdp->rx_dm[i]->dm_mapsize,
		    BUS_DMASYNC_POSTREAD);
		memcpy(&rxsts, m->m_data, 4);
		length = __SHIFTOUT(rxsts, RXSTS_FRAMELEN);
/*
	int desc_count = __SHIFTOUT(rxsts, RXSTS_DESC_COUNT) + 1;
printf("%d,", desc_count);
printf("%d-%d: %02x %02x %02x, ", length, sc->sc_rcvoffset,
m->m_data[sc->sc_rcvoffset], m->m_data[sc->sc_rcvoffset+1], m->m_data[sc->sc_rcvoffset+2]);
*/
		m->m_pkthdr.len = m->m_len = length;
//		m_adj(m, sc->sc_rcvoffset;
		m->m_data += sc->sc_rcvoffset;
		m_set_rcvif(m, ifp);
		if_percpuq_enqueue(ifp->if_percpuq, m);
		amac_new_rxbuf(sc, i);
		bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
		    sizeof(struct tRXdesc) * i, sizeof(struct tRXdesc),
       		    BUS_DMASYNC_PREWRITE);

		sc->sc_rxhead = RXDESC_NEXT(i);
	}

	return 1;
}

static int
amac_txintr(void *arg)
{
	struct amac_softc * const sc = arg;
	struct amac_ring_data * const rdp = sc->sc_rdp;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	int i, remain;
	bool handled = false;
	uint32_t cur, act;

	uint32_t xmtsts0 = amac_read_4(sc, GMAC_XMTSTATUS0);
	uint32_t xmtsts1 = amac_read_4(sc, GMAC_XMTSTATUS1);
//	i = sc->sc_rxhead;
	cur = __SHIFTOUT(xmtsts0, RCV_CURRDSCR);
	act = __SHIFTOUT(xmtsts1, RCV_ACTIVEDSCR);
	remain = 0;
//	i = TXDESC_PREV(sc->sc_txnext);
//printf("MORI %d-%d-%d,", i, cur, act);
	i = cur;
	i = act;
	bus_dmamap_sync(sc->sc_bdt, rdp->tx_dm[i],
	    0, rdp->tx_dm[i]->dm_mapsize,
	    BUS_DMASYNC_POSTWRITE);
	bus_dmamap_unload(sc->sc_bdt, rdp->tx_dm[i]);

	m_freem(rdp->tx_mb[i]);
	rdp->tx_mb[i] = NULL;

	if_statinc(ifp, if_opackets);

	handled = true;

#if 0

	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    0, sizeof(struct tTXdesc) * AMAC_TX_RING_CNT, BUS_DMASYNC_PREREAD);
	for (i = 0; i < AMAC_TX_RING_CNT; i++) {
		if (!(sc->sc_txdesc_ring[i].tx_ctl & GEMTX_USED_MASK)) {
			++remain;
			continue;
		}

		if ((sc->sc_txdesc_ring[i].tx_ctl & GEMTX_BUFRET) == 0)
			continue;

		sc->sc_txbusy = false;

		sc->sc_txdesc_ring[i].tx_ctl = GEMTX_USED_MASK;
		bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
		    sizeof(struct tTXdesc) * i, sizeof(struct tTXdesc),
		    BUS_DMASYNC_PREWRITE);

	}

#endif
	if (remain == 0)
		ifp->if_timer = 0;
	if (handled)
		if_schedule_deferred_start(ifp);

	return handled;
}

static int
amac_alloc_dma(struct amac_softc *sc, size_t size, void **addrp,
              bus_dmamap_t *mapp)
{
	bus_dma_segment_t seglist[1];
	int nsegs, error;

	if ((error = bus_dmamem_alloc(sc->sc_bdt, size, PAGE_SIZE, 0, seglist,
	    1, &nsegs, M_WAITOK)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to allocate DMA buffer, error=%d\n", error);
		goto fail_alloc;
	}

	if ((error = bus_dmamem_map(sc->sc_bdt, seglist, 1, size, addrp,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to map DMA buffer, error=%d\n",
		    error);
		goto fail_map;
	}

	if ((error = bus_dmamap_create(sc->sc_bdt, size, 1, size, 0,
	    BUS_DMA_NOWAIT, mapp)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to create DMA map, error=%d\n", error);
		goto fail_create;
	}

	if ((error = bus_dmamap_load(sc->sc_bdt, *mapp, *addrp, size, NULL,
	    BUS_DMA_NOWAIT)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to load DMA map, error=%d\n", error);
		goto fail_load;
	}

	return 0;

 fail_load:
	bus_dmamap_destroy(sc->sc_bdt, *mapp);
 fail_create:
	bus_dmamem_unmap(sc->sc_bdt, *addrp, size);
 fail_map:
	bus_dmamem_free(sc->sc_bdt, seglist, 1);
 fail_alloc:
	return error;
}
