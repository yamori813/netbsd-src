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

/*
 * STR813X/STR818X Equuleus series SoC
 * Gigabit Ethernet Controller with Embedded 10/100M PHY (GEC)
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#undef DEBUG_GEC_DUMP
#define DEBUG_GEC

#include "vlan.h"
#include "rnd.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/mbuf.h>
#include <sys/device.h>
#include <sys/sockio.h>
#include <sys/kernel.h>
#include <sys/callout.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_ether.h>

#include <netinet/in.h>
#include <netinet/in_systm.h>
#include <netinet/ip.h>

#include <net/bpf.h>
#if NRND > 0
#include <sys/rnd.h>
#endif

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <arm/star/starvar.h>
#include <arm/star/if_gecreg.h>

#define GEC_ETHER_MIN_LEN	(ETHER_MIN_LEN - ETHER_CRC_LEN)	/* not including FCS */
#define GEC_ETHER_MIN_LEN_VLAN	(GEC_ETHER_MIN_LEN + ETHER_VLAN_ENCAP_LEN)	/* not including VLAN-tag and FCS */
#define GEC_MAX_PKT_LEN		1536
#define GEC_MAX_PKT_NSEGS	16	/* XXX */

#define GEC_TX_RING_CNT		256	/* must be 2^n */
#define GEC_TX_RING_MINIMAL_CNT	1	/* ready to build padding */
//#define GEC_RX_RING_CNT		512	/* must be 2^n */
#define GEC_RX_RING_CNT		128	/* must be 2^n */

#define GEC_TX_NEXTIDX(idx)	(((idx) + 1) & (GEC_TX_RING_CNT - 1))
#define GEC_RX_NEXTIDX(idx)	(((idx) + 1) & (GEC_RX_RING_CNT - 1))

#define GEC_ARL_HASHTABLE_USE_CRC	/* use CRC algorithm for ARL hash */

#define ETHER_ALIGN	2	/* recomennded */
#ifndef ETHER_ALIGN
#define ETHER_ALIGN	0
#endif

struct gec_softc {
	device_t sc_dev;

	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	bus_dma_tag_t sc_dmat;

	/* interrupts */
	void *sc_ih_mib;	/* IRQ18 MIB counter, Port status changed */
	void *sc_ih_tx;		/* IRQ19 TX complete */
	void *sc_ih_rx;		/* IRQ20 RX complete */
	/* IRQ21 (Queue Empty) and IRQ22 (Queue Full) are not used */

	/* TX */
	struct gec_txdesc *sc_txdesc_ring;	/* gec_txdesc[GEC_TX_RING_CNT] */
	bus_dmamap_t sc_txdesc_dmamap;
	struct gec_rxdesc *sc_rxdesc_ring;	/* gec_rxdesc[GEC_RX_RING_CNT] */
	bus_dmamap_t sc_rxdesc_dmamap;
	void *sc_padseg;			/* 64byte stub segment */
	bus_dmamap_t sc_padseg_dmamap;		/* pad for short TX frame */
	struct gec_txsoft {
		struct mbuf *txs_mbuf;		/* head of our mbuf chain */
		bus_dmamap_t txs_dmamap;	/* our DMA map */
	} sc_txsoft[GEC_TX_RING_CNT];
	int sc_tx_considx;
	int sc_tx_prodidx;
	int sc_tx_free;

	/* RX */
	struct gec_rxsoft {
		struct mbuf *rxs_mbuf;		/* head of our mbuf chain */
		bus_dmamap_t rxs_dmamap;	/* our DMA map */
	} sc_rxsoft[GEC_RX_RING_CNT];
	int sc_rx_readidx;
	uint32_t sc_rx_last_dptr;
	callout_t sc_rx_ch;			/* RX stall check callout */

	/* misc */
	uint8_t sc_hashtable[GEC_HASH_CTRL_HASHBITSIZE / 8];
	int sc_if_flags;			/* local copy of if_flags */
	int sc_flowflags;			/* 802.3x flow control flags */
	struct ethercom sc_ethercom;		/* interface info */
	struct mii_data sc_mii;
	uint8_t sc_enaddr[ETHER_ADDR_LEN];
#if NRND > 0
	rndsource_element_t sc_rnd_source;
#endif
};

#define TXDESC_WRITEOUT(idx)					\
	bus_dmamap_sync(sc->sc_dmat, sc->sc_txdesc_dmamap,	\
	    sizeof(struct gec_txdesc) * (idx),			\
	    sizeof(struct gec_txdesc),				\
	    BUS_DMASYNC_PREWRITE)

#define TXDESC_READIN(idx)					\
	bus_dmamap_sync(sc->sc_dmat, sc->sc_txdesc_dmamap,	\
	    sizeof(struct gec_txdesc) * (idx),			\
	    sizeof(struct gec_txdesc),				\
	    BUS_DMASYNC_PREREAD)

#define RXDESC_WRITEOUT(idx)					\
	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxdesc_dmamap,	\
	    sizeof(struct gec_rxdesc) * (idx),			\
	    sizeof(struct gec_rxdesc),				\
	    BUS_DMASYNC_PREWRITE)

#define RXDESC_READIN(idx)					\
	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxdesc_dmamap,	\
	    sizeof(struct gec_rxdesc) * (idx),			\
	    sizeof(struct gec_rxdesc),				\
	    BUS_DMASYNC_PREREAD)

#define GEC_REG_SETBIT(sc, reg, bit)					\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, reg,		\
	    bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, reg) | (bit))

#define GEC_REG_CLRBIT(sc, reg, bit)					\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, reg,		\
	    bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, reg) & ~(bit))

#define TX_DPTR2IDX(dptr)					\
	(((dptr) - sc->sc_txdesc_dmamap->dm_segs[0].ds_addr) /	\
	sizeof(struct gec_txdesc))

#define RX_DPTR2IDX(dptr)					\
	(((dptr) - sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr) /	\
	sizeof(struct gec_rxdesc))


static int gec_match(device_t, struct cfdata *, void *);
static void gec_attach(device_t, device_t, void *);

/* interrupt handlers */
static int gec_mib_intr(void *);
static int gec_tx_intr(void *);
static int gec_rx_intr(void *);

/* for ifnet interface */
static void gec_start(struct ifnet *);
static int gec_ioctl(struct ifnet *, u_long, void *);
static int gec_init(struct ifnet *);
static void gec_stop(struct ifnet *, int);
static void gec_watchdog(struct ifnet *);

/* for MII */
static int gec_miibus_readreg(device_t, int, int);
static void gec_miibus_writereg(device_t, int, int, int);
static void gec_miibus_statchg(device_t);

/* misc routines */
static void gec_setmulti(struct gec_softc *);
static void gec_clear_hashtable(struct gec_softc *);
static void gec_add_hashtable(struct gec_softc *, uint8_t *);
static void gec_apply_hashtable(struct gec_softc *);
static void gec_stall_recover(void *arg);
static int gec_stall_check(struct gec_softc *, int);
static void gec_update_mib(struct gec_softc *);
static void gec_clear_mib_counter(struct gec_softc *);
static int gec_encap(struct gec_softc *, struct mbuf *);
static int gec_init_regs(struct gec_softc *);
static int gec_alloc_ring(struct gec_softc *);
static void gec_init_txring(struct gec_softc *);
static int gec_init_rxring(struct gec_softc *);
static void gec_reset_rxdesc(struct gec_softc *, int);
static int gec_alloc_rxbuf(struct gec_softc *, int);
static void gec_drain_txbuf(struct gec_softc *);
static void gec_drain_rxbuf(struct gec_softc *);
static int gec_alloc_dma(struct gec_softc *, size_t, void **,
                         bus_dmamap_t *);

#ifdef DEBUG_GEC
static void gec_dump_txdesc(struct gec_softc *, int, const char *);
static void gec_dump_rxdesc(struct gec_softc *, int, const char *);
#endif
#ifdef DEBUG_GEC_DUMP
static void gec_dump_hashtable(struct gec_softc *);
#endif

CFATTACH_DECL_NEW(gec, sizeof(struct gec_softc),
    gec_match, gec_attach, NULL, NULL);

/* ARGSUSED */
static int
gec_match(device_t parent __unused, struct cfdata *match __unused, void *aux)
{
	struct star_attach_args *sa;

	if (!CPU_IS_STR8100())
		return 0;

	sa = (struct star_attach_args *)aux;

	sa->sa_size = GEC_REG_SIZE;
	return 1;
}

/* ARGSUSED */
static void
gec_attach(device_t parent __unused, device_t self, void *aux)
{
	struct gec_softc *sc;
	struct star_attach_args *sa;
	uint32_t mac_h, mac_l;
	struct ifnet *ifp;

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_addr = sa->sa_addr;
	sc->sc_dmat = sa->sa_dmat;

	aprint_naive("\n");
	aprint_normal(": Gigabit Ethernet Controller\n");

	if (bus_space_map(sc->sc_iot, sc->sc_addr, sa->sa_size, 0,
	    &sc->sc_ioh)) {
		aprint_error_dev(self, "Cannot map registers\n");
		return;
	}

	/* allocate dma buffer */
	if (gec_alloc_ring(sc))
		return;
	gec_init_regs(sc);

	mac_h = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_MY_MAC_H_REG);
	mac_l = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_MY_MAC_L_REG);
	sc->sc_enaddr[0] = mac_h >> 8;
	sc->sc_enaddr[1] = mac_h;
	sc->sc_enaddr[2] = mac_l >> 24;
	sc->sc_enaddr[3] = mac_l >> 16;
	sc->sc_enaddr[4] = mac_l >> 8;
	sc->sc_enaddr[5] = mac_l;
	aprint_normal_dev(self, "Ethernet address %s\n",
	    ether_sprintf(sc->sc_enaddr));

	/*
	 * setup interrupt handlers
	 */
	if ((sc->sc_ih_mib = intr_establish(STAR_IRQ_NIC_STAT, IPL_NET,
	    IST_LEVEL_HIGH, gec_mib_intr, sc)) == NULL) {
		aprint_error_dev(self, "unable to establish MIB interrupt\n");
		goto failure;
	}
	if ((sc->sc_ih_tx = intr_establish(STAR_IRQ_NIC_TX, IPL_NET,
	    IST_EDGE_RISING, gec_tx_intr, sc)) == NULL) {
		aprint_error_dev(self, "unable to establish TX interrupt\n");
		goto failure;
	}
	if ((sc->sc_ih_rx = intr_establish(STAR_IRQ_NIC_RX, IPL_NET,
	    IST_EDGE_RISING, gec_rx_intr, sc)) == NULL) {
		aprint_error_dev(self, "unable to establish RX interrupt\n");
		goto failure;
	}
	callout_init(&sc->sc_rx_ch, 0);
	callout_setfunc(&sc->sc_rx_ch, gec_stall_recover, sc);

	/*
	 * setup ifp
	 */
	ifp = &sc->sc_ethercom.ec_if;
	strlcpy(ifp->if_xname, device_xname(sc->sc_dev), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_mtu = ETHERMTU;
	ifp->if_baudrate = IF_Mbps(100);
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = gec_ioctl;
	ifp->if_start = gec_start;
	ifp->if_init = gec_init;
	ifp->if_stop = gec_stop;
	ifp->if_watchdog = gec_watchdog;

	ifp->if_capabilities =
#if 0 /* XXX: ip4csum-tx wouldn't work. Why??? */
	    IFCAP_CSUM_IPv4_Tx |
#endif
	    IFCAP_CSUM_IPv4_Rx |
	    IFCAP_CSUM_TCPv4_Tx | IFCAP_CSUM_TCPv4_Rx |
	    IFCAP_CSUM_UDPv4_Tx | IFCAP_CSUM_UDPv4_Rx;

	IFQ_SET_MAXLEN(&ifp->if_snd, max(GEC_TX_RING_CNT - 1, IFQ_MAXLEN));
	IFQ_SET_READY(&ifp->if_snd);

	/* setup MII */
	sc->sc_ethercom.ec_mii = &sc->sc_mii;
	sc->sc_mii.mii_ifp = ifp;
	sc->sc_mii.mii_readreg = gec_miibus_readreg;
	sc->sc_mii.mii_writereg = gec_miibus_writereg;
	sc->sc_mii.mii_statchg = gec_miibus_statchg;
	ifmedia_init(&sc->sc_mii.mii_media, 0, ether_mediachange,
	    ether_mediastatus);

	/* At first, try to attach External PHY */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_PHY_CTRL1_REG,
	    GEC_PHY_CTRL1_AN_EN);
	mii_attach(self, &sc->sc_mii, 0xffffffff, MII_PHY_ANY,
	    MII_OFFSET_ANY, 0);

	/* In the next, try to attach Internal PHY */
	if (LIST_FIRST(&sc->sc_mii.mii_phys) == NULL) {
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_PHY_CTRL1_REG,
		    GEC_PHY_CTRL1_INTERNAL_PHY_SEL |
		    GEC_PHY_CTRL1_AN_EN);
		mii_attach(self, &sc->sc_mii, 0xffffffff, 0,
		    MII_OFFSET_ANY, 0);
	}

	if (LIST_FIRST(&sc->sc_mii.mii_phys) == NULL) {
		aprint_error_dev(self, "no PHY found on MII\n");
		ifmedia_add(&sc->sc_mii.mii_media, IFM_ETHER | IFM_MANUAL,
		    0, NULL);
		ifmedia_set(&sc->sc_mii.mii_media, IFM_ETHER | IFM_MANUAL);
	} else {
		ifmedia_set(&sc->sc_mii.mii_media, IFM_ETHER | IFM_AUTO);
	}

	if_attach(ifp);
	ether_ifattach(ifp, sc->sc_enaddr);

#ifdef notyet
	if (!pmf_device_register(self, NULL, gec_resume))
		aprint_error_dev(self, "couldn't establish power handler\n");
	else
		pmf_class_network_register(self, ifp);
#endif

#if NRND > 0
	rnd_attach_source(&sc->sc_rnd_source, device_xname(sc->sc_dev),
	    RND_TYPE_NET, 0);
#endif
	return;

 failure:
	return;
}

static int
gec_mib_intr(void *arg)
{
	struct gec_softc *sc = arg;
	uint32_t status;

	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_INT_STATUS_REG);

#ifdef DEBUG_GEC
	if (status & (GEC_INT_MIB_COUNT_TH | GEC_INT_TX_FIFO_UNDERRUN)) {
		char tmpbuf[128];
		snprintb(tmpbuf, sizeof(tmpbuf), 
		    "\20"
		    "\5MAGIC_PKT_REC"
		    "\4MIB_COUNT_TH"
		    "\3PORT_STATUS_CHG"
		    "\2RX_FIFO_FULL"
		    "\1TX_FIFO_UNDERRUN",
		    status);
		printf("%s:%d: sc=%p, status=%s\n", __func__, __LINE__, sc, tmpbuf);

		if (status & GEC_INT_MIB_COUNT_TH)
			gec_clear_mib_counter(sc);
	}
#endif

	if (status & GEC_INT_RX_FIFO_FULL)
		printf("%s: RX FIFO FULL\n", device_xname(sc->sc_dev));

	if (status & GEC_INT_PORT_STATUS_CHG) {
		uint32_t st;

		st = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_PHY_CTRL1_REG);
#ifdef DEBUG_GEC_DUMP
		printf("%s: status=%08x\n", device_xname(sc->sc_dev), st);
#endif
	}

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_INT_STATUS_REG, status);
	return 1;
}

static int
gec_tx_intr(void *arg)
{
	struct gec_softc *sc;
	struct ifnet *ifp;
	struct gec_txsoft *txs;
	int idx;
	uint32_t status;

	sc = (struct gec_softc *)arg;
	ifp = &sc->sc_ethercom.ec_if;

	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_INT_STATUS_REG);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_INT_STATUS_REG, status);

#ifdef DEBUG_GEC
	if (status) {
		char tmpbuf[128];
		snprintb(tmpbuf, sizeof(tmpbuf), 
		    "\20"
		    "\5MAGIC_PKT_REC"
		    "\4MIB_COUNT_TH"
		    "\3PORT_STATUS_CHG"
		    "\2RX_FIFO_FULL"
		    "\1TX_FIFO_UNDERRUN",
		    status);
		printf("%s:%d: sc=%p, status=%s\n", __func__, __LINE__, sc, tmpbuf);

		printf("%s:%d: RX_DMA_CTRL=%d\n", __func__, __LINE__,
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_RX_DMA_CTRL_REG));
		printf("%s:%d: RX_DPTR-idx=%lu, readidx=%d\n", __func__, __LINE__,
		    RX_DPTR2IDX(bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_RX_DPTR_REG)),
		    sc->sc_rx_readidx);
	}
#endif

	for (idx = sc->sc_tx_considx; idx != sc->sc_tx_prodidx;
	    idx = GEC_TX_NEXTIDX(idx)) {

		txs = &sc->sc_txsoft[idx];

#ifdef DEBUG_GEC_DUMP
		gec_dump_txdesc(sc, idx, "TX_INTR:");
#endif

		TXDESC_READIN(idx);
		if (!(sc->sc_txdesc_ring[idx].tx_ctrl & GEC_TXDESC_CTRL_COWN)) {
			/* This TX Descriptor has not been transmitted yet? */
			break;
		}

		/* txsoft is used only first segment */
		if (sc->sc_txdesc_ring[idx].tx_ctrl & GEC_TXDESC_CTRL_FS) {
			bus_dmamap_unload(sc->sc_dmat,
			    txs->txs_dmamap);
			m_freem(txs->txs_mbuf);

			ifp->if_opackets++;
		}
		sc->sc_tx_free++;
	}
	sc->sc_tx_considx = idx;

#ifdef DEBUG_GEC_DUMP
	printf("TX_INTR:tx_free=%d\n", sc->sc_tx_free);
#endif

	if (sc->sc_tx_free > GEC_TX_RING_MINIMAL_CNT)
		ifp->if_flags &= ~IFF_OACTIVE;

	/*
	 * No more pending TX descriptor,
	 * cancel the watchdog timer.
	 */
	if (sc->sc_tx_free == GEC_TX_RING_CNT)
		ifp->if_timer = 0;

#if 0
	gec_update_mib(sc);
#else
	(void)gec_update_mib;
#endif

	return 1;
}

static int
gec_rx_intr(void *arg)
{
	struct gec_softc *sc;
	struct ifnet *ifp;
	struct gec_rxsoft *rxs;
	int idx, len;
	uint32_t status, ctrl;
	struct mbuf *m;

	sc = (struct gec_softc *)arg;
	ifp = &sc->sc_ethercom.ec_if;

	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_INT_STATUS_REG);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_INT_STATUS_REG, status);

#ifdef DEBUG_GEC
	if (status) {
		char tmpbuf[128];
		snprintb(tmpbuf, sizeof(tmpbuf), 
		    "\20"
		    "\5MAGIC_PKT_REC"
		    "\4MIB_COUNT_TH"
		    "\3PORT_STATUS_CHG"
		    "\2RX_FIFO_FULL"
		    "\1TX_FIFO_UNDERRUN",
		    status);
		printf("%s:%d: sc=%p, status=%s, rx_readidx=%d\n", __func__, __LINE__, sc, tmpbuf, sc->sc_rx_readidx);
	}
#endif

	for (idx = sc->sc_rx_readidx; RX_DPTR2IDX(bus_space_read_4(sc->sc_iot,
	    sc->sc_ioh, GEC_RX_DPTR_REG)) != idx; idx = GEC_RX_NEXTIDX(idx)) {

		rxs = &sc->sc_rxsoft[idx];

		RXDESC_READIN(idx);
		ctrl = sc->sc_rxdesc_ring[idx].rx_ctrl;
		if ((ctrl & GEC_RXDESC_CTRL_COWN) == 0)
			continue;

		/*
		 * build mbuf from RX Descriptor if needed
		 */
		m = rxs->rxs_mbuf;
		if (m == NULL) {
			/* mbuf empty? allocate new mbuf cluster */
			gec_alloc_rxbuf(sc, idx);
			m = rxs->rxs_mbuf;
			if (m == NULL) {
				printf("%s: mbuf allocation failed."
				    " dropping packet. FATAL ERROR!\n",
				    device_xname(sc->sc_dev));
				continue;
			}
		}

		if (ctrl & (GEC_RXDESC_CTRL_OSIZE | GEC_RXDESC_CTRL_CRCE)) {
			printf("rxdesc[%d] OSIZE or CRCE. skip\n", idx);
			ifp->if_ierrors++;
			goto rx_failure;
		}

		len = ctrl & GEC_RXDESC_CTRL_SDL_MASK;
		if (len < ETHER_HDR_LEN) {
			ifp->if_ierrors++;
			goto rx_failure;
		}

		/* packet receive ok */
		ifp->if_ipackets++;
		m->m_pkthdr.rcvif = ifp;
		m->m_pkthdr.len = m->m_len = len;

		/* checksum offloading */
		if (ifp->if_csum_flags_rx & M_CSUM_IPv4) {
			if ((ctrl & GEC_RXDESC_CTRL_PROT_MASK) !=
			    GEC_RXDESC_CTRL_PROT_OTHERS) {
				/* IP packet (including TCP and UDP) */
				m->m_pkthdr.csum_flags |= M_CSUM_IPv4;
				if (ctrl & GEC_RXDESC_CTRL_IPF)
					m->m_pkthdr.csum_flags |= M_CSUM_IPv4_BAD;
			}
		}
		if (ifp->if_csum_flags_rx & M_CSUM_TCPv4) {
			if ((ctrl & GEC_RXDESC_CTRL_PROT_MASK) ==
			    GEC_RXDESC_CTRL_PROT_TCP) {
				m->m_pkthdr.csum_flags |= M_CSUM_TCPv4;
				if (ctrl & GEC_RXDESC_CTRL_L4F)
					m->m_pkthdr.csum_flags |= M_CSUM_TCP_UDP_BAD;
			}
		}
		if (ifp->if_csum_flags_rx & M_CSUM_UDPv4) {
			if ((ctrl & GEC_RXDESC_CTRL_PROT_MASK) ==
			    GEC_RXDESC_CTRL_PROT_UDP) {
				m->m_pkthdr.csum_flags |= M_CSUM_UDPv4;
				if (ctrl & GEC_RXDESC_CTRL_L4F)
					m->m_pkthdr.csum_flags |= M_CSUM_TCP_UDP_BAD;
			}
		}

		bus_dmamap_sync(sc->sc_dmat, rxs->rxs_dmamap, 0,
		    rxs->rxs_dmamap->dm_mapsize, BUS_DMASYNC_PREREAD);

// XXX
{
	uint8_t *p;
	p = mtod(m, uint8_t *);

	if ((p[12] != 0x08) || (p[13] != 0x00)) {
		gec_dump_rxdesc(sc, idx, "XXX ");
	}
}

		/* Pass this up to any BPF listeners. */
		bpf_mtap(ifp, m);

		(*ifp->if_input)(ifp, m);

		/* clear this rxsoft */
		bus_dmamap_unload(sc->sc_dmat, rxs->rxs_dmamap);
		rxs->rxs_mbuf = NULL;

		/* allocate new mbuf cluster or reuse old mbuf*/
		gec_alloc_rxbuf(sc, idx);
		continue;

	 rx_failure:
		gec_reset_rxdesc(sc, idx);
	}
	sc->sc_rx_readidx = idx;

	if (gec_stall_check(sc, idx)) {
		/* possibility of stalling... */
		callout_schedule(&sc->sc_rx_ch, hz / 1000);
	}

	/* re-enable RX DMA */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_RX_DMA_CTRL_REG,
	    GEC_RX_DMA_CTRL_RX_EN);
	return 1;
}

static void
gec_stall_recover(void *arg)
{
	struct gec_softc *sc;
	struct ifnet *ifp;

	sc = (struct gec_softc *)arg;
	ifp = &sc->sc_ethercom.ec_if;

	if (ifp->if_flags & IFF_RUNNING) {
		if (gec_stall_check(sc, sc->sc_rx_readidx)) {
			/* detect stalling... */
			gec_reset_rxdesc(sc, sc->sc_rx_readidx);

			/* re-enable RX */
			bus_space_write_4(sc->sc_iot, sc->sc_ioh,
			    GEC_RX_DMA_CTRL_REG, 1);

			printf("%s: RX Ring stalled. recovered\n",
			    device_xname(sc->sc_dev));
		}
	}
}

static int
gec_stall_check(struct gec_softc *sc, int idx)
{
	if (RX_DPTR2IDX(bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    GEC_RX_DPTR_REG)) == idx) {

		RXDESC_READIN(idx);
		if (sc->sc_rxdesc_ring[idx].rx_ctrl & GEC_RXDESC_CTRL_COWN) {
			return 1;
		}
	}
	return 0;
}


static void
gec_clear_hashtable(struct gec_softc *sc)
{
	memset(sc->sc_hashtable, 0, sizeof(sc->sc_hashtable));
}

static void
gec_add_hashtable(struct gec_softc *sc, uint8_t *eaddr)
{
	uint32_t hashaddr;

#ifdef GEC_ARL_HASHTABLE_USE_CRC
	/*
	 * MAC Address Hashing Algorithm
	 * using 32 bit CRC [8:0] of DA as hashing address
	 */
	hashaddr = ether_crc32_be(eaddr, ETHER_ADDR_LEN) & 0x1ff;
#else
	/*
	 * MAC Address Hashing Algorithm
	 * using DA last 9-bit [40, 7-0] as hashing address
	 */
	hashaddr = eaddr[5] + ((eaddr[0] & 1) ? 0x100 : 0);
#endif
	sc->sc_hashtable[hashaddr / 8] |= (1 << (hashaddr & 7));
}

static void
gec_apply_hashtable(struct gec_softc *sc)
{
	uint32_t hashaddr;
	int hashbit;

	/*
	 * program hash table
	 */
	for (hashaddr = 0; hashaddr < GEC_HASH_CTRL_HASHBITSIZE; hashaddr++) {
		hashbit = (sc->sc_hashtable[hashaddr / 8] &
		    (1 << (hashaddr & 7))) ? 1 : 0;
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_HASH_CTRL_REG,
		    GEC_HASH_CTRL_CMD_START |
		    GEC_HASH_CTRL_CMD_WRITE |
		    GEC_HASH_CTRL_HASHBIT(hashbit) |
		    GEC_HASH_CTRL_HASHADDR(hashaddr));

		/* wait complete */
		while (bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    GEC_HASH_CTRL_REG) & GEC_HASH_CTRL_CMD_START)
			;
	}
}

static void
gec_setmulti(struct gec_softc *sc)
{
	struct ifnet *ifp;
	struct ether_multi *enm;
	struct ether_multistep step;

	ifp = &sc->sc_ethercom.ec_if;
	if (ifp->if_flags & IFF_PROMISC) {
 allmulti:
		ifp->if_flags |= IFF_ALLMULTI;
		return;
	}

	gec_clear_hashtable(sc);
	ifp->if_flags &= ~IFF_ALLMULTI;

	ETHER_FIRST_MULTI(step, &sc->sc_ethercom, enm);
	while (enm != NULL) {
		/*
		 * If multicast range, fall back to ALLMULTI.
		 */
		if (memcmp(enm->enm_addrlo, enm->enm_addrhi,
		    ETHER_ADDR_LEN) != 0)
			goto allmulti;

		gec_add_hashtable(sc, enm->enm_addrlo);
		ETHER_NEXT_MULTI(step, enm);
	}

	gec_apply_hashtable(sc);
}

/*
 * ifnet interfaces
 */
static int
gec_init(struct ifnet *ifp)
{
	struct gec_softc *sc;
	int s, error = 0;

	sc = ifp->if_softc;

#ifdef DEBUG_GEC_DUMP
	printf("%s:%d: ifp=%p\n", __func__, __LINE__, ifp);
#endif

	s = splnet();

	gec_init_regs(sc);
	gec_init_txring(sc);
	error = gec_init_rxring(sc);
	if (error != 0) {
		gec_drain_rxbuf(sc);
		printf("%s: Cannot allocate mbuf cluster\n", device_xname(sc->sc_dev));
		goto init_failure;
	}

	/* reload mac address */
	memcpy(sc->sc_enaddr, CLLADDR(ifp->if_sadl), ETHER_ADDR_LEN);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_MY_MAC_H_REG,
	    (sc->sc_enaddr[0] << 8) | sc->sc_enaddr[1]);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_MY_MAC_L_REG,
	    (sc->sc_enaddr[2] << 24) | (sc->sc_enaddr[3] << 16) |
	    (sc->sc_enaddr[4] << 8) | sc->sc_enaddr[5]);

	/* program multicast address */
	gec_setmulti(sc);

	/* update if_flags */
	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;

	/* update local copy of if_flags */
	sc->sc_if_flags = ifp->if_flags;

	/* mii */
	mii_mediachg(&sc->sc_mii);

	/* start RX DMA */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_RX_DMA_CTRL_REG,
	    GEC_RX_DMA_CTRL_RX_EN);

 init_failure:
	splx(s);
	return error;
}

static void
gec_start(struct ifnet *ifp)
{
	struct gec_softc *sc;
	struct mbuf *m;
	int npkt;

	sc = ifp->if_softc;

#ifdef DEBUG_GEC_DUMP
	printf("%s:%d: ifp=%p, sc=%p\n", __func__, __LINE__, ifp, sc);
#endif

	if ((ifp->if_flags & (IFF_RUNNING | IFF_OACTIVE)) != IFF_RUNNING)
		return;

	for (npkt = 0; ; npkt++) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

		if (sc->sc_tx_free <= GEC_TX_RING_MINIMAL_CNT) {
			/* no tx descriptor now... */
			ifp->if_flags |= IFF_OACTIVE;
#ifdef DEBUG_GEC
	printf("%s:%d: TX Queue full\n", __func__, __LINE__);
#endif
			break;
		}

		IFQ_DEQUEUE(&ifp->if_snd, m);

		if (gec_encap(sc, m) != 0) {
			/* mbuf has too many chains */
			ifp->if_flags |= IFF_OACTIVE;
#ifdef DEBUG_GEC
			printf("%s:%d: TX Queue overflow. dropping packet\n", __func__, __LINE__);
#endif
			m_freem(m);
			ifp->if_oerrors++;
			break;
		}

		/*
		 * If there's a BPF listener, bounce a copy of this frame
		 * to him.
		 */
		bpf_mtap(ifp, m);
	}

	if (npkt) {
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_TX_DMA_CTRL_REG,
		    GEC_TX_DMA_CTRL_TX_EN);

		ifp->if_timer = 5;
	}
}

static void
gec_stop(struct ifnet *ifp, int disable)
{
	struct gec_softc *sc;
	int s;

	sc = ifp->if_softc;

#ifdef DEBUG_GEC_DUMP
	printf("%s:%d: ifp=%p, disable=%d(do drain)\n", __func__, __LINE__, ifp, disable);
#endif

	s = splnet();

	/* suspend DMA transfer */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_DMA_CFG_REG,
	    GEC_DMA_CFG_RX_OFFSET_2B_DIS |
	    GEC_DMA_CFG_TX_SUSPEND |
	    GEC_DMA_CFG_RX_SUSPEND);

	/* stop DMA transfer */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_TX_DMA_CTRL_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_RX_DMA_CTRL_REG, 0);

	callout_stop(&sc->sc_rx_ch);

	/* Mark the interface as down and cancel the watchdog timer. */
	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
	ifp->if_timer = 0;

	if (disable) {
		gec_drain_txbuf(sc);
		gec_drain_rxbuf(sc);
	}

	splx(s);

}

static void
gec_watchdog(struct ifnet *ifp)
{
	struct gec_softc *sc;
	int s;
	uint32_t swrst_bit;

	sc = ifp->if_softc;
	s = splnet();

	printf("%s: watchdog timeout\n", device_xname(sc->sc_dev));
	ifp->if_oerrors++;

#if 0
	{
		int i;

		printf("%s:%d: TX_DMA_CTRL=%08x, DMA_CFG_REG=%08x\n",
		    __func__, __LINE__,
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_TX_DMA_CTRL_REG),
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_DMA_CFG_REG));

		printf("%s:%d: %d, %d, %d, %d, %d, %d, %d\n", __func__, __LINE__,
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXRUNT_REG),
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXLONG_REG),
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXDROP_REG),
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXCRC_REG),
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXARLDROP_REG),
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXVLANDROP_REG),
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXCSERR_REG));

		printf("%s:%d: TX_DPTR-idx=%lu, considx=%d, prodidx=%d, tx_free=%d\n",
		    __func__, __LINE__,
		    TX_DPTR2IDX(bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_TX_DPTR_REG)),
		    sc->sc_tx_considx,
		    sc->sc_tx_prodidx,
		    sc->sc_tx_free);

		for (i = 0; i < GEC_TX_RING_CNT; i++) {
			gec_dump_txdesc(sc, i, "gec_watchdog:");
		}
	}
#endif

	gec_tx_intr(sc);
	gec_rx_intr(sc);

	gec_stop(ifp, 1);

	/* force power off NIC and PHY */
	swrst_bit = STAR_REG_READ32(EQUULEUS_CLKPWR_SOFTRST_REG);
	swrst_bit &= ~EQUULEUS_CLKPWR_SOFTRST_FE_PHY;
	swrst_bit &= ~EQUULEUS_CLKPWR_SOFTRST_NIC;
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_SOFTRST_REG, swrst_bit);
	gec_init(ifp);

	splx(s);
}

static int
gec_ioctl(struct ifnet *ifp, u_long command, void *data)
{
	struct gec_softc *sc;
	struct ifreq *ifr;
	int change, s, error;

	sc = ifp->if_softc;
	ifr = data;

	error = 0;
	s = splnet();

	switch (command) {
	case SIOCSIFFLAGS:
		if ((error = ifioctl_common(ifp, command, data)) != 0)
			break;

		change = ifp->if_flags ^ sc->sc_if_flags;

		/* up or down */
		if (change & IFF_UP) {
			if (ifp->if_flags & IFF_UP) {
#ifdef DEBUG_GEC_DUMP
				printf("%s:%d: NOW %s: DOWN -> UP\n", __func__, __LINE__, ifp->if_flags & IFF_RUNNING ? "RUNNING" : "STOPPED");
#endif
				/* down -> up */
				if ((ifp->if_flags & IFF_RUNNING) == 0)
					gec_init(ifp);
			} else {
#ifdef DEBUG_GEC_DUMP
				printf("%s:%d: NOW %s: UP -> DOWN\n", __func__, __LINE__, ifp->if_flags & IFF_RUNNING ? "RUNNING" : "STOPPED");
#endif
				/* up -> down */
				if ((ifp->if_flags & IFF_RUNNING) != 0)
					gec_stop(ifp, 1);
			}
		}

		/* into promisc mode or not */
		if ((change & (IFF_PROMISC | IFF_ALLMULTI)) != 0) {
			if ((ifp->if_flags & (IFF_PROMISC | IFF_ALLMULTI)) != 0) {
				GEC_REG_SETBIT(sc, GEC_ARL_CFG_REG,
				    GEC_ARL_CFG_MISC_MODE);
			} else {
				GEC_REG_CLRBIT(sc, GEC_ARL_CFG_REG,
				    GEC_ARL_CFG_MISC_MODE);
			}
			gec_setmulti(sc);
		}

#ifdef DEBUG_GEC_DUMP
		if (change & IFF_LINK0)
			gec_dump_hashtable(sc);
#endif

		sc->sc_if_flags = ifp->if_flags;
		error = 0;
		break;
	case SIOCSIFMEDIA:
	case SIOCGIFMEDIA:
#if notyet
		/* Flow control requires full-duplex mode. */
		if (IFM_SUBTYPE(ifr->ifr_media) == IFM_AUTO ||
		    (ifr->ifr_media & IFM_FDX) == 0)
			ifr->ifr_media &= ~IFM_ETH_FMASK;
		if (IFM_SUBTYPE(ifr->ifr_media) != IFM_AUTO) {
			if ((ifr->ifr_media & IFM_ETH_FMASK) == IFM_FLOW) {
				/* We can do both TXPAUSE and RXPAUSE. */
				ifr->ifr_media |=
				    IFM_ETH_TXPAUSE | IFM_ETH_RXPAUSE;
			}
			sc->sc_flowflags = ifr->ifr_media & IFM_ETH_FMASK;
		}
#endif
		error = ifmedia_ioctl(ifp, ifr, &sc->sc_mii.mii_media, command);
		break;
	default:
		error = ether_ioctl(ifp, command, data);
		if (error != ENETRESET)
			break;

		error = 0;

		if (command == SIOCSIFCAP)
			error = (*ifp->if_init)(ifp);
		else if (command != SIOCADDMULTI && command != SIOCDELMULTI)
			;
		else if (ifp->if_flags & IFF_RUNNING) {
			/*
			 * Multicast list has changed; set the hardware filter
			 * accordingly.
			 */
			gec_setmulti(sc);
		}
		break;
	}

	splx(s);

	return error;
}

/*
 * for MII
 */
static int
gec_miibus_readreg(device_t dev, int phy, int reg)
{
	struct gec_softc *sc;
	struct ifnet *ifp;
	int timeout;
	uint32_t val, status;

	sc = device_private(dev);
	ifp = &sc->sc_ethercom.ec_if;

	/* read command */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_PHY_CTRL0_REG,
	    GEC_PHY_CTRL0_RW_OK | GEC_PHY_CTRL0_RD_CMD |
	    GEC_PHY_CTRL0_PHY_REG(reg) | GEC_PHY_CTRL0_PHY_ADDR(phy));

	for (timeout = 5000; timeout > 0; --timeout) {
		status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_PHY_CTRL0_REG);
		if (status & GEC_PHY_CTRL0_RW_OK)
			break;
	}
	if (timeout <= 0) {
		aprint_error_ifnet(ifp, "MII read timeout: reg=0x%02x\n", reg);
		val = -1;
	} else {
		val = (status >> 16) & 0xffff;
	}

#ifdef DEBUG_GEC_DUMP
	printf("%s:%d: phy=%d, reg=%d => val=%08x (status=%08x)\n", __func__, __LINE__, phy, reg, val, status);
#endif

	return val;
}

static void
gec_miibus_writereg(device_t dev, int phy, int reg, int val)
{
	struct gec_softc *sc;
	struct ifnet *ifp;
	int timeout;
	uint32_t status;

	sc = device_private(dev);
	ifp = &sc->sc_ethercom.ec_if;

#ifdef DEBUG_GEC_DUMP
	printf("%s:%d: phy=%d, reg=%d, val=0x%08x\n", __func__, __LINE__, phy, reg, val);
#endif

	/* write command */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_PHY_CTRL0_REG,
	    GEC_PHY_CTRL0_RW_OK | GEC_PHY_CTRL0_WT_CMD |
	    GEC_PHY_CTRL0_PHY_REG(reg) | GEC_PHY_CTRL0_PHY_ADDR(phy) |
	    ((val & 0xffff) << 16));

	for (timeout = 5000; timeout > 0; --timeout) {
		status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_PHY_CTRL0_REG);
		if (status & GEC_PHY_CTRL0_RW_OK)
			break;
	}
	if (timeout <= 0)
		aprint_error_ifnet(ifp, "MII write timeout: reg=0x%02x\n", reg);
}

static void
gec_miibus_statchg(device_t dev)
{
	struct gec_softc *sc;
	struct mii_data *mii;
	struct ifmedia_entry *ife;
	uint32_t status, status0;

	sc = device_private(dev);
	mii = &sc->sc_mii;
	ife = mii->mii_media.ifm_cur;

	status0 = status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_PHY_CTRL1_REG);

	status |= GEC_PHY_CTRL1_AUTO_POLL_DIS;
	status &= ~GEC_PHY_CTRL1_FORCE_SPEED_MASK;
	status &= ~GEC_PHY_CTRL1_AN_EN;

	if ((ife->ifm_media & IFM_GMASK) == IFM_FDX)
		status |= GEC_PHY_CTRL1_FORCE_DUPLEX;
	else
		status &= ~GEC_PHY_CTRL1_FORCE_DUPLEX;

	switch (IFM_SUBTYPE(ife->ifm_media)) {
	case IFM_AUTO:
		status &= ~GEC_PHY_CTRL1_AUTO_POLL_DIS;
		status |= GEC_PHY_CTRL1_AN_EN;
		break;
	case IFM_1000_T:
		status |= GEC_PHY_CTRL1_FORCE_SPEED(2);
		status &= ~GEC_PHY_CTRL1_FORCE_DUPLEX;
		break;
	case IFM_100_TX:
		status |= GEC_PHY_CTRL1_FORCE_SPEED(1);	/* XXX: notwork??? */
		break;
	case IFM_10_T:
		status |= GEC_PHY_CTRL1_FORCE_SPEED(0);
		break;
	default:
		printf("%s: unknown media type: %x\n",
		    device_xname(sc->sc_dev),
		    IFM_SUBTYPE(ife->ifm_media));
		status = status0;
		break;
	}

	if (status != status0) {
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_PHY_CTRL1_REG,
		    status);
	}
}

/*
 * misc routines
 */
static void
gec_init_txring(struct gec_softc *sc)
{
	int i;

	sc->sc_tx_free = GEC_TX_RING_CNT;
	sc->sc_tx_considx = 0;
	sc->sc_tx_prodidx = 0;

	/*
	 * build TX ring
	 */
	for (i = 0; i < GEC_TX_RING_CNT; i++) {
		sc->sc_txdesc_ring[i].tx_sdp = 0;
		sc->sc_txdesc_ring[i].tx_ctrl =
		    GEC_TXDESC_CTRL_COWN |
		    ((i == (GEC_TX_RING_CNT - 1)) ? GEC_TXDESC_CTRL_EOR : 0);
		sc->sc_txdesc_ring[i].tx_vlan = 0;
		sc->sc_txdesc_ring[i].tx_rsvd = 0;
		TXDESC_WRITEOUT(i);
	}
}

static int
gec_init_rxring(struct gec_softc *sc)
{
	int i, error;

	/*
	 * build RX ring
	 */
	for (i = 0; i < GEC_RX_RING_CNT; i++) {
		error = gec_alloc_rxbuf(sc, i);
		if (error != 0)
			return error;
	}

	sc->sc_rx_readidx = 0;

	return 0;
}

static void
gec_reset_rxdesc(struct gec_softc *sc, int idx)
{
	sc->sc_rxdesc_ring[idx].rx_sdp =
	    sc->sc_rxsoft[idx].rxs_dmamap->dm_segs[0].ds_addr +
	    ETHER_ALIGN;
	sc->sc_rxdesc_ring[idx].rx_vlan = 0;
	sc->sc_rxdesc_ring[idx].rx_rsvd = 0;
	sc->sc_rxdesc_ring[idx].rx_ctrl =
	    ((idx == GEC_RX_RING_CNT - 1) ? GEC_RXDESC_CTRL_EOR : 0) |
	    GEC_RXDESC_CTRL_SDL(sc->sc_rxsoft[idx].rxs_dmamap->dm_mapsize -
	    ETHER_ALIGN);
	RXDESC_WRITEOUT(idx);
}

static int
gec_alloc_rxbuf(struct gec_softc *sc, int idx)
{
	struct mbuf *m;
	int error;

	KASSERT((idx >= 0) && (idx < GEC_RX_RING_CNT));

	/* free mbuf if already allocated */
	if (sc->sc_rxsoft[idx].rxs_mbuf != NULL) {
		bus_dmamap_unload(sc->sc_dmat, sc->sc_rxsoft[idx].rxs_dmamap);
		m_freem(sc->sc_rxsoft[idx].rxs_mbuf);
		sc->sc_rxsoft[idx].rxs_mbuf = NULL;
	}

	/* allocate new mbuf cluster */
	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m == NULL)
		return ENOBUFS;
	MCLGET(m, M_DONTWAIT);
	if (!(m->m_flags & M_EXT)) {
		m_freem(m);
		return ENOBUFS;
	}
	error = bus_dmamap_load(sc->sc_dmat, sc->sc_rxsoft[idx].rxs_dmamap,
	    m->m_ext.ext_buf, m->m_ext.ext_size, NULL,
	    BUS_DMA_READ | BUS_DMA_NOWAIT);
	if (error)
		return error;

#if ETHER_ALIGN != 0
	/* for 32bit aligned on IP header; 2(stub) + ETHER_HDR_LEN = 4*n */
	m_adj(m, ETHER_ALIGN);
#endif
	sc->sc_rxsoft[idx].rxs_mbuf = m;

	gec_reset_rxdesc(sc, idx);
	return 0;
}

static void
gec_drain_txbuf(struct gec_softc *sc)
{
	int idx;
	struct gec_txsoft *txs;
	struct ifnet *ifp;

	ifp = &sc->sc_ethercom.ec_if;

	for (idx = sc->sc_tx_considx; idx != sc->sc_tx_prodidx;
	    idx = GEC_TX_NEXTIDX(idx)) {

		/* txsoft is used only first segment */
		txs = &sc->sc_txsoft[idx];
		TXDESC_READIN(idx);
		if (sc->sc_txdesc_ring[idx].tx_ctrl & GEC_TXDESC_CTRL_FS) {
			bus_dmamap_unload(sc->sc_dmat,
			    txs->txs_dmamap);
			m_freem(txs->txs_mbuf);

			ifp->if_oerrors++;
		}
		sc->sc_tx_free++;
	}
}

static void
gec_drain_rxbuf(struct gec_softc *sc)
{
	int i;

#ifdef DEBUG_GEC_DUMP
	printf("%s:%d\n", __func__, __LINE__);
#endif

	for (i = 0; i < GEC_RX_RING_CNT; i++) {
		if (sc->sc_rxsoft[i].rxs_mbuf != NULL) {
			bus_dmamap_unload(sc->sc_dmat, sc->sc_rxsoft[i].rxs_dmamap);
			m_freem(sc->sc_rxsoft[i].rxs_mbuf);
			sc->sc_rxsoft[i].rxs_mbuf = NULL;
		}
	}
}

static int
gec_alloc_ring(struct gec_softc *sc)
{
	int i, error;

	/*
	 * build DMA maps for TX.
	 * TX descriptor must be able to contain mbuf chains,
	 * So, make up GEC_MAX_PKT_NSEGS dmamap.
	 */
	for (i = 0; i < GEC_TX_RING_CNT; i++) {
		error = bus_dmamap_create(sc->sc_dmat, GEC_MAX_PKT_LEN,
		    GEC_MAX_PKT_NSEGS, GEC_MAX_PKT_LEN, 0, BUS_DMA_NOWAIT,
		    &sc->sc_txsoft[i].txs_dmamap);
		if (error) {
			aprint_error_dev(sc->sc_dev,
			    "can't create DMA map for TX descs\n");
			goto fail_1;
		}
	}

	/*
	 * build DMA maps for RX.
	 * RX descripter contains An mbuf cluster,
	 * and make up a dmamap.
	 */
	for (i = 0; i < GEC_RX_RING_CNT; i++) {
		error = bus_dmamap_create(sc->sc_dmat, MCLBYTES,
		    1, MCLBYTES, 0, BUS_DMA_NOWAIT,
		    &sc->sc_rxsoft[i].rxs_dmamap);
		if (error) {
			aprint_error_dev(sc->sc_dev,
			    "can't create DMA map for RX descs\n");
			goto fail_2;
		}
	}

	if (gec_alloc_dma(sc, sizeof(struct gec_txdesc) * GEC_TX_RING_CNT,
	    (void **)&(sc->sc_txdesc_ring), &(sc->sc_txdesc_dmamap)) != 0)
		return -1;
	memset(sc->sc_txdesc_ring, 0, sizeof(struct gec_txdesc) * GEC_TX_RING_CNT);

	if (gec_alloc_dma(sc, sizeof(struct gec_rxdesc) * GEC_RX_RING_CNT,
	    (void **)&(sc->sc_rxdesc_ring), &(sc->sc_rxdesc_dmamap)) != 0)
		return -1;
	memset(sc->sc_rxdesc_ring, 0, sizeof(struct gec_rxdesc) * GEC_RX_RING_CNT);

	if (gec_alloc_dma(sc, GEC_ETHER_MIN_LEN_VLAN,
	    (void **)&(sc->sc_padseg), &(sc->sc_padseg_dmamap)) != 0)
		return -1;
	memset(sc->sc_padseg, 0, GEC_ETHER_MIN_LEN_VLAN);

	return 0;

 fail_2:
	for (i = 0; i < GEC_RX_RING_CNT; i++) {
		if (sc->sc_rxsoft[i].rxs_dmamap != NULL)
			bus_dmamap_destroy(sc->sc_dmat,
			    sc->sc_rxsoft[i].rxs_dmamap);
	}
 fail_1:
	for (i = 0; i < GEC_TX_RING_CNT; i++) {
		if (sc->sc_txsoft[i].txs_dmamap != NULL)
			bus_dmamap_destroy(sc->sc_dmat,
			    sc->sc_txsoft[i].txs_dmamap);
	}
	return error;
}

static int
gec_encap(struct gec_softc *sc, struct mbuf *m)
{
	int idx, i, error;
	int padsize;
	bus_dmamap_t map;
	uint32_t ctrl, dmastat;
	int csumflags;

	idx = sc->sc_tx_prodidx;
	map = sc->sc_txsoft[idx].txs_dmamap;

	csumflags = m->m_pkthdr.csum_flags;

	error = bus_dmamap_load_mbuf(sc->sc_dmat, map, m,
	    BUS_DMA_NOWAIT);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "Error mapping mbuf into TX chain: error=%d\n", error);
		m_freem(m);
		return error;
	}

	if (map->dm_nsegs > sc->sc_tx_free - GEC_TX_RING_MINIMAL_CNT) {
		bus_dmamap_unload(sc->sc_dmat, map);
		return ENOBUFS;
	}

#ifdef DEBUG_GEC_DUMP
	{
		uint8_t *p;
		int len = m_length(m);
		struct mbuf *tmp_m= m;

		printf("TX_INTR: m_flags=%08x, length=%d: ", m->m_flags, len);
		while (tmp_m != NULL) {
			printf("<paddr=0x%08x ", (uint32_t)(tmp_m->m_paddr));
			printf("len=%d>", tmp_m->m_len);

			p = mtod(tmp_m, uint8_t *);
			len = tmp_m->m_len;
			for (i = 0; i < len; i++) {
				printf(" %02x", *p++);
			}
			tmp_m = tmp_m->m_next;
		}
		printf("\n");
	}
#endif

//	padsize = GEC_ETHER_MIN_LEN_VLAN - m->m_pkthdr.len;
	padsize = GEC_ETHER_MIN_LEN - m->m_pkthdr.len;

	/* suspend DMA transfer */
	dmastat = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_DMA_CFG_REG);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_DMA_CFG_REG,
	    dmastat | GEC_DMA_CFG_TX_SUSPEND);

	for (i = 0; i < map->dm_nsegs; i++) {
		ctrl = 0;
		if (i == 0) {
			/* mark first segment */
			ctrl |= GEC_TXDESC_CTRL_FS;
			sc->sc_txsoft[idx].txs_mbuf = m;

			/* checksum offloading */
			if (csumflags & M_CSUM_IPv4) {
				/* XXX: ip4csum-tx wouldn't work */
				printf("%s: ERROR: IP4CSUM-TX not work\n",
				    device_xname(sc->sc_dev));
				ctrl |= GEC_TXDESC_CTRL_ICO;
			}
			if (csumflags & M_CSUM_UDPv4)
				ctrl |= GEC_TXDESC_CTRL_UCO;
			if (csumflags & M_CSUM_TCPv4)
				ctrl |= GEC_TXDESC_CTRL_TCO;

#ifdef DEBUG_GEC_DUMP
			if (csumflags & (M_CSUM_IPv4 | M_CSUM_UDPv4 | M_CSUM_TCPv4)) {
				printf("m=%p, csumflags=%s%s%s\n",
				    m,
				    (csumflags & M_CSUM_IPv4) ? "I" : "",
				    (csumflags & M_CSUM_UDPv4) ? "U" : "",
				    (csumflags & M_CSUM_TCPv4) ? "T" : "");
			}
#endif
		}
		if ((padsize <= 0) && (i == map->dm_nsegs - 1)) {
			/* mark last segment */
			ctrl |= GEC_TXDESC_CTRL_LS | GEC_TXDESC_CTRL_INT;
		}
		if (idx == GEC_TX_RING_CNT - 1) {
			/* mark end of ring */
			ctrl |= GEC_TXDESC_CTRL_EOR;
		}

		sc->sc_txdesc_ring[idx].tx_vlan = 0;
		sc->sc_txdesc_ring[idx].tx_sdp = map->dm_segs[i].ds_addr;
#if 1	/* ? */
		TXDESC_WRITEOUT(idx);
#endif
		sc->sc_txdesc_ring[idx].tx_ctrl = ctrl |
		    GEC_TXDESC_CTRL_SDL(map->dm_segs[i].ds_len);
		TXDESC_WRITEOUT(idx);

#ifdef DEBUG_GEC_DUMP
		gec_dump_txdesc(sc, idx, "G_ENCAP         :");
#endif

		idx = GEC_TX_NEXTIDX(idx);
		sc->sc_tx_free--;
	}

	if (padsize > 0) {
		/* need padding */
		ctrl = GEC_TXDESC_CTRL_LS | GEC_TXDESC_CTRL_INT;
		if (idx == GEC_TX_RING_CNT - 1) {
			/* end of ring */
			ctrl |= GEC_TXDESC_CTRL_EOR;
		}
		sc->sc_txdesc_ring[idx].tx_vlan = 0;
		sc->sc_txdesc_ring[idx].tx_sdp = sc->sc_padseg_dmamap->dm_segs[0].ds_addr;
		TXDESC_WRITEOUT(idx);

		sc->sc_txdesc_ring[idx].tx_ctrl = ctrl |
		    GEC_TXDESC_CTRL_SDL(padsize);
		TXDESC_WRITEOUT(idx);
#ifdef DEBUG_GEC_DUMP
		gec_dump_txdesc(sc, idx, "G_ENCAP(padding):");
#endif

		idx = GEC_TX_NEXTIDX(idx);
		sc->sc_tx_free--;
	}

	bus_dmamap_sync(sc->sc_dmat, map, 0, map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);

	/* continue DMA transfer */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_DMA_CFG_REG,
	    dmastat & ~GEC_DMA_CFG_TX_SUSPEND);

#ifdef DEBUG_GEC_DUMP
	printf("tx_free=%d\n", sc->sc_tx_free);
#endif

	sc->sc_tx_prodidx = idx;

	return 0;
}


static int
gec_init_regs(struct gec_softc *sc)
{
	paddr_t paddr;
	uint32_t status;

	/* activate NIC and FE PHY */
	status = STAR_REG_READ32(EQUULEUS_CLKPWR_SOFTRST_REG);
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_SOFTRST_REG, status |
	    EQUULEUS_CLKPWR_SOFTRST_FE_PHY |
	    EQUULEUS_CLKPWR_SOFTRST_NIC);

#if 0
	/* GPIO_B PIN ENABLE */
	status = STAR_REG_READ32(EQUULEUS_MISC_GPIOB_PIN_REG);
	STAR_REG_WRITE32(EQUULEUS_MISC_GPIOB_PIN_REG, status | 3);

	/* XXX: CLKGATE */
	status = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKGATE0_REG);
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKGATE0_REG,
	    status |
	    EQUULEUS_CLKPWR_CLKGATE0_MAC_CLK |
	    EQUULEUS_CLKPWR_CLKGATE0_MDC_CLK |
	    EQUULEUS_CLKPWR_CLKGATE0_PCLK_NIC |
	    EQUULEUS_CLKPWR_CLKGATE0_HCLK_NIC);

	/* XXX: select NIC clock source */
	status = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKCTRL_REG);
//	status &= ~EQUULEUS_CLKPWR_CLKCTRL_NICCLK_SEL;
	status &= ~EQUULEUS_CLKPWR_CLKCTRL_MDC_DIV(0);
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKCTRL_REG, status);

	/* XXX */
	status = STAR_REG_READ32(EQUULEUS_CLKPWR_PLLCTRL_REG);
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_PLLCTRL_REG, status &
	    ~EQUULEUS_CLKPWR_PLLCTRL_PLLx5_PWD);

	status = STAR_REG_READ32(EQUULEUS_CLKPWR_PADCTRL_REG);
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_PADCTRL_REG,
	    status |
//	    EQUULEUS_CLKPWR_PADCTRL_MII_NOT_BOUNDED |
	    EQUULEUS_CLKPWR_PADCTRL_MII_SPEED);
#endif

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_MAC_CFG_REG,
	    GEC_MAC_CFG_TX_CKS_EN |
	    GEC_MAC_CFG_RX_CKS_EN |
	    GEC_MAC_CFG_ACPT_CKS_ERR |
	    GEC_MAC_CFG_VLAN_STRIP |
	 /* GEC_MAC_CFG_ACPT_CRC_ERR | */
	    GEC_MAC_CFG_CRC_STRIP |
	 /* GEC_MAC_CFG_ACPT_LONG_PKT | */
	    GEC_MAC_CFG_MAX_LEN(1) |
	    GEC_MAC_CFG_IPG(0x1f) |
	 /* GEC_MAC_CFG_DO_NO_SKIP | */
	 /* GEC_MAC_CFG_FAST_RETRY | */
	    0);

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_FC_CFG_REG,
	    GEC_FC_CFG_SEND_PAUSE_TH(0x3b0) |
	 /* GEC_FC_CFG_UC_PAUSE_DIS | */
	    GEC_FC_CFG_BP_ENABLE |
	    GEC_FC_CFG_MAX_BP_COL_EN |
	    GEC_FC_CFG_MAX_BP_COL_CNT(0x0c));

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_ARL_CFG_REG,
	    GEC_ARL_CFG_CPU_LEARN_DIS |
	    GEC_ARL_CFG_REV_MC_FILTER |
#ifdef GEC_ARL_HASHTABLE_USE_CRC
	    GEC_ARL_CFG_HASH_ALG |
#endif
	    0);

#if 0
	{
		/* check hash table */
		int i;
		for (i = 1000 * 5; i > 0; i--) {
			if (bus_space_read_4(sc->sc_iot, sc->sc_ioh,
			    GEC_HASH_CTRL_REG) & GEC_HASH_CTRL_BIST_DONE)
				break;
		}
		if (i <= 0) {
			printf("%s: internal test not complete\n",
			    device_xname(sc->sc_dev));
		} else {
			if ((bus_space_read_4(sc->sc_iot, sc->sc_ioh,
			    GEC_HASH_CTRL_REG) & GEC_HASH_CTRL_BIST_OK) == 0)
				printf("%s: hash table manufacture fault\n",
				    device_xname(sc->sc_dev));
		}
	}
#endif

	/* VLAN ID */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_VLAN_CTRL_REG, 0);

	/* clear TEST Register */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_TEST0_REG, 4);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_TEST1_REG, 0);


	/* TX Descriptor Physical Address */
	paddr = sc->sc_txdesc_dmamap->dm_segs[0].ds_addr;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_TX_DPTR_REG, paddr);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_TX_BASE_ADDR_REG, paddr);

	/* RX Descriptor Physical Address */
	paddr = sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_RX_DPTR_REG, paddr);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_RX_BASE_ADDR_REG, paddr);

	/* sync cache */
	bus_dmamap_sync(sc->sc_dmat, sc->sc_txdesc_dmamap, 0,
	    sc->sc_txdesc_dmamap->dm_mapsize, BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxdesc_dmamap, 0,
	    sc->sc_rxdesc_dmamap->dm_mapsize, BUS_DMASYNC_PREWRITE);


	/* DMA configuration */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_DMA_CFG_REG,
#if ETHER_ALIGN == 0
	    GEC_DMA_CFG_RX_OFFSET_2B_DIS |
#endif
	    0);


	/* enable MIB_COUNT_TH and PORT_STATUS_CHG interrupts */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_INT_MASK_REG,
	    ~(GEC_INT_MIB_COUNT_TH | GEC_INT_PORT_STATUS_CHG));

	/* Interrupts mode, etc */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_DLY_INT_CFG_REG,
#if 0
	    GEC_DLY_INT_CFG_MAX_PEND_TIME(2) |
	    GEC_DLY_INT_CFG_MAX_INT_CNT(5) |
	    GEC_DLY_INT_CFG_DELAY_INT_EN |
#endif
	    0);

#ifdef DEBUG_GEC
	(void)gec_dump_txdesc;
	(void)gec_dump_rxdesc;
#endif

	return 0;
}

static void
gec_update_mib(struct gec_softc *sc)
{
	struct ifnet *ifp;

	ifp = &sc->sc_ethercom.ec_if;
	ifp->if_ierrors +=
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXRUNT_REG) +
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXLONG_REG) +
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXDROP_REG) +
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXCRC_REG) +
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXARLDROP_REG) +
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXVLANDROP_REG) +
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXCSERR_REG);

#if 0
	/* cannot detect output error? */
	ifp->if_oerrors += ...
#endif

	ifp->if_collisions +=
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_TXPAUSECOL_REG);
}

static void
gec_clear_mib_counter(struct gec_softc *sc)
{
	/* seek to reset */
#ifdef DEBUG_GEC_DUMP
	printf("GEC_C_RXOKPKT_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXOKPKT_REG));
	printf("GEC_C_RXOKBYTE_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXOKBYTE_REG));
	printf("GEC_C_RXRUNT_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXRUNT_REG));
	printf("GEC_C_RXLONG_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXLONG_REG));
	printf("GEC_C_RXDROP_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXDROP_REG));
	printf("GEC_C_RXCRC_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXCRC_REG));
	printf("GEC_C_RXARLDROP_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXARLDROP_REG));
	printf("GEC_C_RXVLANDROP_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXVLANDROP_REG));
	printf("GEC_C_RXCSERR_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXCSERR_REG));
	printf("GEC_C_RXPAUSE_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXPAUSE_REG));
	printf("GEC_C_TXOKPKT_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_TXOKPKT_REG));
	printf("GEC_C_TXOKBYTE_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_TXOKBYTE_REG));
	printf("GEC_C_TXPAUSECOL_REG=%u\n", bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_TXPAUSECOL_REG));
#else
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXOKPKT_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXOKBYTE_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXRUNT_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXLONG_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXDROP_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXCRC_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXARLDROP_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXVLANDROP_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXCSERR_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_RXPAUSE_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_TXOKPKT_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_TXOKBYTE_REG);
	bus_space_read_4(sc->sc_iot, sc->sc_ioh, GEC_C_TXPAUSECOL_REG);
#endif
}

static int
gec_alloc_dma(struct gec_softc *sc, size_t size, void **addrp,
              bus_dmamap_t *mapp)
{
	bus_dma_segment_t seglist[1];
	int nsegs, error;

	if ((error = bus_dmamem_alloc(sc->sc_dmat, size, PAGE_SIZE, 0, seglist,
	    1, &nsegs, M_WAITOK)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to allocate DMA buffer, error=%d\n", error);
		goto fail_alloc;
	}

	if ((error = bus_dmamem_map(sc->sc_dmat, seglist, 1, size, addrp,
	    BUS_DMA_NOWAIT | BUS_DMA_COHERENT)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to map DMA buffer, error=%d\n",
		    error);
		goto fail_map;
	}

	if ((error = bus_dmamap_create(sc->sc_dmat, size, 1, size, 0,
	    BUS_DMA_NOWAIT, mapp)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to create DMA map, error=%d\n", error);
		goto fail_create;
	}

	if ((error = bus_dmamap_load(sc->sc_dmat, *mapp, *addrp, size, NULL,
	    BUS_DMA_NOWAIT)) != 0) {
		aprint_error_dev(sc->sc_dev,
		    "unable to load DMA map, error=%d\n", error);
		goto fail_load;
	}

	return 0;

 fail_load:
	bus_dmamap_destroy(sc->sc_dmat, *mapp);
 fail_create:
	bus_dmamem_unmap(sc->sc_dmat, *addrp, size);
 fail_map:
	bus_dmamem_free(sc->sc_dmat, seglist, 1);
 fail_alloc:
	return error;
}


#ifdef DEBUG_GEC
static void
gec_dump_txdesc(struct gec_softc *sc, int idx, const char *prefix)
{
	char tmpbuf[128];

	TXDESC_READIN(idx);

	snprintb(tmpbuf, sizeof(tmpbuf), 
	    "\20"
	    "\40COWN"
	    "\37EOR"
	    "\36FS"
	    "\35LS"
	    "\34INT"
	    "\33INSV"
	    "\32ICO"
	    "\31UCO"
	    "\30TCO",
	    sc->sc_txdesc_ring[idx].tx_ctrl);

	printf("%stxdesc[%3d]:tx_sdp =0x%08x\n", prefix, idx, sc->sc_txdesc_ring[idx].tx_sdp);
	printf("%stxdesc[%3d]:tx_ctrl=%s\n", prefix, idx, tmpbuf);
}


static void
gec_dump_rxdesc(struct gec_softc *sc, int idx, const char *prefix)
{
	char tmpbuf[128];

	RXDESC_READIN(idx);

	snprintb(tmpbuf, sizeof(tmpbuf), 
	    "\20"
	    "\40COWN"
	    "\37EOR"
	    "\36FS"
	    "\35LS"
	    "\32OSIZE"
	    "\31CRCE"
	    "\30RMC"
	    "\27HHIT"
	    "\26MYMAC"
	    "\25VTEC"
	    "\22IPF"
	    "\21L4F",
	    sc->sc_rxdesc_ring[idx].rx_ctrl);

	printf("%srxdesc[%3d]:rx_sdp =0x%08x\n", prefix, idx, sc->sc_rxdesc_ring[idx].rx_sdp);
	printf("%srxdesc[%3d]:rx_ctrl=%s\n", prefix, idx, tmpbuf);

#if 1 /* HEXDUMP */
	{
		unsigned char *p;
		int i;

		p = mtod(sc->sc_rxsoft[idx].rxs_mbuf, char *);
		for (i = 0; i < 128; i++, p++) {
			if ((i & 15) == 0)
				printf("%04x(%08x):", i, p);

			printf(" %02x", p[i]);

			if ((i & 15) == 15)
				printf("\n");
		}
	}
#endif
}
#endif

#ifdef DEBUG_GEC_DUMP
static void
gec_dump_hashtable(struct gec_softc *sc)
{
	uint8_t data[GEC_HASH_CTRL_HASHBITSIZE / 8];
	uint32_t status;
	int addr, bit, i;

	memset(data, 0, sizeof(data));
	for (addr = 0; addr < GEC_HASH_CTRL_HASHBITSIZE; addr++) {
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, GEC_HASH_CTRL_REG,
		    GEC_HASH_CTRL_CMD_START |
		    GEC_HASH_CTRL_CMD_READ |
		    GEC_HASH_CTRL_HASHADDR(addr));
		for (i = 1000; i > 0; i--) {
			status = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
			    GEC_HASH_CTRL_REG);
			if ((status & GEC_HASH_CTRL_CMD_START) == 0)
				break;
		}
		if (i <= 0)
			printf("cmd timeout: addr=%d", addr);

		bit = (status & GEC_HASH_CTRL_HASHBIT_MASK) ? 1 : 0;

		data[addr / 8] |= (bit << (addr & 7));
	}

	printf("\n");
	for (i = 0; i < GEC_HASH_CTRL_HASHBITSIZE; i++) {
		if ((i & 63) == 0)
			printf("%03x:", i);

		if ((i & 7) == 0)
			printf(" ");

		if (data[i / 8] & (1 << (i & 7)))
			printf("#");
		else
			printf("_");

		if ((i & 63) == 63)
			printf("\n");
	}
	printf("\n");
}
#endif
