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
 * STR91XX/STR92XX Orion series SoC
 * Embedded 4-port Gigabit Switch Engine with two 10/100/1000M RGMII/MII ports
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#define DEBUG_GSE
#undef DEBUG_GSE_DUMP
#undef DEBUG_GSE_DUMP_MII

#include "vlan.h"
#include "rnd.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/mbuf.h>
#include <sys/device.h>
#include <sys/sockio.h>
#include <sys/kernel.h>

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
#include <arm/star/if_gsereg.h>

#define GSE_ETHER_MIN_LEN	(ETHER_MIN_LEN - ETHER_CRC_LEN)	/* not including FCS */
#define GSE_ETHER_MIN_LEN_VLAN	(GSE_ETHER_MIN_LEN + ETHER_VLAN_ENCAP_LEN)	/* not including VLAN-tag and FCS */
#define GSE_MAX_PKT_LEN		1536
#define GSE_MAX_PKT_NSEGS	16	/* XXX */

#define GSE_TX_RING_CNT		256	/* must be 2^n */
#define GSE_TX_RING_MINIMAL_CNT	1	/* ready to build padding */
#define GSE_RX_RING_CNT		512	/* must be 2^n */

#define GSE_TX_MASKIDX(idx)	((idx) & (GSE_TX_RING_CNT - 1))
#define GSE_TX_NEXTIDX(idx)	GSE_TX_MASKIDX((idx) + 1)
#define GSE_RX_MASKIDX(idx)	((idx) & (GSE_RX_RING_CNT - 1))
#define GSE_RX_NEXTIDX(idx)	GSE_RX_MASKIDX((idx) + 1)

#define ETHER_ALIGN	2	/* recomennded */
#ifndef ETHER_ALIGN
#define ETHER_ALIGN	0
#endif

struct gse_if_softc {
	device_t sc_dev;

	struct gsec_softc *sc_gsec;
	int sc_portno;
	int sc_if_flags;		/* local copy of if_flags */
	int sc_flowflags;		/* 802.3x flow control flags */
	struct ethercom sc_ethercom;	/* interface info */
	struct mii_data sc_mii;
	uint8_t sc_enaddr[ETHER_ADDR_LEN];
};

struct gsec_softc {
	device_t sc_dev;

	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	bus_dma_tag_t sc_dmat;

	/* interrupts */
	void *sc_ih_stat;	/* IRQ18 status changed */
	void *sc_ih_tx;		/* IRQ19 TX complete */
	void *sc_ih_rx;		/* IRQ20 RX complete */
	/* IRQ21 (Queue Empty) and IRQ22 (Queue Full) are not used */

	/* TX */
	struct gse_txdesc *sc_txdesc_ring;	/* gse_txdesc[GSE_TX_RING_CNT] */
	bus_dmamap_t sc_txdesc_dmamap;
	struct gse_rxdesc *sc_rxdesc_ring;	/* gse_rxdesc[GSE_RX_RING_CNT] */
	bus_dmamap_t sc_rxdesc_dmamap;
	void *sc_padseg;			/* 64byte stub segment */
	bus_dmamap_t sc_padseg_dmamap;		/* pad for short TX frame */
	struct gse_txsoft {
		struct mbuf *txs_mbuf;		/* head of our mbuf chain */
		bus_dmamap_t txs_dmamap;	/* our DMA map */
	} sc_txsoft[GSE_TX_RING_CNT];
	int sc_tx_considx;
	int sc_tx_prodidx;
	int sc_tx_free;

	/* RX */
	struct gse_rxsoft {
		struct mbuf *rxs_mbuf;		/* head of our mbuf chain */
		bus_dmamap_t rxs_dmamap;	/* our DMA map */
	} sc_rxsoft[GSE_RX_RING_CNT];
	int sc_rx_readidx;
	uint32_t sc_rx_last_dptr;

	enum {
		GSEC_PHY_INTERNAL,
		GSEC_PHY_VSC7385
	} sc_phy_type;

	struct gse_if_softc *sc_if_gse[2];
#if NRND > 0
	rndsource_element_t sc_rnd_source;
#endif
};

struct gsec_attach_args {
	int ga_portno;
};

#define TXDESC_WRITEOUT(idx)					\
	bus_dmamap_sync(sc->sc_dmat, sc->sc_txdesc_dmamap,	\
	    sizeof(struct gse_txdesc) * (idx),			\
	    sizeof(struct gse_txdesc),				\
	    BUS_DMASYNC_PREWRITE)

#define TXDESC_READIN(idx)					\
	bus_dmamap_sync(sc->sc_dmat, sc->sc_txdesc_dmamap,	\
	    sizeof(struct gse_txdesc) * (idx),			\
	    sizeof(struct gse_txdesc),				\
	    BUS_DMASYNC_PREREAD)

#define RXDESC_WRITEOUT(idx)					\
	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxdesc_dmamap,	\
	    sizeof(struct gse_rxdesc) * (idx),			\
	    sizeof(struct gse_rxdesc),				\
	    BUS_DMASYNC_PREWRITE)

#define RXDESC_READIN(idx)					\
	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxdesc_dmamap,	\
	    sizeof(struct gse_rxdesc) * (idx),			\
	    sizeof(struct gse_rxdesc),				\
	    BUS_DMASYNC_PREREAD)

#define TX_DPTR2IDX(dptr)					\
	(((dptr) - sc->sc_txdesc_dmamap->dm_segs[0].ds_addr) /	\
	sizeof(struct gse_txdesc))

#define RX_DPTR2IDX(dptr)					\
	(((dptr) - sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr) /	\
	sizeof(struct gse_rxdesc))

#define GSE_REG_SETBIT(sc, reg, bit)					\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, reg,		\
	    bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, reg) | (bit))

#define GSE_REG_CLRBIT(sc, reg, bit)					\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, reg,		\
	    bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, reg) & ~(bit))

static int gsec_match(device_t, struct cfdata *, void *);
static void gsec_attach(device_t, device_t, void *);
int gsec_print(void *, const char *);

static int gse_match(device_t, struct cfdata *, void *);
static void gse_attach(device_t, device_t, void *);

/* interrupt handlers */
static int gsec_stat_intr(void *);
static int gsec_tx_intr(void *);
static int gsec_rx_intr(void *);

/* for ifnet interface */
static void gse_start(struct ifnet *);
static int gse_ioctl(struct ifnet *, u_long, void *);
static int gse_init(struct ifnet *);
static void gse_stop(struct ifnet *, int);
static void gse_watchdog(struct ifnet *);

/* for MII */
static int gse_miibus_readreg(device_t, int, int);
static void gse_miibus_writereg(device_t, int, int, int);
static void gse_miibus_statchg(device_t);
/* vsc7385 */
static int gse_ifmedia_upd(struct ifnet *);
static void gse_ifmedia_sts(struct ifnet *, struct ifmediareq *);

/* misc routines */
static inline uint32_t gsec_arl_wait(struct gsec_softc *);
static void gsec_arl_write(struct gsec_softc *, struct gse_arl *);
static int gsec_arl_read(struct gsec_softc *, uint32_t, struct gse_arl *);
static void gsec_arl_dump(struct gsec_softc *);
static void gsec_arl_clearall(struct gsec_softc *);
static void gse_arl_clear(struct gse_if_softc *);
static void gse_setmulti(struct gse_if_softc *);

static void gsec_enable_intr(struct gsec_softc *);
static void gsec_disable_intr(struct gsec_softc *);
static void gsec_hwreset(struct gsec_softc *);
static int gsec_encap(struct gsec_softc *, struct mbuf **, int);
static int gsec_init_regs(struct gsec_softc *);
static void gsec_init_phy_vsc7385(struct gsec_softc *);

static int gsec_alloc_ring(struct gsec_softc *);
static void gsec_init_txring(struct gsec_softc *);
static int gsec_init_rxring(struct gsec_softc *);
static void gsec_reset_rxdesc(struct gsec_softc *, int);
static int gsec_alloc_rxbuf(struct gsec_softc *, int);
static void gsec_drain_txbuf(struct gsec_softc *);
static void gsec_drain_rxbuf(struct gsec_softc *);
static int gsec_alloc_dma(struct gsec_softc *, size_t, void **,
                         bus_dmamap_t *);

#ifdef DEBUG_GSE
static void gse_dump_txdesc(struct gsec_softc *, int, const char *);
static void gse_dump_rxdesc(struct gsec_softc *, int, const char *);
static void gse_dumpall_txdesc(struct gsec_softc *, const char *);
static void gse_dumpall_rxdesc(struct gsec_softc *, const char *);
#endif

CFATTACH_DECL_NEW(gsec, sizeof(struct gsec_softc),
    gsec_match, gsec_attach, NULL, NULL);

CFATTACH_DECL_NEW(gse, sizeof(struct gse_if_softc),
    gse_match, gse_attach, NULL, NULL);

/* ARGSUSED */
static int
gsec_match(device_t parent __unused, struct cfdata *match __unused, void *aux)
{
	struct star_attach_args *sa;

	if (!CPU_IS_STR9100())
		return 0;

	sa = (struct star_attach_args *)aux;

	sa->sa_size = GSE_REG_SIZE;
	return 1;
}

/* ARGSUSED */
static void
gsec_attach(device_t parent __unused, device_t self, void *aux)
{
	struct gsec_softc *sc;
	struct star_attach_args *sa;
	struct gsec_attach_args gaa;
	int i;

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_addr = sa->sa_addr;
	sc->sc_dmat = sa->sa_dmat;

	sc->sc_ih_stat = NULL;
	sc->sc_ih_tx = NULL;
	sc->sc_ih_rx = NULL;

	aprint_naive("\n");
	aprint_normal(": Gigabit Switch Engine\n");

	if (bus_space_map(sc->sc_iot, sc->sc_addr, sa->sa_size, 0,
	    &sc->sc_ioh)) {
		aprint_error_dev(self, "Cannot map registers\n");
		return;
	}

	/* allocate dma buffer */
	if (gsec_alloc_ring(sc))
		return;

	gsec_init_regs(sc);
	gsec_init_txring(sc);
	if (gsec_init_rxring(sc) != 0) {
		gsec_drain_rxbuf(sc);
		printf("%s: Cannot init RX Ring\n", device_xname(sc->sc_dev));
		goto failure;
	}

	/* clear Address Resolution Logic table */
	gsec_arl_clearall(sc);

	for (i = 0; i < 2; i++) {
		gaa.ga_portno = i;
		(void)config_found(sc->sc_dev, &gaa, gsec_print);
	}

	if ((sc->sc_ih_stat = intr_establish(STAR_IRQ_NIC_STAT, IPL_NET,
	    STAR_INTR_HIGHLEVEL_TRIGGER, gsec_stat_intr, sc)) == NULL) {
		printf("%s: unable to establish STAT interrupt\n",
		    device_xname(sc->sc_dev));
		goto failure;
	}

#ifdef notyet
	if (!pmf_device_register(self, NULL, gse_resume))
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

/* ARGSUSED */
static int
gse_match(device_t parent __unused, struct cfdata *match __unused, void *aux __unused)
{
	return 1;
}

/* ARGSUSED */
static void
gse_attach(device_t parent, device_t self, void *aux)
{
	struct gse_if_softc *sc;
	struct gsec_attach_args *gaa;
	struct ifnet *ifp;

	gaa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_portno = gaa->ga_portno;
	sc->sc_gsec = device_private(parent);
	sc->sc_gsec->sc_if_gse[sc->sc_portno] = sc;

	/* XXX: read from configuration... */
	if (sc->sc_portno == 0) {
		sc->sc_enaddr[0] = 0x00;
		sc->sc_enaddr[1] = 0x08;
		sc->sc_enaddr[2] = 0xa1;
		sc->sc_enaddr[3] = 0xc0;
		sc->sc_enaddr[4] = 0x45;
		sc->sc_enaddr[5] = 0x7e;
	} else {
		sc->sc_enaddr[0] = 0x00;
		sc->sc_enaddr[1] = 0x08;
		sc->sc_enaddr[2] = 0xa1;
		sc->sc_enaddr[3] = 0xc0;
		sc->sc_enaddr[4] = 0x45;
		sc->sc_enaddr[5] = 0x7f;
	}

	aprint_normal("\n");
	aprint_normal_dev(self, "Ethernet address %s\n",
	    ether_sprintf(sc->sc_enaddr));

	ifp = &sc->sc_ethercom.ec_if;
	strlcpy(ifp->if_xname, device_xname(sc->sc_dev), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_mtu = ETHERMTU;
	ifp->if_baudrate = IF_Mbps(1000);
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_ioctl = gse_ioctl;
	ifp->if_start = gse_start;
	ifp->if_init = gse_init;
	ifp->if_stop = gse_stop;
	ifp->if_watchdog = gse_watchdog;
#if 0
	ifp->if_capabilities =
	    IFCAP_CSUM_IPv4_Tx |
	    IFCAP_CSUM_IPv4_Rx |
	    IFCAP_CSUM_TCPv4_Tx | IFCAP_CSUM_TCPv4_Rx |
	    IFCAP_CSUM_UDPv4_Tx | IFCAP_CSUM_UDPv4_Rx;
#endif
	IFQ_SET_MAXLEN(&ifp->if_snd, max(GSE_TX_RING_CNT - 1, IFQ_MAXLEN));
	IFQ_SET_READY(&ifp->if_snd);

	/* setup MII */
	sc->sc_ethercom.ec_mii = &sc->sc_mii;
	sc->sc_mii.mii_ifp = ifp;
	sc->sc_mii.mii_readreg = gse_miibus_readreg;
	sc->sc_mii.mii_writereg = gse_miibus_writereg;
	sc->sc_mii.mii_statchg = gse_miibus_statchg;
	ifmedia_init(&sc->sc_mii.mii_media, 0, ether_mediachange,
	    ether_mediastatus);

	/* At first, try to attach Internal PHY */
	sc->sc_gsec->sc_phy_type = GSEC_PHY_INTERNAL;
	mii_attach(sc->sc_dev, &sc->sc_mii, 0xffffffff, sc->sc_portno,
	    MII_OFFSET_ANY, 0);

	/* In the next,, try to attach RGMII switch */
	if (LIST_FIRST(&sc->sc_mii.mii_phys) == NULL) {
		sc->sc_gsec->sc_phy_type = GSEC_PHY_VSC7385;
		gsec_init_phy_vsc7385(sc->sc_gsec);

		ifmedia_init(&sc->sc_mii.mii_media, IFM_IMASK,
		    gse_ifmedia_upd, gse_ifmedia_sts);
		ifmedia_add(&sc->sc_mii.mii_media, IFM_ETHER | IFM_AUTO,
		    0, NULL);
		ifmedia_add(&sc->sc_mii.mii_media, IFM_ETHER | IFM_NONE,
		    0, NULL);
	}
	/* set default auto */
	ifmedia_set(&sc->sc_mii.mii_media, IFM_ETHER | IFM_AUTO);

	if_attach(ifp);
	ether_ifattach(ifp, sc->sc_enaddr);
}

int
gsec_print(void *aux, const char *pnp)
{
	struct gsec_attach_args *gaa = aux;

	gaa = (struct gsec_attach_args *)aux;
	if (pnp) {
		aprint_normal(" port%d at %s", gaa->ga_portno, pnp);
	} else {
		aprint_normal(" port%d", gaa->ga_portno);
	}
	return UNCONF;
}

static int
gsec_stat_intr(void *arg)
{
	struct gsec_softc *sc;
	uint32_t status;

	sc = (struct gsec_softc *)arg;
	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_INT_STATUS_REG);

#ifdef DEBUG_GSE
	if (status) {
		char tmpbuf[128];
		snprintb(tmpbuf, sizeof(tmpbuf), 
		    "\20"
		    "\40PORT1_INGRESS"
		    "\37PORT1_LOCAL"
		    "\36PORT1_RMC_PAUSE"
		    "\35PORT1_NO_DEST"
		    "\34PORT1_JAMMED"
		    "\33PORT1_RX_ERR"
		    "\32PORT1_BCSTORM"
		    "\31PORT1_NOFREELINK"
		    "\30PORT0_INGRESS"
		    "\27PORT0_LOCAL"
		    "\26PORT0_RMC_PAUSE"
		    "\25PORT0_NO_DEST"
		    "\24PORT0_JAMMED"
		    "\23PORT0_RX_ERR"
		    "\22PORT0_BCSTORM"
		    "\21PORT0_NOFREELINK"
		    "\14CPU_UNKNOWN_VLAN"
		    "\13PORT1_UNKNOWN_VLAN"
		    "\12PORT0_UNKNOWN_VLAN"
		    "\11INTRUDER1"
		    "\10INTRUDER0"
		    "\07PORT_STATUS_CHG"
		    "\06BUFFER_FULL"
		    "\05GLOBAL_Q_FULL"
		    "\04HNAT_Q_FULL"
		    "\03CPU_Q_FULL"
		    "\02PORT1_Q_FULL"
		    "\01PORT0_Q_FULL",
		    status);
		printf("%s:%d: sc=%p, status=%s\n", __func__, __LINE__, sc, tmpbuf);
	}
#endif

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_INT_STATUS_REG, status);
	return 1;
}

static int
gsec_tx_intr(void *arg)
{
	struct gsec_softc *sc;
	struct ifnet *ifp;
	struct gse_txsoft *txs;
	int idx;

	sc = (struct gsec_softc *)arg;

	for (idx = sc->sc_tx_considx; idx != sc->sc_tx_prodidx;
	    idx = GSE_TX_NEXTIDX(idx)) {

		txs = &sc->sc_txsoft[idx];

#ifdef DEBUG_GSE_DUMP
		gse_dump_txdesc(sc, idx, "TX_INTR:");
#endif

		TXDESC_READIN(idx);
		if (!(sc->sc_txdesc_ring[idx].tx_ctrl & GSE_TXDESC_CTRL_COWN)) {
			/* This TX Descriptor has not been transmitted yet? */
//			printf("TX_INTR: TX descriptor has not been transmitted yet?\n");
//			gse_dump_txdesc(sc, idx, "TX_INTR:");
			break;
		}

		if (sc->sc_txdesc_ring[idx].tx_ctrl & GSE_TXDESC_CTRL_PMAP_PORT0)
			ifp = &sc->sc_if_gse[0]->sc_ethercom.ec_if;
		else if (sc->sc_txdesc_ring[idx].tx_ctrl & GSE_TXDESC_CTRL_PMAP_PORT1)
			ifp = &sc->sc_if_gse[1]->sc_ethercom.ec_if;
		else {
			gse_dump_txdesc(sc, idx, "TX_INTR:");
			panic("%s: nether port0 nor port1", device_xname(sc->sc_dev));
		}

		/* txsoft is used only first segment */
		if (sc->sc_txdesc_ring[idx].tx_ctrl & GSE_TXDESC_CTRL_FS) {
			bus_dmamap_unload(sc->sc_dmat,
			    txs->txs_dmamap);
			m_freem(txs->txs_mbuf);
			txs->txs_mbuf = NULL;

			ifp->if_opackets++;
		}
		sc->sc_tx_free++;
	}
	sc->sc_tx_considx = idx;

#ifdef DEBUG_GSE_DUMP
	printf("TX_INTR:tx_free=%d\n", sc->sc_tx_free);
#endif

	if ((sc->sc_tx_free > GSE_TX_RING_MINIMAL_CNT) &&
	    ((sc->sc_if_gse[0]->sc_ethercom.ec_if.if_flags & IFF_OACTIVE) ||
	     (sc->sc_if_gse[1]->sc_ethercom.ec_if.if_flags & IFF_OACTIVE))) {

//		printf("TX_INTR: clear IFF_OACTIVE: tx_free=%d\n", sc->sc_tx_free);

		sc->sc_if_gse[0]->sc_ethercom.ec_if.if_flags &= ~IFF_OACTIVE;
		sc->sc_if_gse[1]->sc_ethercom.ec_if.if_flags &= ~IFF_OACTIVE;
	}

	/*
	 * No more pending TX descriptor,
	 * cancel the watchdog timer.
	 */
	if (sc->sc_tx_free == GSE_TX_RING_CNT) {
		sc->sc_if_gse[0]->sc_ethercom.ec_if.if_timer = 0;
		sc->sc_if_gse[1]->sc_ethercom.ec_if.if_timer = 0;
	}

	return 1;
}

static int debug_rx_dump = 0;

static int
gsec_rx_intr(void *arg)
{
	struct gsec_softc *sc;
	struct ifnet *ifp;
	struct gse_rxsoft *rxs;
	int idx, len;
	uint32_t ctrl;
	struct mbuf *m;

	sc = (struct gsec_softc *)arg;

#if 0
	if (((sc->sc_if_gse[0]->sc_ethercom.ec_if.if_flags & IFF_RUNNING) == 0) &&
	    ((sc->sc_if_gse[1]->sc_ethercom.ec_if.if_flags & IFF_RUNNING) == 0)) {
		return 1;
	}
#endif

	/* stop RX DMA transfer */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_RX_DMA_CTRL_REG, 0);

	for (idx = sc->sc_rx_readidx; RX_DPTR2IDX(bus_space_read_4(sc->sc_iot,
	    sc->sc_ioh, GSE_RX_DPTR_REG)) != idx; idx = GSE_RX_NEXTIDX(idx)) {

		if ((idx < 0) || (idx > GSE_RX_RING_CNT))
			panic("illegal RX index: %d\n", idx);


		rxs = &sc->sc_rxsoft[idx];

		RXDESC_READIN(idx);
		ctrl = sc->sc_rxdesc_ring[idx].rx_ctrl;
		if ((ctrl & GSE_RXDESC_CTRL_COWN) == 0)
			continue;

#if 1	/* XXX: Magic Packet Debugger: e.g. ping -c1 -p 98 str9104 */
		KASSERT(rxs->rxs_mbuf != NULL);
		{
			unsigned char *p;

			p = mtod(rxs->rxs_mbuf, char *);
			if ((p[6+6+2] == 0x45) &&
			    (p[6+6+2+9] == 0x01) &&
			    (p[6+6+2+20] == 0x08)) {

				switch (p[6+6+2+20+16]) {
				case 0x95:
					gse_dumpall_txdesc(sc, "rx_intr:");
					break;
				case 0x96:
					gse_dumpall_rxdesc(sc, "rx_intr:");
					break;
				case 0x97:
					debug_rx_dump ^= 1;
					break;
				case 0x98:
					printf("#");
					break;
				case 0x99:
					Debugger();
					break;
				default:
					break;
				}
			}
		}
#ifdef DEBUG_GSE
		if (debug_rx_dump)
			gse_dump_rxdesc(sc, idx, "RX_INTR:");
#endif
#endif

		switch (ctrl & GSE_RXDESC_CTRL_SP_MASK) {
		case GSE_RXDESC_CTRL_SP_PORT0:
			ifp = &sc->sc_if_gse[0]->sc_ethercom.ec_if;
			break;
		case GSE_RXDESC_CTRL_SP_PORT1:
			ifp = &sc->sc_if_gse[1]->sc_ethercom.ec_if;
			break;
		default:
			gse_dump_rxdesc(sc, idx, "RX_INTR:");
			panic("%s: nether port0 nor port1", device_xname(sc->sc_dev));
		}

		len = ctrl & GSE_RXDESC_CTRL_SDL_MASK;
		if (len < ETHER_HDR_LEN) {
			ifp->if_ierrors++;
			gsec_reset_rxdesc(sc, idx);
			continue;
		}
#ifdef DEBUG_GSE
		if (len > ETHER_MAX_LEN) {
			printf("%s: BIG PACKET?\n",
			    device_xname(sc->sc_dev));
		}
#endif
		if ((ifp->if_flags & IFF_RUNNING) == 0) {
			gsec_reset_rxdesc(sc, idx);
			continue;
		}

		/* packet receive ok */
		ifp->if_ipackets++;

		/*
		 * build mbuf from RX Descriptor if needed
		 */
		m = rxs->rxs_mbuf;
		m->m_pkthdr.rcvif = ifp;
		m->m_pkthdr.len = m->m_len = len;

		/* checksum offloading */
		if (ifp->if_csum_flags_rx & M_CSUM_IPv4) {
			if ((ctrl & GSE_RXDESC_CTRL_PROT_MASK) !=
			    GSE_RXDESC_CTRL_PROT_OTHERS) {
				/* IP packet (including TCP and UDP) */
				m->m_pkthdr.csum_flags |= M_CSUM_IPv4;
				if (ctrl & GSE_RXDESC_CTRL_IPF)
					m->m_pkthdr.csum_flags |= M_CSUM_IPv4_BAD;
			}
		}
		if (ifp->if_csum_flags_rx & M_CSUM_TCPv4) {
			if ((ctrl & GSE_RXDESC_CTRL_PROT_MASK) ==
			    GSE_RXDESC_CTRL_PROT_TCP) {
				m->m_pkthdr.csum_flags |= M_CSUM_TCPv4;
				if (ctrl & GSE_RXDESC_CTRL_L4F)
					m->m_pkthdr.csum_flags |= M_CSUM_TCP_UDP_BAD;
			}
		}
		if (ifp->if_csum_flags_rx & M_CSUM_UDPv4) {
			if ((ctrl & GSE_RXDESC_CTRL_PROT_MASK) ==
			    GSE_RXDESC_CTRL_PROT_UDP) {
				m->m_pkthdr.csum_flags |= M_CSUM_UDPv4;
				if (ctrl & GSE_RXDESC_CTRL_L4F)
					m->m_pkthdr.csum_flags |= M_CSUM_TCP_UDP_BAD;
			}
		}

		bus_dmamap_sync(sc->sc_dmat, rxs->rxs_dmamap, 0,
		    rxs->rxs_dmamap->dm_mapsize, BUS_DMASYNC_PREREAD);

		/* Pass this up to any BPF listeners. */
		bpf_mtap(ifp, m);

		(*ifp->if_input)(ifp, m);

		/* clear this rxsoft, and realloc rxbuf */
		bus_dmamap_unload(sc->sc_dmat, rxs->rxs_dmamap);
		rxs->rxs_mbuf = NULL;
		if (gsec_alloc_rxbuf(sc, idx) != 0)
			panic("%s: cannot allocate new mbuf cluster", device_xname(sc->sc_dev));
	}
	sc->sc_rx_readidx = idx;

	/* re-enable RX DMA */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_RX_DMA_CTRL_REG,
	    GSE_RX_DMA_CTRL_RX_EN);
	return 1;
}

static inline uint32_t
gsec_arl_wait(struct gsec_softc *sc)
{
	uint32_t status;

	while (((status = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    GSE_ARL_TABLE_CTRL1_REG)) & GSE_ARL_TABLE_CTRL1_CMD_COMPLETE) == 0)
		;

	return status;
}

static void
gsec_arl_write(struct gsec_softc *sc, struct gse_arl *arl)
{
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_ARL_TABLE_CTRL1_REG,
	    GSE_ARL_TABLE_CTRL1_MAC1(arl->garl_mac[1]) +
	    GSE_ARL_TABLE_CTRL1_MAC0(arl->garl_mac[0]) +
	    GSE_ARL_TABLE_CTRL1_PORTMAP(arl->garl_port) +
	    GSE_ARL_TABLE_CTRL1_AGE(arl->garl_age) +
	    GSE_ARL_TABLE_CTRL1_VLAN_ID(arl->garl_vlangid) +
	    (arl->garl_mymac ? GSE_ARL_TABLE_CTRL1_VLAN_MAC : 0) +
	    (arl->garl_filter ? GSE_ARL_TABLE_CTRL1_FILTER : 0));

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_ARL_TABLE_CTRL2_REG,
	    GSE_ARL_TABLE_CTRL2_MAC5(arl->garl_mac[5]) +
	    GSE_ARL_TABLE_CTRL2_MAC4(arl->garl_mac[4]) +
	    GSE_ARL_TABLE_CTRL2_MAC3(arl->garl_mac[3]) +
	    GSE_ARL_TABLE_CTRL2_MAC2(arl->garl_mac[2]));

	/* write entry */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_ARL_TABLE_CTRL0_REG,
	    GSE_ARL_TABLE_CTRL0_WT_CMD);
	gsec_arl_wait(sc);
}

static int
gsec_arl_read(struct gsec_softc *sc, uint32_t cmd, struct gse_arl *arl)
{
	uint32_t reg1, reg2;

	/* lookup entry */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_ARL_TABLE_CTRL0_REG,
	    cmd);
	reg1 = gsec_arl_wait(sc);

	/* end of table? */
	if (reg1 & GSE_ARL_TABLE_CTRL1_TABLE_END) {
		memset(arl, 0, sizeof(*arl));
		return -1;
	}

	reg2 = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    GSE_ARL_TABLE_CTRL2_REG);

	arl->garl_filter = (reg1 & GSE_ARL_TABLE_CTRL1_FILTER) ? 1 : 0;
	arl->garl_mymac = (reg1 & GSE_ARL_TABLE_CTRL1_VLAN_MAC) ? 1 : 0;
	arl->garl_vlangid = (reg1 & GSE_ARL_TABLE_CTRL1_VLAN_ID_MASK) >> GSE_ARL_TABLE_CTRL1_VLAN_ID_SHIFT;
	arl->garl_age = (reg1 & GSE_ARL_TABLE_CTRL1_AGE_MASK) >> GSE_ARL_TABLE_CTRL1_AGE_SHIFT;
	arl->garl_port = (reg1 & GSE_ARL_TABLE_CTRL1_PORTMAP_MASK) >> GSE_ARL_TABLE_CTRL1_PORTMAP_SHIFT;
	arl->garl_mac[0] = (reg1 >> 16) & 0xff;
	arl->garl_mac[1] = (reg1 >> 24) & 0xff;
	arl->garl_mac[2] = reg2 & 0xff;
	arl->garl_mac[3] = (reg2 >> 8) & 0xff;
	arl->garl_mac[4] = (reg2 >> 16) & 0xff;
	arl->garl_mac[5] = (reg2 >> 24) & 0xff;

	return 0;
}

static void
gsec_arl_dump(struct gsec_softc *sc)
{
	struct gse_arl arl;
	int i;
	uint32_t cmd;

	printf("[ARL TABLE DUMP]\n");

	i = 0;
	cmd = GSE_ARL_TABLE_CTRL0_SRCH_START_CMD;
	while (gsec_arl_read(sc, cmd, &arl) == 0) {
		printf("ARL[%d]: %s, filter=%d, mymac=%d, vlanid=%08x, age=%d, port=%d\n",
		    i++,
		    ether_sprintf(arl.garl_mac), 
		    arl.garl_filter,
		    arl.garl_mymac,
		    arl.garl_vlangid,
		    arl.garl_age,
		    arl.garl_port);

		cmd = GSE_ARL_TABLE_CTRL0_SRCH_AGAIN_CMD;
	}
}

static void
gsec_arl_clearall(struct gsec_softc *sc)
{
	struct gse_arl arl;

	while (gsec_arl_read(sc, GSE_ARL_TABLE_CTRL0_SRCH_START_CMD, &arl) == 0) {
		arl.garl_age = GARL_AGE_INVALID;
		gsec_arl_write(sc, &arl);
	}
}

static void
gse_arl_clear(struct gse_if_softc *scp)
{
	struct gsec_softc *sc;
	struct gse_arl arl;
	uint32_t cmd;

	cmd = GSE_ARL_TABLE_CTRL0_SRCH_START_CMD;

	sc = scp->sc_gsec;
	while (gsec_arl_read(sc, cmd, &arl) == 0) {
		cmd = GSE_ARL_TABLE_CTRL0_SRCH_AGAIN_CMD;
		if (((scp->sc_portno == 0) && (arl.garl_port == GARL_PORT_0)) ||
		    ((scp->sc_portno == 1) && (arl.garl_port == GARL_PORT_1))) {
			arl.garl_age = GARL_AGE_INVALID;
			gsec_arl_write(sc, &arl);
		}
	}
}

static void
gse_setmulti(struct gse_if_softc *scp)
{
	struct gse_arl arl;
	struct gsec_softc *sc;
	struct ifnet *ifp;
	struct ether_multi *enm;
	struct ether_multistep step;

	sc = scp->sc_gsec;
	ifp = &scp->sc_ethercom.ec_if;

	/* register my mac address */
	arl.garl_filter = 0;
	arl.garl_mymac = 1;
	arl.garl_vlangid = 0;
	arl.garl_age = GARL_AGE_STATIC;
	arl.garl_port = (scp->sc_portno == 0) ? GARL_PORT_0 : GARL_PORT_1;
	memcpy(arl.garl_mac, scp->sc_enaddr, ETHER_ADDR_LEN);
	gsec_arl_write(sc, &arl);

	ifp->if_flags &= ~IFF_ALLMULTI;
	ETHER_FIRST_MULTI(step, &scp->sc_ethercom, enm);
	while (enm != NULL) {
#ifdef DEBUG_GSE
		if (memcmp(enm->enm_addrlo, enm->enm_addrhi, ETHER_ADDR_LEN)) {
			printf("%s: setmulti %s-%s\n", device_xname(scp->sc_dev),
			    ether_sprintf(enm->enm_addrlo),
			    ether_sprintf(enm->enm_addrhi));
		} else {
			printf("%s: setmulti %s\n", device_xname(scp->sc_dev),
			    ether_sprintf(enm->enm_addrlo));
		}
#endif
		/*
		 * If multicast range, fall back to ALLMULTI.
		 */
		if (memcmp(enm->enm_addrlo, enm->enm_addrhi, ETHER_ADDR_LEN)) {
			ifp->if_flags |= IFF_ALLMULTI;
			break;
		}

		/* register mac address */
		arl.garl_filter = 0;
		arl.garl_vlangid = 0;
		arl.garl_age = GARL_AGE_STATIC;
		arl.garl_mymac = 1;
		arl.garl_port = (scp->sc_portno == 0) ? GARL_PORT_0 : GARL_PORT_1;
		memcpy(arl.garl_mac, enm->enm_addrlo, ETHER_ADDR_LEN);
		gsec_arl_write(sc, &arl);

		ETHER_NEXT_MULTI(step, enm);
	}
}

static void
gsec_enable_intr(struct gsec_softc *sc)
{
#ifdef DEBUG_GSE
	printf("%s:%d\n", __func__, __LINE__);
#endif
	if ((sc->sc_ih_tx = intr_establish(STAR_IRQ_NIC_TX, IPL_NET,
	    STAR_INTR_RISING_EDGE, gsec_tx_intr, sc)) == NULL) {
		printf("%s: unable to establish TX interrupt\n",
		    device_xname(sc->sc_dev));
		goto failure;
	}
	if ((sc->sc_ih_rx = intr_establish(STAR_IRQ_NIC_RX, IPL_NET,
	    STAR_INTR_RISING_EDGE, gsec_rx_intr, sc)) == NULL) {
		printf("%s: unable to establish RX interrupt\n",
		    device_xname(sc->sc_dev));
		goto failure;
	}
	return;

 failure:
	gsec_disable_intr(sc);
}

static void
gsec_disable_intr(struct gsec_softc *sc)
{
#ifdef DEBUG_GSE
	printf("%s:%d\n", __func__, __LINE__);
#endif
	if (sc->sc_ih_tx != NULL) {
		star_intr_disestablish(sc->sc_ih_tx);
		sc->sc_ih_tx = NULL;
	}
	if (sc->sc_ih_rx != NULL) {
		star_intr_disestablish(sc->sc_ih_rx);
		sc->sc_ih_rx = NULL;
	}
}

/*
 * ifnet interfaces
 */
static int
gse_init(struct ifnet *ifp)
{
	struct gse_if_softc *scp;
	struct gsec_softc *sc;
	int s, error = 0;

	scp = ifp->if_softc;
	sc = scp->sc_gsec;

#ifdef DEBUG_GSE
	printf("%s:%d: ifp=%p\n", __func__, __LINE__, ifp);
#endif

	s = splnet();

	/* reload mac address */
	memcpy(scp->sc_enaddr, CLLADDR(ifp->if_sadl), ETHER_ADDR_LEN);

	/* program multicast address */
	gse_arl_clear(scp);
	gse_setmulti(scp);

	mii_mediachg(&scp->sc_mii);

	/* need to enable interrupt? */
	if (!(sc->sc_if_gse[0]->sc_ethercom.ec_if.if_flags & IFF_RUNNING) &&
	    !(sc->sc_if_gse[1]->sc_ethercom.ec_if.if_flags & IFF_RUNNING)) {
		gsec_enable_intr(sc);
	}

	/* update if_flags */
	ifp->if_flags |= IFF_RUNNING;
	ifp->if_flags &= ~IFF_OACTIVE;

	/* update local copy of if_flags */
	scp->sc_if_flags = ifp->if_flags;

	/* start RX DMA */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_RX_DMA_CTRL_REG,
	    GSE_RX_DMA_CTRL_RX_EN);

	/* start TX DMA if needed */
	if (sc->sc_tx_free != GSE_TX_RING_CNT) {
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TX_DMA_CTRL_REG,
		    GSE_TX_DMA_CTRL_TX_EN);
	}

	splx(s);
	return error;
}

static void
gse_start(struct ifnet *ifp)
{
	struct gse_if_softc *scp;
	struct gsec_softc *sc;
	struct mbuf *m;
	int npkt;

	scp = ifp->if_softc;
	sc = scp->sc_gsec;

#ifdef DEBUG_GSE_DUMP
	printf("%s:%d: ifp=%p, sc=%p\n", __func__, __LINE__, ifp, sc);
#endif

	if ((ifp->if_flags & (IFF_RUNNING | IFF_OACTIVE)) != IFF_RUNNING)
		return;

	for (npkt = 0; ; npkt++) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

		if (sc->sc_tx_free <= GSE_TX_RING_MINIMAL_CNT) {
			/* no tx descriptor now... */
			ifp->if_flags |= IFF_OACTIVE;
#ifdef DEBUG_GSE
			printf("%s: TX Queue full: : tx_free=%d\n",
			    device_xname(sc->sc_dev), sc->sc_tx_free);
#endif
			break;
		}

		if (gsec_encap(sc, &m, scp->sc_portno) != 0) {
			if (m == NULL) {
				/* cannot mapping TX chain */
				ifp->if_oerrors++;
				IFQ_DEQUEUE(&ifp->if_snd, m);
			} else {
				/*
				 * no space to extract mbuf chain.
				 * need to retry
				 */
				ifp->if_flags |= IFF_OACTIVE;
#ifdef DEBUG_GSE
//				printf("%s: TX Queue full to extract mbuf: "
//				    "tx_free=%d\n",
//				    device_xname(sc->sc_dev), sc->sc_tx_free);
#endif
			}
			break;
		}

		IFQ_DEQUEUE(&ifp->if_snd, m);

		/*
		 * If there's a BPF listener, bounce a copy of this frame
		 * to him.
		 */
		bpf_mtap(ifp, m);
	}

	if (npkt) {
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TX_DMA_CTRL_REG,
		    GSE_TX_DMA_CTRL_TX_EN);

		ifp->if_timer = 5;
	}
}

static void
gse_stop(struct ifnet *ifp, int disable)
{
	struct gse_if_softc *scp;
	struct gsec_softc *sc;
	int s;

	scp = ifp->if_softc;
	sc = scp->sc_gsec;

#ifdef DEBUG_GSE
	printf("%s: %s:%d: ifp=%p, disable=%d(drain)\n",
	    device_xname(sc->sc_dev), __func__, __LINE__, ifp, disable);
#endif

	s = splnet();

	/* stop DMA transfer */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TX_DMA_CTRL_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_RX_DMA_CTRL_REG, 0);

	/* Mark the interface as down and cancel the watchdog timer. */
	ifp->if_flags &= ~(IFF_RUNNING | IFF_OACTIVE);
	ifp->if_timer = 0;

	/* no interrupt needed */
	if (!(sc->sc_if_gse[0]->sc_ethercom.ec_if.if_flags & IFF_RUNNING) &&
	    !(sc->sc_if_gse[1]->sc_ethercom.ec_if.if_flags & IFF_RUNNING)) {
		gsec_disable_intr(sc);
	}

	if (disable) {
		gsec_drain_txbuf(sc);
		gsec_drain_rxbuf(sc);
	}

	splx(s);

}

static void
gse_watchdog(struct ifnet *ifp)
{
	struct gse_if_softc *scp;
	struct gsec_softc *sc;
	int s;

	s = splnet();
	printf("---start gse_watchdog\n");

	scp = ifp->if_softc;
	sc = scp->sc_gsec;

	gsec_tx_intr(sc);
	gsec_rx_intr(sc);

	printf("%s: watchdog timeout\n", device_xname(scp->sc_dev));

	gse_stop(ifp, 1);
	gsec_hwreset(sc);
	gsec_init_regs(sc);
	gse_init(ifp);

	gsec_init_txring(sc);
	if (gsec_init_rxring(sc) != 0) {
		gsec_drain_rxbuf(sc);
		panic("%s: watchdog error: cannot allocate new mbuf cluster",
		    device_xname(sc->sc_dev));
	}

	printf("---end gse_watchdog\n");
	splx(s);
}

static void
gsec_hwreset(struct gsec_softc *sc __unused)
{
	uint32_t swrst_bit;

	/* force power off NIC */
	swrst_bit = STAR_REG_READ32(ORION_CLKPWR_SOFTRST_REG);
	swrst_bit &= ~ORION_CLKPWR_SOFTRST_SW;
	STAR_REG_WRITE32(ORION_CLKPWR_SOFTRST_REG, swrst_bit);
}

static int
gse_ioctl(struct ifnet *ifp, u_long command, void *data)
{
	struct gse_if_softc *scp;
	struct gsec_softc *sc;
	struct ifreq *ifr;
	int change, s, error;

	scp = ifp->if_softc;
	sc = scp->sc_gsec;
	ifr = data;

	error = 0;
	s = splnet();

	switch (command) {
	case SIOCSIFFLAGS:
		if ((error = ifioctl_common(ifp, command, data)) != 0)
			break;

		change = ifp->if_flags ^ scp->sc_if_flags;

		/* up or down */
		if (change & IFF_UP) {
			if (ifp->if_flags & IFF_UP) {
#ifdef DEBUG_GSE
				printf("%s:%d: NOW %s: DOWN -> UP\n", __func__, __LINE__, ifp->if_flags & IFF_RUNNING ? "RUNNING" : "STOPPED");
#endif
				/* down -> up */
				if ((ifp->if_flags & IFF_RUNNING) == 0)
					gse_init(ifp);
			} else {
#ifdef DEBUG_GSE
				printf("%s:%d: NOW %s: UP -> DOWN\n", __func__, __LINE__, ifp->if_flags & IFF_RUNNING ? "RUNNING" : "STOPPED");
#endif
				/* up -> down */
				if ((ifp->if_flags & IFF_RUNNING) != 0)
					gse_stop(ifp, 0);
			}
		}

		/* into promisc mode or not */
		if (change & (IFF_PROMISC | IFF_ALLMULTI)) {
			if (ifp->if_flags & (IFF_PROMISC | IFF_ALLMULTI)) {
				switch (scp->sc_portno) {
				case 0:
					GSE_REG_SETBIT(sc, GSE_SW_CFG_REG,
					    GSE_SW_CFG_SKIP_L2_PORT0);
					break;
				case 1:
					GSE_REG_SETBIT(sc, GSE_SW_CFG_REG,
					    GSE_SW_CFG_SKIP_L2_PORT1);
					break;
				}
			} else {
				switch (scp->sc_portno) {
				case 0:
					GSE_REG_CLRBIT(sc, GSE_SW_CFG_REG,
					    GSE_SW_CFG_SKIP_L2_PORT0);
					break;
				case 1:
					GSE_REG_CLRBIT(sc, GSE_SW_CFG_REG,
					    GSE_SW_CFG_SKIP_L2_PORT1);
					break;
				}
			}
			gse_setmulti(scp);
		}
#ifdef DEBUG_GSE
		if (change & IFF_LINK0) {
			gsec_arl_dump(sc);
		}
		if (change & IFF_LINK1) {
			gse_watchdog(ifp);
		}
#endif
		scp->sc_if_flags = ifp->if_flags;
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
		error = ifmedia_ioctl(ifp, ifr, &scp->sc_mii.mii_media, command);
		break;
	default:
		error = ether_ioctl(ifp, command, data);
		if (error != ENETRESET)
			break;

		error = 0;
		switch (command) {
		case SIOCSIFCAP:
			error = (*ifp->if_init)(ifp);
			break;
		case SIOCDELMULTI:
			if (ifp->if_flags & IFF_RUNNING)
				gse_arl_clear(scp);
			/* FALLTHRU */
		case SIOCADDMULTI:
			if (ifp->if_flags & IFF_RUNNING)
				gse_setmulti(scp);
			break;
		default:
			break;
		}
	}
	splx(s);

	return error;
}

static int
gse_ifmedia_upd(struct ifnet *ifp __unused)
{
	struct gse_if_softc *scp;
	struct gsec_softc *sc;

	scp = ifp->if_softc;
	sc = scp->sc_gsec;
	printf("%s: not implemented yet\n", device_xname(sc->sc_dev));

	return 0;
}

static void
gse_ifmedia_sts(struct ifnet *ifp, struct ifmediareq *ifmr)
{
	struct gse_if_softc *scp;
	struct gsec_softc *sc;
	uint32_t val;

	scp = ifp->if_softc;
	sc = scp->sc_gsec;

	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;

	switch (sc->sc_phy_type) {
	case GSEC_PHY_VSC7385:
		val = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    GSE_MAC_PORT0_CFG_REG);
		if (((val & GSE_MAC_PORT_CFG_LINK_ST) != 0) ||
		    ((val & GSE_MAC_PORT_CFG_TXC_ST) == 0)) {
			ifmr->ifm_status |= IFM_ACTIVE;
		}
		switch (val & GSE_MAC_PORT_CFG_SPEED_ST_MASK) {
		case GSE_MAC_PORT_CFG_SPEED_ST_10M:
			ifmr->ifm_active |= IFM_10_T;
			break;
		case GSE_MAC_PORT_CFG_SPEED_ST_100M:
			ifmr->ifm_active |= (val & GSE_MAC_PORT_CFG_DUPLEX_ST) ?
			    IFM_100_FX : IFM_100_TX;
			break;
		case GSE_MAC_PORT_CFG_SPEED_ST_1000N:
			ifmr->ifm_active |= IFM_1000_T;
			break;
		}
		break;
	case GSEC_PHY_INTERNAL:
	default:
		panic("%s: %s: unexpected callback\n", device_xname(sc->sc_dev), __func__);
		break;
	}
}

/*
 * for MII
 */
static int
gse_miibus_readreg(device_t dev, int phy, int reg)
{
	struct gse_if_softc *scif;
	struct gsec_softc *sc;
	int timeout;
	uint32_t val, status;

	scif = device_private(dev);
	sc = scif->sc_gsec;

	/*
	 * read command
	 */

	/* set phy address[5:1] */
	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_TEST1_REG);
	status &= ~GSE_TEST1_PHY_ADDR_MASK;
	status |= GSE_TEST1_PHY_ADDR(phy);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TEST1_REG, status);

	/* set phy address[0] and command */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_PHY_CTRL_REG,
	    GSE_PHY_CTRL_RW_OK | GSE_PHY_CTRL_RD_CMD |
	    GSE_PHY_CTRL_PHY_REG(reg) | GSE_PHY_CTRL_PHY_ADDR(phy));

	for (timeout = 5000; timeout > 0; --timeout) {
		status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_PHY_CTRL_REG);
		if (status & GSE_PHY_CTRL_RW_OK)
			break;
	}
	if (timeout <= 0) {
		aprint_error("%s: MII read timeout: reg=0x%02x\n", device_xname(sc->sc_dev), reg);
		val = -1;
	} else {
		val = (status >> 16) & 0xffff;
	}

#ifdef DEBUG_GSE_DUMP_MII
	printf("%s:%d: phy=%d, reg=%d => val=%08x (status=%08x)\n", __func__, __LINE__, phy, reg, val, status);
#endif

	return val;
}

static void
gse_miibus_writereg(device_t dev, int phy, int reg, int val)
{
	struct gse_if_softc *scif;
	struct gsec_softc *sc;
	int timeout;
	uint32_t status;

	scif = device_private(dev);
	sc = scif->sc_gsec;

#ifdef DEBUG_GSE_DUMP_MII
	printf("%s:%d: phy=%d, reg=%d, val=0x%08x\n", __func__, __LINE__, phy, reg, val);
#endif

	/*
	 * write command
	 */

	/* set phy address[5:1] */
	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_TEST1_REG);
	status &= ~GSE_TEST1_PHY_ADDR_MASK;
	status |= GSE_TEST1_PHY_ADDR(phy);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TEST1_REG, status);

	/* set phy address[0] and command */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_PHY_CTRL_REG,
	    GSE_PHY_CTRL_RW_OK | GSE_PHY_CTRL_WT_CMD |
	    GSE_PHY_CTRL_PHY_REG(reg) | GSE_PHY_CTRL_PHY_ADDR(phy) |
	    ((val & 0xffff) << 16));

	for (timeout = 5000; timeout > 0; --timeout) {
		status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_PHY_CTRL_REG);
		if (status & GSE_PHY_CTRL_RW_OK)
			break;
	}
	if (timeout <= 0) {
		aprint_error("%s: MII write timeout: reg=0x%02x\n", device_xname(sc->sc_dev), reg);
	}
}

static void
gse_miibus_statchg(device_t dev)
{
	struct gse_if_softc *scif;
	struct gsec_softc *sc;

	scif = device_private(dev);
	sc = scif->sc_gsec;

	(void)&scif;
	(void)&sc;
}

/*
 * misc routines
 */
static void
gsec_init_txring(struct gsec_softc *sc)
{
	int i;

#ifdef DEBUG_GSE
	printf("%s:%d\n", __func__, __LINE__);
#endif

	sc->sc_tx_free = GSE_TX_RING_CNT;
	sc->sc_tx_considx = 0;
	sc->sc_tx_prodidx = 0;

	/*
	 * build TX ring
	 */
	for (i = 0; i < GSE_TX_RING_CNT; i++) {
		sc->sc_txdesc_ring[i].tx_sdp = 0;
		sc->sc_txdesc_ring[i].tx_ctrl =
		    GSE_TXDESC_CTRL_COWN |
		    ((i == (GSE_TX_RING_CNT - 1)) ? GSE_TXDESC_CTRL_EOR : 0);
		sc->sc_txdesc_ring[i].tx_vlan = 0;
		sc->sc_txdesc_ring[i].tx_rsvd = 0;
		TXDESC_WRITEOUT(i);
	}
}

static int
gsec_init_rxring(struct gsec_softc *sc)
{
	int i, error;

#ifdef DEBUG_GSE
	printf("%s:%d\n", __func__, __LINE__);
#endif

	/*
	 * build RX ring
	 */
	for (i = 0; i < GSE_RX_RING_CNT; i++) {
		sc->sc_rxsoft[i].rxs_mbuf = NULL;
		error = gsec_alloc_rxbuf(sc, i);
		if (error != 0)
			return error;

#if 0
		/* need to invalidate cache of mbuf cluster ? */
		bus_dmamap_sync(sc->sc_dmat, sc->sc_rxsoft[i].rxs_dmamap, 0,
		    sc->sc_rxsoft[i].rxs_dmamap->dm_mapsize,
		    BUS_DMASYNC_PREREAD);
#endif
	}

	sc->sc_rx_readidx = 0;

	return 0;
}

static void
gsec_reset_rxdesc(struct gsec_softc *sc, int idx)
{
	sc->sc_rxdesc_ring[idx].rx_sdp =
	    sc->sc_rxsoft[idx].rxs_dmamap->dm_segs[0].ds_addr +
	    ETHER_ALIGN;
	sc->sc_rxdesc_ring[idx].rx_ctrl =
	    ((idx == (GSE_RX_RING_CNT - 1)) ? GSE_RXDESC_CTRL_EOR : 0) +
	    GSE_RXDESC_CTRL_SDL(sc->sc_rxsoft[idx].rxs_dmamap->dm_mapsize -
	    ETHER_ALIGN);
	RXDESC_WRITEOUT(idx);

#if 0
	printf("rx[%d]: sdp=%08x, ctrl=%08x\n", idx,
	    sc->sc_rxdesc_ring[idx].rx_sdp,
	    sc->sc_rxdesc_ring[idx].rx_ctrl);
	if (idx == (GSE_RX_RING_CNT - 1)) {
		printf("write EOR in rx_desc\n");
	}
#endif

}

static int
gsec_alloc_rxbuf(struct gsec_softc *sc, int idx)
{
	struct mbuf *m;
	int error;

//printf("%s: idx=%d\n", __func__, idx);

//	KASSERT((idx >= 0) && (idx < GSE_RX_RING_CNT));
//	KASSERT(sc->sc_rxsoft[idx].rxs_mbuf == NULL);

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

	gsec_reset_rxdesc(sc, idx);

	return 0;
}

static void
gsec_drain_txbuf(struct gsec_softc *sc)
{
	struct gse_txsoft *txs;
	struct ifnet *ifp;
	uint32_t ctrl;
	int idx;

#ifdef DEBUG_GSE_DUMP
	printf("%s:%d\n", __func__, __LINE__);
#endif

	for (idx = sc->sc_tx_considx; idx != sc->sc_tx_prodidx;
	    idx = GSE_TX_NEXTIDX(idx)) {

		/* txsoft is used only first segment */
		txs = &sc->sc_txsoft[idx];
		TXDESC_READIN(idx);
		ctrl = sc->sc_txdesc_ring[idx].tx_ctrl;
		if (ctrl & GSE_TXDESC_CTRL_FS) {
			if (ctrl & GSE_TXDESC_CTRL_PMAP_PORT0)
				ifp = &sc->sc_if_gse[0]->sc_ethercom.ec_if;
			else if (ctrl & GSE_TXDESC_CTRL_PMAP_PORT1)
				ifp = &sc->sc_if_gse[1]->sc_ethercom.ec_if;
			else
				panic("%s: nether port0 nor port1 (drain_txbuf)", device_xname(sc->sc_dev));
			ifp->if_oerrors++;

			bus_dmamap_unload(sc->sc_dmat,
			    txs->txs_dmamap);
			m_freem(txs->txs_mbuf);
			txs->txs_mbuf = NULL;
		}
		sc->sc_tx_free++;
	}
}

static void
gsec_drain_rxbuf(struct gsec_softc *sc)
{
	int i;

#ifdef DEBUG_GSE_DUMP
	printf("%s:%d\n", __func__, __LINE__);
#endif

	for (i = 0; i < GSE_RX_RING_CNT; i++) {
		if (sc->sc_rxsoft[i].rxs_mbuf != NULL) {
			bus_dmamap_unload(sc->sc_dmat, sc->sc_rxsoft[i].rxs_dmamap);
			m_freem(sc->sc_rxsoft[i].rxs_mbuf);
			sc->sc_rxsoft[i].rxs_mbuf = NULL;
		}
	}
}

static int
gsec_alloc_ring(struct gsec_softc *sc)
{
	int i, error;

	/*
	 * build DMA maps for TX.
	 * TX descriptor must be able to contain mbuf chains,
	 * So, make up GSE_MAX_PKT_NSEGS dmamap.
	 */
	for (i = 0; i < GSE_TX_RING_CNT; i++) {
		error = bus_dmamap_create(sc->sc_dmat, GSE_MAX_PKT_LEN,
		    GSE_MAX_PKT_NSEGS, GSE_MAX_PKT_LEN, 0, BUS_DMA_NOWAIT,
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
	for (i = 0; i < GSE_RX_RING_CNT; i++) {
		error = bus_dmamap_create(sc->sc_dmat, MCLBYTES,
		    1, MCLBYTES, 0, BUS_DMA_NOWAIT,
		    &sc->sc_rxsoft[i].rxs_dmamap);
		if (error) {
			aprint_error_dev(sc->sc_dev,
			    "can't create DMA map for RX descs\n");
			goto fail_2;
		}
	}

	if (gsec_alloc_dma(sc, sizeof(struct gse_txdesc) * GSE_TX_RING_CNT,
	    (void **)&(sc->sc_txdesc_ring), &(sc->sc_txdesc_dmamap)) != 0)
		return -1;
	memset(sc->sc_txdesc_ring, 0, sizeof(struct gse_txdesc) * GSE_TX_RING_CNT);

	if (gsec_alloc_dma(sc, sizeof(struct gse_rxdesc) * GSE_RX_RING_CNT,
	    (void **)&(sc->sc_rxdesc_ring), &(sc->sc_rxdesc_dmamap)) != 0)
		return -1;
	memset(sc->sc_rxdesc_ring, 0, sizeof(struct gse_rxdesc) * GSE_RX_RING_CNT);

	/* zero padding data */
	if (gsec_alloc_dma(sc, GSE_ETHER_MIN_LEN_VLAN,
	    (void **)&(sc->sc_padseg), &(sc->sc_padseg_dmamap)) != 0)
		return -1;

	memset(sc->sc_padseg, 0, GSE_ETHER_MIN_LEN_VLAN);
	bus_dmamap_sync(sc->sc_dmat, sc->sc_padseg_dmamap, 0,
	    sc->sc_padseg_dmamap->dm_mapsize, BUS_DMASYNC_PREWRITE);

	return 0;

 fail_2:
	for (i = 0; i < GSE_RX_RING_CNT; i++) {
		if (sc->sc_rxsoft[i].rxs_dmamap != NULL)
			bus_dmamap_destroy(sc->sc_dmat,
			    sc->sc_rxsoft[i].rxs_dmamap);
	}
 fail_1:
	for (i = 0; i < GSE_TX_RING_CNT; i++) {
		if (sc->sc_txsoft[i].txs_dmamap != NULL)
			bus_dmamap_destroy(sc->sc_dmat,
			    sc->sc_txsoft[i].txs_dmamap);
	}
	return error;
}

static int
gsec_encap(struct gsec_softc *sc, struct mbuf **m0, int portno)
{
	bus_dmamap_t map;
	struct gse_txsoft *txs;
	struct mbuf *m;
	uint32_t ctrl, ctrlpmap;
	int csumflags, error, i, idx, padsize;

	ctrlpmap = GSE_TXDESC_CTRL_FR |
	    ((portno == 0) ? GSE_TXDESC_CTRL_PMAP_PORT0 : GSE_TXDESC_CTRL_PMAP_PORT1);

	txs = &sc->sc_txsoft[sc->sc_tx_prodidx];
	map = txs->txs_dmamap;

	m = *m0;
	csumflags = m->m_pkthdr.csum_flags;

	error = bus_dmamap_load_mbuf(sc->sc_dmat, map, m,
	    BUS_DMA_NOWAIT);
	if (error != 0) {
		aprint_error_dev(sc->sc_dev,
		    "Error mapping mbuf into TX chain: error=%d\n", error);
		m_freem(m);
		m0 = NULL;
		return error;
	}

	if (map->dm_nsegs > sc->sc_tx_free - GSE_TX_RING_MINIMAL_CNT) {
		bus_dmamap_unload(sc->sc_dmat, map);
		return ENOBUFS;
	}

#ifdef DEBUG_GSE_DUMP
	{
		uint8_t *p;
		int len = m_length(m);
		struct mbuf *tmp_m = m;

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

//	padsize = GSE_ETHER_MIN_LEN_VLAN - m->m_pkthdr.len;
	padsize = GSE_ETHER_MIN_LEN - m->m_pkthdr.len;

	if (padsize > 0) {
		/* need padding */
		idx = GSE_TX_MASKIDX(sc->sc_tx_prodidx + map->dm_nsegs);

		ctrl = ctrlpmap | GSE_TXDESC_CTRL_LS | GSE_TXDESC_CTRL_INT;
		if (idx == (GSE_TX_RING_CNT - 1)) {
			/* end of ring */
			ctrl |= GSE_TXDESC_CTRL_EOR;
		}

		sc->sc_txdesc_ring[idx].tx_vlan = 0;
		sc->sc_txdesc_ring[idx].tx_sdp =
		    sc->sc_padseg_dmamap->dm_segs[0].ds_addr;
		sc->sc_txdesc_ring[idx].tx_ctrl = ctrl |
		    GSE_TXDESC_CTRL_SDL(padsize);
		TXDESC_WRITEOUT(idx);
#ifdef DEBUG_GSE_DUMP
		gse_dump_txdesc(sc, idx, "G_ENCAP(padding):");
#endif
	}

	bus_dmamap_sync(sc->sc_dmat, map, 0, map->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);

	for (i = (map->dm_nsegs - 1); i >= 0; i--) {
		idx = GSE_TX_MASKIDX(sc->sc_tx_prodidx + i);
		ctrl = ctrlpmap;

		if (i == 0) {
			/* mark first segment */
			ctrl |= GSE_TXDESC_CTRL_FS;
			txs->txs_mbuf = m;

			/* checksum offloading */
			if (csumflags & M_CSUM_IPv4)
				ctrl |= GSE_TXDESC_CTRL_ICO;
			if (csumflags & M_CSUM_UDPv4)
				ctrl |= GSE_TXDESC_CTRL_UCO;
			if (csumflags & M_CSUM_TCPv4)
				ctrl |= GSE_TXDESC_CTRL_TCO;

#ifdef DEBUG_GSE_DUMP
			if (csumflags & (M_CSUM_IPv4 | M_CSUM_UDPv4 | M_CSUM_TCPv4)) {
				printf("m=%p, csumflags=%s%s%s\n",
				    m,
				    (csumflags & M_CSUM_IPv4) ? "I" : "",
				    (csumflags & M_CSUM_UDPv4) ? "U" : "",
				    (csumflags & M_CSUM_TCPv4) ? "T" : "");
			}
#endif
		}
		if ((padsize <= 0) && (i == (map->dm_nsegs - 1))) {
			/* mark last segment */
			ctrl |= GSE_TXDESC_CTRL_LS | GSE_TXDESC_CTRL_INT;
		}
		if (idx == (GSE_TX_RING_CNT - 1)) {
			/* mark end of ring */
			ctrl |= GSE_TXDESC_CTRL_EOR;
		}

		sc->sc_txdesc_ring[idx].tx_vlan = 0;
		sc->sc_txdesc_ring[idx].tx_sdp = map->dm_segs[i].ds_addr;
		sc->sc_txdesc_ring[idx].tx_ctrl = ctrl |
		    GSE_TXDESC_CTRL_SDL(map->dm_segs[i].ds_len);
		TXDESC_WRITEOUT(idx);
#ifdef DEBUG_GSE_DUMP
		gse_dump_txdesc(sc, idx, "G_ENCAP         :");
#endif
	}


#ifdef DEBUG_GSE_DUMP
	printf("tx_free=%d\n", sc->sc_tx_free);
#endif

	sc->sc_tx_prodidx += map->dm_nsegs;
	sc->sc_tx_free -= map->dm_nsegs;
	if (padsize > 0) {
		sc->sc_tx_prodidx++;
		sc->sc_tx_free--;
	}
	sc->sc_tx_prodidx = GSE_TX_MASKIDX(sc->sc_tx_prodidx);

	return 0;
}



static void
gsec_init_phy_vsc7385(struct gsec_softc *sc)
{
	uint32_t val;

	/* port0 configuration */
	val =
	    GSE_MAC_PORT_CFG_LEARN_DIS |
	    GSE_MAC_PORT_CFG_BP_EN |
	    GSE_MAC_PORT_CFG_RGMII_PHY |
	    GSE_MAC_PORT_CFG_FORCE_FC_TX |
	    GSE_MAC_PORT_CFG_FORCE_FC_RX |
	    GSE_MAC_PORT_CFG_FORCE_DUPLEX |
	    GSE_MAC_PORT_CFG_FORCE_SPEED(2);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_MAC_PORT0_CFG_REG, val);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_MAC_PORT1_CFG_REG, val);

	/* set TEST Register */
	val = bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_TEST0_REG) &
	    GSE_TEST0_PORT0_TX_SKEW_MASK &
	    GSE_TEST0_PORT0_RX_SKEW_MASK;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TEST0_REG, val |
	    GSE_TEST0_PORT0_TX_SKEW(2) |
	    GSE_TEST0_PORT0_RX_SKEW(2));
}

static int
gsec_init_regs(struct gsec_softc *sc)
{
	paddr_t paddr;
	uint32_t status, val;

#ifdef DEBUG_GSE
	printf("%s:%d\n", __func__, __LINE__);
#endif

	/* activate NIC */
	status = STAR_REG_READ32(ORION_CLKPWR_CLKGATE_REG);
	STAR_REG_WRITE32(ORION_CLKPWR_CLKGATE_REG, status &
	    ~ORION_CLKPWR_CLKGATE_HCLK_SW);

	status = STAR_REG_READ32(ORION_CLKPWR_SOFTRST_REG);
//	STAR_REG_WRITE32(ORION_CLKPWR_SOFTRST_REG, status &
//	    ~ORION_CLKPWR_SOFTRST_SW);
//	delay(100);
	STAR_REG_WRITE32(ORION_CLKPWR_SOFTRST_REG, status |
	    ORION_CLKPWR_SOFTRST_SW);

	status = STAR_REG_READ32(ORION_CLKPWR_PADCTRL_REG);
	STAR_REG_WRITE32(ORION_CLKPWR_PADCTRL_REG, status |
	    ORION_CLKPWR_PADCTRL_P0_LOW_SPEED |
	    ORION_CLKPWR_PADCTRL_P1_LOW_SPEED);

	/* stop DMA transfer */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TX_DMA_CTRL_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_RX_DMA_CTRL_REG, 0);


	/* switch configuration */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_SW_CFG_REG,
	    GSE_SW_CFG_NIC_MODE |
	    GSE_SW_CFG_CRC_STRIP |
	    GSE_SW_CFG_COL_MODE(3) |
	    GSE_SW_CFG_REV_MC_FILTER |
	    GSE_SW_CFG_BP_MODE(2) |
	    GSE_SW_CFG_BP_JAM_NO(10) |
	    GSE_SW_CFG_BKOFF_MODE(7) |
	    GSE_SW_CFG_HASH_ALG(0) |	/* direct mode */
	    GSE_SW_CFG_MAX_LEN(3) |
	    GSE_SW_CFG_AGE_TIME(0));

	/* port0,1 configuration */
	switch (sc->sc_phy_type) {
	case GSEC_PHY_VSC7385:
		gsec_init_phy_vsc7385(sc);
		break;
	case GSEC_PHY_INTERNAL:
	default:
		val =
		    GSE_MAC_PORT_CFG_LEARN_DIS |
		    GSE_MAC_PORT_CFG_BP_EN |
//		    GSE_MAC_PORT_CFG_RGMII_PHY |
//		    GSE_MAC_PORT_CFG_TXC_CHECK_EN |
//		    GSE_MAC_PORT_CFG_FORCE_FC_TX |
//		    GSE_MAC_PORT_CFG_FORCE_FC_RX |
//		    GSE_MAC_PORT_CFG_FORCE_DUPLEX |
//		    GSE_MAC_PORT_CFG_FORCE_SPEED(2) |
		    GSE_MAC_PORT_CFG_AN_EN;
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_MAC_PORT0_CFG_REG, val);
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_MAC_PORT1_CFG_REG, val);
		break;
	}

	/* cpu port configuration */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_CPU_PORT_CFG_REG,
#if ETHER_ALIGN == 0
	    GSE_CPU_PORT_CFG_OFFSET_2B |
#endif
	    GSE_CPU_PORT_CFG_LEARN_DIS |
	    0);

	/* auto polling phy address, etc */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TEST1_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_AUTOPOLL_PHYADDR_REG, 0x00000040);


	/* priority setup */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_PRI_CTRL_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_UDP_PRI_REG, 0x0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_IP_TOS0_PRI_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_IP_TOS1_PRI_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_IP_TOS2_PRI_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_IP_TOS3_PRI_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_IP_TOS4_PRI_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_IP_TOS5_PRI_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_IP_TOS6_PRI_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_IP_TOS7_PRI_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_SCHED_CTRL_REG, 0);

	/* rate limit */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_RATE_LIMIT_CTRL_REG, 0);

	/* flow control */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_FLOW_CTRL_GLOBAL_THR_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_FLOW_CTRL_PORT_THR_REG, 0x07070707);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_SMART_FLOW_CTRL_REG, 0x000002ac);

	/* Port VID */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_PORT_VID_REG, 0x00000000);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_VLAN_GID01, 0x00002001);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_VLAN_GID23, 0x00004003);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_VLAN_GID45, 0x00006005);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_VLAN_GID67, 0x00008007);
//	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_VLAN_PORT_MAP_REG, 0x00000007);
//	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_VLAN_TAG_PORT_TAG_REG, 0x00000007);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_SESSION_ID01_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_SESSION_ID23_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_SESSION_ID45_REG, 0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_SESSION_ID67_REG, 0);

	/* TX Descriptor Physical Address */
	paddr = sc->sc_txdesc_dmamap->dm_segs[0].ds_addr;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TX_DPTR_REG, paddr);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_TX_BASE_ADDR_REG, paddr);

	/* RX Descriptor Physical Address */
	paddr = sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_RX_DPTR_REG, paddr);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_RX_BASE_ADDR_REG, paddr);

	/* sync cache */
	bus_dmamap_sync(sc->sc_dmat, sc->sc_txdesc_dmamap, 0,
	    sc->sc_txdesc_dmamap->dm_mapsize, BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(sc->sc_dmat, sc->sc_rxdesc_dmamap, 0,
	    sc->sc_rxdesc_dmamap->dm_mapsize, BUS_DMASYNC_PREWRITE);

	/* interrupt mask */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_INT_MASK_REG,
	    GEC_INT_PORT1_RMC_PAUSE |
	    GEC_INT_PORT1_NO_DEST |
	    GEC_INT_PORT0_RMC_PAUSE |
	    GEC_INT_PORT0_NO_DEST |
	    0);

	/* Delayed interrupt configuration is not used */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, GSE_DLY_INT_CFG_REG, 0);

#ifdef DEBUG_GSE
	(void)gse_dump_txdesc;
	(void)gse_dump_rxdesc;
#endif

	return 0;
}

static int
gsec_alloc_dma(struct gsec_softc *sc, size_t size, void **addrp,
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


#ifdef DEBUG_GSE
static void
gse_dump_txdesc(struct gsec_softc *sc, int idx, const char *prefix)
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
	    "\33FP"
	    "\32PRI2"
	    "\31PRI1"
	    "\30PRI0"
	    "\27FR"
	    "\26PMAP2"
	    "\25PMAP1"
	    "\24PMAP0"
	    "\23ICO"
	    "\22UCO"
	    "\21TCO",
	    sc->sc_txdesc_ring[idx].tx_ctrl);

	printf("%stxdesc[%3d]:tx_sdp=0x%08x, tx_ctrl=%s\n", prefix, idx,
	    sc->sc_txdesc_ring[idx].tx_sdp, tmpbuf);
}

static void
gse_dump_rxdesc(struct gsec_softc *sc, int idx, const char *prefix)
{
	char tmpbuf[128];

	RXDESC_READIN(idx);

	snprintb(tmpbuf, sizeof(tmpbuf), 
	    "\20"
	    "\40COWN"
	    "\37EOR"
	    "\36FS"
	    "\35LS"
	    "\34SP1"
	    "\33SP0"
	    "\32HR5"
	    "\31HR4"
	    "\30HR3"
	    "\27HR2"
	    "\26HR1"
	    "\25HR0"
	    "\24PROTO1"
	    "\23PROTO0"
	    "\22IPF"
	    "\21L4F",
	    sc->sc_rxdesc_ring[idx].rx_ctrl);

	printf("%srxdesc[%3d]:rx_sdp=0x%08x, rx_ctrl=%s\n", prefix, idx,
	    sc->sc_rxdesc_ring[idx].rx_sdp, tmpbuf);

#if 0 /* HEXDUMP */
	{
		unsigned char *p;
		int i;

		p = mtod(sc->sc_rxsoft[idx].rxs_mbuf, char *);
		for (i = 0; i < 128; i++) {
			if ((i & 15) == 0)
				printf("%04x:", i);

			printf(" %02x", p[i]);

			if ((i & 15) == 15)
				printf("\n");
		}
	}
#endif
}

static void
gse_dumpall_txdesc(struct gsec_softc *sc, const char *prefix)
{
	int i;

	printf("%s tx_dbase=%08x, tx_dptr=%08x, idx=%lu\n", prefix,
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_TX_BASE_ADDR_REG),
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_TX_DPTR_REG),
	    RX_DPTR2IDX(bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_TX_DPTR_REG)));
	for (i = 0; i < GSE_TX_RING_CNT; i++)
		gse_dump_txdesc(sc, i, prefix);
}

static void
gse_dumpall_rxdesc(struct gsec_softc *sc, const char *prefix)
{
	int i;

	printf("%s rx_dbase=%08x, rx_dptr=%08x, idx=%lu\n", prefix,
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_RX_BASE_ADDR_REG),
	    bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_RX_DPTR_REG),
	    RX_DPTR2IDX(bus_space_read_4(sc->sc_iot, sc->sc_ioh, GSE_RX_DPTR_REG)));
	for (i = 0; i < GSE_RX_RING_CNT; i++)
		gse_dump_rxdesc(sc, i, prefix);
}
#endif
