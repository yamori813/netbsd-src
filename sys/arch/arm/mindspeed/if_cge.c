/*	$NetBSD$	*/

/*
 * Copyright (c) 2022 Hiroki Mori
 * Copyright (c) 2013 Jonathan A. Kollasch
 * Copyright (c) 2012 Damjan Marion <dmarion@Freebsd.org>
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

/* Mindspeed Comcerto 1000 GEMAC Interface. based on ti/if_cpsw.c */

#include <sys/cdefs.h>
__KERNEL_RCSID(1, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/ioctl.h>
#include <sys/intr.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/systm.h>
#include <sys/kernel.h>

#include <net/if.h>
#include <net/if_ether.h>
#include <net/if_media.h>
#include <net/bpf.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <arm/mindspeed/m83xxx_reg.h>
#include <arm/mindspeed/m83xxx_var.h>
#include <arm/mindspeed/m83xxx_intc.h>
#include <arm/mindspeed/if_cgereg.h>
#include <arm/mindspeed/arswitchreg.h>

#define CGE_TXFRAGS	16

static int cge_match(device_t, cfdata_t, void *);
static void cge_attach(device_t, device_t, void *);
static int cge_detach(device_t, int);

static void cge_start(struct ifnet *);
static int cge_ioctl(struct ifnet *, u_long, void *);
static void cge_watchdog(struct ifnet *);
static int cge_init(struct ifnet *);
static void cge_stop(struct ifnet *, int);

static int cge_mii_readreg(device_t, int, int, uint16_t *);
static int cge_mii_writereg(device_t, int, int, uint16_t);
static void cge_mii_statchg(struct ifnet *);

static int cge_new_rxbuf(struct cge_softc * const, const u_int);
//static void cge_tick(void *);

static int cge_intr(void *);
static int cge_rxintr(void *);
static int cge_txintr(void *);

#define TXDESC_NEXT(x) cge_txdesc_adjust((x), 1)
#define TXDESC_PREV(x) cge_txdesc_adjust((x), -1)

#define RXDESC_NEXT(x) cge_rxdesc_adjust((x), 1)
#define RXDESC_PREV(x) cge_rxdesc_adjust((x), -1)

static int cge_alloc_dma(struct cge_softc *, size_t, void **,bus_dmamap_t *);

CFATTACH_DECL_NEW(cge, sizeof(struct cge_softc),
    cge_match, cge_attach, cge_detach, NULL);

int arswitch_readreg(device_t dev, int addr);
int arswitch_writereg(device_t dev, int addr, int value);

void arswitch_writedbg(device_t dev, int phy, uint16_t dbg_addr,
    uint16_t dbg_data);

void
arswitch_writedbg(device_t dev, int phy, uint16_t dbg_addr,
    uint16_t dbg_data)
{
/*
        (void) MDIO_WRITEREG(device_get_parent(dev), phy,
            MII_ATH_DBG_ADDR, dbg_addr);
        (void) MDIO_WRITEREG(device_get_parent(dev), phy,
            MII_ATH_DBG_DATA, dbg_data);
*/
	cge_mii_writereg(dev, phy, MII_ATH_DBG_ADDR, dbg_addr);
	cge_mii_writereg(dev, phy, MII_ATH_DBG_DATA, dbg_data);
}

static inline void
arswitch_split_setpage(device_t dev, uint32_t addr, uint16_t *phy,
    uint16_t *reg)
{
//	struct arswitch_softc *sc = device_get_softc(dev);
	uint16_t page;

	page = (addr >> 9) & 0x1ff;
	*phy = (addr >> 6) & 0x7;
	*reg = (addr >> 1) & 0x1f;

//	if (sc->page != page) {
//		MDIO_WRITEREG(device_get_parent(dev), 0x18, 0, page);
		cge_mii_writereg(dev, 0x18, 0, page);
		delay(2000);
//		sc->page = page;
//	}
}

static inline u_int
cge_txdesc_adjust(u_int x, int y)
{
//	return (((x) + y) & (CGE_TX_RING_CNT - 1));
	int res = x + y + CGE_TX_RING_CNT;
	return res % CGE_TX_RING_CNT;
}

static inline u_int
cge_rxdesc_adjust(u_int x, int y)
{
//	return (((x) + y) & (CGE_RX_RING_CNT - 1));
	int res = x + y + CGE_RX_RING_CNT;
	return res % CGE_RX_RING_CNT;
}

static inline uint32_t
cge_read_4(struct cge_softc * const sc, bus_size_t const offset)
{
	return bus_space_read_4(sc->sc_bst, sc->sc_bsh, offset);
}

static inline void
cge_write_4(struct cge_softc * const sc, bus_size_t const offset,
    uint32_t const value)
{
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, offset, value);
}

static int
cge_match(device_t parent, cfdata_t cf, void *aux)
{

	return 1;
}

static int
cge_detach(device_t self, int flags)
{
	struct cge_softc * const sc = device_private(self);
	struct ifnet *ifp = &sc->sc_ec.ec_if;
	u_int i;

	/* Succeed now if there's no work to do. */
	if (!sc->sc_attached)
		return 0;

	sc->sc_attached = false;

	/* Stop the interface. Callouts are stopped in it. */
	cge_stop(ifp, 1);

#if 0
	/* Destroy our callout. */
	callout_destroy(&sc->sc_tick_ch);
#endif

	/* Let go of the interrupts */
	intr_disestablish(sc->sc_ih);

	ether_ifdetach(ifp);
	if_detach(ifp);

	/* Delete all media. */
	ifmedia_fini(&sc->sc_mii.mii_media);

	/* Destroy all the descriptors */
	for (i = 0; i < CGE_TX_RING_CNT; i++)
		bus_dmamap_destroy(sc->sc_bdt, sc->sc_rdp->tx_dm[i]);
	for (i = 0; i < CGE_RX_RING_CNT; i++)
		bus_dmamap_destroy(sc->sc_bdt, sc->sc_rdp->rx_dm[i]);
	kmem_free(sc->sc_rdp, sizeof(*sc->sc_rdp));

	/* Unmap */
	bus_space_unmap(sc->sc_bst, sc->sc_bsh, sc->sc_bss);

	return 0;
}

static void
cge_attach(device_t parent, device_t self, void *aux)
{
	struct apb_attach_args * const aa = aux;
	struct cge_softc * const sc = device_private(self);
	int i, error;
	struct ethercom * const ec = &sc->sc_ec;
	struct ifnet * const ifp = &ec->ec_if;
	struct mii_data * const mii = &sc->sc_mii;

	sc->sc_dev = self;
	sc->sc_bst = aa->apba_memt;
	sc->sc_bdt = aa->apba_dmat;
	sc->sc_bss = 0x10000;

	aprint_normal(": GEMAC Interface\n");
	aprint_naive("\n");

	mutex_init(&sc->mtx, MUTEX_DEFAULT, IPL_NET);
//	callout_init(&sc->sc_tick_ch, 0);
//	callout_setfunc(&sc->sc_tick_ch, cge_tick, sc);

	error = bus_space_map(aa->apba_memt, aa->apba_addr, sc->sc_bss,
	    0, &sc->sc_bsh);
	if (error) {
		aprint_error_dev(sc->sc_dev,
			"can't map registers: %d\n", error);
		return;
	}

	sc->sc_ih = intr_establish(aa->apba_intr, IPL_NET, IST_LEVEL,
	    cge_intr, sc);

	sc->sc_enaddr[0] = 0xd4;
	sc->sc_enaddr[1] = 0x94;
	sc->sc_enaddr[2] = 0xa1;
	sc->sc_enaddr[3] = 0x97;
	sc->sc_enaddr[4] = 0x03;
	sc->sc_enaddr[5] = 0x94 + device_unit(self);

	sc->sc_rdp = kmem_alloc(sizeof(*sc->sc_rdp), KM_SLEEP);

	for (i = 0; i < CGE_TX_RING_CNT; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES,
		    CGE_TXFRAGS, MCLBYTES, 0, 0,
		    &sc->sc_rdp->tx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create tx DMA map: %d\n", error);
		}
		sc->sc_rdp->tx_mb[i] = NULL;
	}

	for (i = 0; i < CGE_RX_RING_CNT; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES, 1,
		    MCLBYTES, 0, 0, &sc->sc_rdp->rx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create rx DMA map: %d\n", error);
		}
		sc->sc_rdp->rx_mb[i] = NULL;
	}

	if (cge_alloc_dma(sc, sizeof(struct tRXdesc) * CGE_RX_RING_CNT,
	    (void **)&(sc->sc_rxdesc_ring), &(sc->sc_rxdesc_dmamap)) != 0)
		return;

	if (cge_alloc_dma(sc, sizeof(struct tTXdesc) * CGE_TX_RING_CNT,
	    (void **)&(sc->sc_txdesc_ring), &(sc->sc_txdesc_dmamap)) != 0)
		return;

	memset(sc->sc_rxdesc_ring, 0, sizeof(struct tRXdesc) * CGE_RX_RING_CNT);

	for (i = 0; i < CGE_TX_RING_CNT; i++) {
		sc->sc_txdesc_ring[i].tx_data = 0;
		sc->sc_txdesc_ring[i].tx_ctl = GEMTX_USED_MASK;
		if (i == CGE_TX_RING_CNT - 1)
			sc->sc_txdesc_ring[i - 1].tx_ctl |= GEMTX_WRAP;
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

	aprint_normal_dev(sc->sc_dev, "Ethernet address %s\n",
	    ether_sprintf(sc->sc_enaddr));

	strlcpy(ifp->if_xname, device_xname(sc->sc_dev), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_capabilities = 0;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_start = cge_start;
	ifp->if_ioctl = cge_ioctl;
	ifp->if_init = cge_init;
	ifp->if_stop = cge_stop;
	ifp->if_watchdog = cge_watchdog;
	IFQ_SET_READY(&ifp->if_snd);

	cge_stop(ifp, 0);

	mii->mii_ifp = ifp;
	mii->mii_readreg = cge_mii_readreg;
	mii->mii_writereg = cge_mii_writereg;
	mii->mii_statchg = cge_mii_statchg;

	sc->sc_ec.ec_mii = mii;
	ifmedia_init(&mii->mii_media, 0, ether_mediachange, ether_mediastatus);

	if (device_unit(self) == 0) {
		arswitch_writereg(self, AR8X16_REG_MODE,
		    AR8X16_MODE_RGMII_PORT4_ISO);
		/* work around for phy4 rgmii mode */
		arswitch_writedbg(self, 4, 0x12, 0x480c);
		/* rx delay */
		arswitch_writedbg(self, 4, 0x0, 0x824e);
		/* tx delay */
		arswitch_writedbg(self, 4, 0x5, 0x3d47);
		delay(1000);    /* 1ms, again to let things settle */ 
/* not work strang behaviour ???
		arswitch_writereg(self, AR8X16_REG_MASK_CTRL,
		    AR8X16_MASK_CTRL_SOFT_RESET);
		delay(1000);
*/
#ifdef _DEBUG
		for (i = 0;i < 6; ++i)
		aprint_normal_dev(sc->sc_dev, "PORT STS %d %x\n", i,
		    arswitch_readreg(self, 0x100 * (i + 1)));
		for (i = 0;i < 6; ++i)
		aprint_normal_dev(sc->sc_dev, "PORT CTRL %d %x\n", i,
		    arswitch_readreg(self, 0x100 * (i + 1) + 4));
		for (i = 0;i < 6; ++i)
		aprint_normal_dev(sc->sc_dev, "PORT VLAN %d %x\n", i,
		    arswitch_readreg(self, 0x100 * (i + 1) + 8));
#endif
		aprint_normal_dev(sc->sc_dev, "arswitch %x mode %x\n",
		    arswitch_readreg(self, AR8X16_REG_MASK_CTRL),
		    arswitch_readreg(self, AR8X16_REG_MODE));
	}

	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr);

	/* The attach is successful. */
	sc->sc_attached = true;

	return;
}

static void
cge_start(struct ifnet *ifp)
{
	struct cge_softc * const sc = ifp->if_softc;
	struct cge_ring_data * const rdp = sc->sc_rdp;
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

	CGE_LOCK(sc);

	if (__predict_false((ifp->if_flags & IFF_RUNNING) == 0)) {
		device_printf(sc->sc_dev, "if not run\n");
		return;
	}
	if (__predict_false(sc->sc_txbusy)) {
		device_printf(sc->sc_dev, "txbusy\n");
		return;
	}

	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    0, sizeof(struct tTXdesc) * CGE_TX_RING_CNT,
	    BUS_DMASYNC_PREREAD);
	txfree = 0;
	for (i = 0;i < CGE_TX_RING_CNT; ++i) {
		if(sc->sc_txdesc_ring[i].tx_ctl & GEMTX_USED_MASK)
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
		pad = mlen < CGE_MIN_FRAMELEN;

		KASSERT(rdp->tx_mb[sc->sc_txnext] == NULL);
		rdp->tx_mb[sc->sc_txnext] = m;
		IFQ_DEQUEUE(&ifp->if_snd, m);

		bus_dmamap_sync(sc->sc_bdt, dm, 0, dm->dm_mapsize,
		    BUS_DMASYNC_PREWRITE);

		if (txstart == -1)
			txstart = sc->sc_txnext;
		for (seg = 0; seg < dm->dm_nsegs; seg++) {
			sc->sc_txdesc_ring[sc->sc_txnext].tx_data =
			    dm->dm_segs[seg].ds_addr;
			sc->sc_txdesc_ring[sc->sc_txnext].tx_ctl =
			    dm->dm_segs[seg].ds_len;
			sc->sc_txdesc_ring[sc->sc_txnext].tx_ctl |=
				    (GEMTX_FCS | GEMTX_IE | GEMTX_POOLB);
			len += dm->dm_segs[seg].ds_len;
			if (!pad && (seg == dm->dm_nsegs - 1))
				sc->sc_txdesc_ring[sc->sc_txnext].tx_ctl |=
				    GEMTX_LAST;
			if(seg == 0)
				sc->sc_txdesc_ring[sc->sc_txnext].tx_ctl |=
				    GEMTX_BUFRET;
			if (sc->sc_txnext == CGE_TX_RING_CNT - 1)
				sc->sc_txdesc_ring[sc->sc_txnext].tx_ctl |=
				    GEMTX_WRAP;

			txfree--;
			bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
			    sizeof(struct tTXdesc) * sc->sc_txnext,
			    sizeof(struct tTXdesc), BUS_DMASYNC_PREWRITE);
			sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
		}
		if (pad) {
			sc->sc_txdesc_ring[sc->sc_txnext].tx_data =
			    sc->sc_txpad_pa;
			sc->sc_txdesc_ring[sc->sc_txnext].tx_ctl =
			    CGE_MIN_FRAMELEN - mlen;
			len += CGE_MIN_FRAMELEN - mlen;
			sc->sc_txdesc_ring[sc->sc_txnext].tx_ctl |=
			    (GEMTX_FCS | GEMTX_LAST | GEMTX_IE | GEMTX_POOLB);
			if (sc->sc_txnext == CGE_TX_RING_CNT - 1)
				sc->sc_txdesc_ring[sc->sc_txnext].tx_ctl |=
				    GEMTX_WRAP;
			txfree--;
			bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
			    sizeof(struct tTXdesc) * sc->sc_txnext,
			    sizeof(struct tTXdesc), BUS_DMASYNC_PREWRITE);
			sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
		}
		cge_write_4(sc, GEM_SCH_BLOCK + SCH_PACKET_QUEUED, len);
		bpf_mtap(ifp, m, BPF_D_OUT);
	}

	if (txstart >= 0) {
		ifp->if_timer = 300;	/* not immediate interrupt */
	}

	CGE_UNLOCK(sc);
}

static int
cge_ioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	struct cge_softc * const sc = ifp->if_softc;
	const int s = splnet();
	int error = 0;
	int reg;

	switch (cmd) {
	case SIOCSIFFLAGS:
		if ((ifp->if_flags & (IFF_PROMISC | IFF_ALLMULTI)) != 0) {
			reg = cge_read_4(sc, GEM_IP + GEM_NET_CONTROL);
			reg |= GEM_COPY_ALL;
			cge_write_4(sc, GEM_IP + GEM_NET_CONTROL, reg);
		} else {
			reg = cge_read_4(sc, GEM_IP + GEM_NET_CONTROL);
			reg &= ~GEM_COPY_ALL;
			cge_write_4(sc, GEM_IP + GEM_NET_CONTROL, reg);
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
cge_watchdog(struct ifnet *ifp)
{
	struct cge_softc *sc = ifp->if_softc;

	device_printf(sc->sc_dev, "device timeout %d\n", sc->sc_txnext);

	if_statinc(ifp, if_oerrors);
#if 0
	ifp->if_flags |= IFF_RUNNING;
	cge_init(ifp);
	cge_start(ifp);
#endif
}

static int
cge_mii_readreg(device_t dev, int phy, int reg, uint16_t *val)
{
	struct cge_softc * const sc = device_private(dev);
	int result, wdata;

	wdata = 0x60020000;
	wdata |= ((phy << 23) | (reg << 18));
	cge_write_4(sc, GEM_IP + GEM_PHY_MAN, wdata);
	while(!(cge_read_4(sc, GEM_IP + GEM_NET_STATUS) & GEM_PHY_IDLE))
		;
	result = cge_read_4(sc, GEM_IP + GEM_PHY_MAN);

	*val = (uint16_t)result;
	return 0;
}

static int
cge_mii_writereg(device_t dev, int phy, int reg, uint16_t val)
{
	struct cge_softc * const sc = device_private(dev);
	int wdata;

	wdata = 0x50020000;
	wdata |= ((phy << 23) | (reg << 18) | val);
	cge_write_4(sc, GEM_IP + GEM_PHY_MAN, wdata);
	while(!(cge_read_4(sc, GEM_IP + GEM_NET_STATUS) & GEM_PHY_IDLE))
		;
	return 0;
}

static void
cge_mii_statchg(struct ifnet *ifp)
{
	return;
}

static int
cge_new_rxbuf(struct cge_softc * const sc, const u_int i)
{
	struct cge_ring_data * const rdp = sc->sc_rdp;
//	const u_int h = RXDESC_PREV(i);
//	struct cge_cpdma_bd bd;
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
	sc->sc_rxdesc_ring[i].rx_data = rdp->rx_dm[i]->dm_segs[0].ds_addr;
	sc->sc_rxdesc_ring[i].rx_status = RX_INT;
	sc->sc_rxdesc_ring[i].rx_extstatus = 0;

	if (i == CGE_RX_RING_CNT - 1)
		sc->sc_rxdesc_ring[i].rx_status |= GEMRX_WRAP;

	bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
	    sizeof(struct tRXdesc) * i, sizeof(struct tRXdesc),
            BUS_DMASYNC_PREWRITE);

	return error;
}

static int
cge_init(struct ifnet *ifp)
{
	struct cge_softc * const sc = ifp->if_softc;
	int i;
	int reg;
	int mac;
	paddr_t paddr;

	cge_stop(ifp, 0);

	sc->sc_txnext = 0;

	/* Init circular RX list. */
	for (i = 0; i < CGE_RX_RING_CNT; i++) {
		cge_new_rxbuf(sc, i);
	}
	sc->sc_rxhead = 0;

	/*
	 * Give the transmit and receive rings to the chip.
	 */
	paddr = sc->sc_txdesc_dmamap->dm_segs[0].ds_addr;
	cge_write_4(sc, GEM_IP + GEM_QUEUE_BASE0, paddr);
	paddr = sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr;
	cge_write_4(sc, GEM_IP + GEM_RX_QPTR, paddr);

	mac = sc->sc_enaddr[0] | (sc->sc_enaddr[1] << 8) |
	    (sc->sc_enaddr[2] << 16) | (sc->sc_enaddr[3] << 24);
	cge_write_4(sc, GEM_IP + GEM_LADDR1_BOT, mac);
	mac = sc->sc_enaddr[4] | (sc->sc_enaddr[5] << 8);
	cge_write_4(sc, GEM_IP + GEM_LADDR1_TOP, mac);

//	cge_write_4(sc, GEM_SCH_BLOCK + SCH_CONTROL, 1);

#define DEFAULT_RX_COAL_TIME   500 // us
	cge_write_4(sc, GEM_ADM_BLOCK + GEM_ADM_BLOCK,
	    DEFAULT_RX_COAL_TIME * 125);
#define DEFAULT_RX_COAL_PKTS     8
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_BATCHINTRPKTTHRES,
	    DEFAULT_RX_COAL_PKTS);

	cge_write_4(sc, GEM_ADM_BLOCK + ADM_CNFG, 0x84210030);
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_CNFG, 0x000000aa);

	reg = cge_read_4(sc, GEM_ADM_BLOCK + ADM_CONTROL);
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_CONTROL, reg & ~1);

	delay(1000);

	/* XXX 1G support */

	reg = cge_read_4(sc, GEM_IP + GEM_NET_CONFIG);
	/* 
	 u-boot:160080 GEM_MDC_DIV:5, GEM_RX_NO_FCS, GEM_COPY_ALL
	 reset:140000 GEM_MDC_DIV:5
	*/
	reg |= (GEM_COPY_ALL | GEM_RX_NO_FCS);
	cge_write_4(sc, GEM_IP + GEM_NET_CONFIG, reg);

	reg = cge_read_4(sc, GEM_CFG);
	/*
	 u-boot:15f01 GEM_CONF_MODE_SEL_GEM, GEM_CONF_DUPLEX_SEL_GEMi
	 GEM_CONF_DUPLEX_GEM_FULL, GEM_CONF_DUPLEX_PHY_FULL,
	 GEM_CONF_SPEED_SEL_GEM, GEM_CONF_SPEED_GEM_100M,
	 GEM_CONF_SPEED_PHY_100M, GEM_CONF_PHY_LINK_UP
	 reset:16f01 GEM_CONF_SPEED_GEM_1G
	*/

	reg &= ~GEM_CONF_SPEED_MASK;
	reg |= GEM_CONF_SPEED_GEM_1G;
	reg |= GEM_CONF_SPEED_PHY_1G;
	reg &= ~GEM_CONF_MODE_GEM_MASK;
	reg |= GEM_CONF_MODE_GEM_RGMII;
	reg |= GEM_CONF_MODE_SEL_GEM;
	cge_write_4(sc, GEM_CFG, reg);

	/*
	 * Initialize DMA
	 */

	reg = cge_read_4(sc, GEM_IP + GEM_DMA_CONFIG);
	reg |=(1UL<<31); //enable scheduler
	reg &= ~((1UL<<26) | (1UL<<25)); //hardware buffer allocation
	reg |=(1UL<<12); //enable scheduler
	reg &= ~(0x00FF001F); // enable admittance manager
	reg |= 0x00200000; // set buffer size to 2048 bytes
	reg |= 0x00000010; // Attempt to use INCR16 AHB bursts
	reg |=  GEM_RX_SW_ALLOC;
	cge_write_4(sc, GEM_IP + GEM_DMA_CONFIG, reg);
	/* Disabling GEM delay */
	cge_write_4(sc, 0xf00c, 0);
 
	/* Enable the transmit and receive circuitry */
	reg = cge_read_4(sc, GEM_IP + GEM_NET_CONTROL);
	reg |= (GEM_TX_START | GEM_TX_EN | GEM_RX_EN);
	cge_write_4(sc, GEM_IP + GEM_NET_CONTROL, reg);

	cge_write_4(sc, GEM_SCH_BLOCK + SCH_CONTROL, 3);

	/*
	 * Initialize the interrupt mask and enable interrupts.
	 */
//	cge_write_4(sc, GEM_IP + GEM_IRQ_ENABLE, GEM_IRQ_ALL);
	cge_write_4(sc, GEM_IP + GEM_IRQ_ENABLE,
	    GEM_IRQ_RX_DONE | GEM_IRQ_TX_DONE);
	cge_write_4(sc, GEM_IP + GEM_IRQ_MASK, 0);

	ifp->if_flags |= IFF_RUNNING;

	return 0;
}

static void
cge_stop(struct ifnet *ifp, int disable)
{
	struct cge_softc * const sc = ifp->if_softc;
	struct cge_ring_data * const rdp = sc->sc_rdp;
	u_int i;
	int reg;

	aprint_debug_dev(sc->sc_dev, "%s: ifp %p disable %d\n", __func__,
	    ifp, disable);

	if ((ifp->if_flags & IFF_RUNNING) == 0)
		return;

//	callout_stop(&sc->sc_tick_ch);
	mii_down(&sc->sc_mii);

	cge_write_4(sc, GEM_IP + GEM_IRQ_ENABLE, 0);

	reg = cge_read_4(sc, GEM_IP + GEM_NET_CONTROL);
	reg &= ~(GEM_TX_EN | GEM_RX_EN);
	cge_write_4(sc, GEM_IP + GEM_NET_CONTROL, reg);

	/* Release any queued transmit buffers. */
	for (i = 0; i < CGE_TX_RING_CNT; i++) {
		if (rdp->tx_mb[i] != NULL) {
			bus_dmamap_unload(sc->sc_bdt, rdp->tx_dm[i]);
			m_freem(rdp->tx_mb[i]);
			rdp->tx_mb[i] = NULL;
		}
	}

	ifp->if_flags &= ~IFF_RUNNING;
	ifp->if_timer = 0;
	sc->sc_txbusy = false;

	if (!disable)
		return;

	for (i = 0; i < CGE_RX_RING_CNT; i++) {
		bus_dmamap_unload(sc->sc_bdt, rdp->rx_dm[i]);
		m_freem(rdp->rx_mb[i]);
		rdp->rx_mb[i] = NULL;
	}
}

static int
cge_intr(void *arg)
{
	struct cge_softc * const sc = arg;
	int reg;

	reg = cge_read_4(sc, GEM_IP + GEM_IRQ_STATUS);

	if (reg & GEM_IRQ_RX_DONE)
		cge_rxintr(arg);
//	else
	if (reg & GEM_IRQ_TX_DONE)
		cge_txintr(arg);

	cge_write_4(sc, GEM_IP + GEM_IRQ_STATUS, reg);

	return 1;
}

#if 0
static void
cge_tick(void *arg)
{
	struct cge_softc * const sc = arg;
	struct mii_data * const mii = &sc->sc_mii;
	const int s = splnet();

	mii_tick(mii);

	splx(s);

	callout_schedule(&sc->sc_tick_ch, hz);
}
#endif

static int
cge_rxintr(void *arg)
{
	struct cge_softc * const sc = arg;
	struct cge_ring_data * const rdp = sc->sc_rdp;
	u_int i;
	int length;
	struct mbuf *m;
	struct ifnet *ifp = &sc->sc_ec.ec_if;
	int count;

	count = 0;
	for (;;) {
		i = sc->sc_rxhead;
		bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
		    sizeof(struct tRXdesc) * i, sizeof(struct tRXdesc),
		    BUS_DMASYNC_PREREAD);
		if(sc->sc_rxdesc_ring[i].rx_extstatus & GEMRX_OWN) {
			length = sc->sc_rxdesc_ring[i].rx_status &
			    RX_STA_LEN_MASK;
			cge_write_4(sc, GEM_ADM_BLOCK + ADM_PKTDQ,
			    length);
			m = rdp->rx_mb[i];
			if (length < ETHER_HDR_LEN) {
				aprint_error_dev(sc->sc_dev,
				    "RX error packe length\n");
				m_freem(m);
			} else {
				bus_dmamap_sync(sc->sc_bdt, rdp->rx_dm[i],
				    0, rdp->rx_dm[i]->dm_mapsize,
				    BUS_DMASYNC_POSTREAD);
				m_set_rcvif(m, ifp);
				m->m_pkthdr.len = m->m_len = length;
				if_percpuq_enqueue(ifp->if_percpuq, m);
			}
			cge_new_rxbuf(sc, i);
			
			bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
			    sizeof(struct tRXdesc) * i, sizeof(struct tRXdesc),
       			    BUS_DMASYNC_PREWRITE);
			sc->sc_rxhead = RXDESC_NEXT(sc->sc_rxhead);
			++count;
		} else {
			/* may be not happen */
			if (count == 0)
				panic("rxintr occur but no data\n");
			break;
		}
	}

	return 1;
}

static int
cge_txintr(void *arg)
{
	struct cge_softc * const sc = arg;
	struct cge_ring_data * const rdp = sc->sc_rdp;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	bool handled = false;
	int i, remain;

	remain = 0;
	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    0, sizeof(struct tTXdesc) * CGE_TX_RING_CNT, BUS_DMASYNC_PREREAD);
	for (i = 0; i < CGE_TX_RING_CNT; i++) {
		if (!(sc->sc_txdesc_ring[i].tx_ctl & GEMTX_USED_MASK)) {
			++remain;
			continue;
		}

		if ((sc->sc_txdesc_ring[i].tx_ctl & GEMTX_BUFRET) == 0)
			continue;
		bus_dmamap_sync(sc->sc_bdt, rdp->tx_dm[i],
		    0, rdp->tx_dm[i]->dm_mapsize,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_bdt, rdp->tx_dm[i]);

		m_freem(rdp->tx_mb[i]);
		rdp->tx_mb[i] = NULL;

		if_statinc(ifp, if_opackets);

		handled = true;

		sc->sc_txbusy = false;

		sc->sc_txdesc_ring[i].tx_ctl = GEMTX_USED_MASK;
		bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
		    sizeof(struct tTXdesc) * i, sizeof(struct tTXdesc),
		    BUS_DMASYNC_PREWRITE);

	}

	if (remain == 0)
		ifp->if_timer = 0;
	if (handled)
		if_schedule_deferred_start(ifp);

	return handled;
}

static int
cge_alloc_dma(struct cge_softc *sc, size_t size, void **addrp,
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

static uint32_t
arswitch_reg_read32(device_t dev, int phy, int reg)
{
	uint16_t lo, hi;
	cge_mii_readreg(dev, phy, reg, &lo);
	cge_mii_readreg(dev, phy, reg + 1, &hi);

	return (hi << 16) | lo;
}
static int
arswitch_reg_write32(device_t dev, int phy, int reg, uint32_t value)
{
//	struct arswitch_softc *sc;
	int r;
	uint16_t lo, hi;

//	sc = device_get_softc(dev);
	lo = value & 0xffff;
	hi = (uint16_t) (value >> 16);

/*
	if (sc->mii_lo_first) {
		r = cge_mii_writereg(dev, phy, reg, lo);
		r |= cge_mii_writereg(dev, phy, reg + 1, hi);
	} else {
*/
		r = cge_mii_writereg(dev, phy, reg + 1, hi);
		r |= cge_mii_writereg(dev, phy, reg, lo);
//	}

	return r;
}

int
arswitch_readreg(device_t dev, int addr)
{
	uint16_t phy, reg;

	arswitch_split_setpage(dev, addr, &phy, &reg);
	return arswitch_reg_read32(dev, 0x10 | phy, reg);
}

int
arswitch_writereg(device_t dev, int addr, int value)
{
//	struct arswitch_softc *sc;
	uint16_t phy, reg;

//	sc = device_get_softc(dev);

	arswitch_split_setpage(dev, addr, &phy, &reg);
	return (arswitch_reg_write32(dev, 0x10 | phy, reg, value));
}
