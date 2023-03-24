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
#include <net/if_dl.h>
#include <net/bpf.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>

#include <arm/ralink/rt1310_reg.h>
#include <arm/ralink/rt1310_var.h>
#include <arm/ralink/rt1310_intr.h>
#include <arm/ralink/if_fvreg.h>

struct fv_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;
	bus_size_t		sc_bss;
	bus_dma_tag_t		sc_bdt;
	struct ethercom		sc_ec;
	struct mii_data		sc_mii;
	uint8_t			sc_enaddr[ETHER_ADDR_LEN];
	bool			sc_attached;
	kmutex_t		mtx;
	struct fv_ring_data	*sc_rdp;
	struct fv_desc		*sc_rxdesc_ring;
	bus_dmamap_t		sc_rxdesc_dmamap;
	struct fv_desc		*sc_txdesc_ring;
	bus_dmamap_t		sc_txdesc_dmamap;
	volatile u_int		sc_txnext;
	volatile u_int		sc_txhead;
	volatile u_int		sc_rxhead;
	u_int32_t		sc_inten;	/* copy of CSR_INTEN */
	u_int32_t		sc_rxint_mask;	/* mask of Rx interrupts */
	u_int32_t		sc_txint_mask;	/* mask of Tx interrupts */
	bool			sc_txbusy;
        void			*sc_sfbuf;
        bus_dmamap_t		sc_sfbuf_dm;
//	callout_t		sc_tick_ch;
	void			*sc_ih;
};

#define sc_sfbuf_pa sc_sfbuf_dm->dm_segs[0].ds_addr

static int fv_match(device_t, cfdata_t, void *);
static void fv_attach(device_t, device_t, void *);
static int fv_detach(device_t, int);

static int fv_intr(void *);

static int fv_alloc_dma(struct fv_softc *, size_t, void **,bus_dmamap_t *);

static void fv_reset(struct fv_softc *sc);
static int fv_init(struct ifnet *);
static void fv_start(struct ifnet *);
static int fv_ioctl(struct ifnet *, u_long, void *);
static void fv_watchdog(struct ifnet *);
static void fv_stop(struct ifnet *, int);

static int fv_miibus_readreg(device_t, int, int, uint16_t *);
static int fv_miibus_writereg(device_t, int, int, uint16_t);
static void fv_mii_statchg(struct ifnet *);

static int fv_new_rxbuf(struct fv_softc * const, const u_int);
//static void fv_tick(void *);

static int fv_rxintr(void *);
static int fv_txintr(void *);

static void fv_setfilt(struct fv_softc *sc);
static int fv_encap(struct fv_softc *sc, struct mbuf **m_head);
static void fv_start_locked(struct ifnet *ifp);

CFATTACH_DECL_NEW(fv, sizeof(struct fv_softc),
    fv_match, fv_attach, fv_detach, NULL);

#define TXDESC_NEXT(x) fv_txdesc_adjust((x), 1)
#define TXDESC_PREV(x) fv_txdesc_adjust((x), -1)

#define RXDESC_NEXT(x) fv_rxdesc_adjust((x), 1)
#define RXDESC_PREV(x) fv_rxdesc_adjust((x), -1)

#include <sys/kernhist.h>
KERNHIST_DEFINE(fvhist);

#define CPSWHIST_CALLARGS(A,B,C,D)	do {					\
	    KERNHIST_CALLARGS(fvhist, "%jx %jx %jx %jx",			\
		(uintptr_t)(A), (uintptr_t)(B), (uintptr_t)(C), (uintptr_t)(D));\
	} while (0)

static inline u_int
fv_txdesc_adjust(u_int x, int y)
{
//	return (((x) + y) & (CGE_TX_RING_CNT - 1));
	int res = x + y + FV_TX_RING_CNT;
	return res % FV_TX_RING_CNT;
}

static inline u_int
fv_rxdesc_adjust(u_int x, int y)
{
//	return (((x) + y) & (CGE_RX_RING_CNT - 1));
	int res = x + y + FV_RX_RING_CNT;
	return res % FV_RX_RING_CNT;
}

static inline uint32_t
fv_read_4(struct fv_softc * const sc, bus_size_t const offset)
{
	return bus_space_read_4(sc->sc_bst, sc->sc_bsh, offset);
}

static inline void
fv_write_4(struct fv_softc * const sc, bus_size_t const offset,
    uint32_t const value)
{
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, offset, value);
}

static int
fv_match(device_t parent, cfdata_t cf, void *aux)
{

	return 1;
}

static int
fv_detach(device_t self, int flags)
{
	struct fv_softc * const sc = device_private(self);
	struct ifnet *ifp = &sc->sc_ec.ec_if;
	u_int i;

	/* Succeed now if there's no work to do. */
	if (!sc->sc_attached)
		return 0;

	sc->sc_attached = false;

	/* Stop the interface. Callouts are stopped in it. */
	fv_stop(ifp, 1);

	/* Destroy our callout. */
//	callout_destroy(&sc->sc_tick_ch);

	/* Let go of the interrupts */
	intr_disestablish(sc->sc_ih);

	ether_ifdetach(ifp);
	if_detach(ifp);

	/* Delete all media. */
	ifmedia_fini(&sc->sc_mii.mii_media);

	/* Destroy all the descriptors */
	for (i = 0; i < FV_TX_RING_CNT; i++)
		bus_dmamap_destroy(sc->sc_bdt, sc->sc_rdp->tx_dm[i]);
	for (i = 0; i < FV_RX_RING_CNT; i++)
		bus_dmamap_destroy(sc->sc_bdt, sc->sc_rdp->rx_dm[i]);
	kmem_free(sc->sc_rdp, sizeof(*sc->sc_rdp));
	kmem_free(sc->sc_sfbuf, FV_SFRAME_LEN);

	/* Unmap */
	bus_space_unmap(sc->sc_bst, sc->sc_bsh, sc->sc_bss);

	return 0;
}

static void
fv_attach(device_t parent, device_t self, void *aux)
{
	struct ahb_attach_args * const aa = aux;
	struct fv_softc * const sc = device_private(self);
	int error;
	int i;
	struct ethercom * const ec = &sc->sc_ec;
	struct ifnet * const ifp = &ec->ec_if;
	struct mii_data * const mii = &sc->sc_mii;

	sc->sc_dev = self;
	sc->sc_bst = aa->ahba_memt;
	sc->sc_bdt = aa->ahba_dmat;
	sc->sc_bss = 0x20000;

	aprint_normal(": MAC Interface\n");
	aprint_naive("\n");

	mutex_init(&sc->mtx, MUTEX_DEFAULT, IPL_NET);
//	callout_init(&sc->sc_tick_ch, 0);
//	callout_setfunc(&sc->sc_tick_ch, fv_tick, sc);

	error = bus_space_map(aa->ahba_memt, aa->ahba_addr, sc->sc_bss,
	    0, &sc->sc_bsh);
	if (error) {
		aprint_error_dev(sc->sc_dev,
			"can't map registers: %d\n", error);
		return;
	}

	sc->sc_ih = intr_establish(aa->ahba_intr, IPL_NET, IST_LEVEL,
	    fv_intr, sc);

	sc->sc_enaddr[0] = 0xd4;
	sc->sc_enaddr[1] = 0x94;
	sc->sc_enaddr[2] = 0xa1;
	sc->sc_enaddr[3] = 0x97;
	sc->sc_enaddr[4] = 0x03;
	sc->sc_enaddr[5] = 0x94;

	sc->sc_rdp = kmem_alloc(sizeof(*sc->sc_rdp), KM_SLEEP);

	for (i = 0; i < FV_TX_RING_CNT; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES,
		    FV_TXFRAGS, MCLBYTES, 0, 0,
		    &sc->sc_rdp->tx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create tx DMA map: %d\n", error);
		}
		sc->sc_rdp->tx_mb[i] = NULL;
	}

	for (i = 0; i < FV_RX_RING_CNT; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES, 1,
		    MCLBYTES, 0, 0, &sc->sc_rdp->rx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create rx DMA map: %d\n", error);
		}
		sc->sc_rdp->rx_mb[i] = NULL;
	}

	sc->sc_sfbuf = kmem_alloc(FV_SFRAME_LEN, KM_SLEEP);
	bus_dmamap_create(sc->sc_bdt, FV_SFRAME_LEN, 1, FV_SFRAME_LEN, 0,
	    BUS_DMA_WAITOK, &sc->sc_sfbuf_dm);
	bus_dmamap_load(sc->sc_bdt, sc->sc_sfbuf_dm, sc->sc_sfbuf,
	    FV_SFRAME_LEN, NULL, BUS_DMA_WAITOK | BUS_DMA_WRITE);
	bus_dmamap_sync(sc->sc_bdt, sc->sc_sfbuf_dm, 0, FV_SFRAME_LEN,
	    BUS_DMASYNC_PREWRITE);

	if (fv_alloc_dma(sc, sizeof(struct fv_desc) * FV_RX_RING_CNT,
	    (void **)&(sc->sc_rxdesc_ring), &(sc->sc_rxdesc_dmamap)) != 0)
		return;

	if (fv_alloc_dma(sc, sizeof(struct fv_desc) * FV_TX_RING_CNT,
	    (void **)&(sc->sc_txdesc_ring), &(sc->sc_txdesc_dmamap)) != 0)
		return;

	memset(sc->sc_rxdesc_ring, 0, sizeof(struct fv_desc) * FV_RX_RING_CNT);

//	paddr_t paddr;
	for (i = 0; i < FV_TX_RING_CNT; i++) {
/*
		if (i == FV_TX_RING_CNT - 1)
			paddr = FV_TX_RING_ADDR(sc, 0);
		else
			paddr = FV_TX_RING_ADDR(sc, i + 1);
*/
		sc->sc_txdesc_ring[i].fv_stat = 0;
		sc->sc_txdesc_ring[i].fv_devcs = 0;
		sc->sc_txdesc_ring[i].fv_addr = 0;
//		sc->sc_txdesc_ring[i].fv_link = paddr;
//		sc->sc_txdesc_ring[i].fv_link = 0;
	}
	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    0, sizeof(struct fv_desc) * FV_TX_RING_CNT,
	    BUS_DMASYNC_PREWRITE);

	aprint_normal_dev(sc->sc_dev, "Ethernet address %s\n",
	    ether_sprintf(sc->sc_enaddr));

	fv_write_4(sc, CSR_BUSMODE, BUSMODE_SWR);

	strlcpy(ifp->if_xname, device_xname(sc->sc_dev), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_capabilities = 0;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_start = fv_start;
	ifp->if_ioctl = fv_ioctl;
	ifp->if_init = fv_init;
	ifp->if_stop = fv_stop;
	ifp->if_watchdog = fv_watchdog;
	IFQ_SET_READY(&ifp->if_snd);

	fv_stop(ifp, 0);

	mii->mii_ifp = ifp;
	mii->mii_readreg = fv_miibus_readreg;
	mii->mii_writereg = fv_miibus_writereg;
	mii->mii_statchg = fv_mii_statchg;

	sc->sc_ec.ec_mii = mii;
	ifmedia_init(&mii->mii_media, 0, ether_mediachange, ether_mediastatus);

/*
	mii_attach(self, mii, 0xffffffff, MII_PHY_ANY, 0, 0);
	ifmedia_add(&mii->mii_media, IFM_ETHER | IFM_MANUAL, 0, NULL);
	ifmedia_set(&mii->mii_media, IFM_ETHER | IFM_MANUAL);
*/

	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr);

	/* The attach is successful. */
	sc->sc_attached = true;

	return;
}

static void
fv_start(struct ifnet *ifp)
{

	fv_start_locked(ifp);
}

static int
fv_ioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	const int s = splnet();
	int error = 0;

	switch (cmd) {
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
fv_watchdog(struct ifnet *ifp)
{

#if 0
	struct fv_softc *sc = ifp->if_softc;

	device_printf(sc->sc_dev, "device timeout\n");

	if_statinc(ifp, if_oerrors);
	fv_init(ifp);
	fv_start(ifp);
#endif
}

static void
fv_mii_statchg(struct ifnet *ifp)
{
	return;
}

static int
fv_new_rxbuf(struct fv_softc * const sc, const u_int i)
{
	struct fv_ring_data * const rdp = sc->sc_rdp;
//	const u_int h = RXDESC_PREV(i);
//	struct fv_cpdma_bd bd;
//	uint32_t * const dw = bd.word;
	struct mbuf *m;
	int error = ENOBUFS;

	MGETHDR(m, M_DONTWAIT, MT_DATA);
	if (m == NULL) {
		device_printf(sc->sc_dev, "MGETHDR error\n");
		goto reuse;
	}

	MCLGET(m, M_DONTWAIT);
	if ((m->m_flags & M_EXT) == 0) {
		device_printf(sc->sc_dev, "MCLGET error\n");
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
	sc->sc_rxdesc_ring[i].fv_stat = ADSTAT_OWN;
	sc->sc_rxdesc_ring[i].fv_addr = rdp->rx_dm[i]->dm_segs[0].ds_addr;
	sc->sc_rxdesc_ring[i].fv_devcs = PKT_BUF_SZ;
//	sc->sc_rxdesc_ring[i].fv_link = 0;
	if (i == FV_RX_RING_CNT - 1)
		sc->sc_rxdesc_ring[i].fv_devcs |= ADCTL_ER;

	bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
	    sizeof(struct fv_desc) * i, sizeof(struct fv_desc),
            BUS_DMASYNC_PREWRITE);
//            BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);

	return error;
}

static void
fv_reset(struct fv_softc *sc)
{
	int		i;

	fv_write_4(sc, CSR_BUSMODE, BUSMODE_SWR);

	/*
	 * The chip doesn't take itself out of reset automatically.
	 * We need to do so after 2us.
	 */
	DELAY(1000);
	fv_write_4(sc, CSR_BUSMODE, 0);

	for (i = 0; i < 1000; i++) {
		/*
		 * Wait a bit for the reset to complete before peeking
		 * at the chip again.
		 */
		DELAY(1000);
		if ((fv_read_4(sc, CSR_BUSMODE) & BUSMODE_SWR) == 0)
			break;
	}

	if (fv_read_4(sc, CSR_BUSMODE) & BUSMODE_SWR)
		device_printf(sc->sc_dev, "reset time out\n");

	DELAY(1000);
}

static int
fv_init(struct ifnet *ifp)
{
	struct fv_softc * const sc = ifp->if_softc;
	int i;
//	int reg;
//	int orgreg;
//	int mac;
	paddr_t paddr;

	fv_stop(ifp, 0);

	fv_reset(sc);

	sc->sc_txnext = 0;
	sc->sc_txhead = 0;

	/* Init circular RX list. */
	for (i = 0; i < FV_RX_RING_CNT; i++) {
		fv_new_rxbuf(sc, i);
	}
	sc->sc_rxhead = 0;

	fv_write_4(sc, CSR_BUSMODE,
	    /* XXX: not sure if this is a good thing or not... */
	    BUSMODE_BAR | BUSMODE_PBL_32LW);

	/*
	 * Initialize the interrupt mask and enable interrupts.
	 */
 
	/* normal interrupts */
	sc->sc_inten =  STATUS_TI | STATUS_TU | STATUS_RI | STATUS_NIS;

	/* abnormal interrupts */
	sc->sc_inten |= STATUS_TPS | STATUS_TJT | STATUS_UNF |
	    STATUS_RU | STATUS_RPS | STATUS_SE | STATUS_AIS;

	sc->sc_rxint_mask = STATUS_RI|STATUS_RU;
	sc->sc_txint_mask = STATUS_TI|STATUS_UNF|STATUS_TJT;

	sc->sc_rxint_mask &= sc->sc_inten;
	sc->sc_txint_mask &= sc->sc_inten;

	fv_write_4(sc, CSR_INTEN, sc->sc_inten);
	fv_write_4(sc, CSR_STATUS, 0xffffffff);

	/*
	 * Give the transmit and receive rings to the chip.
	 */
	paddr = sc->sc_txdesc_dmamap->dm_segs[0].ds_addr;
	fv_write_4(sc, CSR_TXLIST, paddr);
	paddr = sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr;
	fv_write_4(sc, CSR_RXLIST, paddr);

	/*
	 * Set the station address.
	 */
	fv_setfilt(sc);

	/*
	 * Write out the opmode.
	 */
	fv_write_4(sc, CSR_OPMODE, OPMODE_SR | OPMODE_ST |
	    OPMODE_TR_128 | OPMODE_FDX | OPMODE_SPEED);

	/*
	 * Start the receive process.
	 */
	fv_write_4(sc, CSR_RXPOLL, RXPOLL_RPD);

	ifp->if_flags |= IFF_RUNNING;

	return 0;
}

static void
fv_stop(struct ifnet *ifp, int disable)
{
	struct fv_softc * const sc = ifp->if_softc;
	struct fv_ring_data * const rdp = sc->sc_rdp;
	u_int i;

	aprint_debug_dev(sc->sc_dev, "%s: ifp %p disable %d\n", __func__,
	    ifp, disable);

	if ((ifp->if_flags & IFF_RUNNING) == 0)
		return;

//	callout_stop(&sc->sc_tick_ch);
	mii_down(&sc->sc_mii);

	fv_write_4(sc, CSR_INTEN, 0);

	fv_write_4(sc, CSR_OPMODE, 0);
	fv_write_4(sc, CSR_RXLIST, 0);
	fv_write_4(sc, CSR_TXLIST, 0);

#if 0
	fv_write_4(sc, CPSW_CPDMA_TX_INTMASK_CLEAR, 1);
	fv_write_4(sc, CPSW_CPDMA_RX_INTMASK_CLEAR, 1);
	fv_write_4(sc, CPSW_WR_C_TX_EN(0), 0x0);
	fv_write_4(sc, CPSW_WR_C_RX_EN(0), 0x0);
	fv_write_4(sc, CPSW_WR_C_MISC_EN(0), 0x0);

	fv_write_4(sc, CPSW_CPDMA_TX_TEARDOWN, 0);
	fv_write_4(sc, CPSW_CPDMA_RX_TEARDOWN, 0);
	i = 0;
	while ((sc->sc_txrun || sc->sc_rxrun) && i < 10000) {
		delay(10);
		if ((sc->sc_txrun == true) && fv_txintr(sc) == 0)
			sc->sc_txrun = false;
		if ((sc->sc_rxrun == true) && fv_rxintr(sc) == 0)
			sc->sc_rxrun = false;
		i++;
	}
	//printf("%s toredown complete in %u\n", __func__, i);

	/* Reset wrapper */
	fv_write_4(sc, CPSW_WR_SOFT_RESET, 1);
	while (fv_read_4(sc, CPSW_WR_SOFT_RESET) & 1)
		;

	/* Reset SS */
	fv_write_4(sc, CPSW_SS_SOFT_RESET, 1);
	while (fv_read_4(sc, CPSW_SS_SOFT_RESET) & 1)
		;

	for (i = 0; i < CPSW_ETH_PORTS; i++) {
		fv_write_4(sc, CPSW_SL_SOFT_RESET(i), 1);
		while (fv_read_4(sc, CPSW_SL_SOFT_RESET(i)) & 1)
			;
	}

	/* Reset CPDMA */
	fv_write_4(sc, CPSW_CPDMA_SOFT_RESET, 1);
	while (fv_read_4(sc, CPSW_CPDMA_SOFT_RESET) & 1)
		;

	/* Release any queued transmit buffers. */
	for (i = 0; i < CPSW_NTXDESCS; i++) {
		bus_dmamap_unload(sc->sc_bdt, rdp->tx_dm[i]);
		m_freem(rdp->tx_mb[i]);
		rdp->tx_mb[i] = NULL;
	}
#endif

	ifp->if_flags &= ~IFF_RUNNING;
	ifp->if_timer = 0;
	sc->sc_txbusy = false;

	if (!disable)
		return;

	for (i = 0; i < FV_RX_RING_CNT; i++) {
		bus_dmamap_unload(sc->sc_bdt, rdp->rx_dm[i]);
		m_freem(rdp->rx_mb[i]);
		rdp->rx_mb[i] = NULL;
	}
}

static int
fv_intr(void *arg)
{
	struct fv_softc * const sc = arg;
	int status, rv;

	rv = 0;
	status = fv_read_4(sc, CSR_STATUS);
	/* mask out interrupts */
	while((status & sc->sc_inten) != 0) {
		fv_write_4(sc, CSR_STATUS, status);
		if (status & STATUS_UNF) {
			device_printf(sc->sc_dev, "Transmit Underflow\n");
		}
		if (status & sc->sc_rxint_mask) {
			fv_rxintr(arg);
			rv = 1;
		}
		if (status & sc->sc_txint_mask) {
			fv_txintr(arg);
			rv = 1;
		}
		if (status & STATUS_AIS) {
			device_printf(sc->sc_dev, "Abnormal Interrupt %x\n",
			    status);
                }
		fv_write_4(sc, CSR_FULLDUP, FULLDUP_CS |
		    (1 << FULLDUP_TT_SHIFT) | (3 << FULLDUP_NTP_SHIFT) |
		    (2 << FULLDUP_RT_SHIFT) | (2 << FULLDUP_NRP_SHIFT));

		status = fv_read_4(sc, CSR_STATUS);
	}

	return rv;
}

#if 0
static void
fv_tick(void *arg)
{
	struct fv_softc * const sc = arg;
	struct mii_data * const mii = &sc->sc_mii;
	const int s = splnet();

	mii_tick(mii);

	splx(s);

	callout_schedule(&sc->sc_tick_ch, hz);
}
#endif

static int
fv_rxintr(void *arg)
{
	struct fv_softc * const sc = arg;
	struct fv_ring_data * const rdp = sc->sc_rdp;
	u_int i;
	int length;
	struct mbuf *m;
	struct ifnet *ifp = &sc->sc_ec.ec_if;

#if 0
	for (i = 0; i < FV_RX_RING_CNT; i++) {
		bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
		    sizeof(struct fv_desc) * i, sizeof(struct fv_desc),
//		    BUS_DMASYNC_PREREAD);
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		if(!(sc->sc_rxdesc_ring[i].fv_stat & ADSTAT_OWN)) {
			sc->sc_rxdesc_ring[i].fv_stat = ADSTAT_OWN;
			sc->sc_rxdesc_ring[i].fv_devcs = PKT_BUF_SZ;
			if (i == FV_RX_RING_CNT - 1)
				sc->sc_rxdesc_ring[i].fv_devcs |= ADCTL_ER;
			printf("%d,", i);
			bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
			    sizeof(struct fv_desc) * i, sizeof(struct fv_desc),
			    BUS_DMASYNC_PREWRITE);
		}
	}
	return 0;
#endif
	for (;;) {
		i = sc->sc_rxhead;
		bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
		    sizeof(struct fv_desc) * i, sizeof(struct fv_desc),
		    BUS_DMASYNC_PREREAD);
//		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		if(!(sc->sc_rxdesc_ring[i].fv_stat & ADSTAT_OWN)) {
//			length = sc->sc_rxdesc_ring[i].rx_status &
//			    RX_STA_LEN_MASK;
			length = ADSTAT_Rx_LENGTH(
			    sc->sc_rxdesc_ring[i].fv_stat);
//			length -= ETHER_CRC_LEN;
			m = rdp->rx_mb[i];
			bus_dmamap_sync(sc->sc_bdt, rdp->rx_dm[i],
			    0, rdp->rx_dm[i]->dm_mapsize,
		    	BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		char *p = mtod(m, char *);
		if (p == 0) {
			/* Why happen ??? */
			device_printf(sc->sc_dev, "rx mbuf error\n");
			m_freem(m);
		} else {
//			m_adj(m, 2);
			m_set_rcvif(m, ifp);
			m->m_pkthdr.len = m->m_len = length;
			m->m_flags |= M_HASFCS;
			if_percpuq_enqueue(ifp->if_percpuq, m);
//			m_freem(m);
		}
			fv_new_rxbuf(sc, i);
/*
			bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
			    sizeof(struct fv_desc) * i, sizeof(struct fv_desc),
			    BUS_DMASYNC_POSTWRITE);
*/

			fv_write_4(sc, CSR_RXPOLL, 1);

			sc->sc_rxhead = RXDESC_NEXT(sc->sc_rxhead);
		} else {
			break;
		}
	}
	return 1;
}

static int
fv_txintr(void *arg)
{
	struct fv_softc * const sc = arg;
	struct fv_ring_data * const rdp = sc->sc_rdp;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	bool handled = false;
	int first;

#if 0
	int i;
	int count = 0;
	for (i = 0; i < FV_TX_RING_CNT; i++) {
		bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
		    sizeof(struct fv_desc) * i, sizeof(struct fv_desc),
//		    BUS_DMASYNC_PREREAD);
		    BUS_DMASYNC_POSTREAD | BUS_DMASYNC_POSTWRITE);
		if(!(sc->sc_rxdesc_ring[i].fv_stat & ADSTAT_OWN)) {
			++count;
		}
	}
	sc->sc_txhead = sc->sc_txnext;
	printf("TXI %d,", count);
	return 0;
#endif
	first = sc->sc_txhead;
	for (;;) {
		bus_dmamap_sync(sc->sc_bdt, rdp->tx_dm[sc->sc_txhead],
		    0, rdp->tx_dm[sc->sc_txhead]->dm_mapsize,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_bdt, rdp->tx_dm[sc->sc_txhead]);

		bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
		    sizeof(struct fv_desc) * sc->sc_txhead,
		    sizeof(struct fv_desc), BUS_DMASYNC_PREREAD);
		if (sc->sc_txdesc_ring[sc->sc_txhead].fv_devcs &
		    ADCTL_Tx_SETUP) {
		} else {
		if (first == sc->sc_txhead)
			m_freem(rdp->tx_mb[sc->sc_txhead]);
		rdp->tx_mb[sc->sc_txhead] = NULL;

		if_statinc(ifp, if_opackets);

		handled = true;

		sc->sc_txdesc_ring[sc->sc_txhead].fv_devcs = 0;
		}

		sc->sc_txhead = TXDESC_NEXT(sc->sc_txhead);
		if (sc->sc_txhead == sc->sc_txnext)
			break;
	}
/*
	for (i = 0; i < FV_TX_RING_CNT; i++) {
		sc->sc_txdesc_ring[i].fv_devcs = 0;
	}
*/
	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    0, sizeof(struct fv_desc) * FV_TX_RING_CNT,
	    BUS_DMASYNC_PREWRITE);
	if (handled && sc->sc_txnext == sc->sc_txhead)
		ifp->if_timer = 0;
	if (handled)
		if_schedule_deferred_start(ifp);

	sc->sc_txbusy = false;

	return handled;
}

static int
fv_alloc_dma(struct fv_softc *sc, size_t size, void **addrp,
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

static int
fv_miibus_readbits(struct fv_softc *sc, int count)
{
	int result;

	result = 0;
	while(count--) {
		result <<= 1;
		fv_write_4(sc, CSR_MIIMNG, MII_RD);
		DELAY(10);
		fv_write_4(sc, CSR_MIIMNG, MII_RD | MII_CLK);
		DELAY(10);
		if (fv_read_4(sc, CSR_MIIMNG) & MII_DIN)
			result |= 1;
	}

	return (result);
}

static int
fv_miibus_writebits(struct fv_softc *sc, int data, int count)
{
	int bit;

	while(count--) {
		bit = ((data) >> count) & 0x1 ? MII_DOUT : 0;
		fv_write_4(sc, CSR_MIIMNG, bit | MII_WR);
		DELAY(10);
		fv_write_4(sc, CSR_MIIMNG, bit | MII_WR | MII_CLK);
		DELAY(10);
	}

	return (0);
}

static void
fv_miibus_turnaround(struct fv_softc *sc, int cmd)
{
	if (cmd == MII_WRCMD) {
		fv_miibus_writebits(sc, 0x02, 2);
	} else {
		fv_miibus_readbits(sc, 1);
	}
}

static int
fv_miibus_readreg(device_t dev, int phy, int reg, uint16_t *val)
{
	struct fv_softc * sc = device_private(dev);

//	mtx_lock(&miibus_mtx);
	fv_miibus_writebits(sc, MII_PREAMBLE, 32);
	fv_miibus_writebits(sc, MII_RDCMD, 4);
	fv_miibus_writebits(sc, phy, 5);
	fv_miibus_writebits(sc, reg, 5);
	fv_miibus_turnaround(sc, MII_RDCMD);
	*val = fv_miibus_readbits(sc, 16);
	fv_miibus_turnaround(sc, MII_RDCMD);
//	mtx_unlock(&miibus_mtx);

	return 0;
}

static int
fv_miibus_writereg(device_t dev, int phy, int reg, uint16_t val)
{
	struct fv_softc * sc = device_private(dev);

//	mtx_lock(&miibus_mtx);
	fv_miibus_writebits(sc, MII_PREAMBLE, 32);
	fv_miibus_writebits(sc, MII_WRCMD, 4);
	fv_miibus_writebits(sc, phy, 5);
	fv_miibus_writebits(sc, reg, 5);
	fv_miibus_turnaround(sc, MII_WRCMD);
	fv_miibus_writebits(sc, val, 16);
//	mtx_unlock(&miibus_mtx);

	return 0;
}

/* setup frame code refer dc code */

static void
fv_setfilt(struct fv_softc *sc)
{
	uint16_t eaddr[(ETHER_ADDR_LEN+1)/2];
	uint16_t *sp;
	struct ifnet *ifp = &sc->sc_ec.ec_if;
	struct ethercom *ec = &sc->sc_ec;
	struct ether_multi *enm;
	struct ether_multistep step;
	int i;
	uint8_t *ma;


	sp = (uint16_t *)sc->sc_sfbuf;
	memset(sp, 0xff, FV_SFRAME_LEN);

	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    sizeof(struct fv_desc) * sc->sc_txnext,
	    sizeof(struct fv_desc), BUS_DMASYNC_PREREAD);

	sc->sc_txdesc_ring[sc->sc_txnext].fv_stat = ADSTAT_OWN;
	sc->sc_txdesc_ring[sc->sc_txnext].fv_addr = sc->sc_sfbuf_pa;
	/* This packet is not use interrupt */
	sc->sc_txdesc_ring[sc->sc_txnext].fv_devcs = ADCTL_Tx_SETUP |
			    FV_DMASIZE(FV_SFRAME_LEN);
	/* end of descriptor */
	if (sc->sc_txnext == FV_TX_RING_CNT - 1)
		sc->sc_txdesc_ring[sc->sc_txnext].fv_devcs |= ADCTL_ER;

	i = 0;
	ETHER_LOCK(ec);
	ETHER_FIRST_MULTI(step, ec, enm);
	while (enm != NULL) {
		if (memcmp(enm->enm_addrlo, enm->enm_addrhi, 6) == 0) {
			ma = enm->enm_addrhi;
			sp[i] = sp[i+1] = (ma[1] << 8 | ma[0]);
			i += 2;
			sp[i] = sp[i+1] = (ma[3] << 8 | ma[2]);
			i += 2;
			sp[i] = sp[i+1] = (ma[5] << 8 | ma[4]);
			i += 2;
		} else {
			/* XXX */
		}
		ETHER_NEXT_MULTI(step, enm);
	}
	ETHER_UNLOCK(ec);
	memcpy(eaddr, CLLADDR(ifp->if_sadl), ETHER_ADDR_LEN);
	sp[90] = sp[91] = eaddr[0];
	sp[92] = sp[93] = eaddr[1];
	sp[94] = sp[95] = eaddr[2];

	bus_dmamap_sync(sc->sc_bdt, sc->sc_sfbuf_dm, 0, FV_SFRAME_LEN,
	    BUS_DMASYNC_PREWRITE);
	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    sizeof(struct fv_desc) * sc->sc_txnext,  sizeof(struct fv_desc),
	    BUS_DMASYNC_PREWRITE);
	sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
	fv_write_4(sc, CSR_TXPOLL, 0xFFFFFFFF);

	DELAY(10000);
}

/*
 * Encapsulate an mbuf chain in a descriptor by coupling the mbuf data
 * pointers to the fragment pointers.
 * Use Implicit Chain implementation.
 */
static int
fv_encap(struct fv_softc *sc, struct mbuf **m_head)
{
//	struct fv_txdesc	*txd;
	struct fv_desc		*desc;
	struct mbuf		*m;
	bus_dmamap_t		dm;
	struct fv_ring_data * const rdp = sc->sc_rdp;
//	bus_dma_segment_t	txsegs[FV_MAXFRAGS];
//	int			error, i, nsegs, prod, si;
	int			padlen, error, si, i;
	int			txstat;

//	FV_LOCK_ASSERT(sc);

	/*
	 * Some VIA Rhine wants packet buffers to be longword
	 * aligned, but very often our mbufs aren't. Rather than
	 * waste time trying to decide when to copy and when not
	 * to copy, just do it all the time.
	 */
/*
	m = m_defrag(*m_head, M_DONTWAIT);
	if (m == NULL) {
		device_printf(sc->sc_dev, "fv_encap m_defrag error\n");
		m_freem(*m_head);
		*m_head = NULL;
		return (ENOBUFS);
	}
	*m_head = m;
*/

	/*
	 * The Rhine chip doesn't auto-pad, so we have to make
	 * sure to pad short frames out to the minimum frame length
	 * ourselves.
	 */
	if ((*m_head)->m_pkthdr.len < FV_MIN_FRAMELEN) {
		m = *m_head;
		padlen = FV_MIN_FRAMELEN - m->m_pkthdr.len;
		if (M_UNWRITABLE(m, m->m_pkthdr.len)) {
			/* Get a writable copy. */
			m = m_dup(*m_head, 0, M_COPYALL, M_NOWAIT);
			m_freem(*m_head);
			if (m == NULL) {
				device_printf(sc->sc_dev, "fv_encap m_dup error\n");
				*m_head = NULL;
				return (ENOBUFS);
			}
			*m_head = m;
		}
		if (m->m_next != NULL || M_TRAILINGSPACE(m) < padlen) {
			m = m_defrag(m, M_NOWAIT);
			if (m == NULL) {
				device_printf(sc->sc_dev, "fv_encap m_defrag error\n");
				m_freem(*m_head);
				*m_head = NULL;
				return (ENOBUFS);
			}
		}
		/*
		 * Manually pad short frames, and zero the pad space
		 * to avoid leaking data.
		 */
		bzero(mtod(m, char *) + m->m_pkthdr.len, padlen);
		m->m_pkthdr.len += padlen;
		m->m_len = m->m_pkthdr.len;
		*m_head = m;
	}

//	prod = sc->sc_txnext;
//	txd = &rdp->tx_dm[prod];
//	error = bus_dmamap_load_mbuf_sg(sc->fv_cdata.fv_tx_tag, txd->tx_dmamap,
//	    *m_head, txsegs, &nsegs, BUS_DMA_NOWAIT);
	dm = rdp->tx_dm[sc->sc_txnext];
	error = bus_dmamap_load_mbuf(sc->sc_bdt, dm, *m_head,
	    BUS_DMA_WRITE | BUS_DMA_NOWAIT);
	if (error == EFBIG) {
		device_printf(sc->sc_dev, "fv_encap EFBIG error\n");
		return (ENOBUFS);
	}
#if 0
	if (error == EFBIG) {
		device_printf(sc->sc_dev, "fv_encap EFBIG error\n");
		m = m_defrag(*m_head, M_NOWAIT);
		if (m == NULL) {
			m_freem(*m_head);
			*m_head = NULL;
			return (ENOBUFS);
		}
		*m_head = m;
		error = bus_dmamap_load_mbuf_sg(sc->fv_cdata.fv_tx_tag,
		    txd->tx_dmamap, *m_head, txsegs, &nsegs, BUS_DMA_NOWAIT);
		if (error != 0) {
			m_freem(*m_head);
			*m_head = NULL;
			return (error);
		}

	} else if (error != 0)
		return (error);
	if (nsegs == 0) {
		m_freem(*m_head);
		*m_head = NULL;
		return (EIO);
	}

	/* Check number of available descriptors. */
	if (sc->fv_cdata.fv_tx_cnt + nsegs >= (FV_TX_RING_CNT - 1)) {
		bus_dmamap_unload(sc->fv_cdata.fv_tx_tag, txd->tx_dmamap);
		return (ENOBUFS);
	}

#endif
//	txd->tx_m = *m_head;
	rdp->tx_mb[sc->sc_txnext] = *m_head;
	bus_dmamap_sync(sc->sc_bdt, dm, 0, dm->dm_mapsize,
	    BUS_DMASYNC_PREWRITE);

	si = sc->sc_txnext;

	/*
	 * Make a list of descriptors for this packet.
	 */
	desc = NULL;
	for (i = 0; i < dm->dm_nsegs; i++) {
//		desc = &sc->fv_rdata.fv_tx_ring[sc->sc_txnext];
		desc = &sc->sc_txdesc_ring[sc->sc_txnext];
		desc->fv_stat = ADSTAT_OWN;
		desc->fv_devcs = dm->dm_segs[i].ds_len;
		/* end of descriptor */
		if (sc->sc_txnext == FV_TX_RING_CNT - 1)
			desc->fv_devcs |= ADCTL_ER;
		desc->fv_addr = dm->dm_segs[i].ds_addr;

//		++sc->fv_cdata.fv_tx_cnt;
//		FV_INC(prod, FV_TX_RING_CNT);
		sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
	}

	/*
	 * Set mark last fragment with Last/Intr flag
	 */
	if (desc) {
		desc->fv_devcs |= ADCTL_Tx_IC;
		desc->fv_devcs |= ADCTL_Tx_LS;
	}

	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    0, sizeof(struct fv_desc) * FV_TX_RING_CNT,
	    BUS_DMASYNC_PREREAD | BUS_DMASYNC_PREWRITE);

	txstat = (fv_read_4(sc, CSR_STATUS) >> 20) & 7;
	if (txstat == 0 || txstat == 6) {
		/* Transmit Process Stat is stop or suspended */
		desc = &sc->sc_txdesc_ring[si];
		desc->fv_devcs |= ADCTL_Tx_FS;
	}
	else {
		/* Get previous descriptor */
		si = (si + FV_TX_RING_CNT - 1) % FV_TX_RING_CNT;
		desc = &sc->sc_txdesc_ring[si];
		/* join remain data and flugs */
		desc->fv_devcs &= ~ADCTL_Tx_IC;
		desc->fv_devcs &= ~ADCTL_Tx_LS;
	}

	return (0);
}

static void
fv_start_locked(struct ifnet *ifp)
{
	struct fv_softc		*sc;
	struct mbuf		*m_head;
	int			enq;
	int			txstat;

	sc = ifp->if_softc;

//	FV_LOCK_ASSERT(sc);
	mutex_enter(&(sc)->mtx);

	if (__predict_false((ifp->if_flags & IFF_RUNNING) == 0)) {
		return;
	}
	if (__predict_false(sc->sc_txbusy)) {
		return;
	}

//	for (enq = 0; !IFQ_IS_EMPTY(&ifp->if_snd) &&
//	    sc->fv_cdata.fv_tx_cnt < FV_TX_RING_CNT - 2; ) {
	for (enq = 0; !IFQ_IS_EMPTY(&ifp->if_snd); ) {
		IFQ_DEQUEUE(&ifp->if_snd, m_head);
		if (m_head == NULL)
			break;
		/*
		 * Pack the data into the transmit ring. If we
		 * don't have room, set the OACTIVE flag and wait
		 * for the NIC to drain the ring.
		 */
		if (fv_encap(sc, &m_head)) {
			if (m_head == NULL)
				break;
			IF_PREPEND(&ifp->if_snd, m_head);
			ifp->if_flags |= IFF_OACTIVE;
			break;
		}

		enq++;
		/*
		 * If there's a BPF listener, bounce a copy of this frame
		 * to him.
		 */
//		ETHER_BPF_MTAP(ifp, m_head);
//		bpf_mtap(ifp, m_head, BPF_D_OUT);
	}

	if (enq > 0) {
		txstat = (fv_read_4(sc, CSR_STATUS) >> 20) & 7;
		if (txstat == 0 || txstat == 6)
			fv_write_4(sc, CSR_TXPOLL, TXPOLL_TPD);
	}
	mutex_exit(&(sc)->mtx);
}
