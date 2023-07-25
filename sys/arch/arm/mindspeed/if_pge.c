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

/* Mindspeed Comcerto 2000 PFE GEMAC Interface. */

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

#include <arm/mindspeed/m86xxx_reg.h>
#include <arm/mindspeed/m86xxx_var.h>
#include <arm/mindspeed/m86xxx_intr.h>
#include <arm/mindspeed/if_pgereg.h>
#include <arm/mindspeed/arswitchreg.h>

#include <arm/mindspeed/pfe/pfe_eth.h>
#include <arm/mindspeed/pfe/base/pfe.h>
#include <arm/mindspeed/pfe/pfe_driver.h>


#define	PGE_TXFRAGS	16
#define	MAX_FRAME_SIZE	2048

static int pge_match(device_t, cfdata_t, void *);
static void pge_attach(device_t, device_t, void *);
static int pge_detach(device_t, int);

static void pge_start(struct ifnet *);
static int pge_ioctl(struct ifnet *, u_long, void *);
static void pge_watchdog(struct ifnet *ifp);
static int pge_init(struct ifnet *);
static void pge_stop(struct ifnet *, int);

static int pge_mii_readreg(device_t, int, int, uint16_t *);
static int pge_mii_writereg(device_t, int, int, uint16_t);
static void pge_mii_statchg(struct ifnet *);

static int pge_new_rxbuf(struct pge_softc * const, const u_int);
static void pge_tick(void *);
static int pge_alloc_ddr(struct pge_softc * const);

static int pge_intr(void *);
static int pge_rxintr(void *);
static int pge_txintr(void *);

static int pge_alloc_dma(struct pge_softc *, size_t, void **,bus_dmamap_t *);

static uint32_t pge_emac_base(struct pge_softc * const);

#define TXDESC_NEXT(x) pge_txdesc_adjust((x), 1)
#define TXDESC_PREV(x) pge_txdesc_adjust((x), -1)

#define RXDESC_NEXT(x) pge_rxdesc_adjust((x), 1)
#define RXDESC_PREV(x) pge_rxdesc_adjust((x), -1)

CFATTACH_DECL_NEW(pge, sizeof(struct pge_softc),
    pge_match, pge_attach, pge_detach, NULL);

extern struct pge_softc *pge_sc;

int arswitch_readreg(device_t dev, int addr);
int arswitch_writereg(device_t dev, int addr, int value);

void arswitch_writedbg(device_t dev, int phy, uint16_t dbg_addr,
    uint16_t dbg_data);

static uint32_t pge_emac_base(struct pge_softc * const sc)
{
	int addr;

	if (device_unit(sc->sc_dev) == 0)
		addr = EMAC1_BASE_ADDR;
	else if (device_unit(sc->sc_dev) == 1)
		addr = EMAC2_BASE_ADDR;
	else
		addr = EMAC3_BASE_ADDR;
	
	return addr;
}

static inline u_int
pge_txdesc_adjust(u_int x, int y)
{
	int res = x + y + PGE_TX_RING_CNT;
	return res % PGE_TX_RING_CNT;
}

static inline u_int
pge_rxdesc_adjust(u_int x, int y)
{
	int res = x + y + PGE_RX_RING_CNT;
	return res % PGE_RX_RING_CNT;
}

static int 
pge_match(device_t parent, cfdata_t cf, void *aux)
{
	return 1;
}

static void
pge_attach(device_t parent, device_t self, void *aux)
{
	struct axi_attach_args * const aa = aux;
	struct pge_softc * const sc = device_private(self);
	int i, error;
	struct ethercom * const ec = &sc->sc_ec;
	struct ifnet * const ifp = &ec->ec_if;
	struct mii_data * const mii = &sc->sc_mii;
	paddr_t paddr;

	sc->sc_dev = self;
	sc->sc_bst = aa->aa_iot;
	sc->sc_bdt = aa->aa_dmat;
	sc->sc_bss = 0x1000000;		/* 16M */

	aprint_normal(": PFE GEMAC Interface\n");
	aprint_naive("\n");

	mutex_init(&sc->mtx, MUTEX_DEFAULT, IPL_NET);
	callout_init(&sc->sc_tick_ch, 0);
	callout_setfunc(&sc->sc_tick_ch, pge_tick, sc);

	error = bus_space_map(aa->aa_iot, aa->aa_addr, sc->sc_bss,
	    0, &sc->sc_bsh);
	if (error) {
		aprint_error_dev(sc->sc_dev,
			"can't map registers: %d\n", error);
		return;
	}

	sc->sc_ih = intr_establish(aa->aa_intr, IPL_NET, IST_LEVEL,
	    pge_intr, sc);

	sc->sc_enaddr[0] = 0xd4;
	sc->sc_enaddr[1] = 0x94;
	sc->sc_enaddr[2] = 0xa1;
	sc->sc_enaddr[3] = 0x97;
	sc->sc_enaddr[4] = 0x03;
	sc->sc_enaddr[5] = 0x94 + device_unit(self);

	sc->sc_rdp = kmem_alloc(sizeof(*sc->sc_rdp), KM_SLEEP);

	for (i = 0; i < PGE_TX_RING_CNT; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES,
		    PGE_TXFRAGS, MCLBYTES, 0, 0,
		    &sc->sc_rdp->tx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create tx DMA map: %d\n", error);
		}
		sc->sc_rdp->tx_mb[i] = NULL;
	}

	for (i = 0; i < PGE_RX_RING_CNT; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES, 1,
		    MCLBYTES, 0, 0, &sc->sc_rdp->rx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create rx DMA map: %d\n", error);
		}
		sc->sc_rdp->rx_mb[i] = NULL;
	}

	if (pge_alloc_dma(sc, sizeof(struct bufDesc) * PGE_RX_RING_CNT,
	    (void **)&(sc->sc_rxdesc_ring), &(sc->sc_rxdesc_dmamap)) != 0)
		return;

	memset(sc->sc_rxdesc_ring, 0, sizeof(struct bufDesc) * PGE_RX_RING_CNT);

	paddr = sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr;

	for (i = 0; i < PGE_RX_RING_CNT; i++) {
		sc->sc_rxdesc_ring[i].data = 0;
		if (i == PGE_TX_RING_CNT - 1)
			sc->sc_rxdesc_ring[i].next = (struct bufDesc *)paddr;
		else
			sc->sc_rxdesc_ring[i].next = (struct bufDesc *)paddr +
			    i + 1;
		bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
		    sizeof(struct bufDesc) * i,
		    sizeof(struct bufDesc),
		    BUS_DMASYNC_PREWRITE);
	}

	if (pge_alloc_dma(sc, sizeof(struct bufDesc) * PGE_TX_RING_CNT,
	    (void **)&(sc->sc_txdesc_ring), &(sc->sc_txdesc_dmamap)) != 0)
		return;

	memset(sc->sc_txdesc_ring, 0, sizeof(struct bufDesc) * PGE_TX_RING_CNT);

	paddr = sc->sc_txdesc_dmamap->dm_segs[0].ds_addr;

	for (i = 0; i < PGE_TX_RING_CNT; i++) {
		sc->sc_txdesc_ring[i].data = 0;
		if (i == PGE_TX_RING_CNT - 1)
			sc->sc_txdesc_ring[i].next = (struct bufDesc *)paddr;
		else
			sc->sc_txdesc_ring[i].next = (struct bufDesc *)paddr +
			    i + 1;
		bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
		    sizeof(struct bufDesc) * i,
		    sizeof(struct bufDesc),
		    BUS_DMASYNC_PREWRITE);
	}

	sc->sc_txpad = kmem_zalloc(ETHER_MIN_LEN, KM_SLEEP);
	bus_dmamap_create(sc->sc_bdt, ETHER_MIN_LEN, 1, ETHER_MIN_LEN, 0,
	    BUS_DMA_WAITOK, &sc->sc_txpad_dm);
	bus_dmamap_load(sc->sc_bdt, sc->sc_txpad_dm, sc->sc_txpad,
	    ETHER_MIN_LEN, NULL, BUS_DMA_WAITOK | BUS_DMA_WRITE);
	bus_dmamap_sync(sc->sc_bdt, sc->sc_txpad_dm, 0, ETHER_MIN_LEN,
	    BUS_DMASYNC_PREWRITE);

	sc->sc_ddrsize = PFE_TOTAL_DATA_SIZE;
	error = pge_alloc_ddr(sc);
	if (error != 0) {
		aprint_normal_dev(sc->sc_dev, "pge_alloc_ddr error %d\n",
		    error);
	}
	aprint_normal_dev(sc->sc_dev, "ddr_baseaddr: %x ddr_phys_baseaddr: %x ddr_size: %x\n", (int)sc->sc_ddr, (int)sc->sc_ddr_pa, (int)sc->sc_ddrsize);

	aprint_normal_dev(sc->sc_dev, "Ethernet address %s\n",
	    ether_sprintf(sc->sc_enaddr));

	strlcpy(ifp->if_xname, device_xname(sc->sc_dev), IFNAMSIZ);
	ifp->if_softc = sc;
	ifp->if_capabilities = 0;
	ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
	ifp->if_start = pge_start;
	ifp->if_ioctl = pge_ioctl;
	ifp->if_init = pge_init;
	ifp->if_stop = pge_stop;
	ifp->if_watchdog = pge_watchdog;
	IFQ_SET_READY(&ifp->if_snd);

	pge_stop(ifp, 0);

	mii->mii_ifp = ifp;
	mii->mii_readreg = pge_mii_readreg;
	mii->mii_writereg = pge_mii_writereg;
	mii->mii_statchg = pge_mii_statchg;

	sc->sc_ec.ec_mii = mii;
	ifmedia_init(&mii->mii_media, 0, ether_mediachange, ether_mediastatus);

/*
	pfe_gemac_init((void *)EMAC1_BASE_ADDR, MII, SPEED_100M, DUPLEX_FULL);
	pfe_gemac_init((void *)EMAC2_BASE_ADDR, MII, SPEED_100M, DUPLEX_FULL);
*/
	pge_sc = sc;
	sc->pfe.ddr_baseaddr = sc->sc_ddr;
	sc->pfe.ddr_phys_baseaddr = sc->sc_ddr_pa;
/*
	pfe_probe(&sc->pfe);
	int mac = EMAC1_BASE_ADDR;
	pfe_gemac_init((void *)mac, RGMII, SPEED_1000M, DUPLEX_FULL);
	printf("MDC: %d -> ", gemac_get_mdc_div((void *)mac));
	gemac_set_mdc_div((void *)mac, MDC_DIV_96);
	gemac_enable_mdio((void *)mac);
	printf("%d\n", gemac_get_mdc_div((void *)mac));
	gemac_enable_copy_all((void *)mac);
	gemac_set_bus_width((void *)mac, 32);
	gemac_enable((void *)mac);
	arswitch_writereg(sc->sc_dev, AR8X16_REG_MASK_CTRL,
	    AR8X16_MASK_CTRL_SOFT_RESET);
	delay(1000);
*/
	int mac = EMAC1_BASE_ADDR;
	gemac_set_mdc_div((void *)mac, MDC_DIV_96);
	gemac_enable_mdio((void *)mac);
	uint16_t val;
	printf("SWPHY:");
	for (i = 0; i < 5; ++i) {
		pge_mii_readreg(sc->sc_dev, i, 1, &val);
		printf(" %x", val);
	}
	printf("\n");
		aprint_normal_dev(sc->sc_dev, "arswitch %x\n",
		    arswitch_readreg(self, AR8X16_REG_MASK_CTRL));

#if 0
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
	}

#endif
	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr);

	/* The attach is successful. */
	sc->sc_attached = true;

	return;
}

static int
pge_detach(device_t self, int flags)
{

	return 0;
}

static int
pge_alloc_dma(struct pge_softc *sc, size_t size, void **addrp,
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

static void
pge_watchdog(struct ifnet *ifp)
{
	struct pge_softc *sc = ifp->if_softc;

	device_printf(sc->sc_dev, "device timeout %d\n", sc->sc_txnext);

	if_statinc(ifp, if_oerrors);
}

#define EMAC_PHY_IDLE   (1 << 2)

static int
pge_mii_timeout(struct pge_softc * const sc, int base)
{
	int timeout = 5000;

	while (!(pge_read_4(sc, base + EMAC_NETWORK_STATUS) &
	    EMAC_PHY_IDLE)) {
		--timeout;
		if (timeout == 0) {
			device_printf(sc->sc_dev, "timeout\n");
			return -1;
		}
	}
	return 0;
}

static int
pge_mii_readreg(device_t dev, int phy, int reg, uint16_t *val)
{
	struct pge_softc * const sc = device_private(dev);
	int result, wdata, base;

	base = pge_emac_base(sc);
	wdata = 0x60020000;
	wdata |= ((phy << 23) | (reg << 18));
	pge_write_4(sc, base + EMAC_PHY_MANAGEMENT, wdata);
	if (pge_mii_timeout(sc, base) != 0)
		return -1;
	result = pge_read_4(sc, base + EMAC_PHY_MANAGEMENT);

	*val = (uint16_t)result;

	return 0;
}

static int
pge_mii_writereg(device_t dev, int phy, int reg, uint16_t val)
{
	struct pge_softc * const sc = device_private(dev);
	int wdata, base;

	base = pge_emac_base(sc);
	wdata = 0x50020000;
	wdata |= ((phy << 23) | (reg << 18) | val);
	pge_write_4(sc, base + EMAC_PHY_MANAGEMENT, wdata);
	if (pge_mii_timeout(sc, base) != 0)
		return -1;
	return 0;
}

static void
pge_mii_statchg(struct ifnet *ifp)
{
	return;
}

static int
pge_new_rxbuf(struct pge_softc * const sc, const u_int i)
{
	struct pge_ring_data * const rdp = sc->sc_rdp;
//	const u_int h = RXDESC_PREV(i);
//	struct pge_cpdma_bd bd;
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
	sc->sc_rxdesc_ring[i].data = rdp->rx_dm[i]->dm_segs[0].ds_addr;
//	sc->sc_rxdesc_ring[i].ctrl = MAX_FRAME_SIZE | BD_CTRL_DESC_EN |
//	    BD_CTRL_PKT_INT_EN | BD_CTRL_DIR | BD_CTRL_LIFM;
//	sc->sc_rxdesc_ring[i].ctrl = BD_CTRL_PKT_INT_EN | BD_CTRL_LIFM |
	sc->sc_rxdesc_ring[i].ctrl = BD_CTRL_PKT_INT_EN |
	    BD_CTRL_DIR | BD_CTRL_DESC_EN |
	    BD_BUF_LEN(rdp->rx_dm[i]->dm_segs[0].ds_len);
	sc->sc_rxdesc_ring[i].status = 0;

	bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
	    sizeof(struct bufDesc) * i, sizeof(struct bufDesc),
            BUS_DMASYNC_PREWRITE);

	return error;
}

static int
pge_init(struct ifnet *ifp)
{
	struct pge_softc * const sc = ifp->if_softc;
	int i;
//	int mac;

	pge_stop(ifp, 0);

	sc->sc_txnext = 0;

	/* Init circular RX list. */
	for (i = 0; i < PGE_RX_RING_CNT; i++) {
		pge_new_rxbuf(sc, i);
	}
	sc->sc_rxhead = 0;

	/*
	 * Give the transmit and receive rings to the chip.
	 */

	pfe_probe(&sc->pfe);

	int mac = EMAC1_BASE_ADDR;
	pfe_gemac_init((void *)mac, RGMII, SPEED_1000M, DUPLEX_FULL);

	MAC_ADDR enet_address = {0x0, 0x0};
	gemac_enet_addr_byte_mac(sc->sc_enaddr, &enet_address);
	gemac_set_laddr1((void *)EMAC1_BASE_ADDR, &enet_address);

/*
	printf("MDC: %d -> ", gemac_get_mdc_div((void *)mac));
	gemac_set_mdc_div((void *)mac, MDC_DIV_96);
	gemac_enable_mdio((void *)mac);
	printf("%d\n", gemac_get_mdc_div((void *)mac));
*/
//	gemac_enable_copy_all((void *)mac);
	gpi_enable((void *)mac);
	gemac_set_bus_width((void *)mac, 32);
	gemac_enable((void *)mac);

/*
	uint16_t val;
	printf("PHY:");
	for (i = 0; i < 5; ++i) {
		pge_mii_readreg(sc->sc_dev, i, 3, &val);
		printf(" %x", val);
	}
	printf("\n");

	aprint_normal_dev(sc->sc_dev, "arswitch %x mode %x\n",
	    arswitch_readreg(sc->sc_dev, AR8X16_REG_MASK_CTRL),
	    arswitch_readreg(sc->sc_dev, AR8X16_REG_MODE));
*/

//	callout_schedule(&sc->sc_tick_ch, hz);

	ifp->if_flags |= IFF_RUNNING;

	return 0;
}

static void
pge_stop(struct ifnet *ifp, int disable)
{
	struct pge_softc * const sc = ifp->if_softc;
	struct pge_ring_data * const rdp = sc->sc_rdp;
	u_int i;
//	int reg;

	aprint_debug_dev(sc->sc_dev, "%s: ifp %p disable %d\n", __func__,
	    ifp, disable);

	if ((ifp->if_flags & IFF_RUNNING) == 0)
		return;

//	callout_stop(&sc->sc_tick_ch);
	mii_down(&sc->sc_mii);

/*
	pge_write_4(sc, GEM_IP + GEM_IRQ_ENABLE, 0);

	reg = pge_read_4(sc, GEM_IP + GEM_NET_CONTROL);
	reg &= ~(GEM_TX_EN | GEM_RX_EN);
	pge_write_4(sc, GEM_IP + GEM_NET_CONTROL, reg);
*/

	/* Release any queued transmit buffers. */
	for (i = 0; i < PGE_TX_RING_CNT; i++) {
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

	for (i = 0; i < PGE_RX_RING_CNT; i++) {
		bus_dmamap_unload(sc->sc_bdt, rdp->rx_dm[i]);
		m_freem(rdp->rx_mb[i]);
		rdp->rx_mb[i] = NULL;
	}
}

static int
pge_intr(void *arg)
{
	struct pge_softc * const sc = arg;
	int reg;

	reg = pge_read_4(sc, HIF_INT_SRC);
	if (reg & HIF_RXPKT_INT)
		pge_rxintr(arg);
	if (reg & HIF_TXPKT_INT)
		pge_txintr(arg);

	pge_write_4(sc, HIF_INT_SRC, reg);

	return 1;
}

static void
pge_start(struct ifnet *ifp)
{
	struct pge_softc * const sc = ifp->if_softc;
	struct pge_ring_data * const rdp = sc->sc_rdp;
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

	PGE_LOCK(sc);

	if (__predict_false((ifp->if_flags & IFF_RUNNING) == 0)) {
		device_printf(sc->sc_dev, "if not run\n");
		return;
	}
	if (__predict_false(sc->sc_txbusy)) {
		device_printf(sc->sc_dev, "txbusy\n");
		return;
	}

	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    0, sizeof(struct bufDesc) * PGE_TX_RING_CNT,
	    BUS_DMASYNC_PREREAD);
	txfree = 0;
	for (i = 0;i < PGE_TX_RING_CNT; ++i) {
		if(!(sc->sc_txdesc_ring[i].ctrl & BD_CTRL_DESC_EN))
			++txfree;
	}

	while (txfree > 0) {
		len = 0;
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

		m->m_data -= sizeof(hif_header_t);
		m->m_len += sizeof(hif_header_t);
		m->m_data[0] = 0;
		for (i = 1; i < 6; ++i)
			m->m_data[i] = 0;
/*
		for (i = 0; i < 16; ++i) {
			printf(" %02x", m->m_data[i]);
		}
		printf("\n");
*/
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
		pad = mlen < PGE_MIN_FRAMELEN;

		KASSERT(rdp->tx_mb[sc->sc_txnext] == NULL);
		rdp->tx_mb[sc->sc_txnext] = m;
		IFQ_DEQUEUE(&ifp->if_snd, m);

		bus_dmamap_sync(sc->sc_bdt, dm, 0, dm->dm_mapsize,
		    BUS_DMASYNC_PREWRITE);

		if (txstart == -1)
			txstart = sc->sc_txnext;
		for (seg = 0; seg < dm->dm_nsegs; seg++) {
			sc->sc_txdesc_ring[sc->sc_txnext].data =
			    dm->dm_segs[seg].ds_addr;
			sc->sc_txdesc_ring[sc->sc_txnext].ctrl =
			    dm->dm_segs[seg].ds_len;
			len += dm->dm_segs[seg].ds_len;
			sc->sc_txdesc_ring[sc->sc_txnext].ctrl |=
				    BD_CTRL_DESC_EN;
			if (!pad && (seg == dm->dm_nsegs - 1))
				sc->sc_txdesc_ring[sc->sc_txnext].ctrl |=
				    (BD_CTRL_LIFM | BD_CTRL_BRFETCH_DISABLE |
				    BD_CTRL_RTFETCH_DISABLE |
				    BD_CTRL_PARSE_DISABLE |
				    BD_CTRL_PKT_INT_EN);
			if (seg == 0)
				sc->sc_txdesc_ring[sc->sc_txnext].status = 1;
			else
				sc->sc_txdesc_ring[sc->sc_txnext].status = 0;
			txfree--;
			bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
			    sizeof(struct bufDesc) * sc->sc_txnext,
			    sizeof(struct bufDesc), BUS_DMASYNC_PREWRITE);
			sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
		}
		if (pad) {
			sc->sc_txdesc_ring[sc->sc_txnext].data =
			    sc->sc_txpad_pa;
			sc->sc_txdesc_ring[sc->sc_txnext].ctrl =
			    PGE_MIN_FRAMELEN - mlen;
			len += PGE_MIN_FRAMELEN - mlen;
			sc->sc_txdesc_ring[sc->sc_txnext].ctrl |=
			    (BD_CTRL_DESC_EN | BD_CTRL_LIFM |
			    BD_CTRL_BRFETCH_DISABLE | BD_CTRL_RTFETCH_DISABLE |
			    BD_CTRL_PARSE_DISABLE | BD_CTRL_PKT_INT_EN);
			txfree--;
			bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
			    sizeof(struct bufDesc) * sc->sc_txnext,
			    sizeof(struct bufDesc), BUS_DMASYNC_PREWRITE);
			sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
		}
//		pge_write_4(sc, GEM_SCH_BLOCK + SCH_PACKET_QUEUED, len);
		pge_write_4(sc, HIF_TX_CTRL, HIF_CTRL_DMA_EN |
		    HIF_CTRL_BDP_CH_START_WSTB);
		bpf_mtap(ifp, m, BPF_D_OUT);
	}

	if (txstart >= 0) {
		ifp->if_timer = 300;	/* not immediate interrupt */
	}

	PGE_UNLOCK(sc);
}

static int
pge_ioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	struct pge_softc * const sc = ifp->if_softc;
	const int s = splnet();
	int error = 0;
	int reg;
	uint32_t base;

	base = pge_emac_base(sc);

	switch (cmd) {
	case SIOCSIFFLAGS:
		if ((ifp->if_flags & (IFF_PROMISC | IFF_ALLMULTI)) != 0) {
			reg = pge_read_4(sc, base + EMAC_NETWORK_CONFIG);
			reg |= EMAC_ENABLE_COPY_ALL;
			pge_write_4(sc, base + EMAC_NETWORK_CONFIG, reg);
		} else {
			reg = pge_read_4(sc, base + EMAC_NETWORK_CONFIG);
			reg &= ~EMAC_ENABLE_COPY_ALL;
			pge_write_4(sc, base + EMAC_NETWORK_CONFIG, reg);
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
pge_tick(void *arg)
{
	struct pge_softc * const sc = arg;
	uint32_t reg;

	int use = 0;
	int i;
	for (i = 0; i < PGE_TX_RING_CNT; i++) {
	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    sizeof(struct bufDesc) * i, sizeof(struct bufDesc),
	    BUS_DMASYNC_PREREAD);
	if(sc->sc_txdesc_ring[i].ctrl & BD_CTRL_DESC_EN)
		++use;
	}
printf("MORIMORI %d\n", use);
/*
	if(use) {
	reg = pge_read_4(sc, HIF_RX_CTRL);
	reg |= HIF_CTRL_BDP_CH_START_WSTB;
	pge_write_4(sc, HIF_RX_CTRL, reg);
	}
*/
	reg = pge_read_4(sc, HIF_TX_BDP_ADDR);
	printf("TXDESC reg=%x", reg);
	reg = pge_read_4(sc, HIF_TX_CURR_BD_ADDR);
	printf(" %x", reg);
	reg = pge_read_4(sc, HIF_RX_BDP_ADDR);
	printf(" RXDESC reg=%x", reg);
	reg = pge_read_4(sc, HIF_RX_CURR_BD_ADDR);
	printf(" %x\n", reg);

	printf("TX:");
	int ptr = EMAC1_BASE_ADDR + EMAC_OCT_TX_BOT +
	    offsetof(struct gem_stats, frames_tx);
	for (i = 0; i < 23; i++) {
		reg = pge_read_4(sc, ptr + i * 4);
		printf(" %d", reg);
	}
	printf("\n");

	callout_schedule(&sc->sc_tick_ch, hz);
}

static int
pge_alloc_ddr(struct pge_softc *sc)
{
	int error, nsegs;

	error = bus_dmamem_alloc(sc->sc_bdt, sc->sc_ddrsize, 0x1000, 0,
	    sc->sc_ddrsegs, 1, &nsegs, BUS_DMA_WAITOK);
	if (error)
		return error;
	error = bus_dmamem_map(sc->sc_bdt, sc->sc_ddrsegs, nsegs,
	    sc->sc_ddrsize, &sc->sc_ddr, BUS_DMA_WAITOK | BUS_DMA_COHERENT);
	if (error)
		goto free;
	error = bus_dmamap_create(sc->sc_bdt, sc->sc_ddrsize, 1,
	    sc->sc_ddrsize, 0, BUS_DMA_WAITOK, &sc->sc_ddrmap);
	if (error)
		goto unmap;
	error = bus_dmamap_load(sc->sc_bdt, sc->sc_ddrmap, sc->sc_ddr,
	    sc->sc_ddrsize, NULL, BUS_DMA_WAITOK);
	if (error)
		goto destroy;

	memset(sc->sc_ddr, 0, sc->sc_ddrsize);

	return 0;

destroy:
	bus_dmamap_destroy(sc->sc_bdt, sc->sc_ddrmap);
unmap:
	bus_dmamem_unmap(sc->sc_bdt, sc->sc_ddr, sc->sc_ddrsize);
free:
	bus_dmamem_free(sc->sc_bdt, sc->sc_ddrsegs, nsegs);

	sc->sc_ddrsize = 0;
	sc->sc_ddr = NULL;

	return error;
}


static int
pge_rxintr(void *arg)
{
	struct pge_softc * const sc = arg;
	struct pge_ring_data * const rdp = sc->sc_rdp;
	u_int i;
	int length;
	struct mbuf *m;
	struct ifnet *ifp = &sc->sc_ec.ec_if;
	int count;

	count = 0;
	for (;;) {
		i = sc->sc_rxhead;
		bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
		    sizeof(struct bufDesc) * i, sizeof(struct bufDesc),
		    BUS_DMASYNC_PREREAD);
		if(!(sc->sc_rxdesc_ring[i].ctrl & BD_CTRL_DESC_EN)) {
			length = BD_BUF_LEN(sc->sc_rxdesc_ring[i].ctrl);
			length -= sizeof(hif_header_t);
			m = rdp->rx_mb[i];
			if (length < ETHER_HDR_LEN) {
				aprint_error_dev(sc->sc_dev,
				    "RX error packe length\n");
				m_freem(m);
			} else {
				bus_dmamap_sync(sc->sc_bdt, rdp->rx_dm[i],
				    0, rdp->rx_dm[i]->dm_mapsize,
				    BUS_DMASYNC_POSTREAD);
/*
int j;
for (j = 0; j < 8; ++j) {
for (i = 0; i < 16; ++i) {
printf(" %02x", *(m->m_data + i + j * 16));
}
printf("\n");
}*/
				m_set_rcvif(m, ifp);
				m->m_pkthdr.len = m->m_len = length;
				m->m_data += sizeof(hif_header_t);
				if_percpuq_enqueue(ifp->if_percpuq, m);
			}
			pge_new_rxbuf(sc, i);

			bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
			    sizeof(struct bufDesc) * i, sizeof(struct bufDesc),
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
pge_txintr(void *arg)
{
	struct pge_softc * const sc = arg;
	struct pge_ring_data * const rdp = sc->sc_rdp;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	bool handled = false;
	int i, remain;

	remain = 0;
	bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
	    0, sizeof(struct bufDesc) * PGE_TX_RING_CNT, BUS_DMASYNC_PREREAD);
	for (i = 0; i < PGE_TX_RING_CNT; i++) {
		if (sc->sc_txdesc_ring[i].ctrl & BD_CTRL_DESC_EN) {
			++remain;
			continue;
		}

		if (sc->sc_txdesc_ring[i].status == 0)
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

		sc->sc_txdesc_ring[i].ctrl = 0;
		sc->sc_txdesc_ring[i].status = 0;
		bus_dmamap_sync(sc->sc_bdt, sc->sc_txdesc_dmamap,
		    sizeof(struct bufDesc) * i, sizeof(struct bufDesc),
		    BUS_DMASYNC_PREWRITE);

	}

	if (remain == 0)
		ifp->if_timer = 0;
	if (handled)
		if_schedule_deferred_start(ifp);

	return handled;
}

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
	pge_mii_writereg(dev, phy, MII_ATH_DBG_ADDR, dbg_addr);
	pge_mii_writereg(dev, phy, MII_ATH_DBG_DATA, dbg_data);
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
		pge_mii_writereg(dev, 0x18, 0, page);
		delay(2000);
//		sc->page = page;
//	}
}

static uint32_t
arswitch_reg_read32(device_t dev, int phy, int reg)
{
	uint16_t lo, hi;
	pge_mii_readreg(dev, phy, reg, &lo);
	pge_mii_readreg(dev, phy, reg + 1, &hi);

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

//	if (sc->mii_lo_first) {
		r = pge_mii_writereg(dev, phy, reg, lo);
		r |= pge_mii_writereg(dev, phy, reg + 1, hi);
/*
	} else {
		r = pge_mii_writereg(dev, phy, reg + 1, hi);
		r |= pge_mii_writereg(dev, phy, reg, lo);
	}
*/

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
