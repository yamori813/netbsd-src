/*	$NetBSD$	*/

/*
 * Copyright (c) 2022 Hiroki Mori
 * Copyright (c) 2013 Jonathan A. Kollasch
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

/*-
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
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

#define CGE_TXFRAGS	16

#if 0
#define FDT_INTR_FLAGS	0

#define CPSW_CPPI_RAM_SIZE (0x2000)
#define CPSW_CPPI_RAM_TXDESCS_SIZE (CPSW_CPPI_RAM_SIZE/2)
#define CPSW_CPPI_RAM_RXDESCS_SIZE \
    (CPSW_CPPI_RAM_SIZE - CPSW_CPPI_RAM_TXDESCS_SIZE)
#define CPSW_CPPI_RAM_TXDESCS_BASE (CPSW_CPPI_RAM_OFFSET + 0x0000)
#define CPSW_CPPI_RAM_RXDESCS_BASE \
    (CPSW_CPPI_RAM_OFFSET + CPSW_CPPI_RAM_TXDESCS_SIZE)

#define CPSW_NTXDESCS (CPSW_CPPI_RAM_TXDESCS_SIZE/sizeof(struct cge_cpdma_bd))
#define CPSW_NRXDESCS (CPSW_CPPI_RAM_RXDESCS_SIZE/sizeof(struct cge_cpdma_bd))

CTASSERT(powerof2(CPSW_NTXDESCS));
CTASSERT(powerof2(CPSW_NRXDESCS));

#define CPSW_PAD_LEN (ETHER_MIN_LEN - ETHER_CRC_LEN)

struct cge_ring_data {
	bus_dmamap_t tx_dm[CPSW_NTXDESCS];
	struct mbuf *tx_mb[CPSW_NTXDESCS];
	bus_dmamap_t rx_dm[CPSW_NRXDESCS];
	struct mbuf *rx_mb[CPSW_NRXDESCS];
};

struct cge_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	bus_size_t sc_bss;
	bus_dma_tag_t sc_bdt;
	bus_space_handle_t sc_bsh_txdescs;
	bus_space_handle_t sc_bsh_rxdescs;
	bus_addr_t sc_txdescs_pa;
	bus_addr_t sc_rxdescs_pa;
	struct ethercom sc_ec;
	struct mii_data sc_mii;
	bool sc_phy_has_1000t;
	bool sc_attached;
	callout_t sc_tick_ch;
	void *sc_ih;
	struct cge_ring_data *sc_rdp;
	volatile u_int sc_txnext;
	volatile u_int sc_txhead;
	volatile u_int sc_rxhead;
	bool sc_txbusy;
	void *sc_rxthih;
	void *sc_rxih;
	void *sc_txih;
	void *sc_miscih;
	void *sc_txpad;
	bus_dmamap_t sc_txpad_dm;
#define sc_txpad_pa sc_txpad_dm->dm_segs[0].ds_addr
	uint8_t sc_enaddr[ETHER_ADDR_LEN];
	volatile bool sc_txrun;
	volatile bool sc_rxrun;
	volatile bool sc_txeoq;
	volatile bool sc_rxeoq;
};
#endif
#define TXDESC_NEXT(x) cge_txdesc_adjust((x), 1)
#define TXDESC_PREV(x) cge_txdesc_adjust((x), -1)

#define RXDESC_NEXT(x) cge_rxdesc_adjust((x), 1)
#define RXDESC_PREV(x) cge_rxdesc_adjust((x), -1)

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
#if 0
static int cge_rxthintr(void *);
static int cge_txintr(void *);
static int cge_miscintr(void *);
#endif

static int cge_alloc_dma(struct cge_softc *, size_t, void **,bus_dmamap_t *);

CFATTACH_DECL_NEW(cge, sizeof(struct cge_softc),
    cge_match, cge_attach, cge_detach, NULL);

#include <sys/kernhist.h>
KERNHIST_DEFINE(cgehist);

#define CPSWHIST_CALLARGS(A,B,C,D)	do {					\
	    KERNHIST_CALLARGS(cgehist, "%jx %jx %jx %jx",			\
		(uintptr_t)(A), (uintptr_t)(B), (uintptr_t)(C), (uintptr_t)(D));\
	} while (0)

int arswitch_readreg(device_t dev, int addr);
int arswitch_writereg(device_t dev, int addr, int value);

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
	return (((x) + y) & (CGE_TX_RING_CNT - 1));
}

static inline u_int
cge_rxdesc_adjust(u_int x, int y)
{
	return (((x) + y) & (CGE_RX_RING_CNT - 1));
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

#if 0
static inline void
cge_set_txdesc_next(struct cge_softc * const sc, const u_int i, uint32_t n)
{
	const bus_size_t o = sizeof(struct cge_cpdma_bd) * i + 0;

	KERNHIST_FUNC(__func__);
	CPSWHIST_CALLARGS(sc, i, n, 0);

	bus_space_write_4(sc->sc_bst, sc->sc_bsh_txdescs, o, n);
}
#endif

static inline void
cge_set_rxdesc_next(struct cge_softc * const sc, const u_int i, uint32_t n)
{
	const bus_size_t o = sizeof(struct cge_cpdma_bd) * i + 0;

	KERNHIST_FUNC(__func__);
	CPSWHIST_CALLARGS(sc, i, n, 0);

	bus_space_write_4(sc->sc_bst, sc->sc_bsh_rxdescs, o, n);
}

#if 0
static inline void
cge_get_txdesc(struct cge_softc * const sc, const u_int i,
    struct cge_cpdma_bd * const bdp)
{
	const bus_size_t o = sizeof(struct cge_cpdma_bd) * i;
	uint32_t * const dp = bdp->word;
	const bus_size_t c = __arraycount(bdp->word);

	KERNHIST_FUNC(__func__);
	CPSWHIST_CALLARGS(sc, i, bdp, 0);

	bus_space_read_region_4(sc->sc_bst, sc->sc_bsh_txdescs, o, dp, c);
	KERNHIST_LOG(cgehist, "%08x %08x %08x %08x\n",
	    dp[0], dp[1], dp[2], dp[3]);
}

static inline void
cge_set_txdesc(struct cge_softc * const sc, const u_int i,
    struct cge_cpdma_bd * const bdp)
{
	const bus_size_t o = sizeof(struct cge_cpdma_bd) * i;
	uint32_t * const dp = bdp->word;
	const bus_size_t c = __arraycount(bdp->word);

	KERNHIST_FUNC(__func__);
	CPSWHIST_CALLARGS(sc, i, bdp, 0);
	KERNHIST_LOG(cgehist, "%08x %08x %08x %08x\n",
	    dp[0], dp[1], dp[2], dp[3]);

	bus_space_write_region_4(sc->sc_bst, sc->sc_bsh_txdescs, o, dp, c);
}

static inline void
cge_get_rxdesc(struct cge_softc * const sc, const u_int i,
    struct cge_cpdma_bd * const bdp)
{
	const bus_size_t o = sizeof(struct cge_cpdma_bd) * i;
	uint32_t * const dp = bdp->word;
	const bus_size_t c = __arraycount(bdp->word);

	KERNHIST_FUNC(__func__);
	CPSWHIST_CALLARGS(sc, i, bdp, 0);

	bus_space_read_region_4(sc->sc_bst, sc->sc_bsh_rxdescs, o, dp, c);

	KERNHIST_LOG(cgehist, "%08x %08x %08x %08x\n",
	    dp[0], dp[1], dp[2], dp[3]);
}
#endif

static inline void
cge_set_rxdesc(struct cge_softc * const sc, const u_int i,
    struct cge_cpdma_bd * const bdp)
{
	const bus_size_t o = sizeof(struct cge_cpdma_bd) * i;
	uint32_t * const dp = bdp->word;
	const bus_size_t c = __arraycount(bdp->word);

	KERNHIST_FUNC(__func__);
	CPSWHIST_CALLARGS(sc, i, bdp, 0);
	KERNHIST_LOG(cgehist, "%08x %08x %08x %08x\n",
	    dp[0], dp[1], dp[2], dp[3]);

	bus_space_write_region_4(sc->sc_bst, sc->sc_bsh_rxdescs, o, dp, c);
}

#if 0
static inline bus_addr_t
cge_txdesc_paddr(struct cge_softc * const sc, u_int x)
{
	KASSERT(x < CGE_TX_RING_CNT);
	return sc->sc_txdescs_pa + sizeof(struct cge_cpdma_bd) * x;
}
#endif

static inline bus_addr_t
cge_rxdesc_paddr(struct cge_softc * const sc, u_int x)
{
	KASSERT(x < CGE_RX_RING_CNT);
	return sc->sc_rxdescs_pa + sizeof(struct cge_cpdma_bd) * x;
}

#if 0
static const struct device_compatible_entry compat_data[] = {
	{ .compat = "ti,am335x-cge-switch" },
	{ .compat = "ti,am335x-cge" },
	{ .compat = "ti,cge" },
	DEVICE_COMPAT_EOL
};
#endif

static int
cge_match(device_t parent, cfdata_t cf, void *aux)
{

	return 1;
}

#if 0
static bool
cge_phy_has_1000t(struct cge_softc * const sc)
{
	struct ifmedia_entry *ifm;

	TAILQ_FOREACH(ifm, &sc->sc_mii.mii_media.ifm_list, ifm_list) {
		if (IFM_SUBTYPE(ifm->ifm_media) == IFM_1000_T)
			return true;
	}
	return false;
}
#endif

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

	/* Let go of the interrupts */
	intr_disestablish(sc->sc_rxthih);
	intr_disestablish(sc->sc_rxih);
	intr_disestablish(sc->sc_txih);
	intr_disestablish(sc->sc_miscih);
#endif

	ether_ifdetach(ifp);
	if_detach(ifp);

	/* Delete all media. */
	ifmedia_fini(&sc->sc_mii.mii_media);

	/* Free the packet padding buffer */
	kmem_free(sc->sc_txpad, ETHER_MIN_LEN);
	bus_dmamap_destroy(sc->sc_bdt, sc->sc_txpad_dm);

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

//	callout_init(&sc->sc_tick_ch, 0);
//	callout_setfunc(&sc->sc_tick_ch, cge_tick, sc);

//	error = bus_space_map(aa->apba_memt, aa->apba_addr, aa->apba_size,
	error = bus_space_map(aa->apba_memt, aa->apba_addr, sc->sc_bss,
	    0, &sc->sc_bsh);
	if (error) {
		aprint_error_dev(sc->sc_dev,
			"can't map registers: %d\n", error);
		return;
	}

	intr_establish(aa->apba_intr, IPL_NET, IST_LEVEL,
	    cge_intr, sc);

	sc->sc_enaddr[0] = 0xd4;
	sc->sc_enaddr[1] = 0x94;
	sc->sc_enaddr[2] = 0xa1;
	sc->sc_enaddr[3] = 0x97;
	sc->sc_enaddr[4] = 0x03;
	sc->sc_enaddr[5] = 0x94;

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

	memset(sc->sc_rxdesc_ring, 0, sizeof(struct tRXdesc) * CGE_RX_RING_CNT);
/*
	sc->sc_txpad = kmem_zalloc(ETHER_MIN_LEN, KM_SLEEP);
	bus_dmamap_create(sc->sc_bdt, ETHER_MIN_LEN, 1, ETHER_MIN_LEN, 0,
	    BUS_DMA_WAITOK, &sc->sc_txpad_dm);
	bus_dmamap_load(sc->sc_bdt, sc->sc_txpad_dm, sc->sc_txpad,
	    ETHER_MIN_LEN, NULL, BUS_DMA_WAITOK | BUS_DMA_WRITE);
	bus_dmamap_sync(sc->sc_bdt, sc->sc_txpad_dm, 0, ETHER_MIN_LEN,
	    BUS_DMASYNC_PREWRITE);
*/

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

/*
	uint16_t val;
	for (i = 0; i < 32; ++i) {
	cge_mii_readreg(self, i, 3, &val);
	printf("%04x ", val);
	}
	printf("\n");
*/
	if (device_unit(self) == 0) {
		arswitch_writereg(self, 8, 0x81461bea);
		aprint_normal_dev(sc->sc_dev, "arswitch %x mode %x\n",
		    arswitch_readreg(self, 0),
		    arswitch_readreg(self, 8));
//		arswitch_writereg(self, 0, (1 << 31));
	}

#if 0
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_QFULLTHR, CGE_RX_RING_CNT - 2);
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_QDROPMAXTHR,
	    (CGE_RX_RING_CNT - 8 - 6) << 8);
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_QDROPMINTHR,
	    (CGE_RX_RING_CNT - 8 - 6 - 6) << 8);
#endif

	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr);

	/* The attach is successful. */
	sc->sc_attached = true;
#if 0
	struct fdt_attach_args * const faa = aux;
	struct cge_softc * const sc = device_private(self);
	struct ethercom * const ec = &sc->sc_ec;
	struct ifnet * const ifp = &ec->ec_if;
	struct mii_data * const mii = &sc->sc_mii;
	const int phandle = faa->faa_phandle;
	const uint8_t *macaddr;
	bus_addr_t addr;
	bus_size_t size;
	int error, slave, len;
	char xname[16];
	u_int i;

	KERNHIST_INIT(cgehist, 4096);

	if (fdtbus_get_reg(phandle, 0, &addr, &size) != 0) {
		aprint_error(": couldn't get registers\n");
		return;
	}

	sc->sc_dev = self;

	aprint_normal(": TI Layer 2 3-Port Switch\n");
	aprint_naive("\n");

	callout_init(&sc->sc_tick_ch, 0);
	callout_setfunc(&sc->sc_tick_ch, cge_tick, sc);

	macaddr = NULL;
	slave = of_find_firstchild_byname(phandle, "slave");
	if (slave == -1) {
		slave = of_find_firstchild_byname(phandle, "ethernet-ports");
		if (slave != -1) {
			slave = of_find_firstchild_byname(slave, "port");
		}
	}
	if (slave != -1) {
		macaddr = fdtbus_get_prop(slave, "mac-address", &len);
		if (len != ETHER_ADDR_LEN)
			macaddr = NULL;
	}
	if (macaddr == NULL) {
#if 0
		/* grab mac_id0 from AM335x control module */
		uint32_t reg_lo, reg_hi;

		if (sitara_cm_reg_read_4(OMAP2SCM_MAC_ID0_LO, &reg_lo) == 0 &&
		    sitara_cm_reg_read_4(OMAP2SCM_MAC_ID0_HI, &reg_hi) == 0) {
			sc->sc_enaddr[0] = (reg_hi >>  0) & 0xff;
			sc->sc_enaddr[1] = (reg_hi >>  8) & 0xff;
			sc->sc_enaddr[2] = (reg_hi >> 16) & 0xff;
			sc->sc_enaddr[3] = (reg_hi >> 24) & 0xff;
			sc->sc_enaddr[4] = (reg_lo >>  0) & 0xff;
			sc->sc_enaddr[5] = (reg_lo >>  8) & 0xff;
		} else
#endif
		{
			aprint_error_dev(sc->sc_dev,
			    "using fake station address\n");
			/* 'N' happens to have the Local bit set */
#if 0
			sc->sc_enaddr[0] = 'N';
			sc->sc_enaddr[1] = 'e';
			sc->sc_enaddr[2] = 't';
			sc->sc_enaddr[3] = 'B';
			sc->sc_enaddr[4] = 'S';
			sc->sc_enaddr[5] = 'D';
#else
			/* XXX Glor */
			sc->sc_enaddr[0] = 0xd4;
			sc->sc_enaddr[1] = 0x94;
			sc->sc_enaddr[2] = 0xa1;
			sc->sc_enaddr[3] = 0x97;
			sc->sc_enaddr[4] = 0x03;
			sc->sc_enaddr[5] = 0x94;
#endif
		}
	} else {
		memcpy(sc->sc_enaddr, macaddr, ETHER_ADDR_LEN);
	}

	snprintf(xname, sizeof(xname), "%s rxth", device_xname(self));
	sc->sc_rxthih = fdtbus_intr_establish_xname(phandle, CPSW_INTROFF_RXTH,
	    IPL_VM, FDT_INTR_FLAGS, cge_rxthintr, sc, xname);

	snprintf(xname, sizeof(xname), "%s rx", device_xname(self));
	sc->sc_rxih = fdtbus_intr_establish_xname(phandle, CPSW_INTROFF_RX,
	    IPL_VM, FDT_INTR_FLAGS, cge_rxintr, sc, xname);

	snprintf(xname, sizeof(xname), "%s tx", device_xname(self));
	sc->sc_txih = fdtbus_intr_establish_xname(phandle, CPSW_INTROFF_TX,
	    IPL_VM, FDT_INTR_FLAGS, cge_txintr, sc, xname);

	snprintf(xname, sizeof(xname), "%s misc", device_xname(self));
	sc->sc_miscih = fdtbus_intr_establish_xname(phandle, CPSW_INTROFF_MISC,
	    IPL_VM, FDT_INTR_FLAGS, cge_miscintr, sc, xname);

	sc->sc_bst = faa->faa_bst;
	sc->sc_bss = size;
	sc->sc_bdt = faa->faa_dmat;

	error = bus_space_map(sc->sc_bst, addr, size, 0,
	    &sc->sc_bsh);
	if (error) {
		aprint_error_dev(sc->sc_dev,
			"can't map registers: %d\n", error);
		return;
	}

	sc->sc_txdescs_pa = addr + CPSW_CPPI_RAM_TXDESCS_BASE;
	error = bus_space_subregion(sc->sc_bst, sc->sc_bsh,
	    CPSW_CPPI_RAM_TXDESCS_BASE, CPSW_CPPI_RAM_TXDESCS_SIZE,
	    &sc->sc_bsh_txdescs);
	if (error) {
		aprint_error_dev(sc->sc_dev,
			"can't subregion tx ring SRAM: %d\n", error);
		return;
	}
	aprint_debug_dev(sc->sc_dev, "txdescs at %p\n",
	    (void *)sc->sc_bsh_txdescs);

	sc->sc_rxdescs_pa = addr + CPSW_CPPI_RAM_RXDESCS_BASE;
	error = bus_space_subregion(sc->sc_bst, sc->sc_bsh,
	    CPSW_CPPI_RAM_RXDESCS_BASE, CPSW_CPPI_RAM_RXDESCS_SIZE,
	    &sc->sc_bsh_rxdescs);
	if (error) {
		aprint_error_dev(sc->sc_dev,
			"can't subregion rx ring SRAM: %d\n", error);
		return;
	}
	aprint_debug_dev(sc->sc_dev, "rxdescs at %p\n",
	    (void *)sc->sc_bsh_rxdescs);

	sc->sc_rdp = kmem_alloc(sizeof(*sc->sc_rdp), KM_SLEEP);

	for (i = 0; i < CPSW_NTXDESCS; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES,
		    CPSW_TXFRAGS, MCLBYTES, 0, 0,
		    &sc->sc_rdp->tx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create tx DMA map: %d\n", error);
		}
		sc->sc_rdp->tx_mb[i] = NULL;
	}

	for (i = 0; i < CPSW_NRXDESCS; i++) {
		if ((error = bus_dmamap_create(sc->sc_bdt, MCLBYTES, 1,
		    MCLBYTES, 0, 0, &sc->sc_rdp->rx_dm[i])) != 0) {
			aprint_error_dev(sc->sc_dev,
			    "unable to create rx DMA map: %d\n", error);
		}
		sc->sc_rdp->rx_mb[i] = NULL;
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

	/* Initialize MDIO */
	cge_write_4(sc, MDIOCONTROL,
	    MDIOCTL_ENABLE | MDIOCTL_FAULTENB | MDIOCTL_CLKDIV(0xff));
	/* Clear ALE */
	cge_write_4(sc, CPSW_ALE_CONTROL, ALECTL_CLEAR_TABLE);

	mii_attach(self, mii, 0xffffffff, MII_PHY_ANY, 0, 0);
	if (LIST_FIRST(&mii->mii_phys) == NULL) {
		aprint_error_dev(self, "no PHY found!\n");
		sc->sc_phy_has_1000t = false;
		ifmedia_add(&mii->mii_media, IFM_ETHER | IFM_MANUAL, 0, NULL);
		ifmedia_set(&mii->mii_media, IFM_ETHER | IFM_MANUAL);
	} else {
		sc->sc_phy_has_1000t = cge_phy_has_1000t(sc);

		ifmedia_set(&mii->mii_media, IFM_ETHER | IFM_AUTO);
	}

	if_attach(ifp);
	if_deferred_start_init(ifp, NULL);
	ether_ifattach(ifp, sc->sc_enaddr);

	/* The attach is successful. */
	sc->sc_attached = true;

#endif
	return;
}

static void
cge_start(struct ifnet *ifp)
{
#if 0
	struct cge_softc * const sc = ifp->if_softc;
	struct cge_ring_data * const rdp = sc->sc_rdp;
	struct cge_cpdma_bd bd;
	uint32_t * const dw = bd.word;
	struct mbuf *m;
	bus_dmamap_t dm;
	u_int eopi __diagused = ~0;
	u_int seg;
	u_int txfree;
	int txstart = -1;
	int error;
	bool pad;
	u_int mlen;

	KERNHIST_FUNC(__func__);
	CPSWHIST_CALLARGS(sc, 0, 0, 0);

	if (__predict_false((ifp->if_flags & IFF_RUNNING) == 0)) {
		return;
	}
	if (__predict_false(sc->sc_txbusy)) {
		return;
	}

	if (sc->sc_txnext >= sc->sc_txhead)
		txfree = CPSW_NTXDESCS - 1 + sc->sc_txhead - sc->sc_txnext;
	else
		txfree = sc->sc_txhead - sc->sc_txnext - 1;

	KERNHIST_LOG(cgehist, "start txf %x txh %x txn %x txr %x\n",
	    txfree, sc->sc_txhead, sc->sc_txnext, sc->sc_txrun);

	while (txfree > 0) {
		IFQ_POLL(&ifp->if_snd, m);
		if (m == NULL)
			break;

		dm = rdp->tx_dm[sc->sc_txnext];

		error = bus_dmamap_load_mbuf(sc->sc_bdt, dm, m, BUS_DMA_NOWAIT);
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
		pad = mlen < CPSW_PAD_LEN;

		KASSERT(rdp->tx_mb[sc->sc_txnext] == NULL);
		rdp->tx_mb[sc->sc_txnext] = m;
		IFQ_DEQUEUE(&ifp->if_snd, m);

		bus_dmamap_sync(sc->sc_bdt, dm, 0, dm->dm_mapsize,
		    BUS_DMASYNC_PREWRITE);

		if (txstart == -1)
			txstart = sc->sc_txnext;
		eopi = sc->sc_txnext;
		for (seg = 0; seg < dm->dm_nsegs; seg++) {
			dw[0] = cge_txdesc_paddr(sc,
			    TXDESC_NEXT(sc->sc_txnext));
			dw[1] = dm->dm_segs[seg].ds_addr;
			dw[2] = dm->dm_segs[seg].ds_len;
			dw[3] = 0;

			if (seg == 0)
				dw[3] |= CPDMA_BD_SOP | CPDMA_BD_OWNER |
				    MAX(mlen, CPSW_PAD_LEN);

			if ((seg == dm->dm_nsegs - 1) && !pad)
				dw[3] |= CPDMA_BD_EOP;

			cge_set_txdesc(sc, sc->sc_txnext, &bd);
			txfree--;
			eopi = sc->sc_txnext;
			sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
		}
		if (pad) {
			dw[0] = cge_txdesc_paddr(sc,
			    TXDESC_NEXT(sc->sc_txnext));
			dw[1] = sc->sc_txpad_pa;
			dw[2] = CPSW_PAD_LEN - mlen;
			dw[3] = CPDMA_BD_EOP;

			cge_set_txdesc(sc, sc->sc_txnext, &bd);
			txfree--;
			eopi = sc->sc_txnext;
			sc->sc_txnext = TXDESC_NEXT(sc->sc_txnext);
		}

		bpf_mtap(ifp, m, BPF_D_OUT);
	}

	if (txstart >= 0) {
		ifp->if_timer = 5;
		/* terminate the new chain */
		KASSERT(eopi == TXDESC_PREV(sc->sc_txnext));
		cge_set_txdesc_next(sc, TXDESC_PREV(sc->sc_txnext), 0);
		KERNHIST_LOG(cgehist, "CP %x HDP %x s %x e %x\n",
		    cge_read_4(sc, CPSW_CPDMA_TX_CP(0)),
		    cge_read_4(sc, CPSW_CPDMA_TX_HDP(0)), txstart, eopi);
		/* link the new chain on */
		cge_set_txdesc_next(sc, TXDESC_PREV(txstart),
		    cge_txdesc_paddr(sc, txstart));
		if (sc->sc_txeoq) {
			/* kick the dma engine */
			sc->sc_txeoq = false;
			cge_write_4(sc, CPSW_CPDMA_TX_HDP(0),
			    cge_txdesc_paddr(sc, txstart));
		}
	}
	KERNHIST_LOG(cgehist, "end txf %x txh %x txn %x txr %x\n",
	    txfree, sc->sc_txhead, sc->sc_txnext, sc->sc_txrun);
#endif
}

static int
cge_ioctl(struct ifnet *ifp, u_long cmd, void *data)
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
cge_watchdog(struct ifnet *ifp)
{
#if 0
	struct cge_softc *sc = ifp->if_softc;

	device_printf(sc->sc_dev, "device timeout\n");

	if_statinc(ifp, if_oerrors);
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
#if 0
	/* (re-)setup the descriptor */
	dw[0] = 0;
	dw[1] = rdp->rx_dm[i]->dm_segs[0].ds_addr;
	dw[2] = MIN(0x7ff, rdp->rx_dm[i]->dm_segs[0].ds_len);
//	dw[3] = CPDMA_BD_OWNER;

	cge_set_rxdesc(sc, i, &bd);
	/* and link onto ring */
	cge_set_rxdesc_next(sc, h, cge_rxdesc_paddr(sc, i));
#endif

	sc->sc_rxdesc_ring[i].rx_data = rdp->rx_dm[i]->dm_segs[0].ds_addr;
	sc->sc_rxdesc_ring[i].rx_status = RX_INT;
	sc->sc_rxdesc_ring[i].rx_extstatus = 0;
//	sc->sc_rxdesc_ring[i].rx_extstatus = GEMRX_OWN;

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
//	int orgreg;
	int mac;
	paddr_t paddr;

	cge_stop(ifp, 0);

	sc->sc_txnext = 0;
	sc->sc_txhead = 0;

	/* Init circular RX list. */
	for (i = 0; i < CGE_RX_RING_CNT; i++) {
		cge_new_rxbuf(sc, i);
	}
	sc->sc_rxhead = 0;

	/*
	 * Give the transmit and receive rings to the chip.
	 */
	paddr = sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr;
	cge_write_4(sc, GEM_IP + GEM_RX_QPTR, paddr);

	mac = sc->sc_enaddr[0] | (sc->sc_enaddr[1] << 8) |
	    (sc->sc_enaddr[2] << 16) | (sc->sc_enaddr[3] << 24);
	cge_write_4(sc, GEM_IP + GEM_LADDR1_BOT, mac);
	mac = sc->sc_enaddr[4] | (sc->sc_enaddr[5] << 8);
	cge_write_4(sc, GEM_IP + GEM_LADDR1_TOP, mac);

	cge_write_4(sc, GEM_SCH_BLOCK + SCH_CONTROL, 1);

#if 0
	/* clean up queue */
	while (cge_read_4(sc, GEM_ADM_BLOCK + ADM_QUEUEDEPTH))
		cge_write_4(sc, GEM_ADM_BLOCK + ADM_PKTDQ, 0);

	/* Start the controller */
	cge_write_4(sc, GEM_IP + GEM_RX_OFFSET, 0);

	cge_write_4(sc, GEM_ADM_BLOCK + ADM_QFULLTHR, CGE_RX_RING_CNT - 2);

	/* setup Rx coalescing */
#define DEFAULT_RX_COAL_TIME		500 // us
#define DEFAULT_RX_COAL_PKTS		3
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_BATCHINTRTIMERINIT,
	    DEFAULT_RX_COAL_TIME * 125);
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_BATCHINTRPKTTHRES,
	    DEFAULT_RX_COAL_PKTS);

	cge_write_4(sc, GEM_ADM_BLOCK + ADM_CNFG, 0x84210030);
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_DECAYTIMER, 0x000000aa);

	cge_write_4(sc, GEM_ADM_BLOCK + ADM_BATCHINTRPKTCNT, 0);
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_CONTROL, 0x7F);
#endif
	reg = cge_read_4(sc, GEM_ADM_BLOCK + ADM_CONTROL);
	cge_write_4(sc, GEM_ADM_BLOCK + ADM_CONTROL, reg & ~1);

#if 0
	delay(1000);

	reg = cge_read_4(sc, GEM_IP + GEM_NET_CONFIG);
	reg |= GEM_COPY_ALL;
	cge_write_4(sc, GEM_IP + GEM_NET_CONFIG, reg);

	reg = cge_read_4(sc, GEM_CFG);
	reg &= ~GEM_CONF_SPEED_MASK;
	reg |= GEM_CONF_SPEED_GEM_1G;
	reg &= ~GEM_CONF_MODE_GEM_MASK;
	reg |= GEM_CONF_MODE_GEM_RGMII;
	reg |= GEM_CONF_MODE_SEL_GEM;
	cge_write_4(sc, GEM_CFG, reg);

	/*
	 * Initialize DMA
	 */

	orgreg = reg = cge_read_4(sc, GEM_IP + GEM_DMA_CONFIG);
	reg |=(1UL<<31); //enable scheduler
	reg &= ~((1UL<<26) | (1UL<<25)); //hardware buffer allocation
	reg |=(1UL<<12); //enable scheduler
	reg &= ~(0x00FF001F); // enable admittance manager
	reg |= 0x00200000; // set buffer size to 2048 bytes
	reg |= 0x00000010; // Attempt to use INCR16 AHB bursts
	reg |=  GEM_RX_SW_ALLOC;
	cge_write_4(sc, GEM_IP + GEM_DMA_CONFIG, reg);
printf("GEM_DMA_CONFIG %x %x\n", orgreg, reg);
	/* Disabling GEM delay */
	cge_write_4(sc, 0xf00c, 0);
#endif
 
	/* Enable the receive circuitry */
	reg = cge_read_4(sc, GEM_IP + GEM_NET_CONTROL);
	cge_write_4(sc, GEM_IP + GEM_NET_CONTROL, reg | GEM_RX_EN);

	/*
	 * Initialize the interrupt mask and enable interrupts.
	 */
	cge_write_4(sc, GEM_IP + GEM_IRQ_ENABLE, GEM_IRQ_ALL);

	ifp->if_flags |= IFF_RUNNING;
#if 0
	struct cge_softc * const sc = ifp->if_softc;
	struct mii_data * const mii = &sc->sc_mii;
	int i;

	cge_stop(ifp, 0);

	sc->sc_txnext = 0;
	sc->sc_txhead = 0;

	/* Reset wrapper */
	cge_write_4(sc, CPSW_WR_SOFT_RESET, 1);
	while (cge_read_4(sc, CPSW_WR_SOFT_RESET) & 1)
		;

	/* Reset SS */
	cge_write_4(sc, CPSW_SS_SOFT_RESET, 1);
	while (cge_read_4(sc, CPSW_SS_SOFT_RESET) & 1)
		;

	/* Clear table and enable ALE */
	cge_write_4(sc, CPSW_ALE_CONTROL,
	    ALECTL_ENABLE_ALE | ALECTL_CLEAR_TABLE);

	/* Reset and init Sliver port 1 and 2 */
	for (i = 0; i < CPSW_ETH_PORTS; i++) {
		uint32_t macctl;

		/* Reset */
		cge_write_4(sc, CPSW_SL_SOFT_RESET(i), 1);
		while (cge_read_4(sc, CPSW_SL_SOFT_RESET(i)) & 1)
			;
		/* Set Slave Mapping */
		cge_write_4(sc, CPSW_SL_RX_PRI_MAP(i), 0x76543210);
		cge_write_4(sc, CPSW_PORT_P_TX_PRI_MAP(i+1), 0x33221100);
		cge_write_4(sc, CPSW_SL_RX_MAXLEN(i), 0x5f2);
		/* Set MAC Address */
		cge_write_4(sc, CPSW_PORT_P_SA_HI(i+1),
		    sc->sc_enaddr[0] | (sc->sc_enaddr[1] << 8) |
		    (sc->sc_enaddr[2] << 16) | (sc->sc_enaddr[3] << 24));
		cge_write_4(sc, CPSW_PORT_P_SA_LO(i+1),
		    sc->sc_enaddr[4] | (sc->sc_enaddr[5] << 8));

		/* Set MACCONTROL for ports 0,1 */
		macctl = SLMACCTL_FULLDUPLEX | SLMACCTL_GMII_EN |
		    SLMACCTL_IFCTL_A;
		if (sc->sc_phy_has_1000t)
			macctl |= SLMACCTL_GIG;
		cge_write_4(sc, CPSW_SL_MACCONTROL(i), macctl);

		/* Set ALE port to forwarding(3) */
		cge_write_4(sc, CPSW_ALE_PORTCTL(i+1), 3);
	}

	/* Set Host Port Mapping */
	cge_write_4(sc, CPSW_PORT_P0_CPDMA_TX_PRI_MAP, 0x76543210);
	cge_write_4(sc, CPSW_PORT_P0_CPDMA_RX_CH_MAP, 0);

	/* Set ALE port to forwarding(3) */
	cge_write_4(sc, CPSW_ALE_PORTCTL(0), 3);

	/* Initialize addrs */
	cge_ale_update_addresses(sc, 1);

	cge_write_4(sc, CPSW_SS_PTYPE, 0);
	cge_write_4(sc, CPSW_SS_STAT_PORT_EN, 7);

	cge_write_4(sc, CPSW_CPDMA_SOFT_RESET, 1);
	while (cge_read_4(sc, CPSW_CPDMA_SOFT_RESET) & 1)
		;

	for (i = 0; i < 8; i++) {
		cge_write_4(sc, CPSW_CPDMA_TX_HDP(i), 0);
		cge_write_4(sc, CPSW_CPDMA_RX_HDP(i), 0);
		cge_write_4(sc, CPSW_CPDMA_TX_CP(i), 0);
		cge_write_4(sc, CPSW_CPDMA_RX_CP(i), 0);
	}

	bus_space_set_region_4(sc->sc_bst, sc->sc_bsh_txdescs, 0, 0,
	    CPSW_CPPI_RAM_TXDESCS_SIZE/4);

	sc->sc_txhead = 0;
	sc->sc_txnext = 0;

	cge_write_4(sc, CPSW_CPDMA_RX_FREEBUFFER(0), 0);

	bus_space_set_region_4(sc->sc_bst, sc->sc_bsh_rxdescs, 0, 0,
	    CPSW_CPPI_RAM_RXDESCS_SIZE/4);
	/* Initialize RX Buffer Descriptors */
	cge_set_rxdesc_next(sc, RXDESC_PREV(0), 0);
	for (i = 0; i < CPSW_NRXDESCS; i++) {
		cge_new_rxbuf(sc, i);
	}
	sc->sc_rxhead = 0;

	/* turn off flow control */
	cge_write_4(sc, CPSW_SS_FLOW_CONTROL, 0);

	/* align layer 3 header to 32-bit */
	cge_write_4(sc, CPSW_CPDMA_RX_BUFFER_OFFSET, ETHER_ALIGN);

	/* Clear all interrupt Masks */
	cge_write_4(sc, CPSW_CPDMA_RX_INTMASK_CLEAR, 0xFFFFFFFF);
	cge_write_4(sc, CPSW_CPDMA_TX_INTMASK_CLEAR, 0xFFFFFFFF);

	/* Enable TX & RX DMA */
	cge_write_4(sc, CPSW_CPDMA_TX_CONTROL, 1);
	cge_write_4(sc, CPSW_CPDMA_RX_CONTROL, 1);

	/* Enable TX and RX interrupt receive for core 0 */
	cge_write_4(sc, CPSW_WR_C_TX_EN(0), 1);
	cge_write_4(sc, CPSW_WR_C_RX_EN(0), 1);
	cge_write_4(sc, CPSW_WR_C_MISC_EN(0), 0x1F);

	/* Enable host Error Interrupt */
	cge_write_4(sc, CPSW_CPDMA_DMA_INTMASK_SET, 2);

	/* Enable interrupts for TX and RX Channel 0 */
	cge_write_4(sc, CPSW_CPDMA_TX_INTMASK_SET, 1);
	cge_write_4(sc, CPSW_CPDMA_RX_INTMASK_SET, 1);

	/* Ack stalled irqs */
	cge_write_4(sc, CPSW_CPDMA_CPDMA_EOI_VECTOR, CPSW_INTROFF_RXTH);
	cge_write_4(sc, CPSW_CPDMA_CPDMA_EOI_VECTOR, CPSW_INTROFF_RX);
	cge_write_4(sc, CPSW_CPDMA_CPDMA_EOI_VECTOR, CPSW_INTROFF_TX);
	cge_write_4(sc, CPSW_CPDMA_CPDMA_EOI_VECTOR, CPSW_INTROFF_MISC);

	/* Initialize MDIO - ENABLE, PREAMBLE=0, FAULTENB, CLKDIV=0xFF */
	/* TODO Calculate MDCLK=CLK/(CLKDIV+1) */
	cge_write_4(sc, MDIOCONTROL,
	    MDIOCTL_ENABLE | MDIOCTL_FAULTENB | MDIOCTL_CLKDIV(0xff));

	mii_mediachg(mii);

	/* Write channel 0 RX HDP */
	cge_write_4(sc, CPSW_CPDMA_RX_HDP(0), cge_rxdesc_paddr(sc, 0));
	sc->sc_rxrun = true;
	sc->sc_rxeoq = false;

	sc->sc_txrun = true;
	sc->sc_txeoq = true;
	callout_schedule(&sc->sc_tick_ch, hz);
	ifp->if_flags |= IFF_RUNNING;
	sc->sc_txbusy = false;
#endif
	return 0;
}

static void
cge_stop(struct ifnet *ifp, int disable)
{
#if 0
	struct cge_softc * const sc = ifp->if_softc;
	struct cge_ring_data * const rdp = sc->sc_rdp;
	u_int i;

	aprint_debug_dev(sc->sc_dev, "%s: ifp %p disable %d\n", __func__,
	    ifp, disable);

	if ((ifp->if_flags & IFF_RUNNING) == 0)
		return;

	callout_stop(&sc->sc_tick_ch);
	mii_down(&sc->sc_mii);

	cge_write_4(sc, CPSW_CPDMA_TX_INTMASK_CLEAR, 1);
	cge_write_4(sc, CPSW_CPDMA_RX_INTMASK_CLEAR, 1);
	cge_write_4(sc, CPSW_WR_C_TX_EN(0), 0x0);
	cge_write_4(sc, CPSW_WR_C_RX_EN(0), 0x0);
	cge_write_4(sc, CPSW_WR_C_MISC_EN(0), 0x0);

	cge_write_4(sc, CPSW_CPDMA_TX_TEARDOWN, 0);
	cge_write_4(sc, CPSW_CPDMA_RX_TEARDOWN, 0);
	i = 0;
	while ((sc->sc_txrun || sc->sc_rxrun) && i < 10000) {
		delay(10);
		if ((sc->sc_txrun == true) && cge_txintr(sc) == 0)
			sc->sc_txrun = false;
		if ((sc->sc_rxrun == true) && cge_rxintr(sc) == 0)
			sc->sc_rxrun = false;
		i++;
	}
	//printf("%s toredown complete in %u\n", __func__, i);

	/* Reset wrapper */
	cge_write_4(sc, CPSW_WR_SOFT_RESET, 1);
	while (cge_read_4(sc, CPSW_WR_SOFT_RESET) & 1)
		;

	/* Reset SS */
	cge_write_4(sc, CPSW_SS_SOFT_RESET, 1);
	while (cge_read_4(sc, CPSW_SS_SOFT_RESET) & 1)
		;

	for (i = 0; i < CPSW_ETH_PORTS; i++) {
		cge_write_4(sc, CPSW_SL_SOFT_RESET(i), 1);
		while (cge_read_4(sc, CPSW_SL_SOFT_RESET(i)) & 1)
			;
	}

	/* Reset CPDMA */
	cge_write_4(sc, CPSW_CPDMA_SOFT_RESET, 1);
	while (cge_read_4(sc, CPSW_CPDMA_SOFT_RESET) & 1)
		;

	/* Release any queued transmit buffers. */
	for (i = 0; i < CPSW_NTXDESCS; i++) {
		bus_dmamap_unload(sc->sc_bdt, rdp->tx_dm[i]);
		m_freem(rdp->tx_mb[i]);
		rdp->tx_mb[i] = NULL;
	}

	ifp->if_flags &= ~IFF_RUNNING;
	ifp->if_timer = 0;
	sc->sc_txbusy = false;

	if (!disable)
		return;

	for (i = 0; i < CPSW_NRXDESCS; i++) {
		bus_dmamap_unload(sc->sc_bdt, rdp->rx_dm[i]);
		m_freem(rdp->rx_mb[i]);
		rdp->rx_mb[i] = NULL;
	}
#endif
}

static int
cge_intr(void *arg)
{
	struct cge_softc * const sc = arg;
	int reg;

	reg = cge_read_4(sc, GEM_IP + GEM_IRQ_STATUS);
	printf("CGE intr %x,", reg);

	if (reg & GEM_IRQ_RX_DONE)
		cge_rxintr(arg);

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

static int
cge_rxthintr(void *arg)
{
	struct cge_softc * const sc = arg;

	/* this won't deassert the interrupt though */
	cge_write_4(sc, CPSW_CPDMA_CPDMA_EOI_VECTOR, CPSW_INTROFF_RXTH);

	return 1;
}
#endif

static int
cge_rxintr(void *arg)
{
/*
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct cge_ring_data * const rdp = sc->sc_rdp;
	struct cge_cpdma_bd bd;
	const uint32_t * const dw = bd.word;
	bus_dmamap_t dm;
	struct mbuf *m;
	u_int len, off;
*/
	struct cge_softc * const sc = arg;
	struct cge_ring_data * const rdp = sc->sc_rdp;
	u_int i, count;
	int reg;
	int length;
	struct mbuf *m;
	struct ifnet *ifp = &sc->sc_ec.ec_if;
	count = 0;

	reg = cge_read_4(sc, GEM_IP + GEM_IRQ_STATUS);
	for (i = 0; i < CGE_RX_RING_CNT; i++) {
		if(sc->sc_rxdesc_ring[i].rx_extstatus & GEMRX_OWN) {
			length = sc->sc_rxdesc_ring[i].rx_status &
			    RX_STA_LEN_MASK;
			m = rdp->rx_mb[i];
			m_set_rcvif(m, ifp);
			m->m_pkthdr.len = m->m_len = length;
			if_percpuq_enqueue(ifp->if_percpuq, m);
			cge_new_rxbuf(sc, i);
			
/*
			sc->sc_rxdesc_ring[i].rx_extstatus &= ~GEMRX_OWN;
			bus_dmamap_sync(sc->sc_bdt, sc->sc_rxdesc_dmamap,
			    sizeof(struct tRXdesc) * i, sizeof(struct tRXdesc),
       			    BUS_DMASYNC_PREWRITE);
*/
			++count;
		}
	}
if (count != 0)
printf("MORI %x %d,", reg, count);

	return 1;
}

#if 0
static int
cge_txintr(void *arg)
{
	struct cge_softc * const sc = arg;
	struct ifnet * const ifp = &sc->sc_ec.ec_if;
	struct cge_ring_data * const rdp = sc->sc_rdp;
	struct cge_cpdma_bd bd;
	const uint32_t * const dw = bd.word;
	bool handled = false;
	uint32_t tx0_cp;
	u_int cpi;

	KERNHIST_FUNC(__func__);
	CPSWHIST_CALLARGS(sc, 0, 0, 0);

	KASSERT(sc->sc_txrun);

	KERNHIST_LOG(cgehist, "before txnext %x txhead %x txrun %x\n",
	    sc->sc_txnext, sc->sc_txhead, sc->sc_txrun, 0);

	tx0_cp = cge_read_4(sc, CPSW_CPDMA_TX_CP(0));

	if (tx0_cp == 0xfffffffc) {
		/* Teardown, ack it */
		cge_write_4(sc, CPSW_CPDMA_TX_CP(0), 0xfffffffc);
		cge_write_4(sc, CPSW_CPDMA_TX_HDP(0), 0);
		sc->sc_txrun = false;
		return 0;
	}

	for (;;) {
		tx0_cp = cge_read_4(sc, CPSW_CPDMA_TX_CP(0));
		cpi = (tx0_cp - sc->sc_txdescs_pa) / sizeof(struct cge_cpdma_bd);
		KASSERT(sc->sc_txhead < CPSW_NTXDESCS);

		KERNHIST_LOG(cgehist, "txnext %x txhead %x txrun %x cpi %x\n",
		    sc->sc_txnext, sc->sc_txhead, sc->sc_txrun, cpi);

		cge_get_txdesc(sc, sc->sc_txhead, &bd);

		if (dw[2] == 0) {
			//Debugger();
		}

		if (ISSET(dw[3], CPDMA_BD_SOP) == 0)
			goto next;

		if (ISSET(dw[3], CPDMA_BD_OWNER)) {
			printf("pwned %x %x %x\n", cpi, sc->sc_txhead,
			    sc->sc_txnext);
			break;
		}

		if (ISSET(dw[3], CPDMA_BD_TDOWNCMPLT)) {
			sc->sc_txrun = false;
			return 1;
		}

		bus_dmamap_sync(sc->sc_bdt, rdp->tx_dm[sc->sc_txhead],
		    0, rdp->tx_dm[sc->sc_txhead]->dm_mapsize,
		    BUS_DMASYNC_POSTWRITE);
		bus_dmamap_unload(sc->sc_bdt, rdp->tx_dm[sc->sc_txhead]);

		m_freem(rdp->tx_mb[sc->sc_txhead]);
		rdp->tx_mb[sc->sc_txhead] = NULL;

		if_statinc(ifp, if_opackets);

		handled = true;

		sc->sc_txbusy = false;

next:
		if (ISSET(dw[3], CPDMA_BD_EOP) && ISSET(dw[3], CPDMA_BD_EOQ)) {
			sc->sc_txeoq = true;
		}
		if (sc->sc_txhead == cpi) {
			cge_write_4(sc, CPSW_CPDMA_TX_CP(0),
			    cge_txdesc_paddr(sc, cpi));
			sc->sc_txhead = TXDESC_NEXT(sc->sc_txhead);
			break;
		}
		sc->sc_txhead = TXDESC_NEXT(sc->sc_txhead);
		if (ISSET(dw[3], CPDMA_BD_EOP) && ISSET(dw[3], CPDMA_BD_EOQ)) {
			sc->sc_txeoq = true;
			break;
		}
	}

	cge_write_4(sc, CPSW_CPDMA_CPDMA_EOI_VECTOR, CPSW_INTROFF_TX);

	if ((sc->sc_txnext != sc->sc_txhead) && sc->sc_txeoq) {
		if (cge_read_4(sc, CPSW_CPDMA_TX_HDP(0)) == 0) {
			sc->sc_txeoq = false;
			cge_write_4(sc, CPSW_CPDMA_TX_HDP(0),
			    cge_txdesc_paddr(sc, sc->sc_txhead));
		}
	}

	KERNHIST_LOG(cgehist, "after txnext %x txhead %x txrun %x\n",
	    sc->sc_txnext, sc->sc_txhead, sc->sc_txrun, 0);
	KERNHIST_LOG(cgehist, "CP %x HDP %x\n",
	    cge_read_4(sc, CPSW_CPDMA_TX_CP(0)),
	    cge_read_4(sc, CPSW_CPDMA_TX_HDP(0)), 0, 0);

	if (handled && sc->sc_txnext == sc->sc_txhead)
		ifp->if_timer = 0;

	if (handled)
		if_schedule_deferred_start(ifp);

	return handled;
}

static int
cge_miscintr(void *arg)
{
	struct cge_softc * const sc = arg;
	uint32_t miscstat;
	uint32_t dmastat;
	uint32_t stat;

	miscstat = cge_read_4(sc, CPSW_WR_C_MISC_STAT(0));
	device_printf(sc->sc_dev, "%s %x FIRE\n", __func__, miscstat);

#define CPSW_MISC_HOST_PEND __BIT32(2)
#define CPSW_MISC_STAT_PEND __BIT32(3)

	if (ISSET(miscstat, CPSW_MISC_HOST_PEND)) {
		/* Host Error */
		dmastat = cge_read_4(sc, CPSW_CPDMA_DMA_INTSTAT_MASKED);
		printf("CPSW_CPDMA_DMA_INTSTAT_MASKED %x\n", dmastat);

		printf("rxhead %02x\n", sc->sc_rxhead);

		stat = cge_read_4(sc, CPSW_CPDMA_DMASTATUS);
		printf("CPSW_CPDMA_DMASTATUS %x\n", stat);
		stat = cge_read_4(sc, CPSW_CPDMA_TX_HDP(0));
		printf("CPSW_CPDMA_TX0_HDP %x\n", stat);
		stat = cge_read_4(sc, CPSW_CPDMA_TX_CP(0));
		printf("CPSW_CPDMA_TX0_CP %x\n", stat);
		stat = cge_read_4(sc, CPSW_CPDMA_RX_HDP(0));
		printf("CPSW_CPDMA_RX0_HDP %x\n", stat);
		stat = cge_read_4(sc, CPSW_CPDMA_RX_CP(0));
		printf("CPSW_CPDMA_RX0_CP %x\n", stat);

		//Debugger();

		cge_write_4(sc, CPSW_CPDMA_DMA_INTMASK_CLEAR, dmastat);
		dmastat = cge_read_4(sc, CPSW_CPDMA_DMA_INTSTAT_MASKED);
		printf("CPSW_CPDMA_DMA_INTSTAT_MASKED %x\n", dmastat);
	}

	cge_write_4(sc, CPSW_CPDMA_CPDMA_EOI_VECTOR, CPSW_INTROFF_MISC);

	return 1;
}
#endif

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
