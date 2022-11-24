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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_pci.h"
#include "pci.h"

#include <sys/param.h>
#include <sys/device.h>
#include <sys/errno.h>
#include <sys/extent.h>
#include <sys/mbuf.h>

#include <dev/pci/pcidevs.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <dev/pci/pciconf.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

struct starpci_softc {
	device_t sc_dev;
	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_dma_tag_t sc_dmat;
	bus_space_handle_t sc_ioh_addr;
	bus_space_handle_t sc_ioh_data;
	void *sc_ih_bridge;		/* PCI bridge */
	void *sc_ih_arbiter;	/* PCI bus arbiter */

	/* for PCI devices */
	struct arm32_pci_chipset sc_pc;
};

static int starpci_match(device_t, struct cfdata *, void *);
static void starpci_attach(device_t, device_t, void *);
static void starpci_reginit(struct starpci_softc *);
static void starpci_barinit(struct starpci_softc *);

static int starpci_bridge_intr(void *);
static int starpci_arbiter_intr(void *);

#if NPCI > 0
static void starpci_pciinit(struct starpci_softc *);
void starpci_attach_hook(device_t, device_t, struct pcibus_attach_args *);
int starpci_bus_maxdevs(void *, int);
pcitag_t starpci_make_tag(void *, int, int, int);
void starpci_decompose_tag(void *, pcitag_t, int *, int *, int *);
pcireg_t starpci_conf_read(void *, pcitag_t, int);
void starpci_conf_write(void *, pcitag_t, int, pcireg_t);
static int starpci_intr_map(struct pci_attach_args *, pci_intr_handle_t *);
static const char *starpci_intr_string(void *, pci_intr_handle_t);
static const struct evcnt *starpci_intr_evcnt(void *, pci_intr_handle_t);
static void *starpci_intr_establish(void *, pci_intr_handle_t, int,
                                    int (*)(void *), void *);
static void starpci_intr_disestablish(void *, void *);
static int starpci_conf_hook(pci_chipset_tag_t, int, int, int, pcireg_t);

#endif

CFATTACH_DECL_NEW(starpci, sizeof(struct starpci_softc),
    starpci_match, starpci_attach, NULL, NULL);


/* ARGSUSED */
static int
starpci_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa;

	sa = aux;
	sa->sa_size = 4;
	return 1;
}

/* ARGSUSED */
static void
starpci_attach(device_t parent __unused, device_t self, void *aux)
{
	struct starpci_softc *sc;
	struct star_attach_args *sa;

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_dmat = sa->sa_dmat;
	sc->sc_addr = sa->sa_addr;

	aprint_normal(": PCI Host Bridge\n");
	aprint_naive("\n");

	if (bus_space_map(sc->sc_iot, sc->sc_addr, sa->sa_size, 0,
	    &sc->sc_ioh_data)) {
		aprint_error(": can't map PCI Configuration Data Registers\n");
		return;
	}

	if (bus_space_map(sc->sc_iot, sc->sc_addr + 0x04000000, sa->sa_size, 0,
	    &sc->sc_ioh_addr)) {
		aprint_error(": can't map PCI Configuration Address Registers\n");
		return;
	}

	starpci_reginit(sc);
	starpci_barinit(sc);

	starpci_pciinit(sc);


	sc->sc_ih_bridge = star_intr_establish(STAR_IRQ_PCIBRIDGE, IPL_SERIAL,
	    STAR_INTR_HIGHLEVEL_TRIGGER, starpci_bridge_intr, sc);

	sc->sc_ih_arbiter = star_intr_establish(STAR_IRQ_PCIARBITER, IPL_SERIAL,
	    STAR_INTR_HIGHLEVEL_TRIGGER, starpci_arbiter_intr, sc);

}

static void
starpci_pciinit(struct starpci_softc *sc)
{
	struct pcibus_attach_args pba;
#if NPCI > 0 && defined(PCI_NETBSD_CONFIGURE)
	struct extent *ioext, *memext;
#endif

	sc->sc_pc.pc_conf_v = sc;
	sc->sc_pc.pc_attach_hook = starpci_attach_hook,
	sc->sc_pc.pc_bus_maxdevs = starpci_bus_maxdevs,
	sc->sc_pc.pc_make_tag = starpci_make_tag,
	sc->sc_pc.pc_decompose_tag = starpci_decompose_tag,
	sc->sc_pc.pc_conf_read = starpci_conf_read,
	sc->sc_pc.pc_conf_write = starpci_conf_write,
	sc->sc_pc.pc_intr_v = sc;
	sc->sc_pc.pc_intr_map = starpci_intr_map,
	sc->sc_pc.pc_intr_string = starpci_intr_string,
	sc->sc_pc.pc_intr_evcnt = starpci_intr_evcnt,
	sc->sc_pc.pc_intr_establish = starpci_intr_establish,
	sc->sc_pc.pc_intr_disestablish = starpci_intr_disestablish,
#ifdef __HAVE_PCI_CONF_HOOK
	sc->sc_pc.pc_conf_hook = starpci_conf_hook,
#endif
	sc->sc_pc.pc_cfg_cmd = 0;

#if NPCI > 0 && defined(PCI_NETBSD_CONFIGURE)
	ioext  = extent_create("pciio",
	    STRx100_PCI_IO_SPACE,
	    STRx100_PCI_IO_SPACE + STRx100_PCI_IO_SPACE_SIZE - 1,
	    M_DEVBUF, NULL, 0, EX_NOWAIT);

	memext = extent_create("pcimem",
	    STRx100_PCI_MEM_SPACE,
	    STRx100_PCI_MEM_SPACE + STRx100_PCI_MEM_SPACE_SIZE - 1,
	    M_DEVBUF, NULL, 0, EX_NOWAIT);

	pci_configure_bus(&sc->sc_pc, ioext, memext, NULL, 0, arm_dcache_align);

	extent_destroy(ioext);
	extent_destroy(memext);
#endif

	pba.pba_iot = &star_pci_io_tag;
	pba.pba_memt = &star_pci_mem_tag;
	pba.pba_dmat = sc->sc_dmat;
	pba.pba_dmat64 = NULL;
	pba.pba_pc = &sc->sc_pc;
	pba.pba_flags = PCI_FLAGS_IO_ENABLED | PCI_FLAGS_MEM_ENABLED;
	pba.pba_bus = 0;
	pba.pba_bridgetag = NULL;

	config_found_ia(sc->sc_dev, "pcibus", &pba, NULL);


#if 0
	{
		pcitag_t tag;
		tag = starpci_make_tag(sc, 0, 0, 0);
		pci_activate(&sc->sc_pc, tag, sc->sc_dev, NULL);
		starpci_conf_write(sc, tag, 0, PCI_COMMAND_MASTER_ENABLE);
	}
#endif
}

static void
starpci_reginit(struct starpci_softc *sc __unused)
{
	uint32_t val;

	if (CPU_IS_STR8100()) {
		/* XXX: notyet */
		panic("%s: not implemented\n", __func__);

	} else {
#if 1
		/* pad drive PCI mode */
		val = STAR_REG_READ32(ORION_CLKPWR_PADCTRL_REG) &
		    ~ORION_CLKPWR_PADCTRL_PCI_DRIVE_SEL_MASK;
		STAR_REG_WRITE32(ORION_CLKPWR_PADCTRL_REG, val |
		    ORION_CLKPWR_PADCTRL_PCI_DRIVE_SEL(1));
#endif

		/* Reset Latch */
		val = STAR_REG_READ32(ORION_CLKPWR_RSTLATCHCFG_REG);
		if (val & ORION_CLKPWR_RSTLATCHCFG_PCMCIA_DIS) {
			printf("PCMCIA Interface Disabled\n");
		}
		printf("REGULATION: %08x\n", STAR_REG_READ32(ORION_CLKPWR_REGULATORCTRL_REG));

#if 1
		/* 66MHz Capability */
		val = STAR_REG_READ32(ORION_MISC_PCI_PCI66_CAPABILITY);
		printf("PCI_CAPABILITY: %08x\n", val);
//		STAR_REG_WRITE32(ORION_MISC_PCI_PCI66_CAPABILITY, val | 1);
		STAR_REG_WRITE32(ORION_MISC_PCI_PCI66_CAPABILITY, 0x80 << 1 | 1);
#endif

#if 1
		/* set latency of AHB-to-PCI write access */
		val = STAR_REG_READ32(ORION_MISC_PCI_CONTROL);
		STAR_REG_WRITE32(ORION_MISC_PCI_CONTROL, 0x0000001f);
#endif

#if 1
		/* early termination control register */
		STAR_REG_WRITE32(ORION_MISC_PCI_TRANSFER_CONTROL, 0x0000000f);
#endif

#if 1
		/* reset PCI */
		val = STAR_REG_READ32(ORION_CLKPWR_SOFTRST_REG);
		STAR_REG_WRITE32(ORION_CLKPWR_SOFTRST_REG, val &
		    ~ORION_CLKPWR_SOFTRST_PCI);
		delay(1000);
#endif

#if 1
		/* disable PCMCIA clock */
		val = STAR_REG_READ32(ORION_CLKPWR_CLKGATE_REG);
		STAR_REG_WRITE32(ORION_CLKPWR_CLKGATE_REG, val |
		    ORION_CLKPWR_CLKGATE_HCLK_PCMCIA);
#endif

#if 1
		/* disable PCI clock */
		val = STAR_REG_READ32(ORION_CLKPWR_CLKGATE_REG);
		STAR_REG_WRITE32(ORION_CLKPWR_CLKGATE_REG, val |
		    ORION_CLKPWR_CLKGATE_HCLK_PCI);
#endif

#if 1
		/* activate PLL */
		val = STAR_REG_READ32(ORION_CLKPWR_PLLCTRL_REG);
printf("PLLCTRL=0x%08x\n", val);
		STAR_REG_WRITE32(ORION_CLKPWR_PLLCTRL_REG, val &
		    ~ORION_CLKPWR_PLLCTRL_PLL330_PWD &
		    ~ORION_CLKPWR_PLLCTRL_PLL300_PWD &
		    ~ORION_CLKPWR_PLLCTRL_SYSTEM_XTAL_PWD);
#endif

#if 1 /* XXX */

//	bootup setting:
//	  Hex: 0x00002930
//	  Bin: 0000_0000_0000_0000_0010_1001_0011_0000
//		vdd_18_sel = 101 = 1.799V
//		gm2_18 = 00 = 60uA
//		vdd25_sel = 100 = 2.598V
//		gm2_25 = 11 = 150uA
//		bg_sel = 0;
//		pd_18 = 0;
//		pd_25 = 0;

		/* regulator setup */
		val = STAR_REG_READ32(ORION_CLKPWR_REGULATORCTRL_REG);
		val &= ~ORION_CLKPWR_REGULATORCTRL_GM2_18_MASK;
		val &= ~ORION_CLKPWR_REGULATORCTRL_GM2_25_MASK;
		val &= ~ORION_CLKPWR_REGULATORCTRL_BG_SEL_MASK;

		STAR_REG_WRITE32(ORION_CLKPWR_REGULATORCTRL_REG, val |
		    ORION_CLKPWR_REGULATORCTRL_GM2_18(3) |
		    ORION_CLKPWR_REGULATORCTRL_GM2_25(3) |
		    ORION_CLKPWR_REGULATORCTRL_BG_SEL(3) |
		    0);
#endif


#if 1
		/* clear PCI clock source */
		val = STAR_REG_READ32(ORION_CLKPWR_CLKCTRL_REG) &
		    ~ORION_CLKPWR_CLKCTRL_PCI_DIV_MASK &
		    ~ORION_CLKPWR_CLKCTRL_PCLK_MASK;
#endif
#if 0	/* 66MHz/4 */
		STAR_REG_WRITE32(ORION_CLKPWR_CLKCTRL_REG, val |
		    ORION_CLKPWR_CLKCTRL_PCI_DIV(3) |	/* divided by 4 */
		    ORION_CLKPWR_CLKCTRL_PCLK_66MHZ);
#endif
#if 0	/* 66MHz/3 */
		STAR_REG_WRITE32(ORION_CLKPWR_CLKCTRL_REG, val |
		    ORION_CLKPWR_CLKCTRL_PCI_DIV(2) |	/* divided by 3 */
		    ORION_CLKPWR_CLKCTRL_PCLK_66MHZ);
#endif
#if 1	/* 66MHz/2 */
		STAR_REG_WRITE32(ORION_CLKPWR_CLKCTRL_REG, val |
		    ORION_CLKPWR_CLKCTRL_PCI_DIV(1) |	/* divided by 2 */
		    ORION_CLKPWR_CLKCTRL_PCLK_66MHZ);
#endif
#if 0	/* 66MHz */
		STAR_REG_WRITE32(ORION_CLKPWR_CLKCTRL_REG, val |
		    ORION_CLKPWR_CLKCTRL_PCI_DIV(0) |	/* divided by 1 */
		    ORION_CLKPWR_CLKCTRL_PCLK_66MHZ);
#endif
#if 0	/* APB */
		STAR_REG_WRITE32(ORION_CLKPWR_CLKCTRL_REG, val |
		    ORION_CLKPWR_CLKCTRL_PCI_DIV(0) |	/* divided by 1 */
		    ORION_CLKPWR_CLKCTRL_PCLK_APB);
#endif
#if 0	/* APB/2 */
		STAR_REG_WRITE32(ORION_CLKPWR_CLKCTRL_REG, val |
		    ORION_CLKPWR_CLKCTRL_PCI_DIV(1) |	/* divided by 2 */
		    ORION_CLKPWR_CLKCTRL_PCLK_APB);
#endif
#if 0	/* APB/3 */
		STAR_REG_WRITE32(ORION_CLKPWR_CLKCTRL_REG, val |
		    ORION_CLKPWR_CLKCTRL_PCI_DIV(2) |	/* divided by 3 */
		    ORION_CLKPWR_CLKCTRL_PCLK_APB);
#endif
#if 0	/* APB/4 */
		STAR_REG_WRITE32(ORION_CLKPWR_CLKCTRL_REG, val |
		    ORION_CLKPWR_CLKCTRL_PCI_DIV(3) |	/* divided by 4 */
		    ORION_CLKPWR_CLKCTRL_PCLK_APB);
#endif

#if 1
		/* enable PCI clock */
		val = STAR_REG_READ32(ORION_CLKPWR_CLKGATE_REG);
		STAR_REG_WRITE32(ORION_CLKPWR_CLKGATE_REG, val &
		    ~ORION_CLKPWR_CLKGATE_HCLK_PCI);
#endif

#if 0
		/* enable PCMCIA clock */
		val = STAR_REG_READ32(ORION_CLKPWR_CLKGATE_REG);
		STAR_REG_WRITE32(ORION_CLKPWR_CLKGATE_REG, val &
		    ORION_CLKPWR_CLKGATE_HCLK_PCMCIA);
#endif

#if 1
		/* activate PCI */
		val = STAR_REG_READ32(ORION_CLKPWR_SOFTRST_REG);
		STAR_REG_WRITE32(ORION_CLKPWR_SOFTRST_REG, val |
		    ORION_CLKPWR_SOFTRST_PCI);
#endif
	}
}

static void
starpci_barinit(struct starpci_softc *sc)
{
#if 0
	pcitag_t tag;

	tag = starpci_make_tag(sc, 0, 0, 0);

	starpci_conf_write(sc, tag, 0x10, 0);
	starpci_conf_write(sc, tag, 0x14, 0);
#endif
}

static int
starpci_bridge_intr(void *arg)
{
	struct starpci_softc *sc;
	pcitag_t tag;
	pcireg_t status;

	sc = arg;

	tag = starpci_make_tag(sc, 0, 0, 0);
	status = starpci_conf_read(sc, tag, PCI_COMMAND_STATUS_REG);
	starpci_conf_write(sc, tag, PCI_COMMAND_STATUS_REG, status);

	printf("%s: status=0x%08x, tag=%08x\n", __func__, (uint32_t)status, tag);

	return 1;
}

static int
starpci_arbiter_intr(void *arg)
{
	struct starpci_softc *sc;
	uint32_t status;

	sc = arg;

	status = STAR_REG_READ32(ORION_MISC_PCI_BROKEN_STATUS);
	STAR_REG_WRITE32(ORION_MISC_PCI_BROKEN_STATUS, status & 0x1f);

	printf("%s: status=0x%08x\n", __func__, status);

	return 1;
}

/*
 * pci chipset handlers
 */
void
starpci_attach_hook(device_t parent __unused, device_t self __unused,
    struct pcibus_attach_args *pba __unused)
{
	/* nothing to do */
}

int
starpci_bus_maxdevs(void *v __unused, int busno __unused)
{
#if 0
	if (CPU_IS_STR8100())
		return 2;
	else
		return 3;
#else
	return 32;
#endif
}

pcitag_t
starpci_make_tag(void *v __unused, int bus, int dev, int func)
{
	return (bus << 16) | (dev << 11) | (func << 8);
}

void
starpci_decompose_tag(void *v __unused, pcitag_t tag, int *bp, int *dp, int *fp)
{
	if (bp != NULL)
		*bp = (tag >> 16) & 0xff;
	if (dp != NULL)
		*dp = (tag >> 11) & 0x1f;
	if (fp != NULL)
		*fp = (tag >> 8) & 0x7;
}

pcireg_t
starpci_conf_read(void *v, pcitag_t tag, int reg)
{
	struct starpci_softc *sc;
	uint32_t data;

	sc = v;

	bus_space_write_4(sc->sc_iot, sc->sc_ioh_addr, 0,
	    0x80000000 | tag | reg);
	data = bus_space_read_4(sc->sc_iot, sc->sc_ioh_data, 0);

#if 1
	/* return dummy BAR for PCI Controller */
	if ((tag == 0) && (reg == 0x10)) {
		printf("faked BAR\n");
		data = 0;
	}
#endif

#if 0
	{
		int bus, dev, func;
		starpci_decompose_tag(sc, tag, &bus, &dev, &func);
		printf("%s:%d: tag=%08x(bus=%d, dev=%d, func=%d), reg=%08x, data=%08x\n", __func__, __LINE__, tag, bus, dev, func, reg, data);
	}
#endif


	return data;
}

void
starpci_conf_write(void *v, pcitag_t tag, int reg, pcireg_t val)
{
	struct starpci_softc *sc;
	sc = v;

	bus_space_write_4(sc->sc_iot, sc->sc_ioh_addr, 0,
	    0x80000000 | tag | reg);
#if 0
	{
		int bus, dev, func;
		starpci_decompose_tag(sc, tag, &bus, &dev, &func);
		printf("%s:%d: tag=%08x(bus=%d, dev=%d, func=%d), reg=%08x, data=%08x\n", __func__, __LINE__, tag, bus, dev, func, reg, val);
	}
#endif
	bus_space_write_4(sc->sc_iot, sc->sc_ioh_data, 0, val);
}

static int
starpci_intr_map(struct pci_attach_args *pa, pci_intr_handle_t *ihp)
{
	printf("%s:%d\n", __func__, __LINE__);

	printf("%s: pa_bus,dev,func,pin=%d,%d,%d,%d\n", __func__,
	    pa->pa_bus,
	    pa->pa_device,
	    pa->pa_function,
	    pa->pa_intrpin);

	*ihp = pa->pa_intrpin;
	return 0;
}

static const char *
starpci_intr_string(void *v, pci_intr_handle_t pin)
{
	struct starpci_softc *sc;
	static char buf[8];

	printf("%s:%d\n", __func__, __LINE__);

	sc = v;
	snprintf(buf, sizeof(buf), "pin %lu", pin);
	return buf;
}

static const struct evcnt *
starpci_intr_evcnt(void *v __unused, pci_intr_handle_t pin __unused)
{
	printf("%s:%d\n", __func__, __LINE__);

	return NULL;
}

static void *
starpci_intr_establish(void *v, pci_intr_handle_t pin __unused, int ipl,
    int (*intrhand)(void *), void *intrarg)
{
	struct starpci_softc *sc;

	printf("%s:%d: pin=%d, ipl=%d\n", __func__, __LINE__, pin, ipl);

	sc = v;

#if 0 /* XXX: irq mapping debug */
	star_intr_establish(STAR_IRQ_PCIEXT1, ipl,
	    STAR_INTR_LOWLEVEL_TRIGGER, intrhand, intrarg);
	star_intr_establish(STAR_IRQ_PCIEXT2, ipl,
	    STAR_INTR_LOWLEVEL_TRIGGER, intrhand, intrarg);
#endif

	return star_intr_establish(STAR_IRQ_PCIEXT0, ipl,
	    STAR_INTR_LOWLEVEL_TRIGGER, intrhand, intrarg);
}

static void
starpci_intr_disestablish(void *v __unused, void *ih)
{
	printf("%s:%d\n", __func__, __LINE__);

	star_intr_disestablish(ih);
}

static int
starpci_conf_hook(pci_chipset_tag_t pc __unused, int bus __unused,
    int device __unused, int function __unused, pcireg_t id __unused)
{
	printf("%s:%d\n", __func__, __LINE__);

	return PCI_CONF_ALL;
}

void
pci_conf_interrupt(pci_chipset_tag_t v __unused, int bus,
    int dev, int pin, int swiz, int *iline)
{
	printf("%s:%d: bus=%d, dev=%d, pin=%d, swiz=%d\n", __func__, __LINE__,
	    bus, dev, pin, swiz);

	/* XXX */
	*iline = 1;
}
