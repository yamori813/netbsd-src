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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/errno.h>

#include <dev/pci/pcidevs.h>
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>
#include <dev/usb/ohcireg.h>
#include <dev/usb/ohcivar.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

struct starohci_softc {
	ohci_softc_t sc_ohci;

	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
};

static int starohci_match(device_t, struct cfdata *, void *);
static void starohci_attach(device_t, device_t, void *);
static int starohci_init(struct starohci_softc *);

CFATTACH_DECL2_NEW(starohci, sizeof(struct starohci_softc),
    starohci_match, starohci_attach, NULL, NULL, NULL, ohci_childdet);

/* ARGSUSED */
static int
starohci_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa = aux;

	sa->sa_size = STAR_USB11_OPERATION_REGSIZE;
	return 1;
}

/* ARGSUSED */
static void
starohci_attach(device_t parent __unused, device_t self, void *aux)
{
	struct starohci_softc *sc;
	struct star_attach_args *sa;
	int error;
	usbd_status r;

	sa = aux;
	sc = device_private(self);
	sc->sc_iot = sa->sa_iot;

	sc->sc_ohci.sc_dev = self;
//	sc->sc_ohci.sc_bus.hci_private = sc;
	sc->sc_ohci.sc_bus.ub_hcpriv = sc;
	sc->sc_ohci.iot = sa->sa_iot;

	aprint_normal(": USB1.1 Host Controller\n");
	aprint_naive("\n");

	/* Map USB operation registers */
	if (bus_space_map(sc->sc_ohci.iot, sa->sa_addr, sa->sa_size, 0,
	    &sc->sc_ohci.ioh)) {
		aprint_error(": can't map operation registers\n");
		goto attach_failure;
	}
//	sc->sc_ohci.sc_bus.dmatag = sa->sa_dmat;
	sc->sc_ohci.sc_bus.ub_dmatag = sa->sa_dmat;

	error = starohci_init(sc);
	if (error)
		goto attach_failure_unmap;

	intr_establish(sa->sa_irq, IPL_USB,
	    IST_LEVEL_LOW, ohci_intr, &sc->sc_ohci);

//	strlcpy(sc->sc_ohci.sc_vendor, "Star", sizeof(sc->sc_ohci.sc_vendor));
	r = ohci_init(&sc->sc_ohci);
	if (r != USBD_NORMAL_COMPLETION) {
		aprint_error("%s: init failed, error=%d\n",
		    device_xname(self), r);
		goto attach_failure_unmap;
	}

	/* Attach usb device. */
	sc->sc_ohci.sc_child = config_found(self, &sc->sc_ohci.sc_bus,
	    usbctlprint, CFARGS_NONE);
	return;

 attach_failure_unmap:
	bus_space_unmap(sc->sc_ohci.iot, sc->sc_ohci.ioh, sa->sa_size);
 attach_failure:
	return;
}

static int
starohci_init(struct starohci_softc *sc)
{
	uint32_t status;

	/* Map USB configuration registers */
	if (bus_space_map(sc->sc_iot,
	    STRx100_USB11_CONFIGURATION_REGISTER,
	    STAR_USB11_CONF_REGSIZE, 0, &sc->sc_ioh)) {
		aprint_error(": can't map configuration registers\n");
		return -1;
	}

	if (CPU_IS_STR8100()) {
		/* CLKGATE */
		status = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKGATE0_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKGATE0_REG,
		    status |
		    EQUULEUS_CLKPWR_CLKGATE0_HCLK_USBH);

		/* activate */
		status = STAR_REG_READ32(EQUULEUS_CLKPWR_SOFTRST_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_SOFTRST_REG, status |
		    EQUULEUS_CLKPWR_SOFTRST_USBH);

		/* PLL setup */
		status = STAR_REG_READ32(EQUULEUS_CLKPWR_PLLCTRL_REG);
		status &= ~EQUULEUS_CLKPWR_PLLCTRL_USBH_PHY_PWD;
		status &= ~EQUULEUS_CLKPWR_PLLCTRL_PLLx3_PWD;
		status &= ~EQUULEUS_CLKPWR_PLLCTRL_PLLx8_PWD;
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_PLLCTRL_REG, status);
	} else {
		/* enable clock */
		status = STAR_REG_READ32(ORION_CLKPWR_CLKGATE_REG);
		STAR_REG_WRITE32(ORION_CLKPWR_CLKGATE_REG,
		    status & ~ORION_CLKPWR_CLKGATE_HCLK_USB);

		/* activate */
		status = STAR_REG_READ32(ORION_CLKPWR_SOFTRST_REG);
		STAR_REG_WRITE32(ORION_CLKPWR_SOFTRST_REG,
		    status | ORION_CLKPWR_SOFTRST_USB);

		/* setup PLL */
//		status = STAR_REG_READ32(ORION_CLKPWR_PLLCTRL_REG);
//		// XXX: notyet
//		STAR_REG_WRITE32(ORION_CLKPWR_PLLCTRL_REG, status);
	}
	delay(1000);

	/* linux-star write 0x0146? */
	bus_space_write_2(sc->sc_iot, sc->sc_ioh,
	    STAR_USB11_CONF_COMMAND, 0x0006);

	/* linux-star write 0x0200? */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    STAR_USB11_CONF_MODE_ENABLE, 0x00000260);

	delay(100);

	return 0;
}
