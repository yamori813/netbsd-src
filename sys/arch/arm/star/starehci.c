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
#include <dev/usb/ehcireg.h>
#include <dev/usb/ehcivar.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

struct starehci_softc {
	ehci_softc_t sc_ehci;

	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
};

static int starehci_match(device_t, struct cfdata *, void *);
static void starehci_attach(device_t, device_t, void *);
static int starehci_init(struct starehci_softc *);

CFATTACH_DECL2_NEW(starehci, sizeof(struct starehci_softc),
    starehci_match, starehci_attach, NULL, NULL, NULL, ehci_childdet);

/* ARGSUSED */
static int
starehci_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa = aux;

	sa->sa_size = STAR_USB20_OPERATION_REGSIZE;
	return 1;
}

/* ARGSUSED */
static void
starehci_attach(device_t parent __unused, device_t self, void *aux)
{
	struct starehci_softc *sc;
	struct star_attach_args *sa;
	int error;
	usbd_status r;

	sa = aux;
	sc = device_private(self);
	sc->sc_iot = sa->sa_iot;

	sc->sc_ehci.sc_dev = self;
//	sc->sc_ehci.sc_bus.hci_private = sc;
	sc->sc_ehci.sc_bus.ub_hcpriv = sc;
	sc->sc_ehci.iot = sa->sa_iot;

	aprint_normal(": USB2.0 Host Controller\n");
	aprint_naive("\n");

	/* Map USB operation registers */
	if (bus_space_map(sc->sc_ehci.iot, sa->sa_addr, sa->sa_size, 0,
	    &sc->sc_ehci.ioh)) {
		aprint_error(": can't map operation registers\n");
		goto attach_failure;
	}
//	sc->sc_ehci.sc_bus.ub_dmat = sa->sa_dmat;
	sc->sc_ehci.sc_bus.ub_dmatag = sa->sa_dmat;

	error = starehci_init(sc);
	if (error)
		goto attach_failure_unmap;

	/* Disable interrupts, so we don't get any spurious ones. */
	sc->sc_ehci.sc_offs = EREAD1(&sc->sc_ehci, EHCI_CAPLENGTH);
	EOWRITE2(&sc->sc_ehci, EHCI_USBINTR, 0);

	intr_establish(sa->sa_irq, IPL_USB,
	    IST_LEVEL_LOW, ehci_intr, &sc->sc_ehci);

//	sc->sc_ehci.sc_bus.usbrev = USBREV_2_0;
	sc->sc_ehci.sc_bus.ub_revision = USBREV_2_0;
//	strlcpy(sc->sc_ehci.sc_vendor, "Star", sizeof(sc->sc_ehci.sc_vendor));
printf("@");
	r = ehci_init(&sc->sc_ehci);
printf("@");
	if (r != USBD_NORMAL_COMPLETION) {
		aprint_error("%s: init failed, error=%d\n",
		    device_xname(self), r);
		goto attach_failure_unmap;
	}

	/* Attach usb device. */
	sc->sc_ehci.sc_child = config_found(self, &sc->sc_ehci.sc_bus,
	    usbctlprint, CFARGS_NONE);
	return;

 attach_failure_unmap:
	bus_space_unmap(sc->sc_ehci.iot, sc->sc_ehci.ioh, sa->sa_size);
 attach_failure:
	return;
}

static int
starehci_init(struct starehci_softc *sc)
{
	uint32_t status;

	/* Map USB configuration registers */
	if (bus_space_map(sc->sc_iot,
	    STRx100_USB20_CONFIGURATION_REGISTER,
	    STAR_USB20_CONF_REGSIZE, 0, &sc->sc_ioh)) {
		aprint_error(": can't map configuration registers\n");
		return -1;
	}

	if (CPU_IS_STR8100()) {
		/* enable clock */
		status = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKGATE0_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKGATE0_REG,
		    status | EQUULEUS_CLKPWR_CLKGATE0_HCLK_USBH);

		/* activate */
		status = STAR_REG_READ32(EQUULEUS_CLKPWR_SOFTRST_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_SOFTRST_REG,
		    status | EQUULEUS_CLKPWR_SOFTRST_USBH);

		/* setup PLL */
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
		status = STAR_REG_READ32(ORION_CLKPWR_PLLCTRL_REG);
		// XXX: notyet
		STAR_REG_WRITE32(ORION_CLKPWR_PLLCTRL_REG, status);
	}
	delay(1000);

	/* XXX: linux-star write 0x0106 */
	bus_space_write_2(sc->sc_iot, sc->sc_ioh,
	    STAR_USB20_CONF_COMMAND, 0x0006);

	/* XXX: linux-star write 0x00020060 */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    STAR_USB20_CONF_MODE_ENABLE, (4 << 5));

	delay(100);

	return 0;
}
