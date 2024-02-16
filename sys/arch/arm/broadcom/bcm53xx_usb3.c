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
#define USBH_PRIVATE

#include "locators.h"

#include <sys/cdefs.h>

__KERNEL_RCSID(1, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>

#include <arm/broadcom/bcm53xx_reg.h>
#include <arm/broadcom/bcm53xx_var.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>

#include <dev/usb/xhcireg.h>
#include <dev/usb/xhcivar.h>

struct bcmxusb_softc {
	device_t usbsc_dev;
	bus_dma_tag_t usbsc_dmat;
	bus_space_tag_t usbsc_bst;
	bus_space_handle_t usbsc_xhci_bsh;

	device_t usbsc_xhci_dev;
	void *usbsc_xhci_sc;
	void *usbsc_ih;
};

struct bcmxusb_attach_args {
	const char *usbaa_name;
	bus_dma_tag_t usbaa_dmat;
	bus_space_tag_t usbaa_bst;
	bus_space_handle_t usbaa_bsh;
	bus_size_t usbaa_size;
};

#ifdef XHCI_DEBUG
#define XHCI_DPRINTF(x)	if (xhcidebug) printf x
extern int xhcidebug;
#else
#define XHCI_DPRINTF(x)
#endif

static int xhci_bcmxusb_match(device_t, cfdata_t, void *);
static void xhci_bcmxusb_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(xhci_bcmxusb, sizeof(struct xhci_softc),
	xhci_bcmxusb_match, xhci_bcmxusb_attach, NULL, NULL);

static int
xhci_bcmxusb_match(device_t parent, cfdata_t cf, void *aux)
{
	struct bcmxusb_attach_args * const usbaa = aux;

	if (strcmp(cf->cf_name, usbaa->usbaa_name))
		return 0;

	return 1;
}

static void
xhci_bcmxusb_attach(device_t parent, device_t self, void *aux)
{
	struct xhci_softc * const sc = device_private(self);
	struct bcmxusb_attach_args * const usbaa = aux;

	sc->sc_dev = self;

	sc->sc_iot = usbaa->usbaa_bst;
	sc->sc_ioh = usbaa->usbaa_bsh;
	sc->sc_bus.ub_dmatag = usbaa->usbaa_dmat;
	sc->sc_bus.ub_hcpriv = sc;
	sc->sc_quirks = XHCI_DEFERRED_START;
	sc->sc_bus.ub_revision = USBREV_3_0;

	aprint_naive(": XHCI USB controller\n");
	aprint_normal(": XHCI USB controller\n");

	int error = xhci_init(sc);
	if (error) {
		aprint_error_dev(self, "init failed, error=%d\n", error);
		return;
	}
	/* Attach usb device. */
	sc->sc_child = config_found(self, &sc->sc_bus, usbctlprint, CFARGS_NONE);
	sc->sc_child2 = config_found(self, &sc->sc_bus2, usbctlprint, CFARGS_NONE);

	xhci_start(sc);
}

/*
 * There's only IRQ shared between both OCHI and XHCI devices.
 */
static int
bcmxusb_intr(void *arg)
{
	struct bcmxusb_softc * const usbsc = arg;
	int rv0;

	rv0 = xhci_intr(usbsc->usbsc_xhci_sc);

	return rv0;
}

static int bcmxusb_ccb_match(device_t, cfdata_t, void *);
static void bcmxusb_ccb_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(bcmxusb_ccb, sizeof(struct bcmxusb_softc),
	bcmxusb_ccb_match, bcmxusb_ccb_attach, NULL, NULL);

int
bcmxusb_ccb_match(device_t parent, cfdata_t cf, void *aux)
{
	struct bcmccb_attach_args * const ccbaa = aux;
	const struct bcm_locators * const loc = &ccbaa->ccbaa_loc;

	if (strcmp(cf->cf_name, loc->loc_name) != 0)
		return 0;

	KASSERT(cf->cf_loc[BCMCCBCF_PORT] == BCMCCBCF_PORT_DEFAULT);

	return 1;
}

void
bcmxusb_ccb_attach(device_t parent, device_t self, void *aux)
{
	struct bcmxusb_softc * const usbsc = device_private(self);
	const struct bcmccb_attach_args * const ccbaa = aux;
	const struct bcm_locators * const loc = &ccbaa->ccbaa_loc;

	usbsc->usbsc_bst = ccbaa->ccbaa_ccb_bst;
	usbsc->usbsc_dmat = ccbaa->ccbaa_dmat;

	bus_space_subregion(usbsc->usbsc_bst, ccbaa->ccbaa_ccb_bsh,
	    loc->loc_offset, 0x1000, &usbsc->usbsc_xhci_bsh);

	aprint_naive("\n");
	aprint_normal("\n");

	struct bcmxusb_attach_args usbaa_xhci = {
		.usbaa_name = "xhci",
		.usbaa_dmat = usbsc->usbsc_dmat,
		.usbaa_bst = usbsc->usbsc_bst,
		.usbaa_bsh = usbsc->usbsc_xhci_bsh,
		.usbaa_size = 0x100,
	};

	usbsc->usbsc_xhci_dev = config_found(self, &usbaa_xhci, NULL,
	    CFARGS_NONE);
	if (usbsc->usbsc_xhci_dev != NULL)
		usbsc->usbsc_xhci_sc = device_private(usbsc->usbsc_xhci_dev);

	usbsc->usbsc_ih = intr_establish(loc->loc_intrs[0], IPL_USB, IST_LEVEL,
	    bcmxusb_intr, usbsc);
	if (usbsc->usbsc_ih == NULL) {
		aprint_error_dev(self, "failed to establish interrupt %d\n",
		     loc->loc_intrs[0]);
		return;
	}
	aprint_normal_dev(self, "interrupting on irq %d\n", loc->loc_intrs[0]);
}
