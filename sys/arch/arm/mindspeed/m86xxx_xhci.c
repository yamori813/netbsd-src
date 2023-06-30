/*	$NetBSD$	*/

/*
 * Copyright (c) 2001, 2002 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Lennart Augustsson (lennart@augustsson.net).
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

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "locators.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/proc.h>
#include <sys/queue.h>

#include <sys/bus.h>

#include <arm/mindspeed/m86xxx_reg.h>
#include <arm/mindspeed/m86xxx_var.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>

#include <dev/usb/xhcireg.h>
#include <dev/usb/xhcivar.h>

#ifdef EHCI_DEBUG
#define DPRINTF(x)	if (xhcidebug) printf x
extern int xhcidebug;
#else
#define DPRINTF(x)
#endif

static void m86xhci_init(struct xhci_softc *hsc);

static int
xhci_axi_match(device_t parent, cfdata_t match, void *aux)
{
	struct axi_attach_args * const aa = aux;

	if (aa->aa_addr == AXI_USB3p0_CFG_BASE)
		return 1;

	return 0;
}

static void
xhci_axi_attach(device_t parent, device_t self, void *aux)
{
	struct xhci_softc * const sc = device_private(self);
	struct axi_attach_args * const aa = aux;
	const char * const devname = device_xname(self);

	sc->sc_dev = self;
	sc->sc_iot = aa->aa_iot;
	sc->sc_bus.ub_hcpriv = sc;
	sc->sc_bus.ub_dmatag = aa->aa_dmat;
	sc->sc_quirks = XHCI_DEFERRED_START;
	sc->sc_bus.ub_revision = USBREV_3_0;

	aprint_naive(": USB Interfacer\n");
	aprint_normal(": USB Interface\n");

	if (aa->aa_size == 0)
		aa->aa_size = 0x1000000;

	/* Map I/O registers */
	if (bus_space_map(sc->sc_iot, aa->aa_addr, aa->aa_size,
	    0, &sc->sc_ioh)) {
		aprint_error("%s: can't map memory space\n", devname);
		return;
	}


	/* Disable interrupts, so we don't get any spurious ones. */
/*
	sc->sc_offs = EREAD1(sc, EHCI_CAPLENGTH);
	DPRINTF(("%s: offs=%d\n", devname, sc->sc_offs));
	EOWRITE4(sc, EHCI_USBINTR, 0);
*/

	intr_establish(aa->aa_intr, IPL_USB, IST_LEVEL,
	    xhci_intr, sc);

	m86xhci_init(sc);

	int err = xhci_init(sc);
	if (err) {
		aprint_error("%s: init failed, error=%d\n", devname, err);
		return;
	}

	/* Attach usb device. */
	sc->sc_child = config_found(self, &sc->sc_bus, usbctlprint,
	    CFARGS_NONE);

	xhci_start(sc);
}

#define USBMODE		0xa8
#define USBMODE_CM_HC	3
#define USBMODE_SDIS	0x10

static void
m86xhci_init(struct xhci_softc *sc)
{
#if 0
	uint32_t reg;

	reg = EOREAD4(sc, EHCI_PORTSC(1));
	reg &= ~(EHCI_PS_CSC | EHCI_PS_PEC | EHCI_PS_OCC);
	reg |= EHCI_PS_PP | EHCI_PS_PE;
	EOWRITE4(sc, EHCI_PORTSC(1), reg);

	reg = USBMODE_CM_HC;
	/* Set "Streaming disable mode"  to avoid Tx under run */
	reg |= USBMODE_SDIS;
	EWRITE4(sc, USBMODE, reg);
#endif
}

CFATTACH_DECL2_NEW(xhci_axi, sizeof(struct xhci_softc),
    xhci_axi_match, xhci_axi_attach, NULL, NULL, NULL, NULL);
