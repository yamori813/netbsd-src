/*	$NetBSD$	*/

/*-
 * Copyright (c) 2025 Hiroki Mori
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

#include <arm/telechips/tcc893x_reg.h>
#include <arm/telechips/tcc_var.h>

#ifdef EHCI_DEBUG
#define DPRINTF(x)      if (ehcidebug) printf x
extern int ehcidebug;
#else
#define DPRINTF(x)
#endif

#define USB20_OPERATION_REGSIZE                  0x108

static int tcc893x_ehci_match(device_t, struct cfdata *, void *);
static void tcc893x_ehci_attach(device_t, device_t, void *);

static void tcc893x_ehci_init(struct ehci_softc *hsc);

static void tcc893x_start_ehci(bus_space_tag_t iot);
static void tcc893x_start_ehci(bus_space_tag_t iot)
{
}

static void tcc893x_usb20hphy_cfg(bus_space_tag_t iot);
static void tcc893x_usb20hphy_cfg(bus_space_tag_t iot)
{
	bus_space_handle_t bsh;
	uint32_t reg;

	/* HSIO Control Register */

	bus_space_map(iot, HwHSIOBUSCFG_BASE, 0x100, 0, &bsh);

	reg = bus_space_read_4(iot, bsh, offsetof(HSIOBUSCFG, HSIO_CFG));
	reg &= ~(3 << 6);
	reg |= (0x3 << 6);
	bus_space_write_4(iot, bsh, offsetof(HSIOBUSCFG, HSIO_CFG), reg);

	reg = Hw29 | Hw28 | Hw25 | Hw24 | Hw21 | Hw20 | Hw18 | Hw11 | Hw9;
	reg |= Hw6 | Hw5 | Hw4 | Hw3 | Hw2;
	bus_space_write_4(iot, bsh, offsetof(HSIOBUSCFG, USB20H_PCFG0), reg);

	reg = Hw29 | Hw28 | Hw19 | Hw18 | Hw16 | Hw6 | Hw5 | Hw0;
	bus_space_write_4(iot, bsh, offsetof(HSIOBUSCFG, USB20H_PCFG1), reg);

	reg = Hw15 | Hw5;
	bus_space_write_4(iot, bsh, offsetof(HSIOBUSCFG, USB20H_PCFG2), reg);

	DELAY(10);
	reg = bus_space_read_4(iot, bsh, offsetof(HSIOBUSCFG, USB20H_PCFG1));
	reg |= Hw31;
	bus_space_write_4(iot, bsh, offsetof(HSIOBUSCFG, USB20H_PCFG1), reg);

	DELAY(20);
	bus_space_unmap(iot, bsh, 0x100);
}

/* ARGSUSED */
static int
tcc893x_ehci_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct axi_attach_args *aa = aux;

	aa->aa_size = USB20_OPERATION_REGSIZE;
	return 1;
}

/* ARGSUSED */
static void
tcc893x_ehci_attach(device_t parent __unused, device_t self, void *aux)
{
	struct ehci_softc *sc;
	struct axi_attach_args *aa;
	const char * const devname = device_xname(self);

	aa = aux;
	sc = device_private(self);
	sc->iot = aa->aa_iot;

	sc->sc_dev = self;
	sc->sc_bus.ub_hcpriv = sc;
	sc->iot = aa->aa_iot;
	sc->sc_vendor_init = tcc893x_ehci_init;
//	sc->sc_flags = EHCIF_ETTF;
	sc->sc_bus.ub_revision = USBREV_2_0;

	aprint_naive(": USB2.0 Interface\n");
	aprint_normal(": USB2.0 Interface\n");

	/* Map USB operation registers */
	if (bus_space_map(sc->iot, aa->aa_addr, aa->aa_size, 0,
	    &sc->ioh)) {
		aprint_error("%s: can't map operation registers\n", devname);
		goto attach_failure;
	}

	sc->sc_bus.ub_dmatag = aa->aa_dmat;

	/* Disable interrupts, so we don't get any spurious ones. */
	sc->sc_offs = EREAD1(sc, EHCI_CAPLENGTH);
	DPRINTF(("%s: offs=%d\n", devname, sc->sc_offs));
	EOWRITE2(sc, EHCI_USBINTR, 0);

	intr_establish(aa->aa_intr, IPL_USB,
	    IST_LEVEL_LOW, ehci_intr, sc);

	tcc893x_usb20hphy_cfg(sc->iot);

	tcc893x_start_ehci(sc->iot);

	int err = ehci_init(sc);
	if (err != USBD_NORMAL_COMPLETION) {
		aprint_error("%s: init failed, error=%d\n",
		    device_xname(self), err);
		goto attach_failure_unmap;
	}

	/* Attach usb device. */
	sc->sc_child = config_found(self, &sc->sc_bus, usbctlprint,
	    CFARGS_NONE);
	return;

 attach_failure_unmap:
	bus_space_unmap(sc->iot, sc->ioh, aa->aa_size);
 attach_failure:
	return;
}

#define USBMODE		0xa8
#define USBMODE_CM_HC	3
#define USBMODE_SDIS	0x10

static void tcc893x_ehci_init(struct ehci_softc *sc)
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

CFATTACH_DECL2_NEW(tcc893x_ehci, sizeof(struct ehci_softc),
    tcc893x_ehci_match, tcc893x_ehci_attach, NULL, NULL, NULL, ehci_childdet);
