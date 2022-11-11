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

#include <arm/mindspeed/m83xxx_reg.h>
#include <arm/mindspeed/m83xxx_var.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usb_mem.h>

#include <dev/usb/ehcireg.h>
#include <dev/usb/ehcivar.h>

#ifdef EHCI_DEBUG
#define DPRINTF(x)	if (ehcidebug) printf x
extern int ehcidebug;
#else
#define DPRINTF(x)
#endif

#define	EHCI_HCOTGDEV_INTR_STATUS		0xc0
#define	EHCI_HCOTGDEV_INTR_MASK			0xc4
#define	HC_INT					0x04
#define	OTG_INT					0x02
#define	DEV_INT					0x01
/*
static int
ehci_ahb_intr(void *arg)
{
	struct ehci_softc * const sc = arg;
	int rv = 0;
	uint32_t v;

	v = bus_space_read_4(sc->iot, sc->ioh, EHCI_HCOTGDEV_INTR_STATUS);
	bus_space_write_4(sc->iot, sc->ioh, EHCI_HCOTGDEV_INTR_STATUS, v);
	if (v & HC_INT)
		rv = ehci_intr(sc);
	return rv;
}
*/

void usb_reset(bus_space_tag_t iot);
void usb_reset(bus_space_tag_t iot)
{

	bus_space_handle_t bsh;
	uint32_t reg;

	/* IO Control Register */

	bus_space_map(iot, APB_CLK_BASE, 0x20000, 0, &bsh);

#define USB0_AHBCLK_PD	0x00000010
#define USB_REFCLK_PD		(1 << 24)
#define USB_AHBCLK_PD		(1 << 19)
	/* Power up clocks */
	reg = bus_space_read_4(iot, bsh,
	    CLK_CLK_PWR_DWN);
	bus_space_write_4(iot, bsh,
	    CLK_CLK_PWR_DWN, reg & ~(USB_REFCLK_PD | USB_AHBCLK_PD));

#define USB_MUX_SEL	(1 << 3)
	/* Switch to refclk */
	reg = bus_space_read_4(iot, bsh,
	    CLK_DDR_PCIE_CLK_CNTRL);
	bus_space_write_4(iot, bsh,
	    CLK_DDR_PCIE_CLK_CNTRL, reg & ~USB_MUX_SEL);

#define USB_DIV_VAL_OFFSET	20
#define USB_DIV_VAL_MASK	(0x3f << USB_DIV_VAL_OFFSET)
#define USB_DIV_BYPASS		(1 << 30)
	/* Use refclk / 2 */
	reg = bus_space_read_4(iot, bsh,
	    CLK_DDR_PCIE_CLK_CNTRL);
	reg = (reg & ~USB_DIV_VAL_MASK) | (2 << USB_DIV_VAL_OFFSET);
	bus_space_write_4(iot, bsh,
	    CLK_DDR_PCIE_CLK_CNTRL, reg);
	bus_space_write_4(iot, bsh,
	    CLK_DDR_PCIE_CLK_CNTRL, reg & ~USB_DIV_BYPASS);

#define USB_AHB_RESET_N		(1 << 14)
#define USB_REF_RESET_N		(1 << 20)
	/* De-activate USB and USB phy reset */
	reg = bus_space_read_4(iot, bsh,
	    BLOCK_RESET_REG);
	reg = reg | USB_AHB_RESET_N | USB_REF_RESET_N;
	bus_space_write_4(iot, bsh,
	    BLOCK_RESET_REG, reg);

	bus_space_unmap(iot, bsh, 0x20000);

	bus_space_map(iot, APB_GPIO_BASE, 0x20000, 0, &bsh);

#define USB_FORCE_SUSPEND	(1 << 1)
	reg = bus_space_read_4(iot, bsh,
	    GPIO_GENERAL_CONTROL_REG);
	bus_space_write_4(iot, bsh,
	    GPIO_GENERAL_CONTROL_REG, reg & ~USB_FORCE_SUSPEND);
	bus_space_write_4(iot, bsh,
	    GPIO_USB_PHY_CONF_REG, 0x002D64C2);

	bus_space_unmap(iot, bsh, 0x20000);
}

static int
ehci_ahb_match(device_t parent, cfdata_t match, void *aux)
{
	struct ahb_attach_args * const ahb = aux;

	if (ahb->ahba_addr == AHB_USB0_BASE)
		return 1;

	return 0;
}

static void
ehci_ahb_attach(device_t parent, device_t self, void *aux)
{
	struct ehci_softc * const sc = device_private(self);
	struct ahb_attach_args * const ahb = aux;
	const char * const devname = device_xname(self);

	sc->sc_dev = self;
	sc->sc_bus.ub_hcpriv = sc;
	sc->iot = ahb->ahba_memt;

	aprint_naive(": USB Interfacer\n");
	aprint_normal(": USB Interface\n");

/*
	bus_space_handle_t bsh;
	bus_space_map(sc->iot, APB_GPIO_BASE, 0x20000, 0, &bsh);
	bus_space_write_4(sc->iot, bsh,
	    GPIO_USB_PHY_CONF_REG, 0);
	bus_space_unmap(sc->iot, bsh, 0x20000);
*/

	/* Map I/O registers */
	if (bus_space_map(sc->iot, ahb->ahba_addr+0x100, ahb->ahba_size-0x100, 0,
			   &sc->ioh)) {
		aprint_error("%s: can't map memory space\n", devname);
		return;
	}

	sc->sc_bus.ub_dmatag = ahb->ahba_dmat;

	/* Disable interrupts, so we don't get any spurious ones. */
	sc->sc_offs = EREAD1(sc, EHCI_CAPLENGTH);
	DPRINTF(("%s: offs=%d\n", devname, sc->sc_offs));
	EOWRITE4(sc, EHCI_USBINTR, 0);
	bus_space_write_4(sc->iot, sc->ioh, EHCI_HCOTGDEV_INTR_MASK,
	    OTG_INT|DEV_INT);

//	if (ahb->ahb_intr != OBIOCF_INTR_DEFAULT) {
		intr_establish(ahb->ahba_intr, IPL_USB, IST_LEVEL,
		     //ehci_ahb_intr, sc);
		    ehci_intr, sc);
//	}

	usb_reset(sc->iot);

	sc->sc_bus.ub_revision = USBREV_2_0;

	int err = ehci_init(sc);
	if (err) {
		aprint_error("%s: init failed, error=%d\n", devname, err);
		return;
	}

	/* Attach usb device. */
	sc->sc_child = config_found(self, &sc->sc_bus, usbctlprint, CFARGS_NONE);
}

CFATTACH_DECL2_NEW(ehci_ahb, sizeof(struct ehci_softc),
    ehci_ahb_match, ehci_ahb_attach, NULL, NULL, NULL, ehci_childdet);
