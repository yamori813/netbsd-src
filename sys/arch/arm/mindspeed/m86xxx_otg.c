/* $NetBSD$ */

/*-
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/pool.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdivar.h>
#include <external/bsd/dwc2/dwc2.h>
#include <external/bsd/dwc2/dwc2var.h>
#include "dwc2_core.h"

#include <machine/bus_defs.h>

#include <arm/mindspeed/m86xxx_reg.h>
#include <arm/mindspeed/m86xxx_var.h>
#include <arm/mindspeed/m86xxx_clk.h>

#define	MUSB2_REG_AWIN_VEND0	0x43

static int	m86xxx_otg_match(device_t, cfdata_t, void *);
static void	m86xxx_otg_attach(device_t, device_t, void *);

struct m86xxx_otg_softc {
	struct dwc2_softc	sc_otg;
	struct dwc2_core_params	sc_params;
	struct bus_space	sc_bs;
};

CFATTACH_DECL_NEW(m86xxx_otg, sizeof(struct m86xxx_otg_softc),
	m86xxx_otg_match, m86xxx_otg_attach, NULL, NULL);

#define	REMAPFLAG	0x8000
#define	REGDECL(a, b)	[(a)] = ((b) | REMAPFLAG)

static void
dwc2_fdt_amlogic_params(struct dwc2_core_params *params)
{
	dwc2_set_all_params(params, -1);

	params->otg_cap = DWC2_CAP_PARAM_NO_HNP_SRP_CAPABLE;
/*
	params->speed = DWC2_SPEED_PARAM_HIGH;
	params->dma_enable = 1;
	params->enable_dynamic_fifo = 1;
	params->host_rx_fifo_size = 512;
	params->host_nperio_tx_fifo_size = 500;
	params->host_perio_tx_fifo_size = 500;
	params->host_channels = 16;
	params->phy_type = DWC2_PHY_TYPE_PARAM_UTMI;
	params->reload_ctl = 1;
	params->ahbcfg = GAHBCFG_HBSTLEN_INCR8 << GAHBCFG_HBSTLEN_SHIFT;
*/
#ifdef DWC2_POWER_DOWN_PARAM_NONE
	params->power_down = DWC2_POWER_DOWN_PARAM_NONE;
#endif
}

#if 0
static void
dwc2_fdt_rockchip_params(struct dwc2_fdt_softc *sc, struct dwc2_core_params *params)
{
	dwc2_set_all_params(params, -1);

	params->otg_cap = DWC2_CAP_PARAM_NO_HNP_SRP_CAPABLE;
	params->host_rx_fifo_size = 525;
	params->host_nperio_tx_fifo_size = 128;
	params->host_perio_tx_fifo_size = 256;
	params->ahbcfg = GAHBCFG_HBSTLEN_INCR16 << GAHBCFG_HBSTLEN_SHIFT;
}
#endif

static void
m86xxx_dwc2_deferred(device_t self)
{
	struct m86xxx_otg_softc *sc = device_private(self);
	int error;

	error = dwc2_init(&sc->sc_otg);
	if (error != 0) {
		aprint_error_dev(self, "couldn't initialize host, error=%d\n",
		    error);
		return;
	}
	sc->sc_otg.sc_child = config_found(sc->sc_otg.sc_dev,
	    &sc->sc_otg.sc_bus, usbctlprint, CFARGS_NONE);
}

static int
m86xxx_otg_match(device_t parent, cfdata_t cf, void *aux)
{
	return 1;
}

static void
m86xxx_otg_attach(device_t parent, device_t self, void *aux)
{
	struct m86xxx_otg_softc * const msc = device_private(self);
	struct dwc2_softc * const sc = &msc->sc_otg;
	struct axi_attach_args *axia = aux;
	bus_size_t size;
	int error;
	void *ih;

	size = 0x20000;

	error = bus_space_map(axia->aa_iot, axia->aa_addr, size,
	    0, &sc->sc_ioh);

	if (error) {
		aprint_error(": failed to map register %#lx@%#lx: %d\n",
		    axia->aa_size, axia->aa_addr, error);
		return;
	}
	sc->sc_dev = self;
	sc->sc_iot = axia->aa_iot;
	sc->sc_bus.ub_dmatag = axia->aa_dmat;

	dwc2_fdt_amlogic_params(&msc->sc_params);
	sc->sc_params = &msc->sc_params;
	sc->sc_params = NULL;

	aprint_naive("\n");
	aprint_normal(": DesignWare USB2 OTG\n");

	ih = intr_establish(axia->aa_intr, IPL_USB, IST_LEVEL, dwc2_intr, sc);
	if (ih == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %d\n",
		    axia->aa_intr);
		return;
	}

	config_defer(self, m86xxx_dwc2_deferred);
}
