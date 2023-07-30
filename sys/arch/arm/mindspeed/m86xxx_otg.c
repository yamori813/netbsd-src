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
	struct bus_space	sc_bs;
};

CFATTACH_DECL_NEW(m86xxx_otg, sizeof(struct m86xxx_otg_softc),
	m86xxx_otg_match, m86xxx_otg_attach, NULL, NULL);

#define	REMAPFLAG	0x8000
#define	REGDECL(a, b)	[(a)] = ((b) | REMAPFLAG)

/* copy from bcmdwc2_params */
static struct dwc2_core_params m86xxx_params = {
	.otg_cap			= 0,	/* HNP/SRP capable */
	.otg_ver			= 0,	/* 1.3 */
	.dma_enable			= 1,
	.dma_desc_enable		= 0,
	.speed				= 0,	/* High Speed */
	.enable_dynamic_fifo		= 1,
	.en_multiple_tx_fifo		= 1,
	.host_rx_fifo_size		= 774,	/* 774 DWORDs */
	.host_nperio_tx_fifo_size	= 256,	/* 256 DWORDs */
	.host_perio_tx_fifo_size	= 512,	/* 512 DWORDs */
	.max_transfer_size		= 65535,
	.max_packet_count		= 511,
	.host_channels			= 8,
	.phy_type			= 1,	/* UTMI */
	.phy_utmi_width			= 8,	/* 8 bits */
	.phy_ulpi_ddr			= 0,	/* Single */
	.phy_ulpi_ext_vbus		= 0,
	.i2c_enable			= 0,
	.ulpi_fs_ls			= 0,
	.host_support_fs_ls_low_power	= 0,
	.host_ls_low_power_phy_clk	= 0,	/* 48 MHz */
	.ts_dline			= 0,
	.reload_ctl			= 0,
	.ahbcfg				= 0x10,
	.uframe_sched			= 1,
	.external_id_pin_ctl		= -1,
	.hibernation			= -1,
};

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

	sc->sc_params = &m86xxx_params;

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
