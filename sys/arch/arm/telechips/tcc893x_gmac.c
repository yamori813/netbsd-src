/* $NetBSD$ */

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

/* Taken from sunxi_gmac.c */

#include <sys/cdefs.h>

__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>
#include <sys/gpio.h>
#include <sys/rndsource.h>

#include <net/if.h>
#include <net/if_ether.h>
#include <net/if_media.h>

#include <dev/mii/miivar.h>

#include <dev/ic/dwc_gmac_var.h>
#include <dev/ic/dwc_gmac_reg.h>

#include <arm/telechips/tcc893x_reg.h>
#include <arm/telechips/tcc_var.h>

static int
tcc893x_gmac_intr(void *arg)
{
	return dwc_gmac_intr(arg);
}

static int
tcc893x_gmac_match(device_t parent, cfdata_t cf, void *aux)
{

	return 1;
}

static void
tcc893x_gmac_attach(device_t parent, device_t self, void *aux)
{
	struct dwc_gmac_softc * const sc = device_private(self);
	struct axi_attach_args *axia = aux;
	bus_size_t size;
	int error;
	void *ih;

	size = 0x2000;

	sc->sc_dev = self;
	sc->sc_bst = axia->aa_iot;

	error = bus_space_map(axia->aa_iot, axia->aa_addr, size,
	    0, &sc->sc_bsh);

	sc->sc_dmat = axia->aa_dmat;

	if (error) {
		aprint_error(": failed to map register %#lx@%#lx: %d\n",
		    axia->aa_size, axia->aa_addr, error);
		return;
	}

	aprint_naive("\n");
	aprint_normal(": GMAC\n");

	ih = intr_establish(axia->aa_intr, IPL_NET, IST_LEVEL, tcc893x_gmac_intr, sc);
	if (ih == NULL) {
		aprint_error_dev(self, "couldn't establish interrupt on %d\n",
		    axia->aa_intr);
		return;
	}

	aprint_normal_dev(self, "interrupting on %d\n", axia->aa_intr);

	bus_space_handle_t bsh;
	bus_space_map(&armv7_generic_bs_tag, HwHSIOBUSCFG_BASE, 0x100, 0, &bsh);
        uint32_t reg = bus_space_read_4(&armv7_generic_bs_tag, bsh, offsetof(HSIOBUSCFG, ETHER_CFG1));
	reg &= ~(1 << 31);
        bus_space_write_4(&armv7_generic_bs_tag, bsh, offsetof(HSIOBUSCFG, ETHER_CFG1), reg);
        uint32_t div = bus_space_read_4(&armv7_generic_bs_tag, bsh, offsetof(HSIOBUSCFG, ETHER_CFG0));
	div &= ~(0x3f << 20);
	div |= (0x4 << 20);
        bus_space_write_4(&armv7_generic_bs_tag, bsh, offsetof(HSIOBUSCFG, ETHER_CFG0), div);
	reg |= (1 << 31);
        bus_space_write_4(&armv7_generic_bs_tag, bsh, offsetof(HSIOBUSCFG, ETHER_CFG1), reg);
        bus_space_unmap(&armv7_generic_bs_tag, bsh, 0x100);

	int miiclk;
	int miiclk_rate = 250000000;
	if (miiclk_rate > 250 * 1000 * 1000)
		miiclk = GMAC_MII_CLK_250_300M_DIV124;
	else if (miiclk_rate > 150 * 1000 * 1000)
		miiclk = GMAC_MII_CLK_150_250M_DIV102;
	else if (miiclk_rate > 100 * 1000 * 1000)
		miiclk = GMAC_MII_CLK_100_150M_DIV62;
	else if (miiclk_rate > 60 * 1000 * 1000)
		miiclk = GMAC_MII_CLK_60_100M_DIV42;
	else if (miiclk_rate > 35 * 1000 * 1000)
		miiclk = GMAC_MII_CLK_35_60M_DIV26;
	else
		miiclk = GMAC_MII_CLK_25_35M_DIV16;
//	dwc_gmac_attach(sc, MII_PHY_ANY, GMAC_MII_CLK_150_250M_DIV102);
	dwc_gmac_attach(sc, MII_PHY_ANY, miiclk);
}

CFATTACH_DECL_NEW(tcc893x_gmac, sizeof(struct dwc_gmac_softc),
	tcc893x_gmac_match, tcc893x_gmac_attach, NULL, NULL);
