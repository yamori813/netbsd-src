/*	$NetBSD$	*/

/*
 * Based on arch/arm/xscale/pxa2x0_com.c
 *
 * Copyright 2003 Wasabi Systems, Inc.
 * All rights reserved.
 *
 * Written by Steve C. Woodford for Wasabi Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed for the NetBSD Project by
 *      Wasabi Systems, Inc.
 * 4. The name of Wasabi Systems, Inc. may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY WASABI SYSTEMS, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL WASABI SYSTEMS, INC
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

#include "opt_com.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/termios.h>

#include <machine/intr.h>
#include <sys/bus.h>

#include <arm/pic/picvar.h>

#include <dev/ic/comreg.h>
#include <dev/ic/comvar.h>

//#include <arm/mindspeed/rt1310_reg.h>
#include <arm/ralink/rt1310_var.h>

#include "locators.h"

static int	rt1310_com_match(device_t, cfdata_t , void *);
static void	rt1310_com_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(rt1310_com, sizeof(struct com_softc),
    rt1310_com_match, rt1310_com_attach, NULL, NULL);

static int
rt1310_com_match(device_t parent, cfdata_t cf, void *aux)
{
	struct apb_attach_args *apba = aux;
	bus_space_handle_t bh;
	int rv;

	if (apba->apba_addr == -1 || apba->apba_intr == -1)
	    panic("rt1310_com must have addr and intr specified in config.");

	if (apba->apba_size == 0)
//		apba->apba_size = COMCERTO_UART_SIZE;
		apba->apba_size = 0x1000;

	if (com_is_console(apba->apba_memt, apba->apba_addr, NULL))
		return (1);

	if (bus_space_map(apba->apba_memt, apba->apba_addr, apba->apba_size,
			  0, &bh))
		return (0);

	rv = comprobe1(apba->apba_memt, bh);

	bus_space_unmap(apba->apba_memt, bh, apba->apba_size);

	return (rv);
}

static void
rt1310_com_attach(device_t parent, device_t self, void *aux)
{
	struct com_softc *sc = device_private(self);
	struct apb_attach_args *apba = aux;
	bus_space_tag_t memt;
	bus_space_handle_t ioh;
	bus_addr_t iobase;

	sc->sc_dev = self;
	memt = apba->apba_memt;
	iobase = apba->apba_addr;
//	sc->sc_frequency = COMCERTO_APB_FREQ;
	sc->sc_frequency = 6758400;
//	sc->sc_type = COM_TYPE_16550_NOERS;
//	sc->sc_type = COM_TYPE_TEGRA;
	sc->sc_type = COM_TYPE_NORMAL;

	if (com_is_console(memt, iobase, &ioh) == 0 &&
	    bus_space_map(memt, iobase, apba->apba_size, 0, &ioh)) {
		panic(": can't map registers\n");
		return;
	}
	com_init_regs_stride(&sc->sc_regs, memt, ioh, iobase, 2);

	com_attach_subr(sc);
	aprint_naive("\n");

	intr_establish(apba->apba_intr, IPL_SERIAL, IST_LEVEL_HIGH,
		comintr, sc);

}
