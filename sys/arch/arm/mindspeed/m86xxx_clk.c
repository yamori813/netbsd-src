/*	$NetBSD$	*/

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

//#include "opt_clk.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/termios.h>

#include <machine/intr.h>
#include <sys/bus.h>

#include <arm/pic/picvar.h>

#include <arm/mindspeed/m86xxx_reg.h>
#include <arm/mindspeed/m86xxx_var.h>

struct clk_softc {
	device_t sc_dev;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_hdl;
};

static int	m86xxx_clk_match(device_t, cfdata_t , void *);
static void	m86xxx_clk_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(clk, sizeof(struct clk_softc),
    m86xxx_clk_match, m86xxx_clk_attach, NULL, NULL);

#define	CLK_READ(sc, reg)						\
	bus_space_read_4(sc->sc_iot, sc->sc_hdl, (reg))
#define	CLK_WRITE(sc, reg, val)						\
	bus_space_write_4(sc->sc_iot, sc->sc_hdl, (reg), (val))
#define	CLK_WRITE_2(sc, reg, val)					\
	bus_space_write_2(sc->sc_iot, sc->sc_hdl, (reg), (val))

static int
m86xxx_clk_match(device_t parent, cfdata_t cf, void *aux)
{
	struct axi_attach_args *axia = aux;

	if (axia->aa_addr == -1)
	    panic("m86xxx_clk must have addr in config.");

	if (axia->aa_size == 0)
		axia->aa_size = 0x00100000;

	return (1);
}

static void
m86xxx_clk_attach(device_t parent, device_t self, void *aux)
{
	struct clk_softc *sc = device_private(self);
	struct axi_attach_args *axia = aux;

	sc->sc_dev = self;
	sc->sc_iot = axia->aa_iot;

/*
	if (bus_space_map(axia->aa_iot, axia->aa_addr, axia->aa_size,
			  0, &sc->sc_hdl))
*/

	aprint_normal("\n");
}
