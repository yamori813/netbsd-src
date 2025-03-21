/*	$NetBSD$	*/

/*-
 * Copyright (c) 2025 Hiroki Mori
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the distribution.
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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/systm.h>

#include <arm/telechips/tcc893x_reg.h>
#include <arm/telechips/tcc_var.h>

static int	tcc893x_pic_match(device_t, cfdata_t, void *);
static void	tcc893x_pic_attach(device_t, device_t, void *);

struct tcc893x_pic_softc {
	device_t		sc_dev;
	bus_space_tag_t		sc_bst;
	bus_space_handle_t	sc_bsh;

};

CFATTACH_DECL_NEW(tcc893x_pic, sizeof(struct tcc893x_pic_softc),
    tcc893x_pic_match, tcc893x_pic_attach, NULL, NULL);

#define PIC_READ(sc, reg)			\
    bus_space_read_4((sc)->sc_bst, (sc)->sc_bsh, (reg))
#define PIC_WRITE(sc, reg, val)		\
    bus_space_write_4((sc)->sc_bst, (sc)->sc_bsh, (reg), (val))

static int
tcc893x_pic_match(device_t parent, cfdata_t cf, void *aux)
{

	return 1;
}

static void
tcc893x_pic_attach(device_t parent, device_t self, void *aux)
{
	struct tcc893x_pic_softc * const sc = device_private(self);
	struct axi_attach_args *sa;

	sa = aux;
	sc->sc_dev = self;
	sc->sc_bst = sa->aa_iot;

	if (bus_space_map(sc->sc_bst, sa->aa_addr, 0x1000, 0,
	    &sc->sc_bsh)) {
		aprint_error(": can't map operation registers\n");
		return;
	}

	aprint_naive("\n");
	aprint_normal(": VPIC\n");

	printf("PIC EN0 %08x\n", PIC_READ(sc, offsetof(PIC, IEN0)));
	printf("PIC EN1 %08x\n", PIC_READ(sc, offsetof(PIC, IEN1)));
}
