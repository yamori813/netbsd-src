/*	$NetBSD$	*/

/*-
 * Copyright (c) 2009 SHIMIZU Ryo <ryo@nerv.org>
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
#include <sys/termios.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

#include <dev/ic/comvar.h>


static int star_com_match(device_t, struct cfdata *, void *);
static void star_com_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(staruart, sizeof(struct com_softc),
    star_com_match, star_com_attach, NULL, NULL);


/* ARGSUSED */
static int
star_com_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa;
	bus_space_handle_t ioh;
	int rv;

	sa = aux;
	if (sa->sa_size == 0)
		sa->sa_size = STAR_UART_SIZE;

	switch (sa->sa_addr) {
	case STRx100_UART0_REGISTER:
		break;
	case STR8100_UART1_REGISTER:
		if (CPU_IS_STR9100())
			return 0;
		break;
	default:
		/* unknown UART address */
		return 0;
	}

	if (com_is_console(sa->sa_iot, sa->sa_addr, NULL))
		return 1;

	if (bus_space_map(sa->sa_iot, sa->sa_addr, sa->sa_size, 0, &ioh))
		return 0;

	rv = comprobe1(sa->sa_iot, ioh);
	bus_space_unmap(sa->sa_iot, ioh, sa->sa_size);
	return rv;
}

/* ARGSUSED */
static void
star_com_attach(device_t parent __unused, device_t self, void *aux)
{
	struct com_softc *sc;
	struct star_attach_args *sa;
	bus_space_handle_t ioh;

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;

	sc->sc_frequency = STAR_UART_FREQ;
	sc->sc_type = COM_TYPE_STRx100;
	if (com_is_console(sa->sa_iot, sa->sa_addr, &ioh) == 0 &&
	    bus_space_map(sa->sa_iot, sa->sa_addr, sa->sa_size, 0, &ioh)) {
		aprint_error(": can't map registers\n");
		return;
	}
	com_init_regs(&sc->sc_regs, sa->sa_iot, ioh, sa->sa_addr);

	com_attach_subr(sc);
	aprint_naive("\n");

	intr_establish(sa->sa_irq, IPL_SERIAL,
	    IST_LEVEL_HIGH, comintr, sc);
}
