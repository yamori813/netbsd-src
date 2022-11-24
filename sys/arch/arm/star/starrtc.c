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
#include <sys/device.h>
#include <sys/errno.h>
#include <dev/clock_subr.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

struct starrtc_softc {
	device_t sc_dev;
	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	struct todr_chip_handle sc_todr;
};

static int starrtc_match(device_t, struct cfdata *, void *);
static void starrtc_attach(device_t, device_t, void *);
static void starrtc_init(struct starrtc_softc *);
static int starrtc_gettime(todr_chip_handle_t, struct timeval *);
static int starrtc_settime(todr_chip_handle_t, struct timeval *);

CFATTACH_DECL_NEW(starrtc, sizeof(struct starrtc_softc),
    starrtc_match, starrtc_attach, NULL, NULL);


/* ARGSUSED */
static int
starrtc_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa;

	sa = aux;
	sa->sa_size = STAR_RTC_REGSIZE;

	return 1;
}

/* ARGSUSED */
static void
starrtc_attach(device_t parent __unused, device_t self, void *aux)
{
	struct starrtc_softc *sc;
	struct star_attach_args *sa;

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_addr = sa->sa_addr;

	aprint_normal(": Real-time Clock\n");
	aprint_naive("\n");

	if (bus_space_map(sc->sc_iot, sc->sc_addr, sa->sa_size, 0,
	    &sc->sc_ioh)) {
		aprint_error(": can't map registers\n");
		return;
	}

	starrtc_init(sc);

	sc->sc_todr.cookie = sc;
	sc->sc_todr.todr_gettime = starrtc_gettime;
	sc->sc_todr.todr_settime = starrtc_settime;
	sc->sc_todr.todr_setwen = NULL;
	todr_attach(&sc->sc_todr);
}

static void
starrtc_init(struct starrtc_softc *sc)
{
	uint32_t val;

	if (CPU_IS_STR8100()) {
		/* RTC PCLK Enable */
		val = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKGATE1_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKGATE1_REG,
		    val | EQUULEUS_CLKPWR_CLKGATE1_PCLK_RTC);

		/* select 25MHz for source clock of RTC */
		val = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKCTRL_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKCTRL_REG, val |
		    EQUULEUS_CLKPWR_CLKCTRL_RTC_SEL);
	} else {
		/* no need to activate PCLK on STR9100 */
	}

	/* RTC enable */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_RTC_CTRL, 1);
}

#define READ_RTC_UNIXTIME(sc)	\
	(bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, STAR_RTC_DAYS) * 86400 + \
	 bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, STAR_RTC_HOUR) * 3600 + \
	 bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, STAR_RTC_MIN) * 60 + \
	 bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, STAR_RTC_SEC))

static int
starrtc_gettime(todr_chip_handle_t tch, struct timeval *tv)
{
	struct starrtc_softc *sc;
	time_t x, y;

	sc = tch->cookie;

	/* read twice until same */
	for (x = READ_RTC_UNIXTIME(sc); ; x = y) {
		y = READ_RTC_UNIXTIME(sc);
		if (x == y)
			break;
	}
	/* adjust by record */
	x += bus_space_read_4(sc->sc_iot, sc->sc_ioh, STAR_RTC_RECORD);

	tv->tv_sec = x;
	tv->tv_usec = 0;

	return 0;
}

static int
starrtc_settime(todr_chip_handle_t tch, struct timeval *tv)
{
	struct starrtc_softc *sc;
	time_t x, y;

	sc = tch->cookie;

	/* read twice until same */
	for (x = READ_RTC_UNIXTIME(sc); ; x = y) {
		y = READ_RTC_UNIXTIME(sc);
		if (x == y)
			break;
	}

	/*
	 * calculate difference of RTC, and
	 * record it.
	 */
	y = tv->tv_sec - x;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_RTC_RECORD, y);

	return 0;
}
