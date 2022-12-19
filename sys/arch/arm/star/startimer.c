/*	$NetBSD$ */

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

#include <sys/types.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/atomic.h>
#include <sys/time.h>
#include <sys/timetc.h>
#include <sys/device.h>
#include <sys/bus.h>

#include <dev/clock_subr.h>

#include <machine/intr.h>

#include <arm/cpufunc.h>

#include <arm/pic/picvar.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

/* specified stathz. if set 0, statclock shared with hardclock */
#ifndef STATHZ
#define STATHZ	64
#endif

struct starclk_softc {
	device_t sc_dev;
	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	void *sc_ih1;	/* TIMER1 for hardclock */
	void *sc_ih2;	/* TIMER2 for statclock */
	uint32_t sc_timer1_clk_per_hz;
	uint32_t sc_timer2_clk_per_hz;
};

static struct starclk_softc *clock_sc;
static volatile uint32_t starclk_base;

static int starclk_match(device_t, struct cfdata *, void *);
static void starclk_attach(device_t, device_t, void *);
static u_int starclk_get_timecount(struct timecounter *);
static void star_timer1_set(struct starclk_softc *sc, uint32_t tick);
static void star_timer2_set(struct starclk_softc *sc, uint32_t tick);
static int star_timer1_intr(void *);	/* interrupt handler */
static int star_timer2_intr(void *);	/* interrupt handler */

static struct timecounter starclk_timecounter = {
	.tc_get_timecount = starclk_get_timecount,
	.tc_poll_pps = NULL,
	.tc_counter_mask = ~0U,
	.tc_frequency = STAR_PCLK,
	.tc_name = "starclk",
	.tc_quality = 100,
	.tc_priv = NULL,
	.tc_next = NULL
};

CFATTACH_DECL_NEW(starclk, sizeof(struct starclk_softc),
    starclk_match, starclk_attach, NULL, NULL);

/* ARGSUSED */
static int
starclk_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa;

	sa = aux;

	sa->sa_size = STAR_TIMER_REGSIZE;
	return 1;
}

/* ARGSUSED */
static void
starclk_attach(device_t parent __unused, device_t self, void *aux)
{
	struct starclk_softc *sc;
	struct star_attach_args *sa;

	aprint_normal(": Programmable Timer\n");
	aprint_naive("\n");

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_addr = sa->sa_addr;

	if (bus_space_map(sc->sc_iot, sc->sc_addr, sa->sa_size, 0, &sc->sc_ioh))
	    panic("%s: Cannot map registers", device_xname(self));

	/*
	 * clock_sc would be referenced from cpu_initclocks(),
	 * setstatclockrate(), interrupt routines, etc.
	 */
	clock_sc = sc;

	/*
	 * set TIMER1 enable, reload when overflow, decrement
	 */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER_CTRL,
	    STAR_TIMER_CTRL_TM1ENABLE |
	    STAR_TIMER_CTRL_TM1OVERFLOWENABLE |
	    STAR_TIMER_CTRL_TM1_DECREMENT);

	star_timer1_set(sc, STAR_PCLK / hz);
}

void
setstatclockrate(int newhz)
{
	struct starclk_softc *sc;

	sc = clock_sc;
	star_timer2_set(sc, STAR_PCLK / newhz);
}

static void
star_timer1_set(struct starclk_softc *sc, uint32_t clk)
{
	sc->sc_timer1_clk_per_hz = clk;

	/*
	 * TIMER1 set up as decrement counter between <clk> and 0.
	 * When value is 0, occur an interrupts, and reloaded <clk>.
	 */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER1_COUNTER, clk);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER1_LOAD, clk);

	/* ~0 never match, no event occur */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER1_MATCH1, ~0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER1_MATCH2, ~0);
}

static void
star_timer2_set(struct starclk_softc *sc, uint32_t clk)
{
	sc->sc_timer2_clk_per_hz = clk;

	/*
	 * TIMER2 set up as decrement counter between <clk> and 0.
	 * When value is 0, occur an interrupts, and reloaded <clk>.
	 */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER2_COUNTER, clk);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER2_LOAD, clk);

	/* ~0 never match, no event occur */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER2_MATCH1, ~0);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER2_MATCH2, ~0);
}

void
cpu_initclocks(void)
{
	struct starclk_softc *sc;

	sc = clock_sc;
	if (sc == NULL)
		panic("cpu_initclocks: starclk not attached");

	profhz = stathz = STATHZ;

	/*
	 * reset TIMER1 and TIMER2 counter
	 */
	star_timer1_set(sc, STAR_PCLK / hz);
	star_timer2_set(sc, STAR_PCLK / stathz);

	/*
	 * enable interrupts TIMER1-Overflow
	 */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER_INT_MASK,
	    ~STAR_TIMER_INT_TM1OVERFLOW);

	/* clear all interrupts */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER_INT_STATUS,
	    0xffffffff);

	sc->sc_ih1 = intr_establish(STAR_IRQ_TIMER1, IPL_CLOCK,
	    IST_EDGE_RISING, star_timer1_intr, NULL);
	if (sc->sc_ih1 == NULL)
		panic("cpu_initclocks: unable to register timer1 interrupt");

	if (stathz) {
		/*
		 * enable interrupts TIMER2-Overflow
		 */
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER_INT_MASK,
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    STAR_TIMER_INT_MASK) & ~STAR_TIMER_INT_TM2OVERFLOW);

		/*
		 * set TIMER2 enable, reload when overflow, decrement
		 */
		bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER_CTRL,
		    bus_space_read_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER_CTRL) |
		    STAR_TIMER_CTRL_TM2ENABLE |
		    STAR_TIMER_CTRL_TM2OVERFLOWENABLE |
		    STAR_TIMER_CTRL_TM2_DECREMENT);

		sc->sc_ih2 = intr_establish(STAR_IRQ_TIMER2, IPL_HIGH,
		    IST_EDGE_RISING, star_timer2_intr, NULL);
		if (sc->sc_ih2 == NULL)
			panic("cpu_initclocks: "
			    "unable to register timer2 interrupt");
	}

	tc_init(&starclk_timecounter);
}

static int
star_timer1_intr(void *arg)
{
	struct starclk_softc *sc;

	sc = clock_sc;

	/* clear timer1 overflow */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER_INT_STATUS,
	    STAR_TIMER_INT_TM1OVERFLOW);

	atomic_add_32(&starclk_base, sc->sc_timer1_clk_per_hz);

	hardclock((struct clockframe *)arg);

	return 1;
}

static int
star_timer2_intr(void *arg)
{
	struct starclk_softc *sc;

	sc = clock_sc;

	/* clear timer2 overflow */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER_INT_STATUS,
	    STAR_TIMER_INT_TM2OVERFLOW);

	statclock((struct clockframe *)arg);

	return 1;
}

/* ARGSUSED */
static u_int
starclk_get_timecount(struct timecounter *tc __unused)
{
	struct starclk_softc *sc;
	uint32_t count, base;
	u_int s;

	sc = clock_sc;
	s = disable_interrupts(I32_bit);
	{
		/* TIMER1COUNTER start from (STAR_PCLK/hz) to 0 */
		count = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    STAR_TIMER1_COUNTER);
		base = starclk_base;
	}
	restore_interrupts(s);

	return base + sc->sc_timer1_clk_per_hz - count;
}

void
delay(unsigned int us)
{
	struct starclk_softc *sc;
	uint32_t prev, now, reload;
	int32_t left;

	sc = clock_sc;
	reload = bus_space_read_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER1_LOAD);
	left = (long long)us * STAR_PCLK / 1000000;

	/*
	 * STAR_TIMER1_COUNTER counts down from STAR_PCLK/hz to 0.
	 */
	prev = bus_space_read_4(sc->sc_iot, sc->sc_ioh, STAR_TIMER1_COUNTER);
	do {
		now = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    STAR_TIMER1_COUNTER);
		if (now > prev)
			left -= prev + reload - now;
		else
			left -= prev - now;
		prev = now;
	} while (left > 0);
}
