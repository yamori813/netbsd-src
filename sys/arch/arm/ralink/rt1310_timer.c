/*	$NetBSD$	*/

/*
 * Copyright (c) 2012 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Petri Laakso.
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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/errno.h>
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/timetc.h>

#include <arm/pic/picvar.h>

#include <arm/ralink/rt1310_reg.h>
#include <arm/ralink/rt1310_var.h>

extern int hz;
extern int stathz;

static int	timer_match(device_t, cfdata_t, void *);
static void	timer_attach(device_t, device_t, void *);
static int	timer_activate(device_t, enum devact);

static u_int	rt1310tmr_get_timecount(struct timecounter *);

static void	timer_reset(void);

extern struct cfdriver timer_cd;

/*
 * Timer IRQ handler definitions.
 */
static int	systimer_irq(void *);
static int	stattimer_irq(void *);

void	cpu_initclocks(void);
void 	setstatclockrate(int);

/* Allocated for each timer instance. */
struct timer_softc {
	device_t sc_dev;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_bsh0;
	bus_space_handle_t sc_bsh1;
	bus_space_handle_t sc_bsh2;
	int8_t sc_irq;
	int freq;
};

static struct timecounter rt1310tmr_timecounter = {
	.tc_get_timecount = rt1310tmr_get_timecount,
	.tc_counter_mask = ~0u,
	.tc_name = "rt1310tmr",
	.tc_quality = 1000,
};

static bus_space_tag_t timer_iot;
static bus_space_handle_t timer_hdl;

CFATTACH_DECL3_NEW(timer,
	sizeof(struct timer_softc),
	timer_match,
	timer_attach,
	NULL,
	timer_activate,
	NULL,
	NULL,
	0);

#define MAX_TIMERS	4
#define SYS_TIMER	0
#define STAT_TIMER	1	
#define SCHED_TIMER	2

struct timer_softc *timer_sc;

static void	timer_init(struct timer_softc *);

#define TIMER0_READ(sc, reg)						\
	bus_space_read_4(sc->sc_iot, sc->sc_bsh0, (reg))
#define TIMER0_WRITE(sc, reg, val)					\
	bus_space_write_4(sc->sc_iot, sc->sc_bsh0, (reg), (val))
#define TIMER1_READ(sc, reg)						\
	bus_space_read_4(sc->sc_iot, sc->sc_bsh1, (reg))
#define TIMER1_WRITE(sc, reg, val)					\
	bus_space_write_4(sc->sc_iot, sc->sc_bsh1, (reg), (val))
#define TIMER2_READ(sc, reg)						\
	bus_space_read_4(sc->sc_iot, sc->sc_bsh2, (reg))
#define TIMER2_WRITE(sc, reg, val)					\
	bus_space_write_4(sc->sc_iot, sc->sc_bsh2, (reg), (val))

#define SELECT_32KHZ	0x8	/* Use 32kHz clock source. */
#define SOURCE_32KHZ_HZ	32000	/* Above source in Hz. */

#define IRQ HW_TIMROT_TIMCTRL0_IRQ
#define IRQ_EN HW_TIMROT_TIMCTRL0_IRQ_EN
#define UPDATE HW_TIMROT_TIMCTRL0_UPDATE
#define RELOAD HW_TIMROT_TIMCTRL0_RELOAD

static int
timer_match(device_t parent, cfdata_t match, void *aux)
{
//	struct apb_attach_args *aa = aux;

//	if (aa->apba_addr == APB_TIMER_BASE)
	return 1;
}

static void
timer_attach(device_t parent, device_t self, void *aux)
{
	struct apb_attach_args *aa = aux;
	struct timer_softc *sc = device_private(self);
	static int timer_attached = 0;

	if (!timer_attached) {
		timer_iot = aa->apba_memt;
		if (bus_space_map(timer_iot, aa->apba_addr, RT_TIMER_SIZE,
		    0, &timer_hdl)) {
			aprint_error_dev(sc->sc_dev,
			    "unable to map bus space\n");
			return;
		}
		timer_reset();
		timer_attached = 1;
	}
/*
	if (aa->aa_addr == HW_TIMROT_BASE + HW_TIMROT_TIMCTRL0
	    && aa->aa_size == TIMER_REGS_SIZE
	    && timer_sc[SYS_TIMER] == NULL) {
*/
	sc->sc_iot = timer_iot;
	sc->sc_irq = aa->apba_intr;

	if (bus_space_subregion(timer_iot, timer_hdl,
	    0, 0x10,
	    &sc->sc_bsh0)) {
		aprint_error_dev(sc->sc_dev,
		    "unable to map subregion\n");
		return;
	}

	if (bus_space_subregion(timer_iot, timer_hdl,
	    0x10, 0x10,
	    &sc->sc_bsh1)) {
		aprint_error_dev(sc->sc_dev,
		    "unable to map subregion\n");
		return;
	}

	if (bus_space_subregion(timer_iot, timer_hdl,
	    0x20, 0x10,
	    &sc->sc_bsh2)) {
		aprint_error_dev(sc->sc_dev,
		    "unable to map subregion\n");
		return;
	}

	timer_sc = sc;
	aprint_normal("\n");
	return;
}

static int
timer_activate(device_t self, enum devact act)
{
	return EOPNOTSUPP;
}

/*
 * cpu_initclock is called once at the boot time.
 */
void
cpu_initclocks(void)
{

	timer_init(timer_sc);

	return;
}

/*
 * Change statclock rate when profiling takes place.
 */
void
setstatclockrate(int newhz)
{

	return;
}

/*
 * Generic timer initialization function.
 */
static void
timer_init(struct timer_softc *sc)
{
//	uint32_t reg;

	intr_establish(sc->sc_irq, IPL_SCHED, IST_LEVEL, systimer_irq, NULL);
	intr_establish(sc->sc_irq + 1, IPL_SCHED, IST_LEVEL, stattimer_irq,
	    NULL);

	TIMER0_WRITE(sc, RT_TIMER_CONTROL, 0);
	TIMER0_WRITE(sc, RT_TIMER_LOAD, RT_APB_FREQ);
	TIMER0_WRITE(sc, RT_TIMER_VALUE, RT_APB_FREQ);
	TIMER0_WRITE(sc, RT_TIMER_CONTROL,
	    RT_TIMER_CTRL_ENABLE | RT_TIMER_CTRL_INTCTL);

	TIMER2_WRITE(sc, RT_TIMER_CONTROL, 0);
	TIMER2_WRITE(sc, RT_TIMER_LOAD, ~0u);
	TIMER2_WRITE(sc, RT_TIMER_VALUE, ~0u);
	TIMER2_WRITE(sc, RT_TIMER_CONTROL, RT_TIMER_CTRL_DIV16 |
	    RT_TIMER_CTRL_ENABLE | RT_TIMER_CTRL_PERIODCAL);
	rt1310tmr_timecounter.tc_priv = sc;
	rt1310tmr_timecounter.tc_frequency = RT_APB_FREQ;
	tc_init(&rt1310tmr_timecounter);
#if 0
	if (sc->sc_irq == 31) {
		/* 100 Hz */
		TIMER_WRITE(sc, TIMER0_HIGH_BOUND, 2048000);
		TIMER_WRITE(sc, TIMER0_CURRENT_COUNT, 0);
		reg = TIMER_READ(sc, TIMER_IRQ_MASK);
		TIMER_WRITE(sc, TIMER_IRQ_MASK, reg | 1);

		TIMER_WRITE(sc, TIMER2_HIGH_BOUND, ~0u);
		TIMER_WRITE(sc, TIMER2_CURRENT_COUNT, 0);
		rt1310tmr_timecounter.tc_priv = sc;
		rt1310tmr_timecounter.tc_frequency = COMCERTO_APB_FREQ;
		tc_init(&rt1310tmr_timecounter);
	} else {
		if (sc->freq != 0) {
			TIMER_WRITE(sc, TIMER1_HIGH_BOUND, 2048000);
			TIMER_WRITE(sc, TIMER1_CURRENT_COUNT, 0);
			reg = TIMER_READ(sc, TIMER_IRQ_MASK);
			TIMER_WRITE(sc, TIMER_IRQ_MASK, reg | (1 << 1));
		}
	}
#endif

	return;
}

static u_int
rt1310tmr_get_timecount(struct timecounter *tc)
{
struct timer_softc *sc = tc->tc_priv;

	return TIMER2_READ(sc, RT_TIMER_VALUE);
}

/*
 * Timer IRQ handlers.
 */
static int
systimer_irq(void *frame)
{
	struct timer_softc *sc = timer_sc;

	TIMER0_WRITE(sc, RT_TIMER_CONTROL, 0);
	TIMER0_WRITE(sc, RT_TIMER_LOAD, RT_APB_FREQ);
	TIMER0_WRITE(sc, RT_TIMER_VALUE, RT_APB_FREQ);
	TIMER0_WRITE(sc, RT_TIMER_CONTROL,
	    RT_TIMER_CTRL_ENABLE | RT_TIMER_CTRL_INTCTL);

	hardclock(frame);

	return 1;
}

static int
stattimer_irq(void *frame)
{

	statclock(frame);

	return 1;
}

/*
 * Reset the TIMROT block.
 *
 * Inspired by i.MX23 RM "39.3.10 Correct Way to Soft Reset a Block"
 */
static void
timer_reset(void)
{
	return;
}

void
delay(u_int n)
{
	struct timer_softc * const sc = device_lookup_private(&timer_cd, 0);
	uint32_t start, now;

	start = TIMER2_READ(sc, RT_TIMER_VALUE);
	while(1) {
		now = TIMER2_READ(sc, RT_TIMER_VALUE);
		if (now < start) {
			if(start - now > n * 100)
				break;
		} else {
			if(start + (~0u - now) > n * 100)
				break;
		}
	}
}
