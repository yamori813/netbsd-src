/* $Id$ */

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

#include <arm/pic/picvar.h>

#include <arm/mindspeed/m83xxx_reg.h>
#include <arm/mindspeed/m83xxx_var.h>

extern int hz;
extern int stathz;

static int	timer_match(device_t, cfdata_t, void *);
static void	timer_attach(device_t, device_t, void *);
static int	timer_activate(device_t, enum devact);

static void	timer_reset(void);

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
	bus_space_handle_t sc_hdl;
	int8_t sc_irq;
	int (*irq_handler)(void *);
	int freq;
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

struct timer_softc *timer_sc[MAX_TIMERS];

static void	timer_init(struct timer_softc *);

#define TIMROT_SOFT_RST_LOOP 455 /* At least 1 us ... */
#define TIMROT_READ(reg)						\
	bus_space_read_4(timer_iot, timer_hdl, (reg))
#define TIMROT_WRITE(reg, val)						\
	bus_space_write_4(timer_iot, timer_hdl, (reg), (val))

#define TIMER_REGS_SIZE 0x20

#define TIMER_CTRL	0x00
#define TIMER_CTRL_SET	0x04
#define TIMER_CTRL_CLR	0x08
#define TIMER_CTRL_TOG	0x0C
#define TIMER_COUNT	0x10

#define TIMER_READ(sc, reg)						\
	bus_space_read_4(sc->sc_iot, sc->sc_hdl, (reg))
#define TIMER_WRITE(sc, reg, val)					\
	bus_space_write_4(sc->sc_iot, sc->sc_hdl, (reg), (val))
#define TIMER_WRITE_2(sc, reg, val)					\
	bus_space_write_2(sc->sc_iot, sc->sc_hdl, (reg), (val))

#define SELECT_32KHZ	0x8	/* Use 32kHz clock source. */
#define SOURCE_32KHZ_HZ	32000	/* Above source in Hz. */

#define IRQ HW_TIMROT_TIMCTRL0_IRQ
#define IRQ_EN HW_TIMROT_TIMCTRL0_IRQ_EN
#define UPDATE HW_TIMROT_TIMCTRL0_UPDATE
#define RELOAD HW_TIMROT_TIMCTRL0_RELOAD

static int
timer_match(device_t parent, cfdata_t match, void *aux)
{
#if 0
	struct apb_attach_args *aa = aux;

	if ((aa->aa_addr == HW_TIMROT_BASE + HW_TIMROT_TIMCTRL0
	    && aa->aa_size == TIMER_REGS_SIZE))
		return 1;

	if ((aa->aa_addr == HW_TIMROT_BASE + HW_TIMROT_TIMCTRL1
	    && aa->aa_size == TIMER_REGS_SIZE))
		return 1;

#if 0
	if ((aa->aa_addr == HW_TIMROT_BASE + HW_TIMROT_TIMCTRL2
	    && aa->aa_size == TIMER_REGS_SIZE))
		return 1;

	if ((aa->aa_addr == HW_TIMROT_BASE + HW_TIMROT_TIMCTRL3
	    && aa->aa_size == TIMER_REGS_SIZE))
		return 1;
#endif
	return 0;
#endif
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
		if (bus_space_map(timer_iot, APB_TIMER_BASE, 0x100,
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
	 if(timer_sc[SYS_TIMER] == NULL) {
		if (bus_space_subregion(timer_iot, timer_hdl, 
		    0, 0x100,
		    &sc->sc_hdl)) {
			aprint_error_dev(sc->sc_dev,
			    "unable to map subregion\n");
			return;
		}

		sc->sc_iot = aa->apba_memt;
		sc->sc_irq = aa->apba_intr;
		sc->irq_handler = &systimer_irq;
		sc->freq = hz;

		timer_sc[SYS_TIMER] = sc;

		aprint_normal(": sys clock\n");
	} else {
		if (bus_space_subregion(timer_iot, timer_hdl, 
		    0, 0x100,
		    &sc->sc_hdl)) {
			aprint_error_dev(sc->sc_dev,
			    "unable to map subregion\n");
			return;
		}

		sc->sc_iot = aa->apba_memt;
		sc->sc_irq = aa->apba_intr;
		sc->irq_handler = &stattimer_irq;
		sc->freq = stathz;

		timer_sc[STAT_TIMER] = sc;

		aprint_normal(": stat clock\n");
	}

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
	if (timer_sc[SYS_TIMER] != NULL)
		timer_init(timer_sc[SYS_TIMER]);

	if (timer_sc[STAT_TIMER] != NULL)
		timer_init(timer_sc[STAT_TIMER]);

	return;
}

/*
 * Change statclock rate when profiling takes place.
 */
void
setstatclockrate(int newhz)
{
	struct timer_softc *sc = timer_sc[STAT_TIMER];
	sc->freq = newhz;

#if 0
	TIMER_WRITE_2(sc, TIMER_COUNT,
	    __SHIFTIN(SOURCE_32KHZ_HZ / sc->freq - 1,
	    HW_TIMROT_TIMCOUNT0_FIXED_COUNT));
#endif

	return;
}

/*
 * Generic timer initialization function.
 */
static void
timer_init(struct timer_softc *sc)
{
#if 0
	uint32_t ctrl;

	TIMER_WRITE_2(sc, TIMER_COUNT,
	    __SHIFTIN(SOURCE_32KHZ_HZ / sc->freq - 1,
	    HW_TIMROT_TIMCOUNT0_FIXED_COUNT));
	ctrl = IRQ_EN | UPDATE | RELOAD | SELECT_32KHZ;
	TIMER_WRITE(sc, TIMER_CTRL, ctrl);
#endif
	intr_establish(sc->sc_irq, IPL_SCHED, IST_LEVEL, sc->irq_handler, NULL);

	if (sc->sc_irq == 31) {
		TIMER_WRITE(sc, TIMER0_HIGH_BOUND, 0x20000000);
		TIMER_WRITE(sc, TIMER0_CURRENT_COUNT, 0);
		TIMER_WRITE(sc, TIMER_IRQ_MASK, 1);
	} else {
		TIMER_WRITE(sc, TIMER1_HIGH_BOUND, 0x10000000);
		TIMER_WRITE(sc, TIMER1_CURRENT_COUNT, 0);
		TIMER_WRITE(sc, TIMER_IRQ_MASK, 3);
	}

	return;
}

/*
 * Timer IRQ handlers.
 */
static int
systimer_irq(void *frame)
{
//printf("+");
//	hardclock(frame);

//	TIMER_WRITE(timer_sc[SYS_TIMER], TIMER_CTRL_CLR, IRQ);

	return 1;
}

static int
stattimer_irq(void *frame)
{
//printf(".");
//	statclock(frame);
//	TIMER_WRITE(timer_sc[STAT_TIMER], TIMER_CTRL_CLR, IRQ);

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
#if 0
	unsigned int loop;
	
	/* Prepare for soft-reset by making sure that SFTRST is not currently
	* asserted. Also clear CLKGATE so we can wait for its assertion below.
	*/
	TIMROT_WRITE(HW_TIMROT_ROTCTRL_CLR, HW_TIMROT_ROTCTRL_SFTRST);

	/* Wait at least a microsecond for SFTRST to deassert. */
	loop = 0;
	while ((TIMROT_READ(HW_TIMROT_ROTCTRL) & HW_TIMROT_ROTCTRL_SFTRST) ||
	    (loop < TIMROT_SOFT_RST_LOOP))
		loop++;

	/* Clear CLKGATE so we can wait for its assertion below. */
	TIMROT_WRITE(HW_TIMROT_ROTCTRL_CLR, HW_TIMROT_ROTCTRL_CLKGATE);

	/* Soft-reset the block. */
	TIMROT_WRITE(HW_TIMROT_ROTCTRL_SET, HW_TIMROT_ROTCTRL_SFTRST);

	/* Wait until clock is in the gated state. */
	while (!(TIMROT_READ(HW_TIMROT_ROTCTRL) & HW_TIMROT_ROTCTRL_CLKGATE));

	/* Bring block out of reset. */
	TIMROT_WRITE(HW_TIMROT_ROTCTRL_CLR, HW_TIMROT_ROTCTRL_SFTRST);

	loop = 0;
	while ((TIMROT_READ(HW_TIMROT_ROTCTRL) & HW_TIMROT_ROTCTRL_SFTRST) ||
	    (loop < TIMROT_SOFT_RST_LOOP))
		loop++;

	TIMROT_WRITE(HW_TIMROT_ROTCTRL_CLR, HW_TIMROT_ROTCTRL_CLKGATE);
	/* Wait until clock is in the NON-gated state. */
	while (TIMROT_READ(HW_TIMROT_ROTCTRL) & HW_TIMROT_ROTCTRL_CLKGATE);

#endif
	return;
}
