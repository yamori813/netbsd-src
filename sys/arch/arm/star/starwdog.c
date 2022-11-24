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
#include <sys/wdog.h>
#include <dev/sysmon/sysmonvar.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

/* PCLK or 10Hz mode (default: 10Hz mode) */
#undef STAR_WDOG_USE_PCLK

#ifdef STAR_WDOG_USE_PCLK
#define STAR_WDOG_HZ	STAR_PCLK
#else
#define STAR_WDOG_HZ	10
#endif
#define STAR_WDOG_PERIOD_DEFAULT	30
#define STAR_WDOG_PERIOD_MAX		(0xffffffff / STAR_WDOG_HZ)

struct starwdog_softc {
	device_t sc_dev;
	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	void *sc_ih;

	uint32_t sc_period;
	struct sysmon_wdog sc_smw;
};

static int starwdog_match(device_t, struct cfdata *, void *);
static void starwdog_attach(device_t, device_t, void *);
static void starwdog_init(struct starwdog_softc *);
static void starwdog_enable(struct starwdog_softc *);
static void starwdog_disable(struct starwdog_softc *);
static void starwdog_period(struct starwdog_softc *, uint32_t);
static int starwdog_intr(void *);
static int starwdog_setmode(struct sysmon_wdog *);
static int starwdog_tickle(struct sysmon_wdog *);

CFATTACH_DECL_NEW(starwdog, sizeof(struct starwdog_softc),
    starwdog_match, starwdog_attach, NULL, NULL);


/* ARGSUSED */
static int
starwdog_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa;

	sa = aux;
	sa->sa_size = STAR_WDT_REGSIZE;

	return 1;
}

/* ARGSUSED */
static void
starwdog_attach(device_t parent __unused, device_t self, void *aux)
{
	struct starwdog_softc *sc;
	struct star_attach_args *sa;

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_addr = sa->sa_addr;

	aprint_normal(": Watch Dog Timer\n");
	aprint_naive("\n");

	if (bus_space_map(sc->sc_iot, sc->sc_addr, sa->sa_size, 0,
	    &sc->sc_ioh)) {
		aprint_error(": can't map registers\n");
		return;
	}

	starwdog_init(sc);
	sc->sc_period = STAR_WDOG_PERIOD_DEFAULT;
	starwdog_period(sc, sc->sc_period);

	sc->sc_smw.smw_cookie = sc;
	sc->sc_smw.smw_name = device_xname(sc->sc_dev);
	sc->sc_smw.smw_setmode = starwdog_setmode;
	sc->sc_smw.smw_tickle = starwdog_tickle;
	sc->sc_smw.smw_period = sc->sc_period;
	if (sysmon_wdog_register(&sc->sc_smw) != 0)
		aprint_error("%s: unable to register with sysmon\n",
		    device_xname(sc->sc_dev));

	sc->sc_ih = star_intr_establish(STAR_IRQ_WDOG, IPL_HIGH,
	    STAR_INTR_RISING_EDGE, starwdog_intr, sc);
}

static void
starwdog_init(struct starwdog_softc *sc)
{
	uint32_t val;

	if (CPU_IS_STR8100()) {
		/* GPIOA WDTRST ENABLE */
		val = STAR_REG_READ32(EQUULEUS_MISC_GPIOA_PIN_REG);
		STAR_REG_WRITE32(EQUULEUS_MISC_GPIOA_PIN_REG,
		    val | EQUULEUS_MISC_GPIOA25_WDTRST);

#ifdef STAR_WDOG_USE_PCLK
		/* WDT Controler Enable */
		val = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKGATE1_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKGATE1_REG,
		    val | EQUULEUS_CLKPWR_CLKGATE1_PCLK_WDT);
#endif
		/* activate WDT */
		val = STAR_REG_READ32(EQUULEUS_CLKPWR_SOFTRST_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_SOFTRST_REG,
		    val | EQUULEUS_CLKPWR_SOFTRST_WDT);
	} else {
		/* no need to activate PCLK on STR9100 */
	}

	/* configuration */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_WDT_CONTROL_REG,
#ifdef STAR_WDOG_USE_PCLK
	    STAR_WDT_CONTROL_CLOCK_PCLK |
#else
	    STAR_WDT_CONTROL_CLOCK_10Hz |
#endif
	    STAR_WDT_CONTROL_EXT |
	    STAR_WDT_CONTROL_INTR |
	    STAR_WDT_CONTROL_RESET);

	/* clear interrupt */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_WDT_CLEAR_REG,
	    0xffffffff);

	/* set interrupt length */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_WDT_INTRLEN_REG, 0xff);
}

static void
starwdog_enable(struct starwdog_softc *sc)
{
	uint32_t status;

	/* enable watchdog */
	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, STAR_WDT_CONTROL_REG);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_WDT_CONTROL_REG,
	    status | STAR_WDT_CONTROL_ENABLE);
}

static void
starwdog_disable(struct starwdog_softc *sc)
{
	uint32_t status;

	/* disable watchdog */
	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh, STAR_WDT_CONTROL_REG);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_WDT_CONTROL_REG,
	    status & ~STAR_WDT_CONTROL_ENABLE);
}

static void
starwdog_period(struct starwdog_softc *sc, uint32_t period)
{
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_WDT_LOAD_REG,
	    period * STAR_WDOG_HZ);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, STAR_WDT_RESTART_REG,
	    STAR_WDT_RESTART_MAGIC);
}

static int
starwdog_intr(void *arg)
{
	struct starwdog_softc *sc;
	uint32_t status;

	sc = (struct starwdog_softc *)arg;

	/* clear status */
	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    STAR_WDT_STATUS_REG);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    STAR_WDT_CLEAR_REG, status);

	printf("%s: interrupt occured\n", device_xname(sc->sc_dev));

	return 1;
}

static int
starwdog_setmode(struct sysmon_wdog *smw)
{
	struct starwdog_softc *sc;

	sc = smw->smw_cookie;
	if ((smw->smw_mode & WDOG_MODE_MASK) == WDOG_MODE_DISARMED) {
		starwdog_disable(sc);
		return 0;
	}

	if (smw->smw_period == WDOG_PERIOD_DEFAULT)
		smw->smw_period = sc->sc_period;
	if (smw->smw_period > STAR_WDOG_PERIOD_MAX)
		return EINVAL;

	starwdog_period(sc, smw->smw_period);
	starwdog_enable(sc);

	return 0;
}

static int
starwdog_tickle(struct sysmon_wdog *smw)
{
	struct starwdog_softc *sc;

	sc = smw->smw_cookie;
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    STAR_WDT_RESTART_REG, STAR_WDT_RESTART_MAGIC);

	return 0;
}
