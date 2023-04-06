/*	$NetBSD$	*/

/*
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

#include "gpio.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kmem.h>

#include <machine/intr.h>
#include <sys/bus.h>

#include <arm/xscale/ixp425_sipvar.h>
#include <arm/xscale/ixp425reg.h>
#include <arm/xscale/ixp425var.h>
#include <arm/xscale/ixp425_gpio.h>

#include "locators.h"

#include <sys/gpio.h>
#include <dev/gpio/gpiovar.h>

#define GPIO_NPINS 16

struct gpio_irq_handler {
	struct gpio_irq_handler *gh_next;
	int (*gh_func)(void *);
	void *gh_arg;
	int gh_spl;
	u_int gh_gpio;
	int gh_level;
};

struct ixpgpio_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bust;
	bus_space_handle_t sc_bush;
	void *sc_irqcookie[GPIO_NPINS];
	uint32_t sc_mask;
	struct gpio_irq_handler *sc_handlers[GPIO_NPINS];
	struct gpio_chipset_tag sc_gpio_gc;
	gpio_pin_t sc_gpio_pins[GPIO_NPINS];
};

static int	ixpgpio_match(device_t, cfdata_t, void *);
static void	ixpgpio_attach(device_t, device_t, void *);

#if NGPIO > 0
static int	ixp425_gpio_pin_read(void *, int);
static void	ixp425_gpio_pin_write(void *, int, int);
static void	ixp425_gpio_pin_ctl(void *, int, int);
#endif

CFATTACH_DECL_NEW(ixpgpio, sizeof(struct ixpgpio_softc),
    ixpgpio_match, ixpgpio_attach, NULL, NULL);

static struct ixpgpio_softc *ixpgpio_softc;
static vaddr_t ixpgpio_regs;
#define GPIO_BOOTSTRAP_REG(reg)	\
	(*((volatile uint32_t *)(ixpgpio_regs + (reg))))

static int gpio_dispatch(struct ixpgpio_softc *);
static int gpio_intrN(void *);

static inline uint32_t
ixpgpio_reg_read(struct ixpgpio_softc *sc, int reg)
{

	return (bus_space_read_4(sc->sc_bust, sc->sc_bush, reg));
}

static inline void
ixpgpio_reg_write(struct ixpgpio_softc *sc, int reg, uint32_t val)
{

	bus_space_write_4(sc->sc_bust, sc->sc_bush, reg, val);

	return;
}

static int
ixpgpio_match(device_t parent, cfdata_t cf, void *aux)
{
	struct ixpsip_attach_args *sa = aux;

#if 0
	if (ixpgpio_softc != NULL || ixp->ixp_addr != IXP425_GPIO_BASE)
		return (0);
#endif

	sa->sa_size = 0x1000;

	return (1);
}

static void
ixpgpio_attach(device_t parent, device_t self, void *aux)
{
	struct ixpgpio_softc *sc = device_private(self);
	struct ixpsip_attach_args *sa = aux;

#if NGPIO > 0
	struct gpiobus_attach_args gba;
	int maxpin;
//	int pin;
//	u_int func;
#endif

	sc->sc_dev = self;
	sc->sc_bust = sa->sa_iot;

	sc->sc_mask = 0;

	aprint_normal(": GPIO Controller\n");

	if (bus_space_map(sc->sc_bust, sa->sa_addr, sa->sa_size, 0,
	    &sc->sc_bush)) {
		aprint_error_dev(self, "Can't map registers!\n");
		return;
	}

	ixpgpio_regs = (vaddr_t)bus_space_vaddr(sc->sc_bust, sc->sc_bush);

	memset(sc->sc_handlers, 0, sizeof(sc->sc_handlers));

#if 0
	/*
	 * Disable all GPIO interrupts
	 */
	ixpgpio_reg_write(sc, GPIO_GRER0, 0);
	ixpgpio_reg_write(sc, GPIO_GRER1, 0);
	ixpgpio_reg_write(sc, GPIO_GRER2, 0);
	ixpgpio_reg_write(sc, GPIO_GFER0, 0);
	ixpgpio_reg_write(sc, GPIO_GFER1, 0);
	ixpgpio_reg_write(sc, GPIO_GFER2, 0);
	ixpgpio_reg_write(sc, GPIO_GEDR0, ~0);
	ixpgpio_reg_write(sc, GPIO_GEDR1, ~0);
	ixpgpio_reg_write(sc, GPIO_GEDR2, ~0);
#ifdef	CPU_XSCALE_PXA270
	if (CPU_IS_PXA270) {
		ixpgpio_reg_write(sc, GPIO_GRER3, 0);
		ixpgpio_reg_write(sc, GPIO_GFER3, 0);
		ixpgpio_reg_write(sc, GPIO_GEDR3, ~0);
	}
#endif
#endif

	ixpgpio_softc = sc;
#if NGPIO > 0
	maxpin = GPIO_NPINS;
#if 0
	for (pin = 0; pin < maxpin; ++pin) {

		sc->sc_gpio_pins[pin].pin_num = pin;
		func = ixp425_gpio_get_function(pin);

		if (GPIO_IS_GPIO(func)) {
			sc->sc_gpio_pins[pin].pin_caps = GPIO_PIN_INPUT |
			    GPIO_PIN_OUTPUT;
			sc->sc_gpio_pins[pin].pin_state =
			ixp425_gpio_pin_read(sc, pin);
		} else {
			sc->sc_gpio_pins[pin].pin_caps = 0;
			sc->sc_gpio_pins[pin].pin_state = 0;
		}
	}
#endif

	/* create controller tag */
	sc->sc_gpio_gc.gp_cookie = sc;
	sc->sc_gpio_gc.gp_pin_read = ixp425_gpio_pin_read;
	sc->sc_gpio_gc.gp_pin_write = ixp425_gpio_pin_write;
	sc->sc_gpio_gc.gp_pin_ctl = ixp425_gpio_pin_ctl;

	gba.gba_gc = &sc->sc_gpio_gc;
	gba.gba_pins = sc->sc_gpio_pins;
	gba.gba_npins = maxpin;

	config_found(self, &gba, gpiobus_print, CFARGS_NONE);
#else
	aprint_normal_dev(sc->sc_dev, "no GPIO configured in kernel\n");
#endif
}

void *
ixp425_gpio_intr_establish(u_int irq, int level, int spl, int (*func)(void *),
    void *arg)
{
	struct ixpgpio_softc *sc = ixpgpio_softc;
	struct gpio_irq_handler *gh;
	uint32_t bit, reg;
	int irqs[] = {6, 7, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29};
	int gpio;
	int i;

	for (i = 0;i < sizeof(irqs); ++i) {
		if (irqs[i] == irq) {
			gpio = i;
			break;
		}
	}
	if (i == sizeof(irqs))
		panic("ixp425_gpio_intr_establish: bad irq number: %d", irq);

//	if (!GPIO_IS_GPIO_IN(ixp425_gpio_get_function(gpio)))
//		panic("ixp425_gpio_intr_establish: Pin %d not GPIO_IN", gpio);

	switch (level) {
	case IST_EDGE_FALLING:
	case IST_EDGE_RISING:
	case IST_EDGE_BOTH:
		break;

	default:
		panic("ixp425_gpio_intr_establish: bad level: %d", level);
		break;
	}

	if (sc->sc_handlers[gpio] != NULL)
		panic("ixp425_gpio_intr_establish: illegal shared interrupt");

	irq = irqs[gpio];

	gh = kmem_alloc(sizeof(*gh), KM_SLEEP);
	gh->gh_func = func;
	gh->gh_arg = arg;
	gh->gh_spl = spl;
	gh->gh_gpio = gpio;
	gh->gh_level = level;
	gh->gh_next = sc->sc_handlers[gpio];
	sc->sc_handlers[gpio] = gh;

//	KDASSERT(sc->sc_irqcookie[1] == NULL);
	sc->sc_irqcookie[gpio] = ixp425_intr_establish(irq, spl, gpio_intrN,
	    sc);
//	KDASSERT(sc->sc_irqcookie[1]);

	bit = 0;
	sc->sc_mask |= (1 << gpio);

	switch (level) {
	case IST_EDGE_FALLING:
		bit = GPIO_STYLE_FALLING_EDGE;
		break;

	case IST_EDGE_RISING:
		bit = GPIO_STYLE_RISING_EDGE;
		break;

	case IST_EDGE_BOTH:
		bit = GPIO_STYLE_TRANSITIONAL;
		break;
	}
	if (gpio >= 8) {   /* pins 8-15 */
		reg = ixpgpio_reg_read(sc, GPIO_GPIT2R);
		reg &= ~(GPIO_STYLE_MASK << (gpio - 8));
		reg |= (bit << (gpio - 8));
		ixpgpio_reg_write(sc, GPIO_GPIT2R, reg);
	} else {   /* pins 0-7 */
		reg = ixpgpio_reg_read(sc, GPIO_GPIT1R);
		reg &= ~(GPIO_STYLE_MASK << gpio);
		reg |= (bit << gpio);
		ixpgpio_reg_write(sc, GPIO_GPIT1R, reg);
	}

	return (gh);
}

void
ixp425_gpio_intr_disestablish(void *cookie)
{
#if 0
	struct ixpgpio_softc *sc = ixpgpio_softc;
	struct gpio_irq_handler *gh = cookie;
	uint32_t bit, reg;

	bit = GPIO_BIT(gh->gh_gpio);

	reg = ixpgpio_reg_read(sc, GPIO_REG(GPIO_GFER0, gh->gh_gpio));
	reg &= ~bit;
	ixpgpio_reg_write(sc, GPIO_REG(GPIO_GFER0, gh->gh_gpio), reg);
	reg = ixpgpio_reg_read(sc, GPIO_REG(GPIO_GRER0, gh->gh_gpio));
	reg &= ~bit;
	ixpgpio_reg_write(sc, GPIO_REG(GPIO_GRER0, gh->gh_gpio), reg);

	ixpgpio_reg_write(sc, GPIO_REG(GPIO_GEDR0, gh->gh_gpio), bit);

	sc->sc_mask[GPIO_BANK(gh->gh_gpio)] &= ~bit;
	sc->sc_handlers[gh->gh_gpio] = NULL;

	if (gh->gh_gpio == 0) {
#if 0
		ixp425_intr_disestablish(sc->sc_irqcookie[0]);
		sc->sc_irqcookie[0] = NULL;
#else
		panic("ixp425_gpio_intr_disestablish: can't unhook GPIO#0");
#endif
	} else
	if (gh->gh_gpio == 1) {
#if 0
		ixp425_intr_disestablish(sc->sc_irqcookie[1]);
		sc->sc_irqcookie[1] = NULL;
#else
		panic("ixp425_gpio_intr_disestablish: can't unhook GPIO#1");
#endif
	}

	kmem_free(gh, sizeof(*gh));
#endif
}

static int
gpio_dispatch(struct ixpgpio_softc *sc)
{
	struct gpio_irq_handler **ghp, *gh;
	int i, s, nhandled, handled, pins;
	uint32_t gedr, mask;

	/* Fetch bitmap of pending interrupts on this GPIO bank */
	gedr = ixpgpio_reg_read(sc, GPIO_GPISR);

	/* Bail early if there are no pending interrupts in this bank */
	if (gedr == 0)
		return (0);

	/* Acknowledge pending interrupts. */
	ixpgpio_reg_write(sc, GPIO_GPISR, gedr);

	/*
	 * We're only interested in those for which we have a handler
	 * registered
	 */

	gedr &= sc->sc_mask;
	ghp = &sc->sc_handlers[0];
	pins = GPIO_NPINS;
	handled = 0;

	for (i = 0; i < pins; ++i) {
		mask = 1 << i;
		if ((gedr & mask) == 0)
			continue;
		gedr &= ~mask;

		if ((gh = ghp[i]) == NULL) {
			aprint_error_dev(sc->sc_dev,
			    "unhandled GPIO interrupt. GPIO#%d\n", i);
			continue;
		}

		s = _splraise(gh->gh_spl);
		do {
			nhandled = (gh->gh_func)(gh->gh_arg);
			handled |= nhandled;
			gh = gh->gh_next;
		} while (gh != NULL);
		splx(s);
	}

	return (handled);
}

static int
gpio_intrN(void *arg)
{
	struct ixpgpio_softc *sc = arg;
	int handled;

	handled = gpio_dispatch(sc);

	return (handled);
}

#if 0
u_int
ixp425_gpio_get_function(u_int gpio)
{
	struct ixpgpio_softc *sc = ixpgpio_softc;
	uint32_t rv, io;

	KDASSERT(gpio < GPIO_NPINS);

	rv = ixpgpio_reg_read(sc, GPIO_FN_REG(gpio)) >> GPIO_FN_SHIFT(gpio);
	rv = GPIO_FN(rv);

	io = ixpgpio_reg_read(sc, GPIO_REG(GPIO_GPDR0, gpio));
	if (io & GPIO_BIT(gpio))
		rv |= GPIO_OUT;

	io = ixpgpio_reg_read(sc, GPIO_REG(GPIO_GPLR0, gpio));
	if (io & GPIO_BIT(gpio))
		rv |= GPIO_SET;

	return (rv);
}

u_int
ixp425_gpio_set_function(u_int gpio, u_int fn)
{
	struct ixpgpio_softc *sc = ixpgpio_softc;
	uint32_t rv, bit;
	u_int oldfn;

	KDASSERT(gpio < GPIO_NPINS);

	oldfn = ixp425_gpio_get_function(gpio);

	if (GPIO_FN(fn) == GPIO_FN(oldfn) &&
	    GPIO_FN_IS_OUT(fn) == GPIO_FN_IS_OUT(oldfn)) {
		/*
		 * The pin's function is not changing.
		 * For Alternate Functions and GPIO input, we can just
		 * return now.
		 * For GPIO output pins, check the initial state is
		 * the same.
		 *
		 * Return 'fn' instead of 'oldfn' so the caller can
		 * reliably detect that we didn't change anything.
		 * (The initial state might be different for non-
		 * GPIO output pins).
		 */
		if (!GPIO_IS_GPIO_OUT(fn) ||
		    GPIO_FN_IS_SET(fn) == GPIO_FN_IS_SET(oldfn))
			return (fn);
	}

	/*
	 * See section 4.1.3.7 of the PXA2x0 Developer's Manual for
	 * the correct procedure for changing GPIO pin functions.
	 */

	bit = GPIO_BIT(gpio);

	/*
	 * 1. Configure the correct set/clear state of the pin
	 */
	if (GPIO_FN_IS_SET(fn))
		ixpgpio_reg_write(sc, GPIO_REG(GPIO_GPSR0, gpio), bit);
	else
		ixpgpio_reg_write(sc, GPIO_REG(GPIO_GPCR0, gpio), bit);

	/*
	 * 2. Configure the pin as an input or output as appropriate
	 */
	rv = ixpgpio_reg_read(sc, GPIO_REG(GPIO_GPDR0, gpio)) & ~bit;
	if (GPIO_FN_IS_OUT(fn))
		rv |= bit;
	ixpgpio_reg_write(sc, GPIO_REG(GPIO_GPDR0, gpio), rv);

	/*
	 * 3. Configure the pin's function
	 */
	bit = GPIO_FN_MASK << GPIO_FN_SHIFT(gpio);
	fn = GPIO_FN(fn) << GPIO_FN_SHIFT(gpio);
	rv = ixpgpio_reg_read(sc, GPIO_FN_REG(gpio)) & ~bit;
	ixpgpio_reg_write(sc, GPIO_FN_REG(gpio), rv | fn);

	return (oldfn);
}

/* 
 * Quick function to clear interrupt status on a pin
 * GPIO pins may be toggle in an interrupt and we dont want
 * extra spurious interrupts to occur.
 * Suppose this causes a slight race if a key is pressed while
 * the interrupt handler is running. (yes this is for the keyboard driver)
 */
void
ixp425_gpio_clear_intr(u_int gpio)
{
	struct ixpgpio_softc *sc = ixpgpio_softc;
	int bit;

	bit = GPIO_BIT(gpio);
	ixpgpio_reg_write(sc, GPIO_REG(GPIO_GEDR0, gpio), bit);
}

/*
 * Quick function to mask (disable) a GPIO interrupt
 */
void
ixp425_gpio_intr_mask(void *v)
{
	struct gpio_irq_handler *gh = (struct gpio_irq_handler *)v;

	ixp425_gpio_set_intr_level(gh->gh_gpio, IPL_NONE);
}

/*
 * Quick function to unmask (enable) a GPIO interrupt
 */
void
ixp425_gpio_intr_unmask(void *v)
{
	struct gpio_irq_handler *gh = (struct gpio_irq_handler *)v;

	ixp425_gpio_set_intr_level(gh->gh_gpio, gh->gh_level);
}

/*
 * Configure the edge sensitivity of interrupt pins
 */
void
ixp425_gpio_set_intr_level(u_int gpio, int level)
{
	struct ixpgpio_softc *sc = ixpgpio_softc;
	uint32_t bit;
	uint32_t gfer;
	uint32_t grer;
	int s;

	s = splhigh();

	bit = GPIO_BIT(gpio);
	gfer = ixpgpio_reg_read(sc, GPIO_REG(GPIO_GFER0, gpio));
	grer = ixpgpio_reg_read(sc, GPIO_REG(GPIO_GRER0, gpio));

	switch (level) {
	case IST_NONE:
		gfer &= ~bit;
		grer &= ~bit;
		break;
	case IST_EDGE_FALLING:
		gfer |= bit;
		grer &= ~bit;
		break;
	case IST_EDGE_RISING:
		gfer &= ~bit;
		grer |= bit;
		break;
	case IST_EDGE_BOTH:
		gfer |= bit;
		grer |= bit;
		break;
	default:
		panic("ixp425_gpio_set_intr_level: bad level: %d", level);
		break;
	}

	ixpgpio_reg_write(sc, GPIO_REG(GPIO_GFER0, gpio), gfer);
	ixpgpio_reg_write(sc, GPIO_REG(GPIO_GRER0, gpio), grer);

	splx(s);
}
#endif

#if NGPIO > 0
/* GPIO support functions */
static int
ixp425_gpio_pin_read(void *arg, int pin)
{
	struct ixpgpio_softc *sc = ixpgpio_softc;
	int reg;

	reg = ixpgpio_reg_read(sc, GPIO_GPINR);
	if (reg & (1 << pin))
		return 1;
	else
		return 0;
}

static void
ixp425_gpio_pin_write(void *arg, int pin, int value)
{
	struct ixpgpio_softc *sc = ixpgpio_softc;
	int reg;

	reg = ixpgpio_reg_read(sc, GPIO_GPOUTR);
	if (value)
		reg |= (1 << pin);
	else
		reg &= ~(1 << pin);
	ixpgpio_reg_write(sc, GPIO_GPOUTR, reg);
}

static void
ixp425_gpio_pin_ctl(void *arg, int pin, int flags)
{
	struct ixpgpio_softc *sc = ixpgpio_softc;
	int reg;

	reg = ixpgpio_reg_read(sc, GPIO_GPOER);
	if (flags & GPIO_PIN_OUTPUT)
		reg |= (1 << pin);
	else if (flags & GPIO_PIN_INPUT)
		reg &= ~(1 << pin);
	ixpgpio_reg_write(sc, GPIO_GPOER, reg);
}
#endif

#if 0
/*
 * Configurations of GPIO for PXA25x
 */
#if 0
struct ixp425_gpioconf ixp425_com_btuart_gpioconf[] = {
	{ 42, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* BTRXD */
	{ 43, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* BTTXD */

#if 0	/* optional */
	{ 44, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* BTCTS */
	{ 45, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* BTRTS */
#endif

	{ -1 }
};

struct ixp425_gpioconf ixp425_com_ffuart_gpioconf[] = {
	{ 34, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* FFRXD */

#if 0	/* optional */
	{ 35, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* CTS */
	{ 36, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* DCD */
	{ 37, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* DSR */
	{ 38, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* RI */
#endif

	{ 39, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* FFTXD */

#if 0	/* optional */
	{ 40, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* DTR */
	{ 41, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* RTS */
#endif

	{ -1 }
};

struct ixp425_gpioconf ixp425_com_hwuart_gpioconf[] = {
#if 0	/* We can select and/or. */
	{ 42, GPIO_CLR | GPIO_ALT_FN_3_IN },	/* HWRXD */
	{ 49, GPIO_CLR | GPIO_ALT_FN_2_IN },	/* HWRXD */

	{ 43, GPIO_CLR | GPIO_ALT_FN_3_OUT },	/* HWTXD */
	{ 48, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* HWTXD */

#if 0	/* optional */
	{ 44, GPIO_CLR | GPIO_ALT_FN_3_IN },	/* HWCST */
	{ 51, GPIO_CLR | GPIO_ALT_FN_3_IN },	/* HWCST */

	{ 45, GPIO_CLR | GPIO_ALT_FN_3_OUT },	/* HWRST */
	{ 52, GPIO_CLR | GPIO_ALT_FN_3_OUT },	/* HWRST */
#endif
#endif

	{ -1 }
};

struct ixp425_gpioconf ixp425_com_stuart_gpioconf[] = {
	{ 46, GPIO_CLR | GPIO_ALT_FN_2_IN },	/* RXD */
	{ 47, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* TXD */
	{ -1 }
};

struct ixp425_gpioconf ixp425_i2c_gpioconf[] = {
	{ -1 }
};

struct ixp425_gpioconf ixp425_i2s_gpioconf[] = {
	{ 28, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* BITCLK */
	{ 29, GPIO_CLR | GPIO_ALT_FN_2_IN },	/* SDATA_IN */
	{ 30, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* SDATA_OUT */
	{ 31, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* SYNC */
	{ 32, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* SYSCLK */
	{ -1 }
};

struct ixp425_gpioconf ixp425_pcic_gpioconf[] = {
	{ 48, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* nPOE */
	{ 49, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* nPWE */
	{ 50, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* nPIOR */
	{ 51, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* nPIOW */

#if 0	/* We can select and/or. */
	{ 52, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* nPCE1 */
	{ 53, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* nPCE2 */
#endif

	{ 54, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* pSKTSEL */
	{ 55, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* nPREG */
	{ 56, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* nPWAIT */
	{ 57, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* nIOIS16 */
	{ -1 }
};

struct ixp425_gpioconf ixp425_ixpacu_gpioconf[] = {
	{ 28, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* BITCLK */
	{ 30, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* SDATA_OUT */
	{ 31, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* SYNC */

#if 0	/* We can select and/or. */
	{ 29, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* SDATA_IN0 */
	{ 32, GPIO_CLR | GPIO_ALT_FN_1_IN },	/* SDATA_IN1 */
#endif

	{ -1 }
};

struct ixp425_gpioconf ixp425_ixpmci_gpioconf[] = {
#if 0	/* We can select and/or. */
	{  6, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCLK */
	{ 53, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCLK */
	{ 54, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCLK */

	{  8, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCS0 */
	{ 34, GPIO_CLR | GPIO_ALT_FN_2_OUT },	/* MMCCS0 */
	{ 67, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCS0 */

	{  9, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCS1 */
	{ 39, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCS1 */
	{ 68, GPIO_CLR | GPIO_ALT_FN_1_OUT },	/* MMCCS1 */
#endif

	{  -1 }
};
#endif

void
ixp425_gpio_config(struct ixp425_gpioconf **conflist)
{
	int i, j;

	for (i = 0; conflist[i] != NULL; i++)
		for (j = 0; conflist[i][j].pin != -1; j++)
			ixp425_gpio_set_function(conflist[i][j].pin,
			    conflist[i][j].value);
}
#endif
