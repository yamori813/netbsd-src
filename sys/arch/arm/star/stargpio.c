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

//#define DEBUG_STARGPIO

#include <sys/param.h>
#include <sys/device.h>
#include <sys/errno.h>

#include <sys/gpio.h>
#include <dev/gpio/gpiovar.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

struct stargpio_softc {
	device_t sc_dev;
	struct gpio_chipset_tag sc_gpio_gc;
	gpio_pin_t sc_gpio_pins[STAR_GPIO_PINS];

	uint32_t sc_pin_shared;
	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	void *sc_ih;
};

static int stargpio_match(device_t, struct cfdata *, void *);
static void stargpio_attach(device_t, device_t, void *);
static void stargpio_init(struct stargpio_softc *);
static int stargpio_intr(void *);

static int stargpio_pin_read(void *, int);
static void stargpio_pin_write(void *, int, int);
static void stargpio_pin_ctl(void *, int, int);
static int stargpio_pin_getflags(struct stargpio_softc *, int);

CFATTACH_DECL_NEW(stargpio, sizeof(struct stargpio_softc),
    stargpio_match, stargpio_attach, NULL, NULL);


/* ARGSUSED */
static int
stargpio_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa;

	sa = aux;
	sa->sa_size = STAR_GPIO_REGSIZE;

	switch (sa->sa_addr) {
	case STRx100_GPIOA_REGISTER:
		break;
	case STR8100_GPIOB_REGISTER:
		if (CPU_IS_STR9100())
			return 0;
		break;
	default:
		/* unknown GPIO address */
		return 0;
	}

	return 1;
}

/* ARGSUSED */
static void
stargpio_attach(device_t parent __unused, device_t self, void *aux)
{
	struct stargpio_softc *sc;
	struct star_attach_args *sa;
	struct gpiobus_attach_args gba;
	int pin;

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_addr = sa->sa_addr;

	sc->sc_pin_shared = 0;
	switch (sc->sc_addr) {
	case STRx100_GPIOA_REGISTER:
		/* some pins are shared by other functions */

		if (CPU_IS_STR8100()) {
			aprint_normal(": GPIOA\n");

			/* XXX: pin shared design is dependend by archtecture */
			sc->sc_pin_shared =
			    EQUULEUS_MISC_GPIOA31_SPICS3 |
			    EQUULEUS_MISC_GPIOA30_SPICS2 |
			    EQUULEUS_MISC_GPIOA29_SPICS1 |
			    EQUULEUS_MISC_GPIOA28_SPICS0 |
			    EQUULEUS_MISC_GPIOA27_SPICLK |
			    EQUULEUS_MISC_GPIOA26_SPIDR |
			    EQUULEUS_MISC_GPIOA25_WDTRST |
			    EQUULEUS_MISC_GPIOA24_PHYLED2 |
			    EQUULEUS_MISC_GPIOA23_PHYLED1 |
			    EQUULEUS_MISC_GPIOA22_PHYLED0 |
			    EQUULEUS_MISC_GPIOA21_PCMCLK |
			    EQUULEUS_MISC_GPIOA20_PCMFS |
			    EQUULEUS_MISC_GPIOA19_PCMDT |
			    EQUULEUS_MISC_GPIOA18_PCMDR |
			    EQUULEUS_MISC_GPIOA17_I2SCLK |
			    EQUULEUS_MISC_GPIOA16_I2SWS |
			    EQUULEUS_MISC_GPIOA15_I2SSD |
			    EQUULEUS_MISC_GPIOA14_TWISCL |
			    EQUULEUS_MISC_GPIOA13_TWISDA |
			    EQUULEUS_MISC_GPIOA3_UART_ACT1 |
			    EQUULEUS_MISC_GPIOA2_UART_ACT0 |
			    EQUULEUS_MISC_GPIOA1_EXT_INT30 |
			    EQUULEUS_MISC_GPIOA0_EXT_INT29;
		} else {
			/* STR9100 GPIO */
			aprint_normal(": GPIO\n");

			/* XXX: pin shared design is dependend by archtecture */
			sc->sc_pin_shared =
			    (1 << 0) |	/* shared pin ICE ICK/TCK */
			    (1 << 1) |	/* shared pin ICE IMS/TMS */
			    (1 << 2) |	/* shared pin ICE IDIO/TDO */
			    (1 << 3) |	/* shared pin ICE EXTGOICE/DBGRO */
			    (1 << 4) |	/* shared pin UART UR-TXD */
			    (1 << 5) |	/* shared pin UART UR-RXD */
			    (1 << 6) |	/* shared pin UART UR-CTS */
			    (1 << 7) |	/* shared pin UART UR-RTS  */
			    (1 << 8) |	/* shared pin MII0 P0-CRS */
			    (1 << 9) |	/* shared pin MII0 P0-COL */
			    (1 << 10) |	/* shared pin MII1 P1-CRS */
			    (1 << 11) |	/* shared pin MII1 P1-COL */
			    (1 << 12) |	/* shared pin PCMCIA CE1n */
			    (1 << 13) |	/* shared pin PCMCIA CE2n */
			    (1 << 14) |	/* shared pin PCMCIA IREQn */
			    (1 << 15) |	/* shared pin PCMCIA RESET */
			    (1 << 16) |	/* shared pin PCMCIA WAITn */
			    (1 << 17) |	/* shared pin PCMCIA INPACKn */
			    (1 << 18) |	/* shared pin ICE nTRST */
			    (1 << 19) |	/* shared pin ICE TDI */
			    (1 << 20);	/* shared pin ICE DBGACK */
		}
		break;
	case STR8100_GPIOB_REGISTER:
		aprint_normal(": GPIOB\n");

		/* XXX: pin shared design is dependend by archtecture */
		sc->sc_pin_shared =
		    EQUULEUS_MISC_GPIOB31_PCIGNT1 |
		    EQUULEUS_MISC_GPIOB30_PCIREG1 |
		    EQUULEUS_MISC_GPIOB29_PCIGNT0 |
		    EQUULEUS_MISC_GPIOB28_PCIREQ0 |
		    EQUULEUS_MISC_GPIOB27_PCISERR |
		    EQUULEUS_MISC_GPIOB26_PCIPERR |
		    EQUULEUS_MISC_GPIOB25_PCISTOP |
		    EQUULEUS_MISC_GPIOB24_PCIDEVSEL |
		    EQUULEUS_MISC_GPIOB23_PCITRDY |
		    EQUULEUS_MISC_GPIOB22_UART_RXD1 |
		    EQUULEUS_MISC_GPIOB21_UART_TXD1 |
		    EQUULEUS_MISC_GPIOB20_PWAIT |
		    EQUULEUS_MISC_GPIOB19_REGn |
		    EQUULEUS_MISC_GPIOB18_CE2 |
		    EQUULEUS_MISC_GPIOB17_CE1 |
		    EQUULEUS_MISC_GPIOB16_SRAM_WAIT3 |
		    EQUULEUS_MISC_GPIOB15_SRAM_WAIT2 |
		    EQUULEUS_MISC_GPIOB14_SRAM_WAIT1 |
		    EQUULEUS_MISC_GPIOB13_SRAM_CE3 |
		    EQUULEUS_MISC_GPIOB12_SRAM_CE2 |
		    EQUULEUS_MISC_GPIOB11_SRAM_CE1 |
		    EQUULEUS_MISC_GPIOB10_IDEDIOW |
		    EQUULEUS_MISC_GPIOB9_IDEDIOR |
		    EQUULEUS_MISC_GPIOB8_IDECS1 |
		    EQUULEUS_MISC_GPIOB7_IDECS0 |
		    EQUULEUS_MISC_GPIOB6_IDEINTRQ |
		    EQUULEUS_MISC_GPIOB5_IDEDMACK |
		    EQUULEUS_MISC_GPIOB4_IDEDMARQ |
		    EQUULEUS_MISC_GPIOB3_IDEIORDY |
		    EQUULEUS_MISC_GPIOB2_MIICOL |
		    EQUULEUS_MISC_GPIOB1_PHYMDIO |
		    EQUULEUS_MISC_GPIOB0_PHYMDC;
		break;
	}
	aprint_naive("\n");

	if (bus_space_map(sc->sc_iot, sc->sc_addr, sa->sa_size, 0,
	    &sc->sc_ioh)) {
		aprint_error_dev(self, "can't map registers\n");
		return;
	}

	stargpio_init(sc);

	sc->sc_ih = intr_establish(STAR_IRQ_GPIO, IPL_BIO,
	    STAR_INTR_RISING_EDGE, stargpio_intr, sc);

	/*
	 * for gpiobus
	 */
	sc->sc_gpio_gc.gp_cookie = sc;
	sc->sc_gpio_gc.gp_gc_open = NULL;
	sc->sc_gpio_gc.gp_gc_close = NULL;
	sc->sc_gpio_gc.gp_pin_read = stargpio_pin_read;
	sc->sc_gpio_gc.gp_pin_write = stargpio_pin_write;
	sc->sc_gpio_gc.gp_pin_ctl = stargpio_pin_ctl;

	for (pin = 0; pin < STAR_GPIO_PINS; pin++) {
		sc->sc_gpio_pins[pin].pin_num = pin;
		if (sc->sc_pin_shared & (1 << pin)) {
			/* this pin is used by other function */
			sc->sc_gpio_pins[pin].pin_caps = 0;
			sc->sc_gpio_pins[pin].pin_flags = 0;
			sc->sc_gpio_pins[pin].pin_state = 0;
		} else {
			sc->sc_gpio_pins[pin].pin_caps =
			    GPIO_PIN_INPUT | GPIO_PIN_OUTPUT;
			sc->sc_gpio_pins[pin].pin_flags =
			    stargpio_pin_getflags(sc, pin);
			sc->sc_gpio_pins[pin].pin_state =
			    stargpio_pin_read(sc, pin);

#if 0
			printf("%s: pin%d: %s%s\n", device_xname(sc->sc_dev),
			    pin,
			    (sc->sc_gpio_pins[pin].pin_flags & GPIO_PIN_INPUT) ?
			    "input" : "",
			    (sc->sc_gpio_pins[pin].pin_flags & GPIO_PIN_OUTPUT) ?
			    "output" : "");
#endif
		}
	}
	gba.gba_gc = &sc->sc_gpio_gc;
	gba.gba_pins = sc->sc_gpio_pins;
	gba.gba_npins = STAR_GPIO_PINS;

//	config_found_ia(self, "gpiobus", &gba, gpiobus_print);
	config_found(self, &gba, gpiobus_print, CFARGS_NONE);
}

static void
stargpio_init(struct stargpio_softc *sc)
{
	uint32_t val;

	if (CPU_IS_STR8100()) {
		/* enable GPIO clock */
		val = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKGATE1_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKGATE1_REG, val |
		    EQUULEUS_CLKPWR_CLKGATE1_PCLK_GPIO);

		/* activate GPIO */
		val = STAR_REG_READ32(EQUULEUS_CLKPWR_SOFTRST_REG);
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_SOFTRST_REG, val |
		    EQUULEUS_CLKPWR_SOFTRST_GPIO);
	} else {
		/* no need to activate PCLK on STR9100 */
	}

#if 0
	/* disable interrupt */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    STAR_GPIO_INT_ENABLE_REG, 0);
#else
	/* enable interrupt */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    STAR_GPIO_INT_ENABLE_REG, 0xffffffff);
#endif
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    STAR_GPIO_INT_MASK_REG, 0);
}


static int
stargpio_intr(void *arg)
{
	struct stargpio_softc *sc;
	uint32_t status;

	sc = (struct stargpio_softc *)arg;

	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    STAR_GPIO_INT_MASKSTAT_REG);

#ifdef DEBUG_STARGPIO
	printf("%s: interrupt: status=0x%08x\n", __func__, status);
#endif

	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    STAR_GPIO_INT_CLEAR_REG, status);

	return 1;
}


static int
stargpio_pin_read(void *arg, int pin)
{
	struct stargpio_softc *sc;
	uint32_t val;

	sc = (struct stargpio_softc *)arg;

	if (sc->sc_pin_shared & (1 << pin))
		return GPIO_PIN_LOW;

	val = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    STAR_GPIO_DATA_INPUT_REG);

	return (val & (1 << pin)) ? GPIO_PIN_HIGH : GPIO_PIN_LOW;
}

static void
stargpio_pin_write(void *arg, int pin, int value)
{
	struct stargpio_softc *sc;
	uint32_t pinbit;

	sc = (struct stargpio_softc *)arg;

	if (sc->sc_pin_shared & (1 << pin))
		return;

	pinbit = 1 << pin;
	if (value)
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    STAR_GPIO_DATA_SET_REG, pinbit);
	else
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    STAR_GPIO_DATA_CLEAR_REG, pinbit);
}

static int
stargpio_pin_getflags(struct stargpio_softc *sc, int pin)
{
	uint32_t pinbit, val;

	pinbit = 1 << pin;
	val = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    STAR_GPIO_PINDIR_REG);
	if (val & pinbit)
		return GPIO_PIN_OUTPUT;
	else
		return GPIO_PIN_INPUT;
}

static void
stargpio_pin_ctl(void *arg, int pin, int flags)
{
	struct stargpio_softc *sc;
	uint32_t pinbit, val;

	sc = (struct stargpio_softc *)arg;
	pinbit = 1 << pin;

	if (sc->sc_pin_shared & pinbit)
		return;

	if (flags & GPIO_PIN_INPUT) {
		val = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    STAR_GPIO_PINDIR_REG);
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    STAR_GPIO_PINDIR_REG, val & ~pinbit);
	}
	if (flags & GPIO_PIN_OUTPUT) {
		val = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    STAR_GPIO_PINDIR_REG);
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    STAR_GPIO_PINDIR_REG, val | pinbit);
	}
}
