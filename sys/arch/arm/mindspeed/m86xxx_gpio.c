/*	$NetBSD$	*/

/*-
 * Copyright (c) 2007 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas
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
#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#define _INTR_PRIVATE

#include "locators.h"
#include "gpio.h"
 
#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/errno.h>
#include <sys/systm.h>

#include <uvm/uvm_extern.h>
  
#include <machine/intr.h>
 
#include <arm/cpu.h>
#include <arm/armreg.h>
#include <arm/cpufunc.h>

#include <arm/mindspeed/m86xxx_reg.h>
#include <arm/mindspeed/m86xxx_var.h>

#include <arm/pic/picvar.h>

#if NGPIO > 0
#include <sys/gpio.h>
#include <dev/gpio/gpiovar.h>
#endif

#define GPIN(n)			(1 << n)
#define RESET_BIT		17

static void gpio_pic_block_irqs(struct pic_softc *, size_t, uint32_t);
static void gpio_pic_unblock_irqs(struct pic_softc *, size_t, uint32_t);
static int gpio_pic_find_pending_irqs(struct pic_softc *);
static void gpio_pic_establish_irq(struct pic_softc *, struct intrsource *);

const struct pic_ops gpio_pic_ops = {
	.pic_block_irqs = gpio_pic_block_irqs,
	.pic_unblock_irqs = gpio_pic_unblock_irqs,
	.pic_find_pending_irqs = gpio_pic_find_pending_irqs,
	.pic_establish_irq = gpio_pic_establish_irq,
};

struct gpio_softc {
	device_t gpio_dev;
	struct pic_softc gpio_pic;
	struct intrsource *gpio_is;
	bus_space_tag_t gpio_memt;
	bus_space_handle_t gpio_memh;
	uint32_t gpio_enable_mask;
	uint32_t gpio_edge_mask;
	uint32_t gpio_edge_falling_mask;
	uint32_t gpio_edge_rising_mask;
	uint32_t gpio_level_mask;
	uint32_t gpio_level_hi_mask;
	uint32_t gpio_level_lo_mask;
	uint32_t gpio_inuse_mask;
#if NGPIO > 0
	struct gpio_chipset_tag gpio_chipset;
	gpio_pin_t gpio_pins[64];
#endif
};

#define	PIC_TO_SOFTC(pic) \
	((struct gpio_softc *)((char *)(pic) - \
		offsetof(struct gpio_softc, gpio_pic)))

#define	GPIO_READ(gpio, reg) \
	bus_space_read_4((gpio)->gpio_memt, (gpio)->gpio_memh, (reg))
#define	GPIO_WRITE(gpio, reg, val) \
	bus_space_write_4((gpio)->gpio_memt, (gpio)->gpio_memh, (reg), (val))

void
gpio_pic_unblock_irqs(struct pic_softc *pic, size_t irq_base, uint32_t irq_mask)
{
#if 0
	struct gpio_softc * const gpio = PIC_TO_SOFTC(pic);
	KASSERT(irq_base == 0);

	gpio->gpio_enable_mask |= irq_mask;
	/*
	 * If this a level source, ack it now.  If it's still asserted
	 * it'll come back.
	 */
	GPIO_WRITE(gpio, GEMINI_GPIO_INTRENB, gpio->gpio_enable_mask);
	if (irq_mask & gpio->gpio_level_mask)
		GPIO_WRITE(gpio, GEMINI_GPIO_INTRCLR,
		    irq_mask & gpio->gpio_level_mask);
#endif
}

void
gpio_pic_block_irqs(struct pic_softc *pic, size_t irq_base, uint32_t irq_mask)
{
#if 0
	struct gpio_softc * const gpio = PIC_TO_SOFTC(pic);
	KASSERT(irq_base == 0);

	gpio->gpio_enable_mask &= ~irq_mask;
	GPIO_WRITE(gpio, GEMINI_GPIO_INTRENB, ~irq_mask);
	/*
	 * If any of the sources are edge triggered, ack them now so
	 * we won't lose them.
	 */
	if (irq_mask & gpio->gpio_edge_mask)
		GPIO_WRITE(gpio, GEMINI_GPIO_INTRCLR,
		    irq_mask & gpio->gpio_edge_mask);
#endif
}

int
gpio_pic_find_pending_irqs(struct pic_softc *pic)
{
#if 0
	struct gpio_softc * const gpio = PIC_TO_SOFTC(pic);
	uint32_t pending;

	pending = GPIO_READ(gpio, GEMINI_GPIO_INTRMSKSTATE);
	KASSERT((pending & ~gpio->gpio_enable_mask) == 0);
	if (pending == 0)
		return 0;

	/*
	 * Now find all the pending bits and mark them as pending.
	 */
	(void) pic_mark_pending_sources(&gpio->gpio_pic, 0, pending);
#endif

	return 1;
}

void
gpio_pic_establish_irq(struct pic_softc *pic, struct intrsource *is)
{
#if 0
	struct gpio_softc * const gpio = PIC_TO_SOFTC(pic);
	KASSERT(is->is_irq < 32);
	uint32_t irq_mask = __BIT(is->is_irq);
	uint32_t v;
#if 0
	unsigned int i;
	struct intrsource *maybe_is;
#endif

	/*
	 * Make sure the irq isn't enabled and not asserting.
	 */
	gpio->gpio_enable_mask &= ~irq_mask;
	GPIO_WRITE(gpio, GEMINI_GPIO_INTRENB, gpio->gpio_enable_mask);
	GPIO_WRITE(gpio, GEMINI_GPIO_INTRCLR, irq_mask);

	/*
	 * Convert the type to a gpio type and figure out which bits in what 
	 * register we have to tweak.
	 */
	gpio->gpio_edge_rising_mask &= ~irq_mask;
	gpio->gpio_edge_falling_mask &= ~irq_mask;
	gpio->gpio_level_hi_mask &= ~irq_mask;
	gpio->gpio_level_lo_mask &= ~irq_mask;
	switch (is->is_type) {
	case IST_LEVEL_LOW: gpio->gpio_level_lo_mask |= irq_mask; break;
	case IST_LEVEL_HIGH: gpio->gpio_level_hi_mask |= irq_mask; break;
	case IST_EDGE_FALLING: gpio->gpio_edge_falling_mask |= irq_mask; break;
	case IST_EDGE_RISING: gpio->gpio_edge_rising_mask |= irq_mask; break;
	case IST_EDGE_BOTH:
		gpio->gpio_edge_rising_mask |= irq_mask;
		gpio->gpio_edge_falling_mask |= irq_mask;
		break;
	default:
		panic("%s: unknown is_type %d\n", __FUNCTION__, is->is_type);
	}
	gpio->gpio_edge_mask =
	    gpio->gpio_edge_rising_mask | gpio->gpio_edge_falling_mask;
	gpio->gpio_level_mask =
	    gpio->gpio_level_hi_mask|gpio->gpio_level_lo_mask;
	gpio->gpio_inuse_mask |= irq_mask;

	/*
	 * Set the interrupt type.
	 */
	GPIO_WRITE(gpio, GEMINI_GPIO_INTRTRIG, gpio->gpio_level_mask);
	GPIO_WRITE(gpio, GEMINI_GPIO_INTREDGEBOTH,
		gpio->gpio_edge_rising_mask & gpio->gpio_edge_falling_mask);
	GPIO_WRITE(gpio, GEMINI_GPIO_INTRDIR,
		gpio->gpio_edge_falling_mask | gpio->gpio_level_lo_mask);

	/*
	 * Mark it as input by clearning bit(s) in PINDIR reg
	 */
	v = GPIO_READ(gpio, GEMINI_GPIO_PINDIR);
	v &= ~irq_mask;
	GPIO_WRITE(gpio, GEMINI_GPIO_PINDIR, v); 
#if 0
	for (i = 0, maybe_is = NULL; i < 32; i++) {
		if ((is = pic->pic_sources[i]) != NULL) {
			if (maybe_is == NULL || is->is_ipl > maybe_is->is_ipl)
				maybe_is = is;
		}
	}
	if (maybe_is != NULL) {
		is = gpio->gpio_is;
		KASSERT(is != NULL);
		is->is_ipl = maybe_is->is_ipl;
		(*is->is_pic->pic_ops->pic_establish_irq)(is->is_pic, is);
	} 
#endif
#endif
}

static int gpio_match(device_t, cfdata_t, void *);
static void gpio_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(m86gpio,
	sizeof(struct gpio_softc),
	gpio_match, gpio_attach,
	NULL, NULL);

#if NGPIO > 0 || NGEMINIGMAC > 0

int m86gpio_pin_read(void *arg, int pin);
void m86gpio_pin_write(void *arg, int pin, int value);
void m86gpio_pin_ctl(void *arg, int pin, int flags);

int
m86gpio_pin_read(void *arg, int pin)
{
	struct gpio_softc * const gpio = device_private(arg);

	if (pin < 32) {

		return (GPIO_READ(gpio, GPIO_INPUT_REG) >> pin) & 1;
	} else {

		return (GPIO_READ(gpio, GPIO_63_32_INPUT_REG) >> (pin - 32))
		    & 1;
	}
}

void
m86gpio_pin_write(void *arg, int pin, int value)
{
	struct gpio_softc * const gpio = device_private(arg);
	uint32_t mask;
	uint32_t old, new;

	if (pin < 32) {
		mask = 1 << pin;
		old = GPIO_READ(gpio, GPIO_OUTPUT_REG);
		new = old;
		if (value)
			new |= mask;
		else
			new &= ~mask;
		GPIO_WRITE(gpio, GPIO_OUTPUT_REG, new);
	} else {
		mask = 1 << (pin - 32);
		old = GPIO_READ(gpio, GPIO_63_32_OUTPUT_REG);
		new = old;
		if (value)
			new |= mask;
		else
			new &= ~mask;
		GPIO_WRITE(gpio, GPIO_63_32_OUTPUT_REG, new);
	}
}

void
m86gpio_pin_ctl(void *arg, int pin, int flags)
{
	struct gpio_softc * const gpio = device_private(arg);
	uint32_t mask;
	uint32_t old, new;

	if (pin < 32) {
		mask = 1 << pin;
		old = GPIO_READ(gpio, GPIO_OE_REG);
	} else {
		mask = 1 << (pin - 32);
		old = GPIO_READ(gpio, GPIO_63_32_OE_REG);
	}
	new = old;
	switch (flags & (GPIO_PIN_INPUT|GPIO_PIN_OUTPUT)) {
	case GPIO_PIN_INPUT:	new &= ~mask; break;
	case GPIO_PIN_OUTPUT:	new |=  mask; break;
	default:		return;
	}
	if (old != new) {
		if (pin < 32)
			GPIO_WRITE(gpio, GPIO_OE_REG, new);
		else
			GPIO_WRITE(gpio, GPIO_63_32_OE_REG, new);
	}
}

static void
gpio_defer(device_t self)
{
	struct gpio_softc * const gpio = device_private(self);
	struct gpio_chipset_tag * const gp = &gpio->gpio_chipset;
	struct gpiobus_attach_args gba;
/*
	gpio_pin_t *pins;
	uint32_t mask, dir, valueout, valuein;
	int pin;
*/

	gp->gp_cookie = gpio->gpio_dev;
	gp->gp_pin_read = m86gpio_pin_read;
	gp->gp_pin_write = m86gpio_pin_write;
	gp->gp_pin_ctl = m86gpio_pin_ctl;

	gba.gba_gc = gp;
	gba.gba_pins = gpio->gpio_pins;
	gba.gba_npins = __arraycount(gpio->gpio_pins);

#if 0
	valuein = GPIO_READ(gpio, GPIO_INPUT_REG);
	valueout = GPIO_READ(gpio, GPIO_OUTPUT_REG);
	dir = GPIO_READ(gpio, GPIO_OE_REG);
	for (pin = 0, mask = 1, pins = gpio->gpio_pins;
	     pin < 32; pin++, mask <<= 1, pins++) {
		pins->pin_num = pin;
		if (dir & (1 << pin)) {
			pins->pin_caps = GPIO_PIN_OUTPUT;
			pins->pin_flags = GPIO_PIN_OUTPUT;
			pins->pin_state = (valueout & (1 << pin)) ? 1 : 0;
		} else {
			pins->pin_caps = GPIO_PIN_INPUT;
			pins->pin_flags = GPIO_PIN_INPUT;
			pins->pin_state = (valuein & (1 << pin)) ? 1 : 0;
		}
	}
	dir = GPIO_READ(gpio, GEMINI_GPIO_PINDIR);
	valueout = GPIO_READ(gpio, GEMINI_GPIO_DATAOUT);
	valuein = GPIO_READ(gpio, GEMINI_GPIO_DATAIN);
	for (pin = 0, mask = 1, pins = gpio->gpio_pins;
	     pin < 32; pin++, mask <<= 1, pins++) {
		pins->pin_num = pin;
		if (gpio->gpio_inuse_mask & mask)
			pins->pin_caps = GPIO_PIN_INPUT;
		else
			pins->pin_caps = GPIO_PIN_INPUT|GPIO_PIN_OUTPUT;
		pins->pin_flags =
		    (dir & mask) ? GPIO_PIN_OUTPUT : GPIO_PIN_INPUT;
		pins->pin_state =
		    (((dir & mask) ? valueout : valuein) & mask)
			? GPIO_PIN_HIGH
			: GPIO_PIN_LOW;
	}

#endif
	config_found(self, &gba, gpiobus_print, CFARGS_NONE);
}
#endif /* NGPIO > 0 */

int
gpio_match(device_t parent, cfdata_t cfdata, void *aux)
{
	struct axi_attach_args *aa = aux;

	if (aa->aa_addr == GPIO_BASE + AXI_APB_CFG_BASE)
		return 1;

	return 0;
}

void
gpio_attach(device_t parent, device_t self, void *aux)
{
	struct axi_attach_args * const aa = aux;
	struct gpio_softc * const gpio = device_private(self);
	int error;
//	int oen, i;
//	int oepins[] = {16, 18, 19, 20, 21, 22, 23, 28};

/*
	if (aa->apba_intr == OBIOCF_INTR_DEFAULT)
		panic("\n%s: no intr assigned", device_xname(self));

*/
	if (aa->aa_size != 0x10000)	/* APB is 64K boundly */
		aa->aa_size = 0x10000;

	gpio->gpio_dev = self;
	gpio->gpio_memt = aa->aa_iot;
	error = bus_space_map(aa->aa_iot, aa->aa_addr, aa->aa_size,
	    0, &gpio->gpio_memh);

	if (error) {
		aprint_error(": failed to map register %#lx@%#lx: %d\n",
		    aa->aa_size, aa->aa_addr, error);
		return;
	}
/*
GPIO_6 SW
GPIO_27 HW RESET
*/

#if 0
	uint32_t i, j, val;
	for (i = 0; i < 32; ++i) {
		printf("MORIMORI GPIO %d\n", i);
		val = ~(1 << i);
		GPIO_WRITE(gpio, GPIO_OE_REG, val);
		GPIO_WRITE(gpio, GPIO_OUTPUT_REG, val);
		/* 5 sec is overflow at delay() */
		for (j = 0;j < 1000; ++j)
			delay(1000*5);
	}

	uint32_t reg;
	/* QCA8337 Reset is GPIO_5. Same as reference design */
	reg = GPIO_READ(gpio, GPIO_OUTPUT_REG);
	reg = reg & ~(1 << 5);
	GPIO_WRITE(gpio, GPIO_OUTPUT_REG, reg);
	reg = GPIO_READ(gpio, GPIO_OE_REG);
	reg = reg | (1 << 5);
	GPIO_WRITE(gpio, GPIO_OE_REG, reg);
	reg = GPIO_READ(gpio, GPIO_OUTPUT_REG);
	reg = reg | (1 << 5);
	GPIO_WRITE(gpio, GPIO_OUTPUT_REG, reg);
#endif

#if 0
	GPIO_WRITE(gpio, GPIO_LOCK_REG, 0x55555555);
	GPIO_WRITE(gpio, GPIO_IOCTRL_REG, 0x00000080);

	GPIO_WRITE(gpio, GPIO_PIN_SELECT_REG, GSEL);

	oen = 0;
	for (i = 0; i < sizeof(oepins) / 4; ++i) {
		oen |= (1 << oepins[i]);
	}
        GPIO_WRITE(gpio, GPIO_OE_REG, 
	    GPIO_READ(gpio, GPIO_OE_REG) | oen);

	/* make Status LED is Green */
	GPIO_WRITE(gpio, GPIO_OUTPUT_REG, GPIN(RESET_BIT) | GPIN(21));
	/* do reset */
/*
        GPIO_WRITE(gpio, GPIO_OUTPUT_REG,
	    GPIO_READ(gpio, GPIO_OUTPUT_REG) & ~GPIN(RESET_BIT));
*/
	/* USB Power ON */
        GPIO_WRITE(gpio, GPIO_OUTPUT_REG,
	    GPIO_READ(gpio, GPIO_OUTPUT_REG) | GPIN(16));
#if 0
	if (oa->obio_intrbase != OBIOCF_INTRBASE_DEFAULT) {
		gpio->gpio_pic.pic_ops = &gpio_pic_ops;
		strlcpy(gpio->gpio_pic.pic_name, device_xname(self),
		    sizeof(gpio->gpio_pic.pic_name));
		gpio->gpio_pic.pic_maxsources = 32;
		pic_add(&gpio->gpio_pic, oa->obio_intrbase);
		aprint_normal(": interrupts %d..%d",
		    oa->obio_intrbase, oa->obio_intrbase + 31);
		gpio->gpio_is = intr_establish(oa->obio_intr, 
		    IPL_HIGH, IST_LEVEL_HIGH, pic_handle_intr, &gpio->gpio_pic);
		KASSERT(gpio->gpio_is != NULL);
		aprint_normal(", intr %d", oa->obio_intr);
	}
#endif
#endif
	aprint_normal("\n");
#if NGPIO > 0
	config_interrupts(self, gpio_defer);
#endif
}
