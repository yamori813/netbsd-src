/*	$NetBSD: rt1310_intr.c,v 1.9 2014/02/22 16:14:38 martin Exp $	*/
/*
 * Copyright (c) 2010 KIYOHARA Takashi
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
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: rt1310_intr.c,v 1.9 2014/02/22 16:14:38 martin Exp $");

//#include "opt_rt1310.h"

#define _INTR_PRIVATE

#include <sys/param.h>
#include <sys/evcnt.h>
#include <sys/device.h>
#include <sys/atomic.h>

#include <uvm/uvm_extern.h>

#include <machine/intr.h>

#include <arm/armreg.h>
#include <arm/cpu.h>
#include <arm/pic/picvar.h>

#include <machine/autoconf.h>
#include <sys/bus.h>

#include <arm/ralink/rt1310_reg.h>
#include <arm/ralink/rt1310_var.h>

extern struct cfdriver intc_cd;

static void rt1310_pic_unblock_irqs(struct pic_softc *, size_t, uint32_t);
static void rt1310_pic_block_irqs(struct pic_softc *, size_t, uint32_t);
static void rt1310_pic_establish_irq(struct pic_softc *,
					   struct intrsource *);
static void rt1310_pic_source_name(struct pic_softc *, int, char *,
					 size_t);

#define	INTC_NIRQS	32

struct rt1310_irqdef {
	u_int                   ri_trig;
	u_int                   ri_prio;
};

struct rt1310_irqdef irqdef[INTC_NIRQS] = {
	{RT_INTC_TRIG_HIGH_LVL, 2},	/* 0 */
	{RT_INTC_TRIG_HIGH_LVL, 2},
	{RT_INTC_TRIG_HIGH_LVL, 2},
	{RT_INTC_TRIG_HIGH_LVL, 1},
	{RT_INTC_TRIG_HIGH_LVL, 2},
	{RT_INTC_TRIG_HIGH_LVL, 1},
	{RT_INTC_TRIG_HIGH_LVL, 1},
	{RT_INTC_TRIG_HIGH_LVL, 1},
	{RT_INTC_TRIG_HIGH_LVL, 1},	/* 8 */
	{RT_INTC_TRIG_HIGH_LVL, 1},
	{RT_INTC_TRIG_HIGH_LVL, 2},
	{RT_INTC_TRIG_LOW_LVL, 2},
	{RT_INTC_TRIG_LOW_LVL, 2},
	{RT_INTC_TRIG_LOW_LVL, 4},
	{RT_INTC_TRIG_HIGH_LVL, 2},
	{RT_INTC_TRIG_HIGH_LVL, 2},
	{RT_INTC_TRIG_HIGH_LVL, 2},	/* 16 */
	{RT_INTC_TRIG_HIGH_LVL, 2},
	{RT_INTC_TRIG_LOW_LVL, 2},
	{RT_INTC_TRIG_LOW_LVL, 2},
	{RT_INTC_TRIG_LOW_LVL, 2},
	{RT_INTC_TRIG_LOW_LVL, 2},
	{RT_INTC_TRIG_NEG_EDGE, 2},
	{RT_INTC_TRIG_HIGH_LVL, 3},
	{RT_INTC_TRIG_HIGH_LVL, 2},	/* 24 */
	{RT_INTC_TRIG_POS_EDGE, 2},
	{RT_INTC_TRIG_POS_EDGE, 2},
	{RT_INTC_TRIG_HIGH_LVL, 2},
	{RT_INTC_TRIG_HIGH_LVL, 2},
	{RT_INTC_TRIG_POS_EDGE, 2},
	{RT_INTC_TRIG_POS_EDGE, 3},
	{RT_INTC_TRIG_POS_EDGE, 3},
};

static const char * const sources[] = {
	"SPI",		"UART0", 	"UART1", 	"TIMER0",
	"TIMER1",	"TIMER2",	"RT",		"MAC0",
	"MAC1", 	"UDC"
};

static struct pic_ops rt1310_picops = {
	.pic_unblock_irqs = rt1310_pic_unblock_irqs,
	.pic_block_irqs = rt1310_pic_block_irqs,
	.pic_establish_irq = rt1310_pic_establish_irq,
	.pic_source_name = rt1310_pic_source_name,
};

#define INTC_READ(intc, reg) \
	bus_space_read_4((intc)->intc_memt, (intc)->intc_memh, (reg))
#define INTC_WRITE(intc, reg, val) \
	bus_space_write_4((intc)->intc_memt, (intc)->intc_memh, (reg), (val))

struct intc_softc {
	struct pic_softc intc_pic;
	bus_space_tag_t intc_memt;
	bus_space_handle_t intc_memh;
};

/*
 * Called with interrupts disabled
 */
static int find_pending_irqs(struct intc_softc *sc);
static int
find_pending_irqs(struct intc_softc *sc)
{
	uint32_t pending = INTC_READ(sc, RT_INTC_IPR);

	if (pending == 0)
		return 0;

	return pic_mark_pending_sources(&sc->intc_pic, 0, pending);
}


void
rt1310_irq_handler(void *frame)
{
	struct intc_softc * const intc = device_lookup_private(&intc_cd, 0);
	struct cpu_info * const ci = curcpu();
	const int oldipl = ci->ci_cpl;
	const uint32_t oldipl_mask = __BIT(oldipl);
	int ipl_mask = 0;

	ci->ci_data.cpu_nintr++;

	ipl_mask = find_pending_irqs(intc);

	/*
	 * Record the pending_ipls and deliver them if we can.
	 */
	if ((ipl_mask & ~oldipl_mask) > oldipl_mask)
		pic_do_pending_ints(I32_bit, oldipl, frame);
}

static void
rt1310_pic_unblock_irqs(struct pic_softc *pic, size_t irqbase,
			      uint32_t irq_mask)
{
	struct intc_softc * const intc = (void *) pic;
	uint32_t reg;

	reg = INTC_READ(intc, RT_INTC_IECR);
	reg |= irq_mask;
	INTC_WRITE(intc, RT_INTC_IMR, reg);
	INTC_WRITE(intc, RT_INTC_IECR, reg);
}

static void
rt1310_pic_block_irqs(struct pic_softc *pic, size_t irqbase,
			    uint32_t irq_mask)
{
	struct intc_softc * const intc = (void *) pic;
	uint32_t reg;

	reg = INTC_READ(intc, RT_INTC_IECR);
	reg &= ~irq_mask;
	INTC_WRITE(intc, RT_INTC_IECR, reg);
}

static void
rt1310_pic_establish_irq(struct pic_softc *pic, struct intrsource *is)
{
	/* Nothing */
}

static void
rt1310_pic_source_name(struct pic_softc *pic, int irq, char *buf,
			     size_t len)
{

	strlcpy(buf, sources[irq], len);
}

static int intc_match(device_t, cfdata_t, void *);
static void intc_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(intc, sizeof(struct intc_softc),
    intc_match, intc_attach, NULL, NULL);

int
intc_match(device_t parent, cfdata_t self, void *aux)
{
/*
	struct apb_attach_args * const apba = aux;

	if (apba->apba_addr != APB_INTC_BASE)
		return 0;
*/
	return 1;
}

void
intc_attach(device_t parent, device_t self, void *aux)
{
	struct intc_softc * const intc = device_private(self);
	struct ahb_attach_args * const aa = aux;
	int error;
	int i;

//	KASSERT(aa->ahba_irqbase != AHBCF_IRQBASE_DEFAULT);
//	KASSERT(device_unit(self) == 0);

//	if (aa->ahba_size == AHBCF_SIZE_DEFAULT)
	aa->ahba_size = AHB_INTC_SIZE;

	intc->intc_memt = aa->ahba_memt;
	error = bus_space_map(intc->intc_memt, aa->ahba_addr, aa->ahba_size,
	    0, &intc->intc_memh);
	if (error)
		panic("intc_attach: failed to map register %#lx-%#lx: %d",
		    aa->ahba_addr, aa->ahba_addr + aa->ahba_size - 1,
		    error);

	aa->ahba_irqbase = 0;
	intc->intc_pic.pic_ops = &rt1310_picops;
	intc->intc_pic.pic_maxsources = RT_NIRQ;
	strlcpy(intc->intc_pic.pic_name, device_xname(self),
	    sizeof(intc->intc_pic.pic_name));
	pic_add(&intc->intc_pic, aa->ahba_irqbase);
	aprint_normal(": interrupts %d..%d\n",
	    aa->ahba_irqbase, aa->ahba_irqbase + RT_NIRQ - 1);
	for (i = 0; i < INTC_NIRQS; ++i) {
		INTC_WRITE(intc, RT_INTC_SCR0+i*4,
			(irqdef[i].ri_trig << RT_INTC_TRIG_SHIF) |
			irqdef[i].ri_prio);
		INTC_WRITE(intc, RT_INTC_SVR0+i*4, i);
	}
	INTC_WRITE(intc, RT_INTC_ICCR, ~0);
	INTC_WRITE(intc, RT_INTC_IMR, 0);
}
