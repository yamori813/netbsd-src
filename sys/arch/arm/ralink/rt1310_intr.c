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

static const char * const sources[] = {
    "CPUSelfInt",      "CPUTimer0IntReq", "CPUTimer1IntReq", "CPUWDTimerIntReq",
    "AccessErr",       "Bit64Err",
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

void
rt1310_irq_handler(void *frame)
{
	struct intc_softc * const intc = device_lookup_private(&intc_cd, 0);
	struct pic_softc * const pic = &intc->intc_pic;
	uint32_t irq;

	irq = ffs(INTC_READ(intc, RT_INTC_IPR)) - 1;

	pic_dispatch(pic->pic_sources[irq], frame);
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
//	int i;

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
	INTC_WRITE(intc, RT_INTC_ICCR, ~0);
	INTC_WRITE(intc, RT_INTC_IMR, 0);
#if 0

	aprint_normal(": interrupts %d..%d\n",
	    apba->apba_irqbase, apba->apba_irqbase + 63);

	INTC_WRITE(intc, INTC_ARM0_IRQMASK_0, 0);
	INTC_WRITE(intc, INTC_ARM0_IRQMASK_1, 0);
	INTC_WRITE(intc, INTC_ARM1_IRQMASK_0, 0);
	INTC_WRITE(intc, INTC_ARM1_IRQMASK_1, 0);

	for (i = 0; i < 22; ++i)
		set_intc_priority(intc, comcerto_irq_table[i].num,
		    comcerto_irq_table[i].prio);

/*
	for (i = 0; i < 8; ++i)
		printf("REG %d %x\n", i, INTC_READ(intc, i * 4));
	for (i = 0; i < 8; ++i)
		printf("PRTY %d %x\n", i, INTC_READ(intc, INTC_ARM0_PRTY_0 + i * 4));
*/
#if 0
	softintr_init();
#endif
#endif
}
