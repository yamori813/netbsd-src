/*	$NetBSD$	*/

/*-
 * Copyright (c) 2007 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas.
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

#include <sys/param.h>
#include <sys/evcnt.h>
#include <sys/device.h>
#include <sys/atomic.h>
 
#include <uvm/uvm_extern.h>
  
#include <machine/intr.h>
 
#include <arm/cpu.h>
#include <arm/armreg.h>
#include <arm/cpufunc.h>

#include <machine/autoconf.h>
#include <sys/bus.h>

#include <arm/mindspeed/m83xxx_reg.h>
#include <arm/mindspeed/m83xxx_var.h>
#include <arm/mindspeed/m83xxx_intc.h>

static void intc_unblock_irqs(struct pic_softc *, size_t, uint32_t);
static void intc_block_irqs(struct pic_softc *, size_t, uint32_t);
static void intc_establish_irq(struct pic_softc *, struct intrsource *);
static void intc_source_name(struct pic_softc *, int, char *, size_t);

const struct pic_ops intc_pic_ops = {
	.pic_unblock_irqs = intc_unblock_irqs,
	.pic_block_irqs = intc_block_irqs,
	.pic_establish_irq = intc_establish_irq,
	.pic_source_name = intc_source_name
};

struct intc_softc {
	struct pic_softc intc_pic;
	bus_space_tag_t intc_memt;
	bus_space_handle_t intc_memh;
};

static void set_intc_priority(struct intc_softc *intc, uint32_t irq,
    uint32_t prio);

extern struct cfdriver intc_cd;

#define	INTC_READ(intc, reg) \
	bus_space_read_4((intc)->intc_memt, (intc)->intc_memh, (reg))
#define	INTC_WRITE(intc, reg, val) \
	bus_space_write_4((intc)->intc_memt, (intc)->intc_memh, (reg), (val))
#define	HW_TO_SW_IPL(ipl)	((ipl) + 1)
#define	SW_TO_HW_IPL(ipl)	((ipl) - 1)

/* IRQ configuration table */
static struct comcerto_irq_desc comcerto_irq_table[] =
{
	{IRQ_EMAC0_BATCH,		1},
	{IRQ_EMAC1_BATCH,		2},
	{IRQ_ARAM,			3},
	{IRQ_PCIe0_EXT,			4},
	{IRQ_PCIe1_EXT,			5},
	{IRQ_PCIe0_INT,			6},
	{IRQ_PCIe1_INT,			7},
	{IRQ_USB0,			8},
	{STATUS_REG_1,			9},
	{IRQ_PTP0,			10},
	{IRQ_TIMER1,			11},
	{IRQ_TIMER3,			12},
	{IRQ_TIMER4,			13},
	{IRQ_TIMER5,			14},
	{IRQ_SPI,			15},
	{IRQ_EMAC0,			16},
	{IRQ_EMAC1,			17},
	{IRQ_IPSEC_WRAP,		18},
	{IRQ_IPSEC_CORE,		19},
	{IRQ_I2C,			20},
	{IRQ_14,			21},
	{IRQ_FPP_CAP,			22}
};

void
set_intc_priority(struct intc_softc *intc, uint32_t irq, uint32_t prio)
{
	uint32_t prio_reg, prio_shift, prio_mask;

	prio_reg = INTC_ARM0_PRTY_0 + 4 * (prio / 4);
	prio_shift = ((prio % 4) << 3);
	prio_mask = 0x1f << prio_shift;

	INTC_WRITE(intc, prio_reg, (INTC_READ(intc, prio_reg) & ~prio_mask) |
	    (irq << prio_shift));
}

void
intc_unblock_irqs(struct pic_softc *pic, size_t irq_base, uint32_t irq_mask)
{
	struct intc_softc * const intc = (void *) pic;
	uint32_t reg;

	if (irq_base == 0) {
//		INTC_WRITE(intc, INTC_STATUS_REG_0, irq_mask);
		reg = INTC_READ(intc, INTC_ARM0_IRQMASK_0);
		INTC_WRITE(intc, INTC_ARM0_IRQMASK_0, reg | irq_mask);
	} else {
//		INTC_WRITE(intc, INTC_STATUS_REG_1, irq_mask);
		reg = INTC_READ(intc, INTC_ARM0_IRQMASK_1);
		INTC_WRITE(intc, INTC_ARM0_IRQMASK_1, reg | irq_mask);
	}
}

void
intc_block_irqs(struct pic_softc *pic, size_t irq_base, uint32_t irq_mask)
{
	struct intc_softc * const intc = (void *) pic;
	uint32_t reg;

	if (irq_base == 0) {
		reg = INTC_READ(intc, INTC_ARM0_IRQMASK_0);
		INTC_WRITE(intc, INTC_ARM0_IRQMASK_0, reg & ~irq_mask);
	} else {
		reg = INTC_READ(intc, INTC_ARM0_IRQMASK_1);
		INTC_WRITE(intc, INTC_ARM0_IRQMASK_1, reg & ~irq_mask);
	}
}

void
intc_establish_irq(struct pic_softc *pic, struct intrsource *is)
{
#if 0
	struct intc_softc * const intc = (void *) pic;
	bus_addr_t priority_reg;
	int priority_shift;
	uint32_t v;

	KASSERT(is->is_irq < 64);
	KASSERT(is->is_ipl < 16);
	priority_reg = IMX31_NIPRIORITY0 - (is->is_irq >> 3);
	priority_shift = (is->is_irq & 7) * 4; 
	v = INTC_READ(intc, priority_reg);
	v &= ~(0x0f << priority_shift);
	v |= SW_TO_HW_IPL(is->is_ipl) << priority_shift;
	INTC_WRITE(intc, priority_reg, v);

	KASSERT(is->is_type == IST_LEVEL);
#endif
}

static const char * const intc_intr_source_names[] = AVIC_INTR_SOURCE_NAMES;

void
intc_source_name(struct pic_softc *pic, int irq, char *buf, size_t len)
{
	strlcpy(buf, intc_intr_source_names[irq], len);
}

void
m83_irq_handler(void *frame)
{
	struct cpu_info * const ci = curcpu();
	struct intc_softc * const intc = device_lookup_private(&intc_cd, 0);
	struct pic_softc * const pic = &intc->intc_pic;
	int32_t stat0, mask0;
	int32_t stat1, mask1;
	int32_t irq;

        ci->ci_data.cpu_nintr++;

	stat0 = INTC_READ(intc, INTC_STATUS_REG_0);
	mask0 = INTC_READ(intc, INTC_ARM0_IRQMASK_0);
	stat1 = INTC_READ(intc, INTC_STATUS_REG_1);
	mask1 = INTC_READ(intc, INTC_ARM0_IRQMASK_1);

	do {
		if (stat0 & mask0) {
			irq = ffs(stat0 & mask0) - 1;
		} else {
			irq = ffs(stat1 & mask1) - 1 + 32;
		}

		pic_dispatch(pic->pic_sources[irq], frame);

		/* ack */
		if (irq < 32) {
			INTC_WRITE(intc, INTC_STATUS_REG_0, 1 << irq);
			stat0 &= ~(1 << irq);
		} else {
			INTC_WRITE(intc, INTC_STATUS_REG_1, 1 << (irq & 0x1f));
			stat1 &= ~(1 << (irq & 0x1f));
		}
	} while ((stat0 & mask0) || (stat1 & mask1));
}

static int intc_match(device_t, cfdata_t, void *);
static void intc_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(intc, sizeof(struct intc_softc),
    intc_match, intc_attach, NULL, NULL);

int
intc_match(device_t parent, cfdata_t self, void *aux)
{
	struct apb_attach_args * const apba = aux;

	if (apba->apba_addr != APB_INTC_BASE)
		return 0;

	return 1;
}

void
intc_attach(device_t parent, device_t self, void *aux)
{
	struct intc_softc * const intc = device_private(self);
	struct apb_attach_args * const apba = aux;
	int error;
	int i;

	KASSERT(apba->apba_irqbase != AHBCF_IRQBASE_DEFAULT);
	KASSERT(device_unit(self) == 0);

	if (apba->apba_size == AHBCF_SIZE_DEFAULT)
		apba->apba_size = APB_INTC_SIZE;

	intc->intc_memt = apba->apba_memt;
	error = bus_space_map(intc->intc_memt, apba->apba_addr, apba->apba_size,
	    0, &intc->intc_memh);
	if (error)
		panic("intc_attach: failed to map register %#lx-%#lx: %d",
		    apba->apba_addr, apba->apba_addr + apba->apba_size - 1,
		    error);

	intc->intc_pic.pic_ops = &intc_pic_ops;
	intc->intc_pic.pic_maxsources = 64;
	strlcpy(intc->intc_pic.pic_name, device_xname(self),
	    sizeof(intc->intc_pic.pic_name));

	pic_add(&intc->intc_pic, apba->apba_irqbase);
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
}
