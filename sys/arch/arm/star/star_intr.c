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
#define _INTR_PRIVATE

#include <sys/param.h>
#include <sys/systm.h>
//#include <sys/malloc.h>
#include <sys/kmem.h>
#include <uvm/uvm_extern.h>

#include <machine/intr.h>
//#include <machine/lock.h>
#include <sys/bus.h>
#include <sys/intr.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>
#include <arm/star/star_intr.h>
#include <arm/star/star_equuleus_intr.h>
#include <arm/star/star_orion_intr.h>

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

struct intrhand {
	TAILQ_ENTRY(intrhand) ih_list;	/* link on intrq list */
	int (*ih_func)(void *);		/* handler */
	void *ih_arg;			/* arg for handler */
	int ih_irq;			/* IRQ number */
	int ih_ipl;			/* IPL */
};

struct intrq {
	TAILQ_HEAD(, intrhand) iq_list;	/* handler list */
	struct evcnt iq_evcnt;		/* event counter */
	u_int32_t iq_levels;		/* IPL_*'s this IRQ has */
	const char *iq_irqname;		/* interrupt name */
};

const char *irqnames[STAR_NIRQ] = {
	"Timer1",		/* int 0 */
	"Timer2",		/* int 1 */
	"CPU FREQ",		/* int 2 */
	"WATCHDOG",		/* int 3 */
	"GPIO",			/* int 4 */
	"PCI Ext0",		/* int 5 */
	"PCI Ext1",		/* int 6 */
	"PCI Ext2",		/* int 7 */
	"PCI Bridge",		/* int 8 */
	"UART0",		/* int 9 */
	"UART1",		/* int 10 */
	"DMA",			/* int 11 */
	"DMAErr",		/* int 12 */
	"PCMCIA",		/* int 13 */
	"RTC",			/* int 14 */
	"PCM ExtInt",		/* int 15 */
	"USB",			/* int 16 */
	"IDE",			/* int 17 */
	"NIC STAT",		/* int 18 */
	"NIC TX",		/* int 19 */
	"NIC RX",		/* int 20 */
	"NIC EMPTY",		/* int 21 */
	"NIC FULL",		/* int 22 */
	"USB1.1",		/* int 23 */
	"USB2.0",		/* int 24 */
	"I2S",			/* int 25 */
	"SPI",			/* int 26 */
	"TWI",			/* int 27 */
	"USB VBUS",		/* int 28 */
	"EXT GPIOA0",		/* int 29 */
	"EXT GPIOA1",		/* int 30 */
	"HSDMA"			/* int 31 */
};

/*
 * interrupt dispatch table.
 */
struct intrq irqhandler[STAR_NIRQ];

/*
 * interrupt work
 */
uint32_t star_intr_mask[NIPL];
volatile uint32_t star_intr_pending;
volatile uint32_t star_intr_enabled;
struct pic_softc pic_sc;

void (*star_set_intrmask)(void);

static inline void
star_enable_irq(int irq)
{
	star_intr_enabled |= (1U << irq);
	star_set_intrmask();
}

static inline void
star_disable_irq(int irq)
{
	star_intr_enabled &= ~(1U << irq);
	star_set_intrmask();
}

/*
 * Interrupt Mask Handling
 */
void
star_intr_calculate_masks(void)
{
	struct intrq *iq;
	struct intrhand *ih;
	int irq, ipl;

	/* First, figure out which IPLs each IRQ has. */
	for (irq = 0; irq < STAR_NIRQ; irq++) {
		star_disable_irq(irq);
		irqhandler[irq].iq_levels = 0;
		iq = &irqhandler[irq];
		TAILQ_FOREACH(ih, &iq->iq_list, ih_list) {
			irqhandler[irq].iq_levels |= (1U << ih->ih_ipl);
		}
	}

	/* Next, figure out which IRQs are used by each IPL. */
	for (ipl = 0; ipl < NIPL; ipl++) {
		star_intr_mask[ipl] = 0;
		for (irq = 0; irq < STAR_NIRQ; irq++) {
			if (irqhandler[irq].iq_levels & (1U << ipl)) {
				star_intr_mask[ipl] |= (1U << irq);
			}
		}
	}

	KASSERT(star_intr_mask[IPL_NONE] == 0);

	star_intr_mask[IPL_SOFTBIO]    |= star_intr_mask[IPL_SOFTCLOCK];
	star_intr_mask[IPL_SOFTNET]    |= star_intr_mask[IPL_SOFTBIO];
	star_intr_mask[IPL_SOFTSERIAL] |= star_intr_mask[IPL_SOFTNET];
	star_intr_mask[IPL_VM]         |= star_intr_mask[IPL_SOFTSERIAL];
	star_intr_mask[IPL_SCHED]      |= star_intr_mask[IPL_VM];
	star_intr_mask[IPL_HIGH]       |= star_intr_mask[IPL_SCHED];

	/* Now, enable needed IRQ */
	for (irq = 0; irq < STAR_NIRQ; irq++) {
		iq = &irqhandler[irq];
		if (!TAILQ_EMPTY(&iq->iq_list)) {
			star_enable_irq(irq);
		}
	}
}

void
star_intr_init(void)
{
	struct intrq *iq;
	int i;

	for (i = 0; i < STAR_NIRQ; i++) {
		iq = &irqhandler[i];
		TAILQ_INIT(&iq->iq_list);
		irqhandler[i].iq_irqname = irqnames[i];
		evcnt_attach_dynamic(&irqhandler[i].iq_evcnt,
		    EVCNT_TYPE_INTR, NULL, "cpu", irqhandler[i].iq_irqname);
	}

	curcpu()->ci_intr_depth = 0;
	set_curcpl(0);

	star_intr_calculate_masks();

	pic_sc.pic_ops = &intc_pic_ops;
	pic_sc.pic_maxsources = 32;
	strlcpy(pic_sc.pic_name, "star_intr", sizeof(pic_sc.pic_name));
 
	pic_add(&pic_sc, 0);

	/* Enable IRQs (don't yet use FIQs). */
//	enable_interrupts(I32_bit);
}

#if 0
#undef splx
void
splx(int x)
{
	star_splx(x);
}

#undef _spllower
int
_spllower(int s)
{
	return star_spllower(s);
}

#undef _splraise
int
_splraise(int s)
{
	return star_splraise(s);
}
#endif

static int
find_pending_irqs(struct pic_softc *sc)
{
	uint32_t pending;

	/* read interrupt status, and clear */
	if (CPU_IS_STR8100()) {
		pending = STAR_REG_READ32(EQUULEUS_INT_IRQSTATUS);
		STAR_REG_WRITE32(EQUULEUS_INT_CLEAR, pending);
	} else {
		pending = STAR_REG_READ32(ORION_INT_IRQSTATUS);
		STAR_REG_WRITE32(ORION_INT_CLEAR, pending);
	}

	pending &= star_intr_enabled;

	if (pending == 0)
		return 0;

	return pic_mark_pending_sources(sc, 0, pending);
}

void
star_intr_handler(void *frame)
{
	struct cpu_info * const ci = curcpu();
	const int oldipl = ci->ci_cpl;
	const uint32_t oldipl_mask = __BIT(oldipl);
	int ipl_mask = 0;

	ci->ci_data.cpu_nintr++;

	ipl_mask = find_pending_irqs(&pic_sc);

	/*
	 * Record the pending_ipls and deliver them if we can.
	 */
	if ((ipl_mask & ~oldipl_mask) > oldipl_mask)
		pic_do_pending_ints(I32_bit, oldipl, frame);
}

void
star_intr_disestablish(void *cookie)
{
	struct intrq *iq;
	struct intrhand *ih;
	int irq, s;

	KASSERT(cookie != NULL);

	ih = (struct intrhand *)cookie;
	irq = ih->ih_irq;

	s = disable_interrupts(I32_bit);
	{
		iq = &irqhandler[irq];
		TAILQ_REMOVE(&iq->iq_list, ih, ih_list);
		star_intr_calculate_masks();
	}
	restore_interrupts(s);
}

void
intc_unblock_irqs(struct pic_softc *pic, size_t irq_base, uint32_t irq_mask)
{

	star_intr_enabled |= irq_mask;
	star_set_intrmask();
}

void
intc_block_irqs(struct pic_softc *pic, size_t irq_base, uint32_t irq_mask)
{

	star_intr_enabled &= ~irq_mask;
	star_set_intrmask();
}

void
intc_establish_irq(struct pic_softc *pic, struct intrsource *is)
{
	int s;

	KASSERT(is->is_irq < 32);

	s = disable_interrupts(I32_bit);

	if (CPU_IS_STR8100())
		star_equuleus_set_intrmode(is->is_irq, is->is_type);
	else
		star_orion_set_intrmode(is->is_irq, is->is_type);

	restore_interrupts(s);
}

void
intc_source_name(struct pic_softc *pic, int irq, char *buf, size_t len)
{

	KASSERT((unsigned int)irq < 32);
	strlcpy(buf, irqnames[irq], len);
}

