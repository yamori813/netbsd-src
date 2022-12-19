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
#include <sys/evcnt.h>
#include <sys/systm.h>

#include <machine/intr.h>
#include <machine/lock.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>
#include <arm/star/star_orion_intr.h>

void
star_orion_set_intrmask(void)
{
	STAR_REG_WRITE32(ORION_INT_CLEAR, star_intr_enabled);
	STAR_REG_WRITE32(ORION_INT_MASK, ~star_intr_enabled);
}

void
star_orion_set_intrmode(int irq, int scheme)
{
	uint32_t level, mode, r;

	KASSERT(irq <= STAR_NIRQ);
	KASSERT((scheme == IST_LEVEL_HIGH) ||
	    (scheme == IST_LEVEL_LOW) ||
	    (scheme == IST_EDGE_RISING) ||
	    (scheme == IST_EDGE_FALLING));

	mode = 0;
	level = 0;
	switch (scheme) {
	case IST_LEVEL_HIGH:
		mode = 0;
		level = 0;
		break;
	case IST_LEVEL_LOW:
		mode = 0;
		level = 1;
		break;
	case IST_EDGE_RISING:
		mode = 1;
		level = 0;
		break;
	case IST_EDGE_FALLING:
		mode = 1;
		level = 1;
		break;
	}

	r = STAR_REG_READ32(ORION_INT_TRIGMODE) & ~(1 << irq);
	STAR_REG_WRITE32(ORION_INT_TRIGMODE, r | (mode << irq));

	r = STAR_REG_READ32(ORION_INT_TRIGLEVEL) & ~(1 << irq);
	STAR_REG_WRITE32(ORION_INT_TRIGLEVEL, r | (level << irq));
}

void
star_orion_init(void)
{
	/* set cpu depend handler */
	star_set_intrmask = star_orion_set_intrmask;

	/*
	 * setup Vector Interrupt Control Registers
	 */
	STAR_REG_WRITE32(ORION_INT_TRIGMODE, 0xffffffff);	/* EdgeTrigger */
	STAR_REG_WRITE32(ORION_INT_TRIGLEVEL, 0x00000000);	/* HighLevelTrigger */

//XXX
	STAR_REG_WRITE32(ORION_INT_TRIGMODE, 0x9078200f);
	STAR_REG_WRITE32(ORION_INT_TRIGLEVEL, 0x0f818064);

	STAR_REG_WRITE32(ORION_INT_FIQSEL, 0x00000000);	/* IRQ mode */
	STAR_REG_WRITE32(ORION_INT_CLEAR, 0xffffffff);	/* clear all interrupt */
	STAR_REG_WRITE32(ORION_INT_MASK, 0xffffffff);	/* disable all interrupt */
}
