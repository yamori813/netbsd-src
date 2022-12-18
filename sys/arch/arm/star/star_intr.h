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

#ifndef _STAR_INTR_H_
#define _STAR_INTR_H_

#ifdef _LOCORE

//#define ARM_IRQ_HANDLER	_C_LABEL(star_intr_dispatch)
#define ARM_IRQ_HANDLER	_C_LABEL(star_intr_handler)

#else

#include <arm/cpu.h>
#include <arm/armreg.h>
#include <arm/cpufunc.h>

#include <arm/pic/picvar.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>
#include <arm/star/star_equuleus_intr.h>
#include <arm/star/star_orion_intr.h>

extern uint32_t star_intr_mask[];
extern volatile uint32_t star_intr_enabled;
extern volatile uint32_t star_intr_pending;
extern void (*star_set_intrmask)(void);

#if 0
static inline void __attribute__((__unused__))
star_splx(int ipl)
{
	int oldirqstate, hwpend;

	/* Don't let the compiler re-order this code with preceding code */
	__insn_barrier();

	set_curcpl(ipl);

	hwpend = (star_intr_pending) & ~star_intr_mask[ipl];
	if (hwpend != 0) {
		oldirqstate = disable_interrupts(I32_bit);
		star_intr_enabled |= hwpend;
		star_set_intrmask();
		restore_interrupts(oldirqstate);
	}

#ifdef __HAVE_FAST_SOFTINTS
	cpu_dosoftints();
#endif
}

static inline int __attribute__((__unused__))
star_splraise(int ipl)
{
	int old;

	old = curcpl();
	set_curcpl(ipl);

	/* Don't let the compiler re-order this code with subsequent code */
	__insn_barrier();

	return old;
}

static inline int __attribute__((__unused__))
star_spllower(int ipl)
{
	int old;

	old = curcpl();
	star_splx(ipl);

	return old;
}
#endif

void star_intr_calculate_masks(void);
void splx(int);
int _spllower(int);
int _splraise(int);
void _setsoftintr(int);

#if 0
#if !defined(EVBARM_SPL_NOINLINE)
#define splx(ipl)		star_splx(ipl)
#define _spllower(ipl)		star_spllower(ipl)
#define _splraise(ipl)		star_splraise(ipl)
#define _setsoftintr(ipl)	star_setsoftintr(ipl)
#endif /* EVBARM_SPL_NOINLINE */
#endif

/* same as intr.h */
#define STAR_INTR_HIGHLEVEL_TRIGGER	4
#define STAR_INTR_LOWLEVEL_TRIGGER	3
#define STAR_INTR_RISING_EDGE		5
#define STAR_INTR_FALLING_EDGE		2

void star_intr_init(void);
void star_intr_dispatch(void *);
//void *star_intr_establish(int, int, int, int (*)(void *), void *);
void star_intr_disestablish(void *);

void star_intr_handler(void *frame);

#endif /* _LOCORE */

#endif /* _STAR_INTR_H_ */
