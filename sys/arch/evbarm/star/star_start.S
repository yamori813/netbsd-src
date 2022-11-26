/*	$NetBSD$ */

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

#include <machine/asm.h>
#include <arm/armreg.h>
#include <arm/arm32/pmap.h>
#include <arm/star/starreg.h>

#define SDRAM_START	STAR_SDRAM_MEMORY		/* 0x20000000 */
//#define SDRAM_LOAD	(STAR_SDRAM_MEMORY + 0x8000)	/* 0x20008000 */
#define SDRAM_LOAD	(STAR_SDRAM_MEMORY + 0x40000)	/* 0x20040000 */

/*
 * 0x20000000-0x20004000: temporary stack
 * 0x20004000-0x20008000: L1table
 *      :
 * 0x21xxxxxx           : loaded kernel image
 */
	.global	_C_LABEL(star_start)
_C_LABEL(star_start):
	/* Make sure svc mode and disable IRQs & FIQs */
	mov	r0, #(PSR_SVC32_MODE | I32_bit | F32_bit)
	msr	cpsr_c, r0

	/* Disable MMU, etc */
	mrc	p15, 0, r0, c1, c0, 0
	bic	r0, r0, #CPU_CONTROL_SYST_ENABLE
	bic	r0, r0, #CPU_CONTROL_MMU_ENABLE
	orr	r0, r0, #CPU_CONTROL_IC_ENABLE
	bic	r0, r0, #CPU_CONTROL_DC_ENABLE
	mcr	p15, 0, r0, c1, c0, 0

	/* set stack pointer 0x20000000-0x20004000 */
	ldr	sp, Lstartup_pagetable

	/* copy to relocate myself from *HERE* to SDRAM_LOAD (0x20008000) */
	ldr	r0, Lloadaddr		/* dst */
	adr	r1, star_start		/* src */
	ldr	r2, Lsizeofmyself	/* len */
	bl	relocate_myself

	/* jump to relocated address (0x20008000 + Ljump) */
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0	/* Invalidate Entire I cache */
	mcr	p15, 0, r0, c7, c14, 0	/* Clean & Invalidate Entire D cache */

	adr	r0, Ljumpaddr
	ldr	pc, [r0]

Lsizeofmyself:
	.word	__bss_start - star_start	/* size of kernel */
Lloadaddr:
	.word	SDRAM_LOAD			/* 0x20008000 */
Ljumpaddr:
	.word	start1 - star_start + SDRAM_LOAD


start1:
	/* build page table from scratch */
	ldr	r0, Lstartup_pagetable		/* pagetable */
	adr	r4, mmu_init_table
	b	3f
2:
	str	r3, [r0, r2]
	add	r2, r2, #4
	add	r3, r3, #(L1_S_SIZE)
	adds	r1, r1, #-1
	bhi	2b
3:
	ldmia	r4!, {r1, r2, r3}	/* # of sections, VA, PA|attr */
	cmp	r1, #0
	bne	2b

	/* r0 = pagetable for startup */
	mcr	p15, 0, r0, c2, c0, 0	/* Set Translation Table Base */
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0	/* Invalidate Entire I cache */
	mcr	p15, 0, r0, c7, c14, 0	/* Clean & Invalidate Entire D cache */
	mcr	p15, 0, r0, c7, c5, 6	/* invalidate Branch Target Buffer */
	mcr	p15, 0, r0, c7, c10, 4	/* Drain write buffer */
	mcr	p15, 0, r0, c8, c7, 0	/* Invalidate TLB */

	/* Set the Domain Access register */
	mov	r0, #((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL * 2)) | DOMAIN_CLIENT)
	mcr	p15, 0, r0, c3, c0, 0

	/* Enable MMU */
	mrc	p15, 0, r0, c1, c0, 0
	orr	r0, r0, #CPU_CONTROL_SYST_ENABLE
	orr	r0, r0, #CPU_CONTROL_MMU_ENABLE
	orr	r0, r0, #CPU_CONTROL_IC_ENABLE
	bic	r0, r0, #CPU_CONTROL_DC_ENABLE	/* XXX: disable data cache */
	mcr	p15, 0, r0, c1, c0, 0

	/* Ensure that the coprocessor has finished turning on the MMU */
	mrc	p15, 0, r0, c2, c0, 0	/* arbitrary read of CP15 */
	mov	r0, r0			/* wait for it to complete */

	/* invalidate cache */
	mov	r0, #0
	mcr	p15, 0, r0, c7, c5, 0	/* Invalidate Entire I cache */
	mcr	p15, 0, r0, c7, c14, 0	/* Clean & Invalidate Entire D cache */

	/* Jump to kernel code in TRUE VA */
	adr	r0, Lstart
	ldr	pc, [r0]

Lstart:
	.word	start

Lstartup_pagetable:
	.word	SDRAM_START + L1_TABLE_SIZE		/* =0x20004000 */


#define MMU_INIT(va, pa, n_sec, attr) \
	.word	n_sec					; \
	.word	4 * ((va) >> L1_S_SHIFT)		; \
	.word	(pa) | (attr)				;

mmu_init_table:	
	/* fill all table VA==PA */
	MMU_INIT(0x00000000, 0x00000000,
	    1 << (32 - L1_S_SHIFT), L1_TYPE_S | L1_S_AP(AP_KRW))

	/* map SDRAM VA==PA, WT cacheable */
	MMU_INIT(SDRAM_START, SDRAM_START,
	    128, L1_TYPE_S | L1_S_C | L1_S_AP(AP_KRW))

	/* map VA 0xc0000000..0xc7ffffff to SDRAM PA 0x20000000..0x27ffffff */
	MMU_INIT(0xc0000000, SDRAM_START,
	    128, L1_TYPE_S | L1_S_C | L1_S_AP(AP_KRW))

	/* map UART0 0x78000000 to 0xF7800000 */
	MMU_INIT(STAR_IO_P2V(0x78000000), 0x78000000,
	    1, L1_TYPE_S | L1_S_AP(AP_KRW))

	/* map STR8100 Interrupt Register area 0xFFFFF000 to 0xFFFFF000 */
	MMU_INIT(STAR_IO_P2V(0xFFFFF000), 0xFFFFF000,
	    1, L1_TYPE_S | L1_S_AP(AP_KRW))

	/* map STR9100 Interrupt Register area 0x7D000000 to 0xF7D00000 */
	MMU_INIT(STAR_IO_P2V(0x7D000000), 0x7D000000,
	    1, L1_TYPE_S | L1_S_AP(AP_KRW))

	.word	0			/* end of table */


relocate_myself:
	add	r2, r2, #3	/* 4byte align */
	bic	r2, r2, #3
1:
	ldr	r3, [r1], #4	/* r3 = *src++ */
	subs	r2, r2, #4	/* size -= 4 */
	str	r3, [r0], #4	/* *dst++ = r3 */
	bne	1b
	mov	pc, lr


Luart0adr:
#ifdef DEBUGPRINT_BEFORE_MMU
	.word	0x78000000		/* if use debugprint() before enable MMU */
#else
	.word	STAR_IO_P2V(0x78000000)	/* PA=0x78000000 */
#endif

	.global	_C_LABEL(debugprintx)
_C_LABEL(debugprintx):
	stmfd	sp!, {r0, r3, lr}

	mov	r3, r0

	mov	r0, #'0'
	bl	debugputc
	mov	r0, #'x'
	bl	debugputc

	bl	print_r3

	mov	r0, #0x0d
	bl	debugputc
	mov	r0, #0x0a
	bl	debugputc

	ldmfd	sp!, {r0, r3, pc}

	.global	_C_LABEL(debugprint)
_C_LABEL(debugprint):
	stmfd	sp!, {r0, r1, lr}
	mov	r1, r0
1:
	ldrb	r0, [r1], #1
	cmp	r0, #0
	beq	9f
	bl	debugputc
	b	1b
9:
	ldmfd	sp!, {r0, r1, pc}


print_r3:
	stmfd	sp!, {r0, r3-r6, lr}
	mov	r4, #28
	mov	r5, #0xf
1:
	and	r6, r5, r3, ROR r4
	cmp	r6, #10
	addlt	r0, r6, #'0'
	addge	r0, r6, #('a' - 0x0a)
	bl	debugputc
	subs	r4, r4, #4
	bge	1b
	ldmfd	sp!, {r0, r3-r6, pc}

	.global	_C_LABEL(debugputc)
_C_LABEL(debugputc):
	stmfd	sp!, {r0, r1, lr}

	ldr	r1, Luart0adr
	strb	r0, [r1]

	mov	r1, #16384
1:	subs	r1, r1, #1
	bne	1b

	ldmfd	sp!, {r0, r1, pc}
