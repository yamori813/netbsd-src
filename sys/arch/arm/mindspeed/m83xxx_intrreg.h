/* $NetBSD$ */
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
#ifndef _ARM_MINDSPEED_M83_INTRREG_H_
#define _ARM_MINDSPEED_M83_INTRREG_H_

#define	INTC_STATUS_REG_0		0x0000
#define	INTC_SET_STATUS_REG_0		0x0004
#define	INTC_ARM0_IRQMASK_0		0x0008
#define	INTC_ARM0_FIQMASK_0		0x000c
#define	INTC_ARM1_IRQMASK_0		0x0010
#define	INTC_ARM1_FIQMASK_0		0x0014
#define	INTC_ARM1_CONTROL_REG		0x0018
#define	INTC_IRQ_ACK_TEST_REG		0x001c
#define	INTC_STATUS_REG_1		0x0020
#define	INTC_SET_STATUS_REG_1		0x0024
#define	INTC_ARM0_IRQMASK_1		0x0028
#define	INTC_ARM0_FIQMASK_1		0x002c
#define	INTC_ARM1_IRQMASK_1		0x0030
#define	INTC_ARM1_FIQMASK_1		0x0034
#define	INTC_STATUS_MASK_REG_1		0x0038
#define	INTC_ARM0_PRTY_0		0x0040
#define	INTC_ARM0_IRQ_WNR		0x0060

#endif /* _ARM_MINDSPEED_M83_INTRREG_H_ */
