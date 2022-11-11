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
#ifndef _ARM_MINDSPEED_M83_INTR_H_
#define _ARM_MINDSPEED_M83_INTR_H_

#ifdef _LOCORE

#define	ARM_IRQ_HANDLER	_C_LABEL(m83_irq_handler)

#else

#define AVIC_INTR_SOURCE_NAMES \
{	"status1",	"ptp0",		"ptp1",		"ptp2",		\
	"tdm timer",	"usb0",		"emac0",	"emac1",	\
	"tdma",		"ipsec wrap",	"ipsec core",	"spi",		\
	"mdma0",	"mdma1",	"pcie0 ext",	"pcie0 int",	\
	"pcie1 ext",	"pcie1 int",	"ddrc",		"hwfault",	\
	"thermal",	"i2c",		"aram",		"emac0 batch",	\
	"emac1 batch",	"fpp cap",	"timer5",	"timer4",	\
	"timer3",	"timer2",	"timer1",	"timer0",	\
	"status0",	"g0",		"g1",		"g2",		\
	"g3",		"g4",		"g5",		"g6",		\
	"g7",		"uart0",	"tdma ind",	"ved",		\
	"spdrv acp work done",	"spdrv acp req",	"irq14",	 "cspvwdrx",	\
	"cspvwdtx",	"csp pmu",	"msp pmu",	"fromhost",	\
	"tohost",	"cspved0rx",	"cspces0tx",	"cspved1rx",	\
	"icspved1tx",	"csp hidrv",	"uart1",	"tdma rxahberr",\
	"tdma txahberr",	"reserved1",	"reserved2",	"reserved3"		\
}

#define	PIC_MAXMAXSOURCES	(64+3*32+128)

#include <arm/pic/picvar.h>

const char *
	intr_typename(int);

void m83_irq_handler(void *);

#endif /* !_LOCORE */

#endif /* _ARM_MINDSPEED_M83_INTR_H_ */
