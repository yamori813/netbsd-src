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
#ifndef _ARM_MINDSPEED_M83XXX_INTR_H_
#define _ARM_MINDSPEED_M83XXX_INTR_H_

#ifdef _LOCORE

#define	ARM_IRQ_HANDLER	_C_LABEL(m83_irq_handler)

#else

#define STATUS_REG_1			0
#define IRQ_PTP0			1
#define IRQ_PTP1			2
#define IRQ_PTP2			3
#define IRQ_TDM_TIMER			4
#define IRQ_USB0			5
#define IRQ_EMAC0			6
#define IRQ_EMAC1			7
#define IRQ_TDMA			8
#define IRQ_IPSEC_WRAP			9
#define IRQ_IPSEC_CORE			10
#define IRQ_SPI				11
#define IRQ_MDMA0			12
#define IRQ_MDMA1			13
#define IRQ_PCIe0_EXT			14
#define IRQ_PCIe0_INT			15
#define IRQ_PCIe1_EXT			16
#define IRQ_PCIe1_INT 			17
#define IRQ_DDRC			18
#define IRQ_HWFAULT			19
#define IRQ_THERMAL			20
#define IRQ_I2C			    	21
#define IRQ_ARAM			22
#define IRQ_EMAC0_BATCH			23
#define IRQ_EMAC1_BATCH			24
#define IRQ_FPP_CAP			25
#define IRQ_TIMER5			26
#define IRQ_TIMER4			27
#define IRQ_TIMER3			28
#define IRQ_TIMER2			29
#define IRQ_TIMER1			30
#define IRQ_TIMER0			31

#define STATUS_REG_0			(0 + 32)
#define IRQ_G0				(1 + 32)
#define IRQ_G1				(2 + 32)
#define IRQ_G2				(3 + 32)
#define IRQ_G3				(4 + 32)
#define IRQ_G4				(5 + 32)
#define IRQ_G5				(6 + 32)
#define IRQ_G6				(7 + 32)
#define IRQ_G7				(8 + 32)
#define IRQ_UART0			(9 + 32)
#define IRQ_TDMA_IND			(10 + 32)
#define IRQ_VED				(11 + 32)
#define IRQ_SPDRV_ACP_WORK_DONE		(12 + 32)
#define IRQ_SPDRV_ACP_REQ		(13 + 32)
						  /* also marked by IRQ_FPP_DIAG (in the helper section) */
#define IRQ_14                          (14 + 32) /* Used by fpp diagnostics. fpp diagnostics by default is always disabled */
/*	#define IRQ_Reserved		15*/
/*	#define IRQ_Reserved		16*/
#define IRQ_CSP_PMU			(17 + 32)
#define IRQ_MSP_PMU			(18 + 32)
#define IRQ_FROMHOST			(19 + 32)
#define IRQ_TOHOST			(20 + 32)
#define IRQ_CSPVED0RX			(21 + 32)
#define IRQ_CSPVED0TX			(22 + 32)
#define IRQ_CSPVED1RX			(23 + 32)
#define IRQ_CSPVED1TX			(24 + 32)
#define IRQ_CSP_HIDRV			(25 + 32)
#define IRQ_UART1			(26 + 32)
#define IRQ_TDMA_RxAHBErr   		(27 + 32)
#define IRQ_TDMA_TxAHBErr	 	(28 + 32)
/*	#define IRQ_Reserved		29*/
/*	#define IRQ_Reserved		30*/
/*	#define IRQ_Reserved		31*/
/* INTC1 32 next IRQs*/

struct comcerto_irq_desc {
	char num;
	unsigned int prio;
};

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

#endif /* _ARM_MINDSPEED_M83XXX_INTR_H_ */
