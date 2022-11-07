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
#ifndef _ARM_IMX_IMX31_INTR_H_
#define _ARM_IMX_IMX31_INTR_H_

#define IRQ_PTP0				1
#define IRQ_PTP1				2
#define IRQ_PTP2				3
#define IRQ_EMAC0				4
#define IRQ_PUI					5
#define IRQ_PUDMARX				6
#define IRQ_PUDMATX				7
#define IRQ_TDMA1				8
#define IRQ_IPSEC				9
#define IRQ_I2C					10
#define IRQ_USB0				11
#define IRQ_EMAC1				12
#define IRQ_USB1				13
#define IRQ_SPI					14
#define IRQ_TDM_TIMER				15
#define IRQ_HIF					16
#define IRQ_HIF_RXEMT				17
#define IRQ_HITXFUL				18
#define IRQ_EDMA0RX				19
#define IRQ_EDMA0TX				20
#define IRQ_APBB				21
#define IRQ_DDRC				22
#define IRQ_IPSEC_WRAP				23
/*	#define IRQ_Reserved			24 */
#define IRQ_EDMA1RX				25
#define IRQ_EDMA1TX				26
#define IRQ_MDMA_DONE				27
#define IRQ_28					28 /* Used by fpp Diagnostics */
/*	#define IRQ_Reserved			29 */
#define IRQ_TIMERB				30				/* This IRQ is used only by the CSP */
#define IRQ_TIMERA				31				/* This IRQ is used only by the MSP */
#define IRQ_G0					(1 + 32)
#define IRQ_G1					(2 + 32)
#define IRQ_G2					(3 + 32)
#define IRQ_G3					(4 + 32)
#define IRQ_G4					(5 + 32)
#define IRQ_G5					(6 + 32)
#define IRQ_G6					(7 + 32)
#define IRQ_G7					(8 + 32)
#define IRQ_UART0				(9 + 32)
#define IRQ_TDMA0				(10 + 32)
#define IRQ_TOFPP_DMA				(11 + 32)
#define IRQ_SPDRV_ACP_WORK_DONE			(12 + 32)
#define IRQ_SPDRV_ACP_REQ			(13 + 32)
#define IRQ_L2_BWABT				(14 + 32)
#define IRQ_L2_PARRD				(15 + 32)
#define IRQ_L2_PARRT				(16 + 32)
#define IRQ_CSP_PMU				(17 + 32)
#define IRQ_MSP_PMU				(18 + 32)
#define IRQ_FROMHOST				(19 + 32)
#define IRQ_TOHOST				(20 + 32)
#define IRQ_VDMA0_RX				(21 + 32)
#define IRQ_VDMA0_TX				(22 + 32)
#define IRQ_VDMA1_RX				(23 + 32)
#define IRQ_VDMA1_TX				(24 + 32)
#define IRQ_FPP					(25 + 32)
#define IRQ_UART1				(26 + 32)
#define IRQ_TDMA_RX_AHB_ERR			(27 + 32)
#define IRQ_TDMA_TX_AHB_ERR			(28 + 32)
#define IRQ_IDMA_RX_AHB_ERR			(29 + 32)
#define IRQ_IDMA_TX_AHB_ERR			(30 + 32)
#define IRQ_MDMA_AHB_ERR			(31 + 32) 

#ifdef _LOCORE

#define	ARM_IRQ_HANDLER	_C_LABEL(imx31_irq_handler)

#else

#define AVIC_INTR_SOURCE_NAMES \
{	"reserved 0",	"reserved 1",	"reserved 2",	"i2c #3",	\
	"i2c #2",	"mpeg4 enc",	"rtic",		"fir", 		\
	"mm/sd hc #2",	"mm/sd hc #1",	"i2c #1",	"ssi #2",	\
	"ssi #1",	"cspi #2",	"cspi #1",	"ata",		\
	"mbx rs",	"cspi #3",	"uart #3",	"i2c id",	\
	"sim #1",	"sim #2",	"rnga",		"evtmon",	\
	"kpp",		"rtc",		"pwm",		"epit #2",	\
	"epit #1",	"gpt",		"pwrfail",	"ccm dvfs",	\
	"uart #2",	"nandfc",	"sdma",		"usb hc #1",	\
	"usb hc #2",	"usb otg",	"reserved 38",	"ms hc #1",	\
	"ms hc#2",	"ipu err",	"ipu",		"reserved 43",	\
	"reserved 44",	"uart #1",	"uart #4",	"uart #5",	\
	"ect",		"scc scm",	"scc smn",	"gpio #2",	\
	"gpio #1",	"ccm",		"pcmcia",	"wdog",		\
	"gpio #3",	"reserved 57",	"ext pwrmgt",	"ext temp",	\
	"ext sens #2",	"ext sens #1",	"ext wdog",	"ext tv", }

#define	PIC_MAXMAXSOURCES	(64+3*32+128)

#include <arm/pic/picvar.h>

const char *
	intr_typename(int);

void imx31_irq_handler(void *);

#endif /* !_LOCORE */

#endif /* _ARM_IMX_IMX31_INTR_H_ */
