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

#ifndef _ARM_MINDSPEED_M83REG_H_
#define _ARM_MINDSPEED_M83REG_H_

/***** Physical address on AHB Bus *****/

#define AHB_ARAM_BASE			0x0A000000
#define AHB_DDRCONFIG_BASE		0x0D000000
#define AHB_IPSEC_BASE			0x0E000000
#define AHB_USB0_BASE			0x0F000000
#define AHB_APB_BASE			0x10000000
#define AHB_IBR_BASE			0x11000000
#define AHB_PCIe0CMD_BASE		0x11400000
#define AHB_PCIe1CMD_BASE		0x11410000
#define AHB_EXP_BASE			0x20000000
#define AHB_PCIe0_BASE			0x40000000
#define AHB_PCIe1_BASE			0x50000000
#define AHB_DDR_BASE			0x80000000
#define AHB_HIGHMEMDDR_BASE		0xFFFF0000

/***** Physical address of IO on APB Bus *****/

#define APB_TDM_BASE			0x10000000
#define APB_PCIe0_BASE			0x10010000
#define APB_TDMA_BASE			0x10020000
#define APB_PCIe1_BASE			0x10030000
#define APB_AHB_BASE			0x10040000
#define APB_TIMER_BASE			0x10050000
#define APB_PCIePHY_BASE		0x10060000
#define APB_GPIO_BASE			0x10070000
/*	0x10080000 Reserved*/
#define APB_UART0_BASE			0x10090000
#define APB_UART1_BASE			0x10094000
#define APB_SPI_BASE			0x10098000
#define APB_I2C_BASE			0x1009C000
#define APB_INTC_BASE			0x100A0000
#define APB_CLK_BASE			0x100B0000
/*	0x100C0000 Reserved*/
#define APB_EMAC0_BASE			0x100D0000
/*	0x100E0000 Reserved*/
#define APB_ARAM_BASE			0x100F0000
/*	0x10100000 --> 0x00180000 Reserved*/
#define APB_EMAC1_BASE			0x10190000
#define APB_EXPBUS_BASE		0x101A0000
/*	0x101B0000 --> 0x101C0000 Reserved*/
#define APB_TDMA2_BASE			0x101D0000
#define APB_MDMA_BASE			0x101E0000
/*	0x001F0000 Reserved*/

#define INTC_BASE               0x68000000
#define INTC_SIZE               0x0400

#endif /* _ARM_MINDSPEED_M83REG_H_ */
