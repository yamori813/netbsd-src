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

#ifndef _ARM_MINDSPEED_M83XXX_REG_H_
#define _ARM_MINDSPEED_M83XXX_REG_H_

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

#define	COMCERTO_APB_FREQ		165000000
#define	COMCERTO_UART_SIZE		0x4000

#define	APB_INTC_SIZE			0x0400

#define	TIMER0_HIGH_BOUND		0x0000
#define	TIMER0_CURRENT_COUNT		0x0004
#define	TIMER1_HIGH_BOUND		0x0008
#define	TIMER1_CURRENT_COUNT		0x000c
#define	TIMER2_LOW_BOUND		0x0010
#define	TIMER2_HIGH_BOUND		0x0014
#define	TIMER2_CTRL			0x0018
#define	TIMER2_CURRENT_COUNT		0x001c
#define	TIMER3_LOW_BOUND		0x0020
#define	TIMER3_HIGH_BOUND		0x0024
#define	TIMER3_CTRL			0x0028
#define	TIMER3_CURRENT_COUNT		0x002c
#define	TIMER4_HIGH_BOUND		0x0030
#define	TIMER4_CURRENT_COUNT		0x0034
#define	TIMER5_LOW_BOUND		0x0038
#define	TIMER5_HIGH_BOUND		0x003c
#define	TIMER5_CURRENT_COUNT		0x0040
#define	TIMER5_CTRL			0x0044
#define	TIMER_IRQ_MASK			0x0048
#define	TIMER_STATUS			0x0050
#define	TIMER_STATUS_CLR		0x0050
#define	TIMER_WDT_HIGH_BOUND		0x00d0
#define	TIMER_WDT_CONTROL		0x00d4
#define	TIMER_WDT_CURRENT_COUNT	0x00d8

#define	TIMER0				0x0001
#define	TIMER1				0x0002
#define	TIMER2				0x0004
#define	TIMER3				0x0008
#define	TIMER4				0x0010
#define	TIMER5				0x0020

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

#define	GPIO_OUTPUT_REG			0x00
#define	GPIO_OE_REG			0x04
#define	GPIO_INT_CFG_REG		0x08
#define	GPIO_ARM_UNALIGNED_LOGIC_ENABLE	0x0c
#define	GPIO_INPUT_REG			0x10
#define	GPIO_APB_WS			0x14
#define	GPIO_USB_PHY_CONF_REG		0x18	/* 0x002D64C2 */
#define	GPIO_SYSTEM_CONFIG		0x1c
#define	GPIO_LOCK_REG			0x38
#define	GPIO_IOCTRL_REG			0x44
/* USB PHY Built-In-Self-Test ? */
#define	GPIO_USB_PHY_BIST_STATUS_REG	0x48
#define	GPIO_GENERAL_CONTROL_REG	0x4c
#define GPIO_PIN_SELECT_REG		0x58

#define	CLK_DDR_PCIE_CLK_CNTRL		0x18

#define USB_DIV_BYPASS		(1 << 30)
#define IPSEC1_DIV_BYPASS	(1 << 29)
#define IPSEC0_DIV_BYPASS	(1 << 28)
#define PCIE_DIV_BYPASS		(1 << 27)
#define DDR_DIV_BYPASS		(1 << 26)

#define USB_DIV_VAL_OFFSET	20
#define USB_DIV_VAL_MASK	(0x3f << USB_DIV_VAL_OFFSET)

#define IPSEC_DIV1_VAL_OFFSET	16
#define IPSEC_DIV1_VAL_MASK	(0xf << IPSEC_DIV0_VAL_OFFSET)

#define IPSEC_DIV0_VAL_OFFSET	12
#define IPSEC_DIV0_VAL_MASK	(0xf << IPSEC_DIV1_VAL_OFFSET)

#define PCIE_DIV_VAL_OFFSET	8
#define PCIE_DIV_VAL_MASK	(0xf << PCIE_DIV_VAL_OFFSET)

#define DDR_DIV_VAL_OFFSET	4
#define DDR_DIV_VAL_MASK	(0xf << DDR_DIV_VAL_OFFSET)

#define USB_MUX_SEL		(1 << 3)
#define IPSEC_MUX_SEL		(1 << 2)
#define PCIE_MUX_SEL		(1 << 1)
#define DDR_MUX_SEL		(1 << 0)

#define	CLK_CLK_PWR_DWN			0x40

#define USB_REFCLK_PD		(1 << 24)
#define USB_AHBCLK_PD		(1 << 19)
#define PCIE1_AHBCLK_PD		(1 << 15)
#define PCIE0_AHBCLK_PD		(1 << 14)
#define PCIE_REFCLK_NP_PD	(1 << 6)
#define ARM0_FCLK_PD		(1 << 0)
#define ARM1_FCLK_PD		(1 << 1)
#define GEMAC0_REFCLK_PD	(1 << 2)
#define GEMAC1_REFCLK_PD	(1 << 3)
#define PHY_REFCLK_PD		(1 << 4)
#define DDR_CLK_PD		(1 << 5)
#define IPSEC_CORECLK_PD	(1 << 8)
#define ARM0_AHBCLK_PD		(1 << 9)
#define ARM1_AHBCLK_PD		(1 << 10)
#define GEMAC0_AHBCLK_PD	(1 << 11)
#define GEMAC1_AHBCLK_PD	(1 << 12)
#define DDRCTRL_AHBCLK_PD	(1 << 13)
#define TDM_AHBCLK_PD		(1 << 16)
#define MDMA_AHBCLK_PD		(1 << 17)
#define UART_AHBCLK_PD		(1 << 18)
#define I2CSPI_AHBCLK_PD	(1 << 20)
#define IPSEC_AHBCLK_PD		(1 << 21)
#define TDM_CLK_PD		(1 << 22)
#define IPSEC2_AHBCLK_PD	(1 << 23)

#define	BLOCK_RESET_REG			0x100

#define USB_REF_RESET_N		(1 << 20)
#define NO_BAL_DDR_REF_RST	(1 << 19)
#define IPSEC2_AHB_RST		(1 << 18)
#define RNG_RST			(1 << 17)
#define IPSEC_CORE_RST		(1 << 16)
#define IPSEC_AHB_RST		(1 << 15)
#define USB_AHB_RESET_N		(1 << 14)
#define TDM_REF_RST		(1 << 13)
#define TDM_AHB_RST		(1 << 12)
#define DDR_REF_RST		(1 << 11)
#define DDR_AHB_RST		(1 << 10)
#define PCIE1_REF_RST		(1 << 9)
#define PCIE0_REF_RST		(1 << 8)
#define PCIE1_AHB_RST		(1 << 7)
#define PCIE0_AHB_RST		(1 << 6)
#define GEMAC1_PHY_RST		(1 << 5)
#define GEMAC0_PHY_RST		(1 << 4)
#define GEMAC1_AHB_RST		(1 << 3)
#define GEMAC0_AHB_RST		(1 << 2)
#define ARM1_AHB_RST		(1 << 1)
#define ARM0_AHB_RST		(1 << 0)

#define	EXP_SW_RST_R			0x000
#define	EXP_CS_EN_R			0x004
#define	EXP_CSx_SEG_R			0x008
#define	EXP_CSx_CFG_R			0x01C
#define	EXP_CSx_TMG1_R			0x030
#define	EXP_CSx_TMG2_R			0x044
#define	EXP_CSx_TMG3_R			0x058
#define	EXP_CLOCK_DIV_R			0x06C

#endif /* _ARM_MINDSPEED_M83XXX_REG_H_ */
