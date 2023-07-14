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

#ifndef _ARM_MINDSPEED_M86REG_H_
#define _ARM_MINDSPEED_M86REG_H_

/*
 * IRAM Location to keep C2K Part Number.
 * This location should be fixed and can not be changed.
 * Microloader populates this information to be used by Linux.
 *
 * This location shouldn't overlap IRAM_CLK_REG_MIRROR space
 * defined in clkcore.h
 */
#define IRAM_C2K_PART_NO_LOCATION	0x8300FE00


/*
 * AXI Bus
 */
#define AXI_DDR_BASE		0x00000000 /* 2G */
#define AXI_ACP_BASE		0x80000000 /* 48MB */
#define AXI_IRAM_BASE		0x83000000 /* 48MB */
#define AXI_IBR_BASE		0x90000000 /* 4MB */
#define AXI_APB_CFG_BASE	0x90400000 /* 12MB */
#define AXI_SEM_CFG_BASE	0x91000000 /* 16MB */
#define AXI_USB2p0_CFG_BASE	0x92000000 /* 16MB */
#define AXI_TZ_CFG_BASE		0x93000000 /* 16MB */
#define AXI_DPI0_CFG_BASE	0x94000000 /* 16MB */
#define AXI_DPI1_CFG_BASE	0x95000000 /* 16MB */
#define AXI_UART_SPI_CFG_BASE	0x96000000 /* 16MB */
#define AXI_DDR_CFG_BASE	0x97000000 /* 16MB */
#define AXI_PCIe0_CFG_BASE	0x98000000 /* 16MB */
#define AXI_PCIe1_CFG_BASE	0x99000000 /* 16MB */
#define AXI_ESPAH_CFG_BASE	0x9A000000 /* 16MB */
#define AXI_SPACC_CFG_BASE	0x9B000000 /* 16MB */
#define AXI_PFE_CFG_BASE	0x9C000000 /* 16MB */
#define AXI_SATA_CFG_BASE	0x9D000000 /* 16MB */
#define AXI_DECT_CFG_BASE	0x9E000000 /* 16MB */
#define AXI_USB3p0_CFG_BASE	0x9F000000 /* 16MB */
#define AXI_PCIe0_SLV_BASE	0xA0000000 /* 256MB */
#define AXI_PCIe1_SLV_BASE	0xB0000000 /* 256MB */
#define AXI_EXP_BASE		0xC0000000 /* 256MB */
#define AXI_EXP_ECC_BASE	0xCFFF0000 /* 64KB */
#define AXI_CBUS_BASE		0xC0000000 /* 256MB */

/*
 * APB Bus
 */
#define TDM_BASE			(AXI_APB_CFG_BASE + 0x000000)
#define USB_PHY_SERDES_BASE		(AXI_APB_CFG_BASE + 0x010000)
#define TDMA_BASE			(AXI_APB_CFG_BASE + 0x020000)
#define APB_RESERVED2			(AXI_APB_CFG_BASE + 0x030000)
#define APB_RESERVED3			(AXI_APB_CFG_BASE + 0x040000)
#define TIMER_BASE   			(AXI_APB_CFG_BASE + 0x050000)
#define PCIE_SATA_USB_CTRL_BASE		(AXI_APB_CFG_BASE + 0x060000)
#define GPIO_BASE			(AXI_APB_CFG_BASE + 0x070000)
#define APB_RESERVED5			(AXI_APB_CFG_BASE + 0x080000)
#define UART0_BASE   			(AXI_APB_CFG_BASE + 0x090000)
#define APB_RESERVED6			(AXI_APB_CFG_BASE + 0x094000)
#define SPI_BASE     			(AXI_APB_CFG_BASE + 0x098000)
#define I2C_BASE     			(AXI_APB_CFG_BASE + 0x09C000)
#define USB3_0_BASE  			(AXI_APB_CFG_BASE + 0x0A0000)
#define CLKCORE_BASE 			(AXI_APB_CFG_BASE + 0x0B0000)
#define APB_RESERVED7			(AXI_APB_CFG_BASE + 0x0C0000)
#define APB_RESERVED8			(AXI_APB_CFG_BASE + 0x0D0000)
#define RTC_BASE     			(AXI_APB_CFG_BASE + 0x0E0000)
#define OTP_BASE     			(AXI_APB_CFG_BASE + 0x0F0000)
#define PFEWRAPPER_BASE			(AXI_APB_CFG_BASE + 0x100000)
#define APB_RESERVED10			(AXI_APB_CFG_BASE + 0x110000)
#define APB_RESERVED11			(AXI_APB_CFG_BASE + 0x120000)
#define APB_RESERVED12			(AXI_APB_CFG_BASE + 0x130000)
#define APB_RESERVED13			(AXI_APB_CFG_BASE + 0x140000)
#define APB_RESERVED14			(AXI_APB_CFG_BASE + 0x150000)
#define APB_RESERVED15			(AXI_APB_CFG_BASE + 0x160000)
#define APB_RESERVED16			(AXI_APB_CFG_BASE + 0x170000)
#define APB_RESERVED17			(AXI_APB_CFG_BASE + 0x180000)
#define SERDES_CFG_BASE			(AXI_APB_CFG_BASE + 0x190000)
#define EXP_CONF_BASE			(AXI_APB_CFG_BASE + 0x1A0000)
#define DDR_PHY				(AXI_APB_CFG_BASE + 0x1B0000)
#define APB_RESERVED20			(AXI_APB_CFG_BASE + 0x1C0000)
#define TDMA2_BASE			(AXI_APB_CFG_BASE + 0x1D0000)
#define MDMA_BASE			(AXI_APB_CFG_BASE + 0x1E0000)
#define A9_CORESIGHT_BASE		(AXI_APB_CFG_BASE + 0x200000)

#define APB_USBPHY_SERDES_STAT_BASE	0x90410000
#define APB_GPIO_BASE			0x90470000
#define APB_CLK_BASE			0x904B0000
#define APB_SERDES_BASE			0x90590000
#define APB_EXP_BASE			0x905A0000
#define APB_DDR_BASE			0x97000000
#define APB_DDR_PHY_BASE		0x905B0000
#define SATA_AHCI_BASE			0x9D000000

#define L2CC_BASE			0xFFF10000
#define A9_PERIPH_BASE			0xFFF00000
#define A9_SCU_BASE			(A9_PERIPH_BASE + 0x0000)
#define A9_IC_INT_BASE			(A9_PERIPH_BASE + 0x100)
#define A9_TIMER_BASE			(A9_PERIPH_BASE + 0x600)
#define A9_IC_DIST_BASE			(A9_PERIPH_BASE + 0x1000)

#define APB_SERDES0_BASE		(APB_SERDES_BASE)
#define APB_SERDES1_BASE		(APB_SERDES_BASE + 0x4000)
#define APB_SERDES2_BASE		(APB_SERDES_BASE + 0x8000)
#define SER_DES0_PHY_CFG_BASE 		0x9041002C

#define EXP_SWRST			(APB_EXP_BASE + 0x0)
#define EXP_CSEN			(APB_EXP_BASE + 0x4)
#define EXP_CS0_BASE 			(APB_EXP_BASE + 0x8)
#define EXP_CS0_SEG 			(APB_EXP_BASE + 0x1c)
#define EXP_CSO_CFG			(APB_EXP_BASE + 0x30)

#define DECT_SYS_CFG0			(APB_GPIO_BASE + 0xb0)
#define DECT_SYS_CFG1			(APB_GPIO_BASE + 0xb4)
#define DECT_CTL			(APB_GPIO_BASE + 0xb8)

#define SERDES_REG( _num, _ofst) ((APB_SERDES_BASE + (0x4000 * _num)) + _ofst)

#define TEMP_STACK			STACK_BASE + STACK_SIZE - 16

#define UART_BASEADDR			0x96400000

/*
  * Reference Clock Option in Boot Strap Register
  *
  * BIT [9:8] System PLL Refclk Select
  * '00' - USB XTAL
  * '01' - Serdes #0 Refclk
  * '10' - Serdes #1 Refclk
  * '11' - Serdes XTAL
  *
  * BIT[7]   Serdes OSC PAD - Reference clock frequency selection bootstrap (inverted value of SF1)
  * '0' - 30MHz ~ 50MHz
  * '1' - 15MHz ~ 30MHz
  * Note: SF0 is tied to 1 in GPIO block and might later on be controlled during DFT
  *
  * BIT[5]   USB/Sys PLL OSC PAD - Reference clock frequency selection bootstrap (inverted value of SF1)
  *'0' - 30MHz ~ 50MHz
  * '1' - 15MHz ~ 30MHz
  * Note: SF0 is tied to 1 in GPIO block and might later on be controlled during DFT
  *
  */

#define GPIO_SYS_PLL_REF_CLK_SHIFT	8
#define GPIO_SYS_PLL_REF_CLK_MASK	(0x3 << GPIO_SYS_PLL_REF_CLK_SHIFT)

#define USB_XTAL_REF_CLK		0
#define SERDES_0_REF_CLK		1
#define SERDES_2_REF_CLK		2
#define SERDES_XTAL_REF_CLK		3

#define GPIO_SERDES_OSC_PAD_SHIFT	7
#define GPIO_SERDES_OSC_PAD_MASK	(1 << GPIO_SERDES_OSC_PAD_SHIFT)

#define GPIO_USB_OSC_PAD_SHIFT		5
#define GPIO_USB_OSC_PAD_MASK		(1 << GPIO_USB_OSC_PAD_SHIFT)

#define	COMCERTO_UART_SIZE		0x4000

#define RTC_AXI_RESET_BIT		(1<<7)
#define I2CSPI_AXI_RESET_BIT		(1<<5)
#define DUS_AXI_RESET_BIT		(1<<0)
#define TDM_AXI_RESET_BIT 		(1<<4)
#define PFE_SYS_AXI_RESET_BIT		(1<<3)
#define IPSEC_SPACC_AXI_RESET_BIT	(1<<2)
#define IPSEC_EAPE_AXI_RESET_BIT	(1<<1)
#define DPI_CIE_AXI_RESET_BIT		(1<<5)
#define DPI_DECOMP_AXI_RESET_BIT	(1<<6)
#define USB1_AXI_RESET_BIT		(1<<4)
#define USB1_PHY_RESET_BIT		(1<<4)
#define USB0_AXI_RESET_BIT		(1<<3)
#define USB0_PHY_RESET_BIT		(1<<0)
#define SATA_AXI_RESET_BIT		(1<<2)
#define SATA_PMU_RESET_BIT		(1<<0)
#define SATA_OOB_RESET_BIT		(1<<0)
#define PCIE1_AXI_RESET_BIT		(1<<1)
#define PCIE0_AXI_RESET_BIT		(1<<0)
#define PFE_CORE_RESET_BIT		(1<<0)
#define IPSEC_EAPE_CORE_RESET_BIT	(1<<0)
#define GEMTX_RESET_BIT			(1<<0)
#define L2CC_RESET_BIT			(1<<0)
#define DECT_RESET_BIT			(1<<0)
#define DDR_CNTRL_RESET_BIT		(1<<1)
#define DDR_PHY_RESET_BIT		(1<<0)
#define SERDES0_RESET_BIT		(1<<0)
#define SERDES1_RESET_BIT		(1<<1)
#define SERDES2_RESET_BIT		(1<<2)
#define SERDES_PCIE0_RESET_BIT		((1<<0)|(1<<1))
#define SERDES_PCIE1_RESET_BIT		((1<<2)|(1<<3))
#define SERDES_SATA0_RESET_BIT		((1<<4)|(1<<5))
#define SERDES_SATA1_RESET_BIT		((1<<6)|(1<<7))
#define SGMII_RESET_BIT			(1<<0)
#define USB0_UTMI_RESET_BIT		(1<<1)
#define USB1_UTMI_RESET_BIT		(1<<5)
#define TDMNTG_RESET_BIT		(1<<0)

#endif /* _ARM_MINDSPEED_M86REG_H_ */
