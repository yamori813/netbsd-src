/*	$NetBSD$	*/
/*-
 * Copyright (c) 2012 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry.
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

#include "opt_arm_debug.h"
#include "opt_m86.h"
#include "arml2cc.h"

#define	_ARM32_BUS_DMA_PRIVATE

#include <sys/cdefs.h>

__KERNEL_RCSID(1, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/device.h>

#include <prop/proplib.h>

#include <net/if.h>
#include <net/if_ether.h>

#include <arm/cortex/a9tmr_var.h>
#include <arm/mainbus/mainbus.h>

#include <arm/mindspeed/m86xxx_reg.h>
#include <arm/mindspeed/m86xxx_var.h>
#include <arm/mindspeed/m86xxx_clk.h>

//static struct cpu_softc cpu_softc;
struct m86xxx_clock_info clock_info;

static vaddr_t baseaddr;

static uint32_t readl(int off);
static uint32_t readl(int off)
{

	return *(uint32_t *)(baseaddr + CLKCORE_BASE + off);
}

static void writel(int off, uint32_t val);
static void writel(int off, uint32_t val)
{

	*(uint32_t *)(baseaddr + CLKCORE_BASE + off) = val;
}

static uint32_t readl2(int off);
static uint32_t readl2(int off)
{

	return *(uint32_t *)(baseaddr + 0x100000 + off);
}

static void write_usb3(int off, uint32_t val);
static void write_usb3(int off, uint32_t val)
{

	*(uint32_t *)(baseaddr + USB3_0_BASE + off) = val;
}

static void write_usb2(int off, uint32_t val);
static void write_usb2(int off, uint32_t val)
{

	*(uint32_t *)(baseaddr + USB_PHY_SERDES_BASE + off) = val;
}

static uint32_t readl_dwc(int off);
static uint32_t readl_dwc(int off)
{

	return *(uint32_t *)(baseaddr + PCIE_SATA_USB_CTRL_BASE + off);
}

static void writel_dwc(int off, uint32_t val);
static void writel_dwc(int off, uint32_t val)
{

	*(uint32_t *)(baseaddr + PCIE_SATA_USB_CTRL_BASE + off) = val;
}

static uint32_t m86xxx_get_pll_freq(int pll_no);
static uint32_t m86xxx_get_pll_freq(int pll_no)
{
	uint32_t p;
	uint32_t od;
	uint32_t m;
	uint32_t k;
	uint32_t s;
	uint32_t pll_clk = 0;

	if (pll_no < PLL3)
	{
		/* get NF, NR and OD values */
		switch (pll_no)
		{
		case PLL0:
			m = readl(PLL0_M_LSB) & 0xff;
			m |= (readl(PLL0_M_MSB) & 0x3) << 8;
			p = readl(PLL0_P) & 0x3f;
			s = readl(PLL0_S) & 0x7;
			od = (1 << s); // 2^s;
			break;

		case PLL1:
			m = readl(PLL1_M_LSB) & 0xff;
			m |= (readl(PLL1_M_MSB) & 0x3) << 8;
			p = readl(PLL1_P) & 0x3f;
			s = readl(PLL1_S) & 0x7;
			od = (1 << s);
			break;

		case PLL2:
			m = readl(PLL2_M_LSB) & 0xff;
			m |= (readl(PLL2_M_MSB) & 0x3) << 8;
			p = readl(PLL2_P) & 0x3f;
			s = readl(PLL2_S) & 0x7;
			od = (1 << s);
			break;

		default:
			return 0;
			break;
		}

		/*
		 * Ref Clock divided by 1000000. It should be displayed
		 * in MHz.
		 */
		pll_clk = ((CFG_REFCLKFREQ / 1000000) * m) / p / od ;
	}
	else if (pll_no == PLL3)
	{
		m = readl(PLL3_M_LSB) & 0xff;
		m |= (readl(PLL3_M_MSB) & 0x3) << 8;
		p = readl(PLL3_P) & 0x3f;
		s = readl(PLL3_S) & 0x7;
		k = readl(PLL3_K_LSB) & 0xff;
		k |= (readl(PLL3_K_MSB) & 0xf) << 8;
		od = (1 << s);

		pll_clk = (((CFG_REFCLKFREQ / 1000000) * (m * 1024 + k)) /
		    p / od + 1023) / 1024;
	}

	return pll_clk;
}

static inline uint32_t m86xxx_get_clk_freq(uint32_t ctrl_reg, uint32_t div_reg);
static inline uint32_t m86xxx_get_clk_freq(uint32_t ctrl_reg, uint32_t div_reg)
{
	uint32_t pll_src;
	uint32_t pll_clk;
	uint32_t clk_div;
	uint32_t clk_out;
	int bypass = 0;

	/* get PLL source */
	pll_src = readl(ctrl_reg);
	pll_src = (pll_src >> CLK_PLL_SRC_SHIFT) & CLK_PLL_SRC_MASK;

	/*
	 * get clock divider bypass value from IRAM Clock Divider registers
	 * mirror location
	 */
	clk_div = read_clk_div_bypass_backup(div_reg);

	if (clk_div & CLK_DIV_BYPASS)
		bypass = 1;
	else
	{
		clk_div = readl(div_reg);
		clk_div &= 0x1f;
	}

	pll_clk = m86xxx_get_pll_freq(pll_src);

	if (bypass)
		clk_out = pll_clk;
	else
		clk_out = pll_clk / clk_div;

	return clk_out;
}

static uint32_t m86xxx_get_arm_clk(void);
static uint32_t m86xxx_get_arm_clk(void)
{

	return m86xxx_get_clk_freq(A9DP_CLK_CNTRL, A9DP_CLK_DIV_CNTRL);
}

static uint32_t m86xxx_get_axi_clk(void);
static uint32_t m86xxx_get_axi_clk(void)
{       

	return m86xxx_get_clk_freq(AXI_CLK_CNTRL_0, AXI_CLK_DIV_CNTRL);
}


void
m86xxx_bootstrap(vaddr_t iobase)
{

	baseaddr = iobase;

	/* XXX why double value? */
	clock_info.clk_arm = m86xxx_get_arm_clk() * 1000 * 1000 * 2;
	clock_info.clk_axi = m86xxx_get_axi_clk() * 1000 * 1000 * 2;

	curcpu()->ci_data.cpu_cc_freq = clock_info.clk_arm;

	/* Enable  clock USB2 and USB3 */

	uint32_t reg = readl(AXI_CLK_CNTRL_2);
	reg |= (CLK_DOMAIN_USB0_MASK | CLK_DOMAIN_USB1_MASK);
	writel(AXI_CLK_CNTRL_2, reg);

	/* USB3 */

	reg = readl(USB_RST_CNTRL);
	reg |= USB1_PHY_RESET_BIT;
	writel(USB_RST_CNTRL, reg);
	reg = readl(USB_RST_CNTRL);
	reg |= USB1_UTMI_RESET_BIT;
	writel(USB_RST_CNTRL, reg);
	reg = readl(AXI_RESET_2);
	reg |= USB1_AXI_RESET_BIT;
	writel(AXI_RESET_2, reg);

#if 0
	write_usb3(0x20, 0x420E82A8);	/* 24MHz */
#else
	write_usb3(0x20, 0x420E82A9);	/* 48MHz */
#endif
	write_usb3(0x24, 0x69C34F53);
	write_usb3(0x28, 0x0005D815);
	write_usb3(0x2C, 0x00000801);

	reg = readl(USB_RST_CNTRL);
	reg &= ~USB1_PHY_RESET_BIT;
	writel(USB_RST_CNTRL, reg);
	reg = readl(USB_RST_CNTRL);
	reg &= ~USB1_UTMI_RESET_BIT;
	writel(USB_RST_CNTRL, reg);
	reg = readl(AXI_RESET_2);
	reg &= ~USB1_AXI_RESET_BIT;
	writel(AXI_RESET_2, reg);

	/* USB2 OTG PHY init */

#if 0
	write_usb2(USB0_PHY_CTRL_REG0, 0x00210000);   /* 24MHz */
#else
	write_usb2(USB0_PHY_CTRL_REG0, 0x00220000);   /* 48MHz */
#endif

	reg = readl_dwc(USB_PHY_SCALEDOWN_ADDR);
	reg = ((reg & 0xffff11ff) | 0x00001100);
	writel_dwc(USB_PHY_SCALEDOWN_ADDR, reg);

	reg = readl_dwc(USB_PHY_SCALEDOWN_ADDR);
	reg = ((reg & 0xfffffff0) | 0x0);
	writel_dwc(USB_PHY_SCALEDOWN_ADDR, reg);

	writel(USB_RST_CNTRL, readl(USB_RST_CNTRL) | USB0_UTMI_RESET_BIT);
	writel(USB_RST_CNTRL, readl(USB_RST_CNTRL) | USB0_PHY_RESET_BIT);
	writel(AXI_RESET_2, readl(AXI_RESET_2) | USB0_AXI_RESET_BIT);

	writel(USB_RST_CNTRL, readl(USB_RST_CNTRL) & ~USB0_PHY_RESET_BIT);
	writel(USB_RST_CNTRL, readl(USB_RST_CNTRL) & ~USB0_UTMI_RESET_BIT);
	writel(AXI_RESET_2, readl(AXI_RESET_2) & ~USB0_AXI_RESET_BIT);

	/* PFE Reset move to m86xxx_clk */

/*
	reg = readl(PFE_CLK_CNTRL);
	reg &=~CLK_DOMAIN_MASK;
	writel(PFE_CLK_CNTRL, reg);

	reg = readl(AXI_RESET_1);
	writel(AXI_RESET_1, reg | PFE_SYS_AXI_RESET_BIT);
	delay(1000);
	writel(AXI_RESET_1, reg & ~PFE_SYS_AXI_RESET_BIT);

	reg = readl(PFE_CLK_CNTRL);
	reg |= CLK_DOMAIN_MASK;
	writel(PFE_CLK_CNTRL, reg);

	writel(PFE_RESET, PFE_CORE_RESET_BIT);
	writel(GEMTX_RESET, GEMTX_RESET_BIT);
	delay(10);
	writel(PFE_RESET, 0);
	writel(GEMTX_RESET, 0);
*/
}

void
m86xxx_device_register(device_t self, void *aux)
{
	prop_dictionary_t dict = device_properties(self);

	if (device_is_a(self, "armperiph")
	    && device_is_a(device_parent(self), "mainbus")) {
		/*
		 * XXX KLUDGE ALERT XXX
		 * The iot mainbus supplies is completely wrong since it scales
		 * addresses by 2.  The simplest remedy is to replace with our
		 * bus space used for the armcore registers (which armperiph
		 *  uses).
		 */
		struct mainbus_attach_args * const mb = aux;
		mb->mb_iot = &m83_bs_tag;
		return;
	}

	/*
	 * We need to tell the A9 Global/Watchdog Timer
	 * what frequency it runs at.
	 */
	if (device_is_a(self, "arma9tmr") || device_is_a(self, "a9wdt")) {
		/*
		 * This clock always runs at (arm_clk div 2) and only goes
		 * to timers that are part of the A9 MP core subsystem.
		 */
                prop_dictionary_set_uint32(dict, "frequency",
		    clock_info.clk_arm / 2);
		return;
	}

}

#ifdef MULTIPROCESSOR
void
m86xxx_cpu_hatch(struct cpu_info *ci)
{
	a9tmr_init_cpu_clock(ci);
}

void
m86xxx_cpu1_reset(void)
{
	uint32_t reg;

	reg = readl(A9DP_CPU_RESET);
	writel(A9DP_CPU_RESET, reg & ~CPU1_RST);

	reg = readl(A9DP_PWR_CNTRL);
	writel(A9DP_PWR_CNTRL, reg & ~CLAMP_CORE1);

	reg = readl(A9DP_CPU_CLK_CNTRL);
	writel(A9DP_CPU_CLK_CNTRL, reg | CPU1_CLK_ENABLE);

	reg = readl(A9DP_CPU_RESET);
	writel(A9DP_CPU_RESET, reg & ~NEON1_RST);

	reg = readl(A9DP_CPU_CLK_CNTRL);
	writel(A9DP_CPU_CLK_CNTRL, reg | NEON1_CLK_ENABLE);
}
#endif

