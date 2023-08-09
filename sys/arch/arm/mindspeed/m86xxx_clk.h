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

#ifndef	_ARM_MINDSPEED_M86XXX_CLK_H_
#define	_ARM_MINDSPEED_M86XXX_CLK_H_

#define DEVICE_RST_CNTRL	     0x00
#define SERDES_RST_CNTRL             0x04
#define PCIe_SATA_RST_CNTRL          0x08
#define USB_RST_CNTRL                0x0C
#define A9DP_PWR_STAT                0x28
#define A9DP_PWR_CNTRL               0x2C
#define GNRL_CLK_CNTRL_0	     0x30
#define GNRL_CLK_CNTRL_1	     0x34
#define PLLS_GLOBAL_CNTRL            0x38
#define AXI_CLK_CNTRL_0              0x40
#define AXI_CLK_CNTRL_1              0x44
#define AXI_CLK_CNTRL_2              0x48
#define AXI_CLK_DIV_CNTRL            0x4C
#define AXI_RESET_0                  0x50
#define AXI_RESET_1                  0x54
#define AXI_RESET_2                  0x58
#define A9DP_MPU_CLK_CNTRL           0x68
#define A9DP_MPU_CLK_DIV_CNTRL       0x6C
#define A9DP_MPU_RESET               0x70
#define A9DP_CPU_CLK_CNTRL           0x74
#define A9DP_CPU_RESET               0x78
#define A9DP_CLK_CNTRL               0x80
#define A9DP_CLK_DIV_CNTRL           0x84
#define A9DP_RESET                   0x88
#define L2CC_CLK_CNTRL               0x90
#define L2CC_CLK_DIV_CNTRL           0x94
#define L2CC_RESET                   0x98
#define TPI_CLK_CNTRL                0xA0
#define TPI_CLK_DIV_CNTRL            0xA4
#define TPI_RESET                    0xA8
#define CSYS_CLK_CNTRL               0xB0
#define CSYS_CLK_DIV_CNTRL           0xB4
#define CSYS_RESET                   0xB8
#define EXTPHY0_CLK_CNTRL            0xC0
#define EXTPHY0_CLK_DIV_CNTRL        0xC4
#define EXTPHY0_RESET                0xC8
#define EXTPHY1_CLK_CNTRL            0xD0
#define EXTPHY1_CLK_DIV_CNTRL        0xD4
#define EXTPHY1_RESET                0xD8
#define EXTPHY2_CLK_CNTRL            0xE0
#define EXTPHY2_CLK_DIV_CNTRL        0xE4
#define EXTPHY2_RESET                0xE8
#define DDR_CLK_CNTRL                0xF0
#define DDR_CLK_DIV_CNTRL            0xF4
#define DDR_RESET                    0xF8
#define PFE_CLK_CNTRL                0x100
#define PFE_CLK_DIV_CNTRL            0x104
#define PFE_RESET                    0x108
#define IPSEC_CLK_CNTRL              0x110
#define IPSEC_CLK_DIV_CNTRL          0x114
#define IPSEC_RESET                  0x118
#define DECT_CLK_CNTRL               0x120
#define DECT_CLK_DIV_CNTRL           0x124
#define DECT_RESET                   0x128
#define GEMTX_CLK_CNTRL              0x130
#define GEMTX_CLK_DIV_CNTRL          0x134
#define GEMTX_RESET                  0x138
#define TDMNTG_REF_CLK_CNTRL         0x140
#define TDMNTG_REF_CLK_DIV_CNTRL     0x144
#define TDMNTG_RESET                 0x148
#define TDM_CLK_CNTRL                0x14C
#define TSUNTG_REF_CLK_CNTRL         0x150
#define TSUNTG_REF_CLK_DIV_CNTRL     0x154
#define TSUNTG_RESET                 0x158
#define SATA_PMU_CLK_CNTRL           0x160
#define SATA_PMU_CLK_DIV_CNTRL       0x164
#define SATA_PMU_RESET               0x168
#define SATA_OOB_CLK_CNTRL           0x170
#define SATA_OOB_CLK_DIV_CNTRL       0x174
#define SATA_OOB_RESET               0x178
#define PLL0_M_LSB                   0x1C0
#define PLL0_M_MSB                   0x1C4
#define PLL0_P                       0x1C8
#define PLL0_S                       0x1CC
#define PLL0_CNTRL                   0x1D0
#define PLL0_TEST                    0x1D4
#define PLL0_STATUS                  0x1D8
#define PLL1_M_LSB                   0x1E0
#define PLL1_M_MSB                   0x1E4
#define PLL1_P                       0x1E8
#define PLL1_S                       0x1EC
#define PLL1_CNTRL                   0x1F0
#define PLL1_TEST                    0x1F4
#define PLL1_STATUS                  0x1F8
#define PLL2_M_LSB                   0x200
#define PLL2_M_MSB                   0x204
#define PLL2_P                       0x208
#define PLL2_S                       0x20C
#define PLL2_CNTRL                   0x210
#define PLL2_TEST                    0x214
#define PLL2_STATUS                  0x218
#define PLL3_M_LSB                   0x220
#define PLL3_M_MSB                   0x224
#define PLL3_P                       0x228
#define PLL3_S                       0x22C
#define PLL3_CNTRL                   0x230
#define PLL3_TEST                    0x234
#define PLL3_STATUS                  0x238
#define PLL3_DITHER_CNTRL 	     0x23C
#define PLL3_K_LSB 		     0x240
#define PLL3_K_MSB 		     0x244
#define PLL3_MFR 		     0x248
#define PLL3_MRR 		     0x24C

/* PCIe, SATA,  and SERDES Reset bits*/ 
#define PCIE0_PWR_RST           (1 << 0)
#define PCIE0_REG_RST  	        (1 << 1)
#define PCIE1_PWR_RST           (1 << 2)
#define PCIE1_REG_RST  	        (1 << 3)

#define SATA0_RX_RST           	(1 << 4)
#define SATA0_TX_RST 	       	(1 << 5)
#define SATA1_RX_RST           	(1 << 6)
#define SATA1_TX_RST  	        (1 << 7)

#define PCIE0_AXI_RST  	        (1 << 0)
#define PCIE1_AXI_RST  	        (1 << 1)
#define SATA_AXI_RST	        (1 << 2)
#define AXI_PCIE0_CLK_EN	(1 << 0)
#define AXI_PCIE1_CLK_EN	(1 << 1)
#define AXI_SATA_CLK_EN		(1 << 2)

#define SERDES0_RST           	(1 << 0)
#define SERDES1_RST           	(1 << 1)
#define SERDES2_RST           	(1 << 2)

/* AXI_RESET_1 bits */
#define PFE_AXI_RESET		(1 << 3)

//Clock Divider mirror mechanism in IRAM. It is going to take more than 256 bytes of IRAM
//#define IRAM_CLK_REG_MIRROR	0x8300FC00 //maybe to need to think of the another fixed location
#define IRAM_CLK_REG_MIRROR	0xFC00 //maybe to need to think of the another fixed location
#define CLK_REG_DIV_BUG_BASE	AXI_CLK_DIV_CNTRL	

#define PLL_RESET			(1 << 0)
#define PLL_BYPASS			(1 << 4)
#define PLL_LOCK_EN			(1 << 5)
#define PLL_VSEL			(1 << 6)

#define CLK_DIV_BYPASS			(1 << 7)
#define CLK_A9DP_PERI_DIV_BYPASS	(1 << 3)
#define CLK_A9DP_ACP_DIV_BYPASS		(1 << 7)

#define CLK_PLL_SRC_MASK		0x7
#define CLK_PLL_SRC_SHIFT		1

#define CLK_DIV_VAL_DEFAULT		0x2
#define A9DP_ACP_CLK_DIV_VAL_DEFAULT	0x2
#define A9DP_PERI_CLK_DIV_VAL_DEFAULT	0x2

struct pll_info {
	uint32_t m;
	uint32_t p;
	uint32_t s;
	uint32_t vsel;
};

struct pll3_info {
	uint32_t m;
	uint32_t p;
	uint32_t s;
	uint32_t k;
	uint32_t vsel;
};

struct pll_setting {
        uint32_t pll0_freq_idx;
        uint32_t pll1_freq_idx;
        uint32_t pll2_freq_idx;
        uint32_t pll3_freq_idx;
};

struct clock_cfg_settings 
{
	uint32_t pll_cfg_idx;

	uint32_t arm_clk;
	uint32_t arm_clk_src;

	uint32_t axi_clk;
	uint32_t axi_clk_src;

	uint32_t ddr_clk;
	uint32_t ddr_clk_src;

	uint32_t ipsec_clk;
	uint32_t ipsec_clk_src;

	uint32_t sata_oob_clk;
	uint32_t sata_oob_clk_src;

	uint32_t sata_pmu_clk;
	uint32_t sata_pmu_clk_src;

	uint32_t dect_clk;
	uint32_t dect_clk_src;

	uint32_t l2cc_clk;
	uint32_t l2cc_clk_src;

	uint32_t hfe_clk;
	uint32_t hfe_clk_src;
	
	uint32_t gemtx_clk;
	uint32_t gemtx_clk_src;

	uint32_t extphy0_clk;
	uint32_t extphy0_clk_src;

	uint32_t extphy1_clk;
	uint32_t extphy1_clk_src;

	uint32_t extphy2_clk;
	uint32_t extphy2_clk_src;

	uint32_t tpi_clk;
	uint32_t tpi_clk_src;

	uint32_t csys_clk;
	uint32_t csys_clk_src;
};

enum {
	PLL0 = 0,
	PLL1,
	PLL2,
	PLL3
};

#define CFG_REFCLKFREQ_24	24000000        /* 24 MHz */
#define CFG_REFCLKFREQ_48	48000000        /* 48 MHz */
#define CFG_REFCLKFREQ		CFG_REFCLKFREQ_24

#define PLL_FREQ_2400	2400
#define PLL_FREQ_1800	1800
#define PLL_FREQ_1600	1600
#define PLL_FREQ_1500	1500
#define PLL_FREQ_1400	1400
#define PLL_FREQ_1300	1300
#define PLL_FREQ_1066	1066
#define PLL_FREQ_1000	1000
#define PLL_FREQ_800	800
#define PLL_FREQ_750	750
#define PLL_FREQ_500	500

enum {
	PLL_CFG_1800 = 0,
	PLL_CFG_1600,
	PLL_CFG_1500,
	PLL_CFG_1400,
	PLL_CFG_1300,
	PLL_CFG_1000,
	PLL_CFG_800,
	PLL_CFG_750,
	PLL_CFG_500,
	PLL_CFG_2400	
};

enum {
	PLL3_CFG_1066 = 0,
	PLL3_CFG_800
};

#define PLL0_CFG_2400   PLL_CFG_2400
#define PLL0_CFG_1300   PLL_CFG_1300
#define PLL0_CFG_1800   PLL_CFG_1800
#define PLL0_CFG_750    PLL_CFG_750

#define PLL1_CFG_1000   PLL_CFG_1000
#define PLL1_CFG_800    PLL_CFG_800

#define PLL2_CFG_1500   PLL_CFG_1500
#define PLL2_CFG_500    PLL_CFG_500


enum {
	CLK_CFG1 = 0,
	CLK_CFG2,
	CLK_CFG3,
	CLK_CFG4,
	CLK_CFG5,
	CLK_CFG6	
};

enum {
	PLL_CFG_1800_1000_1500_1066 = 0,
	PLL_CFG_750_1000_1500_800,
	PLL_CFG_1300_800_500_800,
	PLL_CFG_1300_800_500_1066,
	PLL_CFG_1800_800_500_1066,
	PLL_CFG_2400_1000_1500_1066	
};

#if     defined(CONFIG_M86201) || defined(CONFIG_M86202) || defined(CONFIG_M86203) || defined(CONFIG_M86204) || defined(CONFIG_M86206) || defined(CONFIG_M86207) || defined(CONFIG_M86208)
#define CLK_CFG		CLK_CFG6
#elif 	defined(CONFIG_M86260) || defined(CONFIG_M86261) || defined(CONFIG_M86262)
#define CLK_CFG		CLK_CFG3
#elif 	defined(CONFIG_M86261_NAS) 
#define CLK_CFG		CLK_CFG4
#elif   defined (CONFIG_M86291) || defined (CONFIG_M86292) || defined (CONFIG_M86293) || defined (CONFIG_M86294) || defined (CONFIG_M86295) || defined (CONFIG_M86296) || defined (CONFIG_M86297) || defined (CONFIG_M86298)
#define CLK_CFG		CLK_CFG1
#endif

#define ARM_CLK_1200		1200
#define ARM_CLK_900		900
#define ARM_CLK_750		750
#define ARM_CLK_650		650
#define AXI_CLK_250		250
#define AXI_CLK_200		200
#define DDR_CLK_533		533
#define DDR_CLK_400		400
#define SATA_OOB_CLK_125	125
#define SATA_PMU_CLK_30		30
#define SATA_PMU_CLK_25		25
#define	IPSEC_CLK_300		300
#define	IPSEC_CLK_250		250
#define DECT_CLK_250		250
#define L2CC_CLK_600		600
#define L2CC_CLK_450		450
#define L2CC_CLK_375		375
#define L2CC_CLK_325		325
#define PFE_CLK_500		500
#define PFE_CLK_400		400
#define GEMTX_CLK_125		125
#define EXTPHY0_CLK_125		125
#define EXTPHY1_CLK_125		125
#define EXTPHY2_CLK_125		125
#define TPI_CLK_250		250
#define CSYS_CLK_166		166

/*
#define USB0_PHY_CTRL_REG0           0x90410000
#define USB1_PHY_CTRL_REG0           0x90410010
#define USB_PHY_SCALEDOWN_ADDR       0x9046003C
*/
#define USB0_PHY_CTRL_REG0           0x00
#define USB1_PHY_CTRL_REG0           0x10
#define USB_PHY_SCALEDOWN_ADDR       0x3C

#define read_clk_div_bypass_backup(reg) readl2(reg - CLK_REG_DIV_BUG_BASE + IRAM_CLK_REG_MIRROR)
/*
#define write_clk_div_bypass_backup(val, reg) writel(val, reg - CLK_REG_DIV_BUG_BASE + IRAM_CLK_REG_MIRROR)
*/

#define CLK_DOMAIN_MASK			(1<<0)
#define CLK_DOMAIN_SPI_I2C_MASK		(1<<5)
#define CLK_DOMAIN_TDMNTG_MASK		(1<<4)
#define CLK_DOMAIN_UART_MASK		(1<<6)
#define CLK_DOMAIN_PCIE0_MASK		(1<<0)
#define CLK_DOMAIN_PCIE1_MASK		(1<<1)
#define CLK_DOMAIN_IPSEC_SPACC_MASK	(1<<2)
#define CLK_DOMAIN_DPI_CIE_MASK		(1<<5)
#define CLK_DOMAIN_DPI_DECOMP_MASK	(1<<6)
#define CLK_DOMAIN_USB0_MASK		(1<<3)
#define CLK_DOMAIN_USB1_MASK		(1<<4)
#define CLK_DOMAIN_DUS_MASK		(1<<0)
#define CLK_DOMAIN_SATA_MASK		(1<<2)

/* CPU */
#define CPU0_RST                        (1 << 0)
#define NEON0_RST                       (1 << 1)
#define CPU1_RST                        (1 << 2)
#define NEON1_RST                       (1 << 3)

#define CPU0_CLK_ENABLE         (1 << 0)
#define NEON0_CLK_ENABLE        (1 << 1)
#define CPU1_CLK_ENABLE         (1 << 2)
#define NEON1_CLK_ENABLE        (1 << 3)

#define GLOBAL_CLK_ENABLE       (1 << 0)

#define CLAMP_CORE0             (1 << 4)
#define CLAMP_CORE1             (1 << 6)
#define CORE_PWRDWN0        (1 << 5)
#define CORE_PWRDWN1        (1 << 7)
#define MP_PWRDWN               (1 << 0)

#define SCPRE                           (1 << 4)
#define SCALL                           (1 << 5)
#define ISO_EN                          (1 << 6)

#endif    /* _ARM_MINDSPEED_M86XXX_CLK_H_ */
