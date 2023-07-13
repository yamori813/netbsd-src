/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 * Based on DPDK Project
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the NXP nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTERS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/ioctl.h>
#include <sys/intr.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/systm.h>
#include <sys/kernel.h>

#include <arm/mindspeed/if_pgereg.h>
#include <arm/mindspeed/pfe/base/pfe.h>
#include <arm/mindspeed/pfe/pfe_hal.h>

/**************************** TMU ***************************/

/** Initializes TMU block.
* @param[in] cfg	TMU configuration
*/
void tmu_init(TMU_CFG *cfg)
{
	int q, phyno;
	writel(0x3, TMU_SYS_GENERIC_CONTROL);
	writel(750, TMU_INQ_WATERMARK);
	writel(CBUS_VIRT_TO_PFE(EGPI1_BASE_ADDR + GPI_INQ_PKTPTR),
	    TMU_PHY0_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(EGPI2_BASE_ADDR + GPI_INQ_PKTPTR),
	    TMU_PHY1_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(EGPI3_BASE_ADDR + GPI_INQ_PKTPTR),
	    TMU_PHY2_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(HGPI_BASE_ADDR + GPI_INQ_PKTPTR),
	    TMU_PHY3_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(HIF_NOCPY_RX_INQ0_PKTPTR),
	    TMU_PHY4_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(UTIL_INQ_PKTPTR),
	    TMU_PHY5_INQ_ADDR);
	writel(CBUS_VIRT_TO_PFE(BMU2_BASE_ADDR + BMU_FREE_CTRL),
	    TMU_BMU_INQ_ADDR);

	// enabling all 10 schedulers [9:0] of each TDQ 
	writel(0x3FF, TMU_TDQ0_SCH_CTRL);
	writel(0x3FF, TMU_TDQ1_SCH_CTRL);
	writel(0x3FF, TMU_TDQ2_SCH_CTRL);
	writel(0x3FF, TMU_TDQ3_SCH_CTRL);
	
	if (PLL_CLK_EN == 0)
		// Clock ratio: for 1:1 the value is 0
		writel(0x0, TMU_PE_SYS_CLK_RATIO);
	else
		// Clock ratio: for 1:2 the value is 1
		writel(0x1, TMU_PE_SYS_CLK_RATIO);
	
	// Extra packet pointers will be stored from this address onwards
	writel(cfg->llm_base_addr, TMU_LLM_BASE_ADDR);
	
	writel(cfg->llm_queue_len, TMU_LLM_QUE_LEN);
	writel(0x100, TMU_CTRL);
	writel(5, TMU_TDQ_IIFG_CFG);
	writel(DDR_BUF_SIZE, TMU_BMU_BUF_SIZE);

	// set up each queue for tail drop
	for (phyno = 0; phyno < 4; phyno++)
	{
		for (q = 0; q < 16; q++)
		{
			u32 qmax;
			writel((phyno << 8) | q, TMU_TEQ_CTRL);
			writel(1 << 22, TMU_TEQ_QCFG);
			qmax = ((phyno == 3) || (q < 8)) ? 255 : 127;
			writel(qmax << 18, TMU_TEQ_HW_PROB_CFG2);
			writel(qmax >> 14, TMU_TEQ_HW_PROB_CFG3);
		}
	}
	writel(0x05, TMU_TEQ_DISABLE_DROPCHK);
}

/** Enables TMU-PE cores.
* @param[in] pe_mask	TMU PE mask
*/
void tmu_enable(u32 pe_mask)
{
	writel(readl(TMU_TX_CTRL) | (pe_mask & 0xF), TMU_TX_CTRL);
}

/** Disables TMU cores.
* @param[in] pe_mask	TMU PE mask
*/
void tmu_disable(u32 pe_mask)
{
	writel(readl(TMU_TX_CTRL) & ((~pe_mask) & 0xF), TMU_TX_CTRL);
}
