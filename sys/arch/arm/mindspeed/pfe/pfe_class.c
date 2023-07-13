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

/**************************** CLASSIFIER ***************************/

/** Initializes CLASSIFIER block.
* @param[in] cfg	CLASSIFIER configuration
*/
void class_init(CLASS_CFG *cfg)
{
	class_reset();
	
	class_disable();
	
	class_set_config(cfg);
}

/** Resets CLASSIFIER block.
*
*/
void class_reset(void)
{
	writel(CORE_SW_RESET, CLASS_TX_CTRL);
}

/** Enables all CLASS-PE's cores.
*
*/
void class_enable(void)
{
	writel(CORE_ENABLE, CLASS_TX_CTRL);
}

/** Disables all CLASS-PE's cores.
*
*/
void class_disable(void)
{
	writel(CORE_DISABLE, CLASS_TX_CTRL); 
}

/** Sets the configuration of the CLASSIFIER block.
* @param[in] cfg	CLASSIFIER configuration
*/
void class_set_config(CLASS_CFG *cfg)
{
	if (PLL_CLK_EN == 0)
		// Clock ratio: for 1:1 the value is 0
		writel(0x0, CLASS_PE_SYS_CLK_RATIO);
	else
		// Clock ratio: for 1:2 the value is 1
		writel(0x1, CLASS_PE_SYS_CLK_RATIO);

	writel((DDR_HDR_SIZE << 16) | LMEM_HDR_SIZE, CLASS_HDR_SIZE);
	writel(LMEM_BUF_SIZE, CLASS_LMEM_BUF_SIZE);
	writel(CLASS_ROUTE_ENTRY_SIZE(CLASS_ROUTE_SIZE) |
	    CLASS_ROUTE_HASH_SIZE(cfg->route_table_hash_bits),
	    CLASS_ROUTE_HASH_ENTRY_SIZE);
	writel(HASH_CRC_PORT_IP | QB2BUS_LE, CLASS_ROUTE_MULTI);

	writel(cfg->route_table_baseaddr, CLASS_ROUTE_TABLE_BASE);
//	memset(cfg->route_table_baseaddr, 0, ROUTE_TABLE_SIZE);

	writel(CLASS_PE0_RO_DM_ADDR0_VAL, CLASS_PE0_RO_DM_ADDR0);
	writel(CLASS_PE0_RO_DM_ADDR1_VAL, CLASS_PE0_RO_DM_ADDR1);
	writel(CLASS_PE0_QB_DM_ADDR0_VAL, CLASS_PE0_QB_DM_ADDR0);
	writel(CLASS_PE0_QB_DM_ADDR1_VAL, CLASS_PE0_QB_DM_ADDR1);
	writel(CBUS_VIRT_TO_PFE(TMU_PHY_INQ_PKTPTR), CLASS_TM_INQ_ADDR);

	writel(31, CLASS_AFULL_THRES);
	writel(31, CLASS_TSQ_FIFO_THRES);
}
