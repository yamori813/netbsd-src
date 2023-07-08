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
#include <arm/mindspeed/pfe/hal.h>

/**************************** GPI ***************************/

/** Initializes a GPI block.
* @param[in] base	GPI base address
* @param[in] cfg	GPI configuration
*/
void gpi_init(void *base, GPI_CFG *cfg)
{ 
	gpi_reset(base);
	
	gpi_disable(base);
	
	gpi_set_config(base, cfg);
}

/** Resets a GPI block.
* @param[in] base	GPI base address
*/
void gpi_reset(void *base)
{
	writel (CORE_SW_RESET, (int)base + GPI_CTRL);
}

/** Enables a GPI block.
* @param[in] base	GPI base address
*/
void gpi_enable(void *base)
{
	writel (CORE_ENABLE, (int)base + GPI_CTRL);
}

/** Disables a GPI block.
* @param[in] base	GPI base address
*/
void gpi_disable(void *base)
{
	writel (CORE_DISABLE, (int)base + GPI_CTRL);
}


/** Sets the configuration of a GPI block.
* @param[in] base	GPI base address
* @param[in] cfg	GPI configuration
*/
void gpi_set_config(void *base, GPI_CFG *cfg)
{  
	writel (CBUS_VIRT_TO_PFE(BMU1_BASE_ADDR + BMU_ALLOC_CTRL),
	    (int)base + GPI_LMEM_ALLOC_ADDR);
	writel (CBUS_VIRT_TO_PFE(BMU1_BASE_ADDR + BMU_FREE_CTRL),
	    (int)base + GPI_LMEM_FREE_ADDR);
	writel (CBUS_VIRT_TO_PFE(BMU2_BASE_ADDR + BMU_ALLOC_CTRL),
	    (int)base + GPI_DDR_ALLOC_ADDR);
	writel (CBUS_VIRT_TO_PFE(BMU2_BASE_ADDR + BMU_FREE_CTRL),
	    (int)base + GPI_DDR_FREE_ADDR);
	writel (CBUS_VIRT_TO_PFE(CLASS_INQ_PKTPTR),(int)base + GPI_CLASS_ADDR);
 	writel (DDR_HDR_SIZE,(int)base + GPI_DDR_DATA_OFFSET);
	writel (LMEM_HDR_SIZE,(int)base + GPI_LMEM_DATA_OFFSET);
	writel (0, (int)base + GPI_LMEM_SEC_BUF_DATA_OFFSET);
	writel (0, (int)base + GPI_DDR_SEC_BUF_DATA_OFFSET);
	writel ((DDR_HDR_SIZE << 16) | LMEM_HDR_SIZE,(int)base + GPI_HDR_SIZE);
	writel ((DDR_BUF_SIZE << 16) | LMEM_BUF_SIZE,(int)base + GPI_BUF_SIZE);
	
	writel (((cfg->lmem_rtry_cnt << 16) | (GPI_DDR_BUF_EN << 1) |
	    GPI_LMEM_BUF_EN), (int)base + GPI_RX_CONFIG);
	writel (cfg->tmlf_txthres,(int)base + GPI_TMLF_TX);
	writel (cfg->aseq_len,(int)base + GPI_DTX_ASEQ);
}
