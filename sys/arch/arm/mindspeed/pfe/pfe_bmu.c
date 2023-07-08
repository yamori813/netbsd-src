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

/**************************** BMU ***************************/

/** Initializes a BMU block.
* @param[in] base	BMU block base address
* @param[in] cfg	BMU configuration
*/
void bmu_init(void *base, BMU_CFG *cfg)
{
	bmu_reset(base);

	bmu_disable(base);

	bmu_set_config(base, cfg);
}

/** Resets a BMU block.
* @param[in] base	BMU block base address
*/
void bmu_reset(void *base)
{
	writel(CORE_SW_RESET, (int)base + BMU_CTRL);
}

/** Enabled a BMU block.
* @param[in] base	BMU block base address
*/
void bmu_enable(void *base)
{
	writel (CORE_ENABLE, (int)base + BMU_CTRL);
}

/** Disables a BMU block.
* @param[in] base	BMU block base address
*/
void bmu_disable(void *base)
{
	writel (CORE_DISABLE, (int)base + BMU_CTRL);
}

/** Sets the configuration of a BMU block.
* @param[in] base	BMU block base address
* @param[in] cfg	BMU configuration
*/
void bmu_set_config(void *base, BMU_CFG *cfg)
{	
	writel (cfg->baseaddr, (int)base + BMU_UCAST_BASE_ADDR);
	writel (cfg->count & 0xffff, (int)base + BMU_UCAST_CONFIG);
	writel (cfg->size & 0xffff, (int)base + BMU_BUF_SIZE);
//	writel (BMU1_THRES_CNT, (int)base + BMU_THRES);

	/* Interrupts are never used */
//	writel (0x0, (int)base + BMU_INT_SRC);
	writel (0x0, (int)base + BMU_INT_ENABLE);
}
