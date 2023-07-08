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

/**************************** UTIL ***************************/

/** Resets UTIL block.
*/
void util_reset(void)
{
	writel(CORE_SW_RESET, UTIL_TX_CTRL);
}

/** Initializes UTIL block.
* @param[in] cfg	UTIL configuration
*/
void util_init(UTIL_CFG *cfg)
{

	if (PLL_CLK_EN == 0)
		// Clock ratio: for 1:1 the value is 0
		writel(0x0, UTIL_PE_SYS_CLK_RATIO);
	else
		// Clock ratio: for 1:2 the value is 1
		writel(0x1, UTIL_PE_SYS_CLK_RATIO);
}

/** Enables UTIL-PE core.
*
*/
void util_enable(void)
{
	writel(CORE_ENABLE, UTIL_TX_CTRL);
}

/** Disables UTIL-PE core.
*
*/
void util_disable(void)
{
	writel(CORE_DISABLE, UTIL_TX_CTRL);
}

#ifdef NOTUSE
/** GEMAC PHY Statistics - This function return address of the first
* statistics register
* @param[in]	base	GEMAC base address 
*/
unsigned int * gemac_get_stats(void *base)
{
	return (unsigned int *)(base + EMAC_OCT_TX_BOT);
}
#endif   /* NOTUSE */
