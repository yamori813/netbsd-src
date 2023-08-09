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

#ifndef _C2000_ETH_H_
#define _C2000_ETH_H_


#include "pfe_driver.h"

#ifndef SZ_1K
#define SZ_1K 1024
#endif

#ifndef SZ_1M
#define SZ_1M (1024 * 1024)
#endif

#define BMU2_DDR_BASEADDR	0
#define BMU2_BUF_COUNT		(3 * SZ_1K)
#define BMU2_DDR_SIZE		(DDR_BUF_SIZE * BMU2_BUF_COUNT)

#define TMU_LLM_BASEADDR	(BMU2_DDR_BASEADDR + BMU2_DDR_SIZE)
#define TMU_LLM_QUEUE_LEN	(16 * 256)			/**< Must be power of two and at least 16 * 8 = 128 bytes */
#define TMU_LLM_SIZE		(4 * 16 * TMU_LLM_QUEUE_LEN)	/**< (4 TMU's x 16 queues x queue_len) */

#define UTIL_CODE_BASEADDR	(TMU_LLM_BASEADDR + TMU_LLM_SIZE)
#define UTIL_CODE_SIZE		(128 * SZ_1K)

#define UTIL_DDR_DATA_BASEADDR	(UTIL_CODE_BASEADDR + UTIL_CODE_SIZE)
#define UTIL_DDR_DATA_SIZE	(64 * SZ_1K)

#define CLASS_DDR_DATA_BASEADDR	(UTIL_DDR_DATA_BASEADDR + UTIL_DDR_DATA_SIZE)
#define CLASS_DDR_DATA_SIZE	(32 * SZ_1K)

#define TMU_DDR_DATA_BASEADDR	(CLASS_DDR_DATA_BASEADDR + CLASS_DDR_DATA_SIZE)
#define TMU_DDR_DATA_SIZE	(32 * SZ_1K)

#define ROUTE_TABLE_BASEADDR	(TMU_DDR_DATA_BASEADDR + TMU_DDR_DATA_SIZE)
#define ROUTE_TABLE_HASH_BITS_MAX	15	/**< 32K entries */
#define ROUTE_TABLE_HASH_BITS	8	/**< 256 entries */
#define ROUTE_TABLE_SIZE	((1 << ROUTE_TABLE_HASH_BITS_MAX) * CLASS_ROUTE_SIZE)

#define	PFE_TOTAL_DATA_SIZE	(ROUTE_TABLE_BASEADDR + ROUTE_TABLE_SIZE)

#if PFE_TOTAL_DATA_SIZE > (12 * SZ_1M)
#error DDR mapping above 12MiB
#endif

/* LMEM Mapping */
#define BMU1_LMEM_BASEADDR	0
#define BMU1_BUF_COUNT		256
#define BMU1_LMEM_SIZE		(LMEM_BUF_SIZE * BMU1_BUF_COUNT)

#define CONFIG_DDR_PHYS_BASEADDR 0x03800000
#define CONFIG_DDR_BASEADDR      CONFIG_DDR_PHYS_BASEADDR

struct firmware {
	u8 *data;
};

int pfe_probe(struct pfe *pfe);
int pfe_remove(struct pfe *pfe);

//#define dprint(fmt, arg...)	pr_info(fmt, ##arg)
#define dprint(fmt, arg...)
//#define dprint(fmt, arg...)	pr_debug(fmt, ##arg)

#endif //_C2000_ETH_H_

