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
#include <arm/mindspeed/pfe/c2000_eth.h>
#include <arm/mindspeed/pfe/hal.h>
#include <arm/mindspeed/pfe/pfe_driver.h>
#include <arm/mindspeed/pfe/pfe_firmware.h>

/** PFE/Class initialization.
 */
static void pfe_class_init(struct pfe *pfe)
{
	CLASS_CFG class_cfg = {
//		.route_table_baseaddr = pfe->ddr_phys_baseaddr + ROUTE_TABLE_BASEADDR,
		.route_table_baseaddr = pge_sc->sc_ddr_pa + ROUTE_TABLE_BASEADDR,
		.route_table_hash_bits = ROUTE_TABLE_HASH_BITS,
	};

	class_init(&class_cfg);
	printk(KERN_INFO "class init complete\n");
}

/** PFE/TMU initialization.
 */
static void pfe_tmu_init(struct pfe *pfe)
{
	TMU_CFG tmu_cfg = {
//		.llm_base_addr = pfe->ddr_phys_baseaddr + TMU_LLM_BASEADDR,
		.llm_base_addr = pge_sc->sc_ddr_pa + TMU_LLM_BASEADDR,
		.llm_queue_len = TMU_LLM_QUEUE_LEN,
	};

	tmu_init(&tmu_cfg);
	printk(KERN_INFO "tmu init complete\n");
}

/** PFE/BMU (both BMU1 & BMU2) initialization.
 */
static void pfe_bmu_init(struct pfe *pfe)
{
	BMU_CFG bmu1_cfg = {
		.baseaddr = CBUS_VIRT_TO_PFE(LMEM_BASE_ADDR + BMU1_LMEM_BASEADDR),
		.count = BMU1_BUF_COUNT,
		.size = BMU1_BUF_SIZE,
	};

	BMU_CFG bmu2_cfg = {
//		.baseaddr = pfe->ddr_phys_baseaddr + BMU2_DDR_BASEADDR,
		.baseaddr = pge_sc->sc_ddr_pa + BMU2_DDR_BASEADDR,
		.count = BMU2_BUF_COUNT,
		.size = BMU2_BUF_SIZE,
	};

	bmu_init((void *)BMU1_BASE_ADDR, &bmu1_cfg);
	printk(KERN_INFO "bmu1 init: done\n");

	bmu_init((void *)BMU2_BASE_ADDR, &bmu2_cfg);
	printk(KERN_INFO "bmu2 init: done\n");
}

#if !defined(CONFIG_UTIL_PE_DISABLED)
/** PFE/Util initialization function.
 */
static void pfe_util_init(struct pfe *pfe)
{
	UTIL_CFG util_cfg = { };

	util_init(&util_cfg);
	printk(KERN_INFO "util init complete\n");
}
#endif

/** PFE/GPI initialization function.
 *  - egpi1, egpi2, egpi3, hgpi
 */
static void pfe_gpi_init(struct pfe *pfe)
{
	GPI_CFG egpi1_cfg = {
		.lmem_rtry_cnt = EGPI1_LMEM_RTRY_CNT,
		.tmlf_txthres = EGPI1_TMLF_TXTHRES,
		.aseq_len = EGPI1_ASEQ_LEN,
	};

	GPI_CFG egpi2_cfg = {
		.lmem_rtry_cnt = EGPI2_LMEM_RTRY_CNT,
		.tmlf_txthres = EGPI2_TMLF_TXTHRES,
		.aseq_len = EGPI2_ASEQ_LEN,
	};

#if 0
	GPI_CFG egpi3_cfg = {
		.lmem_rtry_cnt = EGPI3_LMEM_RTRY_CNT,
		.tmlf_txthres = EGPI3_TMLF_TXTHRES,
		.aseq_len = EGPI3_ASEQ_LEN,
	};
#endif

	GPI_CFG hgpi_cfg = {
		.lmem_rtry_cnt = HGPI_LMEM_RTRY_CNT,
		.tmlf_txthres = HGPI_TMLF_TXTHRES,
		.aseq_len = HGPI_ASEQ_LEN,
	};

	gpi_init((void *)EGPI1_BASE_ADDR, &egpi1_cfg);
	printk(KERN_INFO "GPI1 init complete\n");
	
	gpi_init((void *)EGPI2_BASE_ADDR, &egpi2_cfg);
	printk(KERN_INFO "GPI2 init complete\n");

#if 0
	gpi_init((void *)EGPI3_BASE_ADDR, &egpi3_cfg);
#endif

	gpi_init((void *)HGPI_BASE_ADDR, &hgpi_cfg);
	printk(KERN_INFO "HGPI init complete\n");
}

/** Helper function for PCI init sequence.
 */
void pfe_gemac_enable_all(void)
{
	gpi_enable((void *)EGPI1_BASE_ADDR);
	gemac_enable((void *)EMAC1_BASE_ADDR);

	gpi_enable((void *)EGPI2_BASE_ADDR);
	gemac_enable((void *)EMAC2_BASE_ADDR);

#if 0
	gpi_enable(EGPI3_BASE_ADDR);
	gemac_enable(EMAC3_BASE_ADDR);
#endif
}

/** GEMAC initialization
 * Initializes the GEMAC registers. 
 *
 * @param[in] gemac_base   Pointer to GEMAC reg base
 * @param[in] mode GEMAC mode to configure (MII config)
 * @param[in] speed GEMAC speed
 * @param[in] duplex
 */
void pfe_gemac_init(void *gemac_base, u32 mode, u32 speed, u32 duplex)
{
	GEMAC_CFG gemac_cfg  = {
		.mode = mode,
		.speed = speed,
		.duplex = duplex,
	};

	dprint("%s: gemac_base=%p\n", __func__, gemac_base);

	gemac_init(gemac_base, &gemac_cfg);
	
	//gemac_set_loop(gemac_base, LB_NONE);
	//gemac_disable_copy_all(gemac_base);
	//gemac_disable_rx_checksum_offload(gemac_base);

	gemac_allow_broadcast(gemac_base); 
	gemac_disable_unicast(gemac_base); /* unicast hash disabled  */
	gemac_disable_multicast(gemac_base); /* multicast hash disabled */
	gemac_disable_fcs_rx(gemac_base);
	gemac_disable_1536_rx(gemac_base);
	gemac_enable_pause_rx(gemac_base);
//	gemac_enable_rx_checksum_offload(gemac_base);
}

/** PFE/HIF initialization function.
 */
static void pfe_hif_init(struct pfe *pfe)
{
	paddr_t paddr;
	uint32_t reg;

	hif_tx_disable();
	hif_rx_disable();

//	hif_tx_desc_init(pfe);
//	hif_rx_desc_init(pfe);
	paddr = pge_sc->sc_txdesc_dmamap->dm_segs[0].ds_addr;
	pge_write_4(pge_sc, HIF_TX_BDP_ADDR, paddr);

	paddr = pge_sc->sc_rxdesc_dmamap->dm_segs[0].ds_addr;
	pge_write_4(pge_sc, HIF_RX_BDP_ADDR, paddr);
	reg = pge_read_4(pge_sc, HIF_RX_CTRL);
	pge_write_4(pge_sc, HIF_RX_CTRL, reg | HIF_CTRL_BDP_CH_START_WSTB);

	hif_init();

	hif_tx_enable();
	hif_rx_enable();

//	hif_rx_desc_dump();
//	hif_tx_desc_dump();

	printk(KERN_INFO "HIF init complete\n");
}

/** PFE initialization
 * - Firmware loading (CLASS-PE and TMU-PE)
 * - BMU1 and BMU2 init
 * - GEMAC init
 * - GPI init 
 * - CLASS-PE init 
 * - TMU-PE init
 * - HIF tx and rx descriptors init
 *
 * @param[in]	edev	Pointer to eth device structure.
 * 
 * @return 0, on success.
 */
static int pfe_hw_init(struct pfe *pfe)
{

	printk("%s: start \n", __func__);

	pfe_class_init(pfe);

	pfe_tmu_init(pfe);

	pfe_bmu_init(pfe);

#if !defined(CONFIG_UTIL_PE_DISABLED)
	pfe_util_init(pfe);
#endif

	pfe_gpi_init(pfe);

	pfe_hif_init(pfe);

	bmu_enable((void *)BMU1_BASE_ADDR);
	printk(KERN_INFO "bmu1 enabled\n");
	
	bmu_enable((void *)BMU2_BASE_ADDR);
	printk(KERN_INFO "bmu2 enabled\n");
	
	printk("%s: done\n", __func__);
	
	/* NOTE: Load PE specific data (if any) */

	return 0;
}

/** PFE probe function.
 * - Initializes pfe_lib
 * - pfe hw init
 * - fw loading and enables PEs
 * - should be executed once.
 * 
 * @param[in] pfe  Pointer the pfe control block
 */
int pfe_probe(struct pfe *pfe)
{
	static int init_done = 0;

	if (init_done)
		return 0;

/*
	printk(KERN_INFO "cbus_baseaddr: %p, ddr_baseaddr: %p, ddr_phys_baseaddr: %08x\n",
	             pfe->cbus_baseaddr, pfe->ddr_baseaddr, (u32)pfe->ddr_phys_baseaddr);
*/

	pfe_lib_init(pfe->cbus_baseaddr, pfe->ddr_baseaddr, pfe->ddr_phys_baseaddr);
                    

	pfe_hw_init(pfe);

	/* Load the class,TM, Util fw
    * by now pfe is,  
    * - out of reset + disabled + configured,
    * Fw loading should be done after pfe_hw_init() */
	pfe_firmware_init();

	init_done = 1;

	return 0;
}

/** PFE remove function
 *  - stopes PEs
 *  - frees tx/rx descriptor resources
 *  - should be called once.
 * 
 * @param[in] pfe Pointer to pfe control block.
 */
int pfe_remove(struct pfe *pfe)
{
/*
	if (g_tx_desc) {
		free(g_tx_desc);
	}

	if (g_rx_desc) {
		free(g_rx_desc);
	}
*/

	pfe_firmware_exit();	

	return 0;
}

