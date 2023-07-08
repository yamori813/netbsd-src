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

/**************************** GEMAC ***************************/

/** MAC Address converter
 * Convert standard byte style ethernet address to format compatible with MAC.
 *  
 * @param[in] enet_byte_addr    Pointer to the mac address in byte format
 * @param[out] Pointer to MAC_ADDR structure
 *  
 * @return      0 on success, -1 on failure
 */
int gemac_enet_addr_byte_mac(u8 *enet_byte_addr, MAC_ADDR *enet_addr)
{
	if ((enet_byte_addr == NULL) || (enet_addr == NULL))
	{
		return -1;
	}
	else
	{
	enet_addr->bottom = enet_byte_addr[0] |
	    (enet_byte_addr[1] << 8) |
	    (enet_byte_addr[2] << 16) |
	    (enet_byte_addr[3] << 24);
	enet_addr->top = enet_byte_addr[4] |
	    (enet_byte_addr[5] << 8);
	return 0;
	}
}


/** GEMAC block initialization.
* @param[in] base	GEMAC base address (GEMAC0, GEMAC1, GEMAC2)
* @param[in] cfg	GEMAC configuration
*/
void gemac_init(void *base, void *cfg)
{
	gemac_set_config(base, cfg);
	gemac_set_bus_width(base, 64);
}

/** GEMAC set speed.
* @param[in] (int)base	GEMAC base address
* @param[in] speed	GEMAC speed (10, 100 or 1000 Mbps)
*/
void gemac_set_speed(void *base, MAC_SPEED gem_speed)
{
	u32 val = readl((int)base + EMAC_NETWORK_CONFIG);

	val = val & ~EMAC_SPEED_MASK;

	switch (gem_speed)
	{
		case SPEED_10M:
			val &= (~EMAC_PCS_ENABLE);
			break;

		case SPEED_100M:
			val = val | EMAC_SPEED_100;
			val &= (~EMAC_PCS_ENABLE);
			break;

		case SPEED_1000M:
			val = val | EMAC_SPEED_1000;
			val &= (~EMAC_PCS_ENABLE);
			break;

		case SPEED_1000M_PCS:
			val = val | EMAC_SPEED_1000;
			val |= EMAC_PCS_ENABLE;
			break;

		default:
			val = val | EMAC_SPEED_100;
			val &= (~EMAC_PCS_ENABLE);
		break;
	}
	
	writel (val, (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC set duplex.
* @param[in] (int)base	GEMAC base address
* @param[in] duplex	GEMAC duplex mode (Full, Half)
*/
void gemac_set_duplex(void *base, int duplex)
{
	u32 val = readl((int)base + EMAC_NETWORK_CONFIG);

	if (duplex == DUPLEX_HALF)
		val = (val & ~EMAC_DUPLEX_MASK) | EMAC_HALF_DUP;
	else
		val = (val & ~EMAC_DUPLEX_MASK) | EMAC_FULL_DUP;
  
	writel (val, (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC set mode.
* @param[in] (int)base	GEMAC base address
* @param[in] mode	GEMAC operation mode (MII, RMII, RGMII, SGMII)
*/
void gemac_set_mode(void *base, int mode)
{
	switch (mode)
	{
	case GMII:
		writel ((readl((int)base + EMAC_CONTROL) & ~EMAC_MODE_MASK) | EMAC_GMII_MODE_ENABLE, (int)base + EMAC_CONTROL);
		writel (readl((int)base + EMAC_NETWORK_CONFIG) & (~EMAC_SGMII_MODE_ENABLE), (int)base + EMAC_NETWORK_CONFIG);
		break;

	case RGMII:
		writel ((readl((int)base + EMAC_CONTROL) & ~EMAC_MODE_MASK) | EMAC_RGMII_MODE_ENABLE, (int)base + EMAC_CONTROL);
		writel (readl((int)base + EMAC_NETWORK_CONFIG) & (~EMAC_SGMII_MODE_ENABLE), (int)base + EMAC_NETWORK_CONFIG);
		break;

	case RMII:
		writel ((readl((int)base + EMAC_CONTROL) & ~EMAC_MODE_MASK) | EMAC_RMII_MODE_ENABLE, (int)base + EMAC_CONTROL);
		writel (readl((int)base + EMAC_NETWORK_CONFIG) & (~EMAC_SGMII_MODE_ENABLE), (int)base + EMAC_NETWORK_CONFIG);
		break;

	case MII:
		writel ((readl((int)base + EMAC_CONTROL) & ~EMAC_MODE_MASK) | EMAC_MII_MODE_ENABLE, (int)base + EMAC_CONTROL);
		writel (readl((int)base + EMAC_NETWORK_CONFIG) & (~EMAC_SGMII_MODE_ENABLE), (int)base + EMAC_NETWORK_CONFIG);
		break;

	case SGMII:
		writel ((readl((int)base + EMAC_CONTROL) & ~EMAC_MODE_MASK) | (EMAC_RMII_MODE_DISABLE | EMAC_RGMII_MODE_DISABLE), (int)base + EMAC_CONTROL);
		writel (readl((int)base + EMAC_NETWORK_CONFIG) | EMAC_SGMII_MODE_ENABLE, (int)base + EMAC_NETWORK_CONFIG);
		break;

	default:
		writel ((readl((int)base + EMAC_CONTROL) & ~EMAC_MODE_MASK) | EMAC_MII_MODE_ENABLE, (int)base + EMAC_CONTROL);
		writel (readl((int)base + EMAC_NETWORK_CONFIG) & (~EMAC_SGMII_MODE_ENABLE), (int)base + EMAC_NETWORK_CONFIG);
		break;
	}
}

/** GEMAC Enable MDIO: Activate the Management interface.  This is required to program the PHY
 * @param[in] base       GEMAC base address
 */
void gemac_enable_mdio(void *base)
{
        u32 data;

        data = readl((int)base + EMAC_NETWORK_CONTROL);
        data |= EMAC_MDIO_EN;
        writel(data, (int)base + EMAC_NETWORK_CONTROL);
}

/** GEMAC Disable MDIO: Disable the Management interface.
 * @param[in] base       GEMAC base address
 */
void gemac_disable_mdio(void *base)
{
        u32 data;

        data = readl((int)base + EMAC_NETWORK_CONTROL);
        data &= ~EMAC_MDIO_EN;
        writel(data, (int)base + EMAC_NETWORK_CONTROL);
}

/** GEMAC Set MDC clock division
 * @param[in] base       GEMAC base address
 * @param[in] base       MDC divider value
 */
void gemac_set_mdc_div(void *base, MAC_MDC_DIV gem_mdcdiv)
{
        u32 data;

        data = readl((int)base + EMAC_NETWORK_CONFIG);
	data &= ~(MDC_DIV_MASK << MDC_DIV_SHIFT);
        data |= (gem_mdcdiv & MDC_DIV_MASK) << MDC_DIV_SHIFT;
        writel(data, (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC reset function.
* @param[in] base	GEMAC base address
*/
void gemac_reset(void *base)
{  
}

/** GEMAC enable function.
* @param[in] base	GEMAC base address
*/
void gemac_enable(void *base)
{  
	writel (readl((int)base + EMAC_NETWORK_CONTROL) | EMAC_TX_ENABLE | EMAC_RX_ENABLE, (int)base + EMAC_NETWORK_CONTROL);
}

/** GEMAC disable function.
* @param[in] base	GEMAC base address
*/
void gemac_disable(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONTROL) & ~(EMAC_TX_ENABLE | EMAC_RX_ENABLE), (int)base + EMAC_NETWORK_CONTROL);
}

/** GEMAC set mac address configuration.
* @param[in] base	GEMAC base address
* @param[in] addr	MAC address to be configured
*/
void gemac_set_address(void *base, SPEC_ADDR *addr)
{ 
	writel(addr->one.bottom, (int)base + EMAC_SPEC1_ADD_BOT);
	writel(addr->one.top, (int)base + EMAC_SPEC1_ADD_TOP); 
	writel(addr->two.bottom, (int)base + EMAC_SPEC2_ADD_BOT);
	writel(addr->two.top, (int)base + EMAC_SPEC2_ADD_TOP);
	writel(addr->three.bottom, (int)base + EMAC_SPEC3_ADD_BOT);
	writel(addr->three.top, (int)base + EMAC_SPEC3_ADD_TOP);
	writel(addr->four.bottom, (int)base + EMAC_SPEC4_ADD_BOT);
	writel(addr->four.top, (int)base + EMAC_SPEC4_ADD_TOP);
} 

/** GEMAC get mac address configuration.
* @param[in] base	GEMAC base address
*
* @return		MAC addresses configured
*/
SPEC_ADDR gemac_get_address(void *base)
{
	SPEC_ADDR addr;
	
	addr.one.bottom = readl((int)base + EMAC_SPEC1_ADD_BOT);
	addr.one.top = readl((int)base + EMAC_SPEC1_ADD_TOP); 
	addr.two.bottom = readl((int)base + EMAC_SPEC2_ADD_BOT);
	addr.two.top = readl((int)base + EMAC_SPEC2_ADD_TOP);
	addr.three.bottom = readl((int)base + EMAC_SPEC3_ADD_BOT);
	addr.three.top = readl((int)base + EMAC_SPEC3_ADD_TOP);
	addr.four.bottom = readl((int)base + EMAC_SPEC4_ADD_BOT);
	addr.four.top = readl((int)base + EMAC_SPEC4_ADD_TOP);
	
	return addr;
}

/** GEMAC set specific local addresses of the MAC.
* Rather than setting up all four specific addresses, this function sets them up individually.
*
* @param[in] base	GEMAC base address
* @param[in] addr	MAC address to be configured
*/
void gemac_set_laddr1(void *base, MAC_ADDR *address)
{
	writel(address->bottom,	(int)base + EMAC_SPEC1_ADD_BOT);
	writel(address->top, (int)base + EMAC_SPEC1_ADD_TOP); 
}


void gemac_set_laddr2(void *base, MAC_ADDR *address)
{
	writel(address->bottom, (int)base + EMAC_SPEC2_ADD_BOT);
	writel(address->top, (int)base + EMAC_SPEC2_ADD_TOP); 
}


void gemac_set_laddr3(void *base, MAC_ADDR *address)
{
	writel(address->bottom, (int)base + EMAC_SPEC3_ADD_BOT);
	writel(address->top, (int)base + EMAC_SPEC3_ADD_TOP); 
}


void gemac_set_laddr4(void *base, MAC_ADDR *address)
{
	writel(address->bottom, (int)base + EMAC_SPEC4_ADD_BOT);
	writel(address->top, (int)base + EMAC_SPEC4_ADD_TOP); 
}

void gemac_set_laddrN(void *base, MAC_ADDR *address, unsigned int entry_index)
{
	if (entry_index < 5)
	{	
		writel(address->bottom,	(int)base + (entry_index * 8) +
		    EMAC_SPEC1_ADD_BOT);
		writel(address->top, (int)base + (entry_index * 8) +
		    EMAC_SPEC1_ADD_TOP);
	} 
	else 
	{
		writel(address->bottom, (int)base + ((entry_index - 5) * 8) +
		    EMAC_SPEC5_ADD_BOT);
		writel(address->top, (int)base + ((entry_index - 5) * 8) +
		    EMAC_SPEC5_ADD_TOP);
	}
}

/** Get specific local addresses of the MAC.
* This allows returning of a single specific address stored in the MAC.
* @param[in] base	GEMAC base address
*
* @return		Specific MAC address 1
* 
*/
MAC_ADDR gem_get_laddr1(void *base)
{
	MAC_ADDR addr;
	addr.bottom = readl((int)base + EMAC_SPEC1_ADD_BOT);
	addr.top = readl((int)base + EMAC_SPEC1_ADD_TOP);
	return addr;
}


MAC_ADDR gem_get_laddr2(void *base)
{
	MAC_ADDR addr;
	addr.bottom = readl((int)base + EMAC_SPEC2_ADD_BOT);
	addr.top = readl((int)base + EMAC_SPEC2_ADD_TOP);
	return addr;
}


MAC_ADDR gem_get_laddr3(void *base)
{
	MAC_ADDR addr;
	addr.bottom = readl((int)base + EMAC_SPEC3_ADD_BOT);
	addr.top = readl((int)base + EMAC_SPEC3_ADD_TOP);
	return addr;
}


MAC_ADDR gem_get_laddr4(void *base)
{
	MAC_ADDR addr;
	addr.bottom = readl((int)base + EMAC_SPEC4_ADD_BOT);
	addr.top = readl((int)base + EMAC_SPEC4_ADD_TOP);
	return addr;
}


MAC_ADDR gem_get_laddrN(void *base, unsigned int entry_index)
{
	MAC_ADDR addr;

	if (entry_index < 5)
	{
		addr.bottom = readl((int)base + (entry_index * 8) +
		    EMAC_SPEC1_ADD_BOT);
		addr.top = readl((int)base + (entry_index * 8) +
		    EMAC_SPEC1_ADD_TOP);
	}
	else
	{
		addr.bottom = readl((int)base + ((entry_index - 5) * 8) +
		    EMAC_SPEC5_ADD_BOT);
		addr.top = readl((int)base + ((entry_index - 5) * 8) +
		    EMAC_SPEC5_ADD_TOP);
	}

	return addr;
}

/** GEMAC allow frames
* @param[in] base	GEMAC base address
*/
void gemac_enable_copy_all(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) & EMAC_ENABLE_COPY_ALL,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC do not allow frames
* @param[in] base	GEMAC base address
*/
void gemac_disable_copy_all(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) & ~EMAC_ENABLE_COPY_ALL,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC allow broadcast function.
* @param[in] base	GEMAC base address
*/
void gemac_allow_broadcast(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) & ~EMAC_NO_BROADCAST,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC no broadcast function.
* @param[in] base	GEMAC base address
*/
void gemac_no_broadcast(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) | EMAC_NO_BROADCAST,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC enable unicast function.
* @param[in] base	GEMAC base address
*/
void gemac_enable_unicast(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) | EMAC_ENABLE_UNICAST,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC disable unicast function.
* @param[in] base	GEMAC base address
*/
void gemac_disable_unicast(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) & ~EMAC_ENABLE_UNICAST,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC enable multicast function.
* @param[in] base	GEMAC base address
*/
void gemac_enable_multicast(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) | EMAC_ENABLE_MULTICAST,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC disable multicast function.
* @param[in]	base	GEMAC base address
*/
void gemac_disable_multicast(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) & ~EMAC_ENABLE_MULTICAST,
	   (int) base + EMAC_NETWORK_CONFIG);
}

/** GEMAC enable fcs rx function.
* @param[in]	base	GEMAC base address
*/
void gemac_enable_fcs_rx(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) | EMAC_ENABLE_FCS_RX,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC disable fcs rx function.
* @param[in]	base	GEMAC base address
*/
void gemac_disable_fcs_rx(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) & ~EMAC_ENABLE_FCS_RX,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC enable 1536 rx function.
* @param[in]	base	GEMAC base address
*/
void gemac_enable_1536_rx(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) | EMAC_ENABLE_1536_RX,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC disable 1536 rx function.
* @param[in]	base	GEMAC base address
*/
void gemac_disable_1536_rx(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) & ~EMAC_ENABLE_1536_RX,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC enable pause rx function.
* @param[in] base	GEMAC base address
*/
void gemac_enable_pause_rx(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) | EMAC_ENABLE_PAUSE_RX,
	   (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC disable pause rx function.
* @param[in] base	GEMAC base address
*/
void gemac_disable_pause_rx(void *base)
{
	writel (readl((int)base + EMAC_NETWORK_CONFIG) & ~EMAC_ENABLE_PAUSE_RX,
	    (int)base + EMAC_NETWORK_CONFIG);
}

/** GEMAC enable rx checksum offload function.
* @param[in] base	GEMAC base address
*/
void gemac_enable_rx_checksum_offload(void *base)
{
	writel(readl((int)base + EMAC_NETWORK_CONFIG) | EMAC_ENABLE_CHKSUM_RX,
	    (int)base + EMAC_NETWORK_CONFIG);
	writel(readl((int)base + CLASS_L4_CHKSUM_ADDR) | IPV4_CHKSUM_DROP,
	    (int)base + CLASS_L4_CHKSUM_ADDR);
}

/** GEMAC disable rx checksum offload function.
* @param[in] base	GEMAC base address
*/
void gemac_disable_rx_checksum_offload(void *base)
{
	writel(readl((int)base + EMAC_NETWORK_CONFIG) & ~EMAC_ENABLE_CHKSUM_RX,
	    (int)base + EMAC_NETWORK_CONFIG);
	writel(readl((int)base + CLASS_L4_CHKSUM_ADDR) & ~IPV4_CHKSUM_DROP,
	    (int)base + CLASS_L4_CHKSUM_ADDR);
}

/** Sets Gemac bus width to 64bit
 * @param[in] base       GEMAC base address
 * @param[in] width	gemac bus width to be set possible values are 32/64/128
 * */
void gemac_set_bus_width(void *base, int width)
{
	u32 val = readl((int)base + EMAC_NETWORK_CONFIG);
	switch(width) 
	{
	case 32:
		val = (val & ~EMAC_DATA_BUS_WIDTH_MASK) | EMAC_DATA_BUS_WIDTH_32;
	case 128:
		val = (val & ~EMAC_DATA_BUS_WIDTH_MASK) | EMAC_DATA_BUS_WIDTH_128;
	case 64:
	default:
		val = (val & ~EMAC_DATA_BUS_WIDTH_MASK) | EMAC_DATA_BUS_WIDTH_64;

	}
	writel (val, (int)base + EMAC_NETWORK_CONFIG);
}

/** Sets Gemac configuration.
* @param[in] base	GEMAC base address
* @param[in] cfg	GEMAC configuration
*/
void gemac_set_config(void *base, GEMAC_CFG *cfg)
{
	gemac_set_mode(base, cfg->mode);

	gemac_set_speed(base, cfg->speed);

	gemac_set_duplex(base,cfg->duplex);
}

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

/**************************** HIF ***************************/

#ifdef NOTUSE
/** Initializes HIF no copy block.
*
*/
void hif_nocpy_init(void)
{
	writel(4, HIF_NOCPY_TX_PORT_NO);
	writel(CBUS_VIRT_TO_PFE(BMU1_BASE_ADDR + BMU_ALLOC_CTRL),
	    HIF_NOCPY_LMEM_ALLOC_ADDR);
	writel(CBUS_VIRT_TO_PFE(CLASS_INQ_PKTPTR), HIF_NOCPY_CLASS_ADDR);
	writel(CBUS_VIRT_TO_PFE(TMU_PHY_INQ_PKTPTR), HIF_NOCPY_TMU_PORT0_ADDR);
}
#endif   /* NOTUSE */

/** Initializes HIF copy block.
*
*/
void hif_init(void)
{
	/*Initialize HIF registers*/
	writel(HIF_RX_POLL_CTRL_CYCLE<<16|HIF_TX_POLL_CTRL_CYCLE,
	    HIF_POLL_CTRL); 
}

/** Enable hif tx DMA and interrupt
*
*/
void hif_tx_enable(void)
{
	writel(HIF_CTRL_DMA_EN, HIF_TX_CTRL);
	writel((readl(HIF_INT_ENABLE) | HIF_INT_EN | HIF_TXPKT_INT_EN),
	    HIF_INT_ENABLE);
}

/** Disable hif tx DMA and interrupt
*
*/
void hif_tx_disable(void)
{
	u32	hif_int;

	writel(0, HIF_TX_CTRL);

	hif_int = readl(HIF_INT_ENABLE);
	hif_int &= HIF_TXPKT_INT_EN;
	writel(hif_int, HIF_INT_ENABLE);
}

/** Enable hif rx DMA and interrupt
*
*/
void hif_rx_enable(void)
{
	writel((HIF_CTRL_DMA_EN | HIF_CTRL_BDP_CH_START_WSTB), HIF_RX_CTRL);
	writel((readl(HIF_INT_ENABLE) | HIF_INT_EN | HIF_TXPKT_INT_EN),
	    HIF_INT_ENABLE);
}

/** Disable hif rx DMA and interrupt
*
*/
void hif_rx_disable(void)
{
	u32	hif_int;

	writel(0, HIF_RX_CTRL);

	hif_int = readl(HIF_INT_ENABLE);
	hif_int &= HIF_RXPKT_INT_EN;
	writel(hif_int, HIF_INT_ENABLE);
}
