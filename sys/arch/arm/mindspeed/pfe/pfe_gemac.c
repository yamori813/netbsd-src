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

/** GEMAC Get MDC clock division
 * @param[in] base       GEMAC base address
 */
int gemac_get_mdc_div(void *base)
{
        u32 data;
	int div[] = {8, 16, 32, 48, 64, 96, 128, 224};

        data = readl((int)base + EMAC_NETWORK_CONFIG);
	data = (data >> MDC_DIV_SHIFT) & MDC_DIV_MASK;

	return div[data];
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
