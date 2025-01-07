/*	$NetBSD$	*/

/*
 * Copyright (c) 2025 Hiroki Mori
 * All rights reserved.
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
 * THIS SOFTWARE IS PROVIDED BY WASABI SYSTEMS, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL WASABI SYSTEMS, INC
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * PCI configuration support for MX83XXX
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_pci.h"
#include "pci.h"

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/device.h>
#include <sys/extent.h>
#include <sys/intr.h>
#include <sys/kernel.h>		/* for 'hz' */
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/extent.h>
#include <sys/queue.h>
#include <sys/mutex.h>
#include <sys/kmem.h>

#include <machine/frame.h>
#include <arm/cpufunc.h>

#include <arm/mindspeed/m83xxx_reg.h>
#include <arm/mindspeed/m83xxx_var.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcidevs.h>
#include <dev/pci/pciconf.h>

#include <arm/mindspeed/m83xxx_pcie.h>



#define PCIE_CONF_LOCK(s)	(s) = disable_interrupts(I32_bit)
#define PCIE_CONF_UNLOCK(s)	restore_interrupts((s))

#define PCIE_READ(sc, reg)					\
	bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh, reg)
#define PCIE_WRITE(sc, reg, val)				\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_ioh, reg, val)
#define PHY_READ(sc, reg)					\
	bus_space_read_4((sc)->sc_iot, (sc)->sc_phy_ioh, reg)
#define PHY_WRITE(sc, reg, val)				\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_phy_ioh, reg, val)
#define INDIRECT_READ(sc, reg)					\
	bus_space_read_4((sc)->sc_iot, (sc)->sc_indirect_ioh, reg)
#define INDIRECT_WRITE(sc, reg, val)				\
	bus_space_write_4((sc)->sc_iot, (sc)->sc_indirect_ioh, reg, val)

static void m83pcie_init(pci_chipset_tag_t, void *);
static void m83pcie_setup(struct m83pcie_softc * const);

static void m83pcie_attach_hook(device_t, device_t, struct pcibus_attach_args *);
static int m83pcie_bus_maxdevs(void *, int);
static pcitag_t m83pcie_make_tag(void *, int, int, int);
static void m83pcie_decompose_tag(void *, pcitag_t, int *, int *, int *);
static pcireg_t m83pcie_conf_read(void *, pcitag_t, int);
static void m83pcie_conf_write(void *, pcitag_t, int, pcireg_t);
#ifdef __HAVE_PCI_CONF_HOOK
static int m83pcie_conf_hook(void *, int, int, int, pcireg_t);
#endif
static void m83pcie_conf_interrupt(void *, int, int, int, int, int *);

static int m83pcie_intr_map(const struct pci_attach_args *, pci_intr_handle_t *);
static const char *m83pcie_intr_string(void *, pci_intr_handle_t, char *, size_t);
const struct evcnt *m83pcie_intr_evcnt(void *, pci_intr_handle_t);
static void * m83pcie_intr_establish(void *, pci_intr_handle_t, int,
    int (*)(void *), void *, const char *);
static void m83pcie_intr_disestablish(void *, void *);

#if 0
static int
m83pcie_linkup_status(struct m83pcie_softc *sc)
{
//	return PCIE_READ(sc, PCIE_PL_DEBUG1) & PCIE_PL_DEBUG1_XMLH_LINK_UP;
	return 0;
}

static int
m83pcie_valid_device(struct m83pcie_softc *sc, int bus, int dev)
{
#if 0
	if (bus != 0 && !m83pcie_linkup_status(sc))
		return 0;
	if (bus <= 1 && dev > 0)
		return 0;
#endif

	return 1;
}
#endif

static int
m83pcie_init_phy(struct m83pcie_softc *sc, int ext_clk)
{
	uint32_t ncy;
	uint32_t ncy5;
	uint32_t prescale;

	/* Default values */
	uint32_t tx_level = 0xa & 0x1f;
	uint32_t tx_boost = 0xb & 0xf;
	uint32_t tx_atten = 0x0 & 0x7;
	uint32_t tx_edge_rate = 0x0 & 0x3;
	uint32_t tx_clk_align = 0x0 & 0x1;

/*
	uint32_t los_lvl = 0x11 & 0x1f;
	uint32_t rx_equal_val = 0x2 & 0x7;
*/

	if (ext_clk) {
		/* Baud rate = 100MHz / Prescale * MPLL_divisor / 0.5 = 100MHz / 2 * 25 / 0.5 = 2.5GHz */
		ncy = 0x5 & 0x1f;
		ncy5 = 0x1 & 0x3;
		prescale = 0x2 & 0x3;
        } else {
		/* Baud rate = 62.5MHz * MPLL_divisor / 0.5 = 62.5MHz * 20 / 0.5 = 2.5GHz */
		ncy = 0x4 & 0x1f;
		ncy5 = 0x0 & 0x3;
		prescale = 0x0 & 0x3;
	}

        PHY_WRITE(sc, PCIE_PHY_MPLL_CTRL, (prescale << 1) | (ncy5 << 3) | (ncy << 5));

        /* fast tech, low = 1.0, high = 2.5 */
        PHY_WRITE(sc, PCIE_PHY_TECHNOLOGY_CTRL, PCIE_TECH_CTL_FAST_TECH);

        PHY_WRITE(sc, PCIE_PHY_TRANSMIT_LEVEL_CTRL, tx_level);

        PHY_WRITE(sc, PCIE_PHY_LANE0_TX_CTRL, tx_edge_rate | (tx_boost << 2) | (tx_atten << 6) | (tx_clk_align << 9));

        PHY_WRITE(sc, PCIE_PHY_LANE1_TX_CTRL, tx_edge_rate | (tx_boost << 2) | (tx_atten << 6) | (tx_clk_align << 9));

	PHY_WRITE(sc, PCIE_PHY_PCS_CTRL, 0x00000000);

	return 0;
}

#if 0
static int
m83pcie_phy_wait_ack(struct m83pcie_softc *sc, int ack)
{
#if 0
	uint32_t v;
	int timeout;

	for (timeout = 10; timeout > 0; --timeout) {
		v = PCIE_READ(sc, PCIE_PL_PHY_STATUS);
		if (!!(v & PCIE_PL_PHY_STATUS_ACK) == !!ack)
			return 0;
		delay(1);
	}
#endif

	return -1;
}

static int
m83pcie_phy_addr(struct m83pcie_softc *sc, uint32_t addr)
{
#if 0
	uint32_t v;

	v = __SHIFTIN(addr, PCIE_PL_PHY_CTRL_DATA);
	PCIE_WRITE(sc, PCIE_PL_PHY_CTRL, v);

	v |= PCIE_PL_PHY_CTRL_CAP_ADR;
	PCIE_WRITE(sc, PCIE_PL_PHY_CTRL, v);

	if (m83pcie_phy_wait_ack(sc, 1))
		return -1;

	v = __SHIFTIN(addr, PCIE_PL_PHY_CTRL_DATA);
	PCIE_WRITE(sc, PCIE_PL_PHY_CTRL, v);

	if (m83pcie_phy_wait_ack(sc, 0))
		return -1;

#endif
	return 0;
}

static int
m83pcie_phy_write(struct m83pcie_softc *sc, uint32_t addr, uint16_t data)
{
	PHY_WRITE(sc, PCIE_PHY_PARALLEL_CR_CTRL_PORT_ADDR, addr);
	PHY_WRITE(sc, PCIE_PHY_PARALLEL_CR_CTRL_PORT_DATA, val);
	return 0;
}

static int
m83pcie_phy_read(struct m83pcie_softc *sc, uint32_t addr)
{
	PHY_WRITE(sc, PCIE_PHY_PARALLEL_CR_CTRL_PORT_ADDR, addr);
        return (PHY_READ(sc, PCIE_PHY_PARALLEL_CR_CTRL_PORT_DATA) & 0xffff);
}
#endif

static int
m83pcie_wait_for_link(struct m83pcie_softc *sc)
{
#if 0
#define LINKUP_RETRY	20000
	for (int retry = LINKUP_RETRY; retry > 0; --retry) {
		if (!m83pcie_linkup_status(sc)) {
			delay(10);
			continue;
		}

		uint32_t valid = m83pcie_phy_read(sc, PCIE_PHY_RX_ASIC_OUT) &
		    PCIE_PHY_RX_ASIC_OUT_VALID;
		uint32_t ltssm = __SHIFTOUT(PCIE_READ(sc, PCIE_PL_DEBUG0),
		    PCIE_PL_DEBUG0_XMLH_LTSSM_STATE);

		if ((ltssm == 0x0d) && !valid) {
			aprint_normal_dev(sc->sc_dev, "resetting PCIe phy\n");

			uint32_t v = m83pcie_phy_read(sc, PCIE_PHY_RX_OVRD_IN_LO);
			v |= PCIE_PHY_RX_OVRD_IN_LO_RX_PLL_EN_OVRD;
			v |= PCIE_PHY_RX_OVRD_IN_LO_RX_DATA_EN_OVRD;
			m83pcie_phy_write(sc, PCIE_PHY_RX_OVRD_IN_LO, v);

			delay(3000);

			v = m83pcie_phy_read(sc, PCIE_PHY_RX_OVRD_IN_LO);
			v &= ~PCIE_PHY_RX_OVRD_IN_LO_RX_PLL_EN_OVRD;
			v &= ~PCIE_PHY_RX_OVRD_IN_LO_RX_DATA_EN_OVRD;
			m83pcie_phy_write(sc, PCIE_PHY_RX_OVRD_IN_LO, v);
		}

		return 0;
	}

	aprint_error_dev(sc->sc_dev, "Link Up failed.\n");
#endif

	return -1;
}

static void
m83pcie_linkup(struct m83pcie_softc *sc)
{
//	uint32_t v;
	int ret;

	m83pcie_setup(sc);

#if 0
	/* GEN1 Operation */
	v = PCIE_READ(sc, PCIE_RC_LCR);
	v &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS;
	v |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN1;
	PCIE_WRITE(sc, PCIE_RC_LCR, v);

	/* Link Up */
	v = sc->sc_gpr_read(sc, IOMUX_GPR12);
	v |= IOMUX_GPR12_APP_LTSSM_ENABLE;
	sc->sc_gpr_write(sc, IOMUX_GPR12, v);

	ret = m83pcie_wait_for_link(sc);
	if (ret)
		goto error;

	/* Allow Gen2 mode */
	v = PCIE_READ(sc, PCIE_RC_LCR);
	v &= ~PCIE_RC_LCR_MAX_LINK_SPEEDS;
	v |= PCIE_RC_LCR_MAX_LINK_SPEEDS_GEN2;
	PCIE_WRITE(sc, PCIE_RC_LCR, v);
#endif
	ret = m83pcie_wait_for_link(sc);
	if (ret)
		goto error;

//	v = PCIE_READ(sc, PCIE_RC_LCSR);
//	aprint_normal_dev(sc->sc_dev, "LinkUp, Gen %d\n",
//	    (int)__SHIFTOUT(v, PCIE_RC_LCSR_LINK_SPEED));

	return;

error:
//	aprint_error_dev(sc->sc_dev, "PCIE_PL_DEBUG0,1 = %08x, %08x\n",
//	    PCIE_READ(sc, PCIE_PL_DEBUG0), PCIE_READ(sc, PCIE_PL_DEBUG1));

	return;
}

static inline void
m83pcie_direct_memory_mapped_cfg(struct m83pcie_softc * const sc)
{
	/* PCIe mem offset */
	PCIE_WRITE(sc, PCIE_CH1_DMA_REMOTE_ADDR_LOW, AHB_PCIe0_BASE & 0xf0000000);
	PCIE_WRITE(sc, PCIE_CH1_DMA_REMOTE_ADDR_HIGH, 0);

	/* FIXME - When should we use 32bit/64bit? */
	/* FIXME - What traffic class, attributes should be used? */
	PCIE_WRITE(sc, PCIE_CH1_DMA_START, PCIE_CH_STR_START | PCIE_CH_STR_ATTR(0) | PCIE_CH_STR_TYPE(TYPE_MEM32) | PCIE_CH_STR_CLASS(0));
}

static int
m83pcie_direct_memory_mapped_init(struct m83pcie_softc * const sc)
{
	PCIE_WRITE(sc, PCIE_CH1_DMA_INT_DISABLE, 0x7ff);
	PCIE_WRITE(sc, PCIE_CH1_DMA_CTRL_CFG, PCIE_CH_CTL_WEIGHT(4) | PCIE_CH_CTL_CMD_QUEUE_FLUSH);

	m83pcie_direct_memory_mapped_cfg(sc);

	return 0;
}

static int
m83pcie_requester_dma_init(struct m83pcie_softc * const sc)
{
	m83pcie_direct_memory_mapped_init(sc);

//	pcie_bulk_mode_init(ctrl, PCIE_BULK_MODE_QUEUE_SIZE);

	PCIE_WRITE(sc, PCIE_DMA_CTRL_CFG, PCIE_READ(sc, PCIE_DMA_CTRL_CFG) | PCIE_DMA_CTL_REQUESTER_ENABLE);

	return 0;
}

static int
m83pcie_completer_dma_init(struct m83pcie_softc * const sc)
{
	uint32_t tmp;

//	if (!ctrl->is_endpoint) {
	if (1) {
		/* FIXME - only one BAR is setup in configuration space */
		/* Use same addresses in AHB and PCIe domains */
		PCIE_WRITE(sc, PCIE_BAR0_ADDR_OFFSET, AHB_DDR_BASE);
		PCIE_WRITE(sc, PCIE_BAR1_ADDR_OFFSET, AHB_ARAM_BASE);
		PCIE_WRITE(sc, PCIE_BAR2_ADDR_OFFSET, 0);

		PCIE_WRITE(sc, PCIE_BAR0_SIZE, BAR_MSK_PREFETCHABLE | 0xa);	/* DDR, 1GiB */
		PCIE_WRITE(sc, PCIE_BAR1_SIZE, BAR_MSK_PREFETCHABLE | 0x0);	/* ARAM, 1MiB */
		PCIE_WRITE(sc, PCIE_BAR2_SIZE, 0);	/* Not used */

		tmp = PCIE_READ(sc, PCIE_PEX_CFG1);
		PCIE_WRITE(sc, PCIE_PEX_CFG1, tmp | PCIE_PEX_CFG1_BAR_MATCH_ENABLE);

	} else {
		/* FIXME - should match configuration space BAR's */
		/* In simulation external host is expecting one 4GiB BAR to access DDR */
		PCIE_WRITE(sc, PCIE_BAR0_ADDR_OFFSET, 0x00000000);
		PCIE_WRITE(sc, PCIE_BAR0_SIZE, BAR_MSK_PREFETCHABLE | 0xc);   /* DDR, 4GiB */
	}

	tmp = PCIE_READ(sc, PCIE_DMA_CTRL_CFG);
	tmp &= ~PCIE_DMA_CTL_AHB64BIT_OVERWRITE;
	tmp |= PCIE_DMA_CTL_COMPLETER_ENABLE | PCIE_DMA_CTL_AHB_EARLY_BURST_TERMINATION;
	PCIE_WRITE(sc, PCIE_DMA_CTRL_CFG, tmp);

	return 0;
}
static void
m83pcie_configuration_space_init(struct m83pcie_softc * const sc)
{
	uint32_t tmp;

//	if (!ctrl->is_endpoint) {
	if (1) {
		/* Type 1 configuration header */

		/* Enable memory IO and bus master */
		tmp = PCIE_READ(sc, PCIE_PEX_IP_COMMAND_STATUS);
		tmp |= PCI_COMMAND_MASTER_ENABLE | PCI_COMMAND_MEM_ENABLE;
		PCIE_WRITE(sc, PCIE_PEX_IP_COMMAND_STATUS, tmp);

		PCIE_WRITE(sc, PCIE_PEX_IP_BAR0, AHB_DDR_BASE);
                PCIE_WRITE(sc, PCIE_PEX_IP_BAR1, 0x0);
	} else {
		/* Type 0 configuration header */

		/* BARS are 64bit */
		PCIE_WRITE(sc, PCIE_PEX_IP_BAR0, AHB_DDR_BASE);
                PCIE_WRITE(sc, PCIE_PEX_IP_BAR1, 0x0);

		PCIE_WRITE(sc, PCIE_PEX_IP_BAR2, AHB_DDR_BASE);
                PCIE_WRITE(sc, PCIE_PEX_IP_BAR3, 0x0);

                PCIE_WRITE(sc, PCIE_PEX_IP_BAR4, 0x00000000);
                PCIE_WRITE(sc, PCIE_PEX_IP_BAR5, 0x00000000);
	}
}

static void
m83pcie_configure(void *cookie)
{
	struct m83pcie_softc * const sc = cookie;

	struct pciconf_resources *pcires = pciconf_resource_init();
/*
	pciconf_resource_add(pcires, PCICONF_RESOURCE_IO,
	    IMX6_PCIE_IO_BASE, IMX6_PCIE_IO_SIZE);
*/
	pciconf_resource_add(pcires, PCICONF_RESOURCE_MEM,
	    AHB_PCIe0_BASE, PCIe_MEM_SIZE);

	int error = pci_configure_bus(&sc->sc_pc, pcires, 0, arm_dcache_align);

	pciconf_resource_fini(pcires);

	if (error) {
		aprint_error_dev(sc->sc_dev, "configuration failed (%d)\n",
		    error);
	}
}

void
m83pcie_attach_common(struct m83pcie_softc * const sc)
{
	struct pcibus_attach_args pba;

	if (bus_space_map(sc->sc_iot, AHB_PCIe0CMD_BASE, 0x10000, 0,
		&sc->sc_indirect_ioh)) {
		aprint_error_dev(sc->sc_dev, "Cannot map indirect address\n");
		return;
	}

	if (bus_space_map(sc->sc_iot, APB_PCIePHY_BASE, 0x10000, 0,
		&sc->sc_phy_ioh)) {
		aprint_error_dev(sc->sc_dev, "Cannot map phy address\n");
		return;
	}

	m83pcie_linkup(sc);

	TAILQ_INIT(&sc->sc_intrs);
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_VM);

	m83pcie_init(&sc->sc_pc, sc);

	m83pcie_configuration_space_init(sc);
	m83pcie_completer_dma_init(sc);
	m83pcie_requester_dma_init(sc);

	if (sc->sc_pci_netbsd_configure != NULL)
		sc->sc_pci_netbsd_configure(sc);

	memset(&pba, 0, sizeof(pba));
	pba.pba_flags = PCI_FLAGS_MEM_OKAY;
//	pba.pba_flags |= PCI_FLAGS_IO_OKAY;
	pba.pba_iot = sc->sc_iot;
	pba.pba_memt = sc->sc_iot;
	pba.pba_dmat = sc->sc_dmat;
	pba.pba_pc = &sc->sc_pc;
	pba.pba_bus = 0;

	config_found(sc->sc_dev, &pba, pcibusprint,
	    CFARGS(.devhandle = device_handle(sc->sc_dev)));
}

int
m83pcie_intr(void *priv)
{
	struct m83pcie_softc *sc = priv;
	struct m83pcie_ih *pcie_ih;

#if 0
	for (int i = 0; i < 8; i++) {
		uint32_t v = PCIE_READ(sc, PCIE_PL_MSICIN_STATUS + i * 0xC);
		int bit;
		while ((bit = ffs(v) - 1) >= 0) {
			PCIE_WRITE(sc, PCIE_PL_MSICIN_STATUS + i * 0xC,
			    __BIT(bit));
			v &= ~__BIT(bit);
		}
	}
#endif

	mutex_enter(&sc->sc_lock);
	int rv = 0;
	const u_int lastgen = sc->sc_intrgen;
	TAILQ_FOREACH(pcie_ih, &sc->sc_intrs, ih_entry) {
		int (*callback)(void *) = pcie_ih->ih_handler;
		void *arg = pcie_ih->ih_arg;
		mutex_exit(&sc->sc_lock);
		rv += callback(arg);
		mutex_enter(&sc->sc_lock);
		if (lastgen != sc->sc_intrgen)
			break;
	}
	mutex_exit(&sc->sc_lock);

	return rv;
}

static void
m83pcie_setup(struct m83pcie_softc * const sc)
{
	bus_space_handle_t bsh;
	uint32_t reg;
	int ext_clk;

	/* IO Control Register */

	bus_space_map(sc->sc_iot, APB_GPIO_BASE, 0x20000, 0, &bsh);

#define PCIE_REFCLK_SRC		(1 << 12)
	reg = bus_space_read_4(sc->sc_iot, bsh, GPIO_SYSTEM_CONFIG);
	if (reg & PCIE_REFCLK_SRC)
		ext_clk = 1;
	else
		ext_clk = 0;
	bus_space_unmap(sc->sc_iot, bsh, 0x20000);

	bus_space_map(sc->sc_iot, APB_CLK_BASE, 0x20000, 0, &bsh);

	reg = bus_space_read_4(sc->sc_iot, bsh, BLOCK_RESET_REG);
	reg &= ~(PCIE0_REF_RST | PCIE0_AHB_RST);
	bus_space_write_4(sc->sc_iot, bsh, BLOCK_RESET_REG, reg);

	if(ext_clk) {
		reg = bus_space_read_4(sc->sc_iot, bsh, CLK_CLK_PWR_DWN);
		reg |= PCIE_REFCLK_NP_PD;
		bus_space_write_4(sc->sc_iot, bsh, CLK_CLK_PWR_DWN, reg);
		reg = bus_space_read_4(sc->sc_iot, bsh, CLK_CLK_PWR_DWN);
		reg &= ~PCIE0_AHBCLK_PD;
		bus_space_write_4(sc->sc_iot, bsh, CLK_CLK_PWR_DWN, reg);
	} else {
		reg = bus_space_read_4(sc->sc_iot, bsh, CLK_CLK_PWR_DWN);
		reg &= ~PCIE0_AHBCLK_PD;
		bus_space_write_4(sc->sc_iot, bsh, CLK_CLK_PWR_DWN, reg);
		reg = bus_space_read_4(sc->sc_iot, bsh, CLK_CLK_PWR_DWN);
		reg &= ~PCIE_REFCLK_NP_PD;
		bus_space_write_4(sc->sc_iot, bsh, CLK_CLK_PWR_DWN, reg);

		/* Set reference clock to 250/4 = 62.5 MHz */
		reg = bus_space_read_4(sc->sc_iot, bsh, CLK_DDR_PCIE_CLK_CNTRL);

		reg &= ~(PCIE_DIV_VAL_MASK | PCIE_DIV_BYPASS);
		reg |= 4 << PCIE_DIV_VAL_OFFSET;

		bus_space_write_4(sc->sc_iot, bsh, CLK_DDR_PCIE_CLK_CNTRL, reg);
		/* Switch to clock output */
		reg &= ~PCIE_MUX_SEL;
		bus_space_write_4(sc->sc_iot, bsh, CLK_DDR_PCIE_CLK_CNTRL, reg);
	}
	/* Take block out of reset */
	reg = bus_space_read_4(sc->sc_iot, bsh, BLOCK_RESET_REG);
	reg |= (PCIE0_REF_RST | PCIE0_AHB_RST);
	bus_space_write_4(sc->sc_iot, bsh, BLOCK_RESET_REG, reg);

	bus_space_unmap(sc->sc_iot, bsh, 0x20000);

	m83pcie_init_phy(sc, ext_clk);
}

void
m83pcie_init(pci_chipset_tag_t pc, void *priv)
{
	pc->pc_conf_v = priv;
	pc->pc_attach_hook = m83pcie_attach_hook;
	pc->pc_bus_maxdevs = m83pcie_bus_maxdevs;
	pc->pc_make_tag = m83pcie_make_tag;
	pc->pc_decompose_tag = m83pcie_decompose_tag;
	pc->pc_conf_read = m83pcie_conf_read;
	pc->pc_conf_write = m83pcie_conf_write;
#ifdef __HAVE_PCI_CONF_HOOK
	pc->pc_conf_hook = m83pcie_conf_hook;
#endif
	pc->pc_conf_interrupt = m83pcie_conf_interrupt;

	pc->pc_intr_v = priv;
	pc->pc_intr_map = m83pcie_intr_map;
	pc->pc_intr_string = m83pcie_intr_string;
	pc->pc_intr_evcnt = m83pcie_intr_evcnt;
	pc->pc_intr_establish = m83pcie_intr_establish;
	pc->pc_intr_disestablish = m83pcie_intr_disestablish;
}

static void
m83pcie_attach_hook(device_t parent, device_t self,
    struct pcibus_attach_args *pba)
{
	/* nothing to do */
}

static int
m83pcie_bus_maxdevs(void *v, int busno)
{
	return 1;
}

static pcitag_t
m83pcie_make_tag(void *v, int b, int d, int f)
{
	return (b << 16) | (d << 11) | (f << 8);
}

static void
m83pcie_decompose_tag(void *v, pcitag_t tag, int *bp, int *dp, int *fp)
{
	if (bp)
		*bp = (tag >> 16) & 0xff;
	if (dp)
		*dp = (tag >> 11) & 0x1f;
	if (fp)
		*fp = (tag >> 8) & 0x7;
}

/*
 * work around.
 * If there is no PCIe devices, DABT will be generated by read/write access to
 * config area, so replace original DABT handler with simple jump-back one.
 */
#if 0
extern u_int data_abort_handler_address;
static bool data_abort_flag;
static void
m83pcie_data_abort_handler(trapframe_t *tf)
{
	data_abort_flag = true;
	tf->tf_pc += 0x4;
	return;
}
#endif

static pcireg_t
m83pcie_conf_read(void *v, pcitag_t tag, int where)
{
	struct m83pcie_softc *sc = v;
	int bus, devfn;
	int offset = where & 0x03;
	uint32_t byte_enables;
	int size = 4;
	int status;
	int s;
	pcireg_t val;

	bus = (tag >> 16) & 0xff;
	devfn = (tag >> 8) & 0xff;

	PCIE_CONF_LOCK(s);

	INDIRECT_WRITE(sc, PCIE_CH0_DMA_REMOTE_ADDR_LOW, ((bus & 0xff) << 24) | ((devfn & 0xff) << 16) | (where & 0xffc));
	INDIRECT_WRITE(sc, PCIE_CH0_DMA_REMOTE_ADDR_HIGH, 0);

	byte_enables = ((1 << size) - 1) << offset;

	INDIRECT_WRITE(sc, PCIE_CH0_DMA_START,
	    PCIE_CH_STR_START | PCIE_CH_STR_LAST_ADDR_BE(0x0) |
	    PCIE_CH_STR_FIRST_ADDR_BE(byte_enables) |
	    PCIE_CH_STR_ATTR(0) | PCIE_CH_STR_TYPE(TYPE_CFG0) |
	    PCIE_CH_STR_CLASS(0) | PCIE_CH_STR_SIMPLE_MODE);
 
	while ((status = INDIRECT_READ(sc, PCIE_CH0_DMA_STATUS)) & PCIE_CH_STS_SIMPLE_REQ_BUSY) ;

	if (PCIE_CH_STS_SIMPLE_REQ_STATUS_GET(status)) {
		aprint_error("config read error\n");
		val = -1;
		goto out;
	}

	val = (INDIRECT_READ(sc, PCIE_CH0_DMA_SIMPLE_READ) >> (8 * offset)) & ((1 << (8 * size)) - 1);

out:
	PCIE_CONF_UNLOCK(s);

	return val;
}

static void
m83pcie_conf_write(void *v, pcitag_t tag, int where, pcireg_t val)
{
	struct m83pcie_softc *sc = v;
	int bus, devfn;
	int offset = where & 0x03;
	uint32_t byte_enables;
	int size = 4;
	int status;
	int s;

	bus = (tag >> 16) & 0xff;
	devfn = (tag >> 8) & 0xff;

	PCIE_CONF_LOCK(s);

	INDIRECT_WRITE(sc, PCIE_CH0_DMA_REMOTE_ADDR_LOW, ((bus & 0xff) << 24) | ((devfn & 0xff) << 16) | (where & 0xffc));
	INDIRECT_WRITE(sc, PCIE_CH0_DMA_REMOTE_ADDR_HIGH, 0);

	byte_enables = ((1 << size) - 1) << offset;

	INDIRECT_WRITE(sc, PCIE_CH0_DMA_SIMPLE_WRITE, val << (8 * offset));

	INDIRECT_WRITE(sc, PCIE_CH0_DMA_START,
	    PCIE_CH_STR_START | PCIE_CH_STR_LAST_ADDR_BE(0x0) | 
	    PCIE_CH_STR_FIRST_ADDR_BE(byte_enables) |
	    PCIE_CH_STR_ATTR(0) | PCIE_CH_STR_TYPE(TYPE_CFG0) |
	    PCIE_CH_STR_CLASS(0) | PCIE_CH_STR_SIMPLE_MODE);
 
	while ((status = INDIRECT_READ(sc, PCIE_CH0_DMA_STATUS)) & PCIE_CH_STS_SIMPLE_REQ_BUSY) ;

	if (PCIE_CH_STS_SIMPLE_REQ_STATUS_GET(status)) {
		aprint_error("config write error\n");
	}

	PCIE_CONF_UNLOCK(s);

	return;
}

#ifdef __HAVE_PCI_CONF_HOOK
static int
m83pcie_conf_hook(void *v, int b, int d, int f, pcireg_t id)
{
	return PCI_CONF_DEFAULT;
}
#endif

static void
m83pcie_conf_interrupt(void *v, int bus, int dev, int ipin, int swiz,
    int *ilinep)
{
	/* nothing to do */
}

static int
m83pcie_intr_map(const struct pci_attach_args *pa, pci_intr_handle_t *ih)
{
	if (pa->pa_intrpin == 0)
		return EINVAL;
	*ih = pa->pa_intrpin;
	return 0;
}

static const char *
m83pcie_intr_string(void *v, pci_intr_handle_t ih, char *buf, size_t len)
{
	if (ih == PCI_INTERRUPT_PIN_NONE)
		return NULL;

	snprintf(buf, len, "pci");

	return buf;
}

const struct evcnt *
m83pcie_intr_evcnt(void *v, pci_intr_handle_t ih)
{
	return NULL;
}

static void *
m83pcie_intr_establish(void *v, pci_intr_handle_t ih, int ipl,
    int (*callback)(void *), void *arg, const char *xname)
{
	struct m83pcie_softc *sc = v;
	struct m83pcie_ih *pcie_ih;

	if (ih == 0)
		return NULL;

	pcie_ih = kmem_alloc(sizeof(*pcie_ih), KM_SLEEP);
	pcie_ih->ih_handler = callback;
	pcie_ih->ih_arg = arg;
	pcie_ih->ih_ipl = ipl;

	mutex_enter(&sc->sc_lock);
	TAILQ_INSERT_TAIL(&sc->sc_intrs, pcie_ih, ih_entry);
	sc->sc_intrgen++;
	mutex_exit(&sc->sc_lock);

	return pcie_ih;
}

static void
m83pcie_intr_disestablish(void *v, void *vih)
{
	struct m83pcie_softc *sc = v;
	struct m83pcie_ih *pcie_ih = vih;

	mutex_enter(&sc->sc_lock);
	TAILQ_REMOVE(&sc->sc_intrs, pcie_ih, ih_entry);
	sc->sc_intrgen++;
	mutex_exit(&sc->sc_lock);

	kmem_free(pcie_ih, sizeof(*pcie_ih));
}

static int	m83pcie_match(device_t, cfdata_t, void *);
static void	m83pcie_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(m83pcie, sizeof(struct m83pcie_softc),
    m83pcie_match, m83pcie_attach, NULL, NULL); 

static int  
m83pcie_match(device_t parent, cfdata_t cf, void *aux)
{        

	return 1;
}    

static void 
m83pcie_attach(device_t parent, device_t self, void *aux)
{
	struct m83pcie_softc * const sc = device_private(self);
	struct ahb_attach_args * const ahb = aux;
	const char * const devname = device_xname(self);

	sc->sc_dev = self;
	sc->sc_iot = ahb->ahba_memt;
	sc->sc_dmat = ahb->ahba_dmat;
	sc->sc_cookie = sc;
	sc->sc_pci_netbsd_configure = m83pcie_configure;

	aprint_naive(": PCIe Interfacer\n");
	aprint_normal(": PCIe Interface\n");

	/* Map I/O registers */
	if (bus_space_map(sc->sc_iot, ahb->ahba_addr, ahb->ahba_size,
	    0, &sc->sc_ioh)) {
		aprint_error("%s: can't map memory space\n", devname);
		return;
	}

	intr_establish(ahb->ahba_intr, IPL_VM, IST_LEVEL, m83pcie_intr, sc);

	m83pcie_attach_common(sc);
}
