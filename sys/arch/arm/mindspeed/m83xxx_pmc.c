/*	$NetBSD$	*/

/*-
 * Copyright (c) 2015 Jared D. McNeill <jmcneill@invisible.ca>
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>
#include <sys/kernel.h>

#include <arm/mindspeed/m83xxx_reg.h>
#include <arm/mindspeed/m83xxx_var.h>

#define USB_REFCLK_PD		(1 << 24)
#define USB_AHBCLK_PD		(1 << 19)
#define PCIE1_AHBCLK_PD		(1 << 15)
#define PCIE0_AHBCLK_PD		(1 << 14)
#define PCIE_REFCLK_NP_PD	(1 << 6)
#define ARM0_FCLK_PD		(1 << 0)
#define ARM1_FCLK_PD		(1 << 1)
#define GEMAC0_REFCLK_PD	(1 << 2)
#define GEMAC1_REFCLK_PD	(1 << 3)
#define PHY_REFCLK_PD		(1 << 4)
#define DDR_CLK_PD		(1 << 5)
#define IPSEC_CORECLK_PD	(1 << 8)
#define ARM0_AHBCLK_PD		(1 << 9)
#define ARM1_AHBCLK_PD		(1 << 10)
#define GEMAC0_AHBCLK_PD	(1 << 11)
#define GEMAC1_AHBCLK_PD	(1 << 12)
#define DDRCTRL_AHBCLK_PD	(1 << 13)
#define TDM_AHBCLK_PD		(1 << 16)
#define MDMA_AHBCLK_PD		(1 << 17)
#define UART_AHBCLK_PD		(1 << 18)
#define I2CSPI_AHBCLK_PD	(1 << 20)
#define IPSEC_AHBCLK_PD		(1 << 21)
#define TDM_CLK_PD		(1 << 22)
#define IPSEC2_AHBCLK_PD	(1 << 23)

static int	m83xxx_pmc_match(device_t, cfdata_t, void *);
static void	m83xxx_pmc_attach(device_t, device_t, void *);
static void	m83xxx_pmc_init(void);

struct m83xxx_pmc_softc {
	device_t		pmc_dev;
	bus_space_tag_t		pmc_memt;
	bus_space_handle_t	pmc_memh;
};

static struct m83xxx_pmc_softc *pmc_softc = NULL;

CFATTACH_DECL_NEW(m83pmc, sizeof(struct m83xxx_pmc_softc),
	m83xxx_pmc_match, m83xxx_pmc_attach, NULL, NULL);

static int
m83xxx_pmc_match(device_t parent, cfdata_t cf, void *aux)
{
	struct apb_attach_args * const apba = aux;

	if (apba->apba_addr != APB_CLK_BASE)
		return 0;

	return 1;
}

static void
m83xxx_pmc_attach(device_t parent, device_t self, void *aux)
{
	struct m83xxx_pmc_softc * const pmc = device_private(self);
	struct apb_attach_args * const apba = aux;
	int error;

	apba->apba_size = APB_INTC_SIZE;
	pmc->pmc_dev = self;
	pmc->pmc_memt = apba->apba_memt;
	error = bus_space_map(pmc->pmc_memt, apba->apba_addr, apba->apba_size,
	    0, &pmc->pmc_memh);
	if (error)
		panic("pmc_attach: failed to map register %#lx-%#lx: %d",
		    apba->apba_addr, apba->apba_addr + apba->apba_size - 1,
		    error);

	KASSERT(pmc_softc == NULL);
	pmc_softc = pmc;

	m83xxx_pmc_init();

	aprint_naive("\n");
	aprint_normal(": PMC\n");
}

static void
m83xxx_pmc_init(void)
{
	uint32_t reg;

	reg = bus_space_read_4(pmc_softc->pmc_memt, pmc_softc->pmc_memh,
	    CLK_CLK_PWR_DWN);
	reg |= PCIE_REFCLK_NP_PD;
	reg |= PCIE0_AHBCLK_PD;
	reg |= PCIE1_AHBCLK_PD;
	reg |= TDM_AHBCLK_PD;
	reg |= ARM1_AHBCLK_PD;
	reg |= I2CSPI_AHBCLK_PD;
	reg |= USB_AHBCLK_PD;
	bus_space_write_4(pmc_softc->pmc_memt, pmc_softc->pmc_memh,
	    CLK_CLK_PWR_DWN, reg);
}

#if 0
void
m83xxx_pmc_power(u_int partid, bool enable)
{
	bus_space_tag_t bst;
	bus_space_handle_t bsh;
	uint32_t status, toggle;
	bool state;
	int retry = 10000;

	m83xxx_pmc_get_bs(&bst, &bsh);

	status = bus_space_read_4(bst, bsh, PMC_PWRGATE_STATUS_0_REG);
	state = !!(status & __BIT(partid));
	if (state == enable)
		return;

	while (--retry > 0) {
		toggle = bus_space_read_4(bst, bsh, PMC_PWRGATE_TOGGLE_0_REG);
		if ((toggle & PMC_PWRGATE_TOGGLE_0_START) == 0)
			break;
		delay(1);
	}
	if (retry == 0) {
		printf("ERROR: Couldn't enable PMC partition %#x\n", partid);
		return;
	}

	bus_space_write_4(bst, bsh, PMC_PWRGATE_TOGGLE_0_REG,
	    __SHIFTIN(partid, PMC_PWRGATE_TOGGLE_0_PARTID) |
	    PMC_PWRGATE_TOGGLE_0_START);
}

void
m83xxx_pmc_remove_clamping(u_int partid)
{
	bus_space_tag_t bst;
	bus_space_handle_t bsh;

	m83xxx_pmc_get_bs(&bst, &bsh);

	if (partid == PMC_PARTID_TD) {
		/*
		 * On Tegra124 and later, the GPU power clamping is
		 * controlled by a separate register
		 */
		bus_space_write_4(bst, bsh, PMC_GPU_RG_CNTRL_REG, 0);
		return;
	}

	bus_space_write_4(bst, bsh, PMC_REMOVE_CLAMPING_CMD_0_REG,
	    __BIT(partid));
}

void
m83xxx_pmc_hdmi_enable(void)
{
	bus_space_tag_t bst;
	bus_space_handle_t bsh;

	m83xxx_pmc_get_bs(&bst, &bsh);

	m83xxx_reg_set_clear(bst, bsh, PMC_IO_DPD_STATUS_REG,
	    0, PMC_IO_DPD_STATUS_HDMI);
	m83xxx_reg_set_clear(bst, bsh, PMC_IO_DPD2_STATUS_REG,
	    0, PMC_IO_DPD2_STATUS_HV);
}
#endif
