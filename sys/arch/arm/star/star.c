/*	$NetBSD$	*/

/*-
 * Copyright (c) 2009 SHIMIZU Ryo <ryo@nerv.org>
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
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>

#include <uvm/uvm_extern.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>
#include <arm/star/if_gecreg.h>
#include <arm/star/star_equuleus_intr.h>
#include <arm/star/star_orion_intr.h>

#include "locators.h"

struct star_softc {
	device_t sc_dev;

	bus_space_tag_t sc_iot;
	bus_space_tag_t sc_a4x_iot;

	bus_dma_tag_t sc_dmat;
};

static int star_match(device_t, struct cfdata *, void *);
static void star_attach(device_t, device_t, void *);
static int star_search(device_t, struct cfdata *, const int *, void *);
static int star_critical_search(device_t, struct cfdata *, const int *, void *);
static int star_search(device_t, struct cfdata *, const int *, void *);
static int star_print(void *, const char *);

CFATTACH_DECL_NEW(star, sizeof(struct star_softc),
    star_match, star_attach, NULL, NULL);

/*
 * statially mapped devices
 */
struct star_devtable star_soc_devtable[] = {
	{
		.paddr = STR8100_FLASH_SRAM_MEMORY_BANK0,
		.size = STAR_IO_SIZE,
		.has_str8100 = 1, .has_str9100 = 0
	},
	{
		.paddr = STR8100_FLASH_SRAM_MEMORY_BANK1,
		.size = STAR_IO_SIZE,
		.has_str8100 = 1, .has_str9100 = 0
	},
	{
		.paddr = STR8100_FLASH_SRAM_MEMORY_BANK2,
		.size = STAR_IO_SIZE,
		.has_str8100 = 1, .has_str9100 = 0
	},
	{
		.paddr = STR8100_FLASH_SRAM_MEMORY_BANK3,
		.size = STAR_IO_SIZE,
		.has_str8100 = 1, .has_str9100 = 0
	},
	{
		.paddr = STR9100_STATIC_MEMORY_CONTROL_REGISTER,
		.size = 0x00000010,
		.has_str8100 = 0, .has_str9100 = 1
	},
	{
		.paddr = STR9100_SDRAM_CONTROL_REGISTER,
		.size = 0x00000030,
		.has_str8100 = 0, .has_str9100 = 1
	},
	{
		.paddr = STRx100_GENERIC_DMA_REGISTER,
		.size = 0x000002f8,
		.has_str8100 = 1, .has_str9100 = 1
	},
	{
		.paddr = STR8100_SDRAM_CONTROL_REGISTER,
		.size = 0x00000040,
		.has_str8100 = 1, .has_str9100 = 0
	},
	{
		.paddr = STR8100_STATIC_MEMORY_CONTROL_REGISTER,
		.size = 0x00000020,
		.has_str8100 = 1, .has_str9100 = 0
	},
	{
		.paddr = STRx100_MISC_REGISTER,
		.size = 0x00000060,
		.has_str8100 = 1, .has_str9100 = 1
	},
	{
		.paddr = STRx100_POWER_MANAGEMENT_REGISTER,
		.size = 0x00000030,
		.has_str8100 = 1, .has_str9100 = 1
	},
	{
		.paddr = STRx100_UART0_REGISTER,
		.size = STAR_UART_SIZE,
		.has_str8100 = 1, .has_str9100 = 1
	},
	{
		.paddr = STR8100_UART1_REGISTER,
		.size = STAR_UART_SIZE,
		.has_str8100 = 1, .has_str9100 = 0
	},
	{
		.paddr = STRx100_TIMER_REGISTER,
		.size = STAR_TIMER_REGSIZE,
		.has_str8100 = 1, .has_str9100 = 1
	},
	{
		.paddr = STR9100_INTERRUPT_CONTROL_REGISTER,
		.size = 0x0000001c,
		.has_str8100 = 0, .has_str9100 = 1
	},
	{
		.paddr = STR8100_INTERRUPT_CONTROL_REGISTER,
		.size = 0x00001000,
		.has_str8100 = 1, .has_str9100 = 0
	},
	{
		0, 0, 0, 0, 0
	}
};

int star_cpu_type;


/* ARGSUSED */
static int
star_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux __unused)
{
	return 1;
}

/* ARGSUSED */
static void
star_attach(device_t parent __unused, device_t self, void *aux __unused)
{
	struct star_softc *sc;
	struct star_attach_args sa;
	int cpuclk;
	const char *cpu_family;

	sc = device_private(self);
	sc->sc_iot = &star_bs_tag;
	sc->sc_a4x_iot = &star_a4x_bs_tag;
	sc->sc_dmat = &star_bus_dma_tag;

	if (CPU_RUNTIME_STR8100()) {
		/* STR8100 */
		cpu_family = "Star Equuleus Family SoC: STR8100";
#ifndef STAR_EQUULEUS
		panic("No support for STR8100 (STAR_EQUULEUS) in kernel");
#endif
		/* cpuclk: 0:175MHz, 1:200MHz, 2:225MHz, 3:250MHz */
		cpuclk = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKCTRL_REG) &
		    EQUULEUS_CLKPWR_CLKCTRL_CPU_PLL_SEL_MASK;
	} else {
		/* STR9100 */
		cpu_family = "Star Orion Family SoC: STR9100";
#ifndef STAR_ORION
		panic("No support for STR9100 (STAR_ORION) in kernel");
#endif
		/* cpuclk: 0:175MHz, 1:200MHz, 2:225MHz, 3:250MHz */
		cpuclk = STAR_REG_READ32(ORION_CLKPWR_CLKCTRL_REG) &
		    ORION_CLKPWR_CLKCTRL_CPU_PLL_SEL_MASK;

#if 0
		/* XXX: when 250MHz or 200MHz, VDD_25_SEL must be 4 */
		switch (cpuclk) {
		case 1:
		case 3:
			{
				uint32_t val;
				val = STAR_REG_READ32(ORION_CLKPWR_REGULATORCTRL_REG) &
				    ~ORION_CLKPWR_REGULATORCTRL_VDD_25_SEL_MASK;
				STAR_REG_WRITE32(ORION_CLKPWR_REGULATORCTRL_REG,
				    val | ORION_CLKPWR_REGULATORCTRL_VDD_25_SEL(4));
			}
			break;
		default:
			break;
		}
#endif
	}

	aprint_normal(": %s", cpu_family);
	if (cpuclk < 0)
		aprint_normal("\n");
	else
		aprint_normal(", %dMHz\n", 175 + cpuclk * 25);
	aprint_naive("\n");
	config_search(self, &sa, CFARGS(.search = star_critical_search));
	config_search(self, &sa, CFARGS(.search = star_search));
}

/* ARGSUSED */
static int
star_critical_search(device_t parent, struct cfdata *cf,
    const int *ldesc __unused, void *aux)
{
	struct star_softc *sc;
	struct star_attach_args *sa;

	sc = device_private(parent);
	sa = aux;

	if (strcmp(cf->cf_name, "starclk") != 0)
		return 0;

	sa->sa_iot = sc->sc_iot;
	sa->sa_dmat = sc->sc_dmat;
	sa->sa_addr = cf->cf_loc[STARCF_ADDR];
	sa->sa_irq = cf->cf_loc[STARCF_IRQ];
	sa->sa_size = 0;

	if (config_match(parent, cf, aux) > 0)
		config_attach(parent, cf, aux, star_print, CFARGS_NONE);

	return 0;
}


/* ARGSUSED */
static int
star_search(device_t parent, struct cfdata *cf, const int *ldesc __unused,
    void *aux)
{
	struct star_softc *sc;
	struct star_attach_args *sa;

	sc = device_private(parent);
	sa = aux;

	if (strcmp(cf->cf_name, "com") == 0) {
		/* 32bit mapped 8bit I/O devices */
		sa->sa_iot = sc->sc_a4x_iot;
	} else {
		/* normal memory mapped device */
		sa->sa_iot = sc->sc_iot;
	}
	sa->sa_dmat = sc->sc_dmat;
	sa->sa_addr = cf->cf_loc[STARCF_ADDR];
	sa->sa_irq = cf->cf_loc[STARCF_IRQ];
	sa->sa_size = 0;

	if (config_match(parent, cf, aux) > 0)
		config_attach(parent, cf, aux, star_print, CFARGS_NONE);

	return 0;
}

/* ARGSUSED */
static int
star_print(void *aux, const char *name __unused)
{
	struct star_attach_args *sa;

	sa = (struct star_attach_args *)aux;

	if (sa->sa_size)
		aprint_normal(" addr 0x%lx", sa->sa_addr);
	if (sa->sa_size > 1)
		aprint_normal("-0x%lx", sa->sa_addr + sa->sa_size - 1);
	if (sa->sa_irq >= 0)
		aprint_normal(" irq %d", sa->sa_irq);

	return UNCONF;
}

/*
 * star_initialize() must be called before using splx() and PA=VA.
 */
void
star_initialize(void)
{
	if (STAR_REG_READ32_PHYS(EQUULEUS_MISC_PCI_ID_CAPABILITY) == 0x8131eeee) {
		star_cpu_type = 8;
	} else if (STAR_REG_READ32_PHYS(ORION_MISC_PCI_ID_CAPABILITY) == 0x0000eeee) {
		star_cpu_type = 9;
	} else {
		star_cpu_type = 0;
	}

	if (CPU_IS_STR8100()) {
#ifdef STAR_EQUULEUS
		star_equuleus_init();
#endif
	} else {
#ifdef STAR_ORION
		star_orion_init();
#endif
	}
}

void
star_reset(void)
{
	if (CPU_IS_STR8100())
		STAR_REG_WRITE32(EQUULEUS_CLKPWR_SOFTRST_REG, 0);
	else
		STAR_REG_WRITE32(ORION_CLKPWR_SOFTRST_REG, 0);
}

int
star_get_memsize(int use_phys)
{
	int memsize;
	uint32_t val;

	memsize = 0;

	/*
	 * Use physical address, because this function would be called
	 * when virtual I/O address is not mapped yet.
	 */

	if (CPU_RUNTIME_STR8100()) {
		/* STR8100 */
		if (use_phys) {
			val = STAR_REG_READ32_PHYS(EQUULEUS_SDRAMCTRL_MEM_CFG_REG);
		} else {
			val = STAR_REG_READ32(EQUULEUS_SDRAMCTRL_MEM_CFG_REG);
		}

		switch (val & EQUULEUS_SDRAMCTRL_MEM_CFG_MEMSZ_MASK) {
		case EQUULEUS_SDRAMCTRL_MEM_CFG_MEMSZ_128Mb:
			memsize = 128 * 1024 * 1024 / 8;
			break;
		case EQUULEUS_SDRAMCTRL_MEM_CFG_MEMSZ_256Mb:
			memsize = 256 * 1024 * 1024 / 8;
			break;
		case EQUULEUS_SDRAMCTRL_MEM_CFG_MEMSZ_512Mb:
			memsize = 512 * 1024 * 1024 / 8;
			break;
		case EQUULEUS_SDRAMCTRL_MEM_CFG_MEMSZ_1Gb:
			memsize = 1024 * 1024 * 1024 / 8;
			break;
		case EQUULEUS_SDRAMCTRL_MEM_CFG_MEMSZ_16Mb:
			memsize = 16 * 1024 * 1024 / 8;
			break;
		case EQUULEUS_SDRAMCTRL_MEM_CFG_MEMSZ_64Mb:
			memsize = 64 * 1024 * 1024 / 8;
			break;
		}
	} else if (CPU_RUNTIME_STR9100()) {
		/* STR9100 */
		if (use_phys) {
			val = STAR_REG_READ32_PHYS(ORION_SDRAMCTRL_BANK_CFG_REG);
		} else {
			val = STAR_REG_READ32(ORION_SDRAMCTRL_BANK_CFG_REG);
		}

		switch (val & ORION_SDRAMCTRL_BANK_CFG_BNK_DSZ_MASK) {
		case ORION_SDRAMCTRL_BANK_CFG_BNK_DSZ_64Mb:
			memsize = 64 * 1024 * 1024 / 8;
			break;
		case ORION_SDRAMCTRL_BANK_CFG_BNK_DSZ_128Mb:
			memsize = 128 * 1024 * 1024 / 8;
			break;
		case ORION_SDRAMCTRL_BANK_CFG_BNK_DSZ_256Mb:
			memsize = 256 * 1024 * 1024 / 8;
			break;
		case ORION_SDRAMCTRL_BANK_CFG_BNK_DSZ_512Mb:
			memsize = 512 * 1024 * 1024 / 8;
			break;
		}

		/* is external bus 32bit */
		if (val & ORION_SDRAMCTRL_BANK_CFG_BNK_MBW)
			memsize *= 2;	/* XXX? */

	} else {
		panic("uninitialized star_cpu_type");
	}

	return memsize;
}

int
star_build_devmap(struct pmap_devmap *devmap, int ndevmap)
{
	int i, n;

	for (n = 0, i = 0; star_soc_devtable[i].size != 0; i++) {
		if ((CPU_IS_STR8100() && star_soc_devtable[i].has_str8100) ||
		    (CPU_IS_STR9100() && star_soc_devtable[i].has_str9100)) {

			devmap[n].pd_pa = star_soc_devtable[i].paddr;
			devmap[n].pd_va = STAR_IO_P2V(star_soc_devtable[i].paddr);
			devmap[n].pd_size = star_soc_devtable[i].size;
			devmap[n].pd_prot = VM_PROT_READ | VM_PROT_WRITE;
			devmap[n].pd_cache = PTE_NOCACHE;

			n++;
			if (n >= ndevmap)
				return -1;
		}
	}

	/* clear last element */
	memset(&devmap[n], 0, sizeof(devmap[n]));

	return n;
}
