/*	$NetBSD$	*/

/*-
 * Copyright (c) 2012 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Matt Thomas of 3am Software Foundry.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_arm_debug.h"
#include "opt_console.h"
#include "opt_evbarm_boardtype.h"
#include "opt_kgdb.h"
#include "opt_m86.h"
#include "com.h"
#include "pci.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/atomic.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/reboot.h>
#include <sys/termios.h>

#include <dev/cons.h>

#include <uvm/uvm_extern.h>

#include <arm/db_machdep.h>
#include <arm/undefined.h>
#include <arm/arm32/machdep.h>

#include <machine/autoconf.h>
#include <machine/bootconfig.h>

#include <arm/cortex/scu_reg.h>
#include <arm/mindspeed/m86xxx_reg.h>
#include <arm/mindspeed/m86xxx_var.h>

#if NCOM == 0
#error missing COM device for console
#endif

#include <dev/ic/comreg.h>
#include <dev/ic/comvar.h>

#define	KERNEL_VM_BASE		(KERNEL_BASE + 0x40000000)
#define KERNEL_VM_SIZE		0x0C000000
#define KERNEL_IO_VBASE		VM_KERNEL_IO_BASE

extern int _end[];
extern int KERNEL_BASE_phys[];
extern int KERNEL_BASE_virt[];

BootConfig bootconfig;
char *boot_args = NULL;

/* filled in before cleaning bss. keep in .data */
u_int uboot_args[4] __attribute__((__section__(".data")));

static void m86xxx_system_reset(void);

#ifndef CONADDR
#define CONADDR		UART_BASEADDR
#endif
#ifndef CONSPEED
#define CONSPEED	B115200
#endif
#ifndef CONMODE
#define CONMODE	((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */
#endif

void m86xxx_mpstart(void);
void m86xxx_platform_early_putchar(char);

#if (NCOM > 0)
static const bus_addr_t comcnaddr = (bus_addr_t)CONADDR;

int comcnspeed = CONSPEED;
int comcnmode = CONMODE | CLOCAL;
#endif

#ifdef KGDB
#include <sys/kgdb.h>
#endif

#ifdef VERBOSE_INIT_ARM
#define VPRINTF(...)    printf(__VA_ARGS__)
#else
#define VPRINTF(...)    __nothing
#endif

static void
earlyconsputc(dev_t dev, int c)
{
	uartputc(c);
}

static int
earlyconsgetc(dev_t dev)
{
	return -1;
}

static struct consdev earlycons = {
	.cn_putc = earlyconsputc,
	.cn_getc = earlyconsgetc,
	.cn_pollc = nullcnpollc,
};

/*
 * Static device mappings. These peripheral registers are mapped at
 * fixed virtual addresses very early in initarm() so that we can use
 * them while booting the kernel, and stay at the same address
 * throughout whole kernel's life time.
 *
 * We use this table twice; once with bootstrap page table, and once
 * with kernel's page table which we build up in initarm().
 *
 * Since we map these registers into the bootstrap page table using
 * pmap_devmap_bootstrap() which calls pmap_map_chunk(), we map
 * registers segment-aligned and segment-rounded in order to avoid
 * using the 2nd page tables.
 */

static const struct pmap_devmap m86xxx_devmap[] = {
	DEVMAP_ENTRY(
		KERNEL_IO_VBASE,	/* 0xf0000000 */
		UART_BASEADDR,
		0x00100000
	),
	DEVMAP_ENTRY(
		KERNEL_IO_VBASE + 0x00100000,
		AXI_APB_CFG_BASE,
		0x00100000
	),
	DEVMAP_ENTRY(
		KERNEL_IO_VBASE + 0x00200000,
		AXI_IRAM_BASE,
		0x00100000
	),
	DEVMAP_ENTRY(
		KERNEL_IO_VBASE + 0x00300000,
		A9_PERIPH_BASE,
		0x00100000
	),
#if 0
	DEVMAP_ENTRY(
		KERNEL_IO_IOREG_VBASE,
		BCM53XX_IOREG_PBASE,		/* 0x18000000 */
		BCM53XX_IOREG_SIZE		/* 2MB */
	),
	DEVMAP_ENTRY(
		KERNEL_IO_ARMCORE_VBASE,
		BCM53XX_ARMCORE_PBASE,		/* 0x19000000 */
		BCM53XX_ARMCORE_SIZE		/* 1MB */
	),
	DEVMAP_ENTRY(
		KERNEL_IO_ROM_REGION_VBASE,
		BCM53XX_ROM_REGION_PBASE,	/* 0xfff00000 */
		BCM53XX_ROM_REGION_SIZE		/* 1MB */
	),
#if NPCI > 0
	DEVMAP_ENTRY(
		KERNEL_IO_PCIE0_OWIN_VBASE,
		BCM53XX_PCIE0_OWIN_PBASE,	/* 0x08000000 */
		BCM53XX_PCIE0_OWIN_SIZE		/* 4MB */
	),
	DEVMAP_ENTRY(
		KERNEL_IO_PCIE1_OWIN_VBASE,
		BCM53XX_PCIE1_OWIN_PBASE,	/* 0x40000000 */
		BCM53XX_PCIE1_OWIN_SIZE		/* 4MB */
	),
	DEVMAP_ENTRY(
		KERNEL_IO_PCIE2_OWIN_VBASE,
		BCM53XX_PCIE2_OWIN_PBASE,	/* 0x48000000 */
		BCM53XX_PCIE2_OWIN_SIZE		/* 4MB */
	),
#endif /* NPCI > 0 */
#endif
	DEVMAP_ENTRY_END
};

static const struct boot_physmem bp_first256 = {
	.bp_start = 0x80000000 / NBPG,
	.bp_pages = 0x10000000 / NBPG,
	.bp_freelist = VM_FREELIST_ISADMA,
	.bp_flags = 0,
};

#define BCM53xx_ROM_CPU_ENTRY	0xffff0400

#ifdef MULTIPROCESSOR
void m86xxx_cpu_hatch(struct cpu_info *ci);
void
m86xxx_cpu_hatch(struct cpu_info *ci)
{
}
#endif

void
m86xxx_mpstart(void)
{
#ifdef MULTIPROCESSOR
#if 0
	/*
	 * Invalidate all SCU cache tags. That is, for all cores (0-3)
	 */
	bus_space_write_4(m86xxx_armcore_bst, m86xxx_armcore_bsh,
	    ARMCORE_SCU_BASE + SCU_INV_ALL_REG, 0xffff);

	uint32_t diagctl = bus_space_read_4(m86xxx_armcore_bst,
	   m86xxx_armcore_bsh, ARMCORE_SCU_BASE + SCU_DIAG_CONTROL);
	diagctl |= SCU_DIAG_DISABLE_MIGBIT;
	bus_space_write_4(m86xxx_armcore_bst, m86xxx_armcore_bsh,
	    ARMCORE_SCU_BASE + SCU_DIAG_CONTROL, diagctl);

	uint32_t scu_ctl = bus_space_read_4(m86xxx_armcore_bst,
	    m86xxx_armcore_bsh, ARMCORE_SCU_BASE + SCU_CTL);
	scu_ctl |= SCU_CTL_SCU_ENA;
	bus_space_write_4(m86xxx_armcore_bst, m86xxx_armcore_bsh,
	    ARMCORE_SCU_BASE + SCU_CTL, scu_ctl);

	armv7_dcache_wbinv_all();

	const paddr_t mpstart = KERN_VTOPHYS((vaddr_t)cpu_mpstart);
	bus_space_tag_t m86xxx_rom_bst = &bcmgen_bs_tag;
	bus_space_handle_t m86xxx_rom_entry_bsh;

	int error = bus_space_map(m86xxx_rom_bst, BCM53xx_ROM_CPU_ENTRY,
	    4, 0, &m86xxx_rom_entry_bsh);

	/*
	 * Before we turn on the MMU, let's the other process out of the
	 * SKU ROM but setting the magic LUT address to our own mp_start
	 * routine.
	 */
	bus_space_write_4(m86xxx_rom_bst, m86xxx_rom_entry_bsh, mpstart);

	dsb(sy);
	sev();

	/* Bitmask of CPUs (non-BP) to start */
	for (u_int cpuindex = 1; cpuindex < arm_cpu_max; cpuindex++) {
		u_int i ;
		for (i = 1500000; i > 0; i--) {
                        if (cpu_hatched_p(cpuindex))
                                break;
                }

                if (i == 0) {
                        ret++;
                        aprint_error("cpu%d: WARNING: AP failed to start\n",
                            cpuindex);
                }
        }
#endif
#endif /* MULTIPROCESSOR */
}

/*
 * vaddr_t initarm(...)
 *
 * Initial entry point on startup. This gets called before main() is
 * entered.
 * It should be responsible for setting up everything that must be
 * in place when main is called.
 * This includes
 *   Taking a copy of the boot configuration structure.
 *   Initialising the physical console so characters can be printed.
 *   Setting up page tables for the kernel
 */
vaddr_t
initarm(void *arg)
{
	cn_tab = &earlycons;

	/*
	 * Heads up ... Setup the CPU / MMU / TLB functions
	 */
	VPRINTF("cpufunc\n");
	if (set_cpufuncs())		// starts PMC counter
		panic("cpu not recognized!");

	VPRINTF("devmap\n");
	extern char ARM_BOOTSTRAP_LxPT[];
	pmap_devmap_bootstrap((vaddr_t)ARM_BOOTSTRAP_LxPT, m86xxx_devmap);

	VPRINTF("bootstrap\n");
	m86xxx_bootstrap(KERNEL_IO_VBASE + 0x00100000);

#ifdef MULTIPROCESSOR
	uint32_t scu_cfg = *(uint32_t *)(KERNEL_IO_VBASE + 0x00300000 +
	    SCU_CFG);
	arm_cpu_max = 1 + (scu_cfg & SCU_CFG_CPUMAX);
	membar_producer();
#endif
	cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2)) | DOMAIN_CLIENT);

	VPRINTF("consinit ");
	consinit();
	VPRINTF("ok\n");

//	m86xxx_cpu_softc_init(curcpu());
//	m86xxx_print_clocks();
	cortex_pmc_ccnt_init();

#if NBCMRNG_CCB > 0
	/*
	 * Start this early since it takes a while to startup up.
	 */
//	m86xxx_rng_start(m86xxx_ioreg_bst, m86xxx_ioreg_bsh);
#endif

	printf("uboot arg = %#x, %#x, %#x, %#x\n",
	    uboot_args[0], uboot_args[1], uboot_args[2], uboot_args[3]);

	/* Talk to the user */
	printf("\nNetBSD/evbarm (" ___STRING(EVBARM_BOARDTYPE) ") booting ...\n");

#if defined(VERBOSE_INIT_ARM) || 1
	printf("initarm: Configuring system");
#ifdef MULTIPROCESSOR
#if 0
	printf(" (%u cpu%s, hatched %#x)",
	    arm_cpu_max + 1, arm_cpu_max + 1 ? "s" : "",
	    arm_cpu_hatched);
#endif
#endif
	printf(", CLIDR=%010o CTR=%#x PMUSERSR=%#x",
	    armreg_clidr_read(), armreg_ctr_read(), armreg_pmuserenr_read());
	printf("\n");
#endif

	psize_t memsize;
#ifndef MEMSIZE
#error missing MEMSIZE
#endif
	memsize = MEMSIZE*1024*1024;
	const bool bigmem_p = (memsize >> 20) > 256;

#ifdef __HAVE_MM_MD_DIRECT_MAPPED_PHYS
	const bool mapallmem_p = true;
#ifndef PMAP_NEED_ALLOC_POOLPAGE
	if (memsize > KERNEL_VM_BASE - KERNEL_BASE) {
		printf("%s: dropping RAM size from %luMB to %uMB\n",
		   __func__, (unsigned long) (ram_size >> 20),
		   (KERNEL_VM_BASE - KERNEL_BASE) >> 20);
		memsize = KERNEL_VM_BASE - KERNEL_BASE;
	}
#endif
#else
	const bool mapallmem_p = false;
#endif
	KASSERT((armreg_pfr1_read() & ARM_PFR1_SEC_MASK) != 0);
	arm32_bootmem_init(KERN_VTOPHYS(KERNEL_BASE), memsize,
	    (paddr_t)KERNEL_BASE_phys);

//	m86xxx_dma_bootstrap(memsize);

	/*
	 * This is going to do all the hard work of setting up the first and
	 * and second level page tables.  Pages of memory will be allocated
	 * and mapped for other structures that are required for system
	 * operation.  When it returns, physical_freestart and free_pages will
	 * have been updated to reflect the allocations that were made.  In
	 * addition, kernel_l1pt, kernel_pt_table[], systempage, irqstack,
	 * abtstack, undstack, kernelstack, msgbufphys will be set to point to
	 * the memory that was allocated for them.
	 */
	arm32_kernel_vm_init(KERNEL_VM_BASE, ARM_VECTORS_HIGH, 0, m86xxx_devmap,
	    mapallmem_p);

	cpu_reset_address = m86xxx_system_reset;
	/* we've a specific device_register routine */
	evbarm_device_register = m86xxx_device_register;
	if (bigmem_p) {
		/*
		 * If we have more than 256MB
		 */
		arm_poolpage_vmfreelist = bp_first256.bp_freelist;
	}

	/*
	 * If we have more than 256MB of RAM, set aside the first 256MB for
	 * non-default VM allocations.
	 */
	vaddr_t sp = initarm_common(KERNEL_VM_BASE, KERNEL_VM_SIZE,
	    (bigmem_p ? &bp_first256 : NULL), (bigmem_p ? 1 : 0));

	/*
	 * initarm_common flushes cache if required before AP start
	 */
	m86xxx_mpstart();

	return sp;
}

int
m83comcnattach(bus_space_tag_t iot, bus_addr_t iobase, int rate,
    int frequency, int type, tcflag_t cflag);
int
m83comcnattach(bus_space_tag_t iot, bus_addr_t iobase, int rate,
    int frequency, int type, tcflag_t cflag)
{
	struct com_regs	regs;

	/*XXX*/
	bus_space_handle_t dummy_bsh;
	memset(&dummy_bsh, 0, sizeof(dummy_bsh));

	/*
	 * dummy_bsh required because com_init_regs() wants it.  A
	 * real bus_space_handle will be filled in by cominit() later.
	 * XXXJRT Detangle this mess eventually, plz.
	 */
	com_init_regs_stride(&regs, iot, dummy_bsh/*XXX*/, iobase,
		2);

	return comcnattach1(&regs, rate, frequency, type, cflag);
}

void
consinit(void)
{
	static bool consinit_called = false;
//	uint32_t v;
	if (consinit_called)
		return;

	consinit_called = true;

#if 0
	/*
	 * Force UART clock to the reference clock
	 */
	v = bus_space_read_4(m86xxx_ioreg_bst, m86xxx_ioreg_bsh,
	    IDM_BASE + IDM_APBX_BASE + IDM_IO_CONTROL_DIRECT);
	v &= ~IO_CONTROL_DIRECT_UARTCLKSEL;
	bus_space_write_4(m86xxx_ioreg_bst, m86xxx_ioreg_bsh,
	    IDM_BASE + IDM_APBX_BASE + IDM_IO_CONTROL_DIRECT, v);

	/*
	 * Switch to the reference clock
	 */
	v = bus_space_read_4(m86xxx_ioreg_bst, m86xxx_ioreg_bsh,
	    CCA_MISC_BASE + MISC_CORECTL);
	v &= ~CORECTL_UART_CLK_OVERRIDE;
	bus_space_write_4(m86xxx_ioreg_bst, m86xxx_ioreg_bsh,
	    CCA_MISC_BASE + MISC_CORECTL, v);

#endif
	if (m83comcnattach(&m83_bs_tag, comcnaddr, comcnspeed,
	    clock_info.clk_axi, COM_TYPE_16550_NOERS, comcnmode))
                panic("Serial console can not be initialized.");
}

static void
m86xxx_system_reset(void)
{
	bus_space_handle_t bsh;
	bus_space_map(&m83_bs_tag, APB_GPIO_BASE, 0x20000, 0, &bsh);
	bus_space_write_4(&m83_bs_tag, bsh, 0x00, 0);
	while (1);
	/* not reach here */
	bus_space_unmap(&m83_bs_tag, bsh, 0x20000);
}


void __noasan
m86xxx_platform_early_putchar(char c)
{
#ifdef CONSADDR

	volatile uint32_t *uartaddr;
	if(cpu_earlydevice_va_p()) {
		uartaddr = (volatile uint32_t *)KERNEL_IO_VBASE;

		while ((uartaddr[5] & (1 << 6)) == 0)
			;

		uartaddr[0] = c;
	} else {
		uartaddr = (volatile uint32_t *)CONSADDR;

		while ((uartaddr[5] & (1 << 6)) == 0)
			;

		uartaddr[0] = c;
	}
#endif
}
