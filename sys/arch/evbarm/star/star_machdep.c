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

#include "opt_kgdb.h"

#include <sys/param.h>
#include <sys/device.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/exec.h>
#include <sys/proc.h>
#include <sys/msgbuf.h>
#include <sys/reboot.h>
#include <sys/termios.h>
#include <sys/ksyms.h>
#include <sys/bus.h>

#include <uvm/uvm_extern.h>

#include <dev/cons.h>

#include <machine/db_machdep.h>
#include <ddb/db_sym.h>
#include <ddb/db_extern.h>
#ifdef KGDB
#include <sys/kgdb.h>
#endif

#include <machine/autoconf.h>
#include <machine/bootconfig.h>
#include <machine/cpu.h>
#include <machine/frame.h>
#include <arm/undefined.h>

#include <arm/arm32/machdep.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

/* Kernel text starts from the bottom of the kernel address space. */
#define	KERNEL_TEXT_BASE	(KERNEL_BASE + 0x00000000)
#define	KERNEL_VM_BASE		(KERNEL_BASE + 0x01000000)

/*
 * The range 0xc1000000 - 0xe1ffffff is available for kernel VM space
 * Core-logic registers and I/O mappings occupy 0xf0000000 - 0xffffffff
 */
#define KERNEL_VM_SIZE		0x20000000

/*
 * Address to call from cpu_reset() to reset the machine.
 * This is machine architecture dependant as it varies depending
 * on where the ROM appears when you turn the MMU off.
 */

//u_int cpu_reset_address = ARM_VECTORS_HIGH;

/* Define various stack sizes in pages */
#define FIQ_STACK_SIZE	1
#define IRQ_STACK_SIZE	1
#define ABT_STACK_SIZE	1
#define UND_STACK_SIZE	1

BootConfig bootconfig;	/* Boot config storage */
char *boot_args = NULL;

//vaddr_t physical_start;
vaddr_t physical_freestart;
vaddr_t physical_freeend;
//vaddr_t physical_end;
u_int free_pages;
//int physmem = 0;

/* Physical and virtual addresses for some global pages */
/*
pv_addr_t fiqstack;
pv_addr_t irqstack;
pv_addr_t undstack;
pv_addr_t abtstack;
pv_addr_t kernelstack;
*/

//paddr_t msgbufphys;
extern paddr_t msgbufphys;

extern u_int data_abort_handler_address;
extern u_int prefetch_abort_handler_address;
//extern u_int undefined_handler_address;

extern char etext[];
extern char _end[];

#define KERNEL_PT_SYS_NUM	1	/* Page table for system vectors */
#define KERNEL_PT_KERNEL_NUM	4	/* Page table for mapping kernel */
#define KERNEL_PT_VMDATA_NUM	4	/* Page tables for mapping kernel VM. start with 16MB of KVM */
#define KERNEL_PT_SOC_NUM	32	/* number of star_devmap[] */

#define NUM_KERNEL_SYS_PTS	(KERNEL_PT_SYS_NUM + KERNEL_PT_KERNEL_NUM + KERNEL_PT_VMDATA_NUM)

pv_addr_t kernel_pt_table[NUM_KERNEL_SYS_PTS + KERNEL_PT_SOC_NUM];
static struct pmap_devmap star_devmap[KERNEL_PT_SOC_NUM];

struct user *proc0paddr;

/*
 * Macros to translate between physical and virtual for a subset of the
 * kernel address space.  *Not* for general use.
 */
#define KERNEL_BASE_PHYS	physical_start
#define STR_KERN_VTOPHYS(va) \
	((paddr_t)((vaddr_t)va - KERNEL_BASE + KERNEL_BASE_PHYS))
#define STR_KERN_PHYSTOV(pa) \
	((vaddr_t)((paddr_t)pa - KERNEL_BASE_PHYS + KERNEL_BASE))

#include "com.h"
#if NCOM > 0
#include <dev/ic/comreg.h>
#include <dev/ic/comvar.h>

#ifndef CONSPEED
#define CONSPEED B38400	/* It's a setting of the default of u-boot, etc. */
#endif
#ifndef CONMODE
#define CONMODE ((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8)	/* 8N1 */
#endif
#ifndef CONUNIT
#define CONUNIT 0
#endif

int comcnspeed = CONSPEED;
int comcnmode = CONMODE;
#endif /* NCOM > 0 */


/* ARGSUSED */
static void
star_device_register(device_t dev __unused, void *aux __unused)
{
}

/* ARGSUSED */
void
cpu_reboot(int howto, char *bootstr __unused)
{
	/*
	 * If we are still cold then hit the air brakes
	 * and crash to earth fast
	 */
	if (cold) {
		doshutdownhooks();
		pmf_system_shutdown(boothowto);
		printf("\n");
		printf("The operating system has halted.\n");
		printf("Please press any key to reboot.\n\n");
		cngetc();
		goto reset;
	}

	/* Disable console buffering */
	cnpollc(1);

	/*
	 * If RB_NOSYNC was not specified sync the discs.
	 * Note: Unless cold is set to 1 here, syslogd will die during the
	 * unmount.  It looks like syslogd is getting woken up only to find
	 * that it cannot page part of the binary in as the filesystem has
	 * been unmounted.
	 */
	if (!(howto & RB_NOSYNC))
		bootsync();

	/* Say NO to interrupts */
	splhigh();

	/* Do a dump if requested. */
	if ((howto & (RB_DUMP | RB_HALT)) == RB_DUMP)
		dumpsys();
	
	/* Run any shutdown hooks */
	doshutdownhooks();

	pmf_system_shutdown(boothowto);

	/* Make sure IRQ's are disabled */
	IRQdisable;

	if (howto & RB_HALT) {
		printf("\n");
		printf("The operating system has halted.\n");
		printf("Please press any key to reboot.\n");
		cngetc();
	}

 reset:
	printf("rebooting...\n");

	/* all interrupts are disabled */
	disable_interrupts(I32_bit | F32_bit);

	delay(100000);

	/* hardware reset */
	star_reset();

	/* ...and if that didn't work, just croak. */
	printf("RESET FAILED! HALT\n");
	for (;;)
		halt();
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
 *   Initialising interrupt controllers to a sane default state
 */
/* ARGSUSED */
vaddr_t
initarm(void *arg __unused)
{
	int i, ndevmap, pt_index;
//	u_int kerneldatasize;
	u_int l1pagetable;

	star_initialize();
	ndevmap = star_build_devmap(star_devmap, KERNEL_PT_SOC_NUM);

	pmap_devmap_register(star_devmap);

	consinit();

	printf("\nNetBSD/evbarm STR8100/9100\n");

	/*
	 * Heads up ... Setup the CPU / MMU / TLB functions
	 */
	if (set_cpufuncs())
		panic("cpu not recognized!");

	/* set memsize */
	bootconfig.dramblocks = 1;
	bootconfig.dram[0].address = STAR_SDRAM_MEMORY;
#ifdef MEMSIZE
	bootconfig.dram[0].pages = MEMSIZE / PAGE_SIZE;
#else
	bootconfig.dram[0].pages = star_get_memsize(1) / PAGE_SIZE;
#endif

//	kerneldatasize = (uint32_t)_end - (uint32_t)KERNEL_TEXT_BASE;

	physical_start = bootconfig.dram[0].address;
	physical_end = physical_start + bootconfig.dram[0].pages * PAGE_SIZE;
	/*
	 * Our kernel is at the beginning of memory, so set our free space to
	 * all the memory after the kernel.
	 */
	physical_freestart = STR_KERN_VTOPHYS(round_page((vaddr_t)_end));
	physical_freeend = physical_end;
	physmem = (physical_end - physical_start) / PAGE_SIZE;
	free_pages = (physical_freeend - physical_freestart) / PAGE_SIZE;

#ifdef VERBOSE_INIT_ARM
	/* Tell the user about the memory */
	printf("physmemory: %d pages at 0x%08lx -> 0x%08lx\n", (int)physmem,
	    physical_start, physical_end - 1);
	printf("freestart = 0x%08lx, free_pages = %d (0x%08x)\n",
	       physical_freestart, free_pages, free_pages);
#endif

#ifdef VERBOSE_INIT_ARM
	printf("Allocating page tables\n");
#endif
	if (ndevmap <= 0)
		panic("not enouch star_devmap[] or kernel_pt_table[]. Increase KERNEL_PT_SOC_NUM");

	/*
	 * Define a macro to simplify memory allocation.  As we allocate the
	 * memory, make sure that we don't walk over our temporary first level
	 * translation table.
	 */
#define valloc_pages(var, np)						\
	(var).pv_pa = physical_freestart;				\
	physical_freestart += ((np) * PAGE_SIZE);			\
	if (physical_freestart > (physical_freeend - L1_TABLE_SIZE))	\
		panic("initarm: out of memory");			\
	free_pages -= (np);						\
	(var).pv_va = STR_KERN_PHYSTOV((var).pv_pa);			\
	memset((char *)(var).pv_va, 0, ((np) * PAGE_SIZE));

#define valloc_pages_tail(var, np)					\
	physical_freeend -= ((np) * PAGE_SIZE);				\
	var = physical_freeend;						\
	if (physical_freestart > (physical_freeend - L1_TABLE_SIZE))	\
		panic("initarm: out of memory");			\
	free_pages -= (np);

	kernel_l1pt.pv_pa = 0;
	kernel_l1pt.pv_va = 0;
	for (pt_index = 0, i = 0; i < (1 + NUM_KERNEL_SYS_PTS + ndevmap); i++) {
		/* Are we 16KB aligned for an L1 ? */
		if ((physical_freestart & (L1_TABLE_SIZE - 1)) == 0 &&
		    kernel_l1pt.pv_pa == 0) {
			valloc_pages(kernel_l1pt, L1_TABLE_SIZE / PAGE_SIZE);
		} else {
			valloc_pages(kernel_pt_table[pt_index],
			    L2_TABLE_SIZE / PAGE_SIZE);
			++pt_index;
		}
	}

#ifdef VERBOSE_INIT_ARM
	printf("%s: kernel_l1pt: %#lx:%#lx\n",
	    __func__, kernel_l1pt.pv_va, kernel_l1pt.pv_pa);
	printf("%s: kernel_pt_table:\n", __func__);
	for (i = 0; i < NUM_KERNEL_SYS_PTS + ndevmap; i++) {
		printf("\t%#lx:%#lx\n", kernel_pt_table[i].pv_va,
		    kernel_pt_table[i].pv_pa);
	}
#endif

	/* This should never be able to happen but better confirm that. */
	if (!kernel_l1pt.pv_pa || (kernel_l1pt.pv_pa & (L1_TABLE_SIZE - 1)) != 0)
		panic("initarm: Failed to align the kernel page directory");

	/*
	 * Allocate a page for the system page mapped to V0xFFFF0000
	 * This page will just contain the system vectors and can be
	 * shared by all processes.
	 */
	valloc_pages(systempage, 1);
	systempage.pv_va = ARM_VECTORS_HIGH;

	/* Allocate stacks for all modes */
	valloc_pages(fiqstack, FIQ_STACK_SIZE);
	valloc_pages(irqstack, IRQ_STACK_SIZE);
	valloc_pages(abtstack, ABT_STACK_SIZE);
	valloc_pages(undstack, UND_STACK_SIZE);
	valloc_pages(kernelstack, UPAGES);

	/* Allocate the message buffer from bottom of memory */
	valloc_pages_tail(msgbufphys, round_page(MSGBUFSIZE) / PAGE_SIZE);

#ifdef VERBOSE_INIT_ARM
	printf("FIQ stack: p0x%08lx v0x%08lx\n", fiqstack.pv_pa,
	    fiqstack.pv_va); 
	printf("IRQ stack: p0x%08lx v0x%08lx\n", irqstack.pv_pa,
	    irqstack.pv_va); 
	printf("ABT stack: p0x%08lx v0x%08lx\n", abtstack.pv_pa,
	    abtstack.pv_va); 
	printf("UND stack: p0x%08lx v0x%08lx\n", undstack.pv_pa,
	    undstack.pv_va); 
	printf("SVC stack: p0x%08lx v0x%08lx\n", kernelstack.pv_pa,
	    kernelstack.pv_va); 
	printf("msgbuf   : p0x%08lx\n", msgbufphys);
#endif


	/*
	 * Ok we have allocated physical pages for the primary kernel
	 * page tables.
	 */
#ifdef VERBOSE_INIT_ARM
	printf("Creating L1 page table at 0x%08lx\n", kernel_l1pt.pv_pa);
#endif

	/*
	 * Now we start construction of the L1 page table
	 * We start by mapping the L2 page tables into the L1.
	 * This means that we can replace L1 mappings later on if necessary
	 */

	/* Map the L2 pages tables in the L1 page table */
	l1pagetable = kernel_l1pt.pv_va;
	pt_index = 0;

	/* KERNEL_PT_SYS */
	pmap_link_l2pt(l1pagetable, ARM_VECTORS_HIGH & ~(0x00400000 - 1),
	    &kernel_pt_table[pt_index++]);

	/* KERNEL_PT_KERNEL */
	for (i = 0; i < KERNEL_PT_KERNEL_NUM; i++)
		pmap_link_l2pt(l1pagetable, KERNEL_BASE + i * 0x00400000,
		    &kernel_pt_table[pt_index++]);

	/* KERNEL_PT_VMDATA */
	for (i = 0; i < KERNEL_PT_VMDATA_NUM; i++)
		pmap_link_l2pt(l1pagetable, KERNEL_VM_BASE + i * 0x00400000,
		    &kernel_pt_table[pt_index++]);

	/* KERNEL_PT_SOC */
	for (i = 0; i < ndevmap; i++)
		pmap_link_l2pt(l1pagetable, star_devmap[i].pd_va & ~(0x00400000 - 1),
		    &kernel_pt_table[pt_index++]);


	/* update the top of the kernel VM */
	pmap_curmaxkvaddr =
	    KERNEL_VM_BASE + (KERNEL_PT_VMDATA_NUM * 0x00400000);

#ifdef VERBOSE_INIT_ARM
	printf("Mapping kernel\n");
#endif

	/* Now we fill in the L2 pagetable for the kernel static code/data */
#define round_L_page(x) (((x) + L2_L_OFFSET) & L2_L_FRAME)
	size_t textsize = round_L_page((uint32_t)etext - KERNEL_TEXT_BASE);
	size_t totalsize = round_L_page((uint32_t)_end - KERNEL_TEXT_BASE);
	/* offset of kernel in RAM */
	u_int offset = (u_int)KERNEL_TEXT_BASE - KERNEL_BASE;

#ifdef DDB
	/* Map text section read-write. */
	offset += pmap_map_chunk(l1pagetable,
	    (vaddr_t)KERNEL_BASE + offset,
	     physical_start + offset, textsize,
//	     VM_PROT_READ|VM_PROT_WRITE|VM_PROT_EXECUTE,
	     VM_PROT_READ|VM_PROT_EXECUTE,
	     PTE_CACHE);
#else
	/* Map text section read-only. */
	offset += pmap_map_chunk(l1pagetable,
	    (vaddr_t)KERNEL_BASE + offset,
	     physical_start + offset, textsize,
	     VM_PROT_READ|VM_PROT_EXECUTE, PTE_CACHE);
#endif

	/* Map data and bss sections read-write. */
	offset += pmap_map_chunk(l1pagetable,
	    (vaddr_t)KERNEL_BASE + offset,
	    physical_start + offset, totalsize - textsize,
	    VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);

#ifdef VERBOSE_INIT_ARM
	printf("Constructing L2 page tables\n");
#endif

	/* Map the stack pages */
	pmap_map_chunk(l1pagetable, fiqstack.pv_va, fiqstack.pv_pa,
	    FIQ_STACK_SIZE * PAGE_SIZE, VM_PROT_READ | VM_PROT_WRITE, PTE_CACHE);
	pmap_map_chunk(l1pagetable, irqstack.pv_va, irqstack.pv_pa,
	    IRQ_STACK_SIZE * PAGE_SIZE, VM_PROT_READ | VM_PROT_WRITE, PTE_CACHE);
	pmap_map_chunk(l1pagetable, abtstack.pv_va, abtstack.pv_pa,
	    ABT_STACK_SIZE * PAGE_SIZE, VM_PROT_READ | VM_PROT_WRITE, PTE_CACHE);
	pmap_map_chunk(l1pagetable, undstack.pv_va, undstack.pv_pa,
	    UND_STACK_SIZE * PAGE_SIZE, VM_PROT_READ | VM_PROT_WRITE, PTE_CACHE);
	pmap_map_chunk(l1pagetable, kernelstack.pv_va, kernelstack.pv_pa,
	    UPAGES * PAGE_SIZE, VM_PROT_READ | VM_PROT_WRITE, PTE_CACHE);

	pmap_map_chunk(l1pagetable, kernel_l1pt.pv_va, kernel_l1pt.pv_pa,
	    L1_TABLE_SIZE, VM_PROT_READ | VM_PROT_WRITE, PTE_PAGETABLE);

	for (i = 0; i < NUM_KERNEL_SYS_PTS + ndevmap; i++) {
		pmap_map_chunk(l1pagetable, kernel_pt_table[i].pv_va,
		    kernel_pt_table[i].pv_pa, L2_TABLE_SIZE,
		    VM_PROT_READ | VM_PROT_WRITE, PTE_PAGETABLE);
	}

	/* Map the vector page. */
#ifdef VERBOSE_INIT_ARM
	printf("Mapping the vector page\n");
#endif
	pmap_map_entry(l1pagetable, ARM_VECTORS_HIGH, systempage.pv_pa,
	    VM_PROT_READ|VM_PROT_WRITE, PTE_CACHE);

	/*
	 * Map integrated peripherals at same address in first level page
	 * table so that we can continue to use console.
	 */
#ifdef VERBOSE_INIT_ARM
	printf("devmap_bootstrap\n");
#endif
	pmap_devmap_bootstrap(l1pagetable, star_devmap);

	/* Switch tables */
#ifdef VERBOSE_INIT_ARM
	printf("switching to new L1 page table  @%#lx...", kernel_l1pt.pv_pa);
#endif
	cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2)) | DOMAIN_CLIENT);
//	setttb(kernel_l1pt.pv_pa);
	cpu_setttb(kernel_l1pt.pv_pa, true);
	cpu_tlb_flushID();
	cpu_domains(DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2));

	/*
	 * Moved from cpu_startup() as data_abort_handler() references
	 * this during uvm init
	 */
	proc0paddr = (struct user *)kernelstack.pv_va;
	lwp0.l_addr = proc0paddr;

#ifdef VERBOSE_INIT_ARM
	printf("done.\n");
#endif

	/*
	 * Pages were allocated during the secondary bootstrap for the
	 * stacks for different CPU modes.
	 * We must now set the r13 registers in the different CPU modes to
	 * point to these stacks.
	 * Since the ARM stacks use STMFD etc. we must set r13 to the top end
	 * of the stack memory.
	 */
#ifdef VERBOSE_INIT_ARM
	printf("init subsystems: stacks ");
#endif
	set_stackptr(PSR_FIQ32_MODE, fiqstack.pv_va + FIQ_STACK_SIZE * PAGE_SIZE);
	set_stackptr(PSR_IRQ32_MODE, irqstack.pv_va + IRQ_STACK_SIZE * PAGE_SIZE);
	set_stackptr(PSR_ABT32_MODE, abtstack.pv_va + ABT_STACK_SIZE * PAGE_SIZE);
	set_stackptr(PSR_UND32_MODE, undstack.pv_va + UND_STACK_SIZE * PAGE_SIZE);

	/*
	 * Well we should set a data abort handler.
	 * Once things get going this will change as we will need a proper
	 * handler.
	 * Until then we will use a handler that just panics but tells us
	 * why.
	 * Initialisation of the vectors will just panic on a data abort.
	 * This just fills in a slightly better one.
	 */
#ifdef VERBOSE_INIT_ARM
	printf("vectors ");
#endif
	arm32_vector_init(ARM_VECTORS_HIGH, ARM_VEC_ALL);
	data_abort_handler_address = (u_int)data_abort_handler;
	prefetch_abort_handler_address = (u_int)prefetch_abort_handler;
	undefined_handler_address = (u_int)undefinedinstruction_bounce;

	/* Initialise the undefined instruction handlers */
#ifdef VERBOSE_INIT_ARM
	printf("undefined ");
#endif
	undefined_init();

	/* Load memory into UVM. */
#ifdef VERBOSE_INIT_ARM
	printf("page ");
#endif
	uvm_setpagesize();	/* initialize PAGE_SIZE-dependent variables */
	uvm_page_physload(atop(physical_freestart), atop(physical_freeend),
	    atop(physical_freestart), atop(physical_freeend),
	    VM_FREELIST_DEFAULT);

	/* Boot strap pmap telling it where the kernel page table is */
#ifdef VERBOSE_INIT_ARM
	printf("pmap ");
#endif
	pmap_bootstrap(KERNEL_VM_BASE, KERNEL_VM_BASE + KERNEL_VM_SIZE);

	/* Setup the IRQ system */
#ifdef VERBOSE_INIT_ARM
	printf("irq ");
#endif
	star_intr_init();

#ifdef VERBOSE_INIT_ARM
	printf("done.\n");
#endif

#ifdef BOOTHOWTO
	boothowto |= BOOTHOWTO;
#endif

#ifdef KGDB
	if (boothowto & RB_KDB) {
		kgdb_debug_init = 1;
		kgdb_connect(1);
	}
#endif

#ifdef DDB
	db_machine_init();
	if (boothowto & RB_KDB)
		Debugger();
#endif

	/* We have our own device_register() */
	evbarm_device_register = star_device_register;

	/* We return the new stack pointer address */
	return kernelstack.pv_va + USPACE_SVC_STACK_TOP;
}

void
consinit(void)
{
	static int consinit_called;

	if (consinit_called != 0)
		return;

	consinit_called = 1;

#if NCOM > 0
	if (comcnattach(&star_a4x_bs_tag, STRx100_UART0_REGISTER, comcnspeed,
	    STAR_UART_FREQ, COM_TYPE_STRx100, comcnmode))
		panic("can't init serial console");
#else
	panic("serial console not configured");
#endif
}
