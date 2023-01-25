/*	$NetBSD$ */
/*
 * Copyright (c) 2023 Hiroki Mori
 * Copyright (c) 2007, 2008, 2010 KIYOHARA Takashi
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
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_arm_debug.h"
#include "opt_console.h"
#include "opt_evbarm_boardtype.h"
#include "opt_ddb.h"
#include "opt_pci.h"
#include "com.h"

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/reboot.h>
#include <sys/systm.h>
#include <sys/termios.h>

#include <prop/proplib.h>

#include <dev/cons.h>
#include <dev/md.h>

#include <machine/autoconf.h>
#include <machine/bootconfig.h>

#include <uvm/uvm_extern.h>

#include <arm/db_machdep.h>
#include <arm/undefined.h>
#include <arm/arm32/machdep.h>

#include <arm/ralink/rt1310_reg.h>
#include <arm/ralink/rt1310_var.h>

#include <ddb/db_extern.h>
#include <ddb/db_sym.h>

#include "ksyms.h"

/*
 * The range 0xc2000000 - 0xdfffffff is available for kernel VM space
 * Core-logic registers and I/O mappings occupy 0xfe000000 - 0xffffffff
 */
#if (KERNEL_BASE & 0xf0000000) == 0x80000000
#define KERNEL_VM_BASE		(KERNEL_BASE + 0x42000000)
#else
#define KERNEL_VM_BASE		(KERNEL_BASE + 0x02000000)
#endif
#define KERNEL_VM_SIZE		0x1e000000

#define MEMSTART	0x40000000

#define KERNEL_BASE_PHYS        0x40000000

BootConfig bootconfig;		/* Boot config storage */
static char bootargs[MAX_BOOT_STRING];
char *boot_args = NULL;

extern int KERNEL_BASE_phys[];
extern char _end[];

#include "com.h"
#if NCOM > 0
#include <dev/ic/comreg.h>
#include <dev/ic/comvar.h>
#endif

#ifndef CONSPEED
#define CONSPEED	B38400	/* It's a setting of the default of u-boot */
#endif

#ifndef CONMODE
#define CONMODE ((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */

int comcnspeed = CONSPEED;
int comcnmode = CONMODE;
#endif

#include "opt_kgdb.h"
#ifdef KGDB
#include <sys/kgdb.h>
#endif

static inline pd_entry_t *
read_ttb(void)
{

	return (pd_entry_t *)(armreg_ttbr_read() & ~((1<<14)-1));
}

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
#define _A(a)	((a) & ~L1_S_OFFSET)
#define _S(s)	(((s) + L1_S_SIZE - 1) & ~(L1_S_SIZE-1))

#ifndef CONMODE
#define CONMODE ((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */
#endif
#ifndef CONSPEED
#define CONSPEED        38400
#endif

int consmode = CONMODE;
int consrate = CONSPEED;


#define RT1310_INTERREGS_VBASE 0xfe000000

static struct pmap_devmap rt1310_devmap[] = {
	{
		RT1310_INTERREGS_VBASE,
		_A(0x1e800000),
		_S(0x200000),
		VM_PROT_READ|VM_PROT_WRITE,
		PTE_NOCACHE,
	},

	{ 0, 0, 0, 0, 0 }
};

extern uint32_t *u_boot_args[];
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
 *   Relocating the kernel to the bottom of physical memory
 */
vaddr_t
initarm(void *arg)
{
//	int cs, cs_end, memtag = 0, iotag = 0;

//	mvsoc_bootstrap(MARVELL_INTERREGS_VBASE);

	/*
	 * Heads up ... Setup the CPU / MMU / TLB functions
	 */
	if (set_cpufuncs())
		panic("cpu not recognized!");

	kern_vtopdiff = KERNEL_BASE - KERNEL_BASE_PHYS;

	/* map some peripheral registers */
	pmap_devmap_bootstrap((vaddr_t)read_ttb(), rt1310_devmap);

	cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2)) | DOMAIN_CLIENT);

/*
	uint8_t* uart_base_addr=(uint8_t*)0xfe040000;
	*(uart_base_addr) = '*';
*/

	consinit();

	/* Talk to the user */
#define BDSTR(s)	_BDSTR(s)
#define _BDSTR(s)	#s
	printf("\nNetBSD/evbarm (" BDSTR(EVBARM_BOARDTYPE) ") booting ...\n");

	/* copy command line U-Boot gave us, if args is valid. */
	if (u_boot_args[3] != 0)	/* XXXXX: need more check?? */
		strncpy(bootargs, (char *)u_boot_args[3], sizeof(bootargs));

#ifdef VERBOSE_INIT_ARM
	printf("initarm: Configuring system ...\n");
#endif

        /* Fake bootconfig structure for the benefit of pmap.c */
        /* XXX must make the memory description h/w independent */
	bootconfig.dramblocks = 1;
	bootconfig.dram[0].address = MEMSTART;
	bootconfig.dram[0].pages = MEMSIZE / PAGE_SIZE;

	arm32_bootmem_init(bootconfig.dram[0].address, MEMSIZE,
	    (uintptr_t) KERNEL_BASE_phys);
	arm32_kernel_vm_init(KERNEL_VM_BASE, ARM_VECTORS_HIGH, 0,
	    rt1310_devmap, true);

	/* we've a specific device_register routine */
//	evbarm_device_register = marvell_device_register;

	/* parse bootargs from U-Boot */
//	boot_args = bootargs;
//	parse_mi_bootargs(boot_args);

	return initarm_common(KERNEL_VM_BASE, KERNEL_VM_SIZE, NULL, 0);
}

int
rtcomcnattach(bus_space_tag_t iot, bus_addr_t iobase, int rate,
    int frequency, int type, tcflag_t cflag);
int
rtcomcnattach(bus_space_tag_t iot, bus_addr_t iobase, int rate,
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
	static int consinit_called = 0;

	if (consinit_called)
		return;

	consinit_called = 1;

	/* initialize the console functions */
	if (rtcomcnattach(&rt1310_bs_tag, 0x1e840000, consrate,
		RT_BASE_BAUD, COM_TYPE_16550_NOERS, consmode))
			panic("Serial console can not be initialized.");
}

void
cpu_reboot(int howto, char *bootstr)
{
	bus_space_tag_t bst;
	bus_space_handle_t bsh;

	bst = &rt1310_bs_tag;

	/* Enable WDT */
	/* Instant assert of RESETOUT_N with pulse length 1ms */
	bus_space_map(bst, 0x1e8c0000, 0x20000, 0, &bsh);
	bus_space_write_4(bst, bsh, 0, 13000);
	bus_space_write_4(bst, bsh, 8, (1<<3) | (1<<4) | 7);
	bus_space_unmap(bst, bsh, 0x20000);

	for (;;)
		continue;
}
