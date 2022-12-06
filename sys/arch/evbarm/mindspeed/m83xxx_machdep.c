/* $NetBSD$ */

/*
 * Copyright (c) 2022  Hiroki Mori.  All rights reserved.
 *
 * Machine dependent functions for kernel setup for Mindspeed Comcerto
 * 1000(M83XXX) boards using U-Boot firmware.
 */
/*
 * Copyright (c) 2002, 2003, 2005  Genetec Corporation.  All rights reserved.
 * Written by Hiroyuki Bessho for Genetec Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of Genetec Corporation may not be used to endorse or
 *    promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GENETEC CORPORATION ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL GENETEC CORPORATION
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Copyright (c) 2001 Wasabi Systems, Inc.
 * All rights reserved.
 *
 * Written by Jason R. Thorpe for Wasabi Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed for the NetBSD Project by
 *	Wasabi Systems, Inc.
 * 4. The name of Wasabi Systems, Inc. may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
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
 * Copyright (c) 1997,1998 Mark Brinicombe.
 * Copyright (c) 1997,1998 Causality Limited.
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Mark Brinicombe
 *	for the NetBSD Project.
 * 4. The name of the company nor the name of the author may be used to
 *    endorse or promote products derived from this software without specific
 *    prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include "opt_arm_debug.h"
#include "opt_console.h"
#include "opt_ddb.h"
#include "opt_kgdb.h"
#include "opt_md.h"
#include "opt_com.h"

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
#include <sys/cpu.h>

#include <uvm/uvm_extern.h>

#include <sys/conf.h>
#include <dev/cons.h>
#include <dev/md.h>

#include <machine/db_machdep.h>
#include <ddb/db_sym.h>
#include <ddb/db_extern.h>
#ifdef KGDB
#include <sys/kgdb.h>
#endif

#include <machine/bootconfig.h>
#include <arm/locore.h>
#include <arm/undefined.h>

#include <arm/arm32/pte.h>
#include <arm/arm32/machdep.h>

#include <arm/mindspeed/m83xxx_reg.h>
#include <evbarm/mindspeed/m83xxx_reg.h>

/* Kernel text starts 1MB in from the bottom of the kernel address space. */
#define	KERNEL_TEXT_BASE	(KERNEL_BASE + 0x00100000)
#define	KERNEL_VM_BASE		(KERNEL_BASE + 0x01000000)

#define KERNEL_BASE_PHYS	0x80000000

/*
 * The range 0x81000000 - 0x8cffffff is available for kernel VM space
 * Core-logic registers and I/O mappings occupy 0xfd000000 - 0xffffffff
 */
#define KERNEL_VM_SIZE		0x0C000000

/* filled in before cleaning bss. keep in .data */
u_int uboot_args[4] __attribute__((__section__(".data")));

BootConfig bootconfig;		/* Boot config storage */
char *boot_args = NULL;
char *boot_file = NULL;

u_int free_pages;

extern struct bus_space m83_bs_tag;

/*int debug_flags;*/
#ifndef PMAP_STATIC_L1S
int max_processes = 64;			/* Default number */
#endif	/* !PMAP_STATIC_L1S */

/* Prototypes */

void m83xxx_platform_early_putchar(char c);

void	kgdb_port_init(void);
void	change_clock(uint32_t v);

#include "com.h"
#if NCOM > 0
#include <dev/ic/comreg.h>
#include <dev/ic/comvar.h>
#endif

/*
 * Static device mappings. These peripheral registers are mapped at
 * fixed virtual addresses very early in m83xxx_start() so that we
 * can use them while booting the kernel, and stay at the same address
 * throughout whole kernel's life time.
 *
 * We use this table twice; once with bootstrap page table, and once
 * with kernel's page table which we build up in initarm().
 */

#define _A(a)   ((a) & ~L1_S_OFFSET)
#define _S(s)   (((s) + L1_S_SIZE - 1) & ~(L1_S_SIZE-1))

#ifndef CONMODE
#define CONMODE ((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */
#endif
#ifndef CONSPEED
#define CONSPEED        115200
#endif

int consmode = CONMODE;
int consrate = CONSPEED;

static const struct pmap_devmap m83xxx_devmap[] = {
    {
	M83XXX_UART1_VBASE,
	_A(APB_UART0_BASE),
	_S(L1_S_SIZE),
	VM_PROT_READ|VM_PROT_WRITE,
	PTE_NOCACHE,
    },
	{0, 0, 0, 0, 0 }
};

#ifndef MEMSTART
#define MEMSTART	0x80000000
#endif
#ifndef MEMSIZE
#define MEMSIZE		0x8000000		/* 128MB */
#endif

static void
m83xxx_system_reset(void)
{
	bus_space_handle_t bsh;
	bus_space_map(&m83_bs_tag, APB_GPIO_BASE, 0x20000, 0, &bsh);
	bus_space_write_4(&m83_bs_tag, bsh, GPIO_OUTPUT_REG, 0);
	/* not reach here */
	bus_space_unmap(&m83_bs_tag, bsh, 0x20000);
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
 *   Relocating the kernel to the bottom of physical memory
 */
vaddr_t
initarm(void *arg)
{
	extern char KERNEL_BASE_phys[];

//	boothowto = (AB_VERBOSE|AB_DEBUG);
	boothowto = AB_VERBOSE;

	/*
	 * Heads up ... Setup the CPU / MMU / TLB functions
	 */
	if (set_cpufuncs())
		panic("cpu not recognized!");

	kern_vtopdiff = KERNEL_BASE - KERNEL_BASE_PHYS;

	/* Register devmap for devices we mapped in start */
	pmap_devmap_bootstrap((vaddr_t)armreg_ttbr_read() & -L1_TABLE_SIZE, m83xxx_devmap);

	cpu_domains((DOMAIN_CLIENT << (PMAP_DOMAIN_KERNEL*2)) | DOMAIN_CLIENT);

#if 0
	/* Calibrate the delay loop. */
#endif

	cpu_reset_address = m83xxx_system_reset;

	consinit();

#ifdef KGDB
	kgdb_port_init();
#endif
	/* Talk to the user */
	printf("\nNetBSD/evbarm (m83xxx) booting ...\n");

#if 0
	/*
	 * Examine the boot args string for options we need to know about
	 * now.
	 */
	process_kernel_args((char *)nwbootinfo.bt_args);
#endif

	printf("initarm: Configuring system ...\n");

	/* Fake bootconfig structure for the benefit of pmap.c */
	/* XXX must make the memory description h/w independent */
	bootconfig.dramblocks = 1;
	bootconfig.dram[0].address = MEMSTART;
	bootconfig.dram[0].pages = MEMSIZE / PAGE_SIZE;

	arm32_bootmem_init(bootconfig.dram[0].address, MEMSIZE,
	     (uintptr_t) KERNEL_BASE_phys);
	arm32_kernel_vm_init(KERNEL_VM_BASE, ARM_VECTORS_HIGH, 0, m83xxx_devmap, true);
	int rt =  initarm_common(KERNEL_VM_BASE, KERNEL_VM_SIZE, NULL, 0);
	return rt;
}

#if 0
void
process_kernel_args(char *args)
{

	boothowto = 0;

	/* Make a local copy of the bootargs */
	strncpy(bootargs, args, MAX_BOOT_STRING);

	args = bootargs;
	boot_file = bootargs;

	/* Skip the kernel image filename */
	while (*args != ' ' && *args != 0)
		++args;

	if (*args != 0)
		*args++ = 0;

	while (*args == ' ')
		++args;

	boot_args = args;

	printf("bootfile: %s\n", boot_file);
	printf("bootargs: %s\n", boot_args);

	parse_mi_bootargs(boot_args);
}
#endif

#ifdef KGDB
#ifndef KGDB_DEVNAME
#define KGDB_DEVNAME "ffuart"
#endif
const char kgdb_devname[] = KGDB_DEVNAME;

#if (NCOM > 0)
#ifndef KGDB_DEVMODE
#define KGDB_DEVMODE ((TTYDEF_CFLAG & ~(CSIZE | CSTOPB | PARENB)) | CS8) /* 8N1 */
#endif
int comkgdbmode = KGDB_DEVMODE;
#endif /* NCOM */

#endif /* KGDB */

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
	static int consinit_called = 0;

	if (consinit_called)
		return;

	consinit_called = 1;

	/* initialize the console functions */
	if (m83comcnattach(&m83_bs_tag, APB_UART0_BASE, consrate,
		COMCERTO_APB_FREQ, COM_TYPE_16550_NOERS, consmode))
			panic("Serial console can not be initialized.");
}

#ifdef KGDB
void
kgdb_port_init(void)
{
#if (NCOM > 0) && defined(COM_PXA2X0)
	paddr_t paddr = 0;
	uint32_t ckenreg = ioreg_read(VIPER_CLKMAN_VBASE+CLKMAN_CKEN);

	if (0 == strcmp(kgdb_devname, "ffuart")) {
		paddr = PXA2X0_FFUART_BASE;
		ckenreg |= CKEN_FFUART;
	}
	else if (0 == strcmp(kgdb_devname, "btuart")) {
		paddr = PXA2X0_BTUART_BASE;
		ckenreg |= CKEN_BTUART;
	}

	if (paddr &&
	    0 == com_kgdb_attach(&imx31_a4x_bs_tag, paddr,
		kgdb_rate, PXA2X0_COM_FREQ, COM_TYPE_PXA2x0, comkgdbmode)) {

		ioreg_write(VIPER_CLKMAN_VBASE+CLKMAN_CKEN, ckenreg);
	}
#endif
}
#endif

void __noasan
m83xxx_platform_early_putchar(char c)
{
#ifdef CONSADDR
//#define CONSADDR_VA     (CONSADDR - EXYNOS_CORE_PBASE + EXYNOS_CORE_VBASE)

	volatile uint32_t *uartaddr;
	if(cpu_earlydevice_va_p()) {
		uartaddr = (volatile uint32_t *)M83XXX_UART1_VBASE;

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
