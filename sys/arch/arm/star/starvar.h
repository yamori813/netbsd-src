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

#ifndef _STARVAR_H
#define _STARVAR_H

#include <sys/bus.h>

struct star_attach_args {
	const char *sa_cpuname;
	int sa_cputype;
	bus_space_tag_t sa_iot;
	bus_addr_t sa_addr;
	bus_size_t sa_size;
	bus_dma_tag_t sa_dmat;
	int sa_irq;
};

extern struct bus_space star_bs_tag;
extern struct bus_space star_a4x_bs_tag;
extern struct bus_space star_pci_io_tag;
extern struct bus_space star_pci_mem_tag;
extern struct bus_space star_pci_dma_tag;
extern struct arm32_bus_dma_tag star_bus_dma_tag;

struct star_devtable {
	bus_addr_t paddr;
	bus_size_t size;

	int has_str8100:1;
	int has_str9100:1;
	int pad:30;
};

extern struct star_devtable star_soc_devtable[];
extern int star_cpu_type;

#define CPU_RUNTIME_STR8100()	(star_cpu_type == 8)
#define CPU_RUNTIME_STR9100()	(star_cpu_type != 8)
#if defined(STAR_EQUULEUS) && defined(STAR_ORION)
#define CPU_IS_STR8100()	CPU_RUNTIME_STR8100()
#define CPU_IS_STR9100()	CPU_RUNTIME_STR9100()
#elif defined(STAR_EQUULEUS)
#define CPU_IS_STR8100()	1
#define CPU_IS_STR9100()	0
#elif defined(STAR_ORION)
#define CPU_IS_STR8100()	0
#define CPU_IS_STR9100()	1
#else
#error nether STAR_EQUULEUS nor STAR_ORION is defined
#endif

struct pmap_devmap;	/* in uvm/uvm_extern.h */

int star_build_devmap(struct pmap_devmap *, int);
void star_initialize(void);
void star_reset(void);
int star_get_memsize(int);

#endif /* _STARVAR_H */
