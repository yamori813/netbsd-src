/*	$NetBSD$ */

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
#include <sys/systm.h>

#include <uvm/uvm_extern.h>

#include <machine/bus.h>

/* Prototypes for all the bus_space structure functions */
bs_protos(star);
bs_protos(generic);
bs_protos(generic_armv4);
bs_protos(bs_notimpl);

struct bus_space star_bs_tag = {
	.bs_cookie	= (void *)NULL,

	/* mapping/unmapping */
	.bs_map		= star_bs_map,
	.bs_unmap	= star_bs_unmap,
	.bs_subregion	= star_bs_subregion,

	/* allocation/deallocation */
	.bs_alloc	= star_bs_alloc,
	.bs_free	= star_bs_free,

	/* get kernel virtual address */
	.bs_vaddr	= star_bs_vaddr,

	/* mmap bus space for user */
	.bs_mmap	= bs_notimpl_bs_mmap,

	/* barrier */
	.bs_barrier	= star_bs_barrier,

	/* read (single) */
	.bs_r_1		= generic_bs_r_1,
	.bs_r_2		= generic_armv4_bs_r_2,
	.bs_r_4		= generic_bs_r_4,
	.bs_r_8		= bs_notimpl_bs_r_8,

	/* read multiple */
	.bs_rm_1	= generic_bs_rm_1,
	.bs_rm_2	= generic_armv4_bs_rm_2,
	.bs_rm_4	= generic_bs_rm_4,
	.bs_rm_8	= bs_notimpl_bs_rm_8,

	/* read region */
	.bs_rr_1	= generic_bs_rr_1,
	.bs_rr_2	= generic_armv4_bs_rr_2,
	.bs_rr_4	= generic_bs_rr_4,
	.bs_rr_8	= bs_notimpl_bs_rr_8,

	/* write (single) */
	.bs_w_1		= generic_bs_w_1,
	.bs_w_2		= generic_armv4_bs_w_2,
	.bs_w_4		= generic_bs_w_4,
	.bs_w_8		= bs_notimpl_bs_w_8,

	/* write multiple */
	.bs_wm_1	= generic_bs_wm_1,
	.bs_wm_2	= generic_armv4_bs_wm_2,
	.bs_wm_4	= generic_bs_wm_4,
	.bs_wm_8	= bs_notimpl_bs_wm_8,

	/* write region */
	.bs_wr_1	= generic_bs_wr_1,
	.bs_wr_2	= generic_armv4_bs_wr_2,
	.bs_wr_4	= generic_bs_wr_4,
	.bs_wr_8	= bs_notimpl_bs_wr_8,

	/* set multiple */
	.bs_sm_1	= bs_notimpl_bs_sm_1,
	.bs_sm_2	= bs_notimpl_bs_sm_2,
	.bs_sm_4	= bs_notimpl_bs_sm_4,
	.bs_sm_8	= bs_notimpl_bs_sm_8,

	/* set region */
	.bs_sr_1	= generic_bs_sr_1,
	.bs_sr_2	= generic_armv4_bs_sr_2,
	.bs_sr_4	= bs_notimpl_bs_sr_4,
	.bs_sr_8	= bs_notimpl_bs_sr_8,

	/* copy */
	.bs_c_1		= bs_notimpl_bs_c_1,
	.bs_c_2		= generic_armv4_bs_c_2,
	.bs_c_4		= bs_notimpl_bs_c_4,
	.bs_c_8		= bs_notimpl_bs_c_8,
};

int
star_bs_map(void *cookie, bus_addr_t address, bus_size_t size,
            int flags, bus_space_handle_t *handlep)
{
	bus_addr_t startpa, endpa, pa;
	vaddr_t va;
	paddr_t offset;
	pt_entry_t *pte;
	const struct pmap_devmap *pd;

	if ((pd = pmap_devmap_find_pa(address, size)) != NULL) {
		/* Device was statically mapped. */
		*handlep = pd->pd_va + (address - pd->pd_pa);
		return 0;
	}

	startpa = trunc_page(address);
	endpa = round_page(address + size);

	/* alloc vm */
	va = uvm_km_alloc(kernel_map, endpa - startpa, 0,
	    UVM_KMF_VAONLY | UVM_KMF_NOWAIT);
	if (va == 0) {
		printf("uvm_km_alloc: failure\n");
		return ENOMEM;
	}

	offset = address - startpa;

	/* store the bus space handle */
	*handlep = va + offset;

	/* map the pages */
	for (pa = startpa; pa < endpa; pa += PAGE_SIZE, va += PAGE_SIZE) {
		pmap_kenter_pa(va, pa, VM_PROT_READ | VM_PROT_WRITE, 0);
		if ((flags & BUS_SPACE_MAP_CACHEABLE) == 0) {
			pte = vtopte(va);
			*pte &= ~L2_S_CACHE_MASK;
			PTE_SYNC(pte);
			/*
			 * XXX: pmap_kenter_pa() also does PTE_SYNC().
			 *      a bit of waste.
			 */
		}
	}
	pmap_update(pmap_kernel());

	return 0;
}

void
star_bs_unmap(void *cookie, bus_space_handle_t handle, bus_size_t size)
{
	vaddr_t va;
	vsize_t sz;

	if (pmap_devmap_find_va(handle, size) != NULL) {
		/* Device was statically mapped; nothing to do. */
		return;
	}

	va = trunc_page(handle);
	sz = round_page(handle + size) - va;

	pmap_kremove(va, sz);
	pmap_update(pmap_kernel());
	uvm_km_free(kernel_map, va, sz, UVM_KMF_VAONLY);
}

int
star_bs_subregion(void *cookie, bus_space_handle_t handle,
                  bus_size_t offset, bus_size_t size,
                  bus_space_handle_t *handlep)
{
	*handlep = handle + offset;
	return 0;
}

void
star_bs_barrier(void *cookie, bus_space_handle_t handle, bus_size_t offset,
                bus_size_t length, int flags)
{
	/* Nothing to do. */
}

void *
star_bs_vaddr(void *cookie, bus_space_handle_t handle)
{
	return (void *)handle;
}

int
star_bs_alloc(void *cookie, bus_addr_t reg_start, bus_addr_t reg_end,
              bus_size_t size, bus_size_t alignment, bus_size_t boundary,
              int flags, bus_addr_t *addrp, bus_space_handle_t *handlep)
{
	panic("%s: not implemented\n", __func__);
}

void
star_bs_free(void *cookie, bus_space_handle_t handle, bus_size_t size)
{
	panic("%s: not implemented\n", __func__);
}
