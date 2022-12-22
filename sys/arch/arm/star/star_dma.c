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

#define _ARM32_BUS_DMA_PRIVATE
#include <sys/param.h>
#include <sys/bus.h>

struct arm32_bus_dma_tag star_bus_dma_tag = {
	._ranges		= NULL,
	._nranges		= 0,

	._cookie		= NULL,

	._dmamap_create		= _bus_dmamap_create,
	._dmamap_destroy	= _bus_dmamap_destroy,
	._dmamap_load		= _bus_dmamap_load,
	._dmamap_load_mbuf	= _bus_dmamap_load_mbuf,
	._dmamap_load_uio	= _bus_dmamap_load_uio,
	._dmamap_load_raw	= _bus_dmamap_load_raw,
	._dmamap_unload		= _bus_dmamap_unload,
	._dmamap_sync_pre	= _bus_dmamap_sync,
	._dmamap_sync_post	= NULL,

	._dmamem_alloc		= _bus_dmamem_alloc,
	._dmamem_free		= _bus_dmamem_free,
	._dmamem_map		= _bus_dmamem_map,
	._dmamem_unmap		= _bus_dmamem_unmap,
	._dmamem_mmap		= _bus_dmamem_mmap,

	._dmatag_subregion	= _bus_dmatag_subregion,
	._dmatag_destroy	= _bus_dmatag_destroy
};
