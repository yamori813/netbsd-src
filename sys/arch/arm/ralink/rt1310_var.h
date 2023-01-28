/*	$NetBSD$	*/

/*-
 * Copyright (c) 2023 Hiroki Mori
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/bus.h>

#ifndef _ARM_RALINK_RT1310_VAR_H
#define _ARM_RALINK_RT1310_VAR_H

extern struct bus_space rt1310_bs_tag;

struct apb_attach_args {
	const char	*apba_name;
	bus_space_tag_t	apba_memt;
	bus_dma_tag_t	apba_dmat;
	bus_addr_t	apba_addr;
	bus_size_t	apba_size;
	int		apba_intr;
	int		apba_irqbase;
};

struct ahb_attach_args {
	const char	*ahba_name;
	bus_space_tag_t	ahba_memt;
	bus_dma_tag_t	ahba_dmat;
	bus_addr_t	ahba_addr;
	bus_size_t	ahba_size;
	int		ahba_intr;
	int		ahba_irqbase;
};

#endif	/* _ARM_RALINK_RT1310_VAR_H */
