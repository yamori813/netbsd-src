/*	$NetBSD$	*/
/*-
 * Copyright (c) 2011 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Cliff Neighbors.
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

/*
 * NOR CFI driver support for arm
 */

#include "opt_flash.h"
#include "locators.h"

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/cdefs.h>
#include <sys/device.h>
#include <sys/endian.h>

#include <sys/bus.h>

#include <arm/mainbus/mainbus.h>

#include <dev/nor/nor.h>
#include <dev/nor/cfi.h>

#include <arm/mindspeed/m83xxx_var.h>

static int  mainbus_cfi_match(device_t, cfdata_t, void *);
static void mainbus_cfi_attach(device_t, device_t, void *);
static int  mainbus_cfi_detach(device_t, int);

struct mainbus_cfi_softc {
	device_t		sc_dev;
	device_t		sc_nordev;
	struct cfi		sc_cfi;
	bus_addr_t		sc_addr;
	bus_size_t		sc_size;
	struct nor_interface	sc_nor_if;
};

CFATTACH_DECL_NEW(mainbus_cfi, sizeof(struct mainbus_cfi_softc), 
    mainbus_cfi_match, mainbus_cfi_attach, mainbus_cfi_detach, NULL);

/*
 * mainbus_cfi_addr - return bus address for the CFI NOR flash
 */
static inline bus_addr_t
mainbus_cfi_addr(struct mainbus_attach_args * const mb)
{
	return 0x20000000;
}

static int
mainbus_cfi_match(device_t parent, cfdata_t match, void *aux)
{
	struct mainbus_attach_args * const mb = aux;
	bus_size_t tmpsize = CFI_QRY_MIN_MAP_SIZE;
	bus_addr_t addr;
	struct cfi cfi;
	int rv;

//	KASSERT(mb->mb_iobase != NULL);

	addr = mainbus_cfi_addr(mb);
#ifdef NOTYET
	if (addr == MAINBUSCF_ADDR_DEFAULT) {
		aprint_error("%s: no base address\n", __func__);
		return 0;
	}
#endif

	cfi.cfi_bst = &m83_bs_tag;
	int error = bus_space_map(cfi.cfi_bst, addr, tmpsize, 0, &cfi.cfi_bsh);
	if (error != 0) {
		aprint_error("%s: cannot map %#" PRIxBUSSIZE"  at offset %#"
		    PRIxBUSADDR ", error %d\n",	__func__, tmpsize, addr, error);
		return false;
	}

	if (! cfi_probe(&cfi)) {
		aprint_debug("%s: probe addr %#" PRIxBUSADDR
		    ", CFI not found\n", __func__, addr);
		rv = 0;
	} else {
		rv = 1;
	}

	bus_space_unmap(cfi.cfi_bst, cfi.cfi_bsh, tmpsize);

	return rv;
}

static void
mainbus_cfi_attach(device_t parent, device_t self, void *aux)
{
	struct mainbus_cfi_softc *sc = device_private(self);
	struct mainbus_attach_args * const mb = aux;
	struct cfi_query_data * const qryp = &sc->sc_cfi.cfi_qry_data;
	const bus_size_t tmpsize = CFI_QRY_MIN_MAP_SIZE;
	bool found;
	int error;

	aprint_normal("\n");

	sc->sc_dev = self;
//	sc->sc_cfi.cfi_bst = mb->mb_iot;
	sc->sc_cfi.cfi_bst = &m83_bs_tag;
	sc->sc_addr = mainbus_cfi_addr(mb);

	/* map enough to identify, remap later when size is known */
	error = bus_space_map(sc->sc_cfi.cfi_bst, sc->sc_addr, tmpsize,
		0, &sc->sc_cfi.cfi_bsh);
	if (error != 0) {
		aprint_error_dev(self, "could not map error %d\n", error);
		return;
	}

	found = cfi_identify(&sc->sc_cfi);

	bus_space_unmap(sc->sc_cfi.cfi_bst, sc->sc_cfi.cfi_bsh, tmpsize);

	if (! found) {
		/* should not happen, we already probed OK in match */
		aprint_error_dev(self, "could not map error %d\n", error);
		return;
	}

	sc->sc_size = 1 << qryp->device_size;

	sc->sc_nor_if = nor_interface_cfi;
	sc->sc_nor_if.private = &sc->sc_cfi;
	sc->sc_nor_if.access_width = (1 << sc->sc_cfi.cfi_portwidth);

	cfi_print(self, &sc->sc_cfi);

	error = bus_space_map(sc->sc_cfi.cfi_bst, sc->sc_addr, sc->sc_size,
		0, &sc->sc_cfi.cfi_bsh);
	if (error != 0) {
		aprint_error_dev(self, "could not map error %d\n", error);
		return;
	}

	if (! pmf_device_register1(self, NULL, NULL, NULL))
		aprint_error_dev(self, "couldn't establish power handler\n");

	sc->sc_nordev = nor_attach_mi(&sc->sc_nor_if, self);

}

static int
mainbus_cfi_detach(device_t self, int flags)
{
	struct mainbus_cfi_softc *sc = device_private(self);
	int rv = 0;

	pmf_device_deregister(self);

	if (sc->sc_nordev != NULL)
		rv = config_detach(sc->sc_nordev, flags);

	bus_space_unmap(sc->sc_cfi.cfi_bst, sc->sc_cfi.cfi_bsh, sc->sc_size);

	return rv;
}
