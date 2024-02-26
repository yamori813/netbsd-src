/*-
 * Copyright (c) 2024 Hiroki Mori
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

#include "locators.h"

#include <sys/cdefs.h>

#define	DMU_PRIVATE

__KERNEL_RCSID(1, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/intr.h>
#include <sys/systm.h>
#include <sys/sysctl.h>

#include <arm/broadcom/bcm53xx_reg.h>
#include <arm/broadcom/bcm53xx_var.h>

struct bcmdmu_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bst;
	bus_space_handle_t sc_bsh;
	bus_dma_tag_t sc_dmat;
	uint32_t temp;
};

static int bcmdmu_ccb_match(device_t, cfdata_t, void *);
static void bcmdmu_ccb_attach(device_t, device_t, void *);
static void bcmdmu_sysctl_init(struct bcmdmu_softc *);
static int sysctl_cputemp(SYSCTLFN_ARGS);

CFATTACH_DECL_NEW(bcmdmu_ccb, sizeof(struct bcmdmu_softc),
	bcmdmu_ccb_match, bcmdmu_ccb_attach, NULL, NULL);

static inline uint32_t
bcmdmu_read_4(struct bcmdmu_softc *sc, bus_size_t o)
{
	return bus_space_read_4(sc->sc_bst, sc->sc_bsh, o);
}
 
static inline void
bcmdmu_write_4(struct bcmdmu_softc *sc, bus_size_t o, uint32_t v)
{
	bus_space_write_4(sc->sc_bst, sc->sc_bsh, o, v);
}

static int
bcmdmu_ccb_match(device_t parent, cfdata_t cf, void *aux)
{
	struct bcmccb_attach_args * const ccbaa = aux;
	const struct bcm_locators * const loc = &ccbaa->ccbaa_loc;

	if (strcmp(cf->cf_name, loc->loc_name))
		return 0;

	KASSERT(cf->cf_loc[BCMCCBCF_PORT] == BCMCCBCF_PORT_DEFAULT);

	return 1;
}

static void
bcmdmu_ccb_attach(device_t parent, device_t self, void *aux)
{
	struct bcmdmu_softc * const sc = device_private(self);
	struct bcmccb_attach_args * const ccbaa = aux;
	const struct bcm_locators * const loc = &ccbaa->ccbaa_loc;

	sc->sc_dev = self;

	sc->sc_bst = ccbaa->ccbaa_ccb_bst;
	sc->sc_dmat = ccbaa->ccbaa_dmat;
	bus_space_subregion(sc->sc_bst, ccbaa->ccbaa_ccb_bsh,
	    loc->loc_offset, loc->loc_size, &sc->sc_bsh);

	aprint_naive("\n");
	aprint_normal("\n");

	bcmdmu_sysctl_init(sc);
}

static void
bcmdmu_sysctl_init(struct bcmdmu_softc *sc)
{
	const struct sysctlnode *node, *cpunode;
	int error;

	error = sysctl_createv(NULL, 0, NULL, &node,
	    CTLFLAG_PERMANENT, CTLTYPE_NODE, "machdep", NULL,
	    NULL, 0, NULL, 0, CTL_MACHDEP, CTL_EOL);
	if (error)
		printf("couldn't create `machdep' node\n");

	error = sysctl_createv(NULL, 0, &node, &cpunode,
	    0, CTLTYPE_NODE, "cpu", NULL,
	    NULL, 0, NULL, 0, CTL_CREATE, CTL_EOL);
	if (error)
		printf("couldn't create `cpu' node\n");

 	error = sysctl_createv(NULL, 0, &cpunode, &node,
	    CTLFLAG_READONLY, CTLTYPE_INT, "temperature", NULL,
	    sysctl_cputemp, 0, (void *)sc, 0,
	    CTL_CREATE, CTL_EOL);
	if (error)
		printf("couldn't create `temperature' node\n");

}

static int
sysctl_cputemp(SYSCTLFN_ARGS)
{
	struct sysctlnode node = *rnode;
	struct bcmdmu_softc *sc = node.sysctl_data;
	uint32_t pvtmonctl0, pvtmonstat;

	pvtmonctl0 = bcmdmu_read_4(sc, DMU_PVTMON_CONTROL0);
	if (pvtmonctl0 & 0xf) {
		pvtmonctl0 &= ~0xf;
		bcmdmu_write_4(sc, DMU_PVTMON_CONTROL0, pvtmonctl0);
	}
	pvtmonstat = bcmdmu_read_4(sc, DMU_PVTMON_CONTROL0 + 0x8);
	sc->temp = 418 - ((5556 * pvtmonstat) / 10000);

	node.sysctl_data = &sc->temp;

	return sysctl_lookup(SYSCTLFN_CALL(&node));
}
