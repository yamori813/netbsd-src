/*	$NetBSD$	*/

/*-
 * Copyright (c) 2010 SHIMIZU Ryo <ryo@nerv.org>
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

#include "locators.h"

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>

#include <uvm/uvm_extern.h>

#include <arm/mindspeed/m86xxx_reg.h>
#include <arm/mindspeed/m86xxx_var.h>

extern struct arm32_bus_dma_tag m83_bus_dma_tag;

struct axi_softc {
	device_t sc_dev;
	bus_space_tag_t sc_iot;
	bus_dma_tag_t sc_dmat;
};

static int axi_match(device_t, struct cfdata *, void *);
static void axi_attach(device_t, device_t, void *);
static int axi_search(device_t, struct cfdata *, const int *, void *);
static int axi_search(device_t, struct cfdata *, const int *, void *);
static void axi_attach_critical(struct axi_softc *);
static int axi_print(void *, const char *);

CFATTACH_DECL_NEW(axi, sizeof(struct axi_softc),
    axi_match, axi_attach, NULL, NULL);

/* ARGSUSED */
static int
axi_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux __unused)
{
	return 1;
}

/* ARGSUSED */
static void
axi_attach(device_t parent __unused, device_t self, void *aux __unused)
{
	struct axi_softc *sc;
	struct axi_attach_args aa;

	aprint_normal(": Advanced eXtensible Interface\n");
	aprint_naive("\n");

	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = &m83_bs_tag;
	sc->sc_dmat = &m83_bus_dma_tag;

	axi_attach_critical(sc);

	aa.aa_name = "axi";
	aa.aa_iot = sc->sc_iot;
	aa.aa_dmat = sc->sc_dmat;
	config_search(self, &aa,
	    CFARGS(.search = axi_search));
}

/* ARGSUSED */
static int
axi_search(device_t parent, struct cfdata *cf, const int *ldesc __unused,
    void *aux)
{
	struct axi_attach_args *aa;

	aa = aux;

	aa->aa_addr = cf->cf_loc[AXICF_ADDR];
	aa->aa_size = cf->cf_loc[AXICF_SIZE];
	aa->aa_intr = cf->cf_loc[AXICF_INTR];
	aa->aa_intrbase = cf->cf_loc[AXICF_IRQBASE];

	if (config_probe(parent, cf, aux))
		config_attach(parent, cf, aux, axi_print, CFARGS_NONE);

	return 0;
}

static int
axi_find(device_t parent, cfdata_t cf, const int *ldesc, void *aux)
{
	struct axi_attach_args * const aa = aux;
	struct axi_attach_args aa0;

	if (strcmp(aa->aa_name, "axi"))
		return 0;
	if (aa->aa_addr != AXICF_ADDR_DEFAULT
	    && aa->aa_addr != cf->cf_loc[AXICF_ADDR])
		return 0;
	if (aa->aa_size != AXICF_SIZE_DEFAULT
	    && aa->aa_size != cf->cf_loc[AXICF_SIZE])
		return 0;
	if (aa->aa_intr != AXICF_INTR_DEFAULT
	    && aa->aa_intr != cf->cf_loc[AXICF_INTR])
		return 0;
	if (aa->aa_intrbase != AXICF_IRQBASE_DEFAULT
	    && aa->aa_intrbase != cf->cf_loc[AXICF_IRQBASE])
		return 0;

	aa0 = *aa;
	aa0.aa_addr = cf->cf_loc[AXICF_ADDR];
	aa0.aa_size = cf->cf_loc[AXICF_SIZE];
	aa0.aa_intr = cf->cf_loc[AXICF_INTR];
	aa0.aa_intrbase = cf->cf_loc[AXICF_IRQBASE];

	return config_match(parent, cf, &aa0);
}

static const struct {
	const char *name;
	bus_addr_t addr;
	bool required;
} critical_devs[] = {
	{ .name = "clk", .addr = 0x904b0000, .required = true }
};

static void
axi_attach_critical(struct axi_softc *sc)
{
	struct axi_attach_args aa;
	cfdata_t cf;
	size_t i;

	for (i = 0; i < __arraycount(critical_devs); i++) {
		aa.aa_name = "axi";
		aa.aa_iot = sc->sc_iot;
		aa.aa_dmat = sc->sc_dmat;

		aa.aa_addr = critical_devs[i].addr;
		aa.aa_size = AXICF_SIZE_DEFAULT;
		aa.aa_intr = AXICF_INTR_DEFAULT;
		aa.aa_intrbase = AXICF_IRQBASE_DEFAULT;

		cf = config_search(sc->sc_dev, &aa,
		    CFARGS(.submatch = axi_find));
		if (cf == NULL && critical_devs[i].required)
			panic("axi_attach_critical: failed to find %s!",
			    critical_devs[i].name);

		aa.aa_addr = cf->cf_loc[AXICF_ADDR];
		aa.aa_size = cf->cf_loc[AXICF_SIZE];
		aa.aa_intr = cf->cf_loc[AXICF_INTR];
		aa.aa_intrbase = cf->cf_loc[AXICF_IRQBASE];
		config_attach(sc->sc_dev, cf, &aa, axi_print, CFARGS_NONE);
	}
}

/* ARGSUSED */
static int
axi_print(void *aux, const char *name __unused)
{
	struct axi_attach_args *aa = (struct axi_attach_args *)aux;

	if (aa->aa_addr != AXICF_ADDR_DEFAULT) {
		aprint_normal(" addr 0x%lx", aa->aa_addr);
		if (aa->aa_size > AXICF_SIZE_DEFAULT)
			aprint_normal("-0x%lx",
			    aa->aa_addr + aa->aa_size-1);
	}
	if (aa->aa_intr != AXICF_INTR_DEFAULT)
		aprint_normal(" intr %d", aa->aa_intr);
	if (aa->aa_intrbase != AXICF_IRQBASE_DEFAULT)
		aprint_normal(" intrbase %d", aa->aa_intrbase);

	return (UNCONF);
}
