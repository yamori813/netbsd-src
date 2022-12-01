/*	$Id$	*/

/* derived from:	*/
/*	$NetBSD$ */

/*
 * Copyright (c) 2002, 2005  Genetec Corporation.  All rights reserved.
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
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed for the NetBSD Project by
 *	Genetec Corporation.
 * 4. The name of Genetec Corporation may not be used to endorse or 
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
 *
 *
 * Autoconfiguration support for the Intel PXA2[15]0 application
 * processor. This code is derived from arm/sa11x0/sa11x0.c
 */

/*-
 * Copyright (c) 2001, The NetBSD Foundation, Inc.  All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by IWAMOTO Toshihiro and Ichiro FUKUHARA.
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
/*-
 * Copyright (c) 1999
 *         Shin Takemura and PocketBSD Project. All rights reserved.
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
 *	This product includes software developed by the PocketBSD project
 *	and its contributors.
 * 4. Neither the name of the project nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$Id$");

#include "locators.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/kernel.h>
#include <sys/reboot.h>

#include <machine/cpu.h>
#include <sys/bus.h>

#include <arm/cpufunc.h>
#include <arm/mainbus/mainbus.h>

#include <machine/intr.h>

#include <arm/mindspeed/m83xxx_var.h>

struct m83apb_softc {
	device_t sc_dev;
	bus_space_tag_t sc_bust;
};

/* prototypes */
static int	m83apb_match(device_t , cfdata_t, void *);
static void	m83apb_attach(device_t , device_t , void *);
static int 	m83apb_search(device_t , cfdata_t, const int *, void *);
static void	m83apb_attach_critical(struct m83apb_softc *sc);
static int	m83apb_print(void *, const char *);

/* attach structures */
CFATTACH_DECL_NEW(apb, sizeof(struct m83apb_softc),
    m83apb_match, m83apb_attach, NULL, NULL);

static int
m83apb_match(device_t parent, cfdata_t match, void *aux)
{
	return 1;
}

static void
m83apb_attach(device_t parent, device_t self, void *aux)
{
	struct m83apb_softc * const sc = device_private(self);
	struct ahb_attach_args * const ahba = aux;

	sc->sc_dev = self;
	sc->sc_bust = ahba->ahba_memt;

	aprint_normal(": AHB to APB Bus Bridge\n");

	/*
	  * Attach critical devices
	 */
	m83apb_attach_critical(sc);

	/*
	 * Attach all other devices
	 */
	config_search(self, sc,
	    CFARGS(.search = m83apb_search));
}

static int
m83apb_search(device_t parent, cfdata_t cf, const int *ldesc, void *aux)
{
	struct m83apb_softc * const sc = aux;
	struct apb_attach_args apba;

	apba.apba_name = "aisp";
	apba.apba_memt = sc->sc_bust;
	apba.apba_addr = cf->cf_loc[APBCF_ADDR];
	apba.apba_size = cf->cf_loc[APBCF_SIZE];
	apba.apba_intr = cf->cf_loc[APBCF_INTR];
	apba.apba_irqbase = cf->cf_loc[APBCF_IRQBASE];

	if (config_probe(parent, cf, &apba))
		config_attach(parent, cf, &apba, m83apb_print, CFARGS_NONE);

	return 0;
}

static int
apb_find(device_t parent, cfdata_t cf, const int *ldesc, void *aux)
{
	struct apb_attach_args * const apba = aux;
	struct apb_attach_args apba0;

	if (strcmp(apba->apba_name, "apb"))
		return 0;
	if (apba->apba_addr != AHBCF_ADDR_DEFAULT
	    && apba->apba_addr != cf->cf_loc[AHBCF_ADDR])
		return 0;
	if (apba->apba_size != AHBCF_SIZE_DEFAULT
	    && apba->apba_size != cf->cf_loc[AHBCF_SIZE])
		return 0;
	if (apba->apba_intr != AHBCF_INTR_DEFAULT
	    && apba->apba_intr != cf->cf_loc[AHBCF_INTR])
		return 0;
	if (apba->apba_irqbase != AHBCF_IRQBASE_DEFAULT
	    && apba->apba_irqbase != cf->cf_loc[AHBCF_IRQBASE])
		return 0;

	apba0 = *apba;
	apba0.apba_addr = cf->cf_loc[AHBCF_ADDR];
	apba0.apba_size = cf->cf_loc[AHBCF_SIZE];
	apba0.apba_intr = cf->cf_loc[AHBCF_INTR];
	apba0.apba_irqbase = cf->cf_loc[AHBCF_IRQBASE];

	return config_match(parent, cf, &apba0);
}

static const struct {
	const char *name;
	bus_addr_t addr;
	bool required;
} critical_devs[] = {
	{ .name = "intc", .addr = 0x100a0000, .required = true }
};

static void
m83apb_attach_critical(struct m83apb_softc *sc)
{
	struct apb_attach_args apba;
	cfdata_t cf;
	size_t i;

	for (i = 0; i < __arraycount(critical_devs); i++) {
		apba.apba_name = "apb";
		apba.apba_memt = sc->sc_bust;

		apba.apba_addr = critical_devs[i].addr;
		apba.apba_size = AHBCF_SIZE_DEFAULT;
		apba.apba_intr = AHBCF_INTR_DEFAULT;
		apba.apba_irqbase = AHBCF_IRQBASE_DEFAULT;

		cf = config_search(sc->sc_dev, &apba,
		    CFARGS(.submatch = apb_find));
		if (cf == NULL && critical_devs[i].required)
			panic("apb_attach_critical: failed to find %s!",
			    critical_devs[i].name);

		apba.apba_addr = cf->cf_loc[AHBCF_ADDR];
		apba.apba_size = cf->cf_loc[AHBCF_SIZE];
		apba.apba_intr = cf->cf_loc[AHBCF_INTR];
		apba.apba_irqbase = cf->cf_loc[AHBCF_IRQBASE];
		config_attach(sc->sc_dev, cf, &apba, m83apb_print, CFARGS_NONE);
	}
}

static int
m83apb_print(void *aux, const char *name)
{
	struct apb_attach_args *apba = aux;

	if (apba->apba_addr != APBCF_ADDR_DEFAULT) {
		aprint_normal(" addr 0x%lx", apba->apba_addr);
		if (apba->apba_size > APBCF_SIZE_DEFAULT)
			aprint_normal("-0x%lx",
			    apba->apba_addr + apba->apba_size-1);
	}
	if (apba->apba_intr != APBCF_INTR_DEFAULT)
		aprint_normal(" intr %d", apba->apba_intr);

	return (UNCONF);
}
