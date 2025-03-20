/*	$NetBSD$	*/

/*-
 * Copyright (c) 2025 Hiroki Mori
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

//#include "opt_ckc.h"
#include "locators.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/termios.h>

#include <machine/intr.h>
#include <sys/bus.h>

#include <arm/pic/picvar.h>

#include <arm/telechips/tcc893x_reg.h>
#include <arm/telechips/tcc_var.h>
#include <arm/telechips/tcc893x_ckc.h>

struct ckc_softc {
	device_t sc_dev;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_hdl;
};

static int	tcc893x_ckc_match(device_t, cfdata_t , void *);
static void	tcc893x_ckc_attach(device_t, device_t, void *);

CFATTACH_DECL_NEW(ckc, sizeof(struct ckc_softc),
    tcc893x_ckc_match, tcc893x_ckc_attach, NULL, NULL);

#define	CKC_READ(sc, reg)						\
	bus_space_read_4(sc->sc_iot, sc->sc_hdl, (reg))
#define	CKC_WRITE(sc, reg, val)						\
	bus_space_write_4(sc->sc_iot, sc->sc_hdl, (reg), (val))

static unsigned int tca_ckc_getpll(struct ckc_softc *sc, unsigned int ch);
unsigned int tca_ckc_getfbusctrl(struct ckc_softc *sc, unsigned int clkname);
static unsigned int tca_ckc_getperi(struct ckc_softc *sc, unsigned int periname);
static unsigned int tcc_ckc_getplldivder(struct ckc_softc *sc, unsigned int ch);
unsigned int tca_ckc_setfbusctrl(struct ckc_softc *sc, unsigned int clkname, unsigned int isenable, unsigned int freq);
unsigned int tca_ckc_setperi(struct ckc_softc *sc, unsigned int periname,unsigned int isenable, unsigned int freq);

static unsigned int	stClockSource[MAX_CLK_SRC];

static int
tcc893x_ckc_match(device_t parent, cfdata_t cf, void *aux)
{
	struct axi_attach_args *axia = aux;

	if (axia->aa_addr == -1)
	    panic("tcc893x_ckc must have addr in config.");

	if (axia->aa_size == 0)
		axia->aa_size = 0x00100000;

	return (1);
}

#if !defined(CONFIG_CHIP_TCC8935S) && !defined(CONFIG_CHIP_TCC8933S) && !defined(CONFIG_CHIP_TCC8937S)
static volatile unsigned	stPKTGENReg[4];
#endif

static void
tcc893x_ckc_attach(device_t parent, device_t self, void *aux)
{
	struct ckc_softc *sc = device_private(self);
	struct axi_attach_args *axia = aux;
	int error;

	sc->sc_dev = self;
	sc->sc_iot = axia->aa_iot;

	if (axia->aa_size == AXICF_SIZE_DEFAULT)
		axia->aa_size = 0x1000;

	error = bus_space_map(axia->aa_iot, axia->aa_addr, axia->aa_size,
	    0, &sc->sc_hdl);

	if (error) {
		aprint_error(": failed to map register %#lx@%#lx: %d\n",
		    axia->aa_size, axia->aa_addr, error);
		return;
	}

	aprint_normal("\n");

#ifdef REGDUMP
	int i;
        for (i = 0 ;i < MAX_TCC_PLL ; ++i) {
                if (i == CPU_SRC_CH) {
                        stClockSource[i] =  0;
		} else {
                        stClockSource[i] =  tca_ckc_getpll(sc, i);
                        printf("PLL %d %d\n", i, stClockSource[i]);
		}
	}
#endif


//	clk_set_rate(g_pOHCIClk, 48*1000*1000);
	unsigned int rate = 48*1000*1000;
	int idx = PERI_USB20H;
	tca_ckc_setperi(sc, idx, CKC_ENABLE, rate / 100);

//	clk_set_rate(gmac_clk, 125*1000*1000);
	rate = 125*1000*1000;
	idx = PERI_GMAC;
	tca_ckc_setperi(sc, idx, CKC_ENABLE, rate / 100);

	rate = 12*1000*1000;
	idx = PERI_OUT1;
	tca_ckc_setperi(sc, idx, CKC_ENABLE, rate / 100);

/*
	rate = 48*1000*1000;
	idx = PERI_USBOTG;
	tca_ckc_setperi(sc, idx, CKC_ENABLE, rate / 100);
*/

#ifdef REGDUMP
	printf("%d %d\n", idx, tca_ckc_getfbusctrl(sc, idx) * 100);
	idx = FBUS_HSIO;
	printf("%d %d\n", idx, tca_ckc_getfbusctrl(sc, idx) * 100);
	idx = FBUS_CPU;
	printf("%d %d\n", idx, tca_ckc_getfbusctrl(sc, idx) * 100);
	idx = FBUS_IO;
	printf("%d %d\n", idx, tca_ckc_getfbusctrl(sc, idx) * 100);
#endif
}

static inline tPCLKTYPE tcc_check_pclk_type(unsigned int periname)
{
#if defined(CONFIG_CHIP_TCC8935S) || defined(CONFIG_CHIP_TCC8933S) || defined(CONFIG_CHIP_TCC8937S)
	if (periname == PERI_HDMIA || periname == PERI_ADAI1 ||
	     periname == PERI_ADAM1 || periname == PERI_SPDIF1 ||
	     periname == PERI_ADAI0 || periname == PERI_ADAM0 ||
	      periname == PERI_ADC)
#else
	if (periname == PERI_HDMIA || periname == PERI_ADAI1 ||
	    periname == PERI_ADAM1 || periname == PERI_SPDIF1 ||
	    periname == PERI_ADAI0 || periname == PERI_ADAM0 ||
	    periname == PERI_SPDIF0 || periname == PERI_ADC)
#endif
		return PCLKCTRL_TYPE_YYY;
#if !defined(CONFIG_CHIP_TCC8935S) && !defined(CONFIG_CHIP_TCC8933S) && !defined(CONFIG_CHIP_TCC8937S)
	else if (periname == PERI_PKTGEN0 || periname == PERI_PKTGEN1
	    || periname == PERI_PKTGEN2 || periname == PERI_PKTGEN3)
		return PCLKCTRL_TYPE_ZZZ;
#endif
	else
		return PCLKCTRL_TYPE_XXX;
}

static unsigned int tcc_ckc_getplldivder(struct ckc_softc *sc, unsigned int ch)
{
	volatile unsigned	CLKDIVC;
	unsigned int		offset=0, fpll=0, pdiv=0;

	if (ch >= MAX_TCC_PLL)
		return 0;

	switch(ch) {
		case 0:
		case 1:
		case 2:
		case 3:
//			CLKDIVC = (volatile unsigned *)REG_CLKDIVC;
			CLKDIVC = CKC_READ(sc, REG_CLKDIVC);
			offset = (3-ch)*8;
			break;
		case 4:
		case 5:
//			CLKDIVC = (volatile unsigned *)REG_CLKDIVC+4;
			CLKDIVC = CKC_READ(sc, REG_CLKDIVC + 4);
			offset = (3-(ch-4))*8;
			break;
		default:
			return 0;
	}
	if (((CLKDIVC >> offset) & 0x80) == 0)	/* check plldivc enable bit */
		return 0;
	pdiv = (CLKDIVC >> offset) & 0x3F;
	if (!pdiv)  /* should not be zero */
		return 0;
	fpll = tca_ckc_getpll(sc, ch);
	return (unsigned int)fpll/(pdiv+1);
}

unsigned int tca_ckc_getfbusctrl(struct ckc_softc *sc, unsigned int clkname)
{
	volatile unsigned   CLKCTRL;
	tCLKCTRL			nCLKCTRL;
	unsigned int		src_freq = 0;

	CLKCTRL = CKC_READ(sc, REG_CLKCTRL + clkname * 4);
	nCLKCTRL.en = (CLKCTRL & (1<<CLKCTRL_EN_SHIFT)) ? 1 : 0;
//	if (nCLKCTRL.en == 0)
//		return 0;

	nCLKCTRL.sel = (CLKCTRL & (CLKCTRL_SEL_MASK<<CLKCTRL_SEL_SHIFT))>>CLKCTRL_SEL_SHIFT;
	switch (nCLKCTRL.sel) {
		case CLKCTRL_SEL_PLL0:
			src_freq =  tca_ckc_getpll(sc, PLL_0);
			break;
		case CLKCTRL_SEL_PLL1:
			src_freq =  tca_ckc_getpll(sc, PLL_1);
			break;
		case CLKCTRL_SEL_PLL2:
			src_freq =  tca_ckc_getpll(sc, PLL_2);
			break;
		case CLKCTRL_SEL_PLL3:
			src_freq =  tca_ckc_getpll(sc, PLL_3);
			break;
		case CLKCTRL_SEL_XIN:
			src_freq =  XIN_CLK_RATE;
			break;
		case CLKCTRL_SEL_PLL0DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_0);
			break;
		case CLKCTRL_SEL_PLL1DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_1);
			break;
		case CLKCTRL_SEL_XTIN:
			src_freq =  XTIN_CLK_RATE;
			break;
#if (MAX_TCC_PLL > 4)
		case CLKCTRL_SEL_PLL4:
			src_freq =  tca_ckc_getpll(sc, PLL_4);
			break;
		case CLKCTRL_SEL_PLL5:
			src_freq =  tca_ckc_getpll(sc, PLL_5);
			break;
#endif
		case CLKCTRL_SEL_PLL2DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_2);
			break;
		case CLKCTRL_SEL_PLL3DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_3);
			break;
#if (MAX_TCC_PLL > 4)
		case CLKCTRL_SEL_PLL4DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_4);
			break;
		case CLKCTRL_SEL_PLL5DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_5);
			break;
#endif
/*
		case CLKCTRL_SEL_XINDIV:
			src_freq =  XIN_CLK_RATE/2;
			break;
		case CLKCTRL_SEL_XTINDIV:
			src_freq =  XTIN_CLK_RATE/2;
			break;
*/
		default: return 0;
	}

	if(clkname == FBUS_CPU) {
		int i, lcnt=0;
		nCLKCTRL.config = (CLKCTRL & (CLKCTRL_CPU_MASK<<CLKCTRL_CONFIG_SHIFT))>>CLKCTRL_CONFIG_SHIFT;
		for(i = 0; i < 16; i++) {
			if((nCLKCTRL.config & 0x1))
				lcnt++;
			nCLKCTRL.config = nCLKCTRL.config>>1;
		}
		nCLKCTRL.freq = (src_freq * lcnt)/16;
	}
	else {
		nCLKCTRL.config = (CLKCTRL & (CLKCTRL_CONFIG_MASK<<CLKCTRL_CONFIG_SHIFT))>>CLKCTRL_CONFIG_SHIFT;
		nCLKCTRL.freq = src_freq / (nCLKCTRL.config+1);
	}

	return nCLKCTRL.freq;
}
static unsigned int tca_ckc_getperi(struct ckc_softc *sc, unsigned int periname)
{
	unsigned int	pPCLKCTRL;
	tPCLKCTRL 	nPCLKCTRL;
	tPCLKTYPE	type = tcc_check_pclk_type(periname);
	unsigned int	src_freq = 0, div_mask;

	pPCLKCTRL = CKC_READ(sc, REG_PCLKCTRL + periname * 4);

#if !defined(CONFIG_CHIP_TCC8935S) && !defined(CONFIG_CHIP_TCC8933S) && !defined(CONFIG_CHIP_TCC8937S)
//	if (type == PCLKCTRL_TYPE_ZZZ)
//		pPCLKCTRL = (volatile unsigned *)(&stPKTGENReg[periname - PERI_PKTGEN0]);
#endif

	nPCLKCTRL.en = (pPCLKCTRL & (PCLKCTRL_EN_MASK << PCLKCTRL_EN_SHIFT)) ? 1 : 0;
	if (nPCLKCTRL.en == 0)
		return 0;

	nPCLKCTRL.sel = (pPCLKCTRL & (PCLKCTRL_SEL_MASK << PCLKCTRL_SEL_SHIFT)) >> PCLKCTRL_SEL_SHIFT;
	switch(nPCLKCTRL.sel) {
		case PCLKCTRL_SEL_PLL0 :
			src_freq =  tca_ckc_getpll(sc, PLL_0);
			break;
		case PCLKCTRL_SEL_PLL1 :
			src_freq =  tca_ckc_getpll(sc, PLL_1);
			break;
		case PCLKCTRL_SEL_PLL2 :
			src_freq =  tca_ckc_getpll(sc, PLL_2);
			break;
		case PCLKCTRL_SEL_PLL3 :
			src_freq =  tca_ckc_getpll(sc,  PLL_3);
			break;
		case PCLKCTRL_SEL_XIN :
			src_freq =  XIN_CLK_RATE;
			break;
		case PCLKCTRL_SEL_PLL0DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_0);
			break;
		case PCLKCTRL_SEL_PLL1DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_1);
			break;
		case PCLKCTRL_SEL_PLL2DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_2);
			break;
		case PCLKCTRL_SEL_PLL3DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_3);
			break;
		case PCLKCTRL_SEL_XTIN:
			src_freq =  XTIN_CLK_RATE;
			break;
		/*
		case PCLKCTRL_SEL_HDMITMDS:
			src_freq =  ;
			break;
		case PCLKCTRL_SEL_HDMIPCLK:
			src_freq =  ;
			break;
		*/
		case PCLKCTRL_SEL_HDMIXIN:
			src_freq =  HDMI_CLK_RATE;
			break;
		/*
		case PCLKCTRL_SEL_XINDIV:
			src_freq =  60000;
			break;
		case PCLKCTRL_SEL_XTINDIV:
			src_freq =  163;
			break;
		*/
#if (MAX_TCC_PLL > 4)
		case PCLKCTRL_SEL_PLL4:
			src_freq =  tca_ckc_getpll(sc, PLL_4);
			break;
		case PCLKCTRL_SEL_PLL5:
			src_freq =  tca_ckc_getpll(sc, PLL_5);
			break;
		case PCLKCTRL_SEL_PLL4DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_4);
			break;
		case PCLKCTRL_SEL_PLL5DIV:
			src_freq =  tcc_ckc_getplldivder(sc, PLL_5);
			break;
#endif
		default :
			return 0;
	}

	switch (type) {
		case PCLKCTRL_TYPE_XXX:
			div_mask = PCLKCTRL_DIV_XXX_MASK;
			nPCLKCTRL.md = PCLKCTRL_MODE_DIVIDER;
			break;
		case PCLKCTRL_TYPE_YYY:
			div_mask = PCLKCTRL_DIV_YYY_MASK;
			break;
		case PCLKCTRL_TYPE_ZZZ:
			div_mask = PCLKCTRL_DIV_ZZZ_MASK;
			break;
		default:
			return 0;
	}
	nPCLKCTRL.freq = 0;
	nPCLKCTRL.div = (pPCLKCTRL&(div_mask<<PCLKCTRL_DIV_SHIFT))>>PCLKCTRL_DIV_SHIFT;
	if (nPCLKCTRL.md == PCLKCTRL_MODE_DIVIDER)
		nPCLKCTRL.freq = src_freq/(nPCLKCTRL.div+1);
	else {
		if (nPCLKCTRL.div > (div_mask+1)/2)
			nPCLKCTRL.freq = (src_freq*((div_mask+1) - nPCLKCTRL.div)) / (div_mask+1);
		else
			nPCLKCTRL.freq = (src_freq*nPCLKCTRL.div) / (div_mask+1);
	}
	return nPCLKCTRL.freq;
}

static unsigned int tca_ckc_getpll(struct ckc_softc *sc, unsigned int ch)
{
	unsigned int  		PLLCFG;
	tPMS			nPLLCFG;
	unsigned int		src_freq;

	if (ch >= MAX_TCC_PLL)
		return 0;

	PLLCFG = CKC_READ(sc, REG_PLL + ch * 4);

	nPLLCFG.p = (PLLCFG >> PLL_P_SHIFT) & (PLL_P_MASK);
	nPLLCFG.m = (PLLCFG >> PLL_M_SHIFT) & (PLL_M_MASK);
	nPLLCFG.s = (PLLCFG >> PLL_S_SHIFT) & (PLL_S_MASK);
	nPLLCFG.en = (PLLCFG >> PLL_EN_SHIFT) & (PLL_EN_MASK);
	nPLLCFG.src = (PLLCFG >> PLL_SRC_SHIFT) & (PLL_SRC_MASK);

	if (nPLLCFG.en == 0)
		return 0;

	switch (nPLLCFG.src) {
		case PLLSRC_XIN:
			src_freq = XIN_CLK_RATE;
			break;
		case PLLSRC_HDMIXI:
			src_freq = HDMI_CLK_RATE;
			break;
		case PLLSRC_EXTCLK0:
			src_freq = tca_ckc_getperi(sc, PERI_OUT0);
			break;
		case PLLSRC_EXTCLK1:
			src_freq = tca_ckc_getperi(sc, PERI_OUT1);
			break;
		default:
			return 0;
	}

	return (((src_freq * nPLLCFG.m) / nPLLCFG.p) >> nPLLCFG.s);
}

//static inline int tcc_find_clkctrl(tCLKCTRL *CLKCTRL)
int tcc_find_clkctrl(tCLKCTRL *CLKCTRL);
int tcc_find_clkctrl(tCLKCTRL *CLKCTRL)
{
	unsigned int i, div[MAX_CLK_SRC], div_100[MAX_CLK_SRC], searchsrc, overclksrc;
	searchsrc = 0xFFFFFFFF;
	overclksrc = 0xFFFFFFFF;

#if (1) /* sometimes the clkctrl can not work, when it setted lower clock */
	if (CLKCTRL->freq < 480000)
		CLKCTRL->freq = 480000;
#endif

	if (CLKCTRL->freq <= (XIN_CLK_RATE/2)) {
		CLKCTRL->sel = CLKCTRL_SEL_XIN;
		CLKCTRL->freq = XIN_CLK_RATE/2;
		CLKCTRL->config = CLKCTRL_CONFIG_MIN;
	}
	else {
		for (i=0 ; i<MAX_CLK_SRC ; i++) {
			if (stClockSource[i] < CLKCTRL->freq || stClockSource[i] == 0)
				continue;
			div_100[i] = stClockSource[i]/(CLKCTRL->freq/100);
			if (div_100[i] > (CLKCTRL_CONFIG_MAX+1)*100)
				div_100[i] = (CLKCTRL_CONFIG_MAX+1)*100;
			/* find maximum frequency pll source */
			if (div_100[i] <= 100) {
				if (overclksrc == 0xFFFFFFFF)
					overclksrc = i;
				else if (stClockSource[i] > stClockSource[overclksrc])
					overclksrc = i;
				continue;
			}
			div[i]= div_100[i]/100;
			if (div_100[i]%100)
				div[i] += 1;
			if (div[i] < 2)
				div[i] = 2;
			div_100[i] = CLKCTRL->freq - stClockSource[i]/div[i];
			if (searchsrc == 0xFFFFFFFF)
				searchsrc = i;
			else {
				/* find similar clock */
				if (div_100[i] < div_100[searchsrc])
					searchsrc = i;
				/* find even division vlaue */
				else if(div_100[i] == div_100[searchsrc]) {
					if (div[searchsrc]%2)
						searchsrc = i;
					else if (div[searchsrc] > div[i])
						searchsrc = i;
				}
			}
		}
		if (searchsrc == 0xFFFFFFFF) {
			if (overclksrc == 0xFFFFFFFF) {
				overclksrc = 0;
				for (i=1 ; i<MAX_CLK_SRC ; i++) {
					if (stClockSource[i] > stClockSource[overclksrc])
						overclksrc = i;
				}
			}
			searchsrc = overclksrc;
			div[searchsrc] = 2;
		}		
		switch(searchsrc) {
			case 0: CLKCTRL->sel = CLKCTRL_SEL_PLL0; break;
			case 1: CLKCTRL->sel = CLKCTRL_SEL_PLL1; break;
			case 2: CLKCTRL->sel = CLKCTRL_SEL_PLL2; break;
			case 3: CLKCTRL->sel = CLKCTRL_SEL_PLL3; break;
#if (MAX_TCC_PLL > 4)
			case 4: CLKCTRL->sel = CLKCTRL_SEL_PLL4; break;
			case 5: CLKCTRL->sel = CLKCTRL_SEL_PLL5; break;
			case 6: CLKCTRL->sel = CLKCTRL_SEL_PLL0DIV; break;
			case 7: CLKCTRL->sel = CLKCTRL_SEL_PLL1DIV; break;
			case 8: CLKCTRL->sel = CLKCTRL_SEL_PLL2DIV; break;
			case 9: CLKCTRL->sel = CLKCTRL_SEL_PLL3DIV; break;
			case 10: CLKCTRL->sel = CLKCTRL_SEL_PLL4DIV; break;
			case 11: CLKCTRL->sel = CLKCTRL_SEL_PLL5DIV; break;
			case 12: CLKCTRL->sel = CLKCTRL_SEL_XIN; break;
#else
			case 4: CLKCTRL->sel = CLKCTRL_SEL_PLL0DIV; break;
			case 5: CLKCTRL->sel = CLKCTRL_SEL_PLL1DIV; break;
			case 6: CLKCTRL->sel = CLKCTRL_SEL_PLL2DIV; break;
			case 7: CLKCTRL->sel = CLKCTRL_SEL_PLL3DIV; break;
			case 8: CLKCTRL->sel = CLKCTRL_SEL_XIN; break;
#endif
			default: return -1;
		}
		if (div[searchsrc] > (CLKCTRL_CONFIG_MAX+1))
			div[searchsrc] = CLKCTRL_CONFIG_MAX+1;
		else if (div[searchsrc] <= CLKCTRL_CONFIG_MIN)
			div[searchsrc] = CLKCTRL_CONFIG_MIN+1;
		CLKCTRL->freq = stClockSource[searchsrc]/div[searchsrc];
		CLKCTRL->config = div[searchsrc] - 1;
	}
	return 0;
}

unsigned int tca_ckc_setfbusctrl(struct ckc_softc *sc, unsigned int clkname,
    unsigned int isenable, unsigned int freq)  /* freq(100Hz) */
{
	volatile unsigned	CLKCTRL;
	tCLKCTRL		nCLKCTRL;

	CLKCTRL = CKC_READ(sc, REG_CLKCTRL + clkname * 4);

	if (clkname == FBUS_CPU) {
#if 0
		tPMS				nPLL;
		volatile unsigned   *PLLCFG = (volatile unsigned *)REG_PLL+CPU_SRC_CH;
		tcc_cpu_write(CLKCTRL, 1, CLKCTRL_CPU_MASK, CLKCTRL_SEL_XIN);
		nPLL.fpll = freq;
		if (tcc_find_pms(&nPLL, XIN_CLK_RATE))
			return 0;
		tcc_pll_write(PLLCFG, nPLL.en, nPLL.vsel, nPLL.p, nPLL.m, nPLL.s, PLLSRC_XIN);
		tcc_cpu_write(CLKCTRL, 1, CLKCTRL_CPU_MASK, CPU_SRC_PLL);
		return tca_ckc_getpll(sc, CPU_SRC_CH);
#endif
		return 0;
	}
	else if (clkname == FBUS_MEM) {
#if defined(CONFIG_SUSPEND_MEMCLK) || defined(CONFIG_CLOCK_TABLE)
		if (freq < 3000000)
			freq = 3000000;
		freq /= 2;
#else
		// do not change memory clock. just return current memroy clock rate.
		return tca_ckc_getfbusctrl(sc, clkname);
#endif
		return 0;
	}

	nCLKCTRL.freq = freq;
	if (tcc_find_clkctrl(&nCLKCTRL))
		return 0;

#if defined(CONFIG_SUSPEND_MEMCLK) || defined(CONFIG_CLOCK_TABLE)
	if (clkname == FBUS_MEM)
		return tcc_ddr_set_clock(nCLKCTRL.freq/10, nCLKCTRL.sel, nCLKCTRL.config);
#endif

	switch (isenable) {
		case CKC_DISABLE:
			nCLKCTRL.en = 0;
			break;
		case CKC_ENABLE:
			nCLKCTRL.en = 1;
			break;
		default:
			nCLKCTRL.en = CLKCTRL & (1 << CLKCTRL_EN_SHIFT) ? 1 : 0;
			break;
	}
//	tcc_clkctrl_write(CLKCTRL, nCLKCTRL.en, nCLKCTRL.config, nCLKCTRL.sel);

	return nCLKCTRL.freq;
}

/*
#define tcc_clkctrl_write(reg,en,config,sel) { \
	*(volatile unsigned *)reg = ((*(volatile unsigned *)reg)&(~(CLKCTRL_SEL_MASK<<CLKCTRL_SEL_SHIFT)))|((sel&CLKCTRL_SEL_MASK)<<CLKCTRL_SEL_SHIFT); \
	while((*(volatile unsigned *)reg) & (1<<CLKCTRL_CHGRQ_SHIFT)); \
	*(volatile unsigned *)reg = ((*(volatile unsigned *)reg)&(~(CLKCTRL_CONFIG_MASK<<CLKCTRL_CONFIG_SHIFT)))|((config&CLKCTRL_CONFIG_MASK)<<CLKCTRL_CONFIG_SHIFT); \
	while((*(volatile unsigned *)reg) & (1<<CLKCTRL_CFGRQ_SHIFT)); \
	*(volatile unsigned *)reg = ((*(volatile unsigned *)reg)&(~(CLKCTRL_EN_MASK<<CLKCTRL_EN_SHIFT)))|((en&CLKCTRL_EN_MASK)<<CLKCTRL_EN_SHIFT); \
	while((*(volatile unsigned *)reg) & (1<<CLKCTRL_CFGRQ_SHIFT)); \
}
*/

static inline int tcc_find_pclk(tPCLKCTRL *PCLKCTRL, tPCLKTYPE type);
static inline int tcc_find_pclk(tPCLKCTRL *PCLKCTRL, tPCLKTYPE type)
{
	unsigned int	div_min, div_max, div[MAX_CLK_SRC], div_100[MAX_CLK_SRC], i, searchsrc, overclksrc, dco_shift=0;

	switch (type) {
		case PCLKCTRL_TYPE_XXX:
			PCLKCTRL->md = PCLKCTRL_MODE_DIVIDER;
			div_max = PCLKCTRL_DIV_XXX_MAX+1;
			div_min = PCLKCTRL_DIV_MIN+1;
			break;
		case PCLKCTRL_TYPE_YYY:
			PCLKCTRL->md = PCLKCTRL_MODE_DCO;
			div_max = PCLKCTRL_DIV_YYY_MAX+1;
			div_min = PCLKCTRL_DIV_DCO_MIN+1;
#ifdef CONFIG_AUDIO_PLL_USE
			if (tcc_find_audio_pclk(PCLKCTRL) == 0)
				return 0;
#endif
			break;
		case PCLKCTRL_TYPE_ZZZ:
			PCLKCTRL->md = PCLKCTRL_MODE_DIVIDER;
			div_max = PCLKCTRL_DIV_XXX_MAX+1;
			div_min = PCLKCTRL_DIV_MIN+1;
			break;
		default:
			return -1;
	}

	searchsrc = 0xFFFFFFFF;
	overclksrc = 0xFFFFFFFF;
	if (PCLKCTRL->md == PCLKCTRL_MODE_DCO) {
		if (PCLKCTRL->freq < div_max)
			dco_shift = 0;
		else if (PCLKCTRL->freq < div_max*2)  //  13.1072 MHz
			dco_shift = 1;
		else if (PCLKCTRL->freq < div_max*4)  //  26.2144 MHz
			dco_shift = 2;
		else if (PCLKCTRL->freq < div_max*8)  //  52.4288 MHz
			dco_shift = 3;
		else if (PCLKCTRL->freq < div_max*16) // 104.8596 MHz
			dco_shift = 4;
		else						 // 209.7152 MHz
			dco_shift = 5;

		for (i=0 ; i<MAX_CLK_SRC ; i++) {
			if (stClockSource[i] == 0 || stClockSource[i] == XIN_CLK_RATE)	// remove XIN clock source
				continue;
			if (stClockSource[i] < PCLKCTRL->freq)
				continue;
			div_100[i] = ((PCLKCTRL->freq*(div_max>>dco_shift))/(stClockSource[i]/100))<<dco_shift;
			if ((div_100[i]%100) > 50) {
				div[i] = div_100[i]/100 + 1;
				div_100[i] = 100 - (div_100[i]%100);
			}
			else {
				div[i] = div_100[i]/100;
				div_100[i] %= 100;
			}
			if (searchsrc == 0xFFFFFFFF)
				searchsrc = i;
			else {
				/* find similar clock */
				if (div_100[i] < div_100[searchsrc])
					searchsrc = i;
			}
		}
		if (searchsrc == 0xFFFFFFFF) {
			if (overclksrc == 0xFFFFFFFF) {
				overclksrc = 0;
				for (i=1 ; i<MAX_CLK_SRC ; i++) {
					if (stClockSource[i] > stClockSource[overclksrc])
						overclksrc = i;
				}
			}
			searchsrc = overclksrc;
			div[searchsrc] = 1;
		}
	}
	else { /* Divider mode */
		for (i=0 ; i<MAX_CLK_SRC ; i++) {
			if (stClockSource[i] == 0)
				continue;
			if (stClockSource[i] < PCLKCTRL->freq)
				continue;
			div_100[i] = stClockSource[i]/(PCLKCTRL->freq/100);
			if (div_100[i] > div_max*100)
				div_100[i] = div_max*100;
			if ((div_100[i]%100) > 50) {
				div[i] = div_100[i]/100 + 1;
				div_100[i] = 100 - (div_100[i]%100);
			}
			else {
				div[i] = div_100[i]/100;
				div_100[i] %= 100;
			}
			if (searchsrc == 0xFFFFFFFF)
				searchsrc = i;
			else {
				/* find similar clock */
				if (div_100[i] < div_100[searchsrc])
					searchsrc = i;
				/* find even division vlaue */
				else if(div_100[i] == div_100[searchsrc]) {
					if (div[searchsrc]%2)
						searchsrc = i;
					else if (div[searchsrc] > div[i])
						searchsrc = i;
				}
			}
		}
		if (searchsrc == 0xFFFFFFFF) {
			if (overclksrc == 0xFFFFFFFF) {
				overclksrc = 0;
				for (i=1 ; i<MAX_CLK_SRC ; i++) {
					if (stClockSource[i] > stClockSource[overclksrc])
						overclksrc = i;
				}
			}
			searchsrc = overclksrc;
			div[searchsrc] = 1;
		}
	}

	switch(searchsrc) {
		case 0: PCLKCTRL->sel = PCLKCTRL_SEL_PLL0; break;
		case 1: PCLKCTRL->sel = PCLKCTRL_SEL_PLL1; break;
		case 2: PCLKCTRL->sel = PCLKCTRL_SEL_PLL2; break;
		case 3: PCLKCTRL->sel = PCLKCTRL_SEL_PLL3; break;
#if (MAX_TCC_PLL > 4)
		case 4: PCLKCTRL->sel = PCLKCTRL_SEL_PLL4; break;
		case 5: PCLKCTRL->sel = PCLKCTRL_SEL_PLL5; break;
		case 6: PCLKCTRL->sel = PCLKCTRL_SEL_PLL0DIV; break;
		case 7: PCLKCTRL->sel = PCLKCTRL_SEL_PLL1DIV; break;
		case 8: PCLKCTRL->sel = PCLKCTRL_SEL_PLL2DIV; break;
		case 9: PCLKCTRL->sel = PCLKCTRL_SEL_PLL3DIV; break;
		case 10: PCLKCTRL->sel = PCLKCTRL_SEL_PLL4DIV; break;
		case 11: PCLKCTRL->sel = PCLKCTRL_SEL_PLL5DIV; break;
		case 12: PCLKCTRL->sel = PCLKCTRL_SEL_XIN; break;
#else
		case 4: PCLKCTRL->sel = PCLKCTRL_SEL_PLL0DIV; break;
		case 5: PCLKCTRL->sel = PCLKCTRL_SEL_PLL1DIV; break;
		case 6: PCLKCTRL->sel = PCLKCTRL_SEL_PLL2DIV; break;
		case 7: PCLKCTRL->sel = PCLKCTRL_SEL_PLL3DIV; break;
		case 8: PCLKCTRL->sel = PCLKCTRL_SEL_XIN; break;
#endif
		default: return -1;
	}

	if (PCLKCTRL->md == PCLKCTRL_MODE_DCO) {
		PCLKCTRL->div = div[searchsrc];
		if (PCLKCTRL->div > div_max/2)
			PCLKCTRL->freq = ((stClockSource[searchsrc]>>dco_shift)*(div_max-PCLKCTRL->div))/(div_max>>dco_shift);
		else
			PCLKCTRL->freq = ((stClockSource[searchsrc]>>dco_shift)*PCLKCTRL->div)/(div_max>>dco_shift);

		if (PCLKCTRL->div < div_min || PCLKCTRL->div > div_max)
			return -1;
	}
	else { /* Divider mode */
		PCLKCTRL->div = div[searchsrc];
		PCLKCTRL->freq = stClockSource[searchsrc]/PCLKCTRL->div;
		if (PCLKCTRL->div >= div_min && PCLKCTRL->div <= div_max)
			PCLKCTRL->div -= 1;
		else
			return -1;
	}
	return 0;
}
unsigned int tca_ckc_setperi(struct ckc_softc *sc, unsigned int periname,unsigned int isenable, unsigned int freq)  /* freq(100Hz) */
{
	volatile unsigned   PCLKCTRL;
	tPCLKCTRL		   nPCLKCTRL;
	tPCLKTYPE		   type = tcc_check_pclk_type(periname);

	PCLKCTRL = CKC_READ(sc, REG_PCLKCTRL + periname * 4);

	nPCLKCTRL.freq = freq;
	nPCLKCTRL.periname = periname;
	nPCLKCTRL.div = 0;
	nPCLKCTRL.md = PCLKCTRL_MODE_DCO;
	nPCLKCTRL.sel = PCLKCTRL_SEL_XIN;

	switch (type) {
		case PCLKCTRL_TYPE_YYY:
			if (nPCLKCTRL.periname == PERI_ADC) {
				nPCLKCTRL.md = PCLKCTRL_MODE_DIVIDER;
				nPCLKCTRL.sel = PCLKCTRL_SEL_XIN;
				nPCLKCTRL.div = (XIN_CLK_RATE+nPCLKCTRL.freq-1)/nPCLKCTRL.freq;
				nPCLKCTRL.freq= XIN_CLK_RATE/nPCLKCTRL.div;
				if (nPCLKCTRL.div > PCLKCTRL_DIV_MIN && nPCLKCTRL.div <= (PCLKCTRL_DIV_YYY_MAX+1))
					nPCLKCTRL.div -= 1;
				else 
					goto tca_ckc_setperi_failed;
			}
			else
			if (tcc_find_pclk(&nPCLKCTRL, type))
				goto tca_ckc_setperi_failed;
			break;

#if !defined(CONFIG_CHIP_TCC8935S) && !defined(CONFIG_CHIP_TCC8933S) && !defined(CONFIG_CHIP_TCC8937S)
		case PCLKCTRL_TYPE_ZZZ:  /* (n=0~63, n== 60,61,62,63) */
			if (nPCLKCTRL.freq == 240000 || nPCLKCTRL.freq == 120000 || nPCLKCTRL.freq == 80000 || nPCLKCTRL.freq == 60000 \
			 || nPCLKCTRL.freq == 40000 || nPCLKCTRL.freq == 30000 || nPCLKCTRL.freq <= 20000) {
				nPCLKCTRL.md = PCLKCTRL_MODE_DIVIDER;
				nPCLKCTRL.sel = PCLKCTRL_SEL_XIN;
				nPCLKCTRL.div = (XIN_CLK_RATE+nPCLKCTRL.freq-1)/nPCLKCTRL.freq;
				nPCLKCTRL.freq= XIN_CLK_RATE/nPCLKCTRL.div;
				if (nPCLKCTRL.div > PCLKCTRL_DIV_MIN && nPCLKCTRL.div <= (PCLKCTRL_DIV_ZZZ_MAX+1))
					nPCLKCTRL.div -= 1;
				else 
					goto tca_ckc_setperi_failed;
			}
			else {
				if (tcc_find_pclk(&nPCLKCTRL, type))
					goto tca_ckc_setperi_failed;
			}
			break;
#endif
		case PCLKCTRL_TYPE_XXX:
			if (nPCLKCTRL.freq == XTIN_CLK_RATE) {
				if ((nPCLKCTRL.periname == PERI_LCD0 || nPCLKCTRL.periname == PERI_LCD1)) {
					nPCLKCTRL.sel = PCLKCTRL_SEL_HDMIPCLK;
					nPCLKCTRL.div = 0;
				}
				else {
					nPCLKCTRL.sel = PCLKCTRL_SEL_XTIN;
					nPCLKCTRL.div = 0;
				}
			}
#if !defined(CONFIG_HDMI_CLK_USE_INTERNAL_PLL)
			else if (nPCLKCTRL.periname == PERI_HDMI && nPCLKCTRL.freq == HDMI_CLK_RATE) {
				#if defined(CONFIG_HDMI_CLK_USE_XIN_24MHZ)
				nPCLKCTRL.sel = PCLKCTRL_SEL_XIN;
				nPCLKCTRL.div = 0;
				nPCLKCTRL.freq = XIN_CLK_RATE;
				#else
				nPCLKCTRL.sel = PCLKCTRL_SEL_HDMIXIN;
				nPCLKCTRL.div = 0;
				nPCLKCTRL.freq = HDMI_CLK_RATE;
				#endif /* CONFIG_HDMI_CLK_USE_XIN_24MHZ */
			}
#endif
			else if (nPCLKCTRL.freq == 240000 || nPCLKCTRL.freq == 120000 || nPCLKCTRL.freq == 80000 || nPCLKCTRL.freq == 60000 \
				  || nPCLKCTRL.freq == 40000 || nPCLKCTRL.freq == 30000 || nPCLKCTRL.freq <= 20000) {
				nPCLKCTRL.sel = PCLKCTRL_SEL_XIN;
				nPCLKCTRL.div = (XIN_CLK_RATE+nPCLKCTRL.freq-1)/nPCLKCTRL.freq;
				nPCLKCTRL.freq = XIN_CLK_RATE/nPCLKCTRL.div;
				if (nPCLKCTRL.div > PCLKCTRL_DIV_MIN && nPCLKCTRL.div <= (PCLKCTRL_DIV_XXX_MAX+1))
					nPCLKCTRL.div -= 1;
				else 
					goto tca_ckc_setperi_failed;
			}
			else {
				if (tcc_find_pclk(&nPCLKCTRL, type))
					goto tca_ckc_setperi_failed;
			}
			break;
		default:
			goto tca_ckc_setperi_failed;
	}

	/* enable bit */
#if !defined(CONFIG_CHIP_TCC8935S) && !defined(CONFIG_CHIP_TCC8933S) && !defined(CONFIG_CHIP_TCC8937S)
	if (type == PCLKCTRL_TYPE_ZZZ) {
		switch (isenable) {
			case CKC_DISABLE:
				nPCLKCTRL.en = 0;
				break;
			case CKC_ENABLE:
				nPCLKCTRL.en = 1;
				break;
			default:
				nPCLKCTRL.en = (stPKTGENReg[periname-PERI_PKTGEN0] & (1<<PCLKCTRL_EN_SHIFT)) ? 1 : 0;
				break;
		}
//		tcc_pclkctrl_write(&stPKTGENReg[periname-PERI_PKTGEN0], nPCLKCTRL.md, nPCLKCTRL.en, nPCLKCTRL.sel, nPCLKCTRL.div, type);
	} else 
#endif
	{
		switch (isenable) {
			case CKC_DISABLE:
				nPCLKCTRL.en = 0;
				break;
			case CKC_ENABLE:
				nPCLKCTRL.en = 1;
				break;
			default:
				nPCLKCTRL.en = (PCLKCTRL & (1<<PCLKCTRL_EN_SHIFT)) ? 1 : 0;
				break;
		}
	}

//	tcc_pclkctrl_write(PCLKCTRL, nPCLKCTRL.md, nPCLKCTRL.en, nPCLKCTRL.sel, nPCLKCTRL.div, type);
	if (type == PCLKCTRL_TYPE_XXX) {
//		printf("XXX %d %d %d %d\n", periname, nPCLKCTRL.en, nPCLKCTRL.sel, nPCLKCTRL.div);
		CKC_WRITE(sc, REG_PCLKCTRL + periname * 4,
		    (nPCLKCTRL.en << PCLKCTRL_EN_SHIFT) |
		    (nPCLKCTRL.sel << PCLKCTRL_SEL_SHIFT) | nPCLKCTRL.div);
	}

	return nPCLKCTRL.freq;

tca_ckc_setperi_failed:
//	tcc_pclkctrl_write(PCLKCTRL, PCLKCTRL_MODE_DIVIDER, CKC_DISABLE, PCLKCTRL_SEL_XIN, 1, type);

	return 0;
}
