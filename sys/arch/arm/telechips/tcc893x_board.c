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

#include "opt_arm_debug.h"
#include "opt_tcc893x.h"
#include "arml2cc.h"

#define	_ARM32_BUS_DMA_PRIVATE

#include <sys/cdefs.h>

__KERNEL_RCSID(1, "$NetBSD$");

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/device.h>

#include <prop/proplib.h>

#include <arm/cortex/a9tmr_var.h>
#include <arm/cortex/pl310_var.h>
#include <arm/mainbus/mainbus.h>

#include <arm/telechips/tcc893x_reg.h>
#include <arm/telechips/tcc_var.h>
#include <arm/telechips/tcc893x_board.h>

static vaddr_t baseaddr;

static uint32_t readckc(int off);
static uint32_t readckc(int off)
{

	return *(uint32_t *)(baseaddr + HwCKC_BASE -  HwGPU_BASE + off);
}

static void writeckc(int off, uint32_t val);
static void writeckc(int off, uint32_t val)
{

	*(uint32_t *)(baseaddr + HwCKC_BASE -  HwGPU_BASE + off) = val;
}

static uint32_t readhsio(int off);
static uint32_t readhsio(int off)
{

	return *(uint32_t *)(baseaddr + HwHSIOBUSCFG_BASE -  HwGPU_BASE + off);
}

/*
static void writehsio(int off, uint32_t val);
static void writehsio(int off, uint32_t val)
{

	*(uint32_t *)(baseaddr + HwHSIOBUSCFG_BASE -  HwGPU_BASE + off) = val;
}
*/

static uint32_t readmembus(int off);
static uint32_t readmembus(int off)
{

	return *(uint32_t *)(baseaddr + HwMBUSCFG_BASE -  HwGPU_BASE + off);
}

static uint32_t readgpio(int off);
static uint32_t readgpio(int off)
{

	return *(uint32_t *)(baseaddr + HwGPIO_BASE -  HwGPU_BASE + off);
}

static void writegpio(int off, uint32_t val);
static void writegpio(int off, uint32_t val)
{

	*(uint32_t *)(baseaddr + HwGPIO_BASE -  HwGPU_BASE + off) = val;
}

static void tcc893x_l2ccinit(void);
static void tcc893x_l2ccinit(void)
{
	bus_space_tag_t tcc893x_armcore_bst = &armv7_generic_bs_tag;
	bus_space_handle_t tcc893x_armcore_bsh;

	int error = bus_space_map(tcc893x_armcore_bst, L2CACHE_BASE,
	    TCC_ARMCORE_SIZE, 0, &tcc893x_armcore_bsh);
	if (error)
		panic("L2CC map error");

	uint32_t id = bus_space_read_4(tcc893x_armcore_bst, tcc893x_armcore_bsh, 0);
	printf("CACHE_ID: %x\n", id);

	arml2cc_init(tcc893x_armcore_bst, tcc893x_armcore_bsh, 0);

	bus_space_unmap(tcc893x_armcore_bst, tcc893x_armcore_bsh,
	    TCC_ARMCORE_SIZE);
}

void
tcc893x_bootstrap(vaddr_t iobase)
{
	uint32_t reg;

	baseaddr = iobase;

	curcpu()->ci_data.cpu_cc_freq = 850000000;

	int i;
	for (i= 0; i <= 0x54; i +=4) {
		printf("CKC:%02x %08x\n", i, readckc(i));
	}

	reg = readckc(offsetof(CKC, CLKCTRL2));
	if ((reg & (1 << 21)) != 0) {
		reg &= ~(1 << 21);
		writeckc(offsetof(CKC, CLKCTRL2), reg);
		printf("Disable DISPALY clock\n");
	}
/*
	reg = readckc(offsetof(CKC, PCLKCTRL12));
	if ((reg & (1 << 29)) == 0) {
		reg |= (1 << 29);
		writeckc(offsetof(CKC, PCLKCTRL12), reg);
		printf("Enable GMAC clock\n");
	}
	reg = readckc(offsetof(CKC, PCLKCTRL16));
	if ((reg & (1 << 29)) == 0) {
		reg |= (1 << 29);
		writeckc(offsetof(CKC, PCLKCTRL16), reg);
		printf("Enable USB 2.0 Host clock\n");
	}
	reg = readckc(offsetof(CKC, PCLKCTRL13));
	if ((reg & (1 << 29)) == 0) {
		reg |= (1 << 29);
		writeckc(offsetof(CKC, PCLKCTRL13), reg);
		printf("Enable OTG clock\n");
	}
*/
	reg = readmembus(offsetof(MEMBUSCFG, SWRESET));
	printf("MEMBUSCFG->SWRESET %x\n", reg);

	reg = readhsio(offsetof(HSIOBUSCFG, PWDN));
	printf("HSIOBUSCFG->PWDN %x\n", reg);

	reg = readhsio(offsetof(HSIOBUSCFG, SWRESET));
	printf("HSIOBUSCFG->SWRESET %x\n", reg);

	reg = readgpio(offsetof(GPIO, GPAEN));
	printf("GPIO A EN %x\n", reg);
	reg = readgpio(offsetof(GPIO, GPADAT));
	printf("GPIO A DAT %x\n", reg);

	reg = readgpio(offsetof(GPIO, GPBEN));
	printf("GPIO B EN %x\n", reg);
	reg = readgpio(offsetof(GPIO, GPBDAT));
	printf("GPIO B DAT %x\n", reg);

	reg = readgpio(offsetof(GPIO, GPCEN));
	printf("GPIO C EN %x\n", reg);
	reg = readgpio(offsetof(GPIO, GPCDAT));
	printf("GPIO C DAT %x\n", reg);

	reg = readgpio(offsetof(GPIO, GPDEN));
	printf("GPIO D EN %x\n", reg);
	reg = readgpio(offsetof(GPIO, GPDDAT));
	printf("GPIO D DAT %x\n", reg);

	reg = readgpio(offsetof(GPIO, GPEEN));
	printf("GPIO E EN %x\n", reg);
	reg = readgpio(offsetof(GPIO, GPEDAT));
	printf("GPIO E DAT %x\n", reg);

	/* USB2_CN Power ON (NCP382 EN1) */
	reg = readgpio(offsetof(GPIO, GPCDAT));
	reg &= ~(1 << 24);
	writegpio(offsetof(GPIO, GPCDAT), reg);
/*
	reg = readhsio(offsetof(HSIOBUSCFG, ETHER_CFG1));
	reg &= ~(1 << 31);
	writehsio(offsetof(HSIOBUSCFG, ETHER_CFG1), reg);
	reg |= (1 << 31) | (1 << 18);
	writehsio(offsetof(HSIOBUSCFG, ETHER_CFG1), reg);
	printf("Enable HSIO clock\n");
*/

	tcc893x_l2ccinit();

}

void
tcc893x_device_register(device_t self, void *aux)
{
	const uint8_t macaddr[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05};

	prop_dictionary_t dict = device_properties(self);

	if (device_is_a(self, "armperiph")
	    && device_is_a(device_parent(self), "mainbus")) {
		/*
		 * XXX KLUDGE ALERT XXX
		 * The iot mainbus supplies is completely wrong since it scales
		 * addresses by 2.  The simplest remedy is to replace with our
		 * bus space used for the armcore registers (which armperiph
		 *  uses).
		 */
		struct mainbus_attach_args * const mb = aux;
		mb->mb_iot = &armv7_generic_bs_tag;
		return;
	}

	/*
	 * We need to tell the A9 Global/Watchdog Timer
	 * what frequency it runs at.
	 */
	if (device_is_a(self, "arma9tmr") || device_is_a(self, "a9wdt")) {
		/*
		 * This clock always runs at (arm_clk div 4) and only goes
		 * to timers that are part of the A9 MP core subsystem.
		 */
                prop_dictionary_set_uint32(dict, "frequency",
//		    clock_info.clk_arm / 4);
		    180000000);
		return;
	}

	/* TCC893X PL310 offset */
	if (device_is_a(self, "arml2cc")) {
                prop_dictionary_set_uint32(dict, "offset", 0x1000);
		return;
	}

	if (device_is_a(self, "awge")) {
		prop_dictionary_set_data(dict, "mac-address", macaddr, sizeof(macaddr));
		return;
	}
}

#ifdef MULTIPROCESSOR
void
m86xxx_cpu_hatch(struct cpu_info *ci)
{
	a9tmr_init_cpu_clock(ci);
}

void
m86xxx_cpu1_reset(void)
{
	uint32_t reg;

	reg = readckc(A9DP_CPU_RESET);
	writeckc(A9DP_CPU_RESET, reg & ~CPU1_RST);

	reg = readckc(A9DP_PWR_CNTRL);
	writeckc(A9DP_PWR_CNTRL, reg & ~CLAMP_CORE1);

	reg = readckc(A9DP_CPU_CLK_CNTRL);
	writeckc(A9DP_CPU_CLK_CNTRL, reg | CPU1_CLK_ENABLE);

	reg = readckc(A9DP_CPU_RESET);
	writeckc(A9DP_CPU_RESET, reg & ~NEON1_RST);

	reg = readckc(A9DP_CPU_CLK_CNTRL);
	writeckc(A9DP_CPU_CLK_CNTRL, reg | NEON1_CLK_ENABLE);
}
#endif

