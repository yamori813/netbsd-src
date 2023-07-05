/*
 * Copyright 2018-2019 NXP
 * All rights reserved.
 * Based on DPDK Project
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the NXP nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTERS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _HAL_H_
#define _HAL_H_

#if defined(CONFIG_PLATFORM_PCI)  
/* For ChipIT */

#include <linux/types.h>
#include <linux/elf.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <asm/io.h>
#include <linux/slab.h>
#include <linux/firmware.h>

#include "pfe_mod.h"

#define free(x)  kfree(x)
#define xzalloc(x)  kmalloc(x, GFP_DMA)
#define printf  printk

//#define dprint(fmt, arg...)	printk(fmt, ##arg)
#define dprint(fmt, arg...)	

#else

/*
#include <linux/types.h>
#include <elf.h>
#include <common.h>
#include <errno.h>
#include <asm/byteorder.h>
#include <miidev.h>
#include <malloc.h>
#include <asm/io.h>
*/


#include "c2000_eth.h"

#endif


#endif /* _HAL_H_ */

