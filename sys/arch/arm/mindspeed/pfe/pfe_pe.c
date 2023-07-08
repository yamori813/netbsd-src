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

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/device.h>
#include <sys/ioctl.h>
#include <sys/intr.h>
#include <sys/kmem.h>
#include <sys/mutex.h>
#include <sys/systm.h>
#include <sys/kernel.h>

#include <arm/mindspeed/if_pgereg.h>
#include <arm/mindspeed/pfe/base/pfe.h>
#include <arm/mindspeed/pfe/hal.h>

void *cbus_base_addr;
void *ddr_base_addr;
unsigned long ddr_phys_base_addr;

struct pge_softc *pge_sc;

static struct pe_info pe[MAX_PE];

/** Initializes the PFE library.
* Must be called before using any of the library functions.
*
* @param[in] cbus_base		CBUS virtual base address
*				(as mapped in the host CPU address space)
* @param[in] ddr_base		DDR virtual base address
*				(as mapped in the host CPU address space)
* @param[in] ddr_phys_base	DDR physical base address
*				(as mapped in platform)
*/
void pfe_lib_init(void *cbus_base, void *ddr_base, unsigned long ddr_phys_base)
{
	cbus_base_addr = cbus_base;
	ddr_base_addr = ddr_base;
	ddr_phys_base_addr = ddr_phys_base;

	pe[CLASS0_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(0);
	pe[CLASS0_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(0);
	pe[CLASS0_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS0_ID].mem_access_wdata = (void *)CLASS_MEM_ACCESS_WDATA;
	pe[CLASS0_ID].mem_access_addr = (void *)CLASS_MEM_ACCESS_ADDR;
	pe[CLASS0_ID].mem_access_rdata = (void *)CLASS_MEM_ACCESS_RDATA;

	pe[CLASS1_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(1);
	pe[CLASS1_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(1);
	pe[CLASS1_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS1_ID].mem_access_wdata = (void *)CLASS_MEM_ACCESS_WDATA;
	pe[CLASS1_ID].mem_access_addr = (void *)CLASS_MEM_ACCESS_ADDR;
	pe[CLASS1_ID].mem_access_rdata = (void *)CLASS_MEM_ACCESS_RDATA;

	pe[CLASS2_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(2);
	pe[CLASS2_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(2);
	pe[CLASS2_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS2_ID].mem_access_wdata = (void *)CLASS_MEM_ACCESS_WDATA;
	pe[CLASS2_ID].mem_access_addr = (void *)CLASS_MEM_ACCESS_ADDR;
	pe[CLASS2_ID].mem_access_rdata = (void *)CLASS_MEM_ACCESS_RDATA;

	pe[CLASS3_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(3);
	pe[CLASS3_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(3);
	pe[CLASS3_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS3_ID].mem_access_wdata = (void *)CLASS_MEM_ACCESS_WDATA;
	pe[CLASS3_ID].mem_access_addr = (void *)CLASS_MEM_ACCESS_ADDR;
	pe[CLASS3_ID].mem_access_rdata = (void *)CLASS_MEM_ACCESS_RDATA;

#if !defined(CONFIG_PLATFORM_PCI)
	pe[CLASS4_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(4);
	pe[CLASS4_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(4);
	pe[CLASS4_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS4_ID].mem_access_wdata = (void *)CLASS_MEM_ACCESS_WDATA;
	pe[CLASS4_ID].mem_access_addr = (void *)CLASS_MEM_ACCESS_ADDR;
	pe[CLASS4_ID].mem_access_rdata = (void *)CLASS_MEM_ACCESS_RDATA;

	pe[CLASS5_ID].dmem_base_addr = CLASS_DMEM_BASE_ADDR(5);
	pe[CLASS5_ID].pmem_base_addr = CLASS_IMEM_BASE_ADDR(5);
	pe[CLASS5_ID].pmem_size = CLASS_IMEM_SIZE;
	pe[CLASS5_ID].mem_access_wdata = (void *)CLASS_MEM_ACCESS_WDATA;
	pe[CLASS5_ID].mem_access_addr = (void *)CLASS_MEM_ACCESS_ADDR;
	pe[CLASS5_ID].mem_access_rdata = (void *)CLASS_MEM_ACCESS_RDATA;
#endif
	pe[TMU0_ID].dmem_base_addr = TMU_DMEM_BASE_ADDR(0);
	pe[TMU0_ID].pmem_base_addr = TMU_IMEM_BASE_ADDR(0);
	pe[TMU0_ID].pmem_size = TMU_IMEM_SIZE;
	pe[TMU0_ID].mem_access_wdata = (void *)TMU_MEM_ACCESS_WDATA;
	pe[TMU0_ID].mem_access_addr = (void *)TMU_MEM_ACCESS_ADDR;
	pe[TMU0_ID].mem_access_rdata = (void *)TMU_MEM_ACCESS_RDATA;

#if !defined(CONFIG_TMU_DUMMY)
	pe[TMU1_ID].dmem_base_addr = TMU_DMEM_BASE_ADDR(1);
	pe[TMU1_ID].pmem_base_addr = TMU_IMEM_BASE_ADDR(1);
	pe[TMU1_ID].pmem_size = TMU_IMEM_SIZE;
	pe[TMU1_ID].mem_access_wdata = (void *)TMU_MEM_ACCESS_WDATA;
	pe[TMU1_ID].mem_access_addr = (void *)TMU_MEM_ACCESS_ADDR;
	pe[TMU1_ID].mem_access_rdata = (void *)TMU_MEM_ACCESS_RDATA;

	pe[TMU2_ID].dmem_base_addr = TMU_DMEM_BASE_ADDR(2);
	pe[TMU2_ID].pmem_base_addr = TMU_IMEM_BASE_ADDR(2);
	pe[TMU2_ID].pmem_size = TMU_IMEM_SIZE;
	pe[TMU2_ID].mem_access_wdata = (void *)TMU_MEM_ACCESS_WDATA;
	pe[TMU2_ID].mem_access_addr = (void *)TMU_MEM_ACCESS_ADDR;
	pe[TMU2_ID].mem_access_rdata = (void *)TMU_MEM_ACCESS_RDATA;

	pe[TMU3_ID].dmem_base_addr = TMU_DMEM_BASE_ADDR(3);
	pe[TMU3_ID].pmem_base_addr = TMU_IMEM_BASE_ADDR(3);
	pe[TMU3_ID].pmem_size = TMU_IMEM_SIZE;
	pe[TMU3_ID].mem_access_wdata = (void *)TMU_MEM_ACCESS_WDATA;
	pe[TMU3_ID].mem_access_addr = (void *)TMU_MEM_ACCESS_ADDR;
	pe[TMU3_ID].mem_access_rdata = (void *)TMU_MEM_ACCESS_RDATA;
#endif

#if !defined(CONFIG_UTIL_PE_DISABLED)
	pe[UTIL_ID].dmem_base_addr = UTIL_DMEM_BASE_ADDR;
	pe[UTIL_ID].mem_access_wdata = (void *)UTIL_MEM_ACCESS_WDATA;
	pe[UTIL_ID].mem_access_addr = (void *)UTIL_MEM_ACCESS_ADDR;
	pe[UTIL_ID].mem_access_rdata = (void *)UTIL_MEM_ACCESS_RDATA;
#endif
}


/** Writes a buffer to PE internal memory from the host
 * through indirect access registers.
 *
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID, ..., UTIL_ID)
 * @param[in] src		Buffer source address
 * @param[in] mem_access_addr	DMEM destination address (must be 32bit aligned)
 * @param[in] len		Number of bytes to copy
 */
void pe_mem_memcpy_to32(int id, u32 mem_access_addr, u8 *src, unsigned int len);
void pe_mem_memcpy_to32(int id, u32 mem_access_addr, u8 *src, unsigned int len)
{
	u32 offset = 0, val, addr;
	unsigned int len32 = len >> 2;
	int i;

	addr = mem_access_addr | PE_MEM_ACCESS_WRITE | PE_MEM_ACCESS_BYTE_ENABLE(0, 4);

	for (i = 0; i < len32; i++, offset += 4, src += 4) {
		val = *(u32 *)src;
		writel(cpu_to_be32(val), (int)pe[id].mem_access_wdata);
		writel(addr + offset, (int)pe[id].mem_access_addr);
	}

	if ((len = (len & 0x3))) {
		val = 0;

		addr = (mem_access_addr | PE_MEM_ACCESS_WRITE | PE_MEM_ACCESS_BYTE_ENABLE(0, len)) + offset;

		for (i = 0; i < len; i++, src++)
			val |= (*(u8 *)src) << (8 * i);

		writel(cpu_to_be32(val), (int)pe[id].mem_access_wdata);
		writel(addr, (int)pe[id].mem_access_addr);
	}
}

/** Writes a buffer to PE internal data memory (DMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID, ..., UTIL_ID)
 * @param[in] src		Buffer source address
 * @param[in] dst		DMEM destination address (must be 32bit aligned)
 * @param[in] len		Number of bytes to copy
 */
void pe_dmem_memcpy_to32(int id, u32 dst, void *src, unsigned int len)
{
	pe_mem_memcpy_to32(id, pe[id].dmem_base_addr | dst | PE_MEM_ACCESS_DMEM, (u8 *)src, len);
}


/** Writes a buffer to PE internal program memory (PMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID, ..., TMU3_ID)
 * @param[in] src		Buffer source address
 * @param[in] dst		PMEM destination address (must be 32bit aligned)
 * @param[in] len		Number of bytes to copy
 */
void pe_pmem_memcpy_to32(int id, u32 dst, void *src, unsigned int len)
{
	pe_mem_memcpy_to32(id, pe[id].pmem_base_addr | (dst & (pe[id].pmem_size - 1)) | PE_MEM_ACCESS_IMEM, (u8 *)src, len);
}


/** Reads PE internal program memory (IMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID, ..., TMU3_ID)
 * @param[in] addr		PMEM read address (must be aligned on size)
 * @param[in] size		Number of bytes to read (maximum 4, must not cross 32bit boundaries)
 * @return			the data read (in PE endianess, i.e BE).
 */
u32 pe_pmem_read(int id, u32 addr, u8 size)
{
	u32 offset = addr & 0x3;
	u32 mask = 0xffffffff >> ((4 - size) << 3);
	u32 val;

	addr = pe[id].pmem_base_addr | ((addr & ~0x3) & (pe[id].pmem_size - 1)) | PE_MEM_ACCESS_READ | PE_MEM_ACCESS_IMEM | PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

	writel(addr, (int)pe[id].mem_access_addr);
	val = be32_to_cpu(readl((int)pe[id].mem_access_rdata));

	return (val >> (offset << 3)) & mask;
}


/** Writes PE internal data memory (DMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID, ..., UTIL_ID)
 * @param[in] addr		DMEM write address (must be aligned on size)
 * @param[in] val		Value to write (in PE endianess, i.e BE)
 * @param[in] size		Number of bytes to write (maximum 4, must not cross 32bit boundaries)
 */
void pe_dmem_write(int id, u32 val, u32 addr, u8 size)
{
	u32 offset = addr & 0x3;

	addr = pe[id].dmem_base_addr | (addr & ~0x3) | PE_MEM_ACCESS_WRITE | PE_MEM_ACCESS_DMEM | PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

	/* Indirect access interface is byte swapping data being written */
	writel(cpu_to_be32(val << (offset << 3)), (int)pe[id].mem_access_wdata);
	writel(addr, (int)pe[id].mem_access_addr);
}


/** Reads PE internal data memory (DMEM) from the host
 * through indirect access registers.
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID, ..., UTIL_ID)
 * @param[in] addr		DMEM read address (must be aligned on size)
 * @param[in] size		Number of bytes to read (maximum 4, must not cross 32bit boundaries)
 * @return			the data read (in PE endianess, i.e BE).
 */
u32 pe_dmem_read(int id, u32 addr, u8 size)
{
	u32 offset = addr & 0x3;
	u32 mask = 0xffffffff >> ((4 - size) << 3);
	u32 val;

	addr = pe[id].dmem_base_addr | (addr & ~0x3) | PE_MEM_ACCESS_READ | PE_MEM_ACCESS_DMEM | PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

	writel(addr, (int)pe[id].mem_access_addr);

	/* Indirect access interface is byte swapping data being read */
	val = be32_to_cpu(readl((int)pe[id].mem_access_rdata));

	return (val >> (offset << 3)) & mask;
}


#ifdef NOTUSE
/** Writes UTIL program memory (DDR) from the host.
 *
 * @param[in] addr	Address to write (virtual, must be aligned on size)
 * @param[in] val		Value to write (in PE endianess, i.e BE)
 * @param[in] size		Number of bytes to write (2 or 4)
 */
static void util_pmem_write(u32 val, void *addr, u8 size)
{
	void *addr64 = (void *)((unsigned long)addr & ~0x7);
	unsigned long off = 8 - ((unsigned long)addr & 0x7) - size;
	
	//IMEM should  be loaded as a 64bit swapped value in a 64bit aligned location
	if (size == 4)
		writel(be32_to_cpu(val), (int)addr64 + off);
	else
		writew(be16_to_cpu((u16)val), (int)addr64 + off);
}


/** Writes a buffer to UTIL program memory (DDR) from the host.
 *
 * @param[in] dst	Address to write (virtual, must be at least 16bit aligned)
 * @param[in] src	Buffer to write (in PE endianess, i.e BE, must have same alignment as dst)
 * @param[in] len	Number of bytes to write (must be at least 16bit aligned)
 */
static void util_pmem_memcpy(void *dst, void *src, unsigned int len)
{
	unsigned int len32;
	int i;

	if ((unsigned long)src & 0x2) {
		util_pmem_write(*(u16 *)src, dst, 2);
		(int)src += 2;
		(int)dst += 2;
		len -= 2;
	}

	len32 = len >> 2;

	for (i = 0; i < len32; i++, (u8 *)dst += 4, (u8 *)src += 4)
		util_pmem_write(*(u32 *)src, dst, 4);

	if (len & 0x2)
		util_pmem_write(*(u16 *)src, dst, len & 0x2);
}
#endif   /* NOTUSE */


/** Loads an elf section into pmem
 * Code needs to be at least 16bit aligned and only PROGBITS sections are supported
 *
 * @param[in] id	PE identification (CLASS0_ID, ..., TMU0_ID, ..., TMU3_ID)
 * @param[in] data	pointer to the elf firmware
 * @param[in] shdr	pointer to the elf section header
 *
 */
static int pe_load_pmem_section(int id, const void *data, Elf32_Shdr *shdr)
{
	u32 offset = be32_to_cpu(shdr->sh_offset);
	u32 addr = be32_to_cpu(shdr->sh_addr);
	u32 size = be32_to_cpu(shdr->sh_size);
	u32 type = be32_to_cpu(shdr->sh_type);

#if !defined(CONFIG_UTIL_PE_DISABLED)
	if (id == UTIL_ID)
	{
		printk(KERN_ERR "%s: unsuported pmem section for UTIL\n", __func__);
		return -EINVAL;
	}
#endif

	if ((((unsigned long)data + offset) & 0x3) != (addr & 0x3))
	{
		printk(KERN_ERR "%s: load address(%x) and elf file address(%lx) don't have the same alignment\n",
			__func__, addr, (unsigned long) data + offset);

		return -EINVAL;
	}

	if (addr & 0x1)
	{
		printk(KERN_ERR "%s: load address(%x) is not 16bit aligned\n", __func__, addr);
		return -EINVAL;
	}

	if (size & 0x1)
	{
		printk(KERN_ERR "%s: load size(%x) is not 16bit aligned\n", __func__, size);
		return -EINVAL;
	}

	switch (type)
        {
        case SHT_PROGBITS:
		pe_pmem_memcpy_to32(id, addr, (void *)((int)data + offset), size);
		break;

	default:
		printk(KERN_ERR "%s: unsuported section type(%x)\n", __func__, type);
		return -EINVAL;
		break;
	}

	return 0;
}


/** Loads an elf section into dmem
 * Data needs to be at least 32bit aligned, NOBITS sections are correctly initialized to 0
 *
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID, ..., UTIL_ID)
 * @param[in] data		pointer to the elf firmware
 * @param[in] shdr		pointer to the elf section header
 *
 */
static int pe_load_dmem_section(int id, void *data, Elf32_Shdr *shdr)
{
	u32 offset = be32_to_cpu(shdr->sh_offset);
	u32 addr = be32_to_cpu(shdr->sh_addr);
	u32 size = be32_to_cpu(shdr->sh_size);
	u32 type = be32_to_cpu(shdr->sh_type);
	u32 size32 = size >> 2;
	int i;

	if ((((unsigned long)data + offset) & 0x3) != (addr & 0x3))
	{
		printk(KERN_ERR "%s: load address(%x) and elf file address(%lx) don't have the same alignment\n",
			__func__, addr, (unsigned long)data + offset);

		return -EINVAL;
	}

	if (addr & 0x3)
	{
		printk(KERN_ERR "%s: load address(%x) is not 32bit aligned\n", __func__, addr);
		return -EINVAL;
	}

	switch (type)
        {
        case SHT_PROGBITS:
		pe_dmem_memcpy_to32(id, addr, (void *)((int)data + offset), size);
		break;

	case SHT_NOBITS:
		for (i = 0; i < size32; i++, addr += 4)
			pe_dmem_write(id, 0, addr, 4);

		if (size & 0x3)
			pe_dmem_write(id, 0, addr, size & 0x3);

		break;

	default:
		printk(KERN_ERR "%s: unsuported section type(%x)\n", __func__, type);
		return -EINVAL;
		break;
	}

	return 0;
}


/** Loads an elf section into DDR
 * Data needs to be at least 32bit aligned, NOBITS sections are correctly initialized to 0
 *
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID, ..., UTIL_ID)
 * @param[in] data		pointer to the elf firmware
 * @param[in] shdr		pointer to the elf section header
 *
 */
static int pe_load_ddr_section(int id, void *data, Elf32_Shdr *shdr)
{
	u32 offset = be32_to_cpu(shdr->sh_offset);
	u32 addr = be32_to_cpu(shdr->sh_addr);
	u32 size = be32_to_cpu(shdr->sh_size);
	u32 type = be32_to_cpu(shdr->sh_type);
	u32 flags = be32_to_cpu(shdr->sh_flags);
//	u32 size32 = size >> 2;
//	int i;

	switch (type)
	{
	case SHT_PROGBITS:
		if (flags & SHF_EXECINSTR)
		{
#if !defined(CONFIG_UTIL_PE_DISABLED)
			if (id == UTIL_ID)
			{
				if ((((unsigned long)data + offset) & 0x3) != (addr & 0x3))
				{
					printk(KERN_ERR "%s: load address(%x) and elf file address(%lx) don't have the same alignment\n",
								__func__, addr, (unsigned long)data + offset);

					return -EINVAL;
				}

				if (addr & 0x1)
				{
					printk(KERN_ERR "%s: load address(%x) is not 16bit aligned\n", __func__, addr);
					return -EINVAL;
				}

				if (size & 0x1)
				{
					printk(KERN_ERR "%s: load length(%x) is not 16bit aligned\n", __func__, size);
					return -EINVAL;
				}
#ifdef NOTUSE
				util_pmem_memcpy((void *)DDR_PHYS_TO_VIRT(addr), data + offset, size);
#endif
				printk(KERN_ERR "%s: util_pmem_memcpy %x %x %x %x\n", __func__, addr, (int)data, offset, size);
			}
			else
#endif
			{
				printk(KERN_ERR "%s: unsuported ddr section type(%x) for PE(%d)\n", __func__, type, id);
				return -EINVAL;
			}

		}
		else
		{
//			memcpy(DDR_PHYS_TO_VIRT(addr), data + offset, size);
			printk(KERN_ERR "%s: memcpy %x %x %x %x\n", __func__, addr, (int)data, offset, size);
		}

		break;

	case SHT_NOBITS:
//		memset(DDR_PHYS_TO_VIRT(addr), 0, size);
		printk(KERN_ERR "%s: memset %x %x %x %x\n", __func__, addr, (int)data, offset, size);

		break;

	default:
		printk(KERN_ERR "%s: unsuported section type(%x)\n", __func__, type);
		return -EINVAL;
		break;
	}

	return 0;
}


/** Loads an elf section into a PE
 * For now only supports loading a section to dmem (all PE's), pmem (class and tmu PE's),
 * DDDR (util PE code)
 *
 * @param[in] id		PE identification (CLASS0_ID, ..., TMU0_ID, ..., UTIL_ID)
 * @param[in] data		pointer to the elf firmware
 * @param[in] shdr		pointer to the elf section header
 *
 */
int pe_load_elf_section(int id, void *data, Elf32_Shdr *shdr)
{
	u32 addr = be32_to_cpu(shdr->sh_addr);
	u32 size = be32_to_cpu(shdr->sh_size);

	if (IS_DMEM(addr, size))
		return pe_load_dmem_section(id, data, shdr);
	else if (IS_PMEM(addr, size))
		return pe_load_pmem_section(id, data, shdr);
	else if (IS_PFE_LMEM(addr, size))
		return 0; /* FIXME */
	else if (IS_PHYS_DDR(addr, size))
		return pe_load_ddr_section(id, data, shdr);
	else if (IS_PE_LMEM(addr, size))
		return 0; /* FIXME */
	else {
		printk(KERN_ERR "%s: unsuported memory range(%x)\n", __func__, addr);
//		return -EINVAL;
	}

	return 0;
}

#ifdef NOTUSE
/** This function is used to write to UTIL internal bus peripherals from the host
* through indirect access registers.
* @param[in]	val	32bits value to write
* @param[in]	addr	Address to write to
* @param[in]	size	Number of bytes to write
*
*/
void util_bus_write(u32 val, u32 addr, u8 size)
{
	u32 offset = addr & 0x3;
	u32 access_addr;

	access_addr = ((addr & ~0x3) & CLASS_BUS_ACCESS_ADDR_MASK) | PE_MEM_ACCESS_WRITE | PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

//	writel((addr & CLASS_BUS_ACCESS_BASE_MASK), CLASS_BUS_ACCESS_BASE);

	writel(cpu_to_be32(val << (offset << 3)), UTIL_BUS_ACCESS_WDATA);
	writel(access_addr, UTIL_BUS_ACCESS_ADDR);
}


/** Reads from UTIL internal bus peripherals from the host
* through indirect access registers.
* @param[in] addr	Address to read from
* @param[in] size	Number of bytes to read
* @return		the read data
*
*/
u32 util_bus_read(u32 addr, u8 size)
{
	u32 offset = addr & 0x3;
	u32 mask = 0xffffffff >> ((4 - size) << 3);
	u32 access_addr, val;

	access_addr = ((addr & ~0x3) & CLASS_BUS_ACCESS_ADDR_MASK) | PE_MEM_ACCESS_READ | PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

//	writel((addr & CLASS_BUS_ACCESS_BASE_MASK), CLASS_BUS_ACCESS_BASE);

	writel(access_addr, UTIL_BUS_ACCESS_ADDR);
	val = be32_to_cpu(readl(UTIL_BUS_ACCESS_RDATA));

	return (val >> (offset << 3)) & mask;
}

/** This function is used to write to CLASS internal bus peripherals (ccu, pe-lem) from the host
* through indirect access registers.
* @param[in]	val	32bits value to write
* @param[in]	addr	Address to write to
* @param[in]	size	Number of bytes to write
*
*/
void class_bus_write(u32 val, u32 addr, u8 size)
{
	u32 offset = addr & 0x3;
	u32 access_addr;

	access_addr = ((addr & ~0x3) & CLASS_BUS_ACCESS_ADDR_MASK) | PE_MEM_ACCESS_WRITE | PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

	writel((addr & CLASS_BUS_ACCESS_BASE_MASK), CLASS_BUS_ACCESS_BASE);

	writel(cpu_to_be32(val << (offset << 3)), CLASS_BUS_ACCESS_WDATA);
	writel(access_addr, CLASS_BUS_ACCESS_ADDR);
}


/** Reads from CLASS internal bus peripherals (ccu, pe-lem) from the host
* through indirect access registers.
* @param[in] addr	Address to read from
* @param[in] size	Number of bytes to read
* @return		the read data
*
*/
u32 class_bus_read(u32 addr, u8 size)
{
	u32 offset = addr & 0x3;
	u32 mask = 0xffffffff >> ((4 - size) << 3);
	u32 access_addr, val;
	
	access_addr = ((addr & ~0x3) & CLASS_BUS_ACCESS_ADDR_MASK) | PE_MEM_ACCESS_READ | PE_MEM_ACCESS_BYTE_ENABLE(offset, size);

	writel((addr & CLASS_BUS_ACCESS_BASE_MASK), CLASS_BUS_ACCESS_BASE);

	writel(access_addr, CLASS_BUS_ACCESS_ADDR);
	val = be32_to_cpu(readl(CLASS_BUS_ACCESS_RDATA));

	return (val >> (offset << 3)) & mask;
}


/** Reads data from the cluster memory (PE_LMEM)
* @param[out] dst		pointer to the source buffer data are copied to
* @param[in] len		length in bytes of the amount of data to read from cluster memory
* @param[in] offset	offset in bytes in the cluster memory where data are read from
*/
void pe_lmem_read(u32 *dst, u32 len, u32 offset)
{
	u32 len32 = len >> 2;
	int i = 0;

	for (i = 0; i < len32; dst++, i++, offset += 4)
		*dst = class_bus_read(PE_LMEM_BASE_ADDR + offset, 4);

	/* FIXME we may have an out of bounds access on dst */
	if (len & 0x03)
		*dst = class_bus_read(PE_LMEM_BASE_ADDR + offset, (len & 0x03));
}

/** Writes data to the cluster memory (PE_LMEM)
* @param[in] src	pointer to the source buffer data are copied from
* @param[in] len	length in bytes of the amount of data to write to the cluster memory
* @param[in] offset	offset in bytes in the cluster memory where data are written to
*/
void pe_lmem_write(u32 *src, u32 len, u32 offset)
{
	u32 len32 = len >> 2;
	int i = 0;

	for (i = 0; i < len32; src++, i++, offset += 4)
		class_bus_write(*src, PE_LMEM_BASE_ADDR + offset, 4);

	/* FIXME we may have an out of bounds access on src */
	if (len & 0x03)
		class_bus_write(*src, PE_LMEM_BASE_ADDR + offset, (len & 0x03));
}
#endif   /* NOTUSE */
