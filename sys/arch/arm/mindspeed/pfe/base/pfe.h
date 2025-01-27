#ifndef _PFE_H_
#define _PFE_H_

#include <sys/exec_elf.h>
#define be16_to_cpu(x) be16toh(x)
#define be32_to_cpu(x) be32toh(x)
#define cpu_to_be16(x) htobe16(x)
#define cpu_to_be32(x) htobe32(x)

#define CLASS_DMEM_BASE_ADDR(i)	(0x00000000 | ((i) << 20))
#define CLASS_PMEM_BASE_ADDR(i)	(0x00000000 | ((i) << 20)) /* Only valid for mem access register interface */
#define CLASS_DMEM_SIZE		0x00002000
#define CLASS_PMEM_SIZE		0x00008000

#define TMU_DMEM_BASE_ADDR(i)	(0x00000000 + ((i) << 20))
#define TMU_PMEM_BASE_ADDR(i)	(0x00000000 + ((i) << 20)) /* Only valid for mem access register interface */
#define TMU_DMEM_SIZE		0x00000800
#define TMU_PMEM_SIZE		0x00002000

#define UTIL_DMEM_BASE_ADDR	0x00000000
#define UTIL_DMEM_SIZE		0x00002000

#define PE_LMEM_BASE_ADDR	0xc3010000
#define PE_LMEM_SIZE		0x8000
#define PE_LMEM_END		(PE_LMEM_BASE_ADDR + PE_LMEM_SIZE)

#define DMEM_BASE_ADDR		0x00000000
#define DMEM_SIZE		0x2000		/**< TMU has less... */
#define DMEM_END		(DMEM_BASE_ADDR + DMEM_SIZE)

#define PMEM_BASE_ADDR		0x00010000
#define PMEM_SIZE		0x8000		/**< TMU has less... */
#define PMEM_END		(PMEM_BASE_ADDR + PMEM_SIZE)


/* These check memory ranges from PE point of view/memory map */
#define IS_DMEM(addr, len)	(((unsigned long)(addr) >= DMEM_BASE_ADDR) && (((unsigned long)(addr) + (len)) <= DMEM_END))
#define IS_PMEM(addr, len)	(((unsigned long)(addr) >= PMEM_BASE_ADDR) && (((unsigned long)(addr) + (len)) <= PMEM_END))
#define IS_PE_LMEM(addr, len)	(((unsigned long)(addr) >= PE_LMEM_BASE_ADDR) && (((unsigned long)(addr) + (len)) <= PE_LMEM_END))

#define IS_PFE_LMEM(addr, len)	(((unsigned long)(addr) >= CBUS_VIRT_TO_PFE(LMEM_BASE_ADDR)) && (((unsigned long)(addr) + (len)) <= CBUS_VIRT_TO_PFE(LMEM_END)))
#define IS_PHYS_DDR(addr, len)	(((unsigned long)(addr) >= DDR_PHYS_BASE_ADDR) && (((unsigned long)(addr) + (len)) <= DDR_PHYS_END))

/* If using a run-time virtual address for the cbus base address use this code */
extern void *cbus_base_addr;
extern void *ddr_base_addr;
extern unsigned long ddr_phys_base_addr;

#if 0
/* If using a run-time virtual address for the cbus base address use this code */
extern void *cbus_base_addr;
#define CBUS_BASE_ADDR		cbus_base_addr
#else
//#define CBUS_BASE_ADDR		0x9c000000
#define CBUS_BASE_ADDR		0
#endif

#define DDR_PHYS_BASE_ADDR	ddr_phys_base_addr
#define DDR_BASE_ADDR		ddr_base_addr

#define DDR_SIZE	0xC00000
#define DDR_PHYS_END	(DDR_PHYS_BASE_ADDR + DDR_SIZE)

#define PFE_CBUS_PHYS_BASE_ADDR	0xc0000000	/**< CBUS physical base address as seen by PE's. */


//#define CBUS_PHYS_TO_VIRT(p)	(((p) - CBUS_PHYS_BASE_ADDR) + CBUS_BASE_ADDR)
//#define CBUS_VIRT_TO_PHYS(v)	(((v) - CBUS_BASE_ADDR) + CBUS_PHYS_BASE_ADDR)

#define DDR_PHYS_TO_VIRT(p)	(((p) - DDR_PHYS_BASE_ADDR) + DDR_BASE_ADDR)
#define DDR_VIRT_TO_PHYS(v)	(((v) - DDR_BASE_ADDR) + DDR_PHYS_BASE_ADDR)

#define CBUS_VIRT_TO_PFE(v)	(((v) - CBUS_BASE_ADDR) + PFE_CBUS_PHYS_BASE_ADDR)
#define CBUS_PFE_TO_VIRT(p)	(((p) - PFE_CBUS_PHYS_BASE_ADDR) + CBUS_BASE_ADDR)

#include "cbus.h"

enum {
	CLASS0_ID = 0,
	CLASS1_ID,
	CLASS2_ID,
	CLASS3_ID,
#if !defined(CONFIG_PLATFORM_PCI)
	CLASS4_ID,
	CLASS5_ID,
#endif
#if !defined(CONFIG_TMU_DUMMY)
	TMU0_ID,
	TMU1_ID,
	TMU2_ID,
	TMU3_ID,
#else
	TMU0_ID,
#endif
#if !defined(CONFIG_UTIL_PE_DISABLED)
	UTIL_ID,
#endif
	MAX_PE
};

#if !defined(CONFIG_PLATFORM_PCI)
#define CLASS_MASK	((1 << CLASS0_ID) | (1 << CLASS1_ID) | (1 << CLASS2_ID) | (1 << CLASS3_ID) | (1 << CLASS4_ID) | (1 << CLASS5_ID))
#define CLASS_MAX_ID	CLASS5_ID
#else
#define CLASS_MASK      ((1 << CLASS0_ID) | (1 << CLASS1_ID) | (1 << CLASS2_ID) | (1 << CLASS3_ID))
#define CLASS_MAX_ID	CLASS3_ID
#endif

#if !defined(CONFIG_TMU_DUMMY)
#define TMU_MASK	((1 << TMU0_ID) | (1 << TMU1_ID) | (1 << TMU2_ID) | (1 << TMU3_ID))
#define TMU_MAX_ID	TMU3_ID
#else
#define TMU_MASK	(1 << TMU0_ID) 
#define TMU_MAX_ID	TMU0_ID
#endif

#if !defined(CONFIG_UTIL_PE_DISABLED)
#define UTIL_MASK	(1 << UTIL_ID)
#endif

struct pe_sync_mailbox
{
	u32 stop;
	u32 stopped;
};

struct pe_msg_mailbox
{
	u32 dst;
	u32 src;
	u32 len;
	u32 request;
};

/** PE information.
 * Structure containing PE's specific information. It is used to create
 * generic C functions common to all PE's.
 * Before using the library functions this structure needs to be initialized with the different registers virtual addresses
 * (according to the ARM MMU mmaping). The default initialization supports a virtual == physical mapping.
 *
 */
struct pe_info
{
	u32 dmem_base_addr;		/**< PE's dmem base address */
	u32 pmem_base_addr;		/**< PE's pmem base address */
	u32 pmem_size;			/**< PE's pmem size */

	void *mem_access_wdata;		/**< PE's _MEM_ACCESS_WDATA register address */
	void *mem_access_addr;		/**< PE's _MEM_ACCESS_ADDR register address */
	void *mem_access_rdata;		/**< PE's _MEM_ACCESS_RDATA register address */
};


void pe_lmem_read(u32 *dst, u32 len, u32 offset);
void pe_lmem_write(u32 *src, u32 len, u32 offset);

void pe_dmem_memcpy_to32(int id, u32 dst, void *src, unsigned int len);
void pe_pmem_memcpy_to32(int id, u32 dst, void *src, unsigned int len);

u32 pe_pmem_read(int id, u32 addr, u8 size);

void pe_dmem_write(int id, u32 val, u32 addr, u8 size);
u32 pe_dmem_read(int id, u32 addr, u8 size);
void class_bus_write(u32 val, u32 addr, u8 size);
u32 class_bus_read(u32 addr, u8 size);
void util_bus_write(u32 val, u32 addr, u8 size);
u32 util_bus_read(u32 addr, u8 size);

#define class_bus_readl(addr)			class_bus_read(addr, 4)
#define class_bus_readw(addr)			class_bus_read(addr, 2)
#define class_bus_readb(addr)			class_bus_read(addr, 1)

#define class_bus_writel(val, addr)		class_bus_write(val, addr, 4)
#define class_bus_writew(val, addr)		class_bus_write(val, addr, 2)
#define class_bus_writeb(val, addr)		class_bus_write(val, addr, 1)

#define pe_mem_readl(id, addr)			pe_mem_read(id, addr, 4)
#define pe_mem_readw(id, addr)			pe_mem_read(id, addr, 2)
#define pe_mem_readb(id, addr)			pe_mem_read(id, addr, 1)

#define pe_mem_writel(id, val, addr)		pe_mem_write(id, val, addr, 4)
#define pe_mem_writew(id, val, addr)		pe_mem_write(id, val, addr, 2)
#define pe_mem_writeb(id, val, addr)		pe_mem_write(id, val, addr, 1)

int pe_load_elf_section(int id, void *data, Elf32_Shdr *shdr);

void pfe_lib_init(void *cbus_base, void *ddr_base, unsigned long ddr_phys_base);

void bmu_init(void *base, BMU_CFG *cfg);
void bmu_reset(void *base);
void bmu_enable(void *base);
void bmu_disable(void *base);
void bmu_set_config(void *base, BMU_CFG *cfg);

int gemac_enet_addr_byte_mac(u8 *enet_byte_addr, MAC_ADDR *enet_addr);
void gemac_init(void *base, void *config);
void gemac_set_speed(void *base, MAC_SPEED gem_speed);
void gemac_set_duplex(void *base, int duplex);
void gemac_set_mode(void *base, int mode);
void gemac_enable_mdio(void *base);
void gemac_disable_mdio(void *base);
void gemac_set_mdc_div(void *base, MAC_MDC_DIV gem_mdcdiv);
int gemac_get_mdc_div(void *base);
void gemac_enable(void *base);
void gemac_disable(void *base);
void gemac_enable_mdio(void *base);
void gemac_disable_mdio(void *base);
void gemac_reset(void *base);
void gemac_set_address(void *base, SPEC_ADDR *addr);
SPEC_ADDR gemac_get_address(void *base);
void gemac_set_laddr1(void *base, MAC_ADDR *address);
void gemac_set_laddr2(void *base, MAC_ADDR *address);
void gemac_set_laddr3(void *base, MAC_ADDR *address);
void gemac_set_laddr4(void *base, MAC_ADDR *address);
void gemac_set_laddrN(void *base, MAC_ADDR *address, unsigned int entry_index);
MAC_ADDR gem_get_laddr1(void *base);
MAC_ADDR gem_get_laddr2(void *base);
MAC_ADDR gem_get_laddr3(void *base);
MAC_ADDR gem_get_laddr4(void *base);
MAC_ADDR gem_get_laddrN(void *base, unsigned int entry_index);
void gemac_set_config(void *base, GEMAC_CFG *cfg);
void gemac_enable_copy_all(void *base);
void gemac_disable_copy_all(void *base);
void gemac_allow_broadcast(void *base);
void gemac_no_broadcast(void *base);
void gemac_enable_unicast(void *base);
void gemac_disable_unicast(void *base);
void gemac_enable_multicast(void *base);
void gemac_disable_multicast(void *base);
void gemac_enable_fcs_rx(void *base);
void gemac_disable_fcs_rx(void *base);
void gemac_enable_1536_rx(void *base);
void gemac_disable_1536_rx(void *base);
void gemac_enable_pause_rx(void *base);
void gemac_disable_pause_rx(void *base);
void gemac_enable_rx_checksum_offload(void *base);
void gemac_disable_rx_checksum_offload(void *base);
unsigned int * gemac_get_stats(void *base);
void gemac_set_bus_width(void *base, int width);

void gpi_init(void *base, GPI_CFG *cfg);
void gpi_reset(void *base);
void gpi_enable(void *base);
void gpi_disable(void *base);
void gpi_set_config(void *base, GPI_CFG *cfg);

void class_init(CLASS_CFG *cfg);
void class_reset(void);
void class_enable(void);
void class_disable(void);
void class_set_config(CLASS_CFG *cfg);

void tmu_init(TMU_CFG *cfg);
void tmu_enable(u32 pe_mask);
void tmu_disable(u32 pe_mask);

void util_init(UTIL_CFG *cfg);
void util_reset(void);
void util_enable(void);
void util_disable(void);

void hif_nocpy_init(void);
void hif_init(void);
void hif_tx_enable(void);
void hif_tx_disable(void);
void hif_rx_enable(void);
void hif_rx_disable(void);

#endif /* _PFE_H_ */
