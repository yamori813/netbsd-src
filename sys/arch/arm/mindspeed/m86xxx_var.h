/*	$NetBSD$	*/

#ifndef _ARM_MINDSPEED_M86VAR_H
#define _ARM_MINDSPEED_M86VAR_H

extern struct bus_space m83_bs_tag;

struct axi_attach_args {
	const char	*aa_name;
	bus_space_tag_t	aa_iot;
	bus_dma_tag_t	aa_dmat;
	bus_addr_t	aa_addr;
	bus_size_t	aa_size;
	int		aa_intr;
	int		aa_intrbase;
};

struct m86xxx_clock_info {
	uint32_t clk_arm;
	uint32_t clk_axi;
};

extern struct m86xxx_clock_info clock_info;

void	m86xxx_bootstrap(vaddr_t iobase);
void	m86xxx_device_register(device_t, void *);

#ifdef MULTIPROCESSOR
void	m86xxx_cpu_hatch(struct cpu_info *);
#endif

#endif	/* _ARM_MINDSPEED_M86VAR_H */
