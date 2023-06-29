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
	int		aa_irq;
	int		aa_irqbase;
};

struct apb_attach_args {
	const char	*apba_name;
	bus_space_tag_t	apba_memt;
	bus_dma_tag_t	apba_dmat;
	bus_addr_t	apba_addr;
	bus_size_t	apba_size;
	int		apba_intr;
	int		apba_irqbase;
};

struct ahb_attach_args {
	const char	*ahba_name;
	bus_space_tag_t	ahba_memt;
	bus_dma_tag_t	ahba_dmat;
	bus_addr_t	ahba_addr;
	bus_size_t	ahba_size;
	int		ahba_intr;
	int		ahba_irqbase;
};

void	m86xxx_device_register(device_t, void *);

#endif	/* _ARM_MINDSPEED_M86VAR_H */
