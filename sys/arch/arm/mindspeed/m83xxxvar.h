/*	$NetBSD$	*/

#ifndef _ARM_MINDSPEED_M83VAR_H
#define _ARM_MINDSPEED_M83VAR_H

extern struct bus_space m83_bs_tag;

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

#endif	/* _ARM_MINDSPEED_M83VAR_H */
