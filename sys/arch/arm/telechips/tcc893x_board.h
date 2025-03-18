/*	$NetBSD$	*/

#ifndef _ARM_TELECHIPS_TCC893X_BOADR_H_
#define _ARM_TELECHIPS_TCC893X_BOADR_H_

void	tcc893x_bootstrap(vaddr_t iobase);
void	tcc893x_device_register(device_t, void *);

#ifdef MULTIPROCESSOR
void	tcc893x_cpu_hatch(struct cpu_info *);
#endif

#endif	/* _ARM_TELECHIPS_TCC893X_BOADR_H_ */
