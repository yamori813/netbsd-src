#	$NetBSD$

SYSTEM_FIRST_OBJ=	star_start.o
SYSTEM_FIRST_SFILE=	${THISARM}/star/star_start.S

#KERNEL_BASE_PHYS=0x20008000
#KERNEL_BASE_VIRT=0xc0008000
KERNEL_BASE_PHYS=0x20040000
KERNEL_BASE_VIRT=0xc0040000

#SYSTEM_LD_TAIL_EXTRA+=; \
#	echo ${OBJCOPY} -S -O binary $@ $@.bin; \
#	${OBJCOPY} -S -O binary $@ $@.bin;
#
#EXTRA_KERNELS+= ${KERNELS:@.KERNEL.@${.KERNEL.}.bin@}
#EXTRA_KERNELS+= ${KERNELS:@.KERNEL.@${.KERNEL.}.bin.gz@}

MKUBOOTIMAGEARGS=	-A arm -T kernel
MKUBOOTIMAGEARGS+=	-a $(KERNEL_BASE_PHYS)
MKUBOOTIMAGEARGS+=	-n "NetBSD/$(BOARDTYPE) ${_OSRELEASE}"
MKUBOOTIMAGEARGS_NONE=	${MKUBOOTIMAGEARGS} -C none
MKUBOOTIMAGEARGS_GZ=	${MKUBOOTIMAGEARGS} -C gz

SYSTEM_LD_TAIL_EXTRA+=; \
	echo ${OBJCOPY} -S -O binary $@ $@.bin; \
	${OBJCOPY} -S -O binary $@ $@.bin; \
	${TOOL_GZIP} -c $@.bin > $@.bin.gz; \
	echo ${TOOL_MKUBOOTIMAGE} ${MKUBOOTIMAGEARGS_GZ} $@.bin.gz $@.gz.ub; \
	${TOOL_MKUBOOTIMAGE} ${MKUBOOTIMAGEARGS_GZ} $@.bin.gz $@.gz.ub; \
	echo ${TOOL_MKUBOOTIMAGE} ${MKUBOOTIMAGEARGS_NONE} $@.bin $@.ub; \
	${TOOL_MKUBOOTIMAGE} ${MKUBOOTIMAGEARGS_NONE} $@.bin $@.ub; \
	echo ${TOOL_MKUBOOTIMAGE} ${MKUBOOTIMAGEARGS_NONE:C/((-a (0x)*)([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2}))/\1 -e \3\7\6\5\4/W} $@.bin $@-old.ub; \
	${TOOL_MKUBOOTIMAGE} ${MKUBOOTIMAGEARGS_NONE:C/((-a (0x)*)([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2}))/\1 -e \3\7\6\5\4/W} $@.bin $@-old.ub; \
	echo

EXTRA_KERNELS+= ${KERNELS:@.KERNEL.@${.KERNEL.}.bin@}
