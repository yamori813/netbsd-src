#	$NetBSD$
#
# standard NetBSD/evbarm for STR81xx/STR91xx

machine	evbarm arm
include 	"conf/std"		# MI standard options
include		"arch/arm/conf/std.arm"	# arch standard options

# Pull in STR81xx, STR91xx config definitions.
include 	"arch/evbarm/conf/files.star"

options 	EXEC_ELF32
options 	EXEC_SCRIPT

# To support easy transit to ../arch/arm/arm32
options 	ARM32
options 	ARM9
#options 	__HAVE_FAST_SOFTINTS		# should be in types.h
options 	__HAVE_PCI_CONF_HOOK		# should be in types.h
makeoptions	BOARDTYPE="star"
options		EVBARM_BOARDTYPE=star

options 	KERNEL_BASE_EXT=0xc0000000
makeoptions	BOARDMKFRAG="${THISARM}/conf/mk.star"
options 	ARM_INTR_IMPL="<arch/arm/star/star_intr.h>"
