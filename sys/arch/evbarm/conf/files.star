#	$NetBSD$
#
# Star Semiconductor STR81xx, STR91xx boards configuration info
#

file	arch/evbarm/star/star_machdep.c

# CPU support and integrated peripherals
include	"arch/arm/star/files.star"

attach	star at mainbus
