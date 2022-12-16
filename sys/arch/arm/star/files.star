#       $NetBSD$
#
# Configuration info for Star STR8100, STR9100
#

include "arch/arm/pic/files.pic"
device	star { [addr=-1], [irq=-1] }: bus_space_generic, pic, pic_splfuncs
file	arch/arm/star/star.c			star
file	arch/arm/star/star_intr.c		star

device  cfi : norbus
attach  cfi at mainbus with mainbus_cfi
file    arch/arm/mainbus/mainbus_cfi.c	mainbus_cfi

# STR8100 Equuleus Family
file	arch/arm/star/star_equuleus_intr.c	star & star_equuleus

# STR9100 Orion Family
file	arch/arm/star/star_orion_intr.c		star & star_orion

file	arch/arm/arm32/irq_dispatch.S
file	arch/arm/star/star_space.c
file	arch/arm/star/star_dma.c

# UART
attach	com at star with staruart
file	arch/arm/star/star_com.c		staruart needs-flag
file	arch/arm/star/star_a4x_space.c		staruart
file	arch/arm/star/star_a4x_io.S 		staruart

# Timer
device	starclk
attach	starclk at star
file	arch/arm/star/startimer.c		starclk

# Real Time Counter
device	starrtc
attach	starrtc at star
file	arch/arm/star/starrtc.c			starrtc

# Watch Dog Timer
device	starwdog: sysmon_wdog
attach	starwdog at star
file	arch/arm/star/starwdog.c		starwdog

# GPIO
device	stargpio: gpiobus
attach	stargpio at star
file	arch/arm/star/stargpio.c		stargpio

# SPI (STR8100 only)
device	starspi: spibus
attach	starspi at star
file	arch/arm/star/starspi.c			starspi

# TWI (I2C) (STR8100 only)
device	startwi: i2cbus
attach	startwi at star
file	arch/arm/star/startwi.c			startwi

# Gigabit Ethernet Controller (STR8100 only)
device	gec: ether, ifnet, arp, mii
attach	gec at star
file	arch/arm/star/if_gec.c			gec

# Gigabit Switch Engine (STR9100 only)
device	gsec { }
attach	gsec at star
device	gse: ether, ifnet, arp, mii
attach	gse at gsec
file	arch/arm/star/if_gse.c			gsec | gse

# UHCI USB1.1 Controller
attach	ohci at star with starohci
file	arch/arm/star/starohci.c		starohci

# EHCI USB2.0 Controller
attach	ehci at star with starehci
file	arch/arm/star/starehci.c		starehci

# PCI Interface
device	starpci: pcibus
attach	starpci at star
file	arch/arm/star/star_pci.c		starpci
file	arch/arm/star/star_pci_space.c		starpci

device	pchb
attach	pchb at pci
file	arch/arm/star/starpchb.c		pchb
