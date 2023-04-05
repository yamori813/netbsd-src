/*	$NetBSD$	*/

/*
 * Copyright 2003 Wasabi Systems, Inc.
 * All rights reserved.
 *
 * Written by Steve C. Woodford for Wasabi Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *      This product includes software developed for the NetBSD Project by
 *      Wasabi Systems, Inc.
 * 4. The name of Wasabi Systems, Inc. may not be used to endorse
 *    or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY WASABI SYSTEMS, INC. ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL WASABI SYSTEMS, INC
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _IXP425_GPIO_H
#define _IXP425_GPIO_H

/*
 * GPIO pin function query/manipulation functions
 */
extern u_int ixp425_gpio_get_function(u_int);
extern u_int ixp425_gpio_set_function(u_int, u_int);
extern int ixp425_gpio_get_bit(u_int);
extern void ixp425_gpio_set_bit(u_int);
extern void ixp425_gpio_clear_bit(u_int);
extern void ixp425_gpio_set_dir(u_int, int);
extern void ixp425_gpio_clear_intr(u_int);

/*
 * Establish/Disestablish interrupt handlers for GPIO pins
 */
extern void *ixp425_gpio_intr_establish(u_int, int, int,
		int (*)(void *), void *);
extern void ixp425_gpio_intr_disestablish(void *);
extern void ixp425_gpio_intr_mask(void *);
extern void ixp425_gpio_intr_unmask(void *);
extern void ixp425_gpio_set_intr_level(u_int, int);


struct ixp425_gpioconf {
	int pin;
	u_int value;
};
void ixp425_gpio_config(struct ixp425_gpioconf **);

extern struct ixp425_gpioconf ixp425_com_ffuart_gpioconf[];
extern struct ixp425_gpioconf ixp425_com_stuart_gpioconf[];
extern struct ixp425_gpioconf ixp425_com_btuart_gpioconf[];
extern struct ixp425_gpioconf ixp425_com_hwuart_gpioconf[];
extern struct ixp425_gpioconf ixp425_i2c_gpioconf[];
extern struct ixp425_gpioconf ixp425_i2s_gpioconf[];
extern struct ixp425_gpioconf ixp425_pcic_gpioconf[];
extern struct ixp425_gpioconf ixp425_pxaacu_gpioconf[];
extern struct ixp425_gpioconf ixp425_pxamci_gpioconf[];

#define GPIO_GPOUTR			0x00
#define GPIO_GPOER			0x04
#define GPIO_GPINR			0x08
#define GPIO_GPISR			0x0C
#define GPIO_GPIT1R			0x10
#define GPIO_GPIT2R			0x14
#define GPIO_GPCLKR			0x18
#define GPIO_GPDBSELR			0x1C

#define GPIO_STYLE_ACTIVE_HIGH		0x0
#define GPIO_STYLE_ACTIVE_LOW		0x1
#define GPIO_STYLE_RISING_EDGE		0x2
#define GPIO_STYLE_FALLING_EDGE		0x3
#define GPIO_STYLE_TRANSITIONAL		0x4

#define GPIO_STYLE_MASK			0x7

#endif /* _IXP425_GPIO_H */
