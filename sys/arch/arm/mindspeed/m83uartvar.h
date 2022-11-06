/* $NetBSD: m83uartvar.h,v 1.6 2017/09/08 05:29:12 hkenken Exp $ */
/*
 * driver include for Freescale i.MX31 and i.MX31L UARTs
 */
/*
 * Copyright (c) 2009, 2010  Genetec Corporation.  All rights reserved.
 * Written by Hiroyuki Bessho for Genetec Corporation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY GENETEC CORPORATION ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL GENETEC CORPORATION
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef	_M83UARTVAR_H
#define	_M83UARTVAR_H


#include  <sys/cdefs.h>
#include  <sys/termios.h>	/* for tcflag_t */

struct m83uart_softc {
	device_t	sc_dev;

	int sc_unit;
	struct m83uart_regs {
		bus_space_tag_t		ur_iot;
		bus_space_handle_t	ur_ioh;
		bus_addr_t		ur_iobase;
#if 0
		bus_size_t		ur_nports;
		bus_size_t		ur_map[16];
#endif
	} sc_regs;

#define	sc_bt	sc_regs.ur_iot
#define	sc_bh	sc_regs.ur_ioh

	uint32_t		sc_intrspec_enb;
	uint32_t	sc_ucr2_d;	/* target value for UCR2 */
	uint32_t	sc_ucr[4];	/* cached value of UCRn */
#define	sc_ucr1	sc_ucr[0]
#define	sc_ucr2	sc_ucr[1]
#define	sc_ucr3	sc_ucr[2]
#define	sc_ucr4	sc_ucr[3]

	uint			sc_init_cnt;

	bus_addr_t		sc_addr;
	bus_size_t		sc_size;
	int			sc_intr;

	u_char	sc_hwflags;
/* Hardware flag masks */
#define	M83UART_HW_FLOW 	__BIT(0)
#define	M83UART_HW_DEV_OK	__BIT(1)
#define	M83UART_HW_CONSOLE	__BIT(2)
#define	M83UART_HW_KGDB 	__BIT(3)

	bool	enabled;

	u_char	sc_swflags;

	u_char sc_rx_flags;
#define	M83UART_RX_TTY_BLOCKED  	__BIT(0)
#define	M83UART_RX_TTY_OVERFLOWED	__BIT(1)
#define	M83UART_RX_IBUF_BLOCKED 	__BIT(2)
#define	M83UART_RX_IBUF_OVERFLOWED	__BIT(3)
#define	M83UART_RX_ANY_BLOCK					\
	(M83UART_RX_TTY_BLOCKED|M83UART_RX_TTY_OVERFLOWED| 	\
	    M83UART_RX_IBUF_BLOCKED|M83UART_RX_IBUF_OVERFLOWED)

	bool	sc_tx_busy, sc_tx_done, sc_tx_stopped;
	bool	sc_rx_ready,sc_st_check;
	u_short	sc_txfifo_len, sc_txfifo_thresh;

	uint16_t	*sc_rbuf;
	u_int		sc_rbuf_size;
	u_int		sc_rbuf_in;
	u_int		sc_rbuf_out;
#define	M83UART_RBUF_AVAIL(sc)					\
	((sc->sc_rbuf_out <= sc->sc_rbuf_in) ?			\
	(sc->sc_rbuf_in - sc->sc_rbuf_out) :			\
	(sc->sc_rbuf_size - (sc->sc_rbuf_out - sc->sc_rbuf_in)))

#define	M83UART_RBUF_SPACE(sc)	\
	((sc->sc_rbuf_in <= sc->sc_rbuf_out ?			    \
	    sc->sc_rbuf_size - (sc->sc_rbuf_out - sc->sc_rbuf_in) : \
	    sc->sc_rbuf_in - sc->sc_rbuf_out) - 1)
/* increment ringbuffer pointer */
#define	M83UART_RBUF_INC(sc,v,i)	(((v) + (i))&((sc->sc_rbuf_size)-1))
	u_int	sc_r_lowat;
	u_int	sc_r_hiwat;

	/* output chunk */
 	u_char *sc_tba;
 	u_int sc_tbc;
	u_int sc_heldtbc;
	/* pending parameter changes */
	u_char	sc_pending;
#define	M83UART_PEND_PARAM	__BIT(0)
#define	M83UART_PEND_SPEED	__BIT(1)


	struct callout sc_diag_callout;
	kmutex_t sc_lock;
	void *sc_ih;		/* interrupt handler */
	void *sc_si;		/* soft interrupt */
	struct tty		*sc_tty;

	/* power management hooks */
	int (*enable)(struct m83uart_softc *);
	void (*disable)(struct m83uart_softc *);

	struct {
		ulong err;
		ulong brk;
		ulong prerr;
		ulong frmerr;
		ulong ovrrun;
	}	sc_errors;

	struct m83uart_baudrate_ratio {
		uint16_t numerator;	/* UBIR */
		uint16_t modulator;	/* UBMR */
	} sc_ratio;

};

void m83uart_attach_common(device_t parent, device_t self,
    bus_space_tag_t, paddr_t, size_t, int, int);

int m83uart_kgdb_attach(bus_space_tag_t, paddr_t, u_int, tcflag_t);
int m83uart_cnattach(bus_space_tag_t, paddr_t, u_int, tcflag_t);

int m83uart_is_console(bus_space_tag_t, bus_addr_t, bus_space_handle_t *);

/*
 * Set platform dependent values
 */
void m83uart_set_frequency(u_int, u_int);

/*
 * defined in m8351uart.c and m8331uart.c
 */
int m83uart_match(device_t, cfdata_t, void *);
void m83uart_attach(device_t, device_t, void *);

void m83uart_attach_subr(struct m83uart_softc *);

int m83uintr(void *);

#endif	/* _M83UARTVAR_H */
