/* $NetBSD: m83uart.c,v 1.30 2022/10/26 23:38:06 riastradh Exp $ */

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

/*
 * derived from sys/dev/ic/com.c
 */

/*-
 * Copyright (c) 1998, 1999, 2004, 2008 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Charles M. Hannum.
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Copyright (c) 1991 The Regents of the University of California.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)com.c	7.5 (Berkeley) 5/16/91
 */

/*
 * driver for UART in i.MX SoC.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: m83uart.c,v 1.30 2022/10/26 23:38:06 riastradh Exp $");

#include "opt_m83uart.h"
#include "opt_ddb.h"
#include "opt_ddbparam.h"
#include "opt_kgdb.h"
#include "opt_lockdebug.h"
#include "opt_multiprocessor.h"
#include "opt_ntp.h"
#include "opt_m83uart.h"

#ifdef RND_COM
#include <sys/rndsource.h>
#endif

#ifndef	M83UART_TOLERANCE
#define	M83UART_TOLERANCE	30	/* baud rate tolerance, in 0.1% units */
#endif

#ifndef	M83UART_FREQDIV
#define	M83UART_FREQDIV		2	/* XXX */
#endif

#ifndef	M83UART_FREQ
#define	M83UART_FREQ	(56900000)
#endif

/*
 * Override cnmagic(9) macro before including <sys/systm.h>.
 * We need to know if cn_check_magic triggered debugger, so set a flag.
 * Callers of cn_check_magic must declare int cn_trapped = 0;
 * XXX: this is *ugly*!
 */
#define	cn_trap()				\
	do {					\
		console_debugger();		\
		cn_trapped = 1;			\
	} while (/* CONSTCOND */ 0)

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/poll.h>
#include <sys/tty.h>
#include <sys/proc.h>
#include <sys/conf.h>
#include <sys/file.h>
#include <sys/uio.h>
#include <sys/kernel.h>
#include <sys/syslog.h>
#include <sys/device.h>
#include <sys/kmem.h>
#include <sys/timepps.h>
#include <sys/vnode.h>
#include <sys/kauth.h>
#include <sys/intr.h>

#include <sys/bus.h>

#include <ddb/db_active.h>

#include <arm/mindspeed/m83uartreg.h>
#include <arm/mindspeed/m83uartvar.h>
#include <dev/cons.h>

#ifndef	M83UART_RING_SIZE
#define	M83UART_RING_SIZE	2048
#endif

void m83xxx_platform_early_putchar(char);

int	m83uspeed(long, struct m83uart_baudrate_ratio *);
int	m83uparam(struct tty *, struct termios *);
void	m83ustart(struct tty *);
int	m83uhwiflow(struct tty *, int);

void	m83uart_shutdown(struct m83uart_softc *);
void	m83uart_loadchannelregs(struct m83uart_softc *);
void	m83uart_hwiflow(struct m83uart_softc *);
void	m83uart_break(struct m83uart_softc *, bool);
void	m83uart_modem(struct m83uart_softc *, int);
void	tiocm_to_m83u(struct m83uart_softc *, u_long, int);
int	m83uart_to_tiocm(struct m83uart_softc *);
void	m83uart_iflush(struct m83uart_softc *);
int	m83uintr(void *);

int	m83uart_common_getc(dev_t, struct m83uart_regs *);
void	m83uart_common_putc(dev_t, struct m83uart_regs *, int);


int	m83uart_init(struct m83uart_regs *, int, tcflag_t, int);

int	m83ucngetc(dev_t);
void	m83ucnputc(dev_t, int);
void	m83ucnpollc(dev_t, int);

static void m83uintr_read(struct m83uart_softc *);
static void m83uintr_send(struct m83uart_softc *);

static void m83uart_enable_debugport(struct m83uart_softc *);
static void m83uart_disable_all_interrupts(struct m83uart_softc *);
static void m83uart_control_rxint(struct m83uart_softc *, bool);
static void m83uart_control_txint(struct m83uart_softc *, bool);
static u_int m83uart_txfifo_space(struct m83uart_softc *sc);

static	uint32_t	cflag_to_ucr2(tcflag_t, uint32_t);

#define	integrate	static inline
void 	m83usoft(void *);
integrate void m83uart_rxsoft(struct m83uart_softc *, struct tty *);
integrate void m83uart_txsoft(struct m83uart_softc *, struct tty *);
integrate void m83uart_stsoft(struct m83uart_softc *, struct tty *);
integrate void m83uart_schedrx(struct m83uart_softc *);
void	m83udiag(void *);
static void m83uart_load_speed(struct m83uart_softc *);
static void m83uart_load_params(struct m83uart_softc *);
integrate void m83uart_load_pendings(struct m83uart_softc *);


extern struct cfdriver m83uart_cd;

dev_type_open(m83uopen);
dev_type_close(m83uclose);
dev_type_read(m83uread);
dev_type_write(m83uwrite);
dev_type_ioctl(m83uioctl);
dev_type_stop(m83ustop);
dev_type_tty(m83utty);
dev_type_poll(m83upoll);

const struct cdevsw m83com_cdevsw = {
	.d_open = m83uopen,
	.d_close = m83uclose,
	.d_read = m83uread,
	.d_write = m83uwrite,
	.d_ioctl = m83uioctl,
	.d_stop = m83ustop,
	.d_tty = m83utty,
	.d_poll = m83upoll,
	.d_mmap = nommap,
	.d_kqfilter = ttykqfilter,
	.d_discard = nodiscard,
	.d_flag = D_TTY
};

/*
 * Make this an option variable one can patch.
 * But be warned:  this must be a power of 2!
 */
u_int m83uart_rbuf_size = M83UART_RING_SIZE;

/* Stop input when 3/4 of the ring is full; restart when only 1/4 is full. */
u_int m83uart_rbuf_hiwat = (M83UART_RING_SIZE * 1) / 4;
u_int m83uart_rbuf_lowat = (M83UART_RING_SIZE * 3) / 4;

static struct m83uart_regs m83uconsregs;
static int m83uconsattached;
static int m83uconsrate;
static tcflag_t m83uconscflag;
static struct cnm_state m83uart_cnm_state;

u_int m83uart_freq = M83UART_FREQ;
u_int m83uart_freqdiv = M83UART_FREQDIV;

#ifdef KGDB
#include <sys/kgdb.h>

static struct m83uart_regs m83u_kgdb_regs;
static int m83u_kgdb_attached;

int	m83uart_kgdb_getc(void *);
void	m83uart_kgdb_putc(void *, int);
#endif /* KGDB */

#define	M83UART_DIALOUT_MASK	TTDIALOUT_MASK

#define	M83UART_UNIT(x)		TTUNIT(x)
#define	M83UART_DIALOUT(x)	TTDIALOUT(x)

#define	M83UART_ISALIVE(sc)	((sc)->enabled != 0 && \
			 device_is_active((sc)->sc_dev))

#define	BR	BUS_SPACE_BARRIER_READ
#define	BW	BUS_SPACE_BARRIER_WRITE
#define	M83UART_BARRIER(r, f) \
	bus_space_barrier((r)->ur_iot, (r)->ur_ioh, 0, IMX_UART_SIZE, (f))


void
m83uart_attach_common(device_t parent, device_t self,
    bus_space_tag_t iot, paddr_t iobase, size_t size, int intr, int flags)
{
	struct m83uart_softc *sc = device_private(self);
	struct m83uart_regs *regsp = &sc->sc_regs;
	bus_space_handle_t ioh;

	aprint_naive("\n");
	aprint_normal("\n");

	sc->sc_dev = self;

	if (size <= 0)
		size = IMX_UART_SIZE;

	sc->sc_intr = intr;
	regsp->ur_iot = iot;
	regsp->ur_iobase = iobase;

	if (bus_space_map(iot, regsp->ur_iobase, size, 0, &ioh)) {
		return;
	}
	regsp->ur_ioh = ioh;

	sc->sc_ih = intr_establish(sc->sc_intr, IPL_SERIAL, IST_LEVEL,
	    m83uintr, sc);
	if (sc->sc_ih == NULL) {
		aprint_error_dev(sc->sc_dev, "intr_establish failed\n");
		return;
	}

	m83uart_attach_subr(sc);
}

void
m83uart_attach_subr(struct m83uart_softc *sc)
{
	struct m83uart_regs *regsp = &sc->sc_regs;
	bus_space_tag_t iot = regsp->ur_iot;
	bus_space_handle_t ioh = regsp->ur_ioh;
	struct tty *tp;

	callout_init(&sc->sc_diag_callout, 0);
	mutex_init(&sc->sc_lock, MUTEX_DEFAULT, IPL_HIGH);

	if (regsp->ur_iobase != m83uconsregs.ur_iobase)
		m83uart_init(&sc->sc_regs, TTYDEF_SPEED, TTYDEF_CFLAG, false);

	bus_space_read_region_4(iot, ioh, IMX_UCR1, sc->sc_ucr, 4);
	sc->sc_ucr2_d = sc->sc_ucr2;

	/* Disable interrupts before configuring the device. */
	m83uart_disable_all_interrupts(sc);

	if (regsp->ur_iobase == m83uconsregs.ur_iobase) {
		m83uconsattached = 1;

		/* Make sure the console is always "hardwired". */
#if 0
		delay(10000);			/* wait for output to finish */
#endif
		SET(sc->sc_hwflags, M83UART_HW_CONSOLE);
		SET(sc->sc_swflags, TIOCFLAG_SOFTCAR);
	}


	tp = tty_alloc();
	tp->t_oproc = m83ustart;
	tp->t_param = m83uparam;
	tp->t_hwiflow = m83uhwiflow;

	sc->sc_tty = tp;
	sc->sc_rbuf = kmem_alloc(sizeof (*sc->sc_rbuf) * m83uart_rbuf_size,
	    KM_SLEEP);
	sc->sc_rbuf_size = m83uart_rbuf_size;
	sc->sc_rbuf_in = sc->sc_rbuf_out = 0;
	sc->sc_txfifo_len = 32;
	sc->sc_txfifo_thresh = 16;	/* when USR1.TRDY, fifo has space
					 * for this many characters */

	tty_attach(tp);

	if (ISSET(sc->sc_hwflags, M83UART_HW_CONSOLE)) {
		int maj;

		/* locate the major number */
		maj = cdevsw_lookup_major(&m83com_cdevsw);

		if (maj != NODEVMAJOR) {
			tp->t_dev = cn_tab->cn_dev = makedev(maj,
			    device_unit(sc->sc_dev));

			aprint_normal_dev(sc->sc_dev, "console\n");
		}
	}

#ifdef KGDB
	/*
	 * Allow kgdb to "take over" this port.  If this is
	 * not the console and is the kgdb device, it has
	 * exclusive use.  If it's the console _and_ the
	 * kgdb device, it doesn't.
	 */
	if (regsp->ur_iobase == m83u_kgdb_regs.ur_iobase) {
		if (!ISSET(sc->sc_hwflags, M83UART_HW_CONSOLE)) {
			m83u_kgdb_attached = 1;

			SET(sc->sc_hwflags, M83UART_HW_KGDB);
		}
		aprint_normal_dev(sc->sc_dev, "kgdb\n");
	}
#endif

	sc->sc_si = softint_establish(SOFTINT_SERIAL, m83usoft, sc);

#ifdef RND_COM
	rnd_attach_source(&sc->rnd_source, device_xname(sc->sc_dev),
			  RND_TYPE_TTY, RND_FLAG_COLLECT_TIME |
					RND_FLAG_ESTIMATE_TIME);
#endif

	/* if there are no enable/disable functions, assume the device
	   is always enabled */
	if (!sc->enable)
		sc->enabled = 1;

	m83uart_enable_debugport(sc);

	SET(sc->sc_hwflags, M83UART_HW_DEV_OK);

	//shutdownhook_establish(m83uart_shutdownhook, sc);


#if 0
	{
		uint32_t reg;
		reg = bus_space_read_4(iot, ioh, IMX_UCR1);
		reg |= IMX_UCR1_TXDMAEN | IMX_UCR1_RXDMAEN;
		bus_space_write_4(iot, ioh, IMX_UCR1, reg);
	}
#endif
}

/*
 * baudrate = RefFreq / (16 * (UMBR + 1)/(UBIR + 1))
 *
 * (UBIR + 1) / (UBMR + 1) = (16 * BaurdRate) / RefFreq
 */

static long
gcd(long m, long n)
{

	if (m < n)
		return gcd(n, m);

	if (n <= 0)
		return m;
	return gcd(n, m % n);
}

int
m83uspeed(long speed, struct m83uart_baudrate_ratio *ratio)
{
#define	divrnd(n, q)	(((n)*2/(q)+1)/2)	/* divide and round off */
	long b = 16 * speed;
	long f = m83uart_freq / m83uart_freqdiv;
	long d;
	int err = 0;

	/* reduce b/f */
	while ((f > (1<<16) || b > (1<<16)) && (d = gcd(f, b)) > 1) {
		f /= d;
		b /= d;
	}


	while (f > (1<<16) || b > (1<<16)) {
		f /= 2;
		b /= 2;
	}
	if (f <= 0 || b <= 0)
		return -1;

#ifdef	DIAGNOSTIC
	err = divrnd(((uint64_t)m83uart_freq) * 1000 / m83uart_freqdiv,
		     (uint64_t)speed * 16 * f / b) - 1000;
	if (err < 0)
		err = -err;
#endif

	ratio->numerator = b-1;
	ratio->modulator = f-1;

	if (err > M83UART_TOLERANCE)
		return -1;

	return 0;
#undef	divrnd
}

#ifdef M83UART_DEBUG
int	m83uart_debug = 0;

void m83ustatus(struct m83uart_softc *, const char *);
void
m83ustatus(struct m83uart_softc *sc, const char *str)
{
	struct tty *tp = sc->sc_tty;

	aprint_normal_dev(sc->sc_dev,
	    "%s %cclocal  %cdcd %cts_carr_on %cdtr %ctx_stopped\n",
	    str,
	    ISSET(tp->t_cflag, CLOCAL) ? '+' : '-',
	    ISSET(sc->sc_msr, MSR_DCD) ? '+' : '-',
	    ISSET(tp->t_state, TS_CARR_ON) ? '+' : '-',
	    ISSET(sc->sc_mcr, MCR_DTR) ? '+' : '-',
	    sc->sc_tx_stopped ? '+' : '-');

	aprint_normal_dev(sc->sc_dev,
	    "%s %ccrtscts %ccts %cts_ttstop  %crts rx_flags=0x%x\n",
	    str,
	    ISSET(tp->t_cflag, CRTSCTS) ? '+' : '-',
	    ISSET(sc->sc_msr, MSR_CTS) ? '+' : '-',
	    ISSET(tp->t_state, TS_TTSTOP) ? '+' : '-',
	    ISSET(sc->sc_mcr, MCR_RTS) ? '+' : '-',
	    sc->sc_rx_flags);
}
#endif

#if 0
int
m83uart_detach(device_t self, int flags)
{
	struct m83uart_softc *sc = device_private(self);
	int maj, mn;

        if (ISSET(sc->sc_hwflags, M83UART_HW_CONSOLE))
		return EBUSY;

	/* locate the major number */
	maj = cdevsw_lookup_major(&m83com_cdevsw);

	/* Nuke the vnodes for any open instances. */
	mn = device_unit(self);
	vdevgone(maj, mn, mn, VCHR);

	mn |= M83UART_DIALOUT_MASK;
	vdevgone(maj, mn, mn, VCHR);

	if (sc->sc_rbuf == NULL) {
		/*
		 * Ring buffer allocation failed in the m83uart_attach_subr,
		 * only the tty is allocated, and nothing else.
		 */
		tty_free(sc->sc_tty);
		return 0;
	}

	/* Free the receive buffer. */
	kmem_free(sc->sc_rbuf, sizeof(*sc->sc_rbuf) * sc->sc_rbuf_size);

	/* Detach and free the tty. */
	tty_detach(sc->sc_tty);
	tty_free(sc->sc_tty);

	/* Unhook the soft interrupt handler. */
	softint_disestablish(sc->sc_si);

#ifdef RND_COM
	/* Unhook the entropy source. */
	rnd_detach_source(&sc->rnd_source);
#endif
	callout_destroy(&sc->sc_diag_callout);

	/* Destroy the lock. */
	mutex_destroy(&sc->sc_lock);

	return (0);
}
#endif

#ifdef notyet
int
m83uart_activate(device_t self, enum devact act)
{
	struct m83uart_softc *sc = device_private(self);
	int rv = 0;

	switch (act) {
	case DVACT_ACTIVATE:
		rv = EOPNOTSUPP;
		break;

	case DVACT_DEACTIVATE:
		if (sc->sc_hwflags & (M83UART_HW_CONSOLE|M83UART_HW_KGDB)) {
			rv = EBUSY;
			break;
		}

		if (sc->disable != NULL && sc->enabled != 0) {
			(*sc->disable)(sc);
			sc->enabled = 0;
		}
		break;
	}

	return (rv);
}
#endif

void
m83uart_shutdown(struct m83uart_softc *sc)
{
	struct tty *tp = sc->sc_tty;

	mutex_spin_enter(&sc->sc_lock);

	/* If we were asserting flow control, then deassert it. */
	SET(sc->sc_rx_flags, M83UART_RX_IBUF_BLOCKED);
	m83uart_hwiflow(sc);

	/* Clear any break condition set with TIOCSBRK. */
	m83uart_break(sc, false);

	/*
	 * Hang up if necessary.  Wait a bit, so the other side has time to
	 * notice even if we immediately open the port again.
	 * Avoid tsleeping above splhigh().
	 */
	if (ISSET(tp->t_cflag, HUPCL)) {
		m83uart_modem(sc, 0);
		mutex_spin_exit(&sc->sc_lock);
		/* XXX will only timeout */
		(void) kpause(ttclos, false, hz, NULL);
		mutex_spin_enter(&sc->sc_lock);
	}

	/* Turn off interrupts. */
	m83uart_disable_all_interrupts(sc);
	/* re-enable recv interrupt for console or kgdb port */
	m83uart_enable_debugport(sc);

	mutex_spin_exit(&sc->sc_lock);

#ifdef	notyet
	if (sc->disable) {
#ifdef DIAGNOSTIC
		if (!sc->enabled)
			panic("m83uart_shutdown: not enabled?");
#endif
		(*sc->disable)(sc);
		sc->enabled = 0;
	}
#endif
}

int
m83uopen(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct m83uart_softc *sc;
	struct tty *tp;
	int s;
	int error;

	sc = device_lookup_private(&m83uart_cd, M83UART_UNIT(dev));
	if (sc == NULL || !ISSET(sc->sc_hwflags, M83UART_HW_DEV_OK) ||
		sc->sc_rbuf == NULL)
		return (ENXIO);

	if (!device_is_active(sc->sc_dev))
		return (ENXIO);

#ifdef KGDB
	/*
	 * If this is the kgdb port, no other use is permitted.
	 */
	if (ISSET(sc->sc_hwflags, M83UART_HW_KGDB))
		return (EBUSY);
#endif

	tp = sc->sc_tty;

	if (kauth_authorize_device_tty(l->l_cred, KAUTH_DEVICE_TTY_OPEN, tp))
		return (EBUSY);

	s = spltty();

	/*
	 * Do the following iff this is a first open.
	 */
	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		struct termios t;

		tp->t_dev = dev;


#ifdef notyet
		if (sc->enable) {
			if ((*sc->enable)(sc)) {
				splx(s);
				aprint_error_dev(sc->sc_dev,
				    "device enable failed\n");
				return (EIO);
			}
			sc->enabled = 1;
		}
#endif

		mutex_spin_enter(&sc->sc_lock);

		m83uart_disable_all_interrupts(sc);

		/* Fetch the current modem control status, needed later. */

#ifdef	M83UART_PPS
		/* Clear PPS capture state on first open. */
		mutex_spin_enter(&timecounter_lock);
		memset(&sc->sc_pps_state, 0, sizeof(sc->sc_pps_state));
		sc->sc_pps_state.ppscap = PPS_CAPTUREASSERT | PPS_CAPTURECLEAR;
		pps_init(&sc->sc_pps_state);
		mutex_spin_exit(&timecounter_lock);
#endif

		mutex_spin_exit(&sc->sc_lock);

		/*
		 * Initialize the termios status to the defaults.  Add in the
		 * sticky bits from TIOCSFLAGS.
		 */
		if (ISSET(sc->sc_hwflags, M83UART_HW_CONSOLE)) {
			t.c_ospeed = m83uconsrate;
			t.c_cflag = m83uconscflag;
		} else {
			t.c_ospeed = TTYDEF_SPEED;
			t.c_cflag = TTYDEF_CFLAG;
		}
		t.c_ispeed = t.c_ospeed;
		if (ISSET(sc->sc_swflags, TIOCFLAG_CLOCAL))
			SET(t.c_cflag, CLOCAL);
		if (ISSET(sc->sc_swflags, TIOCFLAG_CRTSCTS))
			SET(t.c_cflag, CRTSCTS);
		if (ISSET(sc->sc_swflags, TIOCFLAG_MDMBUF))
			SET(t.c_cflag, MDMBUF);
		/* Make sure m83uparam() will do something. */
		tp->t_ospeed = 0;
		(void) m83uparam(tp, &t);
		tp->t_iflag = TTYDEF_IFLAG;
		tp->t_oflag = TTYDEF_OFLAG;
		tp->t_lflag = TTYDEF_LFLAG;
		ttychars(tp);
		ttsetwater(tp);

		mutex_spin_enter(&sc->sc_lock);

		/*
		 * Turn on DTR.  We must always do this, even if carrier is not
		 * present, because otherwise we'd have to use TIOCSDTR
		 * immediately after setting CLOCAL, which applications do not
		 * expect.  We always assert DTR while the device is open
		 * unless explicitly requested to deassert it.
		 */
		m83uart_modem(sc, 1);

		/* Clear the input ring, and unblock. */
		sc->sc_rbuf_in = sc->sc_rbuf_out = 0;
		m83uart_iflush(sc);
		CLR(sc->sc_rx_flags, M83UART_RX_ANY_BLOCK);
		m83uart_hwiflow(sc);

		/* Turn on interrupts. */
		m83uart_control_rxint(sc, true);

#ifdef M83UART_DEBUG
		if (m83uart_debug)
			m83ustatus(sc, "m83uopen  ");
#endif

		mutex_spin_exit(&sc->sc_lock);
	}

	splx(s);

#if 0
	error = ttyopen(tp, M83UART_DIALOUT(dev), ISSET(flag, O_NONBLOCK));
#else
	error = ttyopen(tp, 1, ISSET(flag, O_NONBLOCK));
#endif
	if (error)
		goto bad;

	error = (*tp->t_linesw->l_open)(dev, tp);
	if (error)
		goto bad;

	return (0);

bad:
	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		/*
		 * We failed to open the device, and nobody else had it opened.
		 * Clean up the state as appropriate.
		 */
		m83uart_shutdown(sc);
	}

	return (error);
}

int
m83uclose(dev_t dev, int flag, int mode, struct lwp *l)
{
	struct m83uart_softc *sc =
	    device_lookup_private(&m83uart_cd, M83UART_UNIT(dev));
	struct tty *tp = sc->sc_tty;

	/* XXX This is for cons.c. */
	if (!ISSET(tp->t_state, TS_ISOPEN))
		return (0);

	(*tp->t_linesw->l_close)(tp, flag);
	ttyclose(tp);

	if (M83UART_ISALIVE(sc) == 0)
		return (0);

	if (!ISSET(tp->t_state, TS_ISOPEN) && tp->t_wopen == 0) {
		/*
		 * Although we got a last close, the device may still be in
		 * use; e.g. if this was the dialout node, and there are still
		 * processes waiting for carrier on the non-dialout node.
		 */
		m83uart_shutdown(sc);
	}

	return (0);
}

int
m83uread(dev_t dev, struct uio *uio, int flag)
{
	struct m83uart_softc *sc =
	    device_lookup_private(&m83uart_cd, M83UART_UNIT(dev));
	struct tty *tp = sc->sc_tty;

	if (M83UART_ISALIVE(sc) == 0)
		return (EIO);

	return ((*tp->t_linesw->l_read)(tp, uio, flag));
}

int
m83uwrite(dev_t dev, struct uio *uio, int flag)
{
	struct m83uart_softc *sc =
	    device_lookup_private(&m83uart_cd, M83UART_UNIT(dev));
	struct tty *tp = sc->sc_tty;

	if (M83UART_ISALIVE(sc) == 0)
		return (EIO);

	return ((*tp->t_linesw->l_write)(tp, uio, flag));
}

int
m83upoll(dev_t dev, int events, struct lwp *l)
{
	struct m83uart_softc *sc =
	    device_lookup_private(&m83uart_cd, M83UART_UNIT(dev));
	struct tty *tp = sc->sc_tty;

	if (M83UART_ISALIVE(sc) == 0)
		return (POLLHUP);

	return ((*tp->t_linesw->l_poll)(tp, events, l));
}

struct tty *
m83utty(dev_t dev)
{
	struct m83uart_softc *sc =
	    device_lookup_private(&m83uart_cd, M83UART_UNIT(dev));
	struct tty *tp = sc->sc_tty;

	return (tp);
}

int
m83uioctl(dev_t dev, u_long cmd, void *data, int flag, struct lwp *l)
{
	struct m83uart_softc *sc;
	struct tty *tp;
	int error;

	sc = device_lookup_private(&m83uart_cd, M83UART_UNIT(dev));
	if (sc == NULL)
		return ENXIO;
	if (M83UART_ISALIVE(sc) == 0)
		return (EIO);

	tp = sc->sc_tty;

	error = (*tp->t_linesw->l_ioctl)(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return (error);

	error = ttioctl(tp, cmd, data, flag, l);
	if (error != EPASSTHROUGH)
		return (error);

	error = 0;
	switch (cmd) {
	case TIOCSFLAGS:
		error = kauth_authorize_device_tty(l->l_cred,
		    KAUTH_DEVICE_TTY_PRIVSET, tp);
		break;
	default:
		/* nothing */
		break;
	}
	if (error) {
		return error;
	}

	mutex_spin_enter(&sc->sc_lock);

	switch (cmd) {
	case TIOCSBRK:
		m83uart_break(sc, true);
		break;

	case TIOCCBRK:
		m83uart_break(sc, false);
		break;

	case TIOCSDTR:
		m83uart_modem(sc, 1);
		break;

	case TIOCCDTR:
		m83uart_modem(sc, 0);
		break;

	case TIOCGFLAGS:
		*(int *)data = sc->sc_swflags;
		break;

	case TIOCSFLAGS:
		sc->sc_swflags = *(int *)data;
		break;

	case TIOCMSET:
	case TIOCMBIS:
	case TIOCMBIC:
		tiocm_to_m83u(sc, cmd, *(int *)data);
		break;

	case TIOCMGET:
		*(int *)data = m83uart_to_tiocm(sc);
		break;

#ifdef notyet
	case PPS_IOC_CREATE:
	case PPS_IOC_DESTROY:
	case PPS_IOC_GETPARAMS:
	case PPS_IOC_SETPARAMS:
	case PPS_IOC_GETCAP:
	case PPS_IOC_FETCH:
#ifdef PPS_SYNC
	case PPS_IOC_KCBIND:
#endif
		mutex_spin_enter(&timecounter_lock);
		error = pps_ioctl(cmd, data, &sc->sc_pps_state);
		mutex_spin_exit(&timecounter_lock);
		break;

	case TIOCDCDTIMESTAMP:	/* XXX old, overloaded  API used by xntpd v3 */
		mutex_spin_enter(&timecounter_lock);
#ifndef PPS_TRAILING_EDGE
		TIMESPEC_TO_TIMEVAL((struct timeval *)data,
		    &sc->sc_pps_state.ppsinfo.assert_timestamp);
#else
		TIMESPEC_TO_TIMEVAL((struct timeval *)data,
		    &sc->sc_pps_state.ppsinfo.clear_timestamp);
#endif
		mutex_spin_exit(&timecounter_lock);
		break;
#endif

	default:
		error = EPASSTHROUGH;
		break;
	}

	mutex_spin_exit(&sc->sc_lock);

#ifdef M83UART_DEBUG
	if (m83uart_debug)
		m83ustatus(sc, "m83uioctl ");
#endif

	return (error);
}

integrate void
m83uart_schedrx(struct m83uart_softc *sc)
{
	sc->sc_rx_ready = 1;

	/* Wake up the poller. */
	softint_schedule(sc->sc_si);
}

void
m83uart_break(struct m83uart_softc *sc, bool onoff)
{
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;

	if (onoff)
		SET(sc->sc_ucr1, IMX_UCR1_SNDBRK);
	else
		CLR(sc->sc_ucr1, IMX_UCR1_SNDBRK);

	bus_space_write_4(iot, ioh, IMX_UCR1, sc->sc_ucr1);
}

void
m83uart_modem(struct m83uart_softc *sc, int onoff)
{
#ifdef notyet
	if (sc->sc_mcr_dtr == 0)
		return;

	if (onoff)
		SET(sc->sc_mcr, sc->sc_mcr_dtr);
	else
		CLR(sc->sc_mcr, sc->sc_mcr_dtr);

	if (!sc->sc_heldchange) {
		if (sc->sc_tx_busy) {
			sc->sc_heldtbc = sc->sc_tbc;
			sc->sc_tbc = 0;
			sc->sc_heldchange = 1;
		} else
			m83uart_loadchannelregs(sc);
	}
#endif
}

/*
 * RTS output is controlled by UCR2.CTS bit.
 * DTR output is controlled by UCR3.DSR bit.
 * (i.MX reference manual uses names in DCE mode)
 *
 * note: if UCR2.CTSC == 1 for automatic HW flow control, UCR2.CTS is ignored.
 */
void
tiocm_to_m83u(struct m83uart_softc *sc, u_long how, int ttybits)
{
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;

	uint32_t ucr2 = sc->sc_ucr2_d;
	uint32_t ucr3 = sc->sc_ucr3;

	uint32_t ucr2_mask = 0;
	uint32_t ucr3_mask = 0;


	if (ISSET(ttybits, TIOCM_DTR))
		ucr3_mask = IMX_UCR3_DSR;
	if (ISSET(ttybits, TIOCM_RTS))
		ucr2_mask = IMX_UCR2_CTS;

	switch (how) {
	case TIOCMBIC:
		CLR(ucr2, ucr2_mask);
		CLR(ucr3, ucr3_mask);
		break;

	case TIOCMBIS:
		SET(ucr2, ucr2_mask);
		SET(ucr3, ucr3_mask);
		break;

	case TIOCMSET:
		CLR(ucr2, ucr2_mask);
		CLR(ucr3, ucr3_mask);
		SET(ucr2, ucr2_mask);
		SET(ucr3, ucr3_mask);
		break;
	}

	if (ucr3 != sc->sc_ucr3) {
		bus_space_write_4(iot, ioh, IMX_UCR3, ucr3);
		sc->sc_ucr3 = ucr3;
	}

	if (ucr2 == sc->sc_ucr2_d)
		return;

	sc->sc_ucr2_d = ucr2;
	/* update CTS bit only */
	ucr2 = (sc->sc_ucr2 & ~IMX_UCR2_CTS) |
	    (ucr2 & IMX_UCR2_CTS);

	bus_space_write_4(iot, ioh, IMX_UCR2, ucr2);
	sc->sc_ucr2 = ucr2;
}

int
m83uart_to_tiocm(struct m83uart_softc *sc)
{
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;
	int ttybits = 0;
	uint32_t usr[2];

	if (ISSET(sc->sc_ucr3, IMX_UCR3_DSR))
		SET(ttybits, TIOCM_DTR);
	if (ISSET(sc->sc_ucr2, IMX_UCR2_CTS))
		SET(ttybits, TIOCM_RTS);

	bus_space_read_region_4(iot, ioh, IMX_USR1, usr, 2);

	if (ISSET(usr[0], IMX_USR1_RTSS))
		SET(ttybits, TIOCM_CTS);

	if (ISSET(usr[1], IMX_USR2_DCDIN))
		SET(ttybits, TIOCM_CD);

#if 0
	/* XXXbsh: I couldn't find the way to read ipp_uart_dsr_dte_i signal,
	   although there are bits in UART registers to detect delta of DSR.
	*/
	if (ISSET(m83ubits, MSR_DSR))
		SET(ttybits, TIOCM_DSR);
#endif

	if (ISSET(usr[1], IMX_USR2_RIIN))
		SET(ttybits, TIOCM_RI);


#ifdef	notyet
	if (ISSET(sc->sc_ier, IER_ERXRDY | IER_ETXRDY | IER_ERLS | IER_EMSC))
		SET(ttybits, TIOCM_LE);
#endif

	return (ttybits);
}

static uint32_t
cflag_to_ucr2(tcflag_t cflag, uint32_t oldval)
{
	uint32_t val = oldval;

	CLR(val,IMX_UCR2_WS|IMX_UCR2_PREN|IMX_UCR2_PROE|IMX_UCR2_STPB);

	switch (cflag & CSIZE) {
	case CS5:
	case CS6:
		/* not suppreted. use 7-bits */
	case CS7:
		break;
	case CS8:
		SET(val, IMX_UCR2_WS);
		break;
	}


	if (ISSET(cflag, PARENB)) {
		SET(val, IMX_UCR2_PREN);

		/* odd parity */
		if (!ISSET(cflag, PARODD))
			SET(val, IMX_UCR2_PROE);
	}

	if (ISSET(cflag, CSTOPB))
		SET(val, IMX_UCR2_STPB);

	val |= IMX_UCR2_TXEN| IMX_UCR2_RXEN|IMX_UCR2_SRST;

	return val;
}

int
m83uparam(struct tty *tp, struct termios *t)
{
	struct m83uart_softc *sc =
	    device_lookup_private(&m83uart_cd, M83UART_UNIT(tp->t_dev));
	struct m83uart_baudrate_ratio ratio;
	uint32_t ucr2;
	bool change_speed = tp->t_ospeed != t->c_ospeed;

	if (M83UART_ISALIVE(sc) == 0)
		return (EIO);

	/* Check requested parameters. */
	if (t->c_ispeed && t->c_ispeed != t->c_ospeed)
		return (EINVAL);

	/*
	 * For the console, always force CLOCAL and !HUPCL, so that the port
	 * is always active.
	 */
	if (ISSET(sc->sc_swflags, TIOCFLAG_SOFTCAR) ||
	    ISSET(sc->sc_hwflags, M83UART_HW_CONSOLE)) {
		SET(t->c_cflag, CLOCAL);
		CLR(t->c_cflag, HUPCL);
	}

	/*
	 * If there were no changes, don't do anything.  This avoids dropping
	 * input and improves performance when all we did was frob things like
	 * VMIN and VTIME.
	 */
	if ( !change_speed && tp->t_cflag == t->c_cflag)
		return (0);

	if (change_speed) {
		/* calculate baudrate modulator value */
		if (m83uspeed(t->c_ospeed, &ratio) < 0)
			return (EINVAL);
		sc->sc_ratio = ratio;
	}

	ucr2 = cflag_to_ucr2(t->c_cflag, sc->sc_ucr2_d);

	mutex_spin_enter(&sc->sc_lock);

#if 0	/* flow control stuff.  not yet */
	/*
	 * If we're not in a mode that assumes a connection is present, then
	 * ignore carrier changes.
	 */
	if (ISSET(t->c_cflag, CLOCAL | MDMBUF))
		sc->sc_msr_dcd = 0;
	else
		sc->sc_msr_dcd = MSR_DCD;
	/*
	 * Set the flow control pins depending on the current flow control
	 * mode.
	 */
	if (ISSET(t->c_cflag, CRTSCTS)) {
		sc->sc_mcr_dtr = MCR_DTR;
		sc->sc_mcr_rts = MCR_RTS;
		sc->sc_msr_cts = MSR_CTS;
		sc->sc_efr = EFR_AUTORTS | EFR_AUTOCTS;
	} else if (ISSET(t->c_cflag, MDMBUF)) {
		/*
		 * For DTR/DCD flow control, make sure we don't toggle DTR for
		 * carrier detection.
		 */
		sc->sc_mcr_dtr = 0;
		sc->sc_mcr_rts = MCR_DTR;
		sc->sc_msr_cts = MSR_DCD;
		sc->sc_efr = 0;
	} else {
		/*
		 * If no flow control, then always set RTS.  This will make
		 * the other side happy if it mistakenly thinks we're doing
		 * RTS/CTS flow control.
		 */
		sc->sc_mcr_dtr = MCR_DTR | MCR_RTS;
		sc->sc_mcr_rts = 0;
		sc->sc_msr_cts = 0;
		sc->sc_efr = 0;
		if (ISSET(sc->sc_mcr, MCR_DTR))
			SET(sc->sc_mcr, MCR_RTS);
		else
			CLR(sc->sc_mcr, MCR_RTS);
	}
	sc->sc_msr_mask = sc->sc_msr_cts | sc->sc_msr_dcd;
#endif

	/* And copy to tty. */
	tp->t_ispeed = t->c_ospeed;
	tp->t_ospeed = t->c_ospeed;
	tp->t_cflag = t->c_cflag;

	if (!change_speed && ucr2 == sc->sc_ucr2_d) {
		/* noop */
	}
	else if (!sc->sc_pending && !sc->sc_tx_busy) {
		if (ucr2 != sc->sc_ucr2_d) {
			sc->sc_ucr2_d = ucr2;
			m83uart_load_params(sc);
		}
		if (change_speed)
			m83uart_load_speed(sc);
	}
	else {
		if (!sc->sc_pending) {
			sc->sc_heldtbc = sc->sc_tbc;
			sc->sc_tbc = 0;
		}
		sc->sc_pending |=
		    (ucr2 == sc->sc_ucr2_d ? 0 : M83UART_PEND_PARAM) |
		    (change_speed ? 0 : M83UART_PEND_SPEED);
		sc->sc_ucr2_d = ucr2;
	}

	if (!ISSET(t->c_cflag, CHWFLOW)) {
		/* Disable the high water mark. */
		sc->sc_r_hiwat = 0;
		sc->sc_r_lowat = 0;
		if (ISSET(sc->sc_rx_flags, M83UART_RX_TTY_OVERFLOWED)) {
			CLR(sc->sc_rx_flags, M83UART_RX_TTY_OVERFLOWED);
			m83uart_schedrx(sc);
		}
		if (ISSET(sc->sc_rx_flags,
			M83UART_RX_TTY_BLOCKED|M83UART_RX_IBUF_BLOCKED)) {
			CLR(sc->sc_rx_flags,
			    M83UART_RX_TTY_BLOCKED|M83UART_RX_IBUF_BLOCKED);
			m83uart_hwiflow(sc);
		}
	} else {
		sc->sc_r_hiwat = m83uart_rbuf_hiwat;
		sc->sc_r_lowat = m83uart_rbuf_lowat;
	}

	mutex_spin_exit(&sc->sc_lock);

#if 0
	/*
	 * Update the tty layer's idea of the carrier bit, in case we changed
	 * CLOCAL or MDMBUF.  We don't hang up here; we only do that by
	 * explicit request.
	 */
	(void) (*tp->t_linesw->l_modem)(tp, ISSET(sc->sc_msr, MSR_DCD));
#else
	/* XXX: always report that we have DCD */
	(void) (*tp->t_linesw->l_modem)(tp, 1);
#endif

#ifdef M83UART_DEBUG
	if (m83uart_debug)
		m83ustatus(sc, "m83uparam ");
#endif

	if (!ISSET(t->c_cflag, CHWFLOW)) {
		if (sc->sc_tx_stopped) {
			sc->sc_tx_stopped = 0;
			m83ustart(tp);
		}
	}

	return (0);
}

void
m83uart_iflush(struct m83uart_softc *sc)
{
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;
#ifdef DIAGNOSTIC
	uint32_t reg = 0xffff;
#endif
	int timo;

	timo = 50000;
	/* flush any pending I/O */
	while (ISSET(bus_space_read_4(iot, ioh, IMX_USR2), IMX_USR2_RDR)
	    && --timo)
#ifdef DIAGNOSTIC
		reg =
#else
		    (void)
#endif
		    bus_space_read_4(iot, ioh, IMX_URXD);
#ifdef DIAGNOSTIC
	if (!timo)
		aprint_error_dev(sc->sc_dev, "m83uart_iflush timeout %02x\n", reg);
#endif
}

int
m83uhwiflow(struct tty *tp, int block)
{
	struct m83uart_softc *sc =
	    device_lookup_private(&m83uart_cd, M83UART_UNIT(tp->t_dev));

	if (M83UART_ISALIVE(sc) == 0)
		return (0);

#ifdef notyet
	if (sc->sc_mcr_rts == 0)
		return (0);
#endif

	mutex_spin_enter(&sc->sc_lock);

	if (block) {
		if (!ISSET(sc->sc_rx_flags, M83UART_RX_TTY_BLOCKED)) {
			SET(sc->sc_rx_flags, M83UART_RX_TTY_BLOCKED);
			m83uart_hwiflow(sc);
		}
	} else {
		if (ISSET(sc->sc_rx_flags, M83UART_RX_TTY_OVERFLOWED)) {
			CLR(sc->sc_rx_flags, M83UART_RX_TTY_OVERFLOWED);
			m83uart_schedrx(sc);
		}
		if (ISSET(sc->sc_rx_flags, M83UART_RX_TTY_BLOCKED)) {
			CLR(sc->sc_rx_flags, M83UART_RX_TTY_BLOCKED);
			m83uart_hwiflow(sc);
		}
	}

	mutex_spin_exit(&sc->sc_lock);
	return (1);
}

/*
 * (un)block input via hw flowcontrol
 */
void
m83uart_hwiflow(struct m83uart_softc *sc)
{
#ifdef notyet
	struct m83uart_regs *regsp= &sc->sc_regs;

	if (sc->sc_mcr_rts == 0)
		return;

	if (ISSET(sc->sc_rx_flags, RX_ANY_BLOCK)) {
		CLR(sc->sc_mcr, sc->sc_mcr_rts);
		CLR(sc->sc_mcr_active, sc->sc_mcr_rts);
	} else {
		SET(sc->sc_mcr, sc->sc_mcr_rts);
		SET(sc->sc_mcr_active, sc->sc_mcr_rts);
	}
	UR_WRITE_1(regsp, M83UART_REG_MCR, sc->sc_mcr_active);
#endif
}


void
m83ustart(struct tty *tp)
{
	struct m83uart_softc *sc =
	    device_lookup_private(&m83uart_cd, M83UART_UNIT(tp->t_dev));
	int s;
	u_char *tba;
	int tbc;
	u_int n;
	u_int space;
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;

	if (M83UART_ISALIVE(sc) == 0)
		return;

	s = spltty();
	if (ISSET(tp->t_state, TS_BUSY | TS_TIMEOUT | TS_TTSTOP))
		goto out;
	if (sc->sc_tx_stopped)
		goto out;
	if (!ttypull(tp))
		goto out;

	/* Grab the first contiguous region of buffer space. */
	tba = tp->t_outq.c_cf;
	tbc = ndqb(&tp->t_outq, 0);

	mutex_spin_enter(&sc->sc_lock);

	sc->sc_tba = tba;
	sc->sc_tbc = tbc;

	SET(tp->t_state, TS_BUSY);
	sc->sc_tx_busy = 1;

	space = m83uart_txfifo_space(sc);
	n = MIN(sc->sc_tbc, space);

	if (n > 0) {
		bus_space_write_multi_1(iot, ioh, IMX_UTXD, sc->sc_tba, n);
		sc->sc_tbc -= n;
		sc->sc_tba += n;
	}

	/* Enable transmit completion interrupts */
	m83uart_control_txint(sc, true);

	mutex_spin_exit(&sc->sc_lock);
out:
	splx(s);
	return;
}

/*
 * Stop output on a line.
 */
void
m83ustop(struct tty *tp, int flag)
{
	struct m83uart_softc *sc =
	    device_lookup_private(&m83uart_cd, M83UART_UNIT(tp->t_dev));

	mutex_spin_enter(&sc->sc_lock);
	if (ISSET(tp->t_state, TS_BUSY)) {
		/* Stop transmitting at the next chunk. */
		sc->sc_tbc = 0;
		sc->sc_heldtbc = 0;
		if (!ISSET(tp->t_state, TS_TTSTOP))
			SET(tp->t_state, TS_FLUSH);
	}
	mutex_spin_exit(&sc->sc_lock);
}

void
m83udiag(void *arg)
{
#ifdef notyet
	struct m83uart_softc *sc = arg;
	int overflows, floods;

	mutex_spin_enter(&sc->sc_lock);
	overflows = sc->sc_overflows;
	sc->sc_overflows = 0;
	floods = sc->sc_floods;
	sc->sc_floods = 0;
	sc->sc_errors = 0;
	mutex_spin_exit(&sc->sc_lock);

	log(LOG_WARNING, "%s: %d silo overflow%s, %d ibuf flood%s\n",
	    device_xname(sc->sc_dev),
	    overflows, overflows == 1 ? "" : "s",
	    floods, floods == 1 ? "" : "s");
#endif
}

integrate void
m83uart_rxsoft(struct m83uart_softc *sc, struct tty *tp)
{
	int (*rint)(int, struct tty *) = tp->t_linesw->l_rint;
	u_int cc, scc, outp;
	uint16_t data;
	u_int code;

	scc = cc = M83UART_RBUF_AVAIL(sc);

#if 0
	if (cc == m83uart_rbuf_size-1) {
		sc->sc_floods++;
		if (sc->sc_errors++ == 0)
			callout_reset(&sc->sc_diag_callout, 60 * hz,
			    m83udiag, sc);
	}
#endif

	/* If not yet open, drop the entire buffer content here */
	if (!ISSET(tp->t_state, TS_ISOPEN)) {
		sc->sc_rbuf_out = sc->sc_rbuf_in;
		cc = 0;
	}

	outp = sc->sc_rbuf_out;

#define	ERRBITS (IMX_URXD_PRERR|IMX_URXD_BRK|IMX_URXD_FRMERR|IMX_URXD_OVRRUN)

	while (cc) {
	        data = sc->sc_rbuf[outp];
		code = data & IMX_URXD_RX_DATA;
		if (ISSET(data, ERRBITS)) {
			if (sc->sc_errors.err == 0)
				callout_reset(&sc->sc_diag_callout,
				    60 * hz, m83udiag, sc);
			if (ISSET(data, IMX_URXD_OVRRUN))
				sc->sc_errors.ovrrun++;
			if (ISSET(data, IMX_URXD_BRK)) {
				sc->sc_errors.brk++;
				SET(code, TTY_FE);
			}
			if (ISSET(data, IMX_URXD_FRMERR)) {
				sc->sc_errors.frmerr++;
				SET(code, TTY_FE);
			}
			if (ISSET(data, IMX_URXD_PRERR)) {
				sc->sc_errors.prerr++;
				SET(code, TTY_PE);
			}
		}
		if ((*rint)(code, tp) == -1) {
			/*
			 * The line discipline's buffer is out of space.
			 */
			if (!ISSET(sc->sc_rx_flags, M83UART_RX_TTY_BLOCKED)) {
				/*
				 * We're either not using flow control, or the
				 * line discipline didn't tell us to block for
				 * some reason.  Either way, we have no way to
				 * know when there's more space available, so
				 * just drop the rest of the data.
				 */
				sc->sc_rbuf_out = sc->sc_rbuf_in;
				cc = 0;
			} else {
				/*
				 * Don't schedule any more receive processing
				 * until the line discipline tells us there's
				 * space available (through m83uhwiflow()).
				 * Leave the rest of the data in the input
				 * buffer.
				 */
				SET(sc->sc_rx_flags, M83UART_RX_TTY_OVERFLOWED);
			}
			break;
		}
		outp = M83UART_RBUF_INC(sc, outp, 1);
		cc--;
	}

	if (cc != scc) {
		sc->sc_rbuf_out = outp;
		mutex_spin_enter(&sc->sc_lock);

		cc = M83UART_RBUF_SPACE(sc);

		/* Buffers should be ok again, release possible block. */
		if (cc >= sc->sc_r_lowat) {
			if (ISSET(sc->sc_rx_flags, M83UART_RX_IBUF_OVERFLOWED)) {
				CLR(sc->sc_rx_flags, M83UART_RX_IBUF_OVERFLOWED);
				m83uart_control_rxint(sc, true);
			}
			if (ISSET(sc->sc_rx_flags, M83UART_RX_IBUF_BLOCKED)) {
				CLR(sc->sc_rx_flags, M83UART_RX_IBUF_BLOCKED);
				m83uart_hwiflow(sc);
			}
		}
		mutex_spin_exit(&sc->sc_lock);
	}
}

integrate void
m83uart_txsoft(struct m83uart_softc *sc, struct tty *tp)
{

	CLR(tp->t_state, TS_BUSY);
	if (ISSET(tp->t_state, TS_FLUSH))
		CLR(tp->t_state, TS_FLUSH);
	else
		ndflush(&tp->t_outq, (int)(sc->sc_tba - tp->t_outq.c_cf));
	(*tp->t_linesw->l_start)(tp);
}

integrate void
m83uart_stsoft(struct m83uart_softc *sc, struct tty *tp)
{
#ifdef notyet
	u_char msr, delta;

	mutex_spin_enter(&sc->sc_lock);
	msr = sc->sc_msr;
	delta = sc->sc_msr_delta;
	sc->sc_msr_delta = 0;
	mutex_spin_exit(&sc->sc_lock);

	if (ISSET(delta, sc->sc_msr_dcd)) {
		/*
		 * Inform the tty layer that carrier detect changed.
		 */
		(void) (*tp->t_linesw->l_modem)(tp, ISSET(msr, MSR_DCD));
	}

	if (ISSET(delta, sc->sc_msr_cts)) {
		/* Block or unblock output according to flow control. */
		if (ISSET(msr, sc->sc_msr_cts)) {
			sc->sc_tx_stopped = 0;
			(*tp->t_linesw->l_start)(tp);
		} else {
			sc->sc_tx_stopped = 1;
		}
	}

#endif
#ifdef M83UART_DEBUG
	if (m83uart_debug)
		m83ustatus(sc, "m83uart_stsoft");
#endif
}

void
m83usoft(void *arg)
{
	struct m83uart_softc *sc = arg;
	struct tty *tp;

	if (M83UART_ISALIVE(sc) == 0)
		return;

	tp = sc->sc_tty;

	if (sc->sc_rx_ready) {
		sc->sc_rx_ready = 0;
		m83uart_rxsoft(sc, tp);
	}

	if (sc->sc_st_check) {
		sc->sc_st_check = 0;
		m83uart_stsoft(sc, tp);
	}

	if (sc->sc_tx_done) {
		sc->sc_tx_done = 0;
		m83uart_txsoft(sc, tp);
	}
}

int
m83uintr(void *arg)
{
	struct m83uart_softc *sc = arg;
	uint32_t usr1, usr2;
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;


	if (M83UART_ISALIVE(sc) == 0)
		return (0);

	mutex_spin_enter(&sc->sc_lock);

	usr2 = bus_space_read_4(iot, ioh, IMX_USR2);


	do {
		bus_space_write_4(iot, ioh, IMX_USR2,
		    usr2 & (IMX_USR2_BRCD|IMX_USR2_ORE));
		if (usr2 & IMX_USR2_BRCD) {
			/* Break signal detected */
			int cn_trapped = 0;

			cn_check_magic(sc->sc_tty->t_dev,
				       CNC_BREAK, m83uart_cnm_state);
			if (cn_trapped)
				goto next;
#if defined(KGDB) && !defined(DDB)
			if (ISSET(sc->sc_hwflags, M83UART_HW_KGDB)) {
				kgdb_connect(1);
				goto next;
			}
#endif
		}

		if (usr2 & IMX_USR2_RDR)
			m83uintr_read(sc);

#ifdef	M83UART_PPS
		{
			u_char	msr, delta;

			msr = CSR_READ_1(regsp, M83UART_REG_MSR);
			delta = msr ^ sc->sc_msr;
			sc->sc_msr = msr;
			if ((sc->sc_pps_state.ppsparam.mode & PPS_CAPTUREBOTH) &&
			    (delta & MSR_DCD)) {
				mutex_spin_enter(&timecounter_lock);
				pps_capture(&sc->sc_pps_state);
				pps_event(&sc->sc_pps_state,
				    (msr & MSR_DCD) ?
				    PPS_CAPTUREASSERT :
				    PPS_CAPTURECLEAR);
				mutex_spin_exit(&timecounter_lock);
			}
		}
#endif

#ifdef notyet
		/*
		 * Process normal status changes
		 */
		if (ISSET(delta, sc->sc_msr_mask)) {
			SET(sc->sc_msr_delta, delta);

			/*
			 * Stop output immediately if we lose the output
			 * flow control signal or carrier detect.
			 */
			if (ISSET(~msr, sc->sc_msr_mask)) {
				sc->sc_tbc = 0;
				sc->sc_heldtbc = 0;
#ifdef M83UART_DEBUG
				if (m83uart_debug)
					m83ustatus(sc, "m83uintr  ");
#endif
			}

			sc->sc_st_check = 1;
		}
#endif

next:
		usr2 = bus_space_read_4(iot, ioh, IMX_USR2);
	} while (usr2 & (IMX_USR2_RDR|IMX_USR2_BRCD));

	usr1 = bus_space_read_4(iot, ioh, IMX_USR1);
	if (usr1 & IMX_USR1_TRDY)
		m83uintr_send(sc);

	mutex_spin_exit(&sc->sc_lock);

	/* Wake up the poller. */
	softint_schedule(sc->sc_si);

#ifdef RND_COM
	rnd_add_uint32(&sc->rnd_source, iir | lsr);
#endif

	return (1);
}


/*
 * called when there is least one character in rxfifo
 *
 */

static void
m83uintr_read(struct m83uart_softc *sc)
{
	int cc;
	uint16_t rd;
	uint32_t usr2;
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;

	cc = M83UART_RBUF_SPACE(sc);

	/* clear aging timer interrupt */
	bus_space_write_4(iot, ioh, IMX_USR1, IMX_USR1_AGTIM);

	while (cc > 0) {
		int cn_trapped = 0;

		sc->sc_rbuf[sc->sc_rbuf_in] = rd =
		    bus_space_read_4(iot, ioh, IMX_URXD);

		cn_check_magic(sc->sc_tty->t_dev,
		    rd & 0xff, m83uart_cnm_state);

		if (!cn_trapped) {
#if defined(DDB) && defined(DDB_KEYCODE)
			/*
			 * Temporary hack so that I can force the kernel into
			 * the debugger via the serial port
			 */
			if ((rd & 0xff) == DDB_KEYCODE)
				Debugger();
#endif
			sc->sc_rbuf_in = M83UART_RBUF_INC(sc, sc->sc_rbuf_in, 1);
			cc--;
		}

		usr2 = bus_space_read_4(iot, ioh, IMX_USR2);
		if (!(usr2 & IMX_USR2_RDR))
			break;
	}

	/*
	 * Current string of incoming characters ended because
	 * no more data was available or we ran out of space.
	 * Schedule a receive event if any data was received.
	 * If we're out of space, turn off receive interrupts.
	 */
	if (!ISSET(sc->sc_rx_flags, M83UART_RX_TTY_OVERFLOWED))
		sc->sc_rx_ready = 1;
	/*
	 * See if we are in danger of overflowing a buffer. If
	 * so, use hardware flow control to ease the pressure.
	 */
	if (!ISSET(sc->sc_rx_flags, M83UART_RX_IBUF_BLOCKED) &&
	    cc < sc->sc_r_hiwat) {
		sc->sc_rx_flags |= M83UART_RX_IBUF_BLOCKED;
		m83uart_hwiflow(sc);
	}

	/*
	 * If we're out of space, disable receive interrupts
	 * until the queue has drained a bit.
	 */
	if (!cc) {
		sc->sc_rx_flags |= M83UART_RX_IBUF_OVERFLOWED;
		m83uart_control_rxint(sc, false);
	}
}



/*
 * find how many chars we can put into tx-fifo
 */
static u_int
m83uart_txfifo_space(struct m83uart_softc *sc)
{
	uint32_t usr1, usr2;
	u_int cc;
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;

	usr2 = bus_space_read_4(iot, ioh, IMX_USR2);
	if (usr2 & IMX_USR2_TXFE)
		cc = sc->sc_txfifo_len;
	else {
		usr1 = bus_space_read_4(iot, ioh, IMX_USR1);
		if (usr1 & IMX_USR1_TRDY)
			cc = sc->sc_txfifo_thresh;
		else
			cc = 0;
	}

	return cc;
}

void
m83uintr_send(struct m83uart_softc *sc)
{
	uint32_t usr2;
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;
	int cc = 0;

	usr2 = bus_space_read_4(iot, ioh, IMX_USR2);

	if (sc->sc_pending) {
		if (usr2 & IMX_USR2_TXFE) {
			m83uart_load_pendings(sc);
			sc->sc_tbc = sc->sc_heldtbc;
			sc->sc_heldtbc = 0;
		}
		else {
			/* wait for TX fifo empty */
			m83uart_control_txint(sc, true);
			return;
		}
	}

	cc = m83uart_txfifo_space(sc);
	cc = MIN(cc, sc->sc_tbc);

	if (cc > 0) {
		bus_space_write_multi_1(iot, ioh, IMX_UTXD, sc->sc_tba, cc);
		sc->sc_tbc -= cc;
		sc->sc_tba += cc;
	}

	if (sc->sc_tbc > 0)
		m83uart_control_txint(sc, true);
	else {
		/* no more chars to send.
		   we don't need tx interrupt any more. */
		m83uart_control_txint(sc, false);
		if (sc->sc_tx_busy) {
			sc->sc_tx_busy = 0;
			sc->sc_tx_done = 1;
		}
	}
}

static void
m83uart_disable_all_interrupts(struct m83uart_softc *sc)
{
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;

	sc->sc_ucr1 &= ~M83UART_INTRS_UCR1;
	sc->sc_ucr2 &= ~M83UART_INTRS_UCR2;
	sc->sc_ucr3 &= ~M83UART_INTRS_UCR3;
	sc->sc_ucr4 &= ~M83UART_INTRS_UCR4;


	bus_space_write_region_4(iot, ioh, IMX_UCR1, sc->sc_ucr, 4);
}

static void
m83uart_control_rxint(struct m83uart_softc *sc, bool enable)
{
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;
	uint32_t ucr1, ucr2;

	ucr1 = sc->sc_ucr1;
	ucr2 = sc->sc_ucr2;

	if (enable) {
		ucr1 |= IMX_UCR1_RRDYEN;
		ucr2 |= IMX_UCR2_ATEN;
	}
	else {
		ucr1 &= ~IMX_UCR1_RRDYEN;
		ucr2 &= ~IMX_UCR2_ATEN;
	}

	if (ucr1 != sc->sc_ucr1 || ucr2 != sc->sc_ucr2) {
		sc->sc_ucr1 = ucr1;
		sc->sc_ucr2 = ucr2;
		bus_space_write_region_4(iot, ioh, IMX_UCR1, sc->sc_ucr, 2);
	}
}

static void
m83uart_control_txint(struct m83uart_softc *sc, bool enable)
{
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;
	uint32_t ucr1;
	uint32_t mask;

	/* if parameter change is pending, get interrupt when Tx fifo
	   is completely empty.  otherwise, get interrupt when txfifo
	   has less characters than threshold */
	mask = sc->sc_pending ? IMX_UCR1_TXMPTYEN : IMX_UCR1_TRDYEN;

	ucr1 = sc->sc_ucr1;

	CLR(ucr1, IMX_UCR1_TXMPTYEN|IMX_UCR1_TRDYEN);
	if (enable)
		SET(ucr1, mask);

	if (ucr1 != sc->sc_ucr1) {
		bus_space_write_4(iot, ioh, IMX_UCR1, ucr1);
		sc->sc_ucr1 = ucr1;
	}
}


static void
m83uart_load_params(struct m83uart_softc *sc)
{
	uint32_t ucr2;
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;

	ucr2 = (sc->sc_ucr2_d & ~IMX_UCR2_ATEN) |
	    (sc->sc_ucr2 & IMX_UCR2_ATEN);

	bus_space_write_4(iot, ioh, IMX_UCR2, ucr2);
	sc->sc_ucr2 = ucr2;
}

static void
m83uart_load_speed(struct m83uart_softc *sc)
{
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;
	int n, rfdiv, ufcr;

#ifdef notyet
	/*
	 * Set the FIFO threshold based on the receive speed.
	 *
	 *  * If it's a low speed, it's probably a mouse or some other
	 *    interactive device, so set the threshold low.
	 *  * If it's a high speed, trim the trigger level down to prevent
	 *    overflows.
	 *  * Otherwise set it a bit higher.
	 */
	if (t->c_ospeed <= 1200)
		sc->sc_fifo = FIFO_ENABLE | FIFO_TRIGGER_1;
	else if (t->c_ospeed <= 38400)
		sc->sc_fifo = FIFO_ENABLE | FIFO_TRIGGER_8;
	else
		sc->sc_fifo = FIFO_ENABLE | FIFO_TRIGGER_4;
#endif

	n = 32 - sc->sc_txfifo_thresh;
	n = MAX(2, n);

	rfdiv = IMX_UFCR_DIVIDER_TO_RFDIV(m83uart_freqdiv);

	ufcr = (n << IMX_UFCR_TXTL_SHIFT) |
		(rfdiv << IMX_UFCR_RFDIV_SHIFT) |
		(16 << IMX_UFCR_RXTL_SHIFT);

	/* keep DCE/DTE bit */
	ufcr |= bus_space_read_4(iot, ioh, IMX_UFCR) & IMX_UFCR_DCEDTE;

	bus_space_write_4(iot, ioh, IMX_UFCR, ufcr);

	/* UBIR must updated before UBMR */
	bus_space_write_4(iot, ioh,
	    IMX_UBIR, sc->sc_ratio.numerator);
	bus_space_write_4(iot, ioh,
	    IMX_UBMR, sc->sc_ratio.modulator);


}


static void
m83uart_load_pendings(struct m83uart_softc *sc)
{
	if (sc->sc_pending & M83UART_PEND_PARAM)
		m83uart_load_params(sc);
	if (sc->sc_pending & M83UART_PEND_SPEED)
		m83uart_load_speed(sc);
	sc->sc_pending = 0;
}

/*
 * The following functions are polled getc and putc routines, shared
 * by the console and kgdb glue.
 *
 * The read-ahead code is so that you can detect pending in-band
 * cn_magic in polled mode while doing output rather than having to
 * wait until the kernel decides it needs input.
 */

#define	READAHEAD_RING_LEN	16
static int m83uart_readahead[READAHEAD_RING_LEN];
static int m83uart_readahead_in = 0;
static int m83uart_readahead_out = 0;
#define	READAHEAD_IS_EMPTY()	(m83uart_readahead_in==m83uart_readahead_out)
#define	READAHEAD_IS_FULL()	\
	(((m83uart_readahead_in+1) & (READAHEAD_RING_LEN-1)) ==m83uart_readahead_out)

int
m83uart_common_getc(dev_t dev, struct m83uart_regs *regsp)
{
	int s = splserial();
	u_char c;
	bus_space_tag_t iot = regsp->ur_iot;
	bus_space_handle_t ioh = regsp->ur_ioh;
	uint32_t usr2;

	/* got a character from reading things earlier */
	if (!READAHEAD_IS_EMPTY()) {
		c = m83uart_readahead[m83uart_readahead_out];
		m83uart_readahead_out = (m83uart_readahead_out + 1) &
		    (READAHEAD_RING_LEN-1);
		splx(s);
		return (c);
	}

	/* block until a character becomes available */
	while (!((usr2 = bus_space_read_4(iot, ioh, IMX_USR2)) & IMX_USR2_RDR))
		continue;

	c = 0xff & bus_space_read_4(iot, ioh, IMX_URXD);

	{
		int cn_trapped __unused = 0;
		if (!db_active)
			cn_check_magic(dev, c, m83uart_cnm_state);
	}
	splx(s);
	return (c);
}

void
m83uart_common_putc(dev_t dev, struct m83uart_regs *regsp, int c)
{
	int s = splserial();
	int timo;
	bus_space_tag_t iot = regsp->ur_iot;
	bus_space_handle_t ioh = regsp->ur_ioh;
#if 0
	int cin, timo;
	bus_space_tag_t iot = regsp->ur_iot;
	bus_space_handle_t ioh = regsp->ur_ioh;
	uint32_t usr2;

	if (!READAHEAD_IS_FULL() &&
	    ((usr2 = bus_space_read_4(iot, ioh, IMX_USR2)) & IMX_USR2_RDR)) {

		int __attribute__((__unused__))cn_trapped = 0;
		cin = bus_space_read_4(iot, ioh, IMX_URXD);
		cn_check_magic(dev, cin & 0xff, m83uart_cnm_state);
		m83uart_readahead[m83uart_readahead_in] = cin & 0xff;
		m83uart_readahead_in = (m83uart_readahead_in + 1) &
		    (READAHEAD_RING_LEN-1);
	}

	/* wait for any pending transmission to finish */
	timo = 150000;
	do {
		if (bus_space_read_4(iot, ioh, IMX_USR1) & IMX_USR1_TRDY) {
			bus_space_write_4(iot, ioh, IMX_UTXD, c);
			break;
		}
	} while(--timo > 0);
#endif
	/* wait for any pending transmission to finish */
	timo = 150000;
	do {
		if (bus_space_read_4(iot, ioh, 5*4) & (1 << 6)) {
			bus_space_write_4(iot, ioh, 0, c);
			break;
		}
	} while(--timo > 0);

	M83UART_BARRIER(regsp, BR | BW);

	splx(s);
}
/*
 * Initialize UART
 */
int
m83uart_init(struct m83uart_regs *regsp, int rate, tcflag_t cflag, int domap)
{
/*
	struct m83uart_baudrate_ratio ratio;
	int rfdiv = IMX_UFCR_DIVIDER_TO_RFDIV(m83uart_freqdiv);
	uint32_t ufcr;
*/
	int error;

	if (domap && (error = bus_space_map(regsp->ur_iot, regsp->ur_iobase,
	     IMX_UART_SIZE, 0, &regsp->ur_ioh)) != 0)
		return error;
#if 0
	if (m83uart_freq != 0) {
		if (m83uspeed(rate, &ratio) < 0)
			return EINVAL;

		/* UBIR must updated before UBMR */
		bus_space_write_4(regsp->ur_iot, regsp->ur_ioh,
		    IMX_UBIR, ratio.numerator);
		bus_space_write_4(regsp->ur_iot, regsp->ur_ioh,
		    IMX_UBMR, ratio.modulator);
	}

	/* XXX: DTREN, DPEC */
	bus_space_write_4(regsp->ur_iot, regsp->ur_ioh, IMX_UCR3,
	    IMX_UCR3_DSR|IMX_UCR3_RXDMUXSEL);

	ufcr = bus_space_read_4(regsp->ur_iot, regsp->ur_ioh, IMX_UFCR);
	ufcr &= ~IMX_UFCR_TXTL;
	ufcr |= (8 << IMX_UFCR_TXTL_SHIFT);
	ufcr &= ~IMX_UFCR_RXTL;
	ufcr |= (1 << IMX_UFCR_RXTL_SHIFT);
	if (m83uart_freq != 0) {
		ufcr &= ~IMX_UFCR_RFDIV;
		ufcr |= (rfdiv << IMX_UFCR_RFDIV_SHIFT);
	}
	bus_space_write_4(regsp->ur_iot, regsp->ur_ioh, IMX_UFCR, ufcr);

	if (m83uart_freq != 0) {
		bus_space_write_4(regsp->ur_iot, regsp->ur_ioh, IMX_ONEMS,
		    m83uart_freq / m83uart_freqdiv / 1000);
	}

	bus_space_write_4(regsp->ur_iot, regsp->ur_ioh, IMX_UCR2,
			  IMX_UCR2_IRTS|
			  IMX_UCR2_CTSC|
			  IMX_UCR2_WS|IMX_UCR2_TXEN|
			  IMX_UCR2_RXEN|IMX_UCR2_SRST);
	/* clear status registers */
	bus_space_write_4(regsp->ur_iot, regsp->ur_ioh, IMX_USR1, 0xffff);
	bus_space_write_4(regsp->ur_iot, regsp->ur_ioh, IMX_USR2, 0xffff);


	bus_space_write_4(regsp->ur_iot, regsp->ur_ioh, IMX_UCR1,
	    IMX_UCR1_UARTEN);

#endif
	return (0);
}


/*
 * Following are all routines needed for UART to act as console
 */
struct consdev m83ucons = {
	NULL, NULL, m83ucngetc, m83ucnputc, m83ucnpollc, NULL, NULL, NULL,
	NODEV, CN_NORMAL
};


int
m83uart_cnattach(bus_space_tag_t iot, paddr_t iobase, u_int rate,
    tcflag_t cflag)
{
	struct m83uart_regs regs;
	int res;

	regs.ur_iot = iot;
	regs.ur_iobase = iobase;

	res = m83uart_init(&regs, rate, cflag, true);
	if (res)
		return (res);

	cn_tab = &m83ucons;
	cn_init_magic(&m83uart_cnm_state);
	cn_set_magic("\047\001"); /* default magic is BREAK */

	m83uconsrate = rate;
	m83uconscflag = cflag;

	m83uconsregs = regs;

	return 0;
}

int
m83ucngetc(dev_t dev)
{
	return (m83uart_common_getc(dev, &m83uconsregs));
}

/*
 * Console kernel output character routine.
 */
void
m83ucnputc(dev_t dev, int c)
{
	m83uart_common_putc(dev, &m83uconsregs, c);
}

void
m83ucnpollc(dev_t dev, int on)
{

	m83uart_readahead_in = 0;
	m83uart_readahead_out = 0;
}

#ifdef KGDB
int
m83uart_kgdb_attach(bus_space_tag_t iot, paddr_t iobase, u_int rate,
    tcflag_t cflag)
{
	int res;

	if (iot == m83uconsregs.ur_iot &&
	    iobase == m83uconsregs.ur_iobase) {
#if !defined(DDB)
		return (EBUSY); /* cannot share with console */
#else
		m83u_kgdb_regs.ur_iot = iot;
		m83u_kgdb_regs.ur_ioh = m83uconsregs.ur_ioh;
		m83u_kgdb_regs.ur_iobase = iobase;
#endif
	} else {
		m83u_kgdb_regs.ur_iot = iot;
		m83u_kgdb_regs.ur_iobase = iobase;

		res = m83uart_init(&m83u_kgdb_regs, rate, cflag, true);
		if (res)
			return (res);

		/*
		 * XXXfvdl this shouldn't be needed, but the cn_magic goo
		 * expects this to be initialized
		 */
		cn_init_magic(&m83uart_cnm_state);
		cn_set_magic("\047\001");
	}

	kgdb_attach(m83uart_kgdb_getc, m83uart_kgdb_putc, &m83u_kgdb_regs);
	kgdb_dev = 123; /* unneeded, only to satisfy some tests */

	return (0);
}

/* ARGSUSED */
int
m83uart_kgdb_getc(void *arg)
{
	struct m83uart_regs *regs = arg;

	return (m83uart_common_getc(NODEV, regs));
}

/* ARGSUSED */
void
m83uart_kgdb_putc(void *arg, int c)
{
	struct m83uart_regs *regs = arg;

	m83uart_common_putc(NODEV, regs, c);
}
#endif /* KGDB */

/* helper function to identify the m83u ports used by
 console or KGDB (and not yet autoconf attached) */
int
m83uart_is_console(bus_space_tag_t iot, bus_addr_t iobase, bus_space_handle_t *ioh)
{
	bus_space_handle_t help;

	if (!m83uconsattached &&
	    iot == m83uconsregs.ur_iot && iobase == m83uconsregs.ur_iobase)
		help = m83uconsregs.ur_ioh;
#ifdef KGDB
	else if (!m83u_kgdb_attached &&
	    iot == m83u_kgdb_regs.ur_iot && iobase == m83u_kgdb_regs.ur_iobase)
		help = m83u_kgdb_regs.ur_ioh;
#endif
	else
		return (0);

	if (ioh)
		*ioh = help;
	return (1);
}

#ifdef notyet

bool
m83uart_cleanup(device_t self, int how)
{
/*
 * this routine exists to serve as a shutdown hook for systems that
 * have firmware which doesn't interact properly with a m83uart device in
 * FIFO mode.
 */
	struct m83uart_softc *sc = device_private(self);

	if (ISSET(sc->sc_hwflags, M83UART_HW_FIFO))
		UR_WRITE_1(&sc->sc_regs, M83UART_REG_FIFO, 0);

	return true;
}
#endif

#ifdef notyet
bool
m83uart_suspend(device_t self PMF_FN_ARGS)
{
	struct m83uart_softc *sc = device_private(self);

	UR_WRITE_1(&sc->sc_regs, M83UART_REG_IER, 0);
	(void)CSR_READ_1(&sc->sc_regs, M83UART_REG_IIR);

	return true;
}
#endif

#ifdef notyet
bool
m83uart_resume(device_t self PMF_FN_ARGS)
{
	struct m83uart_softc *sc = device_private(self);

	mutex_spin_enter(&sc->sc_lock);
	m83uart_loadchannelregs(sc);
	mutex_spin_exit(&sc->sc_lock);

	return true;
}
#endif

static void
m83uart_enable_debugport(struct m83uart_softc *sc)
{
	bus_space_tag_t iot = sc->sc_regs.ur_iot;
	bus_space_handle_t ioh = sc->sc_regs.ur_ioh;

	if (sc->sc_hwflags & (M83UART_HW_CONSOLE|M83UART_HW_KGDB)) {

		/* Turn on line break interrupt, set carrier. */

		sc->sc_ucr3 |= IMX_UCR3_DSR;
		bus_space_write_4(iot, ioh, IMX_UCR3, sc->sc_ucr3);

		sc->sc_ucr4 |= IMX_UCR4_BKEN;
		bus_space_write_4(iot, ioh, IMX_UCR4, sc->sc_ucr4);

		sc->sc_ucr2 |= IMX_UCR2_TXEN|IMX_UCR2_RXEN|
		    IMX_UCR2_CTS;
		bus_space_write_4(iot, ioh, IMX_UCR2, sc->sc_ucr2);

		sc->sc_ucr1 |= IMX_UCR1_UARTEN;
		bus_space_write_4(iot, ioh, IMX_UCR1, sc->sc_ucr1);
	}
}


void
m83uart_set_frequency(u_int freq, u_int div)
{
	m83uart_freq = freq;
	m83uart_freqdiv = div;
}
