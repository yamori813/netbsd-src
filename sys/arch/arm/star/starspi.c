//#error SPI driver didn't work...
/*	$NetBSD$	*/

/*-
 * Copyright (c) 2009 SHIMIZU Ryo <ryo@nerv.org>
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
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/device.h>
#include <sys/errno.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

#include <dev/spi/spiflash.h>
#include <dev/spi/spivar.h>

struct starspi_softc {
	device_t sc_dev;
	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	void *sc_ih;

	int sc_slave;
	struct spi_controller sc_spi;
	struct spi_transq sc_spi_transq;
	struct {
		struct spi_transfer *transfer;
		struct spi_chunk *chunk;
	} sc_spictx;
};

static int starspi_match(device_t, struct cfdata *, void *);
static void starspi_attach(device_t, device_t, void *);

/* for spi_controller */
static int starspi_configure(void *, int, int, int);
static int starspi_transfer(void *, struct spi_transfer *);

static void starspi_init(struct starspi_softc *);
static int starspi_intr(void *);
static int starspi_next(struct starspi_softc *sc);
static unsigned char starspi_chunkgetc(struct starspi_softc *);
static int  starspi_chunkputc(struct starspi_softc *, uint8_t);
static int starspi_send(struct starspi_softc *);
static int starspi_recv(struct starspi_softc *);
static int starspi_start(struct starspi_softc *);

#undef USE_FIFO

#define	TX_FIFO_WRITABLE(sc)					\
	((bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh,		\
	    EQUULEUS_SPI_FIFO_TXCFG_REG) &			\
	    EQUULEUS_SPI_FIFO_TXCFG_TXFF_STATMASK) <= 4)

#define	TX_WRITABLE(sc)						\
	(bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh,		\
	    EQUULEUS_SPI_INTSTAT_REG) & EQUULEUS_SPI_INTSTAT_TXBUF_FG)

#define	RX_FIFO_READABLE(sc)					\
	((bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh,		\
	    EQUULEUS_SPI_FIFO_RXCFG_REG) &			\
	    EQUULEUS_SPI_FIFO_RXCFG_RXFF_STATMASK) != 0)

#define	RX_READABLE(sc)						\
	(bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh,		\
	    EQUULEUS_SPI_INTSTAT_REG) & EQUULEUS_SPI_INTSTAT_RXBUF_FG)

#define	SPI_BUSY(sc)						\
	(bus_space_read_4((sc)->sc_iot, (sc)->sc_ioh,		\
	    EQUULEUS_SPI_STAT_REG) & EQUULEUS_SPI_STAT_BUSY)


CFATTACH_DECL_NEW(starspi, sizeof(struct starspi_softc),
    starspi_match, starspi_attach, NULL, NULL);


/* ARGSUSED */
static int
starspi_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa;

	if (!CPU_IS_STR8100())
		return 0;

	sa = aux;
	sa->sa_size = EQUULEUS_SPI_REGSIZE;

	return 1;
}

/* ARGSUSED */
static void
starspi_attach(device_t parent __unused, device_t self, void *aux)
{
	struct starspi_softc *sc;
	struct star_attach_args *sa;
	struct spibus_attach_args sba;

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_addr = sa->sa_addr;

	aprint_normal(": Serial Peripheral Interface\n");
	aprint_naive("\n");

	if (bus_space_map(sc->sc_iot, sc->sc_addr, sa->sa_size, 0,
	    &sc->sc_ioh)) {
		aprint_error(": can't map registers\n");
		return;
	}

	starspi_init(sc);

	sc->sc_ih = intr_establish(STAR_IRQ_SPI, IPL_SERIAL,
	    IST_LEVEL_LOW, starspi_intr, sc);

	/*
	 * Initialize spi controller and queue
	 */
	sc->sc_spi.sct_cookie = sc;
	sc->sc_spi.sct_configure = starspi_configure;
	sc->sc_spi.sct_transfer = starspi_transfer;
	sc->sc_spi.sct_nslaves = 4;
	spi_transq_init(&sc->sc_spi_transq);

	sba.sba_controller = &sc->sc_spi;
	config_found_ia(sc->sc_dev, "spibus", &sba, spibus_print);
}

static void
starspi_init(struct starspi_softc *sc)
{
	uint32_t val;

	val = STAR_REG_READ32(EQUULEUS_CLKPWR_PLLCTRL_REG);
	val &= ~EQUULEUS_CLKPWR_PLLCTRL_PLLx7_PWD;
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_PLLCTRL_REG, val);

	val = STAR_REG_READ32(EQUULEUS_MISC_CHIP_CONFIG_REG);
	STAR_REG_WRITE32(EQUULEUS_MISC_CHIP_CONFIG_REG, val &
	    ~EQUULEUS_MISC_CHIP_CONFIG_SPISERIALFLASH);

//	val = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKGATE0_REG);
//	STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKGATE0_REG, val |
//	    EQUULEUS_CLKPWR_CLKGATE0_PCLK_SMC |
//	    EQUULEUS_CLKPWR_CLKGATE0_HCLK_SMC);

	/* SPI/PCM/I2S/TWI Controler Enable */
	val = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKGATE1_REG);
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKGATE1_REG, val |
	    EQUULEUS_CLKPWR_CLKGATE1_PCLK_GPIO |
	    EQUULEUS_CLKPWR_CLKGATE1_PCLK_P2S |
//	    EQUULEUS_CLKPWR_CLKGATE1_HCLK_SPI |
	    0);

	/* activate SPI/PCM/I2S/TWI */
	val = STAR_REG_READ32(EQUULEUS_CLKPWR_SOFTRST_REG);
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_SOFTRST_REG, val |
	     EQUULEUS_CLKPWR_SOFTRST_P2S);

	/* GPIO_A PIN ENABLE */
	val = STAR_REG_READ32(EQUULEUS_MISC_GPIOA_PIN_REG);
	STAR_REG_WRITE32(EQUULEUS_MISC_GPIOA_PIN_REG, val |
	    EQUULEUS_MISC_GPIOA31_SPICS3 |
	    EQUULEUS_MISC_GPIOA30_SPICS2 |
	    EQUULEUS_MISC_GPIOA29_SPICS1 |
	    EQUULEUS_MISC_GPIOA28_SPICS0 |
	    EQUULEUS_MISC_GPIOA27_SPICLK |
	    EQUULEUS_MISC_GPIOA26_SPIDR);

	/* configuration */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_CFG_REG,
//	    EQUULEUS_SPI_CFG_EN |
	    EQUULEUS_SPI_CFG_MASTER_EN |
#ifdef USE_FIFO
	    EQUULEUS_SPI_CFG_FFEN |
#endif
	    EQUULEUS_SPI_CFG_CHAR_LEN(0));

	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_SPI_TX_DATA_REG, 0);

	/* TX FIFO Threashold and Interrupt delay */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_FIFO_TXCFG_REG,
	    EQUULEUS_SPI_FIFO_TXCFG_TXFF_THREAD_2);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_FIFO_TXCTRL_REG,
	    EQUULEUS_SPI_FIFO_TXCTRL_TXFF_DLY(0));

	/* RX FIFO Threashold */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_FIFO_RXCFG_REG,
	    EQUULEUS_SPI_FIFO_RXCFG_RXFF_THREAD_2);

	/* clear RX */
#ifdef USE_FIFO
	while (RX_FIFO_READABLE(sc))
		bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_RX_DATA_REG);
#else
	while (RX_READABLE(sc))
		bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_RX_DATA_REG);
#endif

	/* Interrupt enable mask */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_INT_ENABLE_REG,
#ifdef USE_FIFO
//	    EQUULEUS_SPI_INT_TXFFERR |
//	    EQUULEUS_SPI_INT_RXFFERR |
////	    EQUULEUS_SPI_INT_TXFF |
//	    EQUULEUS_SPI_INT_RXFF |
#else
//	    EQUULEUS_SPI_INT_TXBFERR |
//	    EQUULEUS_SPI_INT_RXBFERR |
////	    EQUULEUS_SPI_INT_TXBF |
//	    EQUULEUS_SPI_INT_RXBF |
#endif
	    0);

	/* clear interrupt status */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_INTSTAT_REG,
	    EQUULEUS_SPI_INTSTAT_TXBF_UNRN_FG |
	    EQUULEUS_SPI_INTSTAT_RXBF_OVRN_FG |
	    EQUULEUS_SPI_INTSTAT_TXFF_UNRN_FG |
	    EQUULEUS_SPI_INTSTAT_RXFF_OVRN_FG);

	/* configuration */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_CFG_REG,
	    EQUULEUS_SPI_CFG_EN |
	    EQUULEUS_SPI_CFG_MASTER_EN |
#ifdef USE_FIFO
	    EQUULEUS_SPI_CFG_FFEN |
#endif
	    EQUULEUS_SPI_CFG_CHAR_LEN(0));

}

int
starspi_configure(void *cookie, int slave, int mode, int speed)
{
	struct starspi_softc *sc;
	uint32_t v;
	int bitrate, i;

	sc = (struct starspi_softc *)cookie;

	printf("%s: configure: slave=%d, mode=%d, speed=%d\n",
	    device_xname(sc->sc_dev),
	    slave, mode, speed);

	v = bus_space_read_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_CFG_REG);
	switch (mode) {
	case SPI_MODE_0:
		v &= ~EQUULEUS_SPI_CFG_CLKPOL;
		v &= ~EQUULEUS_SPI_CFG_CLKPHA;
	case SPI_MODE_1:
		v &= ~EQUULEUS_SPI_CFG_CLKPOL;
		v |= EQUULEUS_SPI_CFG_CLKPHA;
		break;
		/* FALLTHRU */
	case SPI_MODE_2:
		v |= EQUULEUS_SPI_CFG_CLKPOL;
		v &= ~EQUULEUS_SPI_CFG_CLKPHA;
		break;
	case SPI_MODE_3:
		v |= EQUULEUS_SPI_CFG_CLKPOL;
		v |= EQUULEUS_SPI_CFG_CLKPHA;
		break;
	default:
		return EINVAL;
	}
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_CFG_REG, v);


	v = bus_space_read_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_TX_CTRL_REG);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_SPI_TX_CTRL_REG,
	    (v & ~EQUULEUS_SPI_TX_CTRL_TXCH_NUM_MASK) |
	    EQUULEUS_SPI_TX_CTRL_TXCH_NUM(slave));

	/* set bitrate */
	for (bitrate = STAR_PCLK, i = 0; i <= 7; i++) {
		if (speed >= bitrate)
			break;
		bitrate >>= 1;
	}
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_SPI_BIT_RATE_REG,
	    EQUULEUS_SPI_BIT_RATE(i));

//	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
//	    EQUULEUS_SPI_TX_CTRL_REG,
//	    EQUULEUS_SPI_TX_CTRL_TXCH_NUM(sc_slave));

	sc->sc_slave = slave;

	return 0;
}

static int
starspi_transfer(void *cookie, struct spi_transfer *st)
{
	struct starspi_softc *sc;
	int s;

	sc = (struct starspi_softc *)cookie;

//	printf("%s: transfer: transfer=%p, chunk=%p\n",
//	    device_xname(sc->sc_dev),
//	    st, st->st_chunks);

	s = splserial();
	{
		spi_transq_enqueue(&sc->sc_spi_transq, st);

		/*
		 * if idle start transfer,
		 * otherwise transfer would be continue in interrupt chain.
		 */
		if (sc->sc_spictx.transfer == NULL) {
			starspi_next(sc);
			if (starspi_start(sc) != 0) {
				spi_done(sc->sc_spictx.transfer, 0);
				sc->sc_spictx.transfer = NULL;
				sc->sc_spictx.chunk = NULL;
			}
		}
	}
	splx(s);

	return 0;
}

static int
starspi_next(struct starspi_softc *sc)
{
	/* if no transfer to do, pick up new transfer from queue */
	if (sc->sc_spictx.transfer == NULL) {
		KASSERT(sc->sc_spictx.chunk == NULL);
		sc->sc_spictx.transfer = spi_transq_first(&sc->sc_spi_transq);
		if (sc->sc_spictx.transfer == NULL)
			return 0;	/* no more transfer */

		spi_transq_dequeue(&sc->sc_spi_transq);
		sc->sc_spictx.chunk = sc->sc_spictx.transfer->st_chunks;

		/* clear RX */
#ifdef USE_FIFO
		while (RX_FIFO_READABLE(sc))
			bus_space_read_4(sc->sc_iot, sc->sc_ioh,
			    EQUULEUS_SPI_RX_DATA_REG);
#else
		while (RX_READABLE(sc))
			bus_space_read_4(sc->sc_iot, sc->sc_ioh,
			    EQUULEUS_SPI_RX_DATA_REG);
#endif
	}

	/* new transfer ready */
	return 1;
}


static unsigned char
starspi_chunkgetc(struct starspi_softc *sc)
{
	unsigned char c;

	KASSERT(sc->sc_spictx.chunk != NULL);

	if ((sc->sc_spictx.chunk->chunk_wresid == 0) ||
	    (sc->sc_spictx.chunk->chunk_wptr == NULL))
		return 0;

	c = *sc->sc_spictx.chunk->chunk_wptr++;
	sc->sc_spictx.chunk->chunk_wresid--;

	return c;
}

static int
starspi_chunkputc(struct starspi_softc *sc, uint8_t c)
{
	KASSERT(sc->sc_spictx.chunk != NULL);

	if ((sc->sc_spictx.chunk->chunk_rresid == 0) ||
	    (sc->sc_spictx.chunk->chunk_rptr == NULL))
		return 0;

	*sc->sc_spictx.chunk->chunk_rptr++ = c;
	sc->sc_spictx.chunk->chunk_rresid--;

	return 1;
}

static int
starspi_send(struct starspi_softc *sc)
{
	uint32_t status;
	uint32_t txdata;
	int endofchunk, txeof;

//	printf("%s: send: chunk=%p, resid=%d\n", device_xname(sc->sc_dev),
//	    sc->sc_spictx.chunk,
//	    sc->sc_spictx.chunk->chunk_wresid);

	KASSERT(sc->sc_spictx.chunk != NULL);

	txeof = 0;
	endofchunk = 0;

	do {
		while (SPI_BUSY(sc))
			;
#ifdef USE_FIFO
		while (!TX_FIFO_WRITABLE(sc))
			;
#else
		while (!TX_WRITABLE(sc))
			;
#endif

		txdata = starspi_chunkgetc(sc);
		endofchunk = (sc->sc_spictx.chunk->chunk_wresid == 0) ? 1 : 0;
		if (endofchunk && (sc->sc_spictx.chunk->chunk_next == NULL))
			txeof = 1;
		else
			txeof = 0;

		status = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_TX_CTRL_REG);
		status &=
		    ~EQUULEUS_SPI_TX_CTRL_TXDAT_EOF &
		    ~EQUULEUS_SPI_TX_CTRL_TXCH_NUM_MASK;
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_TX_CTRL_REG,
		    (txeof ? EQUULEUS_SPI_TX_CTRL_TXDAT_EOF : 0) |
		    EQUULEUS_SPI_TX_CTRL_TXCH_NUM(sc->sc_slave));

		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_TX_DATA_REG, txdata);

/* XXX */
#if 1
		{
			uint32_t dummy;
#ifdef USE_FIFO
			while (!RX_FIFO_READABLE(sc))
				;
#else
			while (!RX_READABLE(sc))
				;
#endif
			dummy = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
			    EQUULEUS_SPI_RX_DATA_REG);

			if (dummy) {
				printf("%s: send: dummy read: %08x\n",
				    device_xname(sc->sc_dev), dummy);
			}

		}
#endif

//		printf("%s: send: left %d bytes in chunk\n",
//		    device_xname(sc->sc_dev),
//		    sc->sc_spictx.chunk->chunk_wresid);
	} while (endofchunk == 0);

	sc->sc_spictx.chunk = sc->sc_spictx.chunk->chunk_next;
	if (txeof) {
		printf("%s: send: all chunk complete\n",
		    device_xname(sc->sc_dev));

		/* all chunks complete! */
		spi_done(sc->sc_spictx.transfer, 0);
		sc->sc_spictx.transfer = NULL;
		sc->sc_spictx.chunk = NULL;
	}
//	printf("%s: send: done\n", device_xname(sc->sc_dev));

	return 0;
}


static int
starspi_recv(struct starspi_softc *sc)
{
	uint32_t rxdata, status;
	int endofchunk, txeof, rxeof;

//	printf("%s: recv: chunk=%p, resid=%d\n", device_xname(sc->sc_dev),
//	    sc->sc_spictx.chunk,
//	    sc->sc_spictx.chunk->chunk_rresid);

	KASSERT(sc->sc_spictx.chunk != NULL);


	do {
		while (SPI_BUSY(sc))
			;
#ifdef USE_FIFO
		while (!TX_FIFO_WRITABLE(sc))
			;
#else
		while (!TX_WRITABLE(sc))
			;
#endif

		endofchunk = (sc->sc_spictx.chunk->chunk_rresid == 0) ? 1 : 0;
		if (endofchunk && (sc->sc_spictx.chunk->chunk_next == NULL))
			txeof = 1;
		else
			txeof = 0;

		status = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_TX_CTRL_REG);
		status &=
		    ~EQUULEUS_SPI_TX_CTRL_TXDAT_EOF &
		    ~EQUULEUS_SPI_TX_CTRL_TXCH_NUM_MASK;
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_TX_CTRL_REG,
		    (txeof ? EQUULEUS_SPI_TX_CTRL_TXDAT_EOF : 0) |
		    EQUULEUS_SPI_TX_CTRL_TXCH_NUM(sc->sc_slave));

		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_TX_DATA_REG, 0xff);

#ifdef USE_FIFO
		while (!RX_FIFO_READABLE(sc))
			;
#else
		while (!RX_READABLE(sc))
			;
#endif
		rxeof = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_RX_CTRL_REG) &
		    EQUULEUS_SPI_RX_CTRL_RXDAT_EOF;

		rxdata = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_SPI_RX_DATA_REG);
		starspi_chunkputc(sc, rxdata);

		if (!!txeof != !!rxeof) {
			printf("%s: read: error: TXEOF=%d, RXEOF=%d"
			    " and %d byte left in chunk\n",
			    device_xname(sc->sc_dev),
			    txeof, rxeof,
			    sc->sc_spictx.chunk->chunk_rresid);

			spi_done(sc->sc_spictx.transfer, EIO);
			sc->sc_spictx.transfer = NULL;
			sc->sc_spictx.chunk = NULL;
			return 0;
		}

	} while (endofchunk == 0);

	sc->sc_spictx.chunk = sc->sc_spictx.chunk->chunk_next;
	if (rxeof) {
		spi_done(sc->sc_spictx.transfer, 0);
		sc->sc_spictx.transfer = NULL;
		sc->sc_spictx.chunk = NULL;
		return 0;
	}

	return 1;
}

static int
starspi_start(struct starspi_softc *sc)
{
	while (sc->sc_spictx.chunk != NULL) {
		if (sc->sc_spictx.chunk->chunk_write != NULL) {
			if (starspi_send(sc) != 0)
				return 0;
			continue;
		}
		if (sc->sc_spictx.chunk->chunk_read != NULL) {
			if (starspi_recv(sc) != 0)
				return 0;
			continue;
		}

		printf("%s: nether write data nor read data in transfer=%p, "
		    "chunk=%p\n",
		    device_xname(sc->sc_dev),
		    sc->sc_spictx.transfer,
		    sc->sc_spictx.chunk);
		sc->sc_spictx.chunk = sc->sc_spictx.chunk->chunk_next;
		return -1;
	}

	return 0;
}

static int
starspi_intr(void *arg)
{
	struct starspi_softc *sc;
	uint32_t status;
	int err;

	sc = (struct starspi_softc *)arg;
	err = 0;

	/* read interrupt status and clear */
	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_SPI_INTSTAT_REG);
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_SPI_INTSTAT_REG, status);

	printf("%s: intr: status=%08x, transfer=%p, chunk=%p\n",
	    device_xname(sc->sc_dev), status,
	    sc->sc_spictx.transfer,
	    sc->sc_spictx.chunk);

	if (status & EQUULEUS_SPI_INTSTAT_TXFF_UNRN_FG) {
		printf("%s: TX FIFO underrun\n", device_xname(sc->sc_dev));
		err++;
	}
	if (status & EQUULEUS_SPI_INTSTAT_RXFF_OVRN_FG) {
		printf("%s: RX FIFO overrun\n", device_xname(sc->sc_dev));
		err++;
	}
	if (status & EQUULEUS_SPI_INTSTAT_TXBF_UNRN_FG) {
		printf("%s: TX underrun\n", device_xname(sc->sc_dev));
		err++;
	}
	if (status & EQUULEUS_SPI_INTSTAT_RXBF_OVRN_FG) {
		printf("%s: RX overrun\n", device_xname(sc->sc_dev));
		err++;
	}

	if (err) {
		/* if any error, cancel transfer and spi_done with error */
		spi_done(sc->sc_spictx.transfer, EIO);
		sc->sc_spictx.transfer = NULL;
		sc->sc_spictx.chunk = NULL;
	} else {
		/* continue to send/receive more chunks and return */
		if (sc->sc_spictx.chunk != NULL) {
			if (starspi_start(sc) != 0) {
				spi_done(sc->sc_spictx.transfer, EIO);
				sc->sc_spictx.transfer = NULL;
				sc->sc_spictx.chunk = NULL;
			}
		}
	}

	/* clear TX FIFO Empty interrupt */
	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_SPI_INT_ENABLE_REG);
#if 0
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_SPI_INT_ENABLE_REG, status & ~EQUULEUS_SPI_INT_TXFF);
#else
	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_SPI_INT_ENABLE_REG, status & ~EQUULEUS_SPI_INT_TXBF);
#endif

	/* continue if ready to next transfer */
	if (starspi_next(sc)) {
		if (starspi_start(sc) != 0) {
			spi_done(sc->sc_spictx.transfer, EIO);
			sc->sc_spictx.transfer = NULL;
			sc->sc_spictx.chunk = NULL;
		}
	}

	return 1;
}
