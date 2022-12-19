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
#include <sys/proc.h>

#include <arm/star/starreg.h>
#include <arm/star/starvar.h>

#include <dev/i2c/i2cvar.h>

struct startwi_softc {
	device_t sc_dev;
	bus_addr_t sc_addr;
	bus_space_tag_t sc_iot;
	bus_space_handle_t sc_ioh;
	void *sc_ih;

	int sc_ident;
	int sc_error;

	struct i2c_controller sc_i2c;
	kmutex_t sc_buslock;
};

static int startwi_match(device_t, struct cfdata *, void *);
static void startwi_attach(device_t, device_t, void *);
static void startwi_init(struct startwi_softc *);
static int startwi_intr(void *);
static void startwi_rddata(struct startwi_softc *, uint8_t *, int);
static void startwi_wrdata(struct startwi_softc *, const uint8_t *, int);

/* i2c controller */
static int startwi_i2c_acquire_bus(void *, int);
static void startwi_i2c_release_bus(void *, int);
static int startwi_i2c_exec(void *, i2c_op_t, i2c_addr_t, const void *, size_t,
                            void *, size_t, int);

CFATTACH_DECL_NEW(startwi, sizeof(struct startwi_softc),
    startwi_match, startwi_attach, NULL, NULL);


/* ARGSUSED */
static int
startwi_match(device_t parent __unused, struct cfdata *match __unused,
    void *aux)
{
	struct star_attach_args *sa;

	if (!CPU_IS_STR8100())
		return 0;

	sa = aux;
	sa->sa_size = EQUULEUS_TWI_REGSIZE;

	return 1;
}

/* ARGSUSED */
static void
startwi_attach(device_t parent __unused, device_t self, void *aux)
{
	struct startwi_softc *sc;
	struct star_attach_args *sa;
	struct i2cbus_attach_args iba;

	sa = aux;
	sc = device_private(self);
	sc->sc_dev = self;
	sc->sc_iot = sa->sa_iot;
	sc->sc_addr = sa->sa_addr;

	aprint_normal(": Two-Wire Serial Interface\n");
	aprint_naive("\n");

	if (bus_space_map(sc->sc_iot, sc->sc_addr, sa->sa_size, 0,
	    &sc->sc_ioh)) {
		aprint_error(": can't map registers\n");
		return;
	}

	startwi_init(sc);

	mutex_init(&sc->sc_buslock, MUTEX_DEFAULT, IPL_NONE);
	sc->sc_ih = intr_establish(STAR_IRQ_TWI, IPL_SERIAL,
	    IST_LEVEL_LOW, startwi_intr, sc);

	sc->sc_i2c.ic_cookie = sc;
	sc->sc_i2c.ic_acquire_bus = startwi_i2c_acquire_bus;
	sc->sc_i2c.ic_release_bus = startwi_i2c_release_bus;
	sc->sc_i2c.ic_send_start = NULL;
	sc->sc_i2c.ic_send_stop = NULL;
	sc->sc_i2c.ic_initiate_xfer = NULL;
	sc->sc_i2c.ic_read_byte = NULL;
	sc->sc_i2c.ic_write_byte = NULL;
	sc->sc_i2c.ic_exec = startwi_i2c_exec;

	iba.iba_tag = &sc->sc_i2c;
	config_found_ia(sc->sc_dev, "i2cbus", &iba, iicbus_print);
}

static void
startwi_init(struct startwi_softc *sc)
{
	uint32_t val;

	/* SPI/PCM/I2S/TWI Controler Enable */
	val = STAR_REG_READ32(EQUULEUS_CLKPWR_CLKGATE1_REG);
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_CLKGATE1_REG,
	    val | EQUULEUS_CLKPWR_CLKGATE1_PCLK_P2S);

	/* activate SPI/PCM/I2S/TWI */
	val = STAR_REG_READ32(EQUULEUS_CLKPWR_SOFTRST_REG);
	STAR_REG_WRITE32(EQUULEUS_CLKPWR_SOFTRST_REG,
	    val | EQUULEUS_CLKPWR_SOFTRST_P2S);

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_TWI_INT_ENABLE_REG,
	    EQUULEUS_TWI_INT_ACTDONE |
	    EQUULEUS_TWI_INT_BUSERR);

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_TWI_CTRL_REG,
	    EQUULEUS_TWI_CTRL_EN);

	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_TWI_TIMEOUT_REG,
	    EQUULEUS_TWI_TIMEOUT_CLKDIV(0x3e) |
	    EQUULEUS_TWI_TIMEOUT_OUT_EN |
	    EQUULEUS_TWI_TIMEOUT_OUT_VA(0x40));
}

static int
startwi_intr(void *arg)
{
	struct startwi_softc *sc;
	uint32_t status;
	int error;

	sc = (struct startwi_softc *)arg;

	status = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_TWI_INT_STATUS_REG);

	if (status & EQUULEUS_TWI_INT_ACTDONE)
		printf("%s: ACTDONE\n", device_xname(sc->sc_dev));

	if (status & EQUULEUS_TWI_INT_BUSERR)
		printf("%s: BUSERR\n", device_xname(sc->sc_dev));

	error = 0;
	switch (status & 0x0000ff00) {
	case 0x00002000:
		printf("%s: Slave Address + Transmitted, and NACK\n",
		    device_xname(sc->sc_dev));
		error = EIO;
		break;
	case 0x00003000:
		printf("%s: Transmitted and NACK\n", device_xname(sc->sc_dev));
		error = EIO;
		break;
	case 0x00004800:
		printf("%s: Slave Address + Received, and NACK\n",
		    device_xname(sc->sc_dev));
		error = EIO;
		break;
	case 0x00007000:
		printf("%s: Bus error, SDA stuck low\n",
		    device_xname(sc->sc_dev));
		error = EIO;
		break;
	case 0x00009000:
		printf("%s: Bus error, SCL stuck low\n",
		    device_xname(sc->sc_dev));
		error = EIO;
		break;
	case 0x0000ff00:
		printf("%s: Normal\n", device_xname(sc->sc_dev));
		break;
	default:
		printf("%s: unknown status: 0x%08x\n", device_xname(sc->sc_dev),
		    status);
		error = EIO;
		break;
	}

	/* clear status */
	bus_space_write_4(sc->sc_iot, sc->sc_ioh, EQUULEUS_TWI_INT_STATUS_REG,
	    EQUULEUS_TWI_INT_ACTDONE |
	    EQUULEUS_TWI_INT_BUSERR);

	sc->sc_error = error;
	wakeup(&sc->sc_ident);

	return 1;
}

static int
startwi_i2c_acquire_bus(void *cookie, int flags)
{
	struct startwi_softc *sc;

	sc = cookie;
	if (flags & I2C_F_POLL)
		return 0;

	mutex_enter(&sc->sc_buslock);
	return 0;
}

static void
startwi_i2c_release_bus(void *cookie, int flags)
{
	struct startwi_softc *sc;

	sc = cookie;
	if (flags & I2C_F_POLL)
		return;

	mutex_exit(&sc->sc_buslock);
}

/*
 * data[0]: bit 7:0
 * data[1]: bit 15:8
 * data[2]: bit 23:16
 * data[3]: bit 31:24
 */
static void
startwi_wrdata(struct startwi_softc *sc, const uint8_t *data, int len)
{
	uint32_t txdata;

	if ((len <= 0) || (len > 4))
		return;

	txdata = 0;
	switch (len) {
	case 4:
		txdata |= data[3] << 24;
		/* FALLTHRU */
	case 3:
		txdata |= data[2] << 16;
		/* FALLTHRU */
	case 2:
		txdata |= data[1] << 8;
		/* FALLTHRU */
	case 1:
		txdata |= data[0];
		break;
	}

	bus_space_write_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_TWI_WR_DATA_REG, txdata);
}

static void
startwi_rddata(struct startwi_softc *sc, uint8_t *data, int len)
{
	uint32_t rxdata;

	if ((len <= 0) || (len > 4))
		return;

	rxdata = bus_space_read_4(sc->sc_iot, sc->sc_ioh,
	    EQUULEUS_TWI_RD_DATA_REG);

	switch (len) {
	case 4:
		data[3] = (rxdata >> 24) & 0xff;
		/* FALLTHRU */
	case 3:
		data[2] = (rxdata >> 16) & 0xff;
		/* FALLTHRU */
	case 2:
		data[1] = (rxdata >> 8) & 0xff;
		/* FALLTHRU */
	case 1:
		data[0] = rxdata & 0xff;
		break;
	}
}

/* ARGSUSED */
static int
startwi_i2c_exec(void *cookie, i2c_op_t op, i2c_addr_t addr __unused,
    const void *vcmd, size_t cmdlen, void *vbuf, size_t buflen,
    int flags __unused)
{
	struct startwi_softc *sc;
	uint8_t txbuf[4];
	int error;

	sc = cookie;

	if (I2C_OP_READ_P(op)) {
		/*
		 * write vcmd[cmdlen] and
		 * read vbuf[buflen]
		 */

		if ((cmdlen > 4) || (buflen > 4))
			return EINVAL;
		if (buflen == 0)
			return EINVAL;	/* quick read not supported */

		if (cmdlen == 0) {
			/* read only operation */
			bus_space_write_4(sc->sc_iot, sc->sc_ioh,
			    EQUULEUS_TWI_CTRL_REG,
			    EQUULEUS_TWI_CTRL_EN |
			    EQUULEUS_TWI_CTRL_RUN_START |
			    EQUULEUS_TWI_CTRL_TRANSFER_CMD_RD |
			    EQUULEUS_TWI_CTRL_RDDAT_LEN(buflen - 1));
		} else {
			/* write then read operation */
			startwi_wrdata(sc, vcmd, cmdlen);
			bus_space_write_4(sc->sc_iot, sc->sc_ioh,
			    EQUULEUS_TWI_CTRL_REG,
			    EQUULEUS_TWI_CTRL_EN |
			    EQUULEUS_TWI_CTRL_RUN_START |
			    EQUULEUS_TWI_CTRL_TRANSFER_CMD_WRRD |
			    EQUULEUS_TWI_CTRL_WRDAT_LEN(cmdlen - 1) |
			    EQUULEUS_TWI_CTRL_RDDAT_LEN(buflen - 1));
		}

		error = tsleep(&sc->sc_ident, 0, "i2c", 1000);
		if (error)
			return error;
		if (sc->sc_error)
			return sc->sc_error;

		/* fetch result */
		startwi_rddata(sc, vbuf, buflen);
		return 0;

	} else {
		/*
		 * write vcmd[cmdlen] and
		 * write vbuf[buflen]
		 */

		if ((cmdlen + buflen) > 4)
			return EINVAL;
		if ((cmdlen + buflen) <= 0)
			return EINVAL;	/* quick write not supported */

		/* concatenate cmd and vbuf */
		memcpy(txbuf, vcmd, cmdlen);
		memcpy(txbuf + cmdlen, vbuf, buflen);

		/* write operation */
		startwi_wrdata(sc, txbuf, cmdlen + buflen);
		bus_space_write_4(sc->sc_iot, sc->sc_ioh,
		    EQUULEUS_TWI_CTRL_REG,
		    EQUULEUS_TWI_CTRL_EN |
		    EQUULEUS_TWI_CTRL_RUN_START |
		    EQUULEUS_TWI_CTRL_TRANSFER_CMD_WR |
		    EQUULEUS_TWI_CTRL_WRDAT_LEN(cmdlen + buflen - 1));

		error = tsleep(&sc->sc_ident, 0, "i2c", 1000);
		if (error)
			return error;
		return sc->sc_error;
	}

	return ENODEV;
}
