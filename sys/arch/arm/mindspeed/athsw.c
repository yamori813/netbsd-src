/*	$NetBSD$	*/

/*-
 * Copyright (c) 2023 Hiroki Mori
 * Copyright (c) 2011-2012 Stefan Bethke.
 * Copyright (c) 2006 Sam Leffler, Errno Consulting
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * copy from mvphy.c
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/device.h>
#include <sys/socket.h>
#include <sys/errno.h>

#include <net/if.h>
#include <net/if_media.h>

#include <dev/mii/mii.h>
#include <dev/mii/miivar.h>
#include <dev/mii/miidevs.h>

#include <dev/mii/mvphyreg.h>
#include <arm/mindspeed/arswitchreg.h>

#define	MV_PORT(sc)	((sc)->mii_phy - 16)	/* PHY # to switch port */
#define	MV_CPU_PORT	5			/* port # of CPU port */

#define	MV_READ(p, phy, r, v)						\
	(*(p)->mii_pdata->mii_readreg)(device_parent((p)->mii_dev),	\
	    (phy), (r), (v))
#define	MV_WRITE(p, phy, r, v)						\
	(*(p)->mii_pdata->mii_writereg)(device_parent((p)->mii_dev),	\
	    (phy), (r), (v))

/* XXX sysctl'able */
#define MV_ATUCTRL_ATU_SIZE_DEFAULT	2	/* 1024 entry database */
#define MV_ATUCTRL_AGE_TIME_DEFAULT	19	/* 19 * 16 = 304 seconds */

/*
 * Register manipulation macros that expect bit field defines
 * to follow the convention that an _S suffix is appended for
 * a shift count, while the field mask has no suffix.
 */
/*
#define	SM(_v, _f)	(((_v) << _f##_S) & _f)
#define	MS(_v, _f)	(((_v) & _f) >> _f##_S)
*/

struct athsw_softc {
	struct mii_softc	sc_mii;
	int		page;
	int		is_internal_switch;
	int		chip_ver;
	int		chip_rev;
	int		mii_lo_first;	/* Send low data DWORD before high */
};

static int	athswmatch(device_t, cfdata_t, void *);
static void	athswattach(device_t, device_t, void *);

CFATTACH_DECL_NEW(athsw, sizeof(struct athsw_softc),
    athswmatch, athswattach, mii_phy_detach, mii_phy_activate);

static int	athsw_service(struct mii_softc *, struct mii_data *, int);
static void	athsw_status(struct mii_softc *);
static void	athsw_reset(struct mii_softc *sc);

static const struct mii_phy_funcs athsw_funcs = {
	athsw_service, athsw_status, athsw_reset,
};

static const struct mii_phydesc athsws[] = {
	{0xc82e, 0x0003},	/* QCA8337 */
	{0xc82e, 0x0004},	/* AR8317 */
	MII_PHY_END,
};

/*
 * On AP30/AR5312 the switch is configured in one of two ways:
 * as a ROUTER or as a BRIDGE.  The ROUTER config sets up ports
 * 0-3 as LAN ports, port 4 as the WAN port, and port 5 connects
 * to the MAC in the 5312.  The BRIDGE config sets up ports
 * 0-4 as LAN ports with port 5 connected to the MAC in the 5312.
 */
#ifdef NOTUSE
struct mvPhyConfig {
	uint16_t switchPortAddr;/* switch port associated with PHY */
	uint16_t vlanSetting;	/* VLAN table setting  for PHY */
	uint32_t portControl;	/* switch port control setting for PHY */
};
static const struct mvPhyConfig dumbConfig[] = {
	{ 0x18, 0x2e,		/* PHY port 0 = LAN port 0 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x19, 0x2d,		/* PHY port 1 = LAN port 1 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1a, 0x2b,		/* PHY port 2 = LAN port 2 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1b, 0x27,		/* PHY port 3 = LAN port 3 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1c, 0x25,		/* PHY port 4 = LAN port 4 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1d, 0x1f,		/* PHY port 5 = CPU port */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING }
};
#if 0 /* XXX what are these? */
static const struct mvPhyConfig routerConfig[] = {
	{ 0x18, 0x2e,		/* PHY port 0 = LAN port 0 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x19, 0x2d,		/* PHY port 1 = LAN port 1 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1a, 0x2b,		/* PHY port 2 = LAN port 2 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1b, 0x27,		/* PHY port 3 = LAN port 3 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1c, 0x1020,		/* PHY port 4 = WAN port */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	/* NB: 0x0f =>'s send only to LAN ports */
	{ 0x1d, 0x0f,		/* PHY port 5 = CPU port */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING
#if 0
	  | MV_PORT_CONTROL_INGRESS_TRAILER
	  | MV_PORT_CONTROL_EGRESS_MODE
#endif
	  }
};
static const struct mvPhyConfig bridgeConfig[] = {
	{ 0x18, 0x3e,		/* PHY port 0 = LAN port 0 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x19, 0x3d,		/* PHY port 1 = LAN port 1 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1a, 0x3b,		/* PHY port 2 = LAN port 2 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1b, 0x37,		/* PHY port 3 = LAN port 3 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	{ 0x1c, 0x37,		/* PHY port 4 = LAN port 4 */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING },
	/* NB: 0x1f =>'s send to all ports */
	{ 0x1d, 0x1f,		/* PHY port 5 = CPU port */
	  MV_PORT_CONTROL_PORT_STATE_FORWARDING
#if 0
	  | MV_PORT_CONTROL_INGRESS_TRAILER
	  | MV_PORT_CONTROL_EGRESS_MODE
#endif
	}
};
#endif

static void athsw_switchconfig(struct mii_softc *, int);
static void athsw_flushatu(struct mii_softc *);

#endif   /* NOTUSE */

/* copy from arswitch_reg.c on FreeBSD etherswitch */

void arswitch_writedbg(device_t, int, uint16_t, uint16_t);
void arswitch_writemmd(device_t, int, uint16_t, uint16_t);

void
arswitch_writedbg(device_t dev, int phy, uint16_t dbg_addr,
    uint16_t dbg_data)
{
	struct mii_softc *sc = device_private(dev);
/*
        (void) MDIO_WRITEREG(device_get_parent(dev), phy,
            MII_ATH_DBG_ADDR, dbg_addr);
        (void) MDIO_WRITEREG(device_get_parent(dev), phy,
            MII_ATH_DBG_DATA, dbg_data);
*/
	MV_WRITE(sc, phy, MII_ATH_DBG_ADDR, dbg_addr);
	MV_WRITE(sc, phy, MII_ATH_DBG_DATA, dbg_data);
}

void
arswitch_writemmd(device_t dev, int phy, uint16_t dbg_addr,
    uint16_t dbg_data)
{
	struct mii_softc *sc = device_private(dev);
/*
        (void) MDIO_WRITEREG(device_get_parent(dev), phy,
            MII_ATH_MMD_ADDR, dbg_addr);
        (void) MDIO_WRITEREG(device_get_parent(dev), phy,
            MII_ATH_MMD_DATA, dbg_data);
*/
	MV_WRITE(sc, phy, MII_ATH_MMD_ADDR, dbg_addr);
	MV_WRITE(sc, phy, MII_ATH_MMD_DATA, dbg_data);
}

static inline void
arswitch_split_setpage(device_t dev, uint32_t addr, uint16_t *phy,
    uint16_t *reg)
{
	struct athsw_softc *asc = device_private(dev);
	struct mii_softc *sc = &asc->sc_mii;
	uint16_t page;

	page = (addr >> 9) & 0x1ff;
	*phy = (addr >> 6) & 0x7;
	*reg = (addr >> 1) & 0x1f;

	if (asc->page != page) {
//		MDIO_WRITEREG(device_get_parent(dev), 0x18, 0, page);
		MV_WRITE(sc, 0x18, 0, page);
		delay(2000);
		asc->page = page;
	}
}

static uint32_t
arswitch_reg_read32(device_t dev, int phy, int reg)
{
	struct mii_softc *sc = device_private(dev);
	uint16_t lo, hi;
//	pge_mii_readreg(dev, phy, reg, &lo);
//	pge_mii_readreg(dev, phy, reg + 1, &hi);
	MV_READ(sc, phy, reg, &lo);
	MV_READ(sc, phy, reg + 1, &hi);

	return (hi << 16) | lo;
}

static int
arswitch_reg_write32(device_t dev, int phy, int reg, uint32_t value)
{
	struct athsw_softc *asc = device_private(dev);
	struct mii_softc *sc = &asc->sc_mii;
	int r;
	uint16_t lo, hi;

//	sc = device_get_softc(dev);
	lo = value & 0xffff;
	hi = (uint16_t) (value >> 16);

	r = 1;
	if (asc->mii_lo_first) {
//		r = pge_mii_writereg(dev, phy, reg, lo);
//		r |= pge_mii_writereg(dev, phy, reg + 1, hi);
		MV_WRITE(sc, phy, reg, lo);
		MV_WRITE(sc, phy, reg + 1, hi);
	} else {
//		r = pge_mii_writereg(dev, phy, reg + 1, hi);
//		r |= pge_mii_writereg(dev, phy, reg, lo);
		MV_WRITE(sc, phy, reg + 1, hi);
		MV_WRITE(sc, phy, reg, lo);
	}

	return r;
}

int arswitch_readreg(device_t, int);
int
arswitch_readreg(device_t dev, int addr)
{
	uint16_t phy, reg;

	arswitch_split_setpage(dev, addr, &phy, &reg);
	return arswitch_reg_read32(dev, 0x10 | phy, reg);
}

int arswitch_writereg(device_t, int, int);
int
arswitch_writereg(device_t dev, int addr, int value)
{
	uint16_t phy, reg;

	arswitch_split_setpage(dev, addr, &phy, &reg);
	return (arswitch_reg_write32(dev, 0x10 | phy, reg, value));
}

static int
athswmatch(device_t parent, cfdata_t match, void *aux)
{
	struct mii_attach_args *ma = aux;

	if (mii_phy_match(ma, athsws) != NULL)
		return 10;

	return 0;
}

static void
athswattach(device_t parent, device_t self, void *aux)
{
	struct athsw_softc *asc = device_private(self);
	struct mii_softc *sc = &asc->sc_mii;
	struct mii_attach_args *ma = aux;
	struct mii_data *mii = ma->mii_data;
	uint16_t swid;

	asc->page = -1;

	sc->mii_dev = self;
	sc->mii_inst = mii->mii_instance;
	sc->mii_phy = ma->mii_phyno;
	sc->mii_funcs = &athsw_funcs;
	sc->mii_pdata = mii;
	sc->mii_flags = ma->mii_flags;

	mii_lock(mii);

	swid = arswitch_readreg(self, AR8X16_REG_MASK_CTRL);

	mii_unlock(mii);
	asc->chip_rev = (swid & AR8X16_MASK_CTRL_REV_MASK);
	asc->chip_ver = (swid & AR8X16_MASK_CTRL_VER_MASK) >>
	    AR8X16_MASK_CTRL_VER_SHIFT;

	aprint_naive(": Media interface\n");
	if (asc->chip_ver == 0x13) {
		aprint_normal(": QCA8337 Ethernet Switch\n");
		asc->mii_lo_first = 1;
		int t;
#if 0
		t = arswitch_readreg(self, AR8327_REG_FWD_CTRL0);
		printf("FWD CTRL0 %x\n", t);
		t = arswitch_readreg(self, AR8327_REG_FWD_CTRL1);
		printf("FWD CTRL1 %x\n", t);
		/* enable CPU port and disable mirror port */
		t = AR8327_FWD_CTRL0_CPU_PORT_EN |
		    AR8327_FWD_CTRL0_MIRROR_PORT;
		arswitch_writereg(self, AR8327_REG_FWD_CTRL0, t);
#endif

		/* forward multicast and broadcast frames to CPU */
		t = (AR8327_PORTS_ALL << AR8327_FWD_CTRL1_UC_FLOOD_S) |
		    (AR8327_PORTS_ALL << AR8327_FWD_CTRL1_MC_FLOOD_S) |
		    (AR8327_PORTS_ALL << AR8327_FWD_CTRL1_BC_FLOOD_S);
		arswitch_writereg(self, AR8327_REG_FWD_CTRL1, t);

		/* Disable EEE on all ports due to stability issues */
		t = arswitch_readreg(self, AR8327_REG_EEE_CTRL);
		t |= AR8327_EEE_CTRL_DISABLE_PHY(0) |
		    AR8327_EEE_CTRL_DISABLE_PHY(1) |
		    AR8327_EEE_CTRL_DISABLE_PHY(2) |
		    AR8327_EEE_CTRL_DISABLE_PHY(3) |
		    AR8327_EEE_CTRL_DISABLE_PHY(4);
		arswitch_writereg(self, AR8327_REG_EEE_CTRL, t);

		int port;
		port = 5;
		arswitch_writereg(self, AR8327_REG_PORT_STATUS(port),
		    AR8X16_PORT_STS_LINK_AUTO);
		arswitch_writereg(self, AR8327_REG_PORT_HEADER(port), 0);

		/* PORT5_PAD_CTRL[MAC_RGMII_RXCLK_DELAY_EN] controls all RGMII
		 * interfaces (MAC0, MAC5 and MAC6)
		 * If set, then RGMII interface RXCLK is delayed:.
		 *   1000M:    Delay 2 ns output to CPU.
		 *   10M/100M: Delay value depends on bits[21:20].
		 */
		t = AR8327_PAD_RGMII_RXCLK_DELAY_EN;
		arswitch_writereg(self, AR8327_REG_PAD5_MODE, t);

#if 0
		/* PHY4 conncect to RGMII (same as barebox default) */
		t = AR8327_PAD_PHYX_RGMII_EN;
#endif
		/* RGMII connect to MAC6 */
		t = AR8327_PAD_RGMII_EN |
		    AR8327_PAD_RGMII_TXCLK_DELAY_EN |
		    (2 << AR8327_PAD_RGMII_TXCLK_DELAY_SEL_S);
		arswitch_writereg(self, AR8327_REG_PAD6_MODE, t);

		port = 6;
		t = AR8X16_PORT_STS_TXMAC | AR8X16_PORT_STS_RXMAC;
		t |= AR8X16_PORT_STS_DUPLEX;
		t |= (AR8X16_PORT_STS_RXFLOW | AR8X16_PORT_STS_TXFLOW);
		t |= AR8X16_PORT_STS_SPEED_1000;
		arswitch_writereg(self, AR8327_REG_PORT_STATUS(port), t);
		arswitch_writereg(self, AR8327_REG_PORT_HEADER(port), 0);
#if 0
		/* this is default */
		t = 1 << AR8327_PORT_VLAN0_DEF_SVID_S;
		t |= 1 << AR8327_PORT_VLAN0_DEF_CVID_S;
		arswitch_writereg(self, AR8327_REG_PORT_VLAN0(port), t);
		t = AR8327_PORT_VLAN1_OUT_MODE_UNTOUCH <<
		    AR8327_PORT_VLAN1_OUT_MODE_S;
		arswitch_writereg(self, AR8327_REG_PORT_VLAN1(port), t);
#endif

#if 0
		for (port = 0; port < 7; ++port) {
			uint32_t reg = arswitch_readreg(self,
			    AR8327_REG_PORT_LOOKUP(port));
			printf("PORT LOOKUP %d %x\n", port, reg);
			reg = arswitch_readreg(self,
			    AR8327_REG_PORT_STATUS(port));
			printf("PORT STATUS %d %x\n", port, reg);
			reg = arswitch_readreg(self,
			    AR8327_REG_PORT_VLAN0(port));
			printf("PORT VLAN0 %d %x\n", port, reg);
			reg = arswitch_readreg(self,
			    AR8327_REG_PORT_VLAN1(port));
			printf("PORT VLAN1 %d %x\n", port, reg);
		}
#endif
	} else if (asc->chip_ver == 0x10) {
		aprint_normal(": AR8316 Ethernet Switch\n");
		asc->mii_lo_first = 0;
		arswitch_writereg(self, AR8X16_REG_MODE,
		    AR8X16_MODE_RGMII_PORT4_ISO);
		/* work around for phy4 rgmii mode */
		arswitch_writedbg(self, 4, 0x12, 0x480c);
		/* rx delay */
		arswitch_writedbg(self, 4, 0x0, 0x824e);
		/* tx delay */
		arswitch_writedbg(self, 4, 0x5, 0x3d47);
		delay(1000);    /* 1ms, again to let things settle */
/* not work strang behaviour ???
		arswitch_writereg(self, AR8X16_REG_MASK_CTRL,
		    AR8X16_MASK_CTRL_SOFT_RESET);
		delay(1000);
*/
#ifdef _DEBUG
		int i;
		for (i = 0;i < 6; ++i)
		aprint_normal_dev(self, "PORT STS %d %x\n", i,
		    arswitch_readreg(self, 0x100 * (i + 1)));
		for (i = 0;i < 6; ++i)
		aprint_normal_dev(self, "PORT CTRL %d %x\n", i,
		    arswitch_readreg(self, 0x100 * (i + 1) + 4));
		for (i = 0;i < 6; ++i)
		aprint_normal_dev(self, "PORT VLAN %d %x\n", i,
		    arswitch_readreg(self, 0x100 * (i + 1) + 8));
#endif
		aprint_normal_dev(self, "arswitch %x mode %x\n",
		    arswitch_readreg(self, AR8X16_REG_MASK_CTRL),
		    arswitch_readreg(self, AR8X16_REG_MODE));
	} else {
		aprint_normal(": Unknown Ethernet Switch\n");
	}
//	mii_phy_add_media(sc);
}

static int
athsw_service(struct mii_softc *sc, struct mii_data *mii, int cmd)
{
#if 0
	struct ifmedia_entry *ife = mii->mii_media.ifm_cur;

	KASSERT(mii_locked(mii));

	switch (cmd) {
	case MII_POLLSTAT:
		/* If we're not polling our PHY instance, just return. */
		if (IFM_INST(ife->ifm_media) != sc->mii_inst)
			return 0;
		break;

	case MII_MEDIACHG:
		/*
		 * If the media indicates a different PHY instance,
		 * isolate ourselves.
		 */
		if (IFM_INST(ife->ifm_media) != sc->mii_inst) {
			/* XXX? */
			return 0;
		}

		/* If the interface is not up, don't do anything. */
		if ((mii->mii_ifp->if_flags & IFF_UP) == 0)
			break;

		mii_phy_setmedia(sc);
		break;

	case MII_TICK:
		/* If we're not currently selected, just return. */
		if (IFM_INST(ife->ifm_media) != sc->mii_inst)
			return 0;

		if (mii_phy_tick(sc) == EJUSTRETURN)
			return 0;
		break;

	case MII_DOWN:
		mii_phy_down(sc);
		return 0;
	}

	/* Update the media status. */
	mii_phy_status(sc);

	/* Callback if something changed. */
	mii_phy_update(sc, cmd);
#endif
	return 0;
}

static void
athsw_status(struct mii_softc *sc)
{
#if 0
	struct mii_data *mii = sc->mii_pdata;
//	uint16_t hwstatus;

	KASSERT(mii_locked(mii));

	mii->mii_media_status = IFM_AVALID;
	mii->mii_media_active = IFM_ETHER;

#if 1
	mii->mii_media_status |= IFM_ACTIVE;
	mii->mii_media_active |= IFM_1000_T;
	mii->mii_media_active |= IFM_FDX;
	if (0)
		athsw_flushatu(sc);
#else
	PHY_READ(sc, MII_MV_PHY_SPECIFIC_STATUS, &hwstatus);
	if (hwstatus & MV_STATUS_REAL_TIME_LINK_UP) {
		mii->mii_media_status |= IFM_ACTIVE;
		if (hwstatus & MV_STATUS_RESOLVED_SPEED_100)
			mii->mii_media_active |= IFM_100_TX;
		else
			mii->mii_media_active |= IFM_10_T;
		if (hwstatus & MV_STATUS_RESOLVED_DUPLEX_FULL)
			mii->mii_media_active |= IFM_FDX;
		else
			mii->mii_media_active |= IFM_HDX;
	} else {
		mii->mii_media_active |= IFM_NONE;
		/* XXX flush ATU only on link down transition */
		athsw_flushatu(sc);
	}
#endif
#endif
}

static void
athsw_reset(struct mii_softc *sc)
{
#if 0

	KASSERT(mii_locked(sc->mii_pdata));

	/* XXX handle fixed media config */
	PHY_WRITE(sc, MII_BMCR, BMCR_RESET | BMCR_AUTOEN);
	athsw_switchconfig(sc, MV_PORT(sc));
#endif
}

#ifdef NOTUSE
/*
 * Configure switch for the specified port.
 */
static void
athsw_switchconfig(struct mii_softc *sc, int port)
{
	/* XXX router vs bridge */
	/*const struct mvPhyConfig *conf = &routerConfig[port];*/
	/*const struct mvPhyConfig *conf = &bridgeConfig[port];*/
	const struct mvPhyConfig *conf = &dumbConfig[port];

	MV_WRITE(sc, conf->switchPortAddr, MV_PORT_BASED_VLAN_MAP,
	    conf->vlanSetting);
	/* XXX administrative control of port enable? */
	MV_WRITE(sc, conf->switchPortAddr, MV_PORT_CONTROL, conf->portControl);
	MV_WRITE(sc, conf->switchPortAddr, MV_PORT_ASSOCIATION_VECTOR,
	    1 << port);
}

/*
 * Flush the Address Translation Unit (ATU).
 */
static void
athsw_flushatu(struct mii_softc *sc)
{
	int status;
	uint16_t reg;
	int i;

	/* wait for any previous request to complete */
	/* XXX if busy defer to tick */
	/* XXX timeout */
	for (i = 0; i < 1000; i++) {
		status = MV_READ(sc, MII_MV_SWITCH_GLOBAL_ADDR,
		    MV_ATU_OPERATION, &reg);
		if (MV_ATU_IS_BUSY(status))
			break;
	}
	if (i != 1000) {
		MV_WRITE(sc, MII_MV_SWITCH_GLOBAL_ADDR, MV_ATU_OPERATION,
		    MV_ATU_OP_FLUSH_ALL | MV_ATU_BUSY);
	} /*else
		aprint_error_dev(sc->mii_dev, "timeout waiting for ATU flush\n");*/
}
#endif   /* NOTUSE */
