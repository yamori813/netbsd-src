/*-
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
 * THIS SOFTWFV IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE FV DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR OR HIS RELATIVES BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF MIND, USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWFV, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __IF_FVREG_H__
#define	__IF_FVREG_H__

#define FV_TX_RING_CNT	64
#define FV_RX_RING_CNT	64

#define FV_TXFRAGS	16

struct fv_desc {
	uint32_t	fv_stat;
	uint32_t	fv_devcs;
	uint32_t	fv_addr;
	uint32_t	fv_link;
};

struct fv_ring_data {
	bus_dmamap_t		tx_dm[FV_TX_RING_CNT];
	struct mbuf		*tx_mb[FV_TX_RING_CNT];
	bus_dmamap_t		rx_dm[FV_RX_RING_CNT];
	struct mbuf		*rx_mb[FV_RX_RING_CNT];
};


#endif /* __IF_FVREG_H__ */
