/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2019 Arm Limited
 *
 * Copyright(c) 2019 Intel Corporation
 *
 * Derived do_macswap implementation from app/test-pmd/macswap_sse.h
 */

#ifndef _MACSWAP_NEON_H_
#define _MACSWAP_NEON_H_

#include "macswap_common.h"
#include "rte_vect.h"

static inline void
do_macswap(struct rte_mbuf *pkts[], uint16_t nb,
		struct rte_port *txp)
{
	struct rte_ether_hdr *eth_hdr[4];
	struct rte_mbuf *mb[4];
	uint64_t ol_flags;
	int i;
	int r;
	uint8x16_t v0, v1, v2, v3;
	/**
	 * Index map be used to shuffle the 16 bytes.
	 * byte 0-5 will be swapped with byte 6-11.
	 * byte 12-15 will keep unchanged.
	 */
	const uint8x16_t idx_map = {6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5,
				12, 13, 14, 15};

	ol_flags = ol_flags_init(txp->dev_conf.txmode.offloads);
	vlan_qinq_set(pkts, nb, ol_flags,
			txp->tx_vlan_id, txp->tx_vlan_id_outer);

	i = 0;
	r = nb;

	while (r >= 4) {
		if (r >= 8) {
			rte_prefetch0(rte_pktmbuf_mtod(pkts[i + 4], void *));
			rte_prefetch0(rte_pktmbuf_mtod(pkts[i + 5], void *));
			rte_prefetch0(rte_pktmbuf_mtod(pkts[i + 6], void *));
			rte_prefetch0(rte_pktmbuf_mtod(pkts[i + 7], void *));
		}

		mb[0] = pkts[i++];
		eth_hdr[0] = rte_pktmbuf_mtod(mb[0], struct rte_ether_hdr *);

		mb[1] = pkts[i++];
		eth_hdr[1] = rte_pktmbuf_mtod(mb[1], struct rte_ether_hdr *);

		mb[2] = pkts[i++];
		eth_hdr[2] = rte_pktmbuf_mtod(mb[2], struct rte_ether_hdr *);

		mb[3] = pkts[i++];
		eth_hdr[3] = rte_pktmbuf_mtod(mb[3], struct rte_ether_hdr *);

		v0 = vld1q_u8((uint8_t const *)eth_hdr[0]);
		v1 = vld1q_u8((uint8_t const *)eth_hdr[1]);
		v2 = vld1q_u8((uint8_t const *)eth_hdr[2]);
		v3 = vld1q_u8((uint8_t const *)eth_hdr[3]);

		v0 = vqtbl1q_u8(v0, idx_map);
		v1 = vqtbl1q_u8(v1, idx_map);
		v2 = vqtbl1q_u8(v2, idx_map);
		v3 = vqtbl1q_u8(v3, idx_map);

		vst1q_u8((uint8_t *)eth_hdr[0], v0);
		vst1q_u8((uint8_t *)eth_hdr[1], v1);
		vst1q_u8((uint8_t *)eth_hdr[2], v2);
		vst1q_u8((uint8_t *)eth_hdr[3], v3);

		mbuf_field_set(mb[0], ol_flags);
		mbuf_field_set(mb[1], ol_flags);
		mbuf_field_set(mb[2], ol_flags);
		mbuf_field_set(mb[3], ol_flags);
		r -= 4;
	}

	for ( ; i < nb; i++) {
		if (i < nb - 1)
			rte_prefetch0(rte_pktmbuf_mtod(pkts[i+1], void *));
		mb[0] = pkts[i];
		eth_hdr[0] = rte_pktmbuf_mtod(mb[0], struct rte_ether_hdr *);

		/* Swap dest and src mac addresses. */
		v0 = vld1q_u8((uint8_t const *)eth_hdr[0]);
		v0 = vqtbl1q_u8(v0, idx_map);
		vst1q_u8((uint8_t *)eth_hdr[0], v0);

		mbuf_field_set(mb[0], ol_flags);
	}
}
#ifdef USE_ENSO

static inline void
do_macswap_enso_neon(RxEnsoPipe_t* rx_pipe, struct RXTXState* rx_tx_state ,uint8_t* buf, int new_bytes, int packet_size)
{
	uint8_t* cur_buf = buf;
	uint8_t* end_of_buffer = (uint8_t*)rx_pipe->buf + ENSO_BUF_SIZE;
	struct rte_ether_hdr *eth_hdr[4];

	int i;
	int r;
	int nb_rx;
	uint32_t consumed_bytes;
	uint8x16_t v0, v1, v2, v3;
	/**
	 * Index map be used to shuffle the 16 bytes.
	 * byte 0-5 will be swapped with byte 6-11.
	 * byte 12-15 will keep unchanged.
	 */
	const uint8x16_t idx_map = {6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5,
				12, 13, 14, 15};

	i = 0;

	// considering enso wrap-around buffer
	if(unlikely((buf + new_bytes) > end_of_buffer))
	{
		r = (end_of_buffer - buf)/packet_size;
		nb_rx = r;
	}
	else
	{
		r = new_bytes/packet_size;
		nb_rx = r;
	}
		

	while (r >= 4) {
		if (r >= 8) {
			rte_prefetch0((void*)(cur_buf + (i + 4) * packet_size));
			rte_prefetch0((void*)(cur_buf + (i + 5) * packet_size));
			rte_prefetch0((void*)(cur_buf + (i + 6) * packet_size));
			rte_prefetch0((void*)(cur_buf + (i + 7) * packet_size));
		}

		eth_hdr[0] = (struct rte_ether_hdr*)(cur_buf + (i++) * packet_size);

		eth_hdr[1] = (struct rte_ether_hdr*)(cur_buf + (i++) * packet_size);

		eth_hdr[2] = (struct rte_ether_hdr*)(cur_buf + (i++) * packet_size);

		eth_hdr[3] = (struct rte_ether_hdr*)(cur_buf + (i++) * packet_size);

		v0 = vld1q_u8((uint8_t const *)eth_hdr[0]);
		v1 = vld1q_u8((uint8_t const *)eth_hdr[1]);
		v2 = vld1q_u8((uint8_t const *)eth_hdr[2]);
		v3 = vld1q_u8((uint8_t const *)eth_hdr[3]);

		v0 = vqtbl1q_u8(v0, idx_map);
		v1 = vqtbl1q_u8(v1, idx_map);
		v2 = vqtbl1q_u8(v2, idx_map);
		v3 = vqtbl1q_u8(v3, idx_map);

		vst1q_u8((uint8_t *)eth_hdr[0], v0);
		vst1q_u8((uint8_t *)eth_hdr[1], v1);
		vst1q_u8((uint8_t *)eth_hdr[2], v2);
		vst1q_u8((uint8_t *)eth_hdr[3], v3);

		r -= 4;
		
		// additional logic for enso
		consumed_bytes = 4 * packet_size;
		
		memcpy(rx_tx_state->pending_tx.current_tx_buffer, cur_buf, consumed_bytes);
		rx_tx_state->pending_tx.current_tx_buffer += consumed_bytes;
        rx_tx_state->pending_tx.count += 4;

		// move buffer to next packet
		cur_buf += consumed_bytes;

		rte_eth_rx_confirm_byte(rx_pipe, consumed_bytes);

	}

	for ( ; i < nb_rx; i++) {
		if (i < nb_rx - 1)
			rte_prefetch0((void*)(cur_buf + (i + 1) * packet_size));
		eth_hdr[0] = (struct rte_ether_hdr*)cur_buf;

		/* Swap dest and src mac addresses. */
		v0 = vld1q_u8((uint8_t const *)eth_hdr[0]);
		v0 = vqtbl1q_u8(v0, idx_map);
		vst1q_u8((uint8_t *)eth_hdr[0], v0);

		// additional logic for enso
		memcpy(rx_tx_state->pending_tx.current_tx_buffer, cur_buf, packet_size);
		rx_tx_state->pending_tx.current_tx_buffer += packet_size;
        rx_tx_state->pending_tx.count++;

		// move buffer to next packet
		cur_buf += packet_size;

		rte_eth_rx_confirm_byte(rx_pipe, packet_size);
	}
}

#endif
#endif /* _MACSWAP_NEON_H_ */
