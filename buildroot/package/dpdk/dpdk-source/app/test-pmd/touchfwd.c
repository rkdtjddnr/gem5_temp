/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright 2014-2020 Mellanox Technologies, Ltd
 */

#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <unistd.h>
#include <inttypes.h>

#include <sys/queue.h>
#include <sys/stat.h>

#include <rte_common.h>
#include <rte_byteorder.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_cycles.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_launch.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_mempool.h>
#include <rte_mbuf.h>
#include <rte_interrupts.h>
#include <rte_pci.h>
#include <rte_ether.h>
#include <rte_ethdev.h>
#include <rte_ip.h>
#include <rte_string_fns.h>
#include <rte_flow.h>

#include "testpmd.h"
#if defined(RTE_ARCH_X86)
#include "macswap_sse.h"
#elif defined(__ARM_NEON)
#include "macswap_neon.h"
#else
#include "macswap.h"
#endif

#include <arm_sve.h>

#ifndef USE_ENSO
volatile char flag_touch;
/*
 * MAC swap forwarding mode: Swap the source and the destination Ethernet
 * addresses of packets before forwarding them.
 */
static void
pkt_burst_touch(struct fwd_stream *fs)
{
	struct rte_mbuf  *pkts_burst[MAX_PKT_BURST];
	struct rte_port  *txp;
	struct rte_mbuf  *mb;
	uint16_t nb_rx;
	uint16_t nb_tx;
	uint32_t retry;
	uint64_t start_tsc = 0;

	get_start_cycles(&start_tsc);

	/*
	 * Receive a burst of packets and forward them.
	 */
	nb_rx = rte_eth_rx_burst(fs->rx_port, fs->rx_queue, pkts_burst,
				 nb_pkt_per_burst);
	inc_rx_burst_stats(fs, nb_rx);
	if (unlikely(nb_rx == 0))
		return;

	for (int i = 0; i < nb_rx; i++) {
		if (likely(i < nb_rx - 1)) 
			rte_prefetch0(rte_pktmbuf_mtod(pkts_burst[i + 1], void *));
	
		char *pkt_data;

		mb = pkts_burst[i];
		pkt_data = rte_pktmbuf_mtod(mb, char *);
		
		for (uint j = 0; j < mb->pkt_len; j++)
		{
			// Do something with data here
			if (pkt_data[j] == 255)
				flag_touch = pkt_data[j];
		}
	}

	fs->rx_packets += nb_rx;
	txp = &ports[fs->tx_port];

	do_macswap(pkts_burst, nb_rx, txp);

	nb_tx = rte_eth_tx_burst(fs->tx_port, fs->tx_queue, pkts_burst, nb_rx);
	/*
	 * Retry if necessary
	 */
	if (unlikely(nb_tx < nb_rx) && fs->retry_enabled) {
		retry = 0;
		while (nb_tx < nb_rx && retry++ < burst_tx_retry_num) {
			rte_delay_us(burst_tx_delay_time);
			nb_tx += rte_eth_tx_burst(fs->tx_port, fs->tx_queue,
					&pkts_burst[nb_tx], nb_rx - nb_tx);
		}
	}
	fs->tx_packets += nb_tx;
	inc_tx_burst_stats(fs, nb_tx);
	if (unlikely(nb_tx < nb_rx)) {
		fs->fwd_dropped += (nb_rx - nb_tx);
		do {
			rte_pktmbuf_free(pkts_burst[nb_tx]);
		} while (++nb_tx < nb_rx);
	}
	get_end_cycles(fs, start_tsc);
}

struct fwd_engine touch_fwd_engine = {
	.fwd_mode_name  = "touchfwd",
	.port_fwd_begin = NULL,
	.port_fwd_end   = NULL,
	.packet_fwd     = pkt_burst_touch,
};

#else

volatile char flag_touch;
/* ====================================ENSO SVE version touchfwd=================================== */
#define DO_COMPARE 1
volatile uint64_t matched_count = 0;

static inline void
touch_sve_core(uint8_t* buf, int pkts, int packet_size)
{
	uint8_t* cur_buf = buf;

	int i;
	int r;
	int nb_rx;
	uint32_t consumed_bytes;

	i = 0;
	r = pkts;

	const uint64_t target_value = 0x12345678; // Example target value for comparison
	svuint64_t target_sv = svdup_u64(target_value);
	svbool_t pg = svptrue_b64();
		
	while (r >= 4) {
		if (r >= 8) {
			rte_prefetch0((void*)(cur_buf + (i + 4) * packet_size));
			rte_prefetch0((void*)(cur_buf + (i + 5) * packet_size));
			rte_prefetch0((void*)(cur_buf + (i + 6) * packet_size));
			rte_prefetch0((void*)(cur_buf + (i + 7) * packet_size));
		}

		uint8_t* pkt_data0 = cur_buf + (i++) * packet_size;

		uint8_t* pkt_data1 = cur_buf + (i++) * packet_size;

		uint8_t* pkt_data2 = cur_buf + (i++) * packet_size;

		uint8_t* pkt_data3 = cur_buf + (i++) * packet_size;

		#if DO_COMPARE == 0
		// Touch data
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			volatile uint32_t d = *(uint32_t *)(pkt_data0 + l * 64);
			(void)d; // Prevent unused variable warning
		}
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			volatile uint32_t d = *(uint32_t *)(pkt_data1 + l * 64);
			(void)d; // Prevent unused variable warning
		}
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			volatile uint32_t d = *(uint32_t *)(pkt_data2 + l * 64);
			(void)d; // Prevent unused variable warning
		}
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			volatile uint32_t d = *(uint32_t *)(pkt_data3 + l * 64);
			(void)d; // Prevent unused variable warning
		}
		#elif DO_COMPARE == 1
		uint64_t matched_count_0 = 0;
		uint64_t matched_count_1 = 0;
		uint64_t matched_count_2 = 0;
		uint64_t matched_count_3 = 0;

		// Touch & compare data
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			svuint64_t data_sv = svld1(pg, (const uint64_t *)(pkt_data0 + l * 64));
			svbool_t match_pg = svcmpeq_u64(pg, data_sv, target_sv);
			matched_count_0 += svcntp_b64(pg, match_pg);
		}
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			svuint64_t data_sv = svld1(pg, (const uint64_t *)(pkt_data1 + l * 64));
			svbool_t match_pg = svcmpeq_u64(pg, data_sv, target_sv);
			matched_count_1 += svcntp_b64(pg, match_pg);
		}
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			svuint64_t data_sv = svld1(pg, (const uint64_t *)(pkt_data2 + l * 64));
			svbool_t match_pg = svcmpeq_u64(pg, data_sv, target_sv);
			matched_count_2 += svcntp_b64(pg, match_pg);
		}
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			svuint64_t data_sv = svld1(pg, (const uint64_t *)(pkt_data3 + l * 64));
			svbool_t match_pg = svcmpeq_u64(pg, data_sv, target_sv);
			matched_count_3 += svcntp_b64(pg, match_pg);
		}
		matched_count += matched_count_0 + matched_count_1 + matched_count_2 + matched_count_3;
		#endif

		r -= 4;
		
		// additional logic for enso
		consumed_bytes = 4 * packet_size;

		// move buffer to next packet
		cur_buf += consumed_bytes;
	}

	for ( ; i < nb_rx; i++) {
		if (i < nb_rx - 1)
			rte_prefetch0((void*)(cur_buf + (i + 1) * packet_size));
		uint8_t* pkt_data = cur_buf;

		#if DO_COMPARE == 0
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			volatile uint32_t d = *(uint32_t *)(pkt_data + l * 64);
			(void)d; // Prevent unused variable warning
		}
		#elif DO_COMPARE == 1
		uint64_t matched_count_0 = 0;
		// Touch & compare data
		for (uint32_t l = 0; l < ((packet_size + 63) / 64); l++) {
			svuint64_t data_sv = svld1(pg, (const uint64_t *)(pkt_data + l * 64));
			svbool_t match_pg = svcmpeq_u64(pg, data_sv, target_sv);
			matched_count_0 += svcntp_b64(pg, match_pg);
		}
		matched_count += matched_count_0;
		#endif
		// move buffer to next packet
		cur_buf += packet_size;
	}
}

static inline void
do_touch_sve(RxEnsoPipe_t* rx_pipe, uint8_t* buf, int new_bytes, int packet_size)
{
	uint8_t* end_of_buffer = (uint8_t*)rx_pipe->buf + ENSO_BUF_SIZE;
    int total_pkts = new_bytes / packet_size;
    int first_part_bytes = end_of_buffer - buf;
    int first_part_pkts = (new_bytes <= first_part_bytes) ? total_pkts : first_part_bytes / packet_size;
    int remaining_pkts = total_pkts - first_part_pkts;

    if (likely(first_part_pkts > 0)) {
        touch_sve_core(buf, first_part_pkts, packet_size);
    }

    if (remaining_pkts > 0) {
		// wrap-around for TX 
        touch_sve_core((uint8_t*)rx_pipe->buf, remaining_pkts, packet_size);
    }
}

static void
pkt_burst_touch_enso(struct enso_stream *es)
{
	EnsoDevice_t* enso_device = es->enso_device;
	struct RXTXState rx_tx_state = es->rx_tx_state;
	// ENSO running process demo
	uint8_t* buf = NULL;

	uint32_t next_rx = rte_eth_rx_enso_next(enso_device);
	if(next_rx < 0) return;

	uint32_t new_bytes = rte_eth_rx_enso_burst(enso_device, &buf);
	assert(buf);
	if(new_bytes == 0) return;
	//printf("======== Recieve %u bytes from Rx pipe ========\n", new_bytes);

	// pkt burst touch logic for Enso 
	// similar as Descriptor based logic,
	// but little overhead for calculate packet_size, total_packets 
	const int packet_size = get_pkt_len(buf);

	#if defined(__ARM_NEON)
	do_touch_sve(enso_device->rx_pipe, buf, new_bytes, packet_size);
	#else	
	const int total_packets = (new_bytes / packet_size);
	for (int i = 0; i < total_packets; i++) {
		if (likely(i < total_packets - 1)) 
			rte_prefetch0((void*)(buf + (i + 1) * packet_size));
		
		char *pkt_data = (char *)(buf + i * packet_size);

		for (uint j = 0; j < packet_size; j++) {
			if (pkt_data[j] == 255)
				flag_touch = pkt_data[j];
		}
	}
	#endif

	// set up tx buffer
	uint8_t* tx_buf = rte_eth_alloc_tx_buffer(enso_device, new_bytes);
	assert(tx_buf);
	rx_tx_state.pending_tx.current_tx_buffer = tx_buf;
	rx_tx_state.pending_tx.start_tx_buffer = tx_buf;

	#if defined(__ARM_NEON)
	do_macswap_enso_neon(enso_device, &rx_tx_state, buf, new_bytes, packet_size);
	#else
	do_macswap_enso(enso_device->rx_pipe, &rx_tx_state, buf, new_bytes);
	#endif

	rte_eth_rx_enso_clear(enso_device);

	uint32_t tx_size = cal_tx_size(rx_tx_state.pending_tx.start_tx_buffer, rx_tx_state.pending_tx.current_tx_buffer);

	if (likely(tx_size > 0))
	{
		//printf("======== Send %u packets, %u bytes to Tx pipe ========\n",rx_tx_state.pending_tx.count, tx_size);
		rte_eth_tx_enso_burst(enso_device, tx_size);
	}
		
	rx_tx_state.pending_tx.count = 0;
	fflush(stdout);
}

struct fwd_engine touch_fwd_engine = {
	.fwd_mode_name  = "touchfwd",
	.port_fwd_begin = NULL,
	.port_fwd_end   = NULL,
	.packet_fwd     = pkt_burst_touch_enso,
};
#endif
