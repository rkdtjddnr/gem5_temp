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

#include <gem5/m5ops.h>
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

/*
 * MAC swap forwarding mode: Swap the source and the destination Ethernet
 * addresses of packets before forwarding them.
 */
static void
pkt_burst_mac_swap(struct fwd_stream *fs)
{
	struct rte_mbuf  *pkts_burst[MAX_PKT_BURST];
	struct rte_port  *txp;
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
	if (unlikely(nb_rx == 0)) {
		return;
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


#ifdef USE_ENSO


uint16_t be_to_le_16(const uint16_t le) {
  return ((le & (uint16_t)0x00ff) << 8) | ((le & (uint16_t)0xff00) >> 8);
}

uint16_t get_pkt_len(const uint8_t* addr) {
    const struct rte_ether_hdr* l2_hdr = (struct rte_ether_hdr*)addr;
    const struct rte_ipv4_hdr* l3_hdr = (struct rte_ipv4_hdr*)(l2_hdr + 1);
    const uint16_t total_len = be_to_le_16(l3_hdr->total_length) + sizeof(struct rte_ether_hdr);
    //printf("[DEBUG] host get_pkt_len func total_len %u \n", total_len);
    
    return total_len;
}

uint8_t* getNextPkt(uint8_t* pkt)
{
    uint32_t pkt_len = get_pkt_len(pkt);
    //uint32_t pkt_len = getMemcPktLen(pkt); // for memcached
    uint32_t nb_flits = (pkt_len - 1) / 64 + 1;
    //printf("[DEBUG] pkt_len: %u, nb_flits: %u\n", pkt_len, nb_flits);

    return pkt + nb_flits * 64;
}

void processBatchedPacket(RxEnsoPipe_t* rx_pipe, struct RXTXState* rxTxState, uint8_t* rx_buf, int32_t burstSize, uint32_t availByte)
{
    uint8_t* addr = rx_buf;
    uint8_t* next_addr = getNextPkt(rx_buf);
    uint8_t* end_of_buffer = (uint8_t*)rx_pipe->buf + ENSO_BUF_SIZE;

    int32_t missingMessages = burstSize;
    uint32_t remainingBytes = availByte;

    // Debugging test, read memory barrier
    //rte_io_rmb();                           // memory barrier
    //volatile uint8_t dummy = *addr;        // 강제 memory read
    //(void)dummy;   
    //printf("[DEBUG] end_of_buffer %p \n", end_of_buffer);

    // only process max burst size
    while((missingMessages > 0) && (remainingBytes > 0))
    {
        // Test code
        // Copy RX to TX

        uint32_t nbBytes = next_addr - addr;
        assert(nbBytes > 0);

        // Test code : copy memory RX pipe -> TX pipe
        memcpy(rxTxState->pending_tx.current_tx_buffer, addr, nbBytes);
        
        // macswap operation
        
        struct rte_ether_hdr* l2_hdr = (struct rte_ether_hdr*)rxTxState->pending_tx.current_tx_buffer;
        struct rte_ether_addr original_src_mac = l2_hdr->s_addr;
        l2_hdr->s_addr = l2_hdr->d_addr;
        l2_hdr->d_addr = original_src_mac;
        
        rxTxState->pending_tx.current_tx_buffer += nbBytes;
        rxTxState->pending_tx.count++;

        // confirm rx byte
        rte_eth_rx_confirm_byte(rx_pipe, nbBytes);
        // Todo: update rx pipe state
        // onAdvanceMessage(nbBytes);

        addr = next_addr;

        // check addr wrap-around 
        if(addr >= end_of_buffer)
        {
            printf("[DEBUG] addr %p limit %p \n", addr, end_of_buffer);
            break;
        }

        next_addr = getNextPkt(addr);

        remainingBytes -= nbBytes;
        --missingMessages;
        // maintain accumulated processed bytes??
        // NotifyProcessedBytes(nbBytes);
        // basic iterator implementation
        // add TX ??


    }

    printf("[DEBUG] process complete, remaining %u bytes, %u pkts\n", remainingBytes, missingMessages);
}


// change to enso_stream??
// struct enso_stream *es

static void
pkt_burst_mac_swap_enso(struct enso_stream *es)
{
	EnsoDevice_t* ensoDevice = es->ensoDevice;
	struct RXTXState rxTxState = es->rxTxState;
	// ENSO running process demo
	uint8_t* buf = NULL;
	uint32_t target_size = 1536*1024; // allocate size for TX buffer, need to change??

	uint32_t next_rx = rte_eth_rx_enso_next(ensoDevice);
	if(next_rx < 0) return;

	uint32_t newByte = rte_eth_rx_enso_burst(ensoDevice, &buf);
	assert(buf);
	if(newByte == 0) return;
	printf("======== Recieve %u bytes from Rx pipe ========\n", newByte);

	// set up tx buffer
	uint8_t* tx_buf = rte_eth_alloc_tx_buffer(ensoDevice, target_size);
	assert(tx_buf);
	rxTxState.pending_tx.current_tx_buffer = tx_buf;
	rxTxState.pending_tx.start_tx_buffer = tx_buf;

	processBatchedPacket(ensoDevice->rx_pipe, &rxTxState, buf, 1024, newByte);

	rte_eth_rx_enso_clear(ensoDevice);

	uint32_t tx_size = (rxTxState.pending_tx.current_tx_buffer - rxTxState.pending_tx.start_tx_buffer);

	if (tx_size > 0)
	{
		printf("======== Send %u packets, %u bytes to Tx pipe ========\n",rxTxState.pending_tx.count, tx_size);
		rte_eth_tx_enso_burst(ensoDevice, tx_size);
	}
		
	rxTxState.pending_tx.count = 0;
	fflush(stdout);
}
struct fwd_engine mac_swap_engine = {
	.fwd_mode_name  = "macswap",
	.port_fwd_begin = NULL,
	.port_fwd_end   = NULL,
	.packet_fwd     = pkt_burst_mac_swap_enso,
};
#else
struct fwd_engine mac_swap_engine = {
	.fwd_mode_name  = "macswap",
	.port_fwd_begin = NULL,
	.port_fwd_end   = NULL,
	.packet_fwd     = pkt_burst_mac_swap,
};
#endif


