/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2018 Intel Corporation
 */

#ifndef _MACSWAP_COMMON_H_
#define _MACSWAP_COMMON_H_

static inline uint64_t
ol_flags_init(uint64_t tx_offload)
{
	uint64_t ol_flags = 0;

	ol_flags |= (tx_offload & DEV_TX_OFFLOAD_VLAN_INSERT) ?
			PKT_TX_VLAN : 0;
	ol_flags |= (tx_offload & DEV_TX_OFFLOAD_QINQ_INSERT) ?
			PKT_TX_QINQ : 0;
	ol_flags |= (tx_offload & DEV_TX_OFFLOAD_MACSEC_INSERT) ?
			PKT_TX_MACSEC : 0;

	return ol_flags;
}

static inline void
vlan_qinq_set(struct rte_mbuf *pkts[], uint16_t nb,
		uint64_t ol_flags, uint16_t vlan, uint16_t outer_vlan)
{
	int i;

	if (ol_flags & PKT_TX_VLAN)
		for (i = 0; i < nb; i++)
			pkts[i]->vlan_tci = vlan;
	if (ol_flags & PKT_TX_QINQ)
		for (i = 0; i < nb; i++)
			pkts[i]->vlan_tci_outer = outer_vlan;
}

static inline void
mbuf_field_set(struct rte_mbuf *mb, uint64_t ol_flags)
{
	mb->ol_flags &= IND_ATTACHED_MBUF | EXT_ATTACHED_MBUF;
	mb->ol_flags |= ol_flags;
	mb->l2_len = sizeof(struct rte_ether_hdr);
	mb->l3_len = sizeof(struct rte_ipv4_hdr);
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

uint8_t* get_next_pkt(uint8_t* pkt)
{
    uint32_t pkt_len = get_pkt_len(pkt);
    //uint32_t pkt_len = getMemcPktLen(pkt); // for memcached
    uint32_t nb_flits = (pkt_len - 1) / 64 + 1;
    //printf("[DEBUG] pkt_len: %u, nb_flits: %u\n", pkt_len, nb_flits);

    return pkt + nb_flits * 64;
}

static inline void 
do_macswap_enso(RxEnsoPipe_t* rx_pipe, struct RXTXState* rx_tx_state, uint8_t* rx_buf, uint32_t avail_bytes)
{
    uint8_t* addr = rx_buf;
    uint8_t* next_addr = get_next_pkt(rx_buf);
    uint8_t* end_of_buffer = (uint8_t*)rx_pipe->buf + ENSO_BUF_SIZE;

    uint32_t remaining_bytes = avail_bytes;

    // Debugging test, read memory barrier
    //rte_io_rmb();                           // memory barrier
    //volatile uint8_t dummy = *addr;        
    //(void)dummy;   
    //printf("[DEBUG] end_of_buffer %p \n", end_of_buffer);

    // for Enso MACSWAP, processing all packets from rx pipe
    while(remaining_bytes > 0)
    {
        // Test code
        // Copy RX to TX

        uint32_t consumed_bytes = next_addr - addr;
        assert(consumed_bytes > 0);

        // Test code : copy memory RX pipe -> TX pipe
        memcpy(rx_tx_state->pending_tx.current_tx_buffer, addr, consumed_bytes);
        
        // macswap operation
        
        struct rte_ether_hdr* l2_hdr = (struct rte_ether_hdr*)rx_tx_state->pending_tx.current_tx_buffer;
        struct rte_ether_addr original_src_mac = l2_hdr->s_addr;
        l2_hdr->s_addr = l2_hdr->d_addr;
        l2_hdr->d_addr = original_src_mac;
        
        rx_tx_state->pending_tx.current_tx_buffer += consumed_bytes;
        rx_tx_state->pending_tx.count++;

        // confirm rx byte
        rte_eth_rx_confirm_byte(rx_pipe, consumed_bytes);
        // Todo: update rx pipe state
        // onAdvanceMessage(consumed_bytes);

        addr = next_addr;

        // check addr wrap-around 
        if(addr >= end_of_buffer)
        {
            printf("[DEBUG] addr %p limit %p \n", addr, end_of_buffer);
            break;
        }

        next_addr = get_next_pkt(addr);

        remaining_bytes -= consumed_bytes;
        // maintain accumulated processed bytes??
        // NotifyProcessedBytes(consumed_bytes);
        // basic iterator implementation
        // add TX ??


    }

    printf("[DEBUG] process complete, receive %u bytes, remaining %u bytes\n", avail_bytes, remaining_bytes);
}


#endif

#endif /* _MACSWAP_COMMON_H_ */
