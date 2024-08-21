/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright(c) 2010-2016 Intel Corporation
 */

#include <sys/queue.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <stdarg.h>
#include <inttypes.h>

#include <rte_interrupts.h>
#include <rte_byteorder.h>
#include <rte_common.h>
#include <rte_log.h>
#include <rte_debug.h>
#include <rte_pci.h>
#include <rte_bus_pci.h>
#include <rte_memory.h>
#include <rte_memcpy.h>
#include <rte_memzone.h>
#include <rte_launch.h>
#include <rte_eal.h>
#include <rte_per_lcore.h>
#include <rte_lcore.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_mempool.h>
#include <rte_malloc.h>
#include <rte_mbuf.h>
#include <rte_ether.h>
#include <rte_ethdev_driver.h>
#include <rte_prefetch.h>
#include <rte_ip.h>
#include <rte_udp.h>
#include <rte_tcp.h>
#include <rte_sctp.h>
#include <rte_net.h>
#include <rte_string_fns.h>

#include "e1000_logs.h"
#include "base/e1000_api.h"
#include "e1000_ethdev.h"
#include "base/e1000_osdep.h"

#define	E1000_TXD_VLAN_SHIFT	16

#define E1000_RXDCTL_GRAN	0x01000000 /* RXDCTL Granularity */

#define E1000_TX_OFFLOAD_MASK ( \
		PKT_TX_IPV6 |           \
		PKT_TX_IPV4 |           \
		PKT_TX_IP_CKSUM |       \
		PKT_TX_L4_MASK |        \
		PKT_TX_TCP_SEG |		 \
		PKT_TX_VLAN_PKT)

#define E1000_TX_OFFLOAD_NOTSUP_MASK \
		(PKT_TX_OFFLOAD_MASK ^ E1000_TX_OFFLOAD_MASK)

/* PCI offset for querying configuration status register */
#define PCI_CFG_STATUS_REG                 0x06
#define FLUSH_DESC_REQUIRED               0x100


/**
 * Structure associated with each descriptor of the RX ring of a RX queue.
 */
struct em_rx_entry {
	struct rte_mbuf *mbuf; /**< mbuf associated with RX descriptor. */
};

/**
 * Structure associated with each descriptor of the TX ring of a TX queue.
 */
struct em_tx_entry {
	struct rte_mbuf *mbuf; /**< mbuf associated with TX desc, if any. */
	uint16_t next_id; /**< Index of next descriptor in ring. */
	uint16_t last_id; /**< Index of last scattered descriptor. */
};

/**
 * Structure associated with each RX queue.
 */
struct em_rx_queue {
	struct rte_mempool  *mb_pool;   /**< mbuf pool to populate RX ring. */
	// volatile struct e1000_rx_desc *rx_ring; /**< RX ring virtual address. */
	volatile union e1000_adv_rx_desc *rx_ring; /**< RX ring virtual address. */ //jm
	uint64_t            rx_ring_phys_addr; /**< RX ring DMA address. */
	volatile uint32_t   *rdt_reg_addr; /**< RDT register address. */
	volatile uint32_t   *rdh_reg_addr; /**< RDH register address. */
	struct em_rx_entry *sw_ring;   /**< address of RX software ring. */
	struct rte_mbuf *pkt_first_seg; /**< First segment of current packet. */
	struct rte_mbuf *pkt_last_seg;  /**< Last segment of current packet. */
	uint64_t	    offloads;   /**< Offloads of DEV_RX_OFFLOAD_* */
	uint16_t            nb_rx_desc; /**< number of RX descriptors. */
	uint16_t            rx_tail;    /**< current value of RDT register. */
	uint16_t            nb_rx_hold; /**< number of held free RX desc. */
	uint16_t            rx_free_thresh; /**< max free RX desc to hold. */
	uint16_t            queue_id;   /**< RX queue index. */
	uint16_t            port_id;    /**< Device port identifier. */
	uint8_t             pthresh;    /**< Prefetch threshold register. */
	uint8_t             hthresh;    /**< Host threshold register. */
	uint8_t             wthresh;    /**< Write-back threshold register. */
	uint8_t             crc_len;    /**< 0 if CRC stripped, 4 otherwise. */
	uint32_t			flags;      /**< RX flags. */
};

/**
 * Hardware context number
 */
enum {
	EM_CTX_0    = 0, /**< CTX0 */
	EM_CTX_NUM  = 1, /**< CTX NUM */
};

/** Offload features */
union em_vlan_macip {
	uint64_t data;
	struct {
		uint64_t l3_len:9; /**< L3 (IP) Header Length. */
		uint64_t l2_len:7; /**< L2 (MAC) Header Length. */
		uint64_t vlan_tci:16;
		/**< VLAN Tag Control Identifier (CPU order). */
		uint64_t l4_len:8; /**< L4 (TCP/UDP) Header Length. */
		uint64_t tso_segsz:16; /**< TCP TSO segment size. */
	} f;
};

/*
 * Compare mask for vlan_macip_len.data,
 * should be in sync with em_vlan_macip.f layout.
 * */
#define TX_VLAN_CMP_MASK        0xFFFF0000  /**< VLAN length - 16-bits. */
#define TX_MAC_LEN_CMP_MASK     0x0000FE00  /**< MAC length - 7-bits. */
#define TX_IP_LEN_CMP_MASK      0x000001FF  /**< IP  length - 9-bits. */
/** MAC+IP  length. */
#define TX_MACIP_LEN_CMP_MASK   (TX_MAC_LEN_CMP_MASK | TX_IP_LEN_CMP_MASK) /**< L2L3 header mask. */

#define TX_TCP_LEN_CMP_MASK		0x000000FF00000000ULL /**< TCP header mask. */
#define TX_TSO_MSS_CMP_MASK		0x00FFFF0000000000ULL /**< TSO segsz mask. */
/** Mac + IP + TCP + Mss mask. */
#define TX_TSO_CMP_MASK	\
	(TX_MACIP_LEN_CMP_MASK | TX_TCP_LEN_CMP_MASK | TX_TSO_MSS_CMP_MASK)

/**
 * Structure to check if new context need be built
 */
struct em_ctx_info {
	uint64_t flags;              /**< ol_flags related to context build. */
	uint32_t cmp_mask;           /**< compare mask */
	union em_vlan_macip hdrlen;  /**< L2 and L3 header lenghts */
};

struct em_advctx_info {
	uint64_t flags;           /**< ol_flags related to context build. */
	/** tx offload: vlan, tso, l2-l3-l4 lengths. */
	union em_vlan_macip tx_offload; // it works as a hdrlen of legacy ctx
	/** compare mask for tx offload. */
	union em_vlan_macip tx_offload_mask;
};

/**
 * Structure associated with each TX queue.
 */
struct em_tx_queue {
	// volatile struct e1000_data_desc *tx_ring; /**< TX ring address */
	volatile union e1000_adv_tx_desc *tx_ring; /**< TX ring address */ //jm
	uint64_t               tx_ring_phys_addr; /**< TX ring DMA address. */
	struct em_tx_entry    *sw_ring; /**< virtual address of SW ring. */
	volatile uint32_t      *tdt_reg_addr; /**< Address of TDT register. */
	uint32_t               txd_type;      /**< Device-specific TXD type */
	uint16_t               nb_tx_desc;    /**< number of TX descriptors. */
	uint16_t               tx_tail;  /**< Current value of TDT register. */
	/**< Start freeing TX buffers if there are less free descriptors than
	     this value. */
	uint16_t               tx_free_thresh;
	/**< Number of TX descriptors to use before RS bit is set. */
	uint16_t               tx_rs_thresh;
	/** Number of TX descriptors used since RS bit was set. */
	uint16_t               nb_tx_used;
	/** Index to last TX descriptor to have been cleaned. */
	uint16_t	       last_desc_cleaned;
	/** Total number of TX descriptors ready to be allocated. */
	uint16_t               nb_tx_free;
	uint16_t               queue_id; /**< TX queue index. */
	uint16_t               port_id;  /**< Device port identifier. */
	uint8_t                pthresh;  /**< Prefetch threshold register. */
	uint8_t                hthresh;  /**< Host threshold register. */
	uint8_t                wthresh;  /**< Write-back threshold register. */
	// struct em_ctx_info ctx_cache;
	struct em_advctx_info ctx_cache; // jm
	/**< Hardware context history.*/
	uint64_t	       offloads; /**< offloads of DEV_TX_OFFLOAD_* */
};

#if 1
#define RTE_PMD_USE_PREFETCH
#endif

#ifdef RTE_PMD_USE_PREFETCH
#define rte_em_prefetch(p)	rte_prefetch0(p)
#else
#define rte_em_prefetch(p)	do {} while(0)
#endif

#ifdef RTE_PMD_PACKET_PREFETCH
#define rte_packet_prefetch(p) rte_prefetch1(p)
#else
#define rte_packet_prefetch(p)	do {} while(0)
#endif

#ifndef DEFAULT_TX_FREE_THRESH
#define DEFAULT_TX_FREE_THRESH  32
#endif /* DEFAULT_TX_FREE_THRESH */

#ifndef DEFAULT_TX_RS_THRESH
#define DEFAULT_TX_RS_THRESH  32
#endif /* DEFAULT_TX_RS_THRESH */

#define EM_TSO_MAX_HDRLEN			(512)
#define EM_TSO_MAX_MSS				(9216)


/*********************************************************************
 *
 *  TX function
 *
 **********************************************************************/
/*
 *There're some limitations in hardware for TCP segmentation offload. We
 *should check whether the parameters are valid.
 */
static inline uint64_t
check_tso_para(uint64_t ol_req, union em_vlan_macip ol_para)
{
	if (!(ol_req & PKT_TX_TCP_SEG))
		return ol_req;
	if ((ol_para.f.tso_segsz > EM_TSO_MAX_MSS) || (ol_para.f.l2_len +
			ol_para.f.l3_len + ol_para.f.l4_len > EM_TSO_MAX_HDRLEN)) {
		ol_req &= ~PKT_TX_TCP_SEG;
		ol_req |= PKT_TX_TCP_CKSUM;
	}
	return ol_req;
}

/*
 * Populates TX context descriptor.
 */
static inline void
em_set_xmit_ctx(struct em_tx_queue* txq,
		// volatile struct e1000_context_desc *ctx_txd,
		volatile struct e1000_adv_tx_context_desc *ctx_txd, //jm
		uint64_t ol_flags,
		union em_vlan_macip hdrlen)
{
	// uint32_t cmp_mask, cmd_len;
	// uint16_t ipcse, l2len;
	// // struct e1000_context_desc ctx;
	// struct e1000_adv_tx_context_desc ctx; //jm

	// cmp_mask = 0;
	// cmd_len = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_C;

	// l2len = hdrlen.f.l2_len;
	// ipcse = (uint16_t)(l2len + hdrlen.f.l3_len);

	// /* setup IPCS* fields */
	// ctx.lower_setup.ip_fields.ipcss = (uint8_t)l2len;
	// ctx.lower_setup.ip_fields.ipcso = (uint8_t)(l2len +
	// 		offsetof(struct rte_ipv4_hdr, hdr_checksum));

	// /*
	//  * When doing checksum or TCP segmentation with IPv6 headers,
	//  * IPCSE field should be set t0 0.
	//  */
	// if (flags & PKT_TX_IP_CKSUM) {
	// 	ctx.lower_setup.ip_fields.ipcse =
	// 		(uint16_t)rte_cpu_to_le_16(ipcse - 1);
	// 	cmd_len |= E1000_TXD_CMD_IP;
	// 	cmp_mask |= TX_MACIP_LEN_CMP_MASK;
	// } else {
	// 	ctx.lower_setup.ip_fields.ipcse = 0;
	// }

	// /* setup TUCS* fields */
	// ctx.upper_setup.tcp_fields.tucss = (uint8_t)ipcse;
	// ctx.upper_setup.tcp_fields.tucse = 0;

	// switch (flags & PKT_TX_L4_MASK) {
	// case PKT_TX_UDP_CKSUM:
	// 	ctx.upper_setup.tcp_fields.tucso = (uint8_t)(ipcse +
	// 			offsetof(struct rte_udp_hdr, dgram_cksum));
	// 	cmp_mask |= TX_MACIP_LEN_CMP_MASK;
	// 	break;
	// case PKT_TX_TCP_CKSUM:
	// 	ctx.upper_setup.tcp_fields.tucso = (uint8_t)(ipcse +
	// 			offsetof(struct rte_tcp_hdr, cksum));
	// 	cmd_len |= E1000_TXD_CMD_TCP;
	// 	cmp_mask |= TX_MACIP_LEN_CMP_MASK;
	// 	break;
	// default:
	// 	ctx.upper_setup.tcp_fields.tucso = 0;
	// }

	// ctx.cmd_and_length = rte_cpu_to_le_32(cmd_len);
	// ctx.tcp_seg_setup.data = 0;

	// *ctx_txd = ctx;

	uint32_t type_tucmd_mlhl;
	uint32_t mss_l4len_idx;
	uint32_t ctx_idx, ctx_curr;
	uint32_t vlan_macip_lens;
	union em_vlan_macip tx_offload_mask;

	// ctx_idx = ctx_curr + txq->ctx_start;
	ctx_idx = 0; //jm - we are not using multiple contexts

	tx_offload_mask.data = 0;
	type_tucmd_mlhl = 0;

	/* Specify which HW CTX to upload. */
	mss_l4len_idx = (ctx_idx << E1000_ADVTXD_IDX_SHIFT);

	if (ol_flags & PKT_TX_VLAN_PKT)
		tx_offload_mask.data |= TX_VLAN_CMP_MASK;

	/* check if TCP segmentation required for this packet */
	if (ol_flags & PKT_TX_TCP_SEG) {
		/* implies IP cksum in IPv4 */
		if (ol_flags & PKT_TX_IP_CKSUM)
			type_tucmd_mlhl = E1000_ADVTXD_TUCMD_IPV4 |
				E1000_ADVTXD_TUCMD_L4T_TCP |
				E1000_ADVTXD_DTYP_CTXT | E1000_ADVTXD_DCMD_DEXT;
		else
			type_tucmd_mlhl = E1000_ADVTXD_TUCMD_IPV6 |
				E1000_ADVTXD_TUCMD_L4T_TCP |
				E1000_ADVTXD_DTYP_CTXT | E1000_ADVTXD_DCMD_DEXT;

		tx_offload_mask.data |= TX_TSO_CMP_MASK;
		mss_l4len_idx |= hdrlen.f.tso_segsz << E1000_ADVTXD_MSS_SHIFT;
		mss_l4len_idx |= hdrlen.f.l4_len << E1000_ADVTXD_L4LEN_SHIFT;
	} else { /* no TSO, check if hardware checksum is needed */
		if (ol_flags & (PKT_TX_IP_CKSUM | PKT_TX_L4_MASK))
			tx_offload_mask.data |= TX_MACIP_LEN_CMP_MASK;

		if (ol_flags & PKT_TX_IP_CKSUM)
			type_tucmd_mlhl = E1000_ADVTXD_TUCMD_IPV4;

		switch (ol_flags & PKT_TX_L4_MASK) {
		case PKT_TX_UDP_CKSUM:
			type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_UDP |
				E1000_ADVTXD_DTYP_CTXT | E1000_ADVTXD_DCMD_DEXT;
			mss_l4len_idx |= sizeof(struct rte_udp_hdr)
				<< E1000_ADVTXD_L4LEN_SHIFT;
			break;
		case PKT_TX_TCP_CKSUM:
			type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_TCP |
				E1000_ADVTXD_DTYP_CTXT | E1000_ADVTXD_DCMD_DEXT;
			mss_l4len_idx |= sizeof(struct rte_tcp_hdr)
				<< E1000_ADVTXD_L4LEN_SHIFT;
			break;
		case PKT_TX_SCTP_CKSUM:
			type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_SCTP |
				E1000_ADVTXD_DTYP_CTXT | E1000_ADVTXD_DCMD_DEXT;
			mss_l4len_idx |= sizeof(struct rte_sctp_hdr)
				<< E1000_ADVTXD_L4LEN_SHIFT;
			break;
		default:
			type_tucmd_mlhl |= E1000_ADVTXD_TUCMD_L4T_RSV |
				E1000_ADVTXD_DTYP_CTXT | E1000_ADVTXD_DCMD_DEXT;
			break;
		}
	}

	txq->ctx_cache.flags = ol_flags;
	txq->ctx_cache.tx_offload.data =
		tx_offload_mask.data & hdrlen.data;
	txq->ctx_cache.tx_offload_mask = tx_offload_mask;

	ctx_txd->type_tucmd_mlhl = rte_cpu_to_le_32(type_tucmd_mlhl);
	vlan_macip_lens = (uint32_t)hdrlen.data;
	ctx_txd->vlan_macip_lens = rte_cpu_to_le_32(vlan_macip_lens);
	ctx_txd->mss_l4len_idx = rte_cpu_to_le_32(mss_l4len_idx);
	ctx_txd->u.seqnum_seed = 0;

	// txq->ctx_cache.flags = flags;
	// txq->ctx_cache.cmp_mask = cmp_mask;
	// txq->ctx_cache.hdrlen = hdrlen;
}

/*
 * Check which hardware context can be used. Use the existing match
 * or create a new context descriptor.
 */
static inline uint32_t
what_ctx_update(struct em_tx_queue *txq, uint64_t flags,
		union em_vlan_macip hdrlen)
{
	/* If match with the current context */
	if (likely (txq->ctx_cache.flags == flags &&
			(txq->ctx_cache.tx_offload.data ==
			(txq->ctx_cache.tx_offload_mask.data & hdrlen.data))))
			// ((txq->ctx_cache.hdrlen.data ^ hdrlen.data) &
			// txq->ctx_cache.cmp_mask) == 0))
		return EM_CTX_0;

	/* Mismatch */
	return EM_CTX_NUM;
}

/* Reset transmit descriptors after they have been used */
static inline int
em_xmit_cleanup(struct em_tx_queue *txq)
{
	struct em_tx_entry *sw_ring = txq->sw_ring;
	// volatile struct e1000_data_desc *txr = txq->tx_ring;
	volatile union e1000_adv_tx_desc *txr = txq->tx_ring; //jm
	uint16_t last_desc_cleaned = txq->last_desc_cleaned;
	uint16_t nb_tx_desc = txq->nb_tx_desc;
	uint16_t desc_to_clean_to;
	uint16_t nb_tx_to_clean;

	/* Determine the last descriptor needing to be cleaned */
	desc_to_clean_to = (uint16_t)(last_desc_cleaned + txq->tx_rs_thresh);
	if (desc_to_clean_to >= nb_tx_desc)
		desc_to_clean_to = (uint16_t)(desc_to_clean_to - nb_tx_desc);

	/* Check to make sure the last descriptor to clean is done */
	desc_to_clean_to = sw_ring[desc_to_clean_to].last_id;
	// if (! (txr[desc_to_clean_to].upper.fields.status & E1000_TXD_STAT_DD))
	if (! (txr[desc_to_clean_to].wb.status & E1000_TXD_STAT_DD)) //jm
	{
		PMD_TX_FREE_LOG(DEBUG,
				"TX descriptor %4u is not done"
				"(port=%d queue=%d)", desc_to_clean_to,
				txq->port_id, txq->queue_id);
		/* Failed to clean any descriptors, better luck next time */
		return -(1);
	}

	/* Figure out how many descriptors will be cleaned */
	if (last_desc_cleaned > desc_to_clean_to)
		nb_tx_to_clean = (uint16_t)((nb_tx_desc - last_desc_cleaned) +
							desc_to_clean_to);
	else
		nb_tx_to_clean = (uint16_t)(desc_to_clean_to -
						last_desc_cleaned);

	PMD_TX_FREE_LOG(DEBUG,
			"Cleaning %4u TX descriptors: %4u to %4u "
			"(port=%d queue=%d)", nb_tx_to_clean,
			last_desc_cleaned, desc_to_clean_to, txq->port_id,
			txq->queue_id);

	/*
	 * The last descriptor to clean is done, so that means all the
	 * descriptors from the last descriptor that was cleaned
	 * up to the last descriptor with the RS bit set
	 * are done. Only reset the threshold descriptor.
	 */
	// txr[desc_to_clean_to].upper.fields.status = 0;
	txr[desc_to_clean_to].wb.status = 0; //jm

	/* Update the txq to reflect the last descriptor that was cleaned */
	txq->last_desc_cleaned = desc_to_clean_to;
	txq->nb_tx_free = (uint16_t)(txq->nb_tx_free + nb_tx_to_clean);

	/* No Error */
	return 0;
}

static inline uint32_t
tx_desc_cksum_flags_to_upper(uint64_t ol_flags)
{
	static const uint32_t l4_olinfo[2] = {0, E1000_TXD_POPTS_TXSM << 8};
	static const uint32_t l3_olinfo[2] = {0, E1000_TXD_POPTS_IXSM << 8};
	uint32_t tmp;

	tmp = l4_olinfo[(ol_flags & PKT_TX_L4_MASK) != PKT_TX_L4_NO_CKSUM];
	tmp |= l3_olinfo[(ol_flags & PKT_TX_IP_CKSUM) != 0];
	return tmp;
}

static inline uint32_t
tx_desc_cksum_flags_to_olinfo(uint64_t ol_flags)
{
	static const uint32_t l4_olinfo[2] = {0, E1000_ADVTXD_POPTS_TXSM};
	static const uint32_t l3_olinfo[2] = {0, E1000_ADVTXD_POPTS_IXSM};
	uint32_t tmp;

	tmp  = l4_olinfo[(ol_flags & PKT_TX_L4_MASK)  != PKT_TX_L4_NO_CKSUM];
	tmp |= l3_olinfo[(ol_flags & PKT_TX_IP_CKSUM) != 0];
	tmp |= l4_olinfo[(ol_flags & PKT_TX_TCP_SEG) != 0];
	return tmp;
}

static inline uint32_t
tx_desc_vlan_flags_to_cmdtype(uint64_t ol_flags)
{
	uint32_t cmdtype;
	static uint32_t vlan_cmd[2] = {0, E1000_ADVTXD_DCMD_VLE};
	static uint32_t tso_cmd[2] = {0, E1000_ADVTXD_DCMD_TSE};
	cmdtype = vlan_cmd[(ol_flags & PKT_TX_VLAN_PKT) != 0];
	cmdtype |= tso_cmd[(ol_flags & PKT_TX_TCP_SEG) != 0];
	return cmdtype;
}

uint16_t
eth_em_xmit_pkts(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts)
{
	struct em_tx_queue *txq;
	struct em_tx_entry *sw_ring;
	struct em_tx_entry *txe, *txn;
	// volatile struct e1000_data_desc *txr;
	// volatile struct e1000_data_desc *txd;
	volatile union e1000_adv_tx_desc *txr; //jm
	volatile union e1000_adv_tx_desc *txd; //jm
	struct rte_mbuf     *tx_pkt;
	struct rte_mbuf     *m_seg;
	uint64_t buf_dma_addr;
	uint32_t olinfo_status;
	uint32_t popts_spec;
	uint32_t cmd_type_len;
	uint32_t pkt_len;
	uint16_t slen;
	uint64_t ol_flags;
	uint16_t tx_id;
	uint16_t tx_last;
	uint16_t nb_tx;
	uint16_t nb_used;
	uint64_t tx_ol_req;
	uint32_t ctx;
	uint32_t new_ctx;
	union em_vlan_macip hdrlen;

	txq = tx_queue;
	sw_ring = txq->sw_ring;
	txr     = txq->tx_ring;
	tx_id   = txq->tx_tail;
	txe = &sw_ring[tx_id];

	/* Determine if the descriptor ring needs to be cleaned. */
	 if (txq->nb_tx_free < txq->tx_free_thresh)
		em_xmit_cleanup(txq);

	/* TX loop */
	for (nb_tx = 0; nb_tx < nb_pkts; nb_tx++) {
		new_ctx = 0;
		tx_pkt = *tx_pkts++;
		pkt_len = tx_pkt->pkt_len;

		RTE_MBUF_PREFETCH_TO_FREE(txe->mbuf);

		/*
		 * Determine how many (if any) context descriptors
		 * are needed for offload functionality.
		 */
		ol_flags = tx_pkt->ol_flags;

		/* If hardware offload required */
		// tx_ol_req = (ol_flags & (PKT_TX_IP_CKSUM | PKT_TX_L4_MASK));
		tx_ol_req = (ol_flags & E1000_TX_OFFLOAD_MASK);
		if (tx_ol_req) {
			hdrlen.f.vlan_tci = tx_pkt->vlan_tci;
			hdrlen.f.l2_len = tx_pkt->l2_len;
			hdrlen.f.l3_len = tx_pkt->l3_len;
			//jm
			hdrlen.f.l4_len = tx_pkt->l4_len;
			hdrlen.f.tso_segsz = tx_pkt->tso_segsz;
			tx_ol_req = check_tso_para(tx_ol_req, hdrlen);
			/* If new context to be built or reuse the exist ctx. */
			ctx = what_ctx_update(txq, tx_ol_req, hdrlen);

			/* Only allocate context descriptor if required*/
			new_ctx = (ctx == EM_CTX_NUM);
		}

		/*
		 * Keep track of how many descriptors are used this loop
		 * This will always be the number of segments + the number of
		 * Context descriptors required to transmit the packet
		 */
		nb_used = (uint16_t)(tx_pkt->nb_segs + new_ctx);

		/*
		 * The number of descriptors that must be allocated for a
		 * packet is the number of segments of that packet, plus 1
		 * Context Descriptor for the hardware offload, if any.
		 * Determine the last TX descriptor to allocate in the TX ring
		 * for the packet, starting from the current position (tx_id)
		 * in the ring.
		 */
		tx_last = (uint16_t) (tx_id + nb_used - 1);

		/* Circular ring */
		if (tx_last >= txq->nb_tx_desc)
			tx_last = (uint16_t) (tx_last - txq->nb_tx_desc);

		PMD_TX_LOG(DEBUG, "port_id=%u queue_id=%u pktlen=%u"
			   " tx_first=%u tx_last=%u",
			   (unsigned) txq->port_id,
			   (unsigned) txq->queue_id,
			   (unsigned) tx_pkt->pkt_len,
			   (unsigned) tx_id,
			   (unsigned) tx_last);

		/*
		 * Make sure there are enough TX descriptors available to
		 * transmit the entire packet.
		 * nb_used better be less than or equal to txq->tx_rs_thresh
		 */
		//TODO - jm: don't need to check enough free descriptors like igb_rxtx.c?
		while (unlikely (nb_used > txq->nb_tx_free)) {
			PMD_TX_FREE_LOG(DEBUG, "Not enough free TX descriptors "
					"nb_used=%4u nb_free=%4u "
					"(port=%d queue=%d)",
					nb_used, txq->nb_tx_free,
					txq->port_id, txq->queue_id);

			if (em_xmit_cleanup(txq) != 0) {
				/* Could not clean any descriptors */
				if (nb_tx == 0)
					return 0;
				goto end_of_tx;
			}
		}

		/*
		 * By now there are enough free TX descriptors to transmit
		 * the packet.
		 */

		/*
		 * Set common flags of all TX Data Descriptors.
		 *
		 * The following bits must be set in all Data Descriptors:
		 *    - E1000_TXD_DTYP_DATA
		 *    - E1000_TXD_DTYP_DEXT
		 *
		 * The following bits must be set in the first Data Descriptor
		 * and are ignored in the other ones:
		 *    - E1000_TXD_POPTS_IXSM
		 *    - E1000_TXD_POPTS_TXSM
		 *
		 * The following bits must be set in the last Data Descriptor
		 * and are ignored in the other ones:
		 *    - E1000_TXD_CMD_VLE
		 *    - E1000_TXD_CMD_IFCS
		 *
		 * The following bits must only be set in the last Data
		 * Descriptor:
		 *   - E1000_TXD_CMD_EOP
		 *
		 * The following bits can be set in any Data Descriptor, but
		 * are only set in the last Data Descriptor:
		 *   - E1000_TXD_CMD_RS
		 */
		// cmd_type_len = E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D |
		// 	E1000_TXD_CMD_IFCS;
		// popts_spec = 0;

		// /* Set VLAN Tag offload fields. */
		// if (ol_flags & PKT_TX_VLAN_PKT) {
		// 	cmd_type_len |= E1000_TXD_CMD_VLE;
		// 	popts_spec = tx_pkt->vlan_tci << E1000_TXD_VLAN_SHIFT;
		// }

		//jm - advtxd
		/*
		 * Set common flags of all TX Data Descriptors.
		 *
		 * The following bits must be set in all Data Descriptors:
		 *   - E1000_ADVTXD_DTYP_DATA
		 *   - E1000_ADVTXD_DCMD_DEXT
		 *
		 * The following bits must be set in the first Data Descriptor
		 * and are ignored in the other ones:
		 *   - E1000_ADVTXD_DCMD_IFCS
		 *   - E1000_ADVTXD_MAC_1588
		 *   - E1000_ADVTXD_DCMD_VLE
		 *
		 * The following bits must only be set in the last Data
		 * Descriptor:
		 *   - E1000_TXD_CMD_EOP
		 *
		 * The following bits can be set in any Data Descriptor, but
		 * are only set in the last Data Descriptor:
		 *   - E1000_TXD_CMD_RS
		 */
		cmd_type_len = E1000_ADVTXD_DTYP_DATA |
			E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT;
		if (tx_ol_req & PKT_TX_TCP_SEG)
			pkt_len -= (tx_pkt->l2_len + tx_pkt->l3_len + tx_pkt->l4_len);
		olinfo_status = (pkt_len << E1000_ADVTXD_PAYLEN_SHIFT);

		if (tx_ol_req) {
			/*
			 * Setup the TX Context Descriptor if required
			 */
			if (new_ctx) {
				// volatile struct e1000_context_desc *ctx_txd;
				volatile struct e1000_adv_tx_context_desc *ctx_txd; //jm

				// ctx_txd = (volatile struct e1000_context_desc *)
				ctx_txd = (volatile struct e1000_adv_tx_context_desc *) //jm
					&txr[tx_id];

				txn = &sw_ring[txe->next_id];
				RTE_MBUF_PREFETCH_TO_FREE(txn->mbuf);

				if (txe->mbuf != NULL) {
					rte_pktmbuf_free_seg(txe->mbuf);
					txe->mbuf = NULL;
				}

				em_set_xmit_ctx(txq, ctx_txd, tx_ol_req,
					hdrlen);

				txe->last_id = tx_last;
				tx_id = txe->next_id;
				txe = txn;
			}

			/*
			 * Setup the TX Data Descriptor,
			 * This path will go through
			 * whatever new/reuse the context descriptor
			 */
			cmd_type_len  |= tx_desc_vlan_flags_to_cmdtype(tx_ol_req);
			olinfo_status |= tx_desc_cksum_flags_to_olinfo(tx_ol_req);
			olinfo_status |= (ctx << E1000_ADVTXD_IDX_SHIFT);
			// popts_spec |= tx_desc_cksum_flags_to_upper(ol_flags);
		}

		m_seg = tx_pkt;
		do {
			txd = &txr[tx_id];
			txn = &sw_ring[txe->next_id];

			if (txe->mbuf != NULL)
				rte_pktmbuf_free_seg(txe->mbuf);
			txe->mbuf = m_seg;

			/*
			 * Set up Transmit Data Descriptor.
			 */
			slen = m_seg->data_len;
			buf_dma_addr = rte_mbuf_data_iova(m_seg);

			// txd->buffer_addr = rte_cpu_to_le_64(buf_dma_addr);
			// txd->lower.data = rte_cpu_to_le_32(cmd_type_len | slen);
			// txd->upper.data = rte_cpu_to_le_32(popts_spec);
			
			//jm - advtxd
			txd->read.buffer_addr =
				rte_cpu_to_le_64(buf_dma_addr);
			txd->read.cmd_type_len =
				rte_cpu_to_le_32(cmd_type_len | slen);
			txd->read.olinfo_status =
				rte_cpu_to_le_32(olinfo_status);

			txe->last_id = tx_last;
			tx_id = txe->next_id;
			txe = txn;
			m_seg = m_seg->next;
		} while (m_seg != NULL);

		/*
		 * The last packet data descriptor needs End Of Packet (EOP)
		 */
		// cmd_type_len |= E1000_TXD_CMD_EOP;
		txq->nb_tx_used = (uint16_t)(txq->nb_tx_used + nb_used);
		txq->nb_tx_free = (uint16_t)(txq->nb_tx_free - nb_used);

		/* Set RS bit only on threshold packets' last descriptor */
		if (txq->nb_tx_used >= txq->tx_rs_thresh) {
			PMD_TX_FREE_LOG(DEBUG,
					"Setting RS bit on TXD id=%4u "
					"(port=%d queue=%d)",
					tx_last, txq->port_id, txq->queue_id);

			//jm - advtxd
			txd->read.cmd_type_len |= rte_cpu_to_le_32(E1000_TXD_CMD_RS);

			/* Update txq RS bit counters */
			txq->nb_tx_used = 0;
		}
		// txd->lower.data |= rte_cpu_to_le_32(cmd_type_len);

		//jm - advtxd
		txd->read.cmd_type_len |=
			rte_cpu_to_le_32(E1000_TXD_CMD_EOP);
	}
end_of_tx:
	rte_wmb();

	/*
	 * Set the Transmit Descriptor Tail (TDT)
	 */
	PMD_TX_LOG(DEBUG, "port_id=%u queue_id=%u tx_tail=%u nb_tx=%u",
		(unsigned) txq->port_id, (unsigned) txq->queue_id,
		(unsigned) tx_id, (unsigned) nb_tx);
	E1000_PCI_REG_WRITE_RELAXED(txq->tdt_reg_addr, tx_id);
	txq->tx_tail = tx_id;

	return nb_tx;
}

/*********************************************************************
 *
 *  TX prep functions
 *
 **********************************************************************/
uint16_t
eth_em_prep_pkts(__rte_unused void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts)
{
	int i, ret;
	struct rte_mbuf *m;

	for (i = 0; i < nb_pkts; i++) {
		m = tx_pkts[i];

		//jm - advtxd
		/* Check some limitations for TSO in hardware */
		if (m->ol_flags & PKT_TX_TCP_SEG)
			if ((m->tso_segsz > EM_TSO_MAX_MSS) ||
					(m->l2_len + m->l3_len + m->l4_len >
					EM_TSO_MAX_HDRLEN)) {
				rte_errno = EINVAL;
				return i;
			}

		if (m->ol_flags & E1000_TX_OFFLOAD_NOTSUP_MASK) {
			rte_errno = ENOTSUP;
			return i;
		}

#ifdef RTE_LIBRTE_ETHDEV_DEBUG
		ret = rte_validate_tx_offload(m);
		if (ret != 0) {
			rte_errno = -ret;
			return i;
		}
#endif
		ret = rte_net_intel_cksum_prepare(m);
		if (ret != 0) {
			rte_errno = -ret;
			return i;
		}
	}

	return i;
}

/*********************************************************************
 *
 *  RX functions
 *
 **********************************************************************/
#define EM_PACKET_TYPE_IPV4              0X01
#define EM_PACKET_TYPE_IPV4_TCP          0X11
#define EM_PACKET_TYPE_IPV4_UDP          0X21
#define EM_PACKET_TYPE_IPV4_SCTP         0X41
#define EM_PACKET_TYPE_IPV4_EXT          0X03
#define EM_PACKET_TYPE_IPV4_EXT_SCTP     0X43
#define EM_PACKET_TYPE_IPV6              0X04
#define EM_PACKET_TYPE_IPV6_TCP          0X14
#define EM_PACKET_TYPE_IPV6_UDP          0X24
#define EM_PACKET_TYPE_IPV6_EXT          0X0C
#define EM_PACKET_TYPE_IPV6_EXT_TCP      0X1C
#define EM_PACKET_TYPE_IPV6_EXT_UDP      0X2C
#define EM_PACKET_TYPE_IPV4_IPV6         0X05
#define EM_PACKET_TYPE_IPV4_IPV6_TCP     0X15
#define EM_PACKET_TYPE_IPV4_IPV6_UDP     0X25
#define EM_PACKET_TYPE_IPV4_IPV6_EXT     0X0D
#define EM_PACKET_TYPE_IPV4_IPV6_EXT_TCP 0X1D
#define EM_PACKET_TYPE_IPV4_IPV6_EXT_UDP 0X2D
#define EM_PACKET_TYPE_MAX               0X80
#define EM_PACKET_TYPE_MASK              0X7F
#define EM_PACKET_TYPE_SHIFT             0X04
static inline uint32_t
em_rxd_pkt_info_to_pkt_type(uint16_t pkt_info)
{
	static const uint32_t
		ptype_table[EM_PACKET_TYPE_MAX] __rte_cache_aligned = {
		[EM_PACKET_TYPE_IPV4] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4,
		[EM_PACKET_TYPE_IPV4_EXT] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4_EXT,
		[EM_PACKET_TYPE_IPV6] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6,
		[EM_PACKET_TYPE_IPV4_IPV6] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6,
		[EM_PACKET_TYPE_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6_EXT,
		[EM_PACKET_TYPE_IPV4_IPV6_EXT] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6_EXT,
		[EM_PACKET_TYPE_IPV4_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_TCP,
		[EM_PACKET_TYPE_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_TCP,
		[EM_PACKET_TYPE_IPV4_IPV6_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_TCP,
		[EM_PACKET_TYPE_IPV6_EXT_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6_EXT | RTE_PTYPE_L4_TCP,
		[EM_PACKET_TYPE_IPV4_IPV6_EXT_TCP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_TCP,
		[EM_PACKET_TYPE_IPV4_UDP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_UDP,
		[EM_PACKET_TYPE_IPV6_UDP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6 | RTE_PTYPE_L4_UDP,
		[EM_PACKET_TYPE_IPV4_IPV6_UDP] =  RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6 | RTE_PTYPE_INNER_L4_UDP,
		[EM_PACKET_TYPE_IPV6_EXT_UDP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV6_EXT | RTE_PTYPE_L4_UDP,
		[EM_PACKET_TYPE_IPV4_IPV6_EXT_UDP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_TUNNEL_IP |
			RTE_PTYPE_INNER_L3_IPV6_EXT | RTE_PTYPE_INNER_L4_UDP,
		[EM_PACKET_TYPE_IPV4_SCTP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4 | RTE_PTYPE_L4_SCTP,
		[EM_PACKET_TYPE_IPV4_EXT_SCTP] = RTE_PTYPE_L2_ETHER |
			RTE_PTYPE_L3_IPV4_EXT | RTE_PTYPE_L4_SCTP,
	};
	if (unlikely(pkt_info & E1000_RXDADV_PKTTYPE_ETQF))
		return RTE_PTYPE_UNKNOWN;

	pkt_info = (pkt_info >> EM_PACKET_TYPE_SHIFT) & EM_PACKET_TYPE_MASK;

	return ptype_table[pkt_info];
}

static inline uint64_t
rx_desc_hlen_type_rss_to_pkt_flags(struct em_rx_queue *rxq, uint32_t hl_tp_rs)
{
	uint64_t pkt_flags = ((hl_tp_rs & 0x0F) == 0) ? 0 : PKT_RX_RSS_HASH;

	RTE_SET_USED(rxq);
	return pkt_flags;
}

static inline uint64_t
rx_desc_status_to_pkt_flags(uint32_t rx_status)
{
	uint64_t pkt_flags;

	/* Check if VLAN present */
	pkt_flags = ((rx_status & E1000_RXD_STAT_VP) ?
		PKT_RX_VLAN | PKT_RX_VLAN_STRIPPED : 0);

	return pkt_flags;
}

static inline uint64_t
rx_desc_error_to_pkt_flags(uint32_t rx_error)
{
	// uint64_t pkt_flags = 0;

	// if (rx_error & E1000_RXD_ERR_IPE)
	// 	pkt_flags |= PKT_RX_IP_CKSUM_BAD;
	// if (rx_error & E1000_RXD_ERR_TCPE)
	// 	pkt_flags |= PKT_RX_L4_CKSUM_BAD;
	// return pkt_flags;

	/*
	 * Bit 30: IPE, IPv4 checksum error
	 * Bit 29: L4I, L4I integrity error
	 */

	static uint64_t error_to_pkt_flags_map[4] = {
		PKT_RX_IP_CKSUM_GOOD | PKT_RX_L4_CKSUM_GOOD,
		PKT_RX_IP_CKSUM_GOOD | PKT_RX_L4_CKSUM_BAD,
		PKT_RX_IP_CKSUM_BAD | PKT_RX_L4_CKSUM_GOOD,
		PKT_RX_IP_CKSUM_BAD | PKT_RX_L4_CKSUM_BAD
	};
	return error_to_pkt_flags_map[(rx_error >>
		E1000_RXD_ERR_CKSUM_BIT) & E1000_RXD_ERR_CKSUM_MSK];
}

uint16_t
eth_em_recv_pkts(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts)
{
	// volatile struct e1000_rx_desc *rx_ring;
	// volatile struct e1000_rx_desc *rxdp;
	volatile union e1000_adv_rx_desc *rx_ring;
	volatile union e1000_adv_rx_desc *rxdp;
	struct em_rx_queue *rxq;
	struct em_rx_entry *sw_ring;
	struct em_rx_entry *rxe;
	struct rte_mbuf *rxm;
	struct rte_mbuf *nmb;
	// struct e1000_rx_desc rxd;
	union e1000_adv_rx_desc rxd;
	uint64_t dma_addr;
	uint32_t staterr; //jm
	uint32_t hlen_type_rss; //jm
	uint16_t pkt_len;
	uint16_t rx_id;
	uint16_t nb_rx;
	uint16_t nb_hold;
	uint8_t status;
	uint64_t pkt_flags; //jm
	rxq = rx_queue;

	nb_rx = 0;
	nb_hold = 0;
	rx_id = rxq->rx_tail;
	rx_ring = rxq->rx_ring;
	sw_ring = rxq->sw_ring;
	while (nb_rx < nb_pkts) {
		/*
		 * The order of operations here is important as the DD status
		 * bit must not be read after any other descriptor fields.
		 * rx_ring and rxdp are pointing to volatile data so the order
		 * of accesses cannot be reordered by the compiler. If they were
		 * not volatile, they could be reordered which could lead to
		 * using invalid descriptor fields when read from rxd.
		 */
		rxdp = &rx_ring[rx_id];
		// status = rxdp->status;
		staterr = rxdp->wb.upper.status_error; //jm
		// if (! (status & E1000_RXD_STAT_DD))
		if (! (staterr & rte_cpu_to_le_32(E1000_RXD_STAT_DD))) //jm
			break;
		rxd = *rxdp;

		/*
		 * End of packet.
		 *
		 * If the E1000_RXD_STAT_EOP flag is not set, the RX packet is
		 * likely to be invalid and to be dropped by the various
		 * validation checks performed by the network stack.
		 *
		 * Allocate a new mbuf to replenish the RX ring descriptor.
		 * If the allocation fails:
		 *    - arrange for that RX descriptor to be the first one
		 *      being parsed the next time the receive function is
		 *      invoked [on the same queue].
		 *
		 *    - Stop parsing the RX ring and return immediately.
		 *
		 * This policy do not drop the packet received in the RX
		 * descriptor for which the allocation of a new mbuf failed.
		 * Thus, it allows that packet to be later retrieved if
		 * mbuf have been freed in the mean time.
		 * As a side effect, holding RX descriptors instead of
		 * systematically giving them back to the NIC may lead to
		 * RX ring exhaustion situations.
		 * However, the NIC can gracefully prevent such situations
		 * to happen by sending specific "back-pressure" flow control
		 * frames to its peer(s).
		 */
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_id=%u "
			   "status=0x%x pkt_len=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) staterr,
			//    (unsigned) rte_le_to_cpu_16(rxd.length));
			   (unsigned) rte_le_to_cpu_16(rxd.wb.upper.length)); //jm

		nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
		if (nmb == NULL) {
			PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u "
				   "queue_id=%u",
				   (unsigned) rxq->port_id,
				   (unsigned) rxq->queue_id);
			rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed++;
			break;
		}

		nb_hold++;
		rxe = &sw_ring[rx_id];
		rx_id++;
		if (rx_id == rxq->nb_rx_desc)
			rx_id = 0;

		/* Prefetch next mbuf while processing current one. */
		rte_em_prefetch(sw_ring[rx_id].mbuf);

		/*
		 * When next RX descriptor is on a cache-line boundary,
		 * prefetch the next 4 RX descriptors and the next 8 pointers
		 * to mbufs.
		 */
		if ((rx_id & 0x3) == 0) {
			rte_em_prefetch(&rx_ring[rx_id]);
			rte_em_prefetch(&sw_ring[rx_id]);
		}

		/* Rearm RXD: attach new mbuf and reset status to zero. */

		rxm = rxe->mbuf;
		rxe->mbuf = nmb;
		dma_addr =
			rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
		// rxdp->buffer_addr = dma_addr;
		// rxdp->status = 0;
		rxdp->read.hdr_addr = 0;
		rxdp->read.pkt_addr = dma_addr; //jm

		/*
		 * Initialize the returned mbuf.
		 * 1) setup generic mbuf fields:
		 *    - number of segments,
		 *    - next segment,
		 *    - packet length,
		 *    - RX port identifier.
		 * 2) integrate hardware offload data, if any:
		 *    - RSS flag & hash,
		 *    - IP checksum flag,
		 *    - VLAN TCI, if any,
		 *    - error flags.
		 */
		// pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.length) -
		pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.wb.upper.length) - //jm
				rxq->crc_len);
		rxm->data_off = RTE_PKTMBUF_HEADROOM;
		rte_packet_prefetch((char *)rxm->buf_addr + rxm->data_off);
		rxm->nb_segs = 1;
		rxm->next = NULL;
		rxm->pkt_len = pkt_len;
		rxm->data_len = pkt_len;
		rxm->port = rxq->port_id;

		//jm - rss
		rxm->hash.rss = rxd.wb.lower.hi_dword.rss;
		hlen_type_rss = rte_le_to_cpu_32(rxd.wb.lower.lo_dword.data);

		//jm	
		/*
		 * The vlan_tci field is only valid when PKT_RX_VLAN is
		 * set in the pkt_flags field and must be in CPU byte order.
		 */		
		if ((staterr & rte_cpu_to_le_32(E1000_RXDEXT_STATERR_LB)) &&
				(rxq->flags & 0x01)) {
			rxm->vlan_tci = rte_be_to_cpu_16(rxd.wb.upper.vlan);
		} else {
			rxm->vlan_tci = rte_le_to_cpu_16(rxd.wb.upper.vlan);
		}

		//jm - rss
		pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(rxq, hlen_type_rss);
		pkt_flags = pkt_flags | rx_desc_status_to_pkt_flags(staterr);
		pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(staterr);
		rxm->ol_flags = pkt_flags;
		rxm->packet_type = em_rxd_pkt_info_to_pkt_type(rxd.wb.lower.
						lo_dword.hs_rss.pkt_info);
		// rxm->ol_flags = rx_desc_status_to_pkt_flags(status);
		// rxm->ol_flags = rxm->ol_flags |
		// 		rx_desc_error_to_pkt_flags(rxd.errors);

		/* Only valid if PKT_RX_VLAN set in pkt_flags */
		// rxm->vlan_tci = rte_le_to_cpu_16(rxd.special);

		/*
		 * Store the mbuf address into the next entry of the array
		 * of returned packets.
		 */
		rx_pkts[nb_rx++] = rxm;
	}
	rxq->rx_tail = rx_id;

	/*
	 * If the number of free RX descriptors is greater than the RX free
	 * threshold of the queue, advance the Receive Descriptor Tail (RDT)
	 * register.
	 * Update the RDT with the value of the last processed RX descriptor
	 * minus 1, to guarantee that the RDT register is never equal to the
	 * RDH register, which creates a "full" ring situtation from the
	 * hardware point of view...
	 */
	nb_hold = (uint16_t) (nb_hold + rxq->nb_rx_hold);
	if (nb_hold > rxq->rx_free_thresh) {
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_tail=%u "
			   "nb_hold=%u nb_rx=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) nb_hold,
			   (unsigned) nb_rx);
		rx_id = (uint16_t) ((rx_id == 0) ?
			(rxq->nb_rx_desc - 1) : (rx_id - 1));
		E1000_PCI_REG_WRITE(rxq->rdt_reg_addr, rx_id);
		nb_hold = 0;
	}
	rxq->nb_rx_hold = nb_hold;

	struct rte_ether_hdr *eth_hdr;
	struct rte_mbuf *mb;
	int i;
	for (i = 0; i < nb_rx; i++) {
		mb = rx_pkts[i];
		eth_hdr = rte_pktmbuf_mtod(mb, struct rte_ether_hdr *);
	}
	
	return nb_rx;
}

uint16_t
eth_em_recv_scattered_pkts(void *rx_queue, struct rte_mbuf **rx_pkts,
			 uint16_t nb_pkts)
{
	struct em_rx_queue *rxq;
	// volatile struct e1000_rx_desc *rx_ring;
	// volatile struct e1000_rx_desc *rxdp;
	volatile union e1000_adv_rx_desc *rx_ring;
	volatile union e1000_adv_rx_desc *rxdp;
	struct em_rx_entry *sw_ring;
	struct em_rx_entry *rxe;
	struct rte_mbuf *first_seg;
	struct rte_mbuf *last_seg;
	struct rte_mbuf *rxm;
	struct rte_mbuf *nmb;
	// struct e1000_rx_desc rxd;
	union e1000_adv_rx_desc rxd;
	uint64_t dma; /* Physical address of mbuf data buffer */
	uint32_t staterr;
	uint32_t hlen_type_rss;
	uint16_t rx_id;
	uint16_t nb_rx;
	uint16_t nb_hold;
	uint16_t data_len;
	uint8_t status;
	uint64_t pkt_flags;

	rxq = rx_queue;

	nb_rx = 0;
	nb_hold = 0;
	rx_id = rxq->rx_tail;
	rx_ring = rxq->rx_ring;
	sw_ring = rxq->sw_ring;

	/*
	 * Retrieve RX context of current packet, if any.
	 */
	first_seg = rxq->pkt_first_seg;
	last_seg = rxq->pkt_last_seg;

	while (nb_rx < nb_pkts) {
	next_desc:
		/*
		 * The order of operations here is important as the DD status
		 * bit must not be read after any other descriptor fields.
		 * rx_ring and rxdp are pointing to volatile data so the order
		 * of accesses cannot be reordered by the compiler. If they were
		 * not volatile, they could be reordered which could lead to
		 * using invalid descriptor fields when read from rxd.
		 */
		rxdp = &rx_ring[rx_id];
		// status = rxdp->status;
		staterr = rxdp->wb.upper.status_error;
		if (! (staterr & rte_cpu_to_le_32(E1000_RXD_STAT_DD)))
			break;
		rxd = *rxdp;

		/*
		 * Descriptor done.
		 *
		 * Allocate a new mbuf to replenish the RX ring descriptor.
		 * If the allocation fails:
		 *    - arrange for that RX descriptor to be the first one
		 *      being parsed the next time the receive function is
		 *      invoked [on the same queue].
		 *
		 *    - Stop parsing the RX ring and return immediately.
		 *
		 * This policy does not drop the packet received in the RX
		 * descriptor for which the allocation of a new mbuf failed.
		 * Thus, it allows that packet to be later retrieved if
		 * mbuf have been freed in the mean time.
		 * As a side effect, holding RX descriptors instead of
		 * systematically giving them back to the NIC may lead to
		 * RX ring exhaustion situations.
		 * However, the NIC can gracefully prevent such situations
		 * to happen by sending specific "back-pressure" flow control
		 * frames to its peer(s).
		 */
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_id=%u "
			   "status=0x%x data_len=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) staterr,
			   (unsigned) rte_le_to_cpu_16(rxd.wb.upper.length));
			//    (unsigned) rte_le_to_cpu_16(rxd.length));

		nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
		if (nmb == NULL) {
			PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u "
				   "queue_id=%u", (unsigned) rxq->port_id,
				   (unsigned) rxq->queue_id);
			rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed++;
			break;
		}

		nb_hold++;
		rxe = &sw_ring[rx_id];
		rx_id++;
		if (rx_id == rxq->nb_rx_desc)
			rx_id = 0;

		/* Prefetch next mbuf while processing current one. */
		rte_em_prefetch(sw_ring[rx_id].mbuf);

		/*
		 * When next RX descriptor is on a cache-line boundary,
		 * prefetch the next 4 RX descriptors and the next 8 pointers
		 * to mbufs.
		 */
		if ((rx_id & 0x3) == 0) {
			rte_em_prefetch(&rx_ring[rx_id]);
			rte_em_prefetch(&sw_ring[rx_id]);
		}

		/*
		 * Update RX descriptor with the physical address of the new
		 * data buffer of the new allocated mbuf.
		 */
		rxm = rxe->mbuf;
		rxe->mbuf = nmb;
		dma = rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
		// rxdp->buffer_addr = dma;
		// rxdp->status = 0;
		rxdp->read.pkt_addr = dma;
		rxdp->read.hdr_addr = 0;

		/*
		 * Set data length & data buffer address of mbuf.
		 */
		// data_len = rte_le_to_cpu_16(rxd.length);
		data_len = rte_le_to_cpu_16(rxd.wb.upper.length);
		rxm->data_len = data_len;
		rxm->data_off = RTE_PKTMBUF_HEADROOM;

		/*
		 * If this is the first buffer of the received packet,
		 * set the pointer to the first mbuf of the packet and
		 * initialize its context.
		 * Otherwise, update the total length and the number of segments
		 * of the current scattered packet, and update the pointer to
		 * the last mbuf of the current packet.
		 */
		if (first_seg == NULL) {
			first_seg = rxm;
			first_seg->pkt_len = data_len;
			first_seg->nb_segs = 1;
		} else {
			first_seg->pkt_len += data_len;
			first_seg->nb_segs++;
			last_seg->next = rxm;
		}

		/*
		 * If this is not the last buffer of the received packet,
		 * update the pointer to the last mbuf of the current scattered
		 * packet and continue to parse the RX ring.
		 */
		if (! (status & E1000_RXD_STAT_EOP)) {
			last_seg = rxm;
			goto next_desc;
		}

		/*
		 * This is the last buffer of the received packet.
		 * If the CRC is not stripped by the hardware:
		 *   - Subtract the CRC	length from the total packet length.
		 *   - If the last buffer only contains the whole CRC or a part
		 *     of it, free the mbuf associated to the last buffer.
		 *     If part of the CRC is also contained in the previous
		 *     mbuf, subtract the length of that CRC part from the
		 *     data length of the previous mbuf.
		 */
		rxm->next = NULL;
		if (unlikely(rxq->crc_len > 0)) {
			first_seg->pkt_len -= RTE_ETHER_CRC_LEN;
			if (data_len <= RTE_ETHER_CRC_LEN) {
				rte_pktmbuf_free_seg(rxm);
				first_seg->nb_segs--;
				last_seg->data_len = (uint16_t)
					(last_seg->data_len -
					 (RTE_ETHER_CRC_LEN - data_len));
				last_seg->next = NULL;
			} else
				rxm->data_len = (uint16_t)
					(data_len - RTE_ETHER_CRC_LEN);
		}

		/*
		 * Initialize the first mbuf of the returned packet:
		 *    - RX port identifier,
		 *    - hardware offload data, if any:
		 *    	- RSS flag & hash,
		 *      - IP checksum flag,
		 *      - error flags.
		 */
		first_seg->port = rxq->port_id;
		first_seg->hash.rss = rxd.wb.lower.hi_dword.rss;

		//jm - rss
		/*
		 * The vlan_tci field is only valid when PKT_RX_VLAN is
		 * set in the pkt_flags field and must be in CPU byte order.
		 */
		if ((staterr & rte_cpu_to_le_32(E1000_RXDEXT_STATERR_LB)) &&
				(rxq->flags & 0x01)) { //IGB_RXQ_FLAG_LB_BSWAP_VLAN
			first_seg->vlan_tci =
				rte_be_to_cpu_16(rxd.wb.upper.vlan);
		} else {
			first_seg->vlan_tci =
				rte_le_to_cpu_16(rxd.wb.upper.vlan);
		}
		hlen_type_rss = rte_le_to_cpu_32(rxd.wb.lower.lo_dword.data);
		pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(rxq, hlen_type_rss);
		pkt_flags = pkt_flags | rx_desc_status_to_pkt_flags(staterr);
		pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(staterr);
		first_seg->ol_flags = pkt_flags;
		first_seg->packet_type = em_rxd_pkt_info_to_pkt_type(
			rxd.wb.lower.lo_dword.hs_rss.pkt_info);

		// first_seg->ol_flags = rx_desc_status_to_pkt_flags(status);
		// first_seg->ol_flags = first_seg->ol_flags |
		// 			rx_desc_error_to_pkt_flags(rxd.errors);

		/* Only valid if PKT_RX_VLAN set in pkt_flags */
		// rxm->vlan_tci = rte_le_to_cpu_16(rxd.special);

		/* Prefetch data of first segment, if configured to do so. */
		rte_packet_prefetch((char *)first_seg->buf_addr +
			first_seg->data_off);

		/*
		 * Store the mbuf address into the next entry of the array
		 * of returned packets.
		 */
		rx_pkts[nb_rx++] = first_seg;

		/*
		 * Setup receipt context for a new packet.
		 */
		first_seg = NULL;
	}

	/*
	 * Record index of the next RX descriptor to probe.
	 */
	rxq->rx_tail = rx_id;

	/*
	 * Save receive context.
	 */
	rxq->pkt_first_seg = first_seg;
	rxq->pkt_last_seg = last_seg;

	/*
	 * If the number of free RX descriptors is greater than the RX free
	 * threshold of the queue, advance the Receive Descriptor Tail (RDT)
	 * register.
	 * Update the RDT with the value of the last processed RX descriptor
	 * minus 1, to guarantee that the RDT register is never equal to the
	 * RDH register, which creates a "full" ring situtation from the
	 * hardware point of view...
	 */
	nb_hold = (uint16_t) (nb_hold + rxq->nb_rx_hold);
	if (nb_hold > rxq->rx_free_thresh) {
		PMD_RX_LOG(DEBUG, "port_id=%u queue_id=%u rx_tail=%u "
			   "nb_hold=%u nb_rx=%u",
			   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
			   (unsigned) rx_id, (unsigned) nb_hold,
			   (unsigned) nb_rx);
		rx_id = (uint16_t) ((rx_id == 0) ?
			(rxq->nb_rx_desc - 1) : (rx_id - 1));
		E1000_PCI_REG_WRITE(rxq->rdt_reg_addr, rx_id);
		nb_hold = 0;
	}
	rxq->nb_rx_hold = nb_hold;

	
	//struct rte_ether_hdr *eth_hdr;
	//struct rte_mbuf *mb;
	//int i;
	//for (i = 0; i < nb_rx; i++) {
	//	mb = rx_pkts[i];
	//	eth_hdr = rte_pktmbuf_mtod(mb, struct rte_ether_hdr *);
	//	printf("dest %s \n", eth_hdr->d_addr.addr_bytes);
	//	printf("source %s \n", eth_hdr->s_addr.addr_bytes);
	//}

	return nb_rx;


}

#define	EM_MAX_BUF_SIZE     16384
#define EM_RCTL_FLXBUF_STEP 1024

static void
em_tx_queue_release_mbufs(struct em_tx_queue *txq)
{
	unsigned i;

	if (txq->sw_ring != NULL) {
		for (i = 0; i != txq->nb_tx_desc; i++) {
			if (txq->sw_ring[i].mbuf != NULL) {
				rte_pktmbuf_free_seg(txq->sw_ring[i].mbuf);
				txq->sw_ring[i].mbuf = NULL;
			}
		}
	}
}

static void
em_tx_queue_release(struct em_tx_queue *txq)
{
	if (txq != NULL) {
		em_tx_queue_release_mbufs(txq);
		rte_free(txq->sw_ring);
		rte_free(txq);
	}
}

void
eth_em_tx_queue_release(void *txq)
{
	em_tx_queue_release(txq);
}

/* (Re)set dynamic em_tx_queue fields to defaults */
static void
em_reset_tx_queue(struct em_tx_queue *txq)
{
	uint16_t i, nb_desc, prev;
	// static const struct e1000_data_desc txd_init = {
	// 	.upper.fields = {.status = E1000_TXD_STAT_DD},
	// };
	static const union e1000_adv_tx_desc zeroed_desc = {{0}};

	nb_desc = txq->nb_tx_desc;

	/* Initialize ring entries */

	prev = (uint16_t) (nb_desc - 1);

	for (i = 0; i < nb_desc; i++) {
		txq->tx_ring[i] = zeroed_desc;
		//jm
		volatile union e1000_adv_tx_desc *txd = &(txq->tx_ring[i]);
		txd->wb.status = E1000_TXD_STAT_DD;

		txq->sw_ring[i].mbuf = NULL;
		txq->sw_ring[i].last_id = i;
		txq->sw_ring[prev].next_id = i;
		prev = i;
	}
	//jm
	txq->txd_type = E1000_ADVTXD_DTYP_DATA;

	/*
	 * Always allow 1 descriptor to be un-allocated to avoid
	 * a H/W race condition
	 */
	txq->nb_tx_free = (uint16_t)(nb_desc - 1);
	txq->last_desc_cleaned = (uint16_t)(nb_desc - 1);
	txq->nb_tx_used = 0;
	txq->tx_tail = 0;

	memset((void*)&txq->ctx_cache, 0, sizeof (txq->ctx_cache));
}

uint64_t
em_get_tx_port_offloads_capa(struct rte_eth_dev *dev)
{
	uint64_t tx_offload_capa;

	RTE_SET_USED(dev);
	tx_offload_capa =
		DEV_TX_OFFLOAD_MULTI_SEGS  |
		DEV_TX_OFFLOAD_VLAN_INSERT |
		DEV_TX_OFFLOAD_IPV4_CKSUM  |
		DEV_TX_OFFLOAD_UDP_CKSUM   |
		DEV_TX_OFFLOAD_TCP_CKSUM;

	return tx_offload_capa;
}

uint64_t
em_get_tx_queue_offloads_capa(struct rte_eth_dev *dev)
{
	uint64_t tx_queue_offload_capa;

	/*
	 * As only one Tx queue can be used, let per queue offloading
	 * capability be same to per port queue offloading capability
	 * for better convenience.
	 */
	tx_queue_offload_capa = em_get_tx_port_offloads_capa(dev);

	return tx_queue_offload_capa;
}

int
eth_em_tx_queue_setup(struct rte_eth_dev *dev,
			 uint16_t queue_idx,
			 uint16_t nb_desc,
			 unsigned int socket_id,
			 const struct rte_eth_txconf *tx_conf)
{
	const struct rte_memzone *tz;
	struct em_tx_queue *txq;
	struct e1000_hw     *hw;
	uint32_t tsize;
	uint16_t tx_rs_thresh, tx_free_thresh;
	uint64_t offloads;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	offloads = tx_conf->offloads | dev->data->dev_conf.txmode.offloads;

	/*
	 * Validate number of transmit descriptors.
	 * It must not exceed hardware maximum, and must be multiple
	 * of E1000_ALIGN.
	 */
	if (nb_desc % EM_TXD_ALIGN != 0 ||
			(nb_desc > E1000_MAX_RING_DESC) ||
			(nb_desc < E1000_MIN_RING_DESC)) {
		return -(EINVAL);
	}

	tx_free_thresh = tx_conf->tx_free_thresh;
	if (tx_free_thresh == 0)
		tx_free_thresh = (uint16_t)RTE_MIN(nb_desc / 4,
					DEFAULT_TX_FREE_THRESH);

	tx_rs_thresh = tx_conf->tx_rs_thresh;
	if (tx_rs_thresh == 0)
		tx_rs_thresh = (uint16_t)RTE_MIN(tx_free_thresh,
					DEFAULT_TX_RS_THRESH);

	if (tx_free_thresh >= (nb_desc - 3)) {
		PMD_INIT_LOG(ERR, "tx_free_thresh must be less than the "
			     "number of TX descriptors minus 3. "
			     "(tx_free_thresh=%u port=%d queue=%d)",
			     (unsigned int)tx_free_thresh,
			     (int)dev->data->port_id, (int)queue_idx);
		return -(EINVAL);
	}
	if (tx_rs_thresh > tx_free_thresh) {
		PMD_INIT_LOG(ERR, "tx_rs_thresh must be less than or equal to "
			     "tx_free_thresh. (tx_free_thresh=%u "
			     "tx_rs_thresh=%u port=%d queue=%d)",
			     (unsigned int)tx_free_thresh,
			     (unsigned int)tx_rs_thresh,
			     (int)dev->data->port_id,
			     (int)queue_idx);
		return -(EINVAL);
	}

	/*
	 * If rs_bit_thresh is greater than 1, then TX WTHRESH should be
	 * set to 0. If WTHRESH is greater than zero, the RS bit is ignored
	 * by the NIC and all descriptors are written back after the NIC
	 * accumulates WTHRESH descriptors.
	 */
	if (tx_conf->tx_thresh.wthresh != 0 && tx_rs_thresh != 1) {
		PMD_INIT_LOG(ERR, "TX WTHRESH must be set to 0 if "
			     "tx_rs_thresh is greater than 1. (tx_rs_thresh=%u "
			     "port=%d queue=%d)", (unsigned int)tx_rs_thresh,
			     (int)dev->data->port_id, (int)queue_idx);
		return -(EINVAL);
	}

	/* Free memory prior to re-allocation if needed... */
	if (dev->data->tx_queues[queue_idx] != NULL) {
		em_tx_queue_release(dev->data->tx_queues[queue_idx]);
		dev->data->tx_queues[queue_idx] = NULL;
	}

	/* First allocate the tx queue data structure */
	txq = rte_zmalloc("ethdev TX queue", sizeof(struct em_tx_queue),
							RTE_CACHE_LINE_SIZE);
	if (txq == NULL)
		return -ENOMEM;

	/*
	 * Allocate TX ring hardware descriptors. A memzone large enough to
	 * handle the maximum ring size is allocated in order to allow for
	 * resizing in later calls to the queue setup function.
	 */
	tsize = sizeof(txq->tx_ring[0]) * E1000_MAX_RING_DESC;
	tz = rte_eth_dma_zone_reserve(dev, "tx_ring", queue_idx, tsize,
				    //   RTE_CACHE_LINE_SIZE, socket_id);
					E1000_ALIGN, socket_id);
	if (tz == NULL)
		return -ENOMEM;

	// /* Allocate the tx queue data structure. */
	// if ((txq = rte_zmalloc("ethdev TX queue", sizeof(*txq),
	// 		RTE_CACHE_LINE_SIZE)) == NULL)
	// 	return -ENOMEM;

	/* Allocate software ring */
	if ((txq->sw_ring = rte_zmalloc("txq->sw_ring",
			sizeof(txq->sw_ring[0]) * nb_desc,
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_tx_queue_release(txq);
		return -ENOMEM;
	}

	txq->nb_tx_desc = nb_desc;
	txq->tx_free_thresh = tx_free_thresh;
	txq->tx_rs_thresh = tx_rs_thresh;
	txq->pthresh = tx_conf->tx_thresh.pthresh;
	txq->hthresh = tx_conf->tx_thresh.hthresh;
	txq->wthresh = tx_conf->tx_thresh.wthresh;
	txq->queue_id = queue_idx;
	txq->port_id = dev->data->port_id;

	txq->tdt_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_TDT(queue_idx));
	txq->tx_ring_phys_addr = tz->iova;
	// txq->tx_ring = (struct e1000_data_desc *) tz->addr;
	txq->tx_ring = (union e1000_adv_tx_desc *) tz->addr; //jm

	PMD_INIT_LOG(DEBUG, "sw_ring=%p hw_ring=%p dma_addr=0x%"PRIx64,
		     txq->sw_ring, txq->tx_ring, txq->tx_ring_phys_addr);

	em_reset_tx_queue(txq);

	dev->data->tx_queues[queue_idx] = txq;
	txq->offloads = offloads;
	return 0;
}

static void
em_rx_queue_release_mbufs(struct em_rx_queue *rxq)
{
	unsigned i;

	if (rxq->sw_ring != NULL) {
		for (i = 0; i != rxq->nb_rx_desc; i++) {
			if (rxq->sw_ring[i].mbuf != NULL) {
				rte_pktmbuf_free_seg(rxq->sw_ring[i].mbuf);
				rxq->sw_ring[i].mbuf = NULL;
			}
		}
	}
}

static void
em_rx_queue_release(struct em_rx_queue *rxq)
{
	if (rxq != NULL) {
		em_rx_queue_release_mbufs(rxq);
		rte_free(rxq->sw_ring);
		rte_free(rxq);
	}
}

void
eth_em_rx_queue_release(void *rxq)
{
	em_rx_queue_release(rxq);
}

/* Reset dynamic em_rx_queue fields back to defaults */
static void
em_reset_rx_queue(struct em_rx_queue *rxq)
{
	static const union e1000_adv_rx_desc zeroed_desc = {{0}};
	unsigned i;

	/* Zero out HW ring memory */
	for (i = 0; i < rxq->nb_rx_desc; i++) {
		rxq->rx_ring[i] = zeroed_desc;
	}

	rxq->rx_tail = 0;
	rxq->nb_rx_hold = 0;
	rxq->pkt_first_seg = NULL;
	rxq->pkt_last_seg = NULL;
}

uint64_t
em_get_rx_port_offloads_capa(struct rte_eth_dev *dev)
{
	uint64_t rx_offload_capa;
	uint32_t max_rx_pktlen;

	max_rx_pktlen = em_get_max_pktlen(dev);

	rx_offload_capa =
		DEV_RX_OFFLOAD_VLAN_STRIP  |
		DEV_RX_OFFLOAD_VLAN_FILTER |
		DEV_RX_OFFLOAD_IPV4_CKSUM  |
		DEV_RX_OFFLOAD_UDP_CKSUM   |
		DEV_RX_OFFLOAD_TCP_CKSUM   |
		DEV_RX_OFFLOAD_KEEP_CRC    |
		DEV_RX_OFFLOAD_SCATTER	   |
		DEV_RX_OFFLOAD_RSS_HASH;

	if (max_rx_pktlen > RTE_ETHER_MAX_LEN)
		rx_offload_capa |= DEV_RX_OFFLOAD_JUMBO_FRAME;

	return rx_offload_capa;
}

uint64_t
em_get_rx_queue_offloads_capa(struct rte_eth_dev *dev)
{
	uint64_t rx_queue_offload_capa;

	/*
	 * As only one Rx queue can be used, let per queue offloading
	 * capability be same to per port queue offloading capability
	 * for better convenience.
	 */
	//TODO - jm: need to check this. only one Rx queue..?
	rx_queue_offload_capa = em_get_rx_port_offloads_capa(dev);

	return rx_queue_offload_capa;
}

int
eth_em_rx_queue_setup(struct rte_eth_dev *dev,
		uint16_t queue_idx,
		uint16_t nb_desc,
		unsigned int socket_id,
		const struct rte_eth_rxconf *rx_conf,
		struct rte_mempool *mp)
{
	const struct rte_memzone *rz;
	struct em_rx_queue *rxq;
	struct e1000_hw     *hw;
	uint32_t rsize;
	uint64_t offloads;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	offloads = rx_conf->offloads | dev->data->dev_conf.rxmode.offloads;

	/*
	 * Validate number of receive descriptors.
	 * It must not exceed hardware maximum, and must be multiple
	 * of E1000_ALIGN.
	 */
	if (nb_desc % EM_RXD_ALIGN != 0 ||
			(nb_desc > E1000_MAX_RING_DESC) ||
			(nb_desc < E1000_MIN_RING_DESC)) {
		return -EINVAL;
	}

	/*
	 * EM devices don't support drop_en functionality.
	 * It's an optimization that does nothing on single-queue devices,
	 * so just log the issue and carry on.
	 */
	if (rx_conf->rx_drop_en) {
		PMD_INIT_LOG(NOTICE, "drop_en functionality not supported by "
			     "device");
	}

	/* Free memory prior to re-allocation if needed. */
	if (dev->data->rx_queues[queue_idx] != NULL) {
		em_rx_queue_release(dev->data->rx_queues[queue_idx]);
		dev->data->rx_queues[queue_idx] = NULL;
	}

	// /* Allocate RX ring for max possible mumber of hardware descriptors. */
	// rsize = sizeof(rxq->rx_ring[0]) * E1000_MAX_RING_DESC;
	// rz = rte_eth_dma_zone_reserve(dev, "rx_ring", queue_idx, rsize,
	// 			      RTE_CACHE_LINE_SIZE, socket_id);
	// if (rz == NULL)
	// 	return -ENOMEM;

	/* Allocate the RX queue data structure. */
	if ((rxq = rte_zmalloc("ethdev RX queue", sizeof(struct em_rx_queue),
			RTE_CACHE_LINE_SIZE)) == NULL)
		return -ENOMEM;

	rxq->mb_pool = mp;
	rxq->nb_rx_desc = nb_desc;
	rxq->pthresh = rx_conf->rx_thresh.pthresh;
	rxq->hthresh = rx_conf->rx_thresh.hthresh;
	rxq->wthresh = rx_conf->rx_thresh.wthresh;
	rxq->rx_free_thresh = rx_conf->rx_free_thresh;
	rxq->queue_id = queue_idx;
	rxq->port_id = dev->data->port_id;
	if (dev->data->dev_conf.rxmode.offloads & DEV_RX_OFFLOAD_KEEP_CRC)
		rxq->crc_len = RTE_ETHER_CRC_LEN;
	else
		rxq->crc_len = 0;
		
	/* Allocate RX ring for max possible mumber of hardware descriptors. */
	rsize = sizeof(rxq->rx_ring[0]) * E1000_MAX_RING_DESC;
	rz = rte_eth_dma_zone_reserve(dev, "rx_ring", queue_idx, rsize,
				    //   RTE_CACHE_LINE_SIZE, socket_id);
					E1000_ALIGN, socket_id);
	if (rz == NULL)
		return -ENOMEM;

	/* Allocate software ring. */
	if ((rxq->sw_ring = rte_zmalloc("rxq->sw_ring",
			sizeof (rxq->sw_ring[0]) * nb_desc,
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_rx_queue_release(rxq);
		return -ENOMEM;
	}

	rxq->rdt_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_RDT(queue_idx));
	rxq->rdh_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_RDH(queue_idx));
	rxq->rx_ring_phys_addr = rz->iova;
	// rxq->rx_ring = (struct e1000_rx_desc *) rz->addr;
	rxq->rx_ring = (union e1000_adv_rx_desc *) rz->addr;

	PMD_INIT_LOG(DEBUG, "sw_ring=%p hw_ring=%p dma_addr=0x%"PRIx64,
		     rxq->sw_ring, rxq->rx_ring, rxq->rx_ring_phys_addr);

	dev->data->rx_queues[queue_idx] = rxq;
	em_reset_rx_queue(rxq);
	rxq->offloads = offloads;

	return 0;
}

uint32_t
eth_em_rx_queue_count(struct rte_eth_dev *dev, uint16_t rx_queue_id)
{
#define EM_RXQ_SCAN_INTERVAL 4
	// volatile struct e1000_rx_desc *rxdp;
	volatile union e1000_adv_rx_desc *rxdp;
	struct em_rx_queue *rxq;
	uint32_t desc = 0;

	rxq = dev->data->rx_queues[rx_queue_id];
	rxdp = &(rxq->rx_ring[rxq->rx_tail]);

	while ((desc < rxq->nb_rx_desc) &&
		// (rxdp->status & E1000_RXD_STAT_DD)) {
		(rxdp->wb.upper.status_error & E1000_RXD_STAT_DD)) {
		desc += EM_RXQ_SCAN_INTERVAL;
		rxdp += EM_RXQ_SCAN_INTERVAL;
		if (rxq->rx_tail + desc >= rxq->nb_rx_desc)
			rxdp = &(rxq->rx_ring[rxq->rx_tail +
				desc - rxq->nb_rx_desc]);
	}

	return desc;
}

int
eth_em_rx_descriptor_done(void *rx_queue, uint16_t offset)
{
	// volatile struct e1000_rx_desc *rxdp;
	volatile union e1000_adv_rx_desc *rxdp;
	struct em_rx_queue *rxq = rx_queue;
	uint32_t desc;

	if (unlikely(offset >= rxq->nb_rx_desc))
		return 0;
	desc = rxq->rx_tail + offset;
	if (desc >= rxq->nb_rx_desc)
		desc -= rxq->nb_rx_desc;

	rxdp = &rxq->rx_ring[desc];
	// return !!(rxdp->status & E1000_RXD_STAT_DD);
	return !!(rxdp->wb.upper.status_error & E1000_RXD_STAT_DD);
}

int
eth_em_rx_descriptor_status(void *rx_queue, uint16_t offset)
{
	struct em_rx_queue *rxq = rx_queue;
	volatile uint8_t *status;
	uint32_t desc;

	if (unlikely(offset >= rxq->nb_rx_desc))
		return -EINVAL;

	if (offset >= rxq->nb_rx_desc - rxq->nb_rx_hold)
		return RTE_ETH_RX_DESC_UNAVAIL;

	desc = rxq->rx_tail + offset;
	if (desc >= rxq->nb_rx_desc)
		desc -= rxq->nb_rx_desc;

	// status = &rxq->rx_ring[desc].status;
	// if (*status & E1000_RXD_STAT_DD)
	status = &rxq->rx_ring[desc].wb.upper.status_error;
	if (*status & rte_cpu_to_le_32(E1000_RXD_STAT_DD))
		return RTE_ETH_RX_DESC_DONE;

	return RTE_ETH_RX_DESC_AVAIL;
}

int
eth_em_tx_descriptor_status(void *tx_queue, uint16_t offset)
{
	struct em_tx_queue *txq = tx_queue;
	volatile uint8_t *status;
	uint32_t desc;

	if (unlikely(offset >= txq->nb_tx_desc))
		return -EINVAL;

	desc = txq->tx_tail + offset;
	/* go to next desc that has the RS bit */
	desc = ((desc + txq->tx_rs_thresh - 1) / txq->tx_rs_thresh) *
		txq->tx_rs_thresh;
	if (desc >= txq->nb_tx_desc) {
		desc -= txq->nb_tx_desc;
		if (desc >= txq->nb_tx_desc)
			desc -= txq->nb_tx_desc;
	}

	// status = &txq->tx_ring[desc].upper.fields.status;
	// if (*status & E1000_TXD_STAT_DD)
	status = &txq->tx_ring[desc].wb.status;
	if (*status & rte_cpu_to_le_32(E1000_TXD_STAT_DD))
		return RTE_ETH_TX_DESC_DONE;

	return RTE_ETH_TX_DESC_FULL;
}

void
em_dev_clear_queues(struct rte_eth_dev *dev)
{
	uint16_t i;
	struct em_tx_queue *txq;
	struct em_rx_queue *rxq;

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		txq = dev->data->tx_queues[i];
		if (txq != NULL) {
			em_tx_queue_release_mbufs(txq);
			em_reset_tx_queue(txq);
		}
	}

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		rxq = dev->data->rx_queues[i];
		if (rxq != NULL) {
			em_rx_queue_release_mbufs(rxq);
			em_reset_rx_queue(rxq);
		}
	}
}

void
em_dev_free_queues(struct rte_eth_dev *dev)
{
	uint16_t i;

	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		eth_em_rx_queue_release(dev->data->rx_queues[i]);
		dev->data->rx_queues[i] = NULL;
		rte_eth_dma_zone_free(dev, "rx_ring", i);
	}
	dev->data->nb_rx_queues = 0;

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		eth_em_tx_queue_release(dev->data->tx_queues[i]);
		dev->data->tx_queues[i] = NULL;
		rte_eth_dma_zone_free(dev, "tx_ring", i);
	}
	dev->data->nb_tx_queues = 0;
}

/*
 * Receive Side Scaling (RSS).
 * See section 7.1.1.7 in the following document:
 *     "Intel 82576 GbE Controller Datasheet" - Revision 2.45 October 2009
 *
 * Principles:
 * The source and destination IP addresses of the IP header and the source and
 * destination ports of TCP/UDP headers, if any, of received packets are hashed
 * against a configurable random key to compute a 32-bit RSS hash result.
 * The seven (7) LSBs of the 32-bit hash result are used as an index into a
 * 128-entry redirection table (RETA).  Each entry of the RETA provides a 3-bit
 * RSS output index which is used as the RX queue index where to store the
 * received packets.
 * The following output is supplied in the RX write-back descriptor:
 *     - 32-bit result of the Microsoft RSS hash function,
 *     - 4-bit RSS type field.
*/

/*
 * RSS random key supplied in section 7.1.1.7.3 of the Intel 82576 datasheet.
 * Used as the default key.
 */
static uint8_t rss_intel_key[40] = {
	0x6D, 0x5A, 0x56, 0xDA, 0x25, 0x5B, 0x0E, 0xC2,
	0x41, 0x67, 0x25, 0x3D, 0x43, 0xA3, 0x8F, 0xB0,
	0xD0, 0xCA, 0x2B, 0xCB, 0xAE, 0x7B, 0x30, 0xB4,
	0x77, 0xCB, 0x2D, 0xA3, 0x80, 0x30, 0xF2, 0x0C,
	0x6A, 0x42, 0xB7, 0x3B, 0xBE, 0xAC, 0x01, 0xFA,
};

static void
em_rss_disable(struct rte_eth_dev *dev)
{
	struct e1000_hw *hw;
	uint32_t mrqc;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	mrqc = E1000_READ_REG(hw, E1000_MRQC);
	mrqc &= ~E1000_MRQC_ENABLE_MASK;
	E1000_WRITE_REG(hw, E1000_MRQC, mrqc);
}

static void
em_hw_rss_hash_set(struct e1000_hw *hw, struct rte_eth_rss_conf *rss_conf)
{
	uint8_t  *hash_key;
	uint32_t rss_key;
	uint32_t mrqc;
	uint64_t rss_hf;
	uint16_t i;

	hash_key = rss_conf->rss_key;
	if (hash_key != NULL) {
		/* Fill in RSS hash key */
		for (i = 0; i < 10; i++) {
			rss_key  = hash_key[(i * 4)];
			rss_key |= hash_key[(i * 4) + 1] << 8;
			rss_key |= hash_key[(i * 4) + 2] << 16;
			rss_key |= hash_key[(i * 4) + 3] << 24;
			E1000_WRITE_REG_ARRAY(hw, E1000_RSSRK(0), i, rss_key);
		}
	}

	/* Set configured hashing protocols in MRQC register */
	rss_hf = rss_conf->rss_hf;
	mrqc = E1000_MRQC_ENABLE_RSS_4Q; /* RSS enabled. */
	if (rss_hf & ETH_RSS_IPV4)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV4;
	if (rss_hf & ETH_RSS_NONFRAG_IPV4_TCP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV4_TCP;
	if (rss_hf & ETH_RSS_IPV6)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6;
	if (rss_hf & ETH_RSS_IPV6_EX)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_EX;
	if (rss_hf & ETH_RSS_NONFRAG_IPV6_TCP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_TCP;
	if (rss_hf & ETH_RSS_IPV6_TCP_EX)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_TCP_EX;
	if (rss_hf & ETH_RSS_NONFRAG_IPV4_UDP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV4_UDP;
	if (rss_hf & ETH_RSS_NONFRAG_IPV6_UDP)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_UDP;
	if (rss_hf & ETH_RSS_IPV6_UDP_EX)
		mrqc |= E1000_MRQC_RSS_FIELD_IPV6_UDP_EX;
	E1000_WRITE_REG(hw, E1000_MRQC, mrqc);
}

int
eth_em_rss_hash_update(struct rte_eth_dev *dev,
			struct rte_eth_rss_conf *rss_conf)
{
	struct e1000_hw *hw;
	uint32_t mrqc;
	uint64_t rss_hf;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/*
	 * Before changing anything, first check that the update RSS operation
	 * does not attempt to disable RSS, if RSS was enabled at
	 * initialization time, or does not attempt to enable RSS, if RSS was
	 * disabled at initialization time.
	 */
	rss_hf = rss_conf->rss_hf & EM_RSS_OFFLOAD_ALL;
	mrqc = E1000_READ_REG(hw, E1000_MRQC);
	if (!(mrqc & E1000_MRQC_ENABLE_MASK)) { /* RSS disabled */
		if (rss_hf != 0) /* Enable RSS */
			return -(EINVAL);
		return 0; /* Nothing to do */
	}
	/* RSS enabled */
	if (rss_hf == 0) /* Disable RSS */
		return -(EINVAL);
	em_hw_rss_hash_set(hw, rss_conf);
	return 0;
}

int eth_em_rss_hash_conf_get(struct rte_eth_dev *dev,
			      struct rte_eth_rss_conf *rss_conf)
{
	struct e1000_hw *hw;
	uint8_t *hash_key;
	uint32_t rss_key;
	uint32_t mrqc;
	uint64_t rss_hf;
	uint16_t i;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	hash_key = rss_conf->rss_key;
	if (hash_key != NULL) {
		/* Return RSS hash key */
		for (i = 0; i < 10; i++) {
			rss_key = E1000_READ_REG_ARRAY(hw, E1000_RSSRK(0), i);
			hash_key[(i * 4)] = rss_key & 0x000000FF;
			hash_key[(i * 4) + 1] = (rss_key >> 8) & 0x000000FF;
			hash_key[(i * 4) + 2] = (rss_key >> 16) & 0x000000FF;
			hash_key[(i * 4) + 3] = (rss_key >> 24) & 0x000000FF;
		}
	}

	/* Get RSS functions configured in MRQC register */
	mrqc = E1000_READ_REG(hw, E1000_MRQC);
	if ((mrqc & E1000_MRQC_ENABLE_RSS_4Q) == 0) { /* RSS is disabled */
		rss_conf->rss_hf = 0;
		return 0;
	}
	rss_hf = 0;
	if (mrqc & E1000_MRQC_RSS_FIELD_IPV4)
		rss_hf |= ETH_RSS_IPV4;
	if (mrqc & E1000_MRQC_RSS_FIELD_IPV4_TCP)
		rss_hf |= ETH_RSS_NONFRAG_IPV4_TCP;
	if (mrqc & E1000_MRQC_RSS_FIELD_IPV6)
		rss_hf |= ETH_RSS_IPV6;
	if (mrqc & E1000_MRQC_RSS_FIELD_IPV6_EX)
		rss_hf |= ETH_RSS_IPV6_EX;
	if (mrqc & E1000_MRQC_RSS_FIELD_IPV6_TCP)
		rss_hf |= ETH_RSS_NONFRAG_IPV6_TCP;
	if (mrqc & E1000_MRQC_RSS_FIELD_IPV6_TCP_EX)
		rss_hf |= ETH_RSS_IPV6_TCP_EX;
	if (mrqc & E1000_MRQC_RSS_FIELD_IPV4_UDP)
		rss_hf |= ETH_RSS_NONFRAG_IPV4_UDP;
	if (mrqc & E1000_MRQC_RSS_FIELD_IPV6_UDP)
		rss_hf |= ETH_RSS_NONFRAG_IPV6_UDP;
	if (mrqc & E1000_MRQC_RSS_FIELD_IPV6_UDP_EX)
		rss_hf |= ETH_RSS_IPV6_UDP_EX;
	rss_conf->rss_hf = rss_hf;
	return 0;
}

static void
em_rss_configure(struct rte_eth_dev *dev)
{
	struct rte_eth_rss_conf rss_conf;
	struct e1000_hw *hw;
	uint32_t shift;
	uint16_t i;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Fill in redirection table. */
	shift = 0; //we assume we can use q_idx directly
	for (i = 0; i < 128; i++) {
		union e1000_reta {
			uint32_t dword;
			uint8_t  bytes[4];
		} reta;
		uint8_t q_idx;

		q_idx = (uint8_t) ((dev->data->nb_rx_queues > 1) ?
				   i % dev->data->nb_rx_queues : 0);
		reta.bytes[i & 3] = (uint8_t) (q_idx << shift);
		if ((i & 3) == 3)
			E1000_WRITE_REG(hw, E1000_RETA(i >> 2), reta.dword);
	}

	/*
	 * Configure the RSS key and the RSS protocols used to compute
	 * the RSS hash of input packets.
	 */
	rss_conf = dev->data->dev_conf.rx_adv_conf.rss_conf;
	if ((rss_conf.rss_hf & EM_RSS_OFFLOAD_ALL) == 0) {
		em_rss_disable(dev);
		return;
	}
	if (rss_conf.rss_key == NULL)
		rss_conf.rss_key = rss_intel_key; /* Default hash key */
	em_hw_rss_hash_set(hw, &rss_conf);
}

/*
 * Takes as input/output parameter RX buffer size.
 * Returns (BSIZE | BSEX | FLXBUF) fields of RCTL register.
 */
static uint32_t
em_rctl_bsize(__rte_unused enum e1000_mac_type hwtyp, uint32_t *bufsz)
{
	/*
	 * For BSIZE & BSEX all configurable sizes are:
	 * 16384: rctl |= (E1000_RCTL_SZ_16384 | E1000_RCTL_BSEX);
	 *  8192: rctl |= (E1000_RCTL_SZ_8192  | E1000_RCTL_BSEX);
	 *  4096: rctl |= (E1000_RCTL_SZ_4096  | E1000_RCTL_BSEX);
	 *  2048: rctl |= E1000_RCTL_SZ_2048;
	 *  1024: rctl |= E1000_RCTL_SZ_1024;
	 *   512: rctl |= E1000_RCTL_SZ_512;
	 *   256: rctl |= E1000_RCTL_SZ_256;
	 */
	static const struct {
		uint32_t bufsz;
		uint32_t rctl;
	} bufsz_to_rctl[] = {
		{16384, (E1000_RCTL_SZ_16384 | E1000_RCTL_BSEX)},
		{8192,  (E1000_RCTL_SZ_8192  | E1000_RCTL_BSEX)},
		{4096,  (E1000_RCTL_SZ_4096  | E1000_RCTL_BSEX)},
		{2048,  E1000_RCTL_SZ_2048},
		{1024,  E1000_RCTL_SZ_1024},
		{512,   E1000_RCTL_SZ_512},
		{256,   E1000_RCTL_SZ_256},
	};

	int i;
	uint32_t rctl_bsize;

	rctl_bsize = *bufsz;

	/*
	 * Starting from 82571 it is possible to specify RX buffer size
	 * by RCTL.FLXBUF. When this field is different from zero, the
	 * RX buffer size = RCTL.FLXBUF * 1K
	 * (e.g. t is possible to specify RX buffer size  1,2,...,15KB).
	 * It is working ok on real HW, but by some reason doesn't work
	 * on VMware emulated 82574L.
	 * So for now, always use BSIZE/BSEX to setup RX buffer size.
	 * If you don't plan to use it on VMware emulated 82574L and
	 * would like to specify RX buffer size in 1K granularity,
	 * uncomment the following lines:
	 * ***************************************************************
	 * if (hwtyp >= e1000_82571 && hwtyp <= e1000_82574 &&
	 *		rctl_bsize >= EM_RCTL_FLXBUF_STEP) {
	 *	rctl_bsize /= EM_RCTL_FLXBUF_STEP;
	 *	*bufsz = rctl_bsize;
	 *	return (rctl_bsize << E1000_RCTL_FLXBUF_SHIFT &
	 *		E1000_RCTL_FLXBUF_MASK);
	 * }
	 * ***************************************************************
	 */

	for (i = 0; i != sizeof(bufsz_to_rctl) / sizeof(bufsz_to_rctl[0]);
			i++) {
		if (rctl_bsize >= bufsz_to_rctl[i].bufsz) {
			*bufsz = bufsz_to_rctl[i].bufsz;
			return bufsz_to_rctl[i].rctl;
		}
	}

	/* Should never happen. */
	return -EINVAL;
}

static int
em_alloc_rx_queue_mbufs(struct em_rx_queue *rxq)
{
	struct em_rx_entry *rxe = rxq->sw_ring;
	uint64_t dma_addr;
	unsigned i;
	// static const struct e1000_rx_desc rxd_init = {
	// 	.buffer_addr = 0,
	// };

	/* Initialize software ring entries */
	for (i = 0; i < rxq->nb_rx_desc; i++) {
		// volatile struct e1000_rx_desc *rxd;
		volatile union e1000_adv_rx_desc *rxd;
		struct rte_mbuf *mbuf = rte_mbuf_raw_alloc(rxq->mb_pool);

		if (mbuf == NULL) {
			PMD_INIT_LOG(ERR, "RX mbuf alloc failed "
				     "queue_id=%hu", rxq->queue_id);
			return -ENOMEM;
		}

		dma_addr =
			rte_cpu_to_le_64(rte_mbuf_data_iova_default(mbuf));

		/* Clear HW ring memory */
		// rxq->rx_ring[i] = rxd_init;

		rxd = &rxq->rx_ring[i];
		// rxd->buffer_addr = dma_addr;
		rxd->read.hdr_addr = 0;
		rxd->read.pkt_addr = dma_addr;
		rxe[i].mbuf = mbuf;
	}

	return 0;
}

/*********************************************************************
 *
 *  Enable receive unit.
 *
 **********************************************************************/

#define E1000_MRQC_DEF_Q_SHIFT               (3)
static int
em_dev_mq_rx_configure(struct rte_eth_dev *dev)
{
	struct e1000_hw *hw =
		E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	uint32_t mrqc;

	if (RTE_ETH_DEV_SRIOV(dev).active == ETH_8_POOLS) {
		/*
		 * SRIOV active scheme
		 * FIXME if support RSS together with VMDq & SRIOV
		 */
		mrqc = E1000_MRQC_ENABLE_VMDQ;
		/* 011b Def_Q ignore, according to VT_CTL.DEF_PL */
		mrqc |= 0x3 << E1000_MRQC_DEF_Q_SHIFT;
		E1000_WRITE_REG(hw, E1000_MRQC, mrqc);
	} else if(RTE_ETH_DEV_SRIOV(dev).active == 0) {
		/*
		 * SRIOV inactive scheme
		 */
		switch (dev->data->dev_conf.rxmode.mq_mode) {
			case ETH_MQ_RX_RSS:
				em_rss_configure(dev);
				break;
			case ETH_MQ_RX_VMDQ_ONLY:
				// not supported yet
				PMD_INIT_LOG(ERR, "EM MQ_RX_VMDQ_ONLY not supported");
				break;
			case ETH_MQ_RX_NONE:
				/* if mq_mode is none, disable rss mode.*/
			default:
				em_rss_disable(dev);
				break;
		}
	}

	return 0;
}

int
eth_em_rx_init(struct rte_eth_dev *dev)
{
	struct e1000_hw *hw;
	struct em_rx_queue *rxq;
	struct rte_eth_rxmode *rxmode;
	uint32_t rctl;
	uint32_t rfctl;
	uint32_t rxcsum;
	uint32_t rctl_bsize;
	uint16_t i;
	int ret;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	rxmode = &dev->data->dev_conf.rxmode;

	/*
	 * Make sure receives are disabled while setting
	 * up the descriptor ring.
	 */
	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);

	rfctl = E1000_READ_REG(hw, E1000_RFCTL);

	/* Disable extended descriptor type. */
	rfctl &= ~E1000_RFCTL_EXTEN;
	/* Disable accelerated acknowledge */
	if (hw->mac.type == e1000_82574)
		rfctl |= E1000_RFCTL_ACK_DIS;

	E1000_WRITE_REG(hw, E1000_RFCTL, rfctl);

	/*
	 * XXX TEMPORARY WORKAROUND: on some systems with 82573
	 * long latencies are observed, like Lenovo X60. This
	 * change eliminates the problem, but since having positive
	 * values in RDTR is a known source of problems on other
	 * platforms another solution is being sought.
	 */
	if (hw->mac.type == e1000_82573)
		E1000_WRITE_REG(hw, E1000_RDTR, 0x20);

	dev->rx_pkt_burst = (eth_rx_burst_t)eth_em_recv_pkts;

	/* Determine RX bufsize. */
	rctl_bsize = EM_MAX_BUF_SIZE;
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		uint32_t buf_size;

		rxq = dev->data->rx_queues[i];
		buf_size = rte_pktmbuf_data_room_size(rxq->mb_pool) -
			RTE_PKTMBUF_HEADROOM;
		rctl_bsize = RTE_MIN(rctl_bsize, buf_size);
	}

	rctl |= em_rctl_bsize(hw->mac.type, &rctl_bsize);

	/* Configure and enable each RX queue. */
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		uint64_t bus_addr;
		uint32_t rxdctl;

		rxq = dev->data->rx_queues[i];

		/* Allocate buffers for descriptor rings and setup queue */
		ret = em_alloc_rx_queue_mbufs(rxq);
		if (ret)
			return ret;

		/*
		 * Reset crc_len in case it was changed after queue setup by a
		 *  call to configure
		 */
		if (dev->data->dev_conf.rxmode.offloads & DEV_RX_OFFLOAD_KEEP_CRC)
			rxq->crc_len = RTE_ETHER_CRC_LEN;
		else
			rxq->crc_len = 0;

		bus_addr = rxq->rx_ring_phys_addr;
		E1000_WRITE_REG(hw, E1000_RDLEN(i),
				rxq->nb_rx_desc *
				sizeof(*rxq->rx_ring));
		E1000_WRITE_REG(hw, E1000_RDBAH(i),
				(uint32_t)(bus_addr >> 32));
		E1000_WRITE_REG(hw, E1000_RDBAL(i), (uint32_t)bus_addr);

		E1000_WRITE_REG(hw, E1000_RDH(i), 0);
		E1000_WRITE_REG(hw, E1000_RDT(i), rxq->nb_rx_desc - 1);

		// rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(0));
		rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(i));
		rxdctl |= E1000_RXDCTL_QUEUE_ENABLE; // TODO - jm: Have to check, if it is correct - it is from igb_rxtx.c
		rxdctl &= 0xFE000000; // TODO - jm: why this is FE000000.. it is FFF00000 in igb_rxtx.c
		rxdctl |= rxq->pthresh & 0x3F;
		rxdctl |= (rxq->hthresh & 0x3F) << 8;
		rxdctl |= (rxq->wthresh & 0x3F) << 16;
		rxdctl |= E1000_RXDCTL_GRAN;
		E1000_WRITE_REG(hw, E1000_RXDCTL(i), rxdctl);

		/*
		 * Due to EM devices not having any sort of hardware
		 * limit for packet length, jumbo frame of any size
		 * can be accepted, thus we have to enable scattered
		 * rx if jumbo frames are enabled (or if buffer size
		 * is too small to accommodate non-jumbo packets)
		 * to avoid splitting packets that don't fit into
		 * one buffer.
		 */
		if (rxmode->offloads & DEV_RX_OFFLOAD_JUMBO_FRAME ||
				rctl_bsize < RTE_ETHER_MAX_LEN) {
			if (!dev->data->scattered_rx)
				PMD_INIT_LOG(DEBUG, "forcing scatter mode");
			dev->rx_pkt_burst =
				(eth_rx_burst_t)eth_em_recv_scattered_pkts;
			dev->data->scattered_rx = 1;
		}
	}

	if (dev->data->dev_conf.rxmode.offloads & DEV_RX_OFFLOAD_SCATTER) {
		if (!dev->data->scattered_rx)
			PMD_INIT_LOG(DEBUG, "forcing scatter mode");
		dev->rx_pkt_burst = eth_em_recv_scattered_pkts;
		dev->data->scattered_rx = 1;
	}

	/*
	 * Configure RSS if device configured with multiple RX queues.
	 */
	em_dev_mq_rx_configure(dev);

	/* Update the rctl since igb_dev_mq_rx_configure may change its value */
	rctl |= E1000_READ_REG(hw, E1000_RCTL);

	/*
	 * Setup the Checksum Register.
	 * Receive Full-Packet Checksum Offload is mutually exclusive with RSS.
	 */
	rxcsum = E1000_READ_REG(hw, E1000_RXCSUM);

	if (rxmode->offloads & DEV_RX_OFFLOAD_CHECKSUM)
		rxcsum |= E1000_RXCSUM_IPOFL;
	else
		rxcsum &= ~E1000_RXCSUM_IPOFL;
	E1000_WRITE_REG(hw, E1000_RXCSUM, rxcsum);

	/* No MRQ or RSS support for now */ /* jm - enable at above code */

	/* Set early receive threshold on appropriate hw */
	if ((hw->mac.type == e1000_ich9lan ||
			hw->mac.type == e1000_pch2lan ||
			hw->mac.type == e1000_ich10lan) &&
			rxmode->offloads & DEV_RX_OFFLOAD_JUMBO_FRAME) {
		u32 rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(0));
		E1000_WRITE_REG(hw, E1000_RXDCTL(0), rxdctl | 3);
		E1000_WRITE_REG(hw, E1000_ERT, 0x100 | (1 << 13));
	}

	if (hw->mac.type == e1000_pch2lan) {
		if (rxmode->offloads & DEV_RX_OFFLOAD_JUMBO_FRAME)
			e1000_lv_jumbo_workaround_ich8lan(hw, TRUE);
		else
			e1000_lv_jumbo_workaround_ich8lan(hw, FALSE);
	}

	/* Setup the Receive Control Register. */
	if (dev->data->dev_conf.rxmode.offloads & DEV_RX_OFFLOAD_KEEP_CRC)
		rctl &= ~E1000_RCTL_SECRC; /* Do not Strip Ethernet CRC. */
	else
		rctl |= E1000_RCTL_SECRC; /* Strip Ethernet CRC. */

	rctl &= ~(3 << E1000_RCTL_MO_SHIFT);
	rctl |= E1000_RCTL_EN | E1000_RCTL_BAM | E1000_RCTL_LBM_NO |
		E1000_RCTL_RDMTS_HALF |
		(hw->mac.mc_filter_type << E1000_RCTL_MO_SHIFT);

	/* Make sure VLAN Filters are off. */
	rctl &= ~E1000_RCTL_VFE;
	/* Don't store bad packets. */
	rctl &= ~E1000_RCTL_SBP;
	/* Legacy descriptor type. */
	rctl &= ~E1000_RCTL_DTYP_MASK;

	/*
	 * Configure support of jumbo frames, if any.
	 */
	if (rxmode->offloads & DEV_RX_OFFLOAD_JUMBO_FRAME)
		rctl |= E1000_RCTL_LPE;
	else
		rctl &= ~E1000_RCTL_LPE;

	/* Enable Receives. */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

	return 0;
}

/*********************************************************************
 *
 *  Enable transmit unit.
 *
 **********************************************************************/
void
eth_em_tx_init(struct rte_eth_dev *dev)
{
	struct e1000_hw     *hw;
	struct em_tx_queue *txq;
	uint32_t tctl;
	uint32_t txdctl;
	uint16_t i;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Setup the Base and Length of the Tx Descriptor Rings. */
	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		uint64_t bus_addr;

		txq = dev->data->tx_queues[i];
		bus_addr = txq->tx_ring_phys_addr;
		E1000_WRITE_REG(hw, E1000_TDLEN(i),
				txq->nb_tx_desc *
				sizeof(*txq->tx_ring));
		E1000_WRITE_REG(hw, E1000_TDBAH(i),
				(uint32_t)(bus_addr >> 32));
		E1000_WRITE_REG(hw, E1000_TDBAL(i), (uint32_t)bus_addr);

		/* Setup the HW Tx Head and Tail descriptor pointers. */
		E1000_WRITE_REG(hw, E1000_TDT(i), 0);
		E1000_WRITE_REG(hw, E1000_TDH(i), 0);

		/* Setup Transmit threshold registers. */
		txdctl = E1000_READ_REG(hw, E1000_TXDCTL(i));
		/*
		 * bit 22 is reserved, on some models should always be 0,
		 * on others  - always 1.
		 */
		txdctl &= E1000_TXDCTL_COUNT_DESC;
		txdctl |= txq->pthresh & 0x3F;
		txdctl |= (txq->hthresh & 0x3F) << 8;
		txdctl |= (txq->wthresh & 0x3F) << 16;
		txdctl |= E1000_TXDCTL_GRAN;
		E1000_WRITE_REG(hw, E1000_TXDCTL(i), txdctl);
	}

	/* Program the Transmit Control Register. */
	tctl = E1000_READ_REG(hw, E1000_TCTL);
	tctl &= ~E1000_TCTL_CT;
	tctl |= (E1000_TCTL_PSP | E1000_TCTL_RTLC | E1000_TCTL_EN |
		 (E1000_COLLISION_THRESHOLD << E1000_CT_SHIFT));

	/* SPT and CNP Si errata workaround to avoid data corruption */
	if (hw->mac.type == e1000_pch_spt) {
		uint32_t reg_val;
		reg_val = E1000_READ_REG(hw, E1000_IOSFPC);
		reg_val |= E1000_RCTL_RDMTS_HEX;
		E1000_WRITE_REG(hw, E1000_IOSFPC, reg_val);

		/* Dropping the number of outstanding requests from
		 * 3 to 2 in order to avoid a buffer overrun.
		 */
		reg_val = E1000_READ_REG(hw, E1000_TARC(0));
		reg_val &= ~E1000_TARC0_CB_MULTIQ_3_REQ;
		reg_val |= E1000_TARC0_CB_MULTIQ_2_REQ;
		E1000_WRITE_REG(hw, E1000_TARC(0), reg_val);
	}

	/* This write will effectively turn on the transmit unit. */
	E1000_WRITE_REG(hw, E1000_TCTL, tctl);
}

void
em_rxq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_rxq_info *qinfo)
{
	struct em_rx_queue *rxq;

	rxq = dev->data->rx_queues[queue_id];

	qinfo->mp = rxq->mb_pool;
	qinfo->scattered_rx = dev->data->scattered_rx;
	qinfo->nb_desc = rxq->nb_rx_desc;
	qinfo->conf.rx_free_thresh = rxq->rx_free_thresh;
	qinfo->conf.offloads = rxq->offloads;
}

void
em_txq_info_get(struct rte_eth_dev *dev, uint16_t queue_id,
	struct rte_eth_txq_info *qinfo)
{
	struct em_tx_queue *txq;

	txq = dev->data->tx_queues[queue_id];

	qinfo->nb_desc = txq->nb_tx_desc;

	qinfo->conf.tx_thresh.pthresh = txq->pthresh;
	qinfo->conf.tx_thresh.hthresh = txq->hthresh;
	qinfo->conf.tx_thresh.wthresh = txq->wthresh;
	qinfo->conf.tx_free_thresh = txq->tx_free_thresh;
	qinfo->conf.tx_rs_thresh = txq->tx_rs_thresh;
	qinfo->conf.offloads = txq->offloads;
}

//jm - for now, I will not implement this function - because of descriptor type diff
// static void
// e1000_flush_tx_ring(struct rte_eth_dev *dev)
// {
// 	struct e1000_hw *hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
// 	volatile struct e1000_data_desc *tx_desc;
// 	volatile uint32_t *tdt_reg_addr;
// 	uint32_t tdt, tctl, txd_lower = E1000_TXD_CMD_IFCS;
// 	uint16_t size = 512;
// 	struct em_tx_queue *txq;
// 	int i;

// 	if (dev->data->tx_queues == NULL)
// 		return;
// 	tctl = E1000_READ_REG(hw, E1000_TCTL);
// 	E1000_WRITE_REG(hw, E1000_TCTL, tctl | E1000_TCTL_EN);
// 	for (i = 0; i < dev->data->nb_tx_queues &&
// 		i < E1000_I219_MAX_TX_QUEUE_NUM; i++) {
// 		txq = dev->data->tx_queues[i];
// 		tdt = E1000_READ_REG(hw, E1000_TDT(i));
// 		if (tdt != txq->tx_tail)
// 			return;
// 		tx_desc = &txq->tx_ring[txq->tx_tail];
// 		tx_desc->buffer_addr = rte_cpu_to_le_64(txq->tx_ring_phys_addr);
// 		tx_desc->lower.data = rte_cpu_to_le_32(txd_lower | size);
// 		tx_desc->upper.data = 0;

// 		rte_io_wmb();
// 		txq->tx_tail++;
// 		if (txq->tx_tail == txq->nb_tx_desc)
// 			txq->tx_tail = 0;
// 		tdt_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_TDT(i));
// 		E1000_PCI_REG_WRITE(tdt_reg_addr, txq->tx_tail);
// 		usec_delay(250);
// 	}
// }

static void
e1000_flush_rx_ring(struct rte_eth_dev *dev)
{
	uint32_t rctl, rxdctl;
	struct e1000_hw *hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	int i;

	rctl = E1000_READ_REG(hw, E1000_RCTL);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
	E1000_WRITE_FLUSH(hw);
	usec_delay(150);

	for (i = 0; i < dev->data->nb_rx_queues &&
		i < E1000_I219_MAX_RX_QUEUE_NUM; i++) {
		rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(i));
		/* zero the lower 14 bits (prefetch and host thresholds) */
		rxdctl &= 0xffffc000;

		/* update thresholds: prefetch threshold to 31,
		 * host threshold to 1 and make sure the granularity
		 * is "descriptors" and not "cache lines"
		 */
		rxdctl |= (0x1F | (1UL << 8) | E1000_RXDCTL_THRESH_UNIT_DESC);

		E1000_WRITE_REG(hw, E1000_RXDCTL(i), rxdctl);
	}
	/* momentarily enable the RX ring for the changes to take effect */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl | E1000_RCTL_EN);
	E1000_WRITE_FLUSH(hw);
	usec_delay(150);
	E1000_WRITE_REG(hw, E1000_RCTL, rctl & ~E1000_RCTL_EN);
}

/**
 * em_flush_desc_rings - remove all descriptors from the descriptor rings
 *
 * In i219, the descriptor rings must be emptied before resetting/closing the
 * HW. Failure to do this will cause the HW to enter a unit hang state which
 * can only be released by PCI reset on the device
 *
 */

// void
// em_flush_desc_rings(struct rte_eth_dev *dev)
// {
// 	uint32_t fextnvm11, tdlen;
// 	struct e1000_hw *hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
// 	struct rte_pci_device *pci_dev = RTE_ETH_DEV_TO_PCI(dev);
// 	uint16_t pci_cfg_status = 0;
// 	int ret;

// 	fextnvm11 = E1000_READ_REG(hw, E1000_FEXTNVM11);
// 	E1000_WRITE_REG(hw, E1000_FEXTNVM11,
// 			fextnvm11 | E1000_FEXTNVM11_DISABLE_MULR_FIX);
// 	tdlen = E1000_READ_REG(hw, E1000_TDLEN(0));
// 	ret = rte_pci_read_config(pci_dev, &pci_cfg_status,
// 		   sizeof(pci_cfg_status), PCI_CFG_STATUS_REG);
// 	if (ret < 0) {
// 		PMD_DRV_LOG(ERR, "Failed to read PCI offset 0x%x",
// 			    PCI_CFG_STATUS_REG);
// 		return;
// 	}

// 	/* do nothing if we're not in faulty state, or if the queue is empty */
// 	if ((pci_cfg_status & FLUSH_DESC_REQUIRED) && tdlen) {
// 		/* flush desc ring */
// 		e1000_flush_tx_ring(dev);
// 		ret = rte_pci_read_config(pci_dev, &pci_cfg_status,
// 				sizeof(pci_cfg_status), PCI_CFG_STATUS_REG);
// 		if (ret < 0) {
// 			PMD_DRV_LOG(ERR, "Failed to read PCI offset 0x%x",
// 					PCI_CFG_STATUS_REG);
// 			return;
// 		}

// 		if (pci_cfg_status & FLUSH_DESC_REQUIRED)
// 			e1000_flush_rx_ring(dev);
// 	}
// }
