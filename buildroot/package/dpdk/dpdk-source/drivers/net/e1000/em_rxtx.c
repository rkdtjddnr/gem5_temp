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

#ifdef RTE_ARM_USE_SVE

#include <arm_sve.h>

#endif

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

#define M2FUNC_DTA_FLIT_SIZE 64
#define M2FUNC_DTA_FLIT_ARRAY_SIZE (M2FUNC_DTA_FLIT_SIZE / sizeof(uint64_t))
#define M2FUNC_DTA_CACHLINE_SIZE 64  // 64B cacheline size

#define M2FUNC_DTA_TX_JOB_FLIT_METADATA_SIZE 24  // 24B job flit metadata size (8B descriptor addr + 8B completion addr + 8B num packets)
#define M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE ((M2FUNC_DTA_FLIT_SIZE - M2FUNC_DTA_TX_JOB_FLIT_METADATA_SIZE) / sizeof(uint64_t)) // 6 mbuf addresses in the first flit
#define M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE (M2FUNC_DTA_FLIT_SIZE / sizeof(uint64_t)) // 8 mbuf addresses in the remaining flits

// JM
#define E1000_PCI_REG_WRITE64B(reg, value)		\
	rte_write64B(value, reg)

#define E1000_PCI_REG_READ64B(addr, packet_buffer)  \
	rte_read64B(addr, packet_buffer)

static __rte_always_inline void
rte_write64B(const void *packet, volatile void *addr)
{
    rte_io_wmb();  // Ensure memory writes happen in order before the I/O operation

    // ARM SVE assembly to transfer 64 bytes (512 bits) at once
	#ifdef RTE_ARM_USE_SVE
    asm volatile (
        "ptrue p0.d\n\t"                // Predicate register to select all elements in the vector
        "ld1d {z0.d}, p0/z, [%x[val]]\n\t"   // Load 64 bytes from packet into z0 register
        "st1d {z0.d}, p0, [%x[addr]]\n\t"     // Store 64 bytes from z0 register to the register (addr)
        :
        : [addr] "r"(addr), [val] "r"(packet)
        : "memory", "p0", "z0"
    );
	#endif
}

static __rte_always_inline void
rte_read64B(const volatile void *addr, void *packet_buffer)
{	
	#ifdef RTE_ARM_USE_SVE
    asm volatile (
        "ptrue p0.d\n\t"               // Predicate register to select all elements in the vector
        "ld1d {z0.d}, p0/z, [%x[addr]]\n\t"  // Load 64 bytes from addr into z0 register
        "st1d {z0.d}, p0, [%x[packet_buffer]]\n\t" // Store 64 bytes from z0 register into packet_buffer
        :
        : [addr] "r"(addr), [packet_buffer] "r"(packet_buffer)
        : "memory", "p0", "z0"
    );
	#endif

    rte_io_rmb();  // Memory barrier to ensure proper memory ordering after read
}


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

struct em_vec_tx_entry {
	struct rte_mbuf *mbuf; /**< mbuf associated with TX desc, if any. */
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
	volatile uint32_t   *rx_m2func_reg_addr; /**< Address of M2FUNC register. */
	volatile uint32_t   *dta_job_submit_reg_addr; /**< Address of DTA job submit register. */
	rte_iova_t			completion_addr; /**< Address of completion array. */
	volatile int64_t            *completion_buffer; /**< Completion buffer. */
	rte_iova_t			completion_addr2; /**< Address of completion array. */
	volatile int64_t            *completion_buffer2; /**< Completion buffer. */
	bool				started; /**< For DTA with two comp buffers, we have to check load_gen is started at gem5 */
	int8_t   		    completion_buffer_index; /**< Completion buffer index. - To enable double buffer*/				
	struct rte_mbuf **rx_bufs; /**< For Double buffer */
	struct rte_mbuf **rx_bufs2; /**< For Double buffer */
	uint64_t			   job_packet[M2FUNC_DTA_FLIT_ARRAY_SIZE]; /**< For DTA with two comp buffers, we have to check load_gen is started at gem5 */
	uint64_t			   job_packet2[M2FUNC_DTA_FLIT_ARRAY_SIZE]; /**< For DTA with two comp buffers, we have to check load_gen is started at gem5 */
	struct em_rx_entry *mbuf_array; /**JM - test for checking the mbuf alloc overhead. This array is for prep-allocating mbufs. */
	uint64_t			mbuf_addr_array[1024]; /**JM - test for checking the mbuf alloc overhead. This array is for prep-allocating mbufs. */
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

	struct rte_mbuf fake_mbuf; /**< dummy mbuf */
	uint16_t rxrearm_nb;	/**< number of remaining to be re-armed */
	uint16_t rxrearm_start;	/**< the idx we start the re-arming from */
	uint64_t mbuf_initializer; /**< value to init mbufs */
	uint8_t offset_table[10]; /* offset_table: used for vector, to solve execute re-order problem - from hns3. maybe prevent read rxd before check valid bit.*/
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
	#ifdef EM_SVE_512
	struct em_vec_tx_entry *sw_ring; /**< virtual address of SW ring for vector */
	#else
	struct em_tx_entry    *sw_ring; /**< virtual address of SW ring */
	#endif
	volatile uint32_t      *tdt_reg_addr; /**< Address of TDT register. */
	volatile uint32_t      *tx_m2func_reg_addr; /**< Address of M2FUNC register. */
	volatile uint32_t      *dta_job_submit_reg_addr; /**< Address of DTA job submit register. */
	rte_iova_t			   completion_addr; /**< Address of completion array. */
	volatile int64_t            	   *completion_buffer; /**< Completion buffer. */
	rte_iova_t			   descriptor_addr; /**< Address of descriptor array. */
	struct e1000_adv_tx_desc_m2func *descriptor_buffer; /**< Descriptor buffer. */
	rte_iova_t			   completion_addr2; /**< Address of completion array. */
	volatile int64_t            	   *completion_buffer2; /**< Completion buffer. */
	rte_iova_t			   descriptor_addr2; /**< Address of descriptor array. */
	struct e1000_adv_tx_desc_m2func *descriptor_buffer2; /**< Descriptor buffer. */
	struct rte_mbuf		   **tx_bufs; /**< TX bufs to free on release. */
	struct rte_mbuf		   **tx_bufs2; /**< TX bufs to free on release. */
	bool				started; /**< For DTA with two comp buffers, we have to check load_gen is started at gem5 */
	int8_t   		    completion_buffer_index; /**< Completion buffer index. - To enable double buffer*/		
	uint64_t			   zero_copy_job_packet[M2FUNC_DTA_FLIT_ARRAY_SIZE]; /**< Zero copy job packet. */
	uint64_t			   zero_copy_job_packet2[M2FUNC_DTA_FLIT_ARRAY_SIZE]; /**< Zero copy job packet. */
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
	uint16_t tx_next_dd; /**< next desc to scan for DD bit */
	uint16_t tx_next_rs; /**< next desc to set RS bit */
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
 * Populates TX context descriptor.
 */
static inline void
em_set_xmit_ctx_m2func(struct em_tx_queue* txq,
		uint64_t ol_flags,
		union em_vlan_macip hdrlen)
{
	struct e1000_adv_tx_context_desc ctx_txd;

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

	vlan_macip_lens = (uint32_t)hdrlen.data;
	// Write the context descriptor to the descriptor ring
	uint64_t ctx_buffer[8];

	// Pack the first and second halves of the context descriptor
	//for CXL, place d2 part at first and d1 part at second - to match with not context case
	ctx_buffer[1] = rte_cpu_to_le_32(((uint64_t)vlan_macip_lens << 32)) | 0;
	ctx_buffer[0] = rte_cpu_to_le_32(((uint64_t)type_tucmd_mlhl << 32)) | rte_cpu_to_le_32(mss_l4len_idx);

	// Fill the remaining 48B with 0 (NULL) for padding
	for (int i = 2; i < 8; i++) {
		ctx_buffer[i] = 0;
	}

	// Send each 64-bit portion using E1000_PCI_REG_WRITE64B
	E1000_PCI_REG_WRITE64B(txq->tx_m2func_reg_addr, ctx_buffer);

}

/*
 * Populates TX context descriptor.
 */
static inline void
em_set_xmit_ctx_m2func_dta(struct em_tx_queue* txq,
		volatile struct e1000_adv_tx_context_desc *ctx_txd,
		uint64_t ol_flags,
		union em_vlan_macip hdrlen)
{
	// TODO - JM
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
	#ifndef EM_SVE_512
	desc_to_clean_to = sw_ring[desc_to_clean_to].last_id;
	#endif
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


/* Reset transmit descriptors after they have been used */
static inline int
em_xmit_cleanup_m2func(struct em_tx_queue *txq)
{
	struct em_tx_entry *sw_ring = txq->sw_ring;
	volatile union e1000_adv_tx_desc *txr = txq->tx_ring; // NIC TX ring
	uint16_t last_desc_cleaned = txq->last_desc_cleaned;
	uint16_t nb_tx_desc = txq->nb_tx_desc;
	uint16_t desc_to_clean_to;
	uint16_t nb_tx_to_clean;
	uint16_t nb_tx_to_clean_goal; // JM
	uint8_t bitmask[64];  // 512-bit (64-byte) bitmask, byte-level array
	uint64_t bitmask_len = 512; // 512-bit (64-byte) bitmask
	int bit_idx = 0;
	uint16_t desc_idx;

	/* Read the 512-bit bitmask from the NIC's device register */
    E1000_PCI_REG_READ64B(txq->tx_m2func_reg_addr, bitmask);  // Function to read 64B from NIC register

	/* Determine the last descriptor needing to be cleaned */
	desc_to_clean_to = (uint16_t)(last_desc_cleaned + txq->tx_rs_thresh);
	if (desc_to_clean_to >= nb_tx_desc)
		desc_to_clean_to = (uint16_t)(desc_to_clean_to - nb_tx_desc);

	/* Check to make sure the last descriptor to clean is done */
	#ifndef EM_SVE_512
	desc_to_clean_to = sw_ring[desc_to_clean_to].last_id;
	#endif

	/* Iterate over the bitmask to clean descriptors */
	nb_tx_to_clean = 0;
	for (desc_idx = last_desc_cleaned; desc_idx != desc_to_clean_to; desc_idx = (desc_idx + 1) % nb_tx_desc) {
        /* Check if the corresponding bit in the bitmask is set (indicating DD = 1) */
		if (bit_idx >= bitmask_len) {
			PMD_TX_FREE_LOG(ERROR,
			"At TX descriptor %4u, bit_idx is over 512 (port=%d queue=%d), bit_idx=%d. So break", desc_idx,
			txq->port_id, txq->queue_id, bit_idx);
			return -(1);
		}
		if (bitmask[bit_idx / 8] & (1ULL << (bit_idx % 8))) { // 8-bit mask
			/* Descriptor is done, reset the status */
			nb_tx_to_clean++;
			txr[desc_idx].wb.status = 0; 
			PMD_TX_FREE_LOG(WARNING,
			"TX descriptor %4u is done (port=%d queue=%d), bit_idx=%d", desc_idx,
			txq->port_id, txq->queue_id, bit_idx);
		} else {
			/* Descriptor is not done, break out of the loop */
			PMD_TX_FREE_LOG(WARNING,
			"TX descriptor %4u is not done (port=%d queue=%d), bit_idx=%d. So break", desc_idx,
			txq->port_id, txq->queue_id, bit_idx);

			break;
		}
		bit_idx++;
    }

	if (nb_tx_to_clean == 0) {
		/* Failed to clean any descriptors, better luck next time */
		PMD_TX_FREE_LOG(WARNING,
				"TX descriptor nb_tx_to_clean=%d, last_desc_cleaned=%d, desc_to_clean_to=%d, cannot clean any descriptors (port=%d queue=%d)", 
				nb_tx_to_clean, last_desc_cleaned, desc_to_clean_to, txq->port_id, txq->queue_id);
		return -(1);
	}

	/* Figure out how many descriptors will be cleaned */
	if (last_desc_cleaned > desc_to_clean_to)
		nb_tx_to_clean_goal = (uint16_t)((nb_tx_desc - last_desc_cleaned) +
							desc_to_clean_to);
	else
		nb_tx_to_clean_goal = (uint16_t)(desc_to_clean_to -
						last_desc_cleaned);

	PMD_TX_FREE_LOG(WARNING,
			"Cleaning GOAL: %4u TX descriptors: %4u to %4u, RESULTS: %4u TX descriptors: %4u to %4u "
			"(port=%d queue=%d)", nb_tx_to_clean_goal, last_desc_cleaned, desc_to_clean_to, 
			nb_tx_to_clean, last_desc_cleaned, desc_idx, txq->port_id, txq->queue_id);	

	/* Update the txq to reflect the last descriptor that was cleaned */
	txq->last_desc_cleaned = desc_idx; // start index to look for next time
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
	volatile union e1000_adv_tx_desc *txr;
	volatile union e1000_adv_tx_desc *txd;
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

				#ifndef EM_SVE_512
				txn = &sw_ring[txe->next_id];
				#endif
				RTE_MBUF_PREFETCH_TO_FREE(txn->mbuf);

				if (txe->mbuf != NULL) {
					rte_pktmbuf_free_seg(txe->mbuf);
					txe->mbuf = NULL;
				}

				em_set_xmit_ctx(txq, ctx_txd, tx_ol_req,
					hdrlen);

				#ifndef EM_SVE_512
				txe->last_id = tx_last;
				tx_id = txe->next_id;
				#endif
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
			#ifndef EM_SVE_512
			txn = &sw_ring[txe->next_id];
			#endif

			if (txe->mbuf != NULL)
				rte_pktmbuf_free_seg(txe->mbuf);
			txe->mbuf = m_seg;

			/*
			 * Set up Transmit Data Descriptor.
			 */
			slen = m_seg->data_len;
			buf_dma_addr = rte_mbuf_data_iova(m_seg);
			
			//jm - advtxd
			txd->read.buffer_addr =
				rte_cpu_to_le_64(buf_dma_addr);
			txd->read.cmd_type_len =
				rte_cpu_to_le_32(cmd_type_len | slen);
			txd->read.olinfo_status =
				rte_cpu_to_le_32(olinfo_status);

			#ifndef EM_SVE_512
			txe->last_id = tx_last;
			tx_id = txe->next_id;
			#endif
			txe = txn;
			m_seg = m_seg->next;
		} while (m_seg != NULL);

		/*
		 * The last packet data descriptor needs End Of Packet (EOP)
		 */
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
	E1000_PCI_REG_WRITE_RELAXED(txq->tdt_reg_addr, tx_id); // TODO - gem5, tdt_reg_addr prints log
	txq->tx_tail = tx_id;

	return nb_tx;
}

static __rte_always_inline void 
eth_em_tx_backlog_entry_sve512(struct em_vec_tx_entry *txep, 
				struct rte_mbuf **tx_pkts, uint16_t nb_pkts)
{
	int i;

	for (i = 0; i < (int)nb_pkts; ++i)
		txep[i].mbuf = tx_pkts[i];
	
	// // parallel version store using SVE
	// svuint64_t base_addr;
	// uint32_t i = 0;
	// svbool_t pg = svwhilelt_b64_u64(i, nb_pkts);

	// do {
	// 	base_addr = svld1_u64(pg, (uint64_t *)tx_pkts);
	// 	svst1_u64(pg, (uint64_t *)txep, base_addr);

	// 	i += svcntd();
	// 	tx_pkts += svcntd();
	// 	txep += svcntd();
	// 	pg = svwhilelt_b64_u64(i, nb_pkts);

	// } while (svptest_any(svptrue_b64(), pg));
}

static inline void 
eth_em_vtx1(volatile union e1000_adv_tx_desc *txdp, struct rte_mbuf *pkt, uint64_t cmd_type)
{
	uint64_t high_qw = ((uint64_t)cmd_type) | ((uint64_t)pkt->data_len);

	// In this case, just use scalar store. Because sve style have to use scatter store
	txdp->read.buffer_addr = rte_cpu_to_le_64(pkt->buf_iova + pkt->data_off);
	txdp->read.cmd_type_len = rte_cpu_to_le_32(high_qw);
	txdp->read.olinfo_status = 0; // TODO - to enable TSO, have to set paylen part of olinfo_status (46-63 bits)
}

static inline void 
eth_em_vtx(volatile union e1000_adv_tx_desc *txdp, 
	struct rte_mbuf **pkt, uint16_t nb_pkts, uint64_t cmd_type)
{				
	// TODO - to enable TSO, have to set paylen part of olinfo_status (46-63 bits)
	for (; nb_pkts > 3; txdp += 4, pkt += 4, nb_pkts -= 4) {
		// Utilize sve intrinsic to do the same thing as ice_vtx() that uses avx512
		// According to GPT, sve doesn't need to flip the order of the packets as in avx-512
		uint64_t desc_values[8] = {
			pkt[0]->buf_iova + pkt[0]->data_off,
			cmd_type | ((uint64_t)pkt[0]->data_len),
			pkt[1]->buf_iova + pkt[1]->data_off,
			cmd_type | ((uint64_t)pkt[1]->data_len),
			pkt[2]->buf_iova + pkt[2]->data_off,
			cmd_type | ((uint64_t)pkt[2]->data_len),
			pkt[3]->buf_iova + pkt[3]->data_off,
			cmd_type | ((uint64_t)pkt[3]->data_len)
		};
		
		// Make desc0_3 (64B)
		svuint64_t desc0_3 = svld1_u64(svptrue_b64(), desc_values);

		// Store desc0_3 to txdp
		svst1_u64(svptrue_b64(), (uint64_t *)txdp, desc0_3);
	}

	/* do any last ones */
	while (nb_pkts) {
		eth_em_vtx1(txdp, *pkt, cmd_type);
		txdp++, pkt++, nb_pkts--;
	}

}

static __rte_always_inline int 
eth_em_tx_free_bufs_sve512(struct em_tx_queue *txq)
{
	#define EM_TX_MAX_FREE_BUF_SZ 64
	struct em_vec_tx_entry *txep;
	uint32_t n;
	uint32_t i;
	int nb_free = 0;
	struct rte_mbuf *m, *free[EM_TX_MAX_FREE_BUF_SZ];

	/* check DD bits on threshold descriptor */
	if (!(txq->tx_ring[txq->tx_next_dd].wb.status & E1000_TXD_STAT_DD))
		return 0;
	
	n = txq->tx_rs_thresh;

	/*
	 * first buffer to free from S/W ring is at index
	 * tx_next_dd - (tx_rs_thresh - 1)
	 */
	txep = (void *)txq->sw_ring;
	txep += txq->tx_next_dd - (n - 1);

	if (txq->offloads & DEV_TX_OFFLOAD_MBUF_FAST_FREE && (n & 31) == 0) {
		// TODO: have to check TX_OFFLOAD_MBUF_FAST_FREE is enabled or not!
		struct rte_mempool *mp = txep[0].mbuf->pool;
		void **cache_objs;
		struct rte_mempool_cache *cache = rte_mempool_default_cache(mp,
				rte_lcore_id());
		
		if (!cache || cache->len == 0)
			goto normal;

		cache_objs = &cache->objs[cache->len];

		if (n > RTE_MEMPOOL_CACHE_MAX_SIZE) {
			rte_mempool_ops_enqueue_bulk(mp, (void *)txep, n);
			goto done;
		}

		/* The cache follows the following algorithm
		 *   1. Add the objects to the cache
		 *   2. Anything greater than the cache min value (if it
		 *   crosses the cache flush threshold) is flushed to the ring.
		 */
		/* Add elements back into the cache */
		uint32_t copied = 0;
		/* n is multiple of 32 */
		while (copied < n) {
			// Make the sve version
			svuint64_t a = svld1_u64(svptrue_b64(), (uint64_t *)&txep[copied]);
			svuint64_t b = svld1_u64(svptrue_b64(), (uint64_t *)&txep[copied + 8]);
            svuint64_t c = svld1_u64(svptrue_b64(), (uint64_t *)&txep[copied + 16]);
            svuint64_t d = svld1_u64(svptrue_b64(), (uint64_t *)&txep[copied + 24]);

			svst1_u64(svptrue_b64(), (uint64_t *)&cache_objs[copied], a);
            svst1_u64(svptrue_b64(), (uint64_t *)&cache_objs[copied + 8], b);
            svst1_u64(svptrue_b64(), (uint64_t *)&cache_objs[copied + 16], c);
            svst1_u64(svptrue_b64(), (uint64_t *)&cache_objs[copied + 24], d);
            copied += 32;
		}
		cache->len += n;
		
		if (cache->len >= cache->flushthresh) {
			rte_mempool_ops_enqueue_bulk
				(mp, &cache->objs[cache->size],
				 cache->len - cache->size);
			cache->len = cache->size;
		}
		goto done;
	}

normal:
	m = rte_pktmbuf_prefree_seg(txep[0].mbuf);
	if (likely(m)) {
		free[0] = m;
		nb_free = 1;
		for (i = 1; i < n; i++) {
			m = rte_pktmbuf_prefree_seg(txep[i].mbuf);
			if (likely(m)) {
				if (likely(m->pool == free[0]->pool)) {
					free[nb_free++] = m;
				} else {
					rte_mempool_put_bulk(free[0]->pool,
							     (void *)free,
							     nb_free);
					free[0] = m;
					nb_free = 1;
				}
			}
		}
		rte_mempool_put_bulk(free[0]->pool, (void **)free, nb_free);
	} else {
		for (i = 1; i < n; i++) {
			m = rte_pktmbuf_prefree_seg(txep[i].mbuf);
			if (m)
				rte_mempool_put(m->pool, m);
		}
	}

done:
	/* buffers were freed, update counters */
	txq->nb_tx_free = (uint16_t)(txq->nb_tx_free + txq->tx_rs_thresh);
	txq->tx_next_dd = (uint16_t)(txq->tx_next_dd + txq->tx_rs_thresh);
	if (txq->tx_next_dd >= txq->nb_tx_desc)
		txq->tx_next_dd = (uint16_t)(txq->tx_rs_thresh - 1);
	
	return txq->tx_rs_thresh;
}

static inline uint16_t 
eth_em_xmit_fixed_burst_vec_sve512(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts)
{
	/*Utilize ice_rxtx_vec_avx512.c file. But we will use arm sve intrinsic*/
	struct em_tx_queue *txq = (struct em_tx_queue *)tx_queue;
	volatile union e1000_adv_tx_desc *txdp; //NIC TX ring
	struct em_vec_tx_entry *txep; //sw_ring
	uint16_t n, nb_commit, tx_id;
	uint64_t cmd_type = (uint64_t)(E1000_ADVTXD_DTYP_DATA |
				E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT |
				E1000_TXD_CMD_EOP);
	uint64_t cmd_type_rs = (uint64_t)(E1000_ADVTXD_DTYP_DATA |
				E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT |
				E1000_TXD_CMD_EOP | E1000_TXD_CMD_RS);

	/* cross rs_thresh boundary is not allowed*/
	nb_pkts = RTE_MIN(nb_pkts, txq->tx_rs_thresh);

	if (txq->nb_tx_free < txq->tx_free_thresh)
		eth_em_tx_free_bufs_sve512(txq);

	nb_commit = nb_pkts = (uint16_t)RTE_MIN(nb_pkts, txq->nb_tx_free);
	if (unlikely(nb_pkts == 0))
		return 0;
	
	tx_id = txq->tx_tail;
	printf("DPDK_SVE[TX]: tx_tail=%d\n", tx_id);
	fflush(stdout);
	txdp = &txq->tx_ring[tx_id];
	txep = (void *)txq->sw_ring;
	txep += tx_id;

	txq->nb_tx_free = (uint16_t)(txq->nb_tx_free - nb_pkts);

	n = (uint16_t)(txq->nb_tx_desc - tx_id);
	if (nb_commit >= n) {
		// For the case of reaching the end of the ring
		eth_em_tx_backlog_entry_sve512(txep, tx_pkts, n);

		eth_em_vtx(txdp, tx_pkts, n - 1, cmd_type);
		tx_pkts += (n - 1);
		txdp += (n - 1);

		eth_em_vtx1(txdp, *tx_pkts++, cmd_type_rs);

		nb_commit = (uint16_t)(nb_commit - n);

		tx_id = 0;
		txq->tx_next_rs = (uint16_t)(txq->tx_rs_thresh - 1);

		/* avoid reach the end of ring*/
		txdp = txq->tx_ring;
		txep = (void *)txq->sw_ring;
	}

	eth_em_tx_backlog_entry_sve512(txep, tx_pkts, nb_commit);

	eth_em_vtx(txdp, tx_pkts, nb_commit, cmd_type);

	tx_id = (uint16_t)(tx_id + nb_commit);
	if (tx_id > txq->tx_next_rs) {
		txq->tx_ring[txq->tx_next_rs].read.cmd_type_len |=
			rte_cpu_to_le_32(E1000_TXD_CMD_RS);
		txq->tx_next_rs = (uint16_t)(txq->tx_next_rs + txq->tx_rs_thresh);
	}

	txq->tx_tail = tx_id;

	E1000_PCI_REG_WRITE_RELAXED(txq->tdt_reg_addr, txq->tx_tail);

	return nb_pkts;
}

// For vectorized transmit
uint16_t 
eth_em_xmit_pkts_vec_sve512(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts)
{
	/*Utilize ice_rxtx_vec_avx512.c file. But we will use arm sve intrinsic*/
	uint16_t nb_tx = 0;
	struct em_tx_queue *txq = (struct em_tx_queue *)tx_queue;

	while (nb_pkts) {
		uint16_t ret, num;

		num = (uint16_t)RTE_MIN(nb_pkts, txq->tx_rs_thresh);
		ret = eth_em_xmit_fixed_burst_vec_sve512(tx_queue, 
								&tx_pkts[nb_tx], num);
		nb_tx += ret;
		nb_pkts -= ret;
		if (ret < num)
			break;
	}

	return nb_tx;
}


uint16_t
eth_em_xmit_pkts_m2func(void *tx_queue, struct rte_mbuf **tx_pkts,
        uint16_t nb_pkts)
{
    struct em_tx_queue *txq;
    struct em_tx_entry *sw_ring;
	struct em_tx_entry *txe, *txn;
    struct rte_mbuf     *tx_pkt;
    struct rte_mbuf     *m_seg;
    uint32_t olinfo_status;
    uint32_t cmd_type_len; 
    uint32_t pkt_len;
	uint64_t ol_flags;
	uint16_t tx_id;
	uint16_t tx_last;
	uint16_t nb_tx;
	uint16_t nb_used;
	uint64_t tx_ol_req;
	uint32_t ctx;
	uint32_t new_ctx;
	union em_vlan_macip hdrlen;
    void *seg_buf;
    int offset;
	unsigned desc_len = 8; //8B descriptor length
	unsigned flit_len = 64; //64B flit length
	unsigned payload_len_within_flit = flit_len - desc_len; //56B payload length within a 64B flit

    txq = tx_queue;
    sw_ring = txq->sw_ring; // DPDK TX ring
    tx_id = txq->tx_tail; // TX tail id
	txe = &sw_ring[tx_id]; // DPDK TX ring entry

    /* Clean up if necessary */
    if (txq->nb_tx_free < txq->tx_free_thresh)
        em_xmit_cleanup_m2func(txq);

    /* TX loop */
    for (nb_tx = 0; nb_tx < nb_pkts; nb_tx++) {
		new_ctx = 0;
        tx_pkt = *tx_pkts++;
        pkt_len = tx_pkt->pkt_len;
		
		RTE_MBUF_PREFETCH_TO_FREE(txe->mbuf);

        ol_flags = tx_pkt->ol_flags;
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

		if (tx_pkt->nb_segs > 1) {
			PMD_TX_FREE_LOG(WARNING, "Scattered packets not supported");
			if (nb_tx == 0)
				return 0;
			goto end_of_tx;
		}

		nb_used = (uint16_t)(tx_pkt->nb_segs + new_ctx);
		tx_last = (uint16_t) (tx_id + nb_used - 1);

		/* Circular ring */
		if (tx_last >= txq->nb_tx_desc)
			tx_last = (uint16_t) (tx_last - txq->nb_tx_desc);
		
		PMD_TX_LOG(INFO, "port_id=%u queue_id=%u pktlen=%u"
			   " tx_first=%u tx_last=%u",
			   (unsigned) txq->port_id,
			   (unsigned) txq->queue_id,
			   (unsigned) tx_pkt->pkt_len,
			   (unsigned) tx_id,
			   (unsigned) tx_last);
		
		while (unlikely (nb_used > txq->nb_tx_free)) {
			PMD_TX_FREE_LOG(WARNING, "Not enough free TX descriptors "
					"nb_used=%4u nb_free=%4u "
					"(port=%d queue=%d)",
					nb_used, txq->nb_tx_free,
					txq->port_id, txq->queue_id);

			if (em_xmit_cleanup_m2func(txq) != 0) {
				/* Could not clean any descriptors */
				if (nb_tx == 0)
					return 0;
				goto end_of_tx;
			}
		}

		txq->nb_tx_used = (uint16_t)(txq->nb_tx_used + nb_used);
		txq->nb_tx_free = (uint16_t)(txq->nb_tx_free - nb_used);
        
        /* Setup descriptor fields */
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
				#ifndef EM_SVE_512
				txn = &sw_ring[txe->next_id];
				#endif
				RTE_MBUF_PREFETCH_TO_FREE(txn->mbuf);

				if (txe->mbuf != NULL) {
					rte_pktmbuf_free_seg(txe->mbuf);
					txe->mbuf = NULL;
				}

				em_set_xmit_ctx_m2func(txq, tx_ol_req, hdrlen);

				#ifndef EM_SVE_512
				txe->last_id = tx_last;
				tx_id = txe->next_id;
				#endif
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
        }

        /* First 64B: Combine descriptor and first 56B of packet data */
        m_seg = tx_pkt;
		#ifndef EM_SVE_512
		txn = &sw_ring[txe->next_id]; // DPDK TX ring entry - next to txe
		#endif
		if (txe->mbuf != NULL) // DPDK TX ring entry - current
				rte_pktmbuf_free_seg(txe->mbuf);
		txe->mbuf = m_seg; // set DPDK TX ring entry - current

		/* 
		 * Set up Transmit Data Descriptor.
		 */
        uint8_t tx_buffer[64];  // Temporary buffer to hold 64B

        /* Pack descriptor (cmd_type_len + olinfo_status) into the first 8B of tx_buffer */
		if (txq->nb_tx_used >= txq->tx_rs_thresh) {
			PMD_TX_FREE_LOG(WARNING,
					"Setting RS bit on TXD id=%4u "
					"(port=%d queue=%d)",
					tx_last, txq->port_id, txq->queue_id);

			cmd_type_len |= E1000_ADVTXD_DCMD_RS;
			txq->nb_tx_used = 0;
		}
		/* without scattered packets, EOP is set */
		cmd_type_len |= E1000_ADVTXD_DCMD_EOP;
		*((uint32_t *)tx_buffer) = rte_cpu_to_le_32(cmd_type_len | m_seg->data_len); 
        *((uint32_t *)(tx_buffer + 4)) = rte_cpu_to_le_32(olinfo_status);

        /* Copy first 56B of the packet data into tx_buffer */
		seg_buf = rte_pktmbuf_mtod(m_seg, void *);
		unsigned flit_real_payload_len = payload_len_within_flit; //length to copy in the first flit
		unsigned remain_payload_len = m_seg->data_len; //remaining length to send
		void * tx_buffer_ptr = tx_buffer + desc_len; //pointer to the payload in tx_buffer

		if (m_seg->data_len > payload_len_within_flit) {
			flit_real_payload_len = payload_len_within_flit;
			remain_payload_len = m_seg->data_len - payload_len_within_flit;
		} else {
			flit_real_payload_len = m_seg->data_len;
			remain_payload_len = 0;
		}

		rte_memcpy(tx_buffer_ptr, seg_buf, (size_t) flit_real_payload_len);

		/* Send the first 64B (descriptor + packet data) */
        E1000_PCI_REG_WRITE64B(txq->tx_m2func_reg_addr, tx_buffer);

        /* Now send the rest of the packet data in 64B chunks */
		offset = flit_real_payload_len;
        while (offset < m_seg->data_len) {
			E1000_PCI_REG_WRITE64B(txq->tx_m2func_reg_addr, ((char *) seg_buf + offset));
            offset += flit_len;
        }

		#ifndef EM_SVE_512
		txe->last_id = tx_last;
		tx_id = txe->next_id;
		#endif
		txe = txn;

        /* Handle multiple segments in case of scattered packets - Not supported now!! */
        m_seg = m_seg->next;
        if (m_seg != NULL) {
			PMD_TX_FREE_LOG(WARNING, "Scattered packets not supported");
			if (nb_tx == 0)
				return 0;
			goto end_of_tx;
		}
    }
end_of_tx:
	rte_wmb();

    /* Final update of tail ID */
	PMD_TX_LOG(WARNING, "port_id=%u queue_id=%u tx_tail=%u nb_tx=%u",
		(unsigned) txq->port_id, (unsigned) txq->queue_id,
		(unsigned) tx_id, (unsigned) nb_tx);
    txq->tx_tail = tx_id;

    return nb_tx;
}

uint16_t
eth_em_xmit_pkts_m2func_dta(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts)
{
	// JM - SET THIS VALUE!
	bool zero_copy = true;
	struct em_tx_queue *txq;
	txq = tx_queue;
    struct rte_mbuf     *tx_pkt;
    uint32_t olinfo_status;
    uint32_t cmd_type_len; 
    uint32_t pkt_len;
	uint64_t ol_flags;
	uint64_t tx_ol_req;
	uint32_t ctx;
	uint32_t new_ctx;
	union em_vlan_macip hdrlen;

	uint64_t descriptor_addr = 0;
	uint64_t completion_addr = 0;
	struct e1000_adv_tx_desc_m2func *descriptor_buffer;
	volatile int64_t *completion_buffer;
	uint64_t mbuf_addr_buffer[nb_pkts];
	int64_t completed_pkts = 0;

	uint16_t xmit_empty_threshold = 100;
	uint16_t xmit_empty_count = 0;

	struct e1000_adv_tx_desc_m2func txd;   

	descriptor_buffer = txq->descriptor_buffer;
	descriptor_addr = rte_cpu_to_le_64(txq->descriptor_addr);

	completion_buffer = txq->completion_buffer;
	completion_addr = rte_cpu_to_le_64(txq->completion_addr);

	if (!zero_copy) {
		// Fill descriptor and mbuf address buffers
		for (uint16_t i = 0; i < nb_pkts; i++) {
			tx_pkt = *tx_pkts++;
			// Descriptor
			pkt_len = tx_pkt->pkt_len;
			ol_flags = tx_pkt->ol_flags;
			tx_ol_req = (ol_flags & E1000_TX_OFFLOAD_MASK);
			if (tx_ol_req) {
				hdrlen.f.vlan_tci = tx_pkt->vlan_tci;
				hdrlen.f.l2_len = tx_pkt->l2_len;
				hdrlen.f.l3_len = tx_pkt->l3_len;
				hdrlen.f.l4_len = tx_pkt->l4_len;
				hdrlen.f.tso_segsz = tx_pkt->tso_segsz;
				tx_ol_req = check_tso_para(tx_ol_req, hdrlen);
				ctx = what_ctx_update(txq, tx_ol_req, hdrlen);
				new_ctx = (ctx == EM_CTX_NUM);
			}
			cmd_type_len = E1000_ADVTXD_DTYP_DATA | E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT;
			if (tx_ol_req & PKT_TX_TCP_SEG)
				pkt_len -= (tx_pkt->l2_len + tx_pkt->l3_len + tx_pkt->l4_len);
			olinfo_status = (pkt_len << E1000_ADVTXD_PAYLEN_SHIFT);
			if (tx_ol_req) {
				if (new_ctx) {
					// TODO - JM: Implement context descriptor
				}
				cmd_type_len  |= tx_desc_vlan_flags_to_cmdtype(tx_ol_req);
				olinfo_status |= tx_desc_cksum_flags_to_olinfo(tx_ol_req);
				olinfo_status |= (ctx << E1000_ADVTXD_IDX_SHIFT);
			}
			/* without scattered packets, EOP is set */
			cmd_type_len |= E1000_ADVTXD_DCMD_EOP;
			descriptor_buffer[i].cmd_type_len = rte_cpu_to_le_32(cmd_type_len | tx_pkt->data_len);
			descriptor_buffer[i].olinfo_status = rte_cpu_to_le_32(olinfo_status);
			
			// Mbuf address
			mbuf_addr_buffer[i] = rte_cpu_to_le_64(rte_mbuf_data_iova(tx_pkt));
			
		}

		// Submit job to DTA
		// Create a job submission packet
		uint16_t total_packets = nb_pkts;
		uint16_t packet_index = 0;
		while (total_packets > 0) {
			uint16_t current_batch_size = 0;

			if (packet_index == 0) {
				current_batch_size = total_packets > M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE ? M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE : total_packets;
			} else {
				current_batch_size = total_packets > M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE ? M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE : total_packets;
			}

			// Fill job packet
			// Packet format
			// 1. descriptor address (8B)
			// 2. completion address (8B)
			// 3. number of packets (8B)
			// ---------24 bytes metadata---------
			// 4. mbuf addresses (8B * x)

			// If packet index is 0, fill the metadata
			if (packet_index == 0) {
				txq->zero_copy_job_packet[2] = rte_cpu_to_le_64(nb_pkts);
				// Add mbuf addresses
				memcpy(&(txq->zero_copy_job_packet[3]), mbuf_addr_buffer, current_batch_size * sizeof(uint64_t));
				
				// Submit the job to DTA
				E1000_PCI_REG_WRITE64B(txq->dta_job_submit_reg_addr, txq->zero_copy_job_packet);
			} else {
				uint64_t job_packet[M2FUNC_DTA_FLIT_ARRAY_SIZE] = {0}; // 64B job packet
				// Add mbuf addresses
				memcpy(job_packet, &mbuf_addr_buffer[M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE + (packet_index - 1) * M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE], current_batch_size * sizeof(uint64_t));
				// printf("DPDK[TX]: packet index[%d]\n", packet_index);
				// for (int i = 0; i < M2FUNC_DTA_FLIT_ARRAY_SIZE; i++) {
				// 	printf("DPDK[TX]: job_packet[%d]: %ld\n", i, job_packet[i]);
				// }
				// Submit the job to DTA
				E1000_PCI_REG_WRITE64B(txq->dta_job_submit_reg_addr, job_packet);
			}			

			// Update counters
			total_packets -= current_batch_size;
			packet_index++;
		}
	} else {
		// Zero-copy. So don't need to send mbuf addresses to DTA

		// Fill the descriptor buffer
		for (uint16_t i = 0; i < nb_pkts; i++) {
			tx_pkt = *tx_pkts++;
			// Descriptor
			pkt_len = tx_pkt->pkt_len;
			ol_flags = tx_pkt->ol_flags;
			tx_ol_req = (ol_flags & E1000_TX_OFFLOAD_MASK);
			if (tx_ol_req) {
				hdrlen.f.vlan_tci = tx_pkt->vlan_tci;
				hdrlen.f.l2_len = tx_pkt->l2_len;
				hdrlen.f.l3_len = tx_pkt->l3_len;
				hdrlen.f.l4_len = tx_pkt->l4_len;
				hdrlen.f.tso_segsz = tx_pkt->tso_segsz;
				tx_ol_req = check_tso_para(tx_ol_req, hdrlen);
				ctx = what_ctx_update(txq, tx_ol_req, hdrlen);
				new_ctx = (ctx == EM_CTX_NUM);
			}
			cmd_type_len = E1000_ADVTXD_DTYP_DATA | E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT;
			if (tx_ol_req & PKT_TX_TCP_SEG)
				pkt_len -= (tx_pkt->l2_len + tx_pkt->l3_len + tx_pkt->l4_len);
			olinfo_status = (pkt_len << E1000_ADVTXD_PAYLEN_SHIFT);
			if (tx_ol_req) {
				if (new_ctx) {
					// TODO - JM: Implement context descriptor
				}
				cmd_type_len  |= tx_desc_vlan_flags_to_cmdtype(tx_ol_req);
				olinfo_status |= tx_desc_cksum_flags_to_olinfo(tx_ol_req);
				olinfo_status |= (ctx << E1000_ADVTXD_IDX_SHIFT);
			}
			/* without scattered packets, EOP is set */
			cmd_type_len |= E1000_ADVTXD_DCMD_EOP;
			descriptor_buffer[i].cmd_type_len = rte_cpu_to_le_32(cmd_type_len | tx_pkt->data_len);
			descriptor_buffer[i].olinfo_status = rte_cpu_to_le_32(olinfo_status);
		}

		// Submit job to DTA
		// Create a job submission packet - only send descriptor addresses, completion address and number of packets
		// Set txq->zero_copy_job_packet[2] with nb_pkts
		txq->zero_copy_job_packet[2] = rte_cpu_to_le_64(nb_pkts);
		// printf("DPDK[TX]: Submitting job to DTA\n");
		// fflush(stdout);
		// Submit the job to DTA
		E1000_PCI_REG_WRITE64B(txq->dta_job_submit_reg_addr, txq->zero_copy_job_packet);

	}

	rte_mb();

	// Poll for completion
	while (completed_pkts < (int64_t) nb_pkts) {
		completed_pkts = rte_le_to_cpu_64(*completion_buffer); //64B buffer
		if (completed_pkts == -1) {
			PMD_TX_LOG(ERR, "TX Error in DTA processing");
			return 0;
		} 
		
		if (completed_pkts == 0) {
			xmit_empty_count++;
			if (xmit_empty_count > xmit_empty_threshold) {
				PMD_TX_LOG(ERR, "TX Timeout in DTA processing");
				break;
			}
		} else {
			xmit_empty_count = 0;
		}
	}

	rte_mb();

	// printf("DPDK[TX]: Completed %ld packets\n", completed_pkts);
	// fflush(stdout);

	if (completed_pkts == 0) {
		printf("DPDK[TX]: No packets completed. So return\n");
		return 0;
	}

	// Initialize the completion buffer
	*completion_buffer = 0;

	// Free the mbufs
	for (uint16_t i = 0; i < nb_pkts; i++) {
		rte_pktmbuf_free(tx_pkts[i]);
	}

	return nb_pkts;
}

uint16_t
eth_em_xmit_pkts_m2func_dta_double_comp(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts)
{
	struct em_tx_queue *txq;
	txq = tx_queue;
    struct rte_mbuf     *tx_pkt;
	uint16_t nb_tx;
    uint32_t olinfo_status;
    uint32_t cmd_type_len; 
    uint32_t pkt_len;
	uint64_t ol_flags;
	uint64_t tx_ol_req;
	uint32_t ctx;
	uint32_t new_ctx;
	union em_vlan_macip hdrlen;

	struct e1000_adv_tx_desc_m2func *descriptor_buffer;
	struct e1000_adv_tx_desc_m2func *descriptor_buffer2;
	volatile int64_t *completion_buffer;
	volatile int64_t *completion_buffer2;
	uint64_t mbuf_addr_buffer[nb_pkts];
	int64_t completed_pkts = 0;

	uint16_t xmit_empty_threshold = 100;
	uint16_t xmit_empty_count = 0;

	struct e1000_adv_tx_desc_m2func txd;   

	descriptor_buffer = txq->descriptor_buffer;
	descriptor_buffer2 = txq->descriptor_buffer2;

	completion_buffer = txq->completion_buffer;
	completion_buffer2 = txq->completion_buffer2;

	// Fill descriptor and mbuf address buffers
	for (nb_tx = 0; nb_tx < nb_pkts; nb_tx++) {
		tx_pkt = *tx_pkts++;
		// Descriptor
		pkt_len = tx_pkt->pkt_len;
		ol_flags = tx_pkt->ol_flags;
		tx_ol_req = (ol_flags & E1000_TX_OFFLOAD_MASK);
		if (tx_ol_req) {
			hdrlen.f.vlan_tci = tx_pkt->vlan_tci;
			hdrlen.f.l2_len = tx_pkt->l2_len;
			hdrlen.f.l3_len = tx_pkt->l3_len;
			hdrlen.f.l4_len = tx_pkt->l4_len;
			hdrlen.f.tso_segsz = tx_pkt->tso_segsz;
			tx_ol_req = check_tso_para(tx_ol_req, hdrlen);
			ctx = what_ctx_update(txq, tx_ol_req, hdrlen);
			new_ctx = (ctx == EM_CTX_NUM);
		}
		cmd_type_len = E1000_ADVTXD_DTYP_DATA | E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT;
		if (tx_ol_req & PKT_TX_TCP_SEG)
			pkt_len -= (tx_pkt->l2_len + tx_pkt->l3_len + tx_pkt->l4_len);
		olinfo_status = (pkt_len << E1000_ADVTXD_PAYLEN_SHIFT);
		if (tx_ol_req) {
			if (new_ctx) {
				// TODO - JM: Implement context descriptor
			}
			cmd_type_len  |= tx_desc_vlan_flags_to_cmdtype(tx_ol_req);
			olinfo_status |= tx_desc_cksum_flags_to_olinfo(tx_ol_req);
			olinfo_status |= (ctx << E1000_ADVTXD_IDX_SHIFT);
		}
		/* without scattered packets, EOP is set */
		cmd_type_len |= E1000_ADVTXD_DCMD_EOP;
		if (txq->completion_buffer_index == 0) {
			descriptor_buffer[nb_tx].cmd_type_len = rte_cpu_to_le_32(cmd_type_len | tx_pkt->data_len);
			descriptor_buffer[nb_tx].olinfo_status = rte_cpu_to_le_32(olinfo_status);
		} else {
			descriptor_buffer2[nb_tx].cmd_type_len = rte_cpu_to_le_32(cmd_type_len | tx_pkt->data_len);
			descriptor_buffer2[nb_tx].olinfo_status = rte_cpu_to_le_32(olinfo_status);
		}
		
		// Mbuf address
		mbuf_addr_buffer[nb_tx] = rte_cpu_to_le_64(rte_mbuf_data_iova(tx_pkt));
		// Store the tx_pkt pointer in the txq->tx_bufs array for the mbuf free operation - done at the next phase
		if (txq->completion_buffer_index == 0) {
			txq->tx_bufs[nb_tx] = tx_pkt;
		} else {
			txq->tx_bufs2[nb_tx] = tx_pkt;
		}
		
	}

	// Submit job to DTA
	// Create a job submission packet
	uint16_t total_packets = nb_pkts;
	uint16_t packet_index = 0;
	while (total_packets > 0) {
		uint16_t current_batch_size = 0;

		if (packet_index == 0) {
			current_batch_size = total_packets > M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE ? M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE : total_packets;
		} else {
			current_batch_size = total_packets > M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE ? M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE : total_packets;
		}

		// Fill job packet
		// Packet format
		// 1. descriptor address (8B)
		// 2. completion address (8B)
		// 3. number of packets (8B)
		// ---------24 bytes metadata---------
		// 4. mbuf addresses (8B * x)

		// If packet index is 0, fill the metadata
		if (packet_index == 0) {
			if (txq->completion_buffer_index == 0) {
				txq->zero_copy_job_packet[2] = rte_cpu_to_le_64(nb_pkts);
				// Add mbuf addresses
				memcpy(&(txq->zero_copy_job_packet[3]), mbuf_addr_buffer, current_batch_size * sizeof(uint64_t));
				
				// Submit the job to DTA
				E1000_PCI_REG_WRITE64B(txq->dta_job_submit_reg_addr, txq->zero_copy_job_packet);
			} else {
				txq->zero_copy_job_packet2[2] = rte_cpu_to_le_64(nb_pkts);
				// Add mbuf addresses
				memcpy(&(txq->zero_copy_job_packet2[3]), mbuf_addr_buffer, current_batch_size * sizeof(uint64_t));

				// Submit the job to DTA
				E1000_PCI_REG_WRITE64B(txq->dta_job_submit_reg_addr, txq->zero_copy_job_packet2);
			}
		} else {
			uint64_t job_packet[M2FUNC_DTA_FLIT_ARRAY_SIZE] = {0}; // 64B job packet
			// Add mbuf addresses
			memcpy(job_packet, &mbuf_addr_buffer[M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE + (packet_index - 1) * M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE], current_batch_size * sizeof(uint64_t));
			// printf("DPDK[TX]: packet index[%d]\n", packet_index);
			// for (int i = 0; i < M2FUNC_DTA_FLIT_ARRAY_SIZE; i++) {
			// 	printf("DPDK[TX]: job_packet[%d]: %ld\n", i, job_packet[i]);
			// }
			// Submit the job to DTA
			E1000_PCI_REG_WRITE64B(txq->dta_job_submit_reg_addr, job_packet);
		}			

		// Update counters
		total_packets -= current_batch_size;
		packet_index++;
	}

	if (!txq->started) {
		txq->started = true;
		printf("DPDK[TX]: Load_gen is started. So set txq->started to true. & Not polling\n");
		fflush(stdout);

	} else {
		rte_mb();

		// Poll for completion for the previous batch's completion buffer
		// If current txq->completion_buffer_index is 0, then poll for completion_buffer2
		completed_pkts = (txq->completion_buffer_index == 0) ? rte_le_to_cpu_64(*completion_buffer2) : rte_le_to_cpu_64(*completion_buffer);
		while (completed_pkts < (int64_t) nb_pkts) {
			completed_pkts = (txq->completion_buffer_index == 0) ? rte_le_to_cpu_64(*completion_buffer2) : rte_le_to_cpu_64(*completion_buffer); //64B buffer
			if (completed_pkts == -1) {
				// Free the mbufs
				for (uint16_t i = 0; i < nb_pkts; i++) {
					if (txq->completion_buffer_index == 0) {
						rte_pktmbuf_free_seg(txq->tx_bufs2[i]);
					} else {
						rte_pktmbuf_free_seg(txq->tx_bufs[i]);
					}
				}

				// Initialize the completion buffer
				if (txq->completion_buffer_index == 0) {
					*completion_buffer2 = 0;
				} else {
					*completion_buffer = 0;
				}

				// Set the completion buffer index
				txq->completion_buffer_index = (txq->completion_buffer_index == 0) ? 1 : 0;

				PMD_TX_LOG(ERR, "TX Error in DTA processing");
				printf("DPDK[TX]: TX Error in DTA processing. Completed packets: -1. Changed Completion buffer index: %d\n", txq->completion_buffer_index);
				fflush(stdout);
				return 0;
			} 
			
			if (completed_pkts == 0) {
				xmit_empty_count++;
				if (xmit_empty_count > xmit_empty_threshold) {
					PMD_TX_LOG(ERR, "TX Timeout in DTA processing");
					printf("DPDK[TX]: TX Timeout in DTA processing. Completed packets: 0. Completion buffer index: %d\n", txq->completion_buffer_index);
					fflush(stdout);
					break;
				}
			} else {
				xmit_empty_count = 0;
			}
		}

		rte_mb();

		// printf("DPDK[TX]: Completed %ld packets\n", completed_pkts);
		// fflush(stdout);

		if (completed_pkts == 0) {
			printf("DPDK[TX]: No packets completed. So return\n");
			fflush(stdout);
			return 0;
		}

		// Free the mbufs
		for (int64_t i = 0; i < completed_pkts; i++) {
			if (txq->completion_buffer_index == 0) {
				rte_pktmbuf_free_seg(txq->tx_bufs2[i]);
			} else {
				rte_pktmbuf_free_seg(txq->tx_bufs[i]);
			}
		}

		// Initialize the completion buffer
		if (txq->completion_buffer_index == 0) {
			*completion_buffer2 = 0;
		} else {
			*completion_buffer = 0;
		}
	}

	// Set the completion buffer index
	txq->completion_buffer_index = (txq->completion_buffer_index == 0) ? 1 : 0;

	return nb_tx;
}

uint16_t
eth_em_xmit_pkts_m2func_dta_double_comp_sve512(void *tx_queue, struct rte_mbuf **tx_pkts,
		uint16_t nb_pkts)
{
	struct em_tx_queue *txq;
	txq = tx_queue;
    struct rte_mbuf     *tx_pkt;
	uint16_t nb_tx;
    uint32_t olinfo_status;
    uint32_t cmd_type_len; 
    uint32_t pkt_len;
	uint64_t ol_flags;
	uint64_t tx_ol_req;
	uint32_t ctx;
	uint32_t new_ctx;
	union em_vlan_macip hdrlen;

	struct e1000_adv_tx_desc_m2func *descriptor_buffer;
	struct e1000_adv_tx_desc_m2func *descriptor_buffer2;
	volatile int64_t *completion_buffer;
	volatile int64_t *completion_buffer2;
	uint64_t mbuf_addr_buffer[nb_pkts];
	int64_t completed_pkts = 0;

	uint16_t xmit_empty_threshold = 100;
	uint16_t xmit_empty_count = 0;

	struct e1000_adv_tx_desc_m2func txd;   

	descriptor_buffer = txq->descriptor_buffer;
	descriptor_buffer2 = txq->descriptor_buffer2;

	completion_buffer = txq->completion_buffer;
	completion_buffer2 = txq->completion_buffer2;

	// Fill descriptor and mbuf address buffers
	for (nb_tx = 0; nb_tx < nb_pkts; nb_tx++) {
		tx_pkt = *tx_pkts++;
		// Descriptor
		pkt_len = tx_pkt->pkt_len;
		ol_flags = tx_pkt->ol_flags;
		tx_ol_req = (ol_flags & E1000_TX_OFFLOAD_MASK);
		if (tx_ol_req) {
			hdrlen.f.vlan_tci = tx_pkt->vlan_tci;
			hdrlen.f.l2_len = tx_pkt->l2_len;
			hdrlen.f.l3_len = tx_pkt->l3_len;
			hdrlen.f.l4_len = tx_pkt->l4_len;
			hdrlen.f.tso_segsz = tx_pkt->tso_segsz;
			tx_ol_req = check_tso_para(tx_ol_req, hdrlen);
			ctx = what_ctx_update(txq, tx_ol_req, hdrlen);
			new_ctx = (ctx == EM_CTX_NUM);
		}
		cmd_type_len = E1000_ADVTXD_DTYP_DATA | E1000_ADVTXD_DCMD_IFCS | E1000_ADVTXD_DCMD_DEXT;
		if (tx_ol_req & PKT_TX_TCP_SEG)
			pkt_len -= (tx_pkt->l2_len + tx_pkt->l3_len + tx_pkt->l4_len);
		olinfo_status = (pkt_len << E1000_ADVTXD_PAYLEN_SHIFT);
		if (tx_ol_req) {
			if (new_ctx) {
				// TODO - JM: Implement context descriptor
			}
			cmd_type_len  |= tx_desc_vlan_flags_to_cmdtype(tx_ol_req);
			olinfo_status |= tx_desc_cksum_flags_to_olinfo(tx_ol_req);
			olinfo_status |= (ctx << E1000_ADVTXD_IDX_SHIFT);
		}
		/* without scattered packets, EOP is set */
		cmd_type_len |= E1000_ADVTXD_DCMD_EOP;
		if (txq->completion_buffer_index == 0) {
			descriptor_buffer[nb_tx].cmd_type_len = rte_cpu_to_le_32(cmd_type_len | tx_pkt->data_len);
			descriptor_buffer[nb_tx].olinfo_status = rte_cpu_to_le_32(olinfo_status);
		} else {
			descriptor_buffer2[nb_tx].cmd_type_len = rte_cpu_to_le_32(cmd_type_len | tx_pkt->data_len);
			descriptor_buffer2[nb_tx].olinfo_status = rte_cpu_to_le_32(olinfo_status);
		}
		
		// Mbuf address
		mbuf_addr_buffer[nb_tx] = rte_cpu_to_le_64(rte_mbuf_data_iova(tx_pkt));
		// Store the tx_pkt pointer in the txq->tx_bufs array for the mbuf free operation - done at the next phase
		if (txq->completion_buffer_index == 0) {
			txq->tx_bufs[nb_tx] = tx_pkt;
		} else {
			txq->tx_bufs2[nb_tx] = tx_pkt;
		}
		
	}

	// Submit job to DTA
	// Create a job submission packet
	uint16_t total_packets = nb_pkts;
	uint16_t packet_index = 0;
	while (total_packets > 0) {
		uint16_t current_batch_size = 0;

		if (packet_index == 0) {
			current_batch_size = total_packets > M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE ? M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE : total_packets;
		} else {
			current_batch_size = total_packets > M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE ? M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE : total_packets;
		}

		// Fill job packet
		// Packet format
		// 1. descriptor address (8B)
		// 2. completion address (8B)
		// 3. number of packets (8B)
		// ---------24 bytes metadata---------
		// 4. mbuf addresses (8B * x)

		// If packet index is 0, fill the metadata
		if (packet_index == 0) {
			if (txq->completion_buffer_index == 0) {
				txq->zero_copy_job_packet[2] = rte_cpu_to_le_64(nb_pkts);
				// Add mbuf addresses
				memcpy(&(txq->zero_copy_job_packet[3]), mbuf_addr_buffer, current_batch_size * sizeof(uint64_t));
				
				// Submit the job to DTA
				E1000_PCI_REG_WRITE64B(txq->dta_job_submit_reg_addr, txq->zero_copy_job_packet);
			} else {
				txq->zero_copy_job_packet2[2] = rte_cpu_to_le_64(nb_pkts);
				// Add mbuf addresses
				memcpy(&(txq->zero_copy_job_packet2[3]), mbuf_addr_buffer, current_batch_size * sizeof(uint64_t));

				// Submit the job to DTA
				E1000_PCI_REG_WRITE64B(txq->dta_job_submit_reg_addr, txq->zero_copy_job_packet2);
			}
		} else {
			uint64_t job_packet[M2FUNC_DTA_FLIT_ARRAY_SIZE] = {0}; // 64B job packet
			// Add mbuf addresses
			memcpy(job_packet, &mbuf_addr_buffer[M2FUNC_DTA_TX_FIRST_FLIT_BATCH_SIZE + (packet_index - 1) * M2FUNC_DTA_TX_REST_FLIT_BATCH_SIZE], current_batch_size * sizeof(uint64_t));
			// printf("DPDK[TX]: packet index[%d]\n", packet_index);
			// for (int i = 0; i < M2FUNC_DTA_FLIT_ARRAY_SIZE; i++) {
			// 	printf("DPDK[TX]: job_packet[%d]: %ld\n", i, job_packet[i]);
			// }
			// Submit the job to DTA
			E1000_PCI_REG_WRITE64B(txq->dta_job_submit_reg_addr, job_packet);
		}			

		// Update counters
		total_packets -= current_batch_size;
		packet_index++;
	}

	if (!txq->started) {
		txq->started = true;
		printf("DPDK[TX]: Load_gen is started. So set txq->started to true. & Not polling\n");
		fflush(stdout);

	} else {
		rte_mb();

		// Poll for completion for the previous batch's completion buffer
		// If current txq->completion_buffer_index is 0, then poll for completion_buffer2
		completed_pkts = (txq->completion_buffer_index == 0) ? rte_le_to_cpu_64(*completion_buffer2) : rte_le_to_cpu_64(*completion_buffer);
		while (completed_pkts < (int64_t) nb_pkts) {
			completed_pkts = (txq->completion_buffer_index == 0) ? rte_le_to_cpu_64(*completion_buffer2) : rte_le_to_cpu_64(*completion_buffer); //64B buffer
			if (completed_pkts == -1) {
				// Free the mbufs
				for (uint16_t i = 0; i < nb_pkts; i++) {
					if (txq->completion_buffer_index == 0) {
						rte_pktmbuf_free_seg(txq->tx_bufs2[i]);
					} else {
						rte_pktmbuf_free_seg(txq->tx_bufs[i]);
					}
				}

				// Initialize the completion buffer
				if (txq->completion_buffer_index == 0) {
					*completion_buffer2 = 0;
				} else {
					*completion_buffer = 0;
				}

				// Set the completion buffer index
				txq->completion_buffer_index = (txq->completion_buffer_index == 0) ? 1 : 0;

				PMD_TX_LOG(ERR, "TX Error in DTA processing");
				printf("DPDK[TX]: TX Error in DTA processing. Completed packets: -1. Changed Completion buffer index: %d\n", txq->completion_buffer_index);
				fflush(stdout);
				return 0;
			} 
			
			if (completed_pkts == 0) {
				xmit_empty_count++;
				if (xmit_empty_count > xmit_empty_threshold) {
					PMD_TX_LOG(ERR, "TX Timeout in DTA processing");
					printf("DPDK[TX]: TX Timeout in DTA processing. Completed packets: 0. Completion buffer index: %d\n", txq->completion_buffer_index);
					fflush(stdout);
					break;
				}
			} else {
				xmit_empty_count = 0;
			}
		}

		rte_mb();

		// printf("DPDK[TX]: Completed %ld packets\n", completed_pkts);
		// fflush(stdout);

		if (completed_pkts == 0) {
			printf("DPDK[TX]: No packets completed. So return\n");
			fflush(stdout);
			return 0;
		}

		// Free the mbufs
		for (int64_t i = 0; i < completed_pkts; i++) {
			if (txq->completion_buffer_index == 0) {
				rte_pktmbuf_free_seg(txq->tx_bufs2[i]);
			} else {
				rte_pktmbuf_free_seg(txq->tx_bufs[i]);
			}
		}

		// Initialize the completion buffer
		if (txq->completion_buffer_index == 0) {
			*completion_buffer2 = 0;
		} else {
			*completion_buffer = 0;
		}
	}

	// Set the completion buffer index
	txq->completion_buffer_index = (txq->completion_buffer_index == 0) ? 1 : 0;

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
		staterr = rxdp->wb.upper.status_error; //jm
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
		//jm
		pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.wb.upper.length) - 
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
		E1000_PCI_REG_WRITE(rxq->rdt_reg_addr, rx_id); // TODO - gem5, set rdt_reg_addr to print logs
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

#define EM_DESCS_PER_LOOP_SVE512 8
#define EM_RXQ_REARM_THRESH 32

#define PG16_128BIT		svwhilelt_b16(0, 8)
#define PG16_256BIT		svwhilelt_b16(0, 16)
#define PG32_256BIT		svwhilelt_b32(0, 8)
#define PG64_64BIT		svwhilelt_b64(0, 1)
#define PG64_128BIT		svwhilelt_b64(0, 2)
#define PG64_256BIT		svwhilelt_b64(0, 4)
#define PG64_ALLBIT		svptrue_b64()

#define DESC_SIZE      16

#define DESC_FIELD_PKTADDR 0 // u64
#define DESC_FIELD_HDRADDR 8 // u64

#define DESC_FIELD_HLEN_TYPE_RSS 0 //u32 - to get pkt_info, have to use lower 16 bits
#define DESC_FIELD_RSS 4 // u32
#define DESC_FIELD_STATERR 8 // u32
#define DESC_FIELD_XLEN 12 // u16
#define DESC_FIELD_VLAN 14 // u16

typedef struct {
	uint32_t staterr[EM_DESCS_PER_LOOP_SVE512];
	uint32_t hlen_type_rss[EM_DESCS_PER_LOOP_SVE512];
} EM_SVE_KEY_FIELD_S;

static inline void
eth_em_rx_prefetch_mbuf_sve(struct em_rx_entry *sw_ring)
{
	svuint64_t prf1st = svld1_u64(PG64_256BIT, (uint64_t *)&sw_ring[0]); // 4 mbuf pointers
	svuint64_t prf2st = svld1_u64(PG64_256BIT, (uint64_t *)&sw_ring[4]); // 4 mbuf pointers
	svprfd_gather_u64base(PG64_256BIT, prf1st, SV_PLDL1KEEP); // Prefetch mbuf's part to L1 $
	svprfd_gather_u64base(PG64_256BIT, prf2st, SV_PLDL1KEEP); // Prefetch mbuf's part to L1 $
}

static inline uint16_t 
_eth_em_recv_raw_pkts_vec_sve512(struct em_rx_queue *rxq, 
	struct rte_mbuf **rx_pkts, uint16_t nb_pkts)
{
#define XLEN_ADJUST_LEN		32
#define RSS_ADJUST_LEN		16
#define GEN_VLD_U8_ZIP_INDEX	svindex_s8(28, -4)
	uint16_t rx_id = rxq->rx_tail;
	printf("DPDK_SVE512[RX]: rx_tail: %d\n", rx_id);
	fflush(stdout);
	struct em_rx_entry *sw_ring = &rxq->sw_ring[rx_id];
	volatile union e1000_adv_rx_desc *rxdp = &rxq->rx_ring[rx_id];
	volatile union e1000_adv_rx_desc *rxdp2;
	EM_SVE_KEY_FIELD_S key_field;
	uint64_t desc_valid_num;
	uint16_t nb_rx = 0;
	int pos, offset;

	uint16_t xlen_adjust[XLEN_ADJUST_LEN] = {
		0,  0xffff, 1,  0xffff,    /* 1st mbuf: xlen */
		2,  0xffff, 3,  0xffff,    /* 2st mbuf: xlen */
		4,  0xffff, 5,  0xffff,    /* 3st mbuf: xlen */
		6,  0xffff, 7,  0xffff,    /* 4st mbuf: xlen */
		8,  0xffff, 9,  0xffff,    /* 5st mbuf: xlen */
		10, 0xffff, 11, 0xffff,    /* 6st mbuf: xlen */
		12, 0xffff, 13, 0xffff,    /* 7st mbuf: xlen */
		14, 0xffff, 15, 0xffff,    /* 8st mbuf: xlen */
	};

	uint32_t rss_adjust[RSS_ADJUST_LEN] = {
		0, 0xffff,        /* 1st mbuf: rss */
		1, 0xffff,        /* 2st mbuf: rss */
		2, 0xffff,        /* 3st mbuf: rss */
		3, 0xffff,        /* 4st mbuf: rss */
		4, 0xffff,        /* 5st mbuf: rss */
		5, 0xffff,        /* 6st mbuf: rss */
		6, 0xffff,        /* 7st mbuf: rss */
		7, 0xffff,        /* 8st mbuf: rss */
	};

	svbool_t pg32 = svwhilelt_b32(0, EM_DESCS_PER_LOOP_SVE512); // To load data only for 8 mbufs
	svuint16_t xlen_tbl1 = svld1_u16(PG16_256BIT, xlen_adjust);
	svuint16_t xlen_tbl2 = svld1_u16(PG16_256BIT, &xlen_adjust[16]);
	svuint32_t rss_tbl1 = svld1_u32(PG32_256BIT, rss_adjust);
	svuint32_t rss_tbl2 = svld1_u32(PG32_256BIT, &rss_adjust[8]);

	for (pos = 0; pos < nb_pkts; pos += EM_DESCS_PER_LOOP_SVE512,
					rxdp += EM_DESCS_PER_LOOP_SVE512) {
		svuint64_t vld_clz, mbp1st, mbp2st, mbuf_init;
		svuint64_t xlen1st, xlen2st, rss1st, rss2st, vlan1st, vlan2st, xlen_vlan_1st, xlen_vlan_2st;
		svuint32_t hlen_type_rss, vld, vld2, xlen_vlan, rss, xlen, vlan;
		svuint8_t  vld_u8;

		/* Calculate how many desc. valid: part 1*/
		vld = svld1_gather_u32offset_u32(pg32, (uint32_t *)rxdp,
			svindex_u32(DESC_FIELD_STATERR, DESC_SIZE)); // 8 status_error
		// Have to get only DD bit from status_error. DD bit offset from staterr is 0x1
		vld2 = svlsl_n_u32_z(pg32, vld,
				    32 - 1 - 1); //32-bit staterr, 1-bit DD bit, 1-bit offset 
		vld2 = svreinterpret_u32_s32(svasr_n_s32_z(pg32,
			svreinterpret_s32_u32(vld2), 32 - 1)); // Again shift to get only DD-bit
		
		/* load 4 mbuf pointer */
		mbp1st = svld1_u64(PG64_256BIT, (uint64_t *)&sw_ring[pos]);

		/* Calculate how many desc. valid: part 2*/
		vld_u8 = svtbl_u8(svreinterpret_u8_u32(vld2),
				  svreinterpret_u8_s8(GEN_VLD_U8_ZIP_INDEX));
		vld_clz = svnot_u64_z(PG64_64BIT, svreinterpret_u64_u8(vld_u8));
		vld_clz = svclz_u64_z(PG64_64BIT, vld_clz);
		svst1_u64(PG64_64BIT, &desc_valid_num, vld_clz);
		desc_valid_num /= 8; //8-bits

		/* load 4 more mbuf pointer */
		mbp2st = svld1_u64(PG64_256BIT, (uint64_t *)&sw_ring[pos + 4]);

		/* use offset to control below data load oper ordering */
		offset = rxq->offset_table[desc_valid_num];
		rxdp2 = rxdp + offset;

		/* store 4 mbuf pointer into rx_pkts */
		svst1_u64(PG64_256BIT, (uint64_t *)&rx_pkts[pos], mbp1st);

		/* load key field to vector reg */
		hlen_type_rss = svld1_gather_u32offset_u32(pg32, (uint32_t *)rxdp2,
				svindex_u32(DESC_FIELD_HLEN_TYPE_RSS, DESC_SIZE));
		rss = svld1_gather_u32offset_u32(pg32, (uint32_t *)rxdp2,
				svindex_u32(DESC_FIELD_RSS, DESC_SIZE));
		
		/* store 4 mbuf pointer into rx_pkts again */
		svst1_u64(PG64_256BIT, (uint64_t *)&rx_pkts[pos + 4], mbp2st);
		
		/* load xlen_vlan to extract datalen, pktlen and vlan*/
		xlen_vlan = svld1_gather_u32offset_u32(pg32, (uint32_t *)rxdp2,
                          svindex_u32(DESC_FIELD_XLEN, DESC_SIZE));
		xlen = svand_n_u32_z(PG32_256BIT, xlen_vlan, 0x0000FFFF); // extract lower 16 bits
		vlan = svlsr_n_u32_z(PG32_256BIT, xlen_vlan, 16); // extract upper 16 bits

		/* store key field to stash buffer */
		svst1_u32(pg32, (uint32_t *)key_field.hlen_type_rss, hlen_type_rss);
		svst1_u32(pg32, (uint32_t *)key_field.staterr, vld);

		/* sub crc_len for xlen */
		xlen = svsub_n_u32_z(PG32_256BIT, xlen, rxq->crc_len);

		/* init mbuf_initializer */
		mbuf_init = svdup_n_u64((uint64_t)rxq->mbuf_initializer);

		/* Make datalen, pktlen, vlan and rss */
		rss1st = svreinterpret_u64_u32(
			svtbl_u32(svreinterpret_u32_u32(rss), rss_tbl1));
		rss2st = svreinterpret_u64_u32(
			svtbl_u32(svreinterpret_u32_u32(rss), rss_tbl2));
        
        xlen1st = svreinterpret_u64_u16(
			svtbl_u16(svreinterpret_u16_u32(xlen), xlen_tbl1));
        xlen2st = svreinterpret_u64_u16(
                svtbl_u16(svreinterpret_u16_u32(xlen), xlen_tbl2));
        vlan1st = svreinterpret_u64_u16(
                svtbl_u16(svreinterpret_u16_u32(vlan), xlen_tbl1));
        vlan2st = svreinterpret_u64_u16(
                svtbl_u16(svreinterpret_u16_u32(vlan), xlen_tbl2));
        
        /* Make 64-bit pktlen_datalen_vlantci */
        xlen_vlan_1st = svorr_u64_z(PG64_256BIT,
            svlsl_n_u64_z(PG64_256BIT, vlan1st, 48), // VLAN_TCI MSB 16 bit
            svorr_u64_z(PG64_256BIT,
                svlsl_n_u64_z(PG64_256BIT, xlen1st, 32), // DATA_LEN Next MSB 16bit
                xlen1st)); // PKT_LEN LSB 32bit
        xlen_vlan_2st = svorr_u64_z(PG64_256BIT,
            svlsl_n_u64_z(PG64_256BIT, vlan2st, 48), // VLAN_TCI MSB 16 bit
            svorr_u64_z(PG64_256BIT,
                svlsl_n_u64_z(PG64_256BIT, xlen2st, 32), // DATA_LEN Next MSB 16bit
                xlen2st)); // PKT_LEN LSB 32bit

		/* save mbuf_initializer */
		svst1_scatter_u64base_offset_u64(PG64_256BIT, mbp1st,
			offsetof(struct rte_mbuf, rearm_data), mbuf_init);
		svst1_scatter_u64base_offset_u64(PG64_256BIT, mbp2st,
			offsetof(struct rte_mbuf, rearm_data), mbuf_init);
		
		/* save datalen,pktlen,vlan and rss */
		svst1_scatter_u64base_offset_u64(PG64_256BIT, mbp1st,
            offsetof(struct rte_mbuf, pkt_len), xlen_vlan_1st);
        svst1_scatter_u64base_offset_u64(PG64_256BIT, mbp1st,
            offsetof(struct rte_mbuf, hash.rss), rss1st);
        svst1_scatter_u64base_offset_u64(PG64_256BIT, mbp2st,
            offsetof(struct rte_mbuf, pkt_len), xlen_vlan_2st);
        svst1_scatter_u64base_offset_u64(PG64_256BIT, mbp2st,
            offsetof(struct rte_mbuf, hash.rss), rss2st);   
		
		rte_prefetch_non_temporal(rxdp +
					  EM_DESCS_PER_LOOP_SVE512);

		for (uint64_t i = 0; i < desc_valid_num; i++) {
			uint64_t pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(rxq, key_field.hlen_type_rss[i]);
			pkt_flags = pkt_flags | rx_desc_status_to_pkt_flags(key_field.staterr[i]);
			pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(key_field.staterr[i]);
			rx_pkts[pos + i]->ol_flags = pkt_flags;
			rx_pkts[pos + i]->packet_type = em_rxd_pkt_info_to_pkt_type((uint16_t)(key_field.hlen_type_rss[i]));
		}

		eth_em_rx_prefetch_mbuf_sve(&sw_ring[pos +
					EM_DESCS_PER_LOOP_SVE512]);

		nb_rx += desc_valid_num;
		if (unlikely(desc_valid_num < EM_DESCS_PER_LOOP_SVE512))
			break;
	}

	rxq->rx_tail += nb_rx;
	rxq->rxrearm_nb += nb_rx;
	if (rxq->rx_tail >= rxq->nb_rx_desc)
		rxq->rx_tail = 0;

	return nb_rx;
}

static __rte_always_inline void 
eth_em_rxq_rearm_common(struct em_rx_queue *rxq)
{
#define REARM_LOOP_STEP_NUM	4
	struct em_rx_entry *rxep = &rxq->sw_ring[rxq->rxrearm_start];
	volatile union e1000_adv_rx_desc *rxdp = rxq->rx_ring + rxq->rxrearm_start;
	int i;
	uint16_t rx_id;

	/* Pull 'n' more MBUFs into the software ring */
	if (rte_mempool_get_bulk(rxq->mb_pool,
				 (void *)rxep,
				 EM_RXQ_REARM_THRESH) < 0) {
		if (rxq->rxrearm_nb + EM_RXQ_REARM_THRESH >=
		    rxq->nb_rx_desc) {
			svuint64_t dma_addr0 = svdup_n_u64(0); // 64-bit 0 value

			for (int i = 0; i < REARM_LOOP_STEP_NUM; i++) {
				rxep[i].mbuf = &rxq->fake_mbuf;
			}

			// Descriptor rearm
			svst1_scatter_u64offset_u64(PG64_256BIT, // 4 desc
				(uint64_t *)&rxdp[0].read.pkt_addr,
				svindex_u64(DESC_FIELD_PKTADDR, DESC_SIZE), 
				dma_addr0);
			svst1_scatter_u64offset_u64(PG64_256BIT, // 4 desc
				(uint64_t *)&rxdp[0].read.pkt_addr,
				svindex_u64(DESC_FIELD_HDRADDR, DESC_SIZE), 
				dma_addr0);
		}
		rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed +=
			EM_RXQ_REARM_THRESH;
		return;
	}

	/* fill up the rxd in vector, process 8 mbufs in one loop */
	svuint64_t hdr_addr0 = svdup_n_u64(0);
	for (i = 0; i < EM_RXQ_REARM_THRESH; i += 8) {
		uint64_t iova[8];
		iova[0] = rxep[0].mbuf->buf_iova;
		iova[1] = rxep[1].mbuf->buf_iova;
		iova[2] = rxep[2].mbuf->buf_iova;
		iova[3] = rxep[3].mbuf->buf_iova;
		iova[4] = rxep[4].mbuf->buf_iova;
		iova[5] = rxep[5].mbuf->buf_iova;
		iova[6] = rxep[6].mbuf->buf_iova;
		iova[7] = rxep[7].mbuf->buf_iova;
		svuint64_t siova = svld1_u64(PG64_ALLBIT, iova);
		svuint64_t iova_addrs = svadd_n_u64_z(PG64_ALLBIT, siova,
			RTE_PKTMBUF_HEADROOM);
		svst1_scatter_u64offset_u64(PG64_ALLBIT,
			(uint64_t *)&rxdp[0].read.pkt_addr,
			svindex_u64(DESC_FIELD_PKTADDR, DESC_SIZE), iova_addrs);
		svst1_scatter_u64offset_u64(PG64_ALLBIT,
			(uint64_t *)&rxdp[0].read.pkt_addr,
			svindex_u64(DESC_FIELD_HDRADDR, DESC_SIZE), 
			hdr_addr0);
		
		rxep += 8, rxdp += 8;
	}

	rxq->rxrearm_start += EM_RXQ_REARM_THRESH;
	if (rxq->rxrearm_start >= rxq->nb_rx_desc)
		rxq->rxrearm_start = 0;

	rxq->rxrearm_nb -= EM_RXQ_REARM_THRESH;

	rx_id = (uint16_t)((rxq->rxrearm_start == 0) ?
			     (rxq->nb_rx_desc - 1) : (rxq->rxrearm_start - 1));

	/* Update the tail pointer on the NIC */
	E1000_PCI_REG_WRITE(rxq->rdt_reg_addr, rx_id);

}

static __rte_always_inline void 
eth_em_rxq_rearm(struct em_rx_queue *rxq)
{
#define REARM_LOOP_STEP_NUM	4
	struct em_rx_entry *rxep = &rxq->sw_ring[rxq->rxrearm_start];
	volatile union e1000_adv_rx_desc *rxdp = rxq->rx_ring + rxq->rxrearm_start;
	struct rte_mempool_cache *cache = rte_mempool_default_cache(rxq->mb_pool,
			rte_lcore_id());
	int i;
	uint16_t rx_id;

	if (unlikely(!cache))
		return eth_em_rxq_rearm_common(rxq);
	
	/* We need to pull 'n' more mbufs into the sw ring*/
	if (cache->len < EM_RXQ_REARM_THRESH) {
		uint32_t req = EM_RXQ_REARM_THRESH + (cache->size -
				cache->len);

		int ret = rte_mempool_ops_dequeue_bulk(rxq->mb_pool,
				&cache->objs[cache->len], req);
		if (ret == 0) {
			cache->len += req;
		} else {
			if (rxq->rxrearm_nb + EM_RXQ_REARM_THRESH >= 
				rxq->nb_rx_desc) {
				svuint64_t dma_addr0 = svdup_n_u64(0); // 64-bit 0 value

				for (int i = 0; i < REARM_LOOP_STEP_NUM; i++) {
					rxep[i].mbuf = &rxq->fake_mbuf;
				}

				// Descriptor rearm
				svst1_scatter_u64offset_u64(PG64_256BIT, // 4 desc
					(uint64_t *)&rxdp[0].read.pkt_addr,
					svindex_u64(DESC_FIELD_PKTADDR, DESC_SIZE), 
					dma_addr0);
				svst1_scatter_u64offset_u64(PG64_256BIT, // 4 desc
					(uint64_t *)&rxdp[0].read.pkt_addr,
					svindex_u64(DESC_FIELD_HDRADDR, DESC_SIZE), 
					dma_addr0);
			}
			rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed +=
				EM_RXQ_REARM_THRESH;
			return;
		}
	}

	/* fill up the rxd in vector, process 8 mbufs in one loop */
	svuint64_t hdr_addr0 = svdup_n_u64(0);
	for (i = 0; i < EM_RXQ_REARM_THRESH; i += 8) {
		svuint64_t mbuf_ptrs = svld1_u64(PG64_ALLBIT, (uint64_t *)(&cache->objs[cache->len - 8]));
		svst1_u64(PG64_ALLBIT, (uint64_t *)&rxep[0], mbuf_ptrs);
		svuint64_t iova_base_addrs = svld1_gather_u64base_offset_u64(PG64_ALLBIT,
			mbuf_ptrs, offsetof(struct rte_mbuf, buf_iova));
		svuint64_t iova_addrs = svadd_n_u64_z(PG64_ALLBIT, iova_base_addrs,
			RTE_PKTMBUF_HEADROOM);
		svst1_scatter_u64offset_u64(PG64_ALLBIT,
			(uint64_t *)&rxdp[0].read.pkt_addr,
			svindex_u64(DESC_FIELD_PKTADDR, DESC_SIZE), iova_addrs);
		svst1_scatter_u64offset_u64(PG64_ALLBIT,
			(uint64_t *)&rxdp[0].read.pkt_addr,
			svindex_u64(DESC_FIELD_HDRADDR, DESC_SIZE), 
			hdr_addr0);
		
		rxep += 8, rxdp += 8, cache->len -= 8;
	}

	rxq->rxrearm_start += EM_RXQ_REARM_THRESH;
	if (rxq->rxrearm_start >= rxq->nb_rx_desc)
		rxq->rxrearm_start = 0;

	rxq->rxrearm_nb -= EM_RXQ_REARM_THRESH;

	rx_id = (uint16_t)((rxq->rxrearm_start == 0) ?
			     (rxq->nb_rx_desc - 1) : (rxq->rxrearm_start - 1));

	/* Update the tail pointer on the NIC */
	printf("DPDK_SVE512[RX]: rx_rearm_start: %d\n", rxq->rxrearm_start);
	fflush(stdout);
	E1000_PCI_REG_WRITE(rxq->rdt_reg_addr, rx_id);

}

uint16_t 
eth_em_recv_pkts_sve512(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts)
{
	struct em_rx_queue *rxq = rx_queue;
	struct em_rx_entry *sw_ring = &rxq->sw_ring[rxq->rx_tail];
	volatile union e1000_adv_rx_desc *rxdp = rxq->rx_ring + rxq->rx_tail;
	uint16_t nb_rx;

	rte_prefetch0(rxdp);
	
	/* nb_pkts has to be floor-aligned to DESCS_PER_LOOP_SVE512 8*/
	nb_pkts = RTE_ALIGN_FLOOR(nb_pkts, EM_DESCS_PER_LOOP_SVE512);

	/* See if we need to rearm the RX queue - gives the prefetch a bit
	 * of time to act
	 */
	if (rxq->rxrearm_nb > EM_RXQ_REARM_THRESH)
		eth_em_rxq_rearm(rxq);
	
	/* Before we start moving massive data around, check to see if
	 * there is actually a packet available
	 */
	if (!(rxdp->wb.upper.status_error & rte_cpu_to_le_32(E1000_RXD_STAT_DD)))
		return 0;
	
	/* Prefetch 8 mbuf */
	eth_em_rx_prefetch_mbuf_sve(sw_ring);

	if (likely(nb_pkts <= EM_RXQ_REARM_THRESH)) {
		nb_rx = _eth_em_recv_raw_pkts_vec_sve512(rxq, rx_pkts, nb_pkts);
		return nb_rx;
	}

	nb_rx = 0;
	while (nb_pkts > 0) {
		uint16_t ret, n;

		n = RTE_MIN(nb_pkts, EM_RXQ_REARM_THRESH);
		ret = _eth_em_recv_raw_pkts_vec_sve512(rxq, &rx_pkts[nb_rx], n);
		nb_pkts -= ret;
		nb_rx += ret;

		if (ret < n)
			break;
		
		if (rxq->rxrearm_nb > EM_RXQ_REARM_THRESH)
			eth_em_rxq_rearm(rxq);
	}

	return nb_rx;	
}

uint16_t
eth_em_recv_pkts_m2func(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts)
{
	struct em_rx_queue *rxq;
	struct em_rx_entry *sw_ring;
	struct em_rx_entry *rxe;
	struct rte_mbuf *rxm;
	struct rte_mbuf *nmb;
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
	uint16_t flit_size = 64; // 64B flit size
	uint16_t desc_size = 16; // 16B descriptor size
	uint16_t partial_payload_size = flit_size - desc_size; // 48B payload size
	uint8_t read_buffer[64]; // 64B buffer for reading from PCIe
	void *read_buffer_ptr = (void *) read_buffer; // Pointer to read buffer
	rxq = rx_queue;

	nb_rx = 0;
	nb_hold = 0;
	rx_id = rxq->rx_tail;
	sw_ring = rxq->sw_ring; // DPDK RX ring
	while (nb_rx < nb_pkts) {
		/*
         * Step 1: Read 64B from NIC device register to get both descriptor (16B) and packet data.
         */
		//Read 64B from NIC device register & Do not convert le_to_cpu at E1000_PCI_REG_READ64 -> because it needs to copy to descriptor, which is in little-endian format
		E1000_PCI_REG_READ64B(rxq->rx_m2func_reg_addr, read_buffer); // Read 64B from PCIe
        /*
         * Step 2: Extract the descriptor (16B) from the first part of the read buffer.
         */
        rte_memcpy(&rxd, read_buffer, sizeof(rxd));		

        staterr = rxd.wb.upper.status_error;
        if (!(staterr & rte_cpu_to_le_32(E1000_RXD_STAT_DD))) {
            break; // If DD bit is not set, break the loop.
        }

        /*
         * Step 3: Read pkt_len from the descriptor to know how much data to fetch.
         */
        pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.wb.upper.length) - rxq->crc_len);

        /*
         * Step 4: Allocate a new mbuf and ensure it succeeds.
         */
        nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
        if (nmb == NULL) {
            PMD_RX_LOG(WARNING, "RX mbuf alloc failed port_id=%u queue_id=%u",
                       (unsigned) rxq->port_id, (unsigned) rxq->queue_id);
            rte_eth_devices[rxq->port_id].data->rx_mbuf_alloc_failed++;
            break;
        }

        nb_hold++;
        rxe = &sw_ring[rx_id];
        rx_id++;
		if (rx_id == rxq->nb_rx_desc)
			rx_id = 0;

        /* Prefetch next mbuf while processing current one */
        rte_em_prefetch(sw_ring[rx_id].mbuf);

		/*
		 * When next RX descriptor is on a cache-line boundary,
		 * prefetch the next 4 RX descriptors and the next 8 pointers
		 * to mbufs.
		 */
		if ((rx_id & 0x3) == 0) {
			rte_em_prefetch(&sw_ring[rx_id]);
		}

        /* Step 5: Rearm RXD: attach new mbuf */
        rxm = rxe->mbuf;
        rxe->mbuf = nmb;        

        /*
         * Step 6: Copy the remaining 48B of the read buffer into the rxm as part of the payload.
         */
		rxm->data_off = RTE_PKTMBUF_HEADROOM;
		rte_packet_prefetch((char *)rxm->buf_addr + rxm->data_off);
        void *data_ptr = rte_pktmbuf_mtod(rxm, void *); // Get pointer to mbuf data    
		read_buffer_ptr = ((char *) read_buffer + desc_size); // Move pointer to 48B after descriptor
		// JM: maybe we don't have to convert le_to_cpu before copy to mbuf
		rte_memcpy(data_ptr, read_buffer_ptr, (size_t) partial_payload_size); // Copy 48B from read buffer to payload
        
		data_ptr = ((char *) data_ptr + partial_payload_size); // Move pointer to the end of the copied data

        /*
         * Step 7: If the packet length exceeds 48B, keep reading the remaining payload data.
         */
        uint16_t remaining_len = pkt_len - partial_payload_size;
        while (remaining_len > 0) {
            uint16_t chunk_size = (remaining_len > flit_size) ? flit_size : remaining_len;
			// JM: maybe we don't have to read 64B from NIC device register & Convert le_to_cpu before copy to mbuf
            if (chunk_size == flit_size) {
				E1000_PCI_REG_READ64B(rxq->rx_m2func_reg_addr, data_ptr); // Read 64B from PCIe
			} else {
				uint8_t temp_buffer[64]; // Temporary buffer to hold 64B
				E1000_PCI_REG_READ64B(rxq->rx_m2func_reg_addr, temp_buffer); // Read 64B from PCIe
				rte_memcpy(data_ptr, temp_buffer, (size_t) chunk_size); // Copy chunk_size from temp_buffer to mbuf
			}
            data_ptr = ((char *) data_ptr + chunk_size); // Move pointer to the end of the copied data
            remaining_len -= chunk_size;
        }

        /*
         * Step 8: Initialize the mbuf fields with descriptor values and packet info.
         */        
        rxm->nb_segs = 1;
        rxm->next = NULL;
        rxm->pkt_len = pkt_len;
        rxm->data_len = pkt_len;
        rxm->port = rxq->port_id;

        rxm->hash.rss = rxd.wb.lower.hi_dword.rss;
        uint32_t hlen_type_rss = rte_le_to_cpu_32(rxd.wb.lower.lo_dword.data);

        /*
         * Step 9: Set packet flags and handle VLAN tag.
         */
		if ((staterr & rte_cpu_to_le_32(E1000_RXDEXT_STATERR_LB)) &&
				(rxq->flags & 0x01)) {
			rxm->vlan_tci = rte_be_to_cpu_16(rxd.wb.upper.vlan);
		} else {
			rxm->vlan_tci = rte_le_to_cpu_16(rxd.wb.upper.vlan);
		}

        pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(rxq, hlen_type_rss);
		pkt_flags = pkt_flags | rx_desc_status_to_pkt_flags(staterr);
		pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(staterr);
		rxm->ol_flags = pkt_flags;
		rxm->packet_type = em_rxd_pkt_info_to_pkt_type(rxd.wb.lower.
						lo_dword.hs_rss.pkt_info);

        /* Store the mbuf address into the array of returned packets */
        rx_pkts[nb_rx++] = rxm;
	}
	rxq->rx_tail = rx_id;

	struct rte_ether_hdr *eth_hdr;
	struct rte_mbuf *mb;
	int i;
	for (i = 0; i < nb_rx; i++) {
		mb = rx_pkts[i];
		eth_hdr = rte_pktmbuf_mtod(mb, struct rte_ether_hdr *);
	}
	
	return nb_rx;
}

#define M2FUNC_DTA_RX_JOB_FLIT_METADATA_SIZE 16 // 16B job flit metadata size (8B completion address + 8B number of packets)
#define M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE ((M2FUNC_DTA_FLIT_SIZE - M2FUNC_DTA_RX_JOB_FLIT_METADATA_SIZE) / sizeof(uint64_t)) // 6 mbuf addresses in the first flit
#define M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE (M2FUNC_DTA_FLIT_SIZE / sizeof(uint64_t)) // 8 mbuf addresses in the remaining flits

uint16_t
eth_em_recv_pkts_m2func_dta(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts)
{
	// JM - SET THIS VALUE!
	bool use_pre_allocated_mbufs = false;
	struct em_rx_queue *rxq;
	rxq = rx_queue;
	struct rte_mbuf *nmb_list[nb_pkts]; // Pre-allocated mbuf list
	uint64_t mbuf_addrs[nb_pkts]; // Pre-allocated mbuf addresses
	uint16_t allocated = 0;
	uint16_t submitted = 0;
	uint64_t completion_addr = rxq->completion_addr; // Address of the completion descriptor
	int64_t completed_pkts = 0;

	volatile uint8_t *descriptor_start = (volatile uint8_t *)((volatile uint8_t *)(rxq->completion_buffer) + M2FUNC_DTA_CACHLINE_SIZE);

	union e1000_adv_rx_desc rxd;
	uint64_t dma_addr;
	uint32_t staterr; //jm
	uint32_t hlen_type_rss; //jm
	uint16_t pkt_len;
	uint16_t nb_rx;
	uint16_t nb_hold;
	uint8_t status;
	uint64_t pkt_flags; //jm
	
	uint16_t receive_empty_threshold = 100; // Receive empty threshold
	uint16_t receive_empty_counter = 0; // Receive empty counter

	nb_rx = 0;
	nb_hold = 0;

	if (!use_pre_allocated_mbufs) {
		// Step 1: Pre-allocate mbufs for the batch
		while (allocated < nb_pkts) {
			struct rte_mbuf *nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
			if (!nmb) {
				PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u queue_id=%u",
						(unsigned) rxq->port_id, (unsigned) rxq->queue_id);
				break;
			}
			nmb_list[allocated] = nmb;
			mbuf_addrs[allocated] = rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
			allocated++;
		}
	} else {
		// Use pre-allocated mbufs
		allocated = nb_pkts;
	}

	// Step 2: Submit job to DTA
	if (allocated > 0) {
		// Create a job submission packet
		uint16_t total_packets = allocated;
		uint16_t packet_index = 0;
		while (total_packets > 0) {
			uint16_t current_batch_size = 0;

			if (packet_index == 0) {
				current_batch_size = total_packets > M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE ? M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE : total_packets;
			} else {
				current_batch_size = total_packets > M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE ? M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE : total_packets;
			}
			
			uint64_t job_packet[M2FUNC_DTA_FLIT_ARRAY_SIZE] = {0}; // 64B job packet
			job_packet[0] = 0; // Initialize the header part
			
			// Packet format
			// 1. completion address (64-bit)
			// 2. number of packets (64-bit) 
			// -------16 Bytes-------
			// 3. mbuf addresses (64-bit * 6)

			// If packet_index is 0, then completion address and the number of packets are added
			if (packet_index == 0) {
				job_packet[0] = rte_cpu_to_le_64(completion_addr);
				job_packet[1] = rte_cpu_to_le_64((uint64_t) allocated);

				// Add mbuf addresses
				if (!use_pre_allocated_mbufs) {
					memcpy(&job_packet[2], &mbuf_addrs[0], current_batch_size * sizeof(uint64_t));
				} else {
					memcpy(&job_packet[2], &(rxq->mbuf_addr_array[0]), current_batch_size * sizeof(uint64_t));
				}
				// printf("DPDK[RX]: packet_index[%d]\n", packet_index);
				// for (int i = 0; i < M2FUNC_DTA_FLIT_ARRAY_SIZE; i++) {
				// 	printf("DPDK[RX]: Job packet[%d]: %ld\n", i, job_packet[i]);
				// }
			} else {
				// Add mbuf addresses
				if (!use_pre_allocated_mbufs) {
					memcpy(&job_packet[0], &mbuf_addrs[M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE + (packet_index - 1) * M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE], current_batch_size * sizeof(uint64_t));
				} else {
					memcpy(&job_packet[0], &(rxq->mbuf_addr_array[M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE + (packet_index - 1) * M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE]), current_batch_size * sizeof(uint64_t));
				}
				
				// printf("DPDK[RX]: packet_index[%d]\n", packet_index);
				// for (int i = 0; i < M2FUNC_DTA_FLIT_ARRAY_SIZE; i++) {
				// 	printf("DPDK[RX]: Job packet[%d]: %ld\n", i, job_packet[i]);
				// }
			}

			// Submit the job to DTA
			// printf("DPDK[RX]: Submitting job to DTA\n");
			// fflush(stdout);
			E1000_PCI_REG_WRITE64B(rxq->dta_job_submit_reg_addr, job_packet);
			

			total_packets -= current_batch_size;
			packet_index++;
		}

	} else {
		PMD_RX_LOG(DEBUG, "No mbufs allocated for the batch");
		printf("DPDK[RX]: No mbufs allocated for the batch\n");
		fflush(stdout);
		return 0;
	}

	// Step 3: Poll completion address for results
	rte_mb();
	completed_pkts = rxq->completion_buffer[0];
	// printf("DPDK[RX]: Completed packets: %ld\n", completed_pkts);
	// fflush(stdout);
	while (completed_pkts < (int64_t) allocated) {
		completed_pkts = rxq->completion_buffer[0];
		if (completed_pkts == -1) {
			// Initialize the completion buffer
			rxq->completion_buffer[0] = 0;
			if (!use_pre_allocated_mbufs) {
				// Free the allocated mbufs
				for (int i = 0; i < allocated; i++) {
					rte_pktmbuf_free(nmb_list[i]);
				}
			}
			PMD_RX_LOG(ERR, "RX Error in DTA processing");
			printf("DPDK[RX]: Completed packets: -1. So error in DTA processing\n");
			fflush(stdout);
			return 0;
		}

		if (completed_pkts == 0) {
			receive_empty_counter++;
			if (receive_empty_counter > receive_empty_threshold) {
				PMD_RX_LOG(DEBUG, "RX Empty threshold reached");
				printf("DPDK[RX]: RX Empty threshold reached. Return\n");
				fflush(stdout);
				break;
			}
		} else {
			receive_empty_counter = 0;
		}
	}

	rte_mb();

	// printf("DPDK[RX]: Completed packets: %ld\n", completed_pkts);
	// fflush(stdout);
	if (completed_pkts == 0) {
		// No packets received
		if (!use_pre_allocated_mbufs) {
			// Free the allocated mbufs
			for (int i = 0; i < allocated; i++) {
				rte_pktmbuf_free(nmb_list[i]);
			}
		}
		printf("DPDK[RX]: No packets received. So return\n");
		fflush(stdout);
		return 0;
	}

	// Initialize the completion buffer
	rxq->completion_buffer[0] = 0;

	// Step 4: Read the processed descriptors and copy the descriptors to mbufs
	for (int64_t i = 0; i < completed_pkts; i++) {
		struct rte_mbuf *nmb;
		if (!use_pre_allocated_mbufs) {
			nmb = nmb_list[i];
		} else {
			nmb = rxq->mbuf_array[i].mbuf;
		}

		// printf("DPDK[RX]: Try reading descriptor[%ld] at vaddr: %p\n", i, descriptor_start + i * sizeof(rxd));
		// fflush(stdout);
		// rte_mb();

		rte_memcpy(&rxd, descriptor_start + i * sizeof(rxd), sizeof(rxd));

		// Prefetch next mbuf while processing current one
		// if (i < completed_pkts - 1) {
		// 	rte_em_prefetch(nmb_list[i + 1]);
		// }

		// // When next RX descriptor is on a cache-line boundary, prefetch the next 4 RX descriptors.
		// if ((i & 0x3) == 0) {
		// 	rte_em_prefetch(descriptor_start + (i + 4) * sizeof(rxd));
		// }

		// TODO - JM : maybe we can consider offloading the below code to DTA. Give the nmb address to DTA
		nmb->data_off = RTE_PKTMBUF_HEADROOM;
		rte_packet_prefetch((char *)nmb->buf_addr + nmb->data_off);

		staterr = rxd.wb.upper.status_error;
		pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.wb.upper.length) - rxq->crc_len);
		nmb->nb_segs = 1;
		nmb->next = NULL;
		nmb->pkt_len = pkt_len;
		nmb->data_len = pkt_len;
		nmb->port = rxq->port_id;

		nmb->hash.rss = rxd.wb.lower.hi_dword.rss;
		hlen_type_rss = rte_le_to_cpu_32(rxd.wb.lower.lo_dword.data);

		if ((staterr & rte_cpu_to_le_32(E1000_RXDEXT_STATERR_LB)) &&
				(rxq->flags & 0x01)) {
			nmb->vlan_tci = rte_be_to_cpu_16(rxd.wb.upper.vlan);
		} else {
			nmb->vlan_tci = rte_le_to_cpu_16(rxd.wb.upper.vlan);
		}

		pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(rxq, hlen_type_rss);
		pkt_flags = pkt_flags | rx_desc_status_to_pkt_flags(staterr);
		pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(staterr);
		nmb->ol_flags = pkt_flags;
		nmb->packet_type = em_rxd_pkt_info_to_pkt_type(rxd.wb.lower.
						lo_dword.hs_rss.pkt_info);
		
		rx_pkts[nb_rx++] = nmb;
	}

	struct rte_ether_hdr *eth_hdr;
	struct rte_mbuf *mb;
	int i;
	for (i = 0; i < nb_rx; i++) {
		mb = rx_pkts[i];
		eth_hdr = rte_pktmbuf_mtod(mb, struct rte_ether_hdr *);
	}

	// printf("DPDK[RX]: nb_rx: %u, return with success\n", nb_rx);
	// fflush(stdout);
	return nb_rx;

	
}

uint16_t
eth_em_recv_pkts_m2func_dta_double_comp(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts)
{
	struct em_rx_queue *rxq;
	rxq = rx_queue;
	struct rte_mbuf *nmb_list[nb_pkts]; // Pre-allocated mbuf list
	uint64_t mbuf_addrs[nb_pkts]; // Pre-allocated mbuf addresses
	uint16_t allocated = 0;
	uint16_t allocated2 = 0;
	uint16_t submitted = 0;
	int64_t completed_pkts = 0;

	volatile uint8_t *descriptor_start = (volatile uint8_t *)((volatile uint8_t *)(rxq->completion_buffer) + M2FUNC_DTA_CACHLINE_SIZE);
	volatile uint8_t *descriptor_start2 = (volatile uint8_t *)((volatile uint8_t *)(rxq->completion_buffer2) + M2FUNC_DTA_CACHLINE_SIZE);

	union e1000_adv_rx_desc rxd;
	uint64_t dma_addr;
	uint32_t staterr; //jm
	uint32_t hlen_type_rss; //jm
	uint16_t pkt_len;
	uint16_t nb_rx;
	uint16_t nb_hold;
	uint8_t status;
	uint64_t pkt_flags; //jm
	
	uint16_t receive_empty_threshold = 100; // Receive empty threshold
	uint16_t receive_empty_counter = 0; // Receive empty counter

	nb_rx = 0;
	nb_hold = 0;

	if (!rxq->started) {
		// Step 1: Pre-allocate mbufs for the batch
		while (allocated < nb_pkts) {
			struct rte_mbuf *nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
			if (!nmb) {
				PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u queue_id=%u",
						(unsigned) rxq->port_id, (unsigned) rxq->queue_id);
				break;
			}
			nmb_list[allocated] = nmb;
			mbuf_addrs[allocated] = rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
			allocated++;
		}
		
		// Step 2: Submit job to DTA
		if (allocated > 0) {
			// Create a job submission packet
			uint16_t total_packets = allocated;
			uint16_t packet_index = 0;
			while (total_packets > 0) {
				uint16_t current_batch_size = 0;

				if (packet_index == 0) {
					current_batch_size = total_packets > M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE ? M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE : total_packets;
				} else {
					current_batch_size = total_packets > M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE ? M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE : total_packets;
				}
				
				// Packet format
				// 1. completion address (64-bit)
				// 2. number of packets (64-bit) 
				// -------16 Bytes-------
				// 3. mbuf addresses (64-bit * 6)

				// If packet_index is 0, then completion address and the number of packets are added
				if (packet_index == 0) {
					rxq->job_packet[1] = rte_cpu_to_le_64((uint64_t) allocated);

					// Add mbuf addresses
					memcpy(&(rxq->job_packet[2]), &mbuf_addrs[0], current_batch_size * sizeof(uint64_t));

					// Submit the job to DTA
					E1000_PCI_REG_WRITE64B(rxq->dta_job_submit_reg_addr, rxq->job_packet);
				} else {
					// Add mbuf addresses
					uint64_t job_packet[M2FUNC_DTA_FLIT_ARRAY_SIZE] = {0}; // 64B job packet
				
					memcpy(&job_packet[0], &mbuf_addrs[M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE + (packet_index - 1) * M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE], current_batch_size * sizeof(uint64_t));

					// Submit the job to DTA
					E1000_PCI_REG_WRITE64B(rxq->dta_job_submit_reg_addr, job_packet);
				}				

				total_packets -= current_batch_size;
				packet_index++;
			}

		} else {
			PMD_RX_LOG(DEBUG, "No mbufs allocated for the batch");
			printf("DPDK[RX]: Not started. No mbufs allocated for the batch\n");
			fflush(stdout);
			return 0;
		}

		// Step 3: Poll completion address for results
		rte_mb();
	
		completed_pkts = rxq->completion_buffer[0];
		// printf("DPDK[RX]: Completed packets: %ld\n", completed_pkts);
		// fflush(stdout);
		while (completed_pkts < (int64_t) allocated) {
			completed_pkts = rxq->completion_buffer[0];
			if (completed_pkts == -1) {
				// Initialize the completion buffer
				rxq->completion_buffer[0] = 0;
				
				// Free the allocated mbufs
				for (int i = 0; i < allocated; i++) {
					rte_pktmbuf_free(nmb_list[i]);
				}
				PMD_RX_LOG(ERR, "RX Error in DTA processing");
				printf("DPDK[RX]: Completed packets: -1. So error in DTA processing. RX not started & completion_buffer_index: %d\n", rxq->completion_buffer_index);
				fflush(stdout);
				return 0;
			}

			if (completed_pkts == 0) {
				receive_empty_counter++;
				if (receive_empty_counter > receive_empty_threshold) {
					PMD_RX_LOG(DEBUG, "RX Empty threshold reached");
					printf("DPDK[RX]: Not started, RX Empty threshold reached. Return\n");
					fflush(stdout);
					break;
				}
			} else {
				receive_empty_counter = 0;
				if ((!rxq->started) && (completed_pkts < (int64_t) allocated)) {
					rxq->started = true;
					
					// Step 1: Pre-allocate mbufs for the batch
					while (allocated2 < nb_pkts) {
						struct rte_mbuf *nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
						if (!nmb) {
							PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u queue_id=%u",
									(unsigned) rxq->port_id, (unsigned) rxq->queue_id);
							printf("DPDK[RX]: RX mbuf alloc failed. So return\n");
							return 0;
						}
						rxq->rx_bufs2[allocated2] = nmb;
						mbuf_addrs[allocated2] = rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
						allocated2++;
					}

					// Step 2: Submit job to DTA
					if (allocated2 > 0) {
						// Create a job submission packet
						uint16_t total_packets = allocated2;
						uint16_t packet_index = 0;
						while (total_packets > 0) {
							uint16_t current_batch_size = 0;

							if (packet_index == 0) {
								current_batch_size = total_packets > M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE ? M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE : total_packets;
							} else {
								current_batch_size = total_packets > M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE ? M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE : total_packets;
							}
														
							// Packet format
							// 1. completion address (64-bit)
							// 2. number of packets (64-bit) 
							// -------16 Bytes-------
							// 3. mbuf addresses (64-bit * 6)

							// If packet_index is 0, then completion address and the number of packets are added
							if (packet_index == 0) {
								rxq->job_packet2[1] = rte_cpu_to_le_64((uint64_t) allocated2);

								// Add mbuf addresses
								memcpy(&(rxq->job_packet2[2]), &mbuf_addrs[0], current_batch_size * sizeof(uint64_t));

								// Submit the job to DTA
								E1000_PCI_REG_WRITE64B(rxq->dta_job_submit_reg_addr, rxq->job_packet2);
							} else {
								uint64_t job_packet[M2FUNC_DTA_FLIT_ARRAY_SIZE] = {0}; // 64B job packet
								// Add mbuf addresses
								memcpy(&job_packet[0], &mbuf_addrs[M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE + (packet_index - 1) * M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE], current_batch_size * sizeof(uint64_t));
								
								// Submit the job to DTA
								E1000_PCI_REG_WRITE64B(rxq->dta_job_submit_reg_addr, job_packet);
							}

							total_packets -= current_batch_size;
							packet_index++;
						}

					} else {
						printf("DPDK[RX]: No mbufs allocated for the batch\n");
						fflush(stdout);
						return 0;
					}
				}

			}
		}

		if (completed_pkts == 0) {
			// No packets received
			// Free the allocated mbufs
			for (int i = 0; i < allocated; i++) {
				rte_pktmbuf_free(nmb_list[i]);
			}
			printf("DPDK[RX]: Not started, No packets received. So return\n");
			fflush(stdout);
			return 0;
		}

		rte_mb();

		// Initialize the completion buffer
		rxq->completion_buffer[0] = 0;

		// Step 4: Read the processed descriptors and copy the descriptors to mbufs
		for (int64_t i = 0; i < completed_pkts; i++) {
			struct rte_mbuf *nmb;
			nmb = nmb_list[i];
			

			// printf("DPDK[RX]: Try reading descriptor[%ld] at vaddr: %p\n", i, descriptor_start + i * sizeof(rxd));
			// fflush(stdout);
			// rte_mb();

			if (rxq->completion_buffer_index == 0) {
				rte_memcpy(&rxd, descriptor_start + i * sizeof(rxd), sizeof(rxd));
			} else {
				rte_memcpy(&rxd, descriptor_start2 + i * sizeof(rxd), sizeof(rxd));
			}

			// Prefetch next mbuf while processing current one
			// if (i < completed_pkts - 1) {
			// 	rte_em_prefetch(nmb_list[i + 1]);
			// }

			// // When next RX descriptor is on a cache-line boundary, prefetch the next 4 RX descriptors.
			// if ((i & 0x3) == 0) {
			// 	rte_em_prefetch(descriptor_start + (i + 4) * sizeof(rxd));
			// }

			// TODO - JM : maybe we can consider offloading the below code to DTA. Give the nmb address to DTA
			nmb->data_off = RTE_PKTMBUF_HEADROOM;
			rte_packet_prefetch((char *)nmb->buf_addr + nmb->data_off);

			staterr = rxd.wb.upper.status_error;
			pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.wb.upper.length) - rxq->crc_len);
			nmb->nb_segs = 1;
			nmb->next = NULL;
			nmb->pkt_len = pkt_len;
			nmb->data_len = pkt_len;
			nmb->port = rxq->port_id;

			nmb->hash.rss = rxd.wb.lower.hi_dword.rss;
			hlen_type_rss = rte_le_to_cpu_32(rxd.wb.lower.lo_dword.data);

			if ((staterr & rte_cpu_to_le_32(E1000_RXDEXT_STATERR_LB)) &&
					(rxq->flags & 0x01)) {
				nmb->vlan_tci = rte_be_to_cpu_16(rxd.wb.upper.vlan);
			} else {
				nmb->vlan_tci = rte_le_to_cpu_16(rxd.wb.upper.vlan);
			}

			pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(rxq, hlen_type_rss);
			pkt_flags = pkt_flags | rx_desc_status_to_pkt_flags(staterr);
			pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(staterr);
			nmb->ol_flags = pkt_flags;
			nmb->packet_type = em_rxd_pkt_info_to_pkt_type(rxd.wb.lower.
							lo_dword.hs_rss.pkt_info);
						
			rx_pkts[nb_rx++] = nmb;
		}
	} else {
		// RX is started - load_gen is started and we will poll the completion buffer corresponding to the completion_buffer_index
		
		// Step 1: Pre-allocate mbufs for the batch
		while (allocated < nb_pkts) {
			struct rte_mbuf *nmb = rte_mbuf_raw_alloc(rxq->mb_pool);
			if (!nmb) {
				PMD_RX_LOG(DEBUG, "RX mbuf alloc failed port_id=%u queue_id=%u",
						(unsigned) rxq->port_id, (unsigned) rxq->queue_id);
				printf("DPDK[RX]: RX mbuf alloc failed. So return\n");
				break;
			}
			if (rxq->completion_buffer_index == 0) {
				rxq->rx_bufs2[allocated] = nmb;
			} else {
				rxq->rx_bufs[allocated] = nmb;
			}
			mbuf_addrs[allocated] = rte_cpu_to_le_64(rte_mbuf_data_iova_default(nmb));
			allocated++;
		}
		fflush(stdout);

		// Step 2: Submit job to DTA
		if (allocated > 0) {
			// Create a job submission packet
			uint16_t total_packets = allocated;
			uint16_t packet_index = 0;
			while (total_packets > 0) {
				uint16_t current_batch_size = 0;

				if (packet_index == 0) {
					current_batch_size = total_packets > M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE ? M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE : total_packets;
				} else {
					current_batch_size = total_packets > M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE ? M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE : total_packets;
				}
				
				// Packet format
				// 1. completion address (64-bit)
				// 2. number of packets (64-bit) 
				// -------16 Bytes-------
				// 3. mbuf addresses (64-bit * 6)

				// If packet_index is 0, then completion address and the number of packets are added
				if (packet_index == 0) {
					// Add mbuf addresses
					if (rxq->completion_buffer_index == 0) {
						rxq->job_packet2[1] = rte_cpu_to_le_64((uint64_t) allocated);
						memcpy(&(rxq->job_packet2[2]), mbuf_addrs, current_batch_size * sizeof(uint64_t));

						// Submit the job to DTA
						E1000_PCI_REG_WRITE64B(rxq->dta_job_submit_reg_addr, rxq->job_packet2);
					} else {
						rxq->job_packet[1] = rte_cpu_to_le_64((uint64_t) allocated);
						memcpy(&(rxq->job_packet[2]), mbuf_addrs, current_batch_size * sizeof(uint64_t));

						// Submit the job to DTA
						E1000_PCI_REG_WRITE64B(rxq->dta_job_submit_reg_addr, rxq->job_packet);
					}
					
				} else {
					uint64_t job_packet[M2FUNC_DTA_FLIT_ARRAY_SIZE] = {0}; // 64B job packet
					// Add mbuf addresses
					memcpy(&job_packet[0], &(mbuf_addrs[M2FUNC_DTA_RX_FIRST_FLIT_BATCH_SIZE + (packet_index - 1) * M2FUNC_DTA_RX_REST_FLIT_BATCH_SIZE]), current_batch_size * sizeof(uint64_t));
					
					// Submit the job to DTA
					E1000_PCI_REG_WRITE64B(rxq->dta_job_submit_reg_addr, job_packet);
				}

				total_packets -= current_batch_size;
				packet_index++;
			}

		} else {
			printf("DPDK[RX]: No mbufs allocated for the batch.\n");
			fflush(stdout);
			return 0;
		}

		// Step 3: Poll completion address for results for the previous completion buffer
		rte_mb();
		completed_pkts = (rxq->completion_buffer_index == 0) ? rxq->completion_buffer[0] : rxq->completion_buffer2[0];
		while (completed_pkts < (int64_t) allocated) {
			completed_pkts = (rxq->completion_buffer_index == 0) ? rxq->completion_buffer[0] : rxq->completion_buffer2[0];
			if (completed_pkts == -1) {
				// Initialize the completion buffer
				if (rxq->completion_buffer_index == 0) {
					rxq->completion_buffer[0] = 0;
				} else {
					rxq->completion_buffer2[0] = 0;
				}
				
				// Free the allocated mbufs
				for (int i = 0; i < allocated; i++) {
					if (rxq->completion_buffer_index == 0) {
						rte_pktmbuf_free(rxq->rx_bufs[i]);
					} else {
						rte_pktmbuf_free(rxq->rx_bufs2[i]);
					}
				}

				// Change the completion buffer index
				rxq->completion_buffer_index = (rxq->completion_buffer_index == 0) ? 1 : 0;
				
				PMD_RX_LOG(ERR, "RX Error in DTA processing");
				printf("DPDK[RX]: Completed packets: -1. So error in DTA processing. Changed Completion_buffer_index: %d\n", rxq->completion_buffer_index);
				fflush(stdout);
				return 0;
			}

			if (completed_pkts == 0) {
				receive_empty_counter++;
				if (receive_empty_counter > receive_empty_threshold) {
					PMD_RX_LOG(DEBUG, "RX Empty threshold reached");
					printf("DPDK[RX]: RX Empty threshold reached. Return\n");
					fflush(stdout);
					break;
				}
			} else {
				receive_empty_counter = 0;
			}
		}

		if (completed_pkts == 0) {
			// No packets received
			// Free the allocated mbufs
			for (int i = 0; i < allocated; i++) {
				if (rxq->completion_buffer_index == 0) {
					rte_pktmbuf_free(rxq->rx_bufs[i]);
				} else {
					rte_pktmbuf_free(rxq->rx_bufs2[i]);
				}
			}

			// Change the completion buffer index
			rxq->completion_buffer_index = (rxq->completion_buffer_index == 0) ? 1 : 0;

			printf("DPDK[RX]: No packets received. So return. Changed Completion_buffer_index: %ld\n", rxq->completion_buffer_index);
			fflush(stdout);
			return 0;
		}

		rte_mb();

		// Initialize the completion buffer
		if (rxq->completion_buffer_index == 0) {
			rxq->completion_buffer[0] = 0;
		} else {
			rxq->completion_buffer2[0] = 0;
		}

		for (int64_t i = 0; i < completed_pkts; i++) {
			struct rte_mbuf *nmb;
			if (rxq->completion_buffer_index == 0) {
				nmb = rxq->rx_bufs[i];
			} else {
				nmb = rxq->rx_bufs2[i];
			}			

			// printf("DPDK[RX]: Try reading descriptor[%ld] at vaddr: %p\n", i, descriptor_start + i * sizeof(rxd));
			// fflush(stdout);
			// rte_mb();

			if (rxq->completion_buffer_index == 0) {
				rte_memcpy(&rxd, descriptor_start + i * sizeof(rxd), sizeof(rxd));
			} else {
				rte_memcpy(&rxd, descriptor_start2 + i * sizeof(rxd), sizeof(rxd));
			}

			// Prefetch next mbuf while processing current one
			// if (i < completed_pkts - 1) {
			// 	rte_em_prefetch(nmb_list[i + 1]);
			// }

			// // When next RX descriptor is on a cache-line boundary, prefetch the next 4 RX descriptors.
			// if ((i & 0x3) == 0) {
			// 	rte_em_prefetch(descriptor_start + (i + 4) * sizeof(rxd));
			// }

			// TODO - JM : maybe we can consider offloading the below code to DTA. Give the nmb address to DTA
			nmb->data_off = RTE_PKTMBUF_HEADROOM;
			rte_packet_prefetch((char *)nmb->buf_addr + nmb->data_off);

			staterr = rxd.wb.upper.status_error;
			pkt_len = (uint16_t) (rte_le_to_cpu_16(rxd.wb.upper.length) - rxq->crc_len);
			nmb->nb_segs = 1;
			nmb->next = NULL;
			nmb->pkt_len = pkt_len;
			nmb->data_len = pkt_len;
			nmb->port = rxq->port_id;

			nmb->hash.rss = rxd.wb.lower.hi_dword.rss;
			hlen_type_rss = rte_le_to_cpu_32(rxd.wb.lower.lo_dword.data);

			if ((staterr & rte_cpu_to_le_32(E1000_RXDEXT_STATERR_LB)) &&
					(rxq->flags & 0x01)) {
				nmb->vlan_tci = rte_be_to_cpu_16(rxd.wb.upper.vlan);
			} else {
				nmb->vlan_tci = rte_le_to_cpu_16(rxd.wb.upper.vlan);
			}

			pkt_flags = rx_desc_hlen_type_rss_to_pkt_flags(rxq, hlen_type_rss);
			pkt_flags = pkt_flags | rx_desc_status_to_pkt_flags(staterr);
			pkt_flags = pkt_flags | rx_desc_error_to_pkt_flags(staterr);
			nmb->ol_flags = pkt_flags;
			nmb->packet_type = em_rxd_pkt_info_to_pkt_type(rxd.wb.lower.
							lo_dword.hs_rss.pkt_info);
			
			rx_pkts[nb_rx++] = nmb;
		}

	}

	// Set the completion buffer index
	rxq->completion_buffer_index = (rxq->completion_buffer_index == 0) ? 1 : 0;

	// struct rte_ether_hdr *eth_hdr;
	// struct rte_mbuf *mb;
	// int i;
	// for (i = 0; i < nb_rx; i++) {
	// 	mb = rx_pkts[i];
	// 	eth_hdr = rte_pktmbuf_mtod(mb, struct rte_ether_hdr *);
	// }

	return nb_rx;

	
}


uint16_t
eth_em_recv_pkts_m2func_poll_test(void *rx_queue, struct rte_mbuf **rx_pkts,
		uint16_t nb_pkts)
{
	// For testing the polling mechanism to the completion buffer
	// Just check the completion buffer and return the number of packets
	// Also, use the counter to check the number of times the polling is done
	struct em_rx_queue *rxq;
	rxq = rx_queue;
	uint64_t completion_addr = rxq->completion_addr; // Address of the completion descriptor
	uint16_t receive_empty_threshold = 100; // Receive empty threshold
	uint16_t receive_empty_counter = 0; // Receive empty counter

	int64_t completed_pkts = 0;

	printf("DPDK[RX-TEST]: Now Begin Polling rxq->completion_buffer[0]. completion vaddr: %p, paddr: %p\n", (void *)(rxq->completion_buffer), (void *)rxq->completion_addr);
	fflush(stdout);
	rte_mb();
	completed_pkts = rxq->completion_buffer[0];
	rte_mb();
	printf("DPDK[RX-TEST]: Completed packets: %ld\n", completed_pkts);
	fflush(stdout);
	rte_mb();
	while (completed_pkts < (int64_t) nb_pkts) {
		rte_mb();
		completed_pkts = rxq->completion_buffer[0];
		rte_mb();
		if (completed_pkts == -1) {
			PMD_RX_LOG(ERR, "RX Error in DTA processing");
			return 0;
		}

		if (completed_pkts == 0) {
			receive_empty_counter++;
			if (receive_empty_counter % 10 == 0) {
				printf("DPDK[RX-TEST]: Receive empty counter: %d\n", receive_empty_counter);
				fflush(stdout);
			}
			if (receive_empty_counter > receive_empty_threshold) {
				PMD_RX_LOG(DEBUG, "RX Empty threshold reached");
				break;
			}
		} else {
			receive_empty_counter = 0;
		}
	}

	printf("DPDK[RX-TEST]: Completed packets: %ld We will Return 0 for test purpose\n", completed_pkts);
	fflush(stdout);

	return 0;
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
		#ifndef EM_SVE_512
		txq->sw_ring[i].last_id = i;
		txq->sw_ring[prev].next_id = i;
		#endif
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

	txq->tx_next_dd = (uint16_t)(txq->tx_rs_thresh - 1);
	txq->tx_next_rs = (uint16_t)(txq->tx_rs_thresh - 1);

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
	txq->tx_m2func_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_TXM2FUNC(queue_idx));
	txq->dta_job_submit_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_DTA_TX_JOB_SUBMIT(queue_idx));
	printf("======== txq[%d]->tdt_reg_addr: 0x%lx ========\n", queue_idx, txq->tdt_reg_addr);
	printf("======== txq[%d]->tx_m2func_reg_addr: 0x%lx ========\n", queue_idx, txq->tx_m2func_reg_addr);
	printf("======== txq[%d]->dta_job_submit_reg_addr: 0x%lx ========\n", queue_idx, txq->dta_job_submit_reg_addr);
	printf("======== txq[%d]->sw_ring phys addr: 0x%lx ========\n", queue_idx, rte_malloc_virt2iova(txq->sw_ring));
	printf("======== em_tx_entry size: %ld ========\n", sizeof(struct em_tx_entry));
	fflush(stdout);
	txq->tx_ring_phys_addr = tz->iova;
	// txq->tx_ring = (struct e1000_data_desc *) tz->addr;
	txq->tx_ring = (union e1000_adv_tx_desc *) tz->addr; //jm

	// For DTA
	if ((txq->completion_buffer = rte_zmalloc("txq->completion_buffer",
			RTE_CACHE_LINE_SIZE,
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_tx_queue_release(txq);
		return -ENOMEM;
	}
	txq->completion_addr = rte_malloc_virt2iova(txq->completion_buffer);

	if ((txq->descriptor_buffer = rte_zmalloc("txq->descriptor_buffer",
			sizeof(struct e1000_adv_tx_desc_m2func) * (nb_desc / 8),
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_tx_queue_release(txq);
		return -ENOMEM;
	}
	txq->descriptor_addr = rte_malloc_virt2iova(txq->descriptor_buffer);

	// For DTA double buffer
	if ((txq->completion_buffer2 = rte_zmalloc("txq->completion_buffer2",
			RTE_CACHE_LINE_SIZE,
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_tx_queue_release(txq);
		return -ENOMEM;
	}
	txq->completion_addr2 = rte_malloc_virt2iova(txq->completion_buffer2);

	if ((txq->descriptor_buffer2 = rte_zmalloc("txq->descriptor_buffer2",
			sizeof(struct e1000_adv_tx_desc_m2func) * (nb_desc / 8),
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_tx_queue_release(txq);
		return -ENOMEM;
	}
	txq->descriptor_addr2 = rte_malloc_virt2iova(txq->descriptor_buffer2);

	if ((txq->tx_bufs = rte_zmalloc("txq->tx_bufs",
			sizeof(txq->tx_bufs[0]) * nb_desc,
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_tx_queue_release(txq);
		printf("Failed to allocate txq->tx_bufs\n");
		return -ENOMEM;
	}
	if ((txq->tx_bufs2 = rte_zmalloc("txq->tx_bufs2",
			sizeof(txq->tx_bufs2[0]) * nb_desc,
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_tx_queue_release(txq);
		printf("Failed to allocate txq->tx_bufs2\n");
		return -ENOMEM;
	}

	// Initialize uint64_t array with 0 (zero_copy_job_packet)
	for (int i = 0; i < M2FUNC_DTA_FLIT_ARRAY_SIZE; i++) {
		txq->zero_copy_job_packet[i] = 0;
	}

	// Set the first entry to descriptor_addr and second entry to completion_addr
	txq->zero_copy_job_packet[0] = rte_cpu_to_le_64(txq->descriptor_addr);
	txq->zero_copy_job_packet[1] = rte_cpu_to_le_64(txq->completion_addr);

	// Initialize uint64_t array with 0 (zero_copy_job_packet2)
	for (int i = 0; i < M2FUNC_DTA_FLIT_ARRAY_SIZE; i++) {
		txq->zero_copy_job_packet2[i] = 0;
	}

	// Set the first entry to descriptor_addr and second entry to completion_addr
	txq->zero_copy_job_packet2[0] = rte_cpu_to_le_64(txq->descriptor_addr2);
	txq->zero_copy_job_packet2[1] = rte_cpu_to_le_64(txq->completion_addr2);

	// Initialize the completion index & started flag
	txq->started = false;
	txq->completion_buffer_index = 0;

	printf("======== txq[%d]->completion_addr: 0x%lx ========\n", queue_idx, txq->completion_addr);
	printf("======== txq[%d]->descriptor_addr: 0x%lx ========\n", queue_idx, txq->descriptor_addr);
	printf("======== txq[%d]->completion_addr2: 0x%lx ========\n", queue_idx, txq->completion_addr2);
	printf("======== txq[%d]->descriptor_addr2: 0x%lx ========\n", queue_idx, txq->descriptor_addr2);
	printf("======== txq[%d]->tx_bufs phys addr: 0x%lx ========\n", queue_idx, rte_malloc_virt2iova(txq->tx_bufs));
	printf("======== txq[%d]->tx_bufs2 phys addr: 0x%lx ========\n", queue_idx, rte_malloc_virt2iova(txq->tx_bufs2));
	printf("======== txq[%d]->started: %d ========\n", queue_idx, txq->started);
	printf("======== txq[%d]->completion_buffer_index: %d ========\n", queue_idx, txq->completion_buffer_index);
	fflush(stdout);

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

	memset(&rxq->fake_mbuf, 0x0, sizeof(rxq->fake_mbuf));

	rxq->rx_tail = 0;
	rxq->nb_rx_hold = 0;
	rxq->pkt_first_seg = NULL;
	rxq->pkt_last_seg = NULL;

	rxq->rxrearm_start = 0;
	rxq->rxrearm_nb = 0;

	memset(rxq->offset_table, 0, sizeof(rxq->offset_table));

	// Make mbuf_initializer
	uintptr_t p;
	struct rte_mbuf mb_def = { .buf_addr = 0 }; /* zeroed mbuf */

	mb_def.nb_segs = 1;
	mb_def.data_off = RTE_PKTMBUF_HEADROOM;
	mb_def.port = rxq->port_id;
	rte_mbuf_refcnt_set(&mb_def, 1);

	/* prevent compiler reordering: rearm_data covers previous fields */
	rte_compiler_barrier();
	p = (uintptr_t)&mb_def.rearm_data;
	rxq->mbuf_initializer = *(uint64_t *)p;
	printf("======== rxq->mbuf_initializer: 0x%lx ========\n", rxq->mbuf_initializer);
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
	
	printf("======== rxq[%d]->crc_len: %d ========\n", queue_idx, rxq->crc_len);
	fflush(stdout);
		
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
	rxq->rx_m2func_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_RXM2FUNC(queue_idx));
	rxq->dta_job_submit_reg_addr = E1000_PCI_REG_ADDR(hw, E1000_DTA_RX_JOB_SUBMIT(queue_idx));
	printf("======== rxq[%d]->rdt_reg_addr: 0x%lx ========\n", queue_idx, rxq->rdt_reg_addr);
	printf("======== rxq[%d]->rdh_reg_addr: 0x%lx ========\n", queue_idx, rxq->rdh_reg_addr);
	printf("======== rxq[%d]->rx_m2func_reg_addr: 0x%lx ========\n", queue_idx, rxq->rx_m2func_reg_addr);
	printf("======== rxq[%d]->dta_job_submit_reg_addr: 0x%lx ========\n", queue_idx, rxq->dta_job_submit_reg_addr);
	printf("======== rxq[%d]->sw_ring phys addr: 0x%lx ========\n", queue_idx, rte_malloc_virt2iova(rxq->sw_ring));
	printf("======== em_rx_entry size: %ld ========\n", sizeof(struct em_rx_entry));
	fflush(stdout);
	rxq->rx_ring_phys_addr = rz->iova;
	// rxq->rx_ring = (struct e1000_rx_desc *) rz->addr;
	rxq->rx_ring = (union e1000_adv_rx_desc *) rz->addr;

	// For DTA, we have to malloc completion buffer - TODO: maybe we can consider making double buffer to avoid the cache coherency overhead
	if ((rxq->completion_buffer = rte_zmalloc("rxq->completion_buffer",
			sizeof (rxq->completion_buffer[0]) * (nb_desc/8),
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_rx_queue_release(rxq);
		return -ENOMEM;
	}
	rxq->completion_addr = rte_malloc_virt2iova(rxq->completion_buffer);

	printf("======== rxq[%d]->completion_addr: 0x%lx ========\n", queue_idx, rxq->completion_addr);
	fflush(stdout);

	// For DTA, we have to malloc 2nd completion buffer
	if ((rxq->completion_buffer2 = rte_zmalloc("rxq->completion_buffer2",
			sizeof (rxq->completion_buffer2[0]) * (nb_desc/8),
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_rx_queue_release(rxq);
		return -ENOMEM;
	}
	rxq->completion_addr2 = rte_malloc_virt2iova(rxq->completion_buffer2);

	printf("======== rxq[%d]->completion_addr2: 0x%lx ========\n", queue_idx, rxq->completion_addr2);
	fflush(stdout);

	rxq->started = false;
	rxq->completion_buffer_index = 0; // Firstly, use the first completion buffer

	// For DTA double buffer
	if ((rxq->rx_bufs = rte_zmalloc("rxq->rx_bufs",
			sizeof(rxq->rx_bufs[0]) * nb_desc,
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_rx_queue_release(rxq);
		printf("Failed to allocate rxq->rx_bufs\n");
		return -ENOMEM;
	}
	if ((rxq->rx_bufs2 = rte_zmalloc("rxq->rx_bufs2",
			sizeof(rxq->rx_bufs2[0]) * nb_desc,
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_rx_queue_release(rxq);
		printf("Failed to allocate rxq->rx_bufs2\n");
		return -ENOMEM;
	}

	printf("======== rxq[%d]->rx_bufs phys addr: 0x%lx ========\n", queue_idx, rte_malloc_virt2iova(rxq->rx_bufs));
	printf("======== rxq[%d]->rx_bufs2 phys addr: 0x%lx ========\n", queue_idx, rte_malloc_virt2iova(rxq->rx_bufs2));

	// For DTA, job packet
	// Initialize uint64_t array with 0 (job_packet)
	for (int i = 0; i < M2FUNC_DTA_FLIT_ARRAY_SIZE; i++) {
		rxq->job_packet[i] = 0;
		rxq->job_packet2[i] = 0;
	}
	// Set the completion address to the first entry
	rxq->job_packet[0] = rte_cpu_to_le_64(rxq->completion_addr);
	rxq->job_packet2[0] = rte_cpu_to_le_64(rxq->completion_addr2);

	// For testing, we will pre-allocate the mbufs and will use them in the receive function
	// Allocate mbuf_array
	if ((rxq->mbuf_array = rte_zmalloc("rxq->mbuf_array",
			sizeof (rxq->mbuf_array[0]) * 1024,
			RTE_CACHE_LINE_SIZE)) == NULL) {
		em_rx_queue_release(rxq);
		return -ENOMEM;
	}
	// Allocate mbufs and set mbuf_addr_array
	for (int i = 0; i < 1024; i++) {
		struct rte_mbuf *mbuf = rte_mbuf_raw_alloc(rxq->mb_pool);
		if (mbuf == NULL) {
			em_rx_queue_release(rxq);
			return -ENOMEM;
		}
		rxq->mbuf_array[i].mbuf = mbuf;
		rxq->mbuf_addr_array[i] = rte_cpu_to_le_64(rte_mbuf_data_iova_default(mbuf));
		if (i < 32) {
			printf("======== rxq[%d]->mbuf_addr_array[%d]: 0x%lx priv_size: %u ========\n", queue_idx, i, rxq->mbuf_addr_array[i], mbuf->priv_size);			
			fflush(stdout);
		}
	}
	
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

	// while ((desc < rxq->nb_rx_desc) &&
	// 	// (rxdp->status & E1000_RXD_STAT_DD)) {
	// 	(rxdp->wb.upper.status_error & E1000_RXD_STAT_DD)) {
	// 	desc += EM_RXQ_SCAN_INTERVAL;
	// 	rxdp += EM_RXQ_SCAN_INTERVAL;
	// 	if (rxq->rx_tail + desc >= rxq->nb_rx_desc)
	// 		rxdp = &(rxq->rx_ring[rxq->rx_tail +
	// 			desc - rxq->nb_rx_desc]);
	// }

	// JM - If reached here, we have to implement the same logic as above
	// Just print the status for now
	PMD_RX_LOG(WARNING, "eth_em_rx_queue_count is called!!!!!! port_id=%u queue_id=%u rx_tail=%u ",
		   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
		   (unsigned) rxq->rx_tail);

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
	PMD_RX_LOG(WARNING, "eth_em_rx_descriptor_done is called!!!!!! port_id=%u queue_id=%u rx_tail=%u ",
		   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
		   (unsigned) rxq->rx_tail);

	return !!(rxdp->wb.upper.status_error & E1000_RXD_STAT_DD);
}

int
eth_em_rx_descriptor_status(void *rx_queue, uint16_t offset)
{
	struct em_rx_queue *rxq = rx_queue;
	volatile uint8_t *status;
	uint32_t desc;

	PMD_RX_LOG(WARNING, "eth_em_rx_descriptor_status is called!!!!!! port_id=%u queue_id=%u rx_tail=%u ",
		   (unsigned) rxq->port_id, (unsigned) rxq->queue_id,
		   (unsigned) rxq->rx_tail);

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

	PMD_TX_LOG(WARNING, "eth_em_tx_descriptor_status is called!!!!!! port_id=%u queue_id=%u tx_tail=%u ",
		   (unsigned) txq->port_id, (unsigned) txq->queue_id,
		   (unsigned) txq->tx_tail);

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
		struct em_rx_queue *rxq = dev->data->rx_queues[i];
		rte_free(rxq->completion_buffer);
		eth_em_rx_queue_release(dev->data->rx_queues[i]);
		dev->data->rx_queues[i] = NULL;
		rte_eth_dma_zone_free(dev, "rx_ring", i);
	}
	dev->data->nb_rx_queues = 0;

	for (i = 0; i < dev->data->nb_tx_queues; i++) {
		struct em_tx_queue *txq = dev->data->tx_queues[i];
		rte_free(txq->completion_buffer);
		rte_free(txq->descriptor_buffer);
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
		printf("=======DPDK: RSS offload disabled\n");
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
				printf("========DPDK - RSS CONFIGURED========\n");
				break;
			case ETH_MQ_RX_VMDQ_ONLY:
				// not supported yet
				PMD_INIT_LOG(ERR, "EM MQ_RX_VMDQ_ONLY not supported");
				break;
			case ETH_MQ_RX_NONE:
				/* if mq_mode is none, disable rss mode.*/
			default:
				em_rss_disable(dev);
				printf("========DPDK - RSS DISABLED========\n");
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
	uint32_t srrctl;
	uint32_t rxcsum;
	uint32_t rctl_bsize;
	uint16_t i;
	int ret;

	hw = E1000_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	rxmode = &dev->data->dev_conf.rxmode;
	srrctl = 0;

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

	// Ring buffer
	#ifdef EM_SVE_512
	dev->rx_pkt_burst = (eth_rx_burst_t)eth_em_recv_pkts_sve512;
	#else
	dev->rx_pkt_burst = (eth_rx_burst_t)eth_em_recv_pkts;
	#endif
	// JM - CHANGE
	// dev->rx_pkt_burst = (eth_rx_burst_t)eth_em_recv_pkts_m2func;
	// JM - CHANGE DTA + M2func
	// dev->rx_pkt_burst = (eth_rx_burst_t)eth_em_recv_pkts_m2func_dta;
	// JM - CHANGE M2func + DTA (double comp)
	// dev->rx_pkt_burst = (eth_rx_burst_t)eth_em_recv_pkts_m2func_dta_double_comp;

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
		
		printf("========DPDK - rxq[%d]->crc_len = %d========\n", i, rxq->crc_len);

		bus_addr = rxq->rx_ring_phys_addr;
		E1000_WRITE_REG(hw, E1000_RDLEN(i),
				rxq->nb_rx_desc *
				sizeof(*rxq->rx_ring));
		E1000_WRITE_REG(hw, E1000_RDBAH(i),
				(uint32_t)(bus_addr >> 32));
		E1000_WRITE_REG(hw, E1000_RDBAL(i), (uint32_t)bus_addr);
		printf("========DPDK - WRITE RDBAH[%d] = 0x%x========\n", i, (uint32_t)(bus_addr >> 32));
		printf("========DPDK - WRITE RDBAL[%d] = 0x%x========\n", i, (uint32_t)bus_addr);
		
		srrctl = E1000_SRRCTL_DESCTYPE_ADV_ONEBUF; //set to use ADV_ONEBUF
		E1000_WRITE_REG(hw, E1000_SRRCTL(i), srrctl);
		printf("========DPDK - WRITE SRRCTL[%d] = ADV_ONEBUF========\n", i);

		// rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(0));
		rxdctl = E1000_READ_REG(hw, E1000_RXDCTL(i));
		rxdctl |= E1000_RXDCTL_QUEUE_ENABLE; // This is not needed, because now gem5 doesn't use this value
		rxdctl &= 0xFE000000; // because gem5's NIC threshold is 6 bits
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
	// rctl &= ~E1000_RCTL_DTYP_MASK;

	/*
	 * Configure support of jumbo frames, if any.
	 */
	if (rxmode->offloads & DEV_RX_OFFLOAD_JUMBO_FRAME)
		rctl |= E1000_RCTL_LPE;
	else
		rctl &= ~E1000_RCTL_LPE;

	/* Enable Receives. */
	E1000_WRITE_REG(hw, E1000_RCTL, rctl);

	/*
	 * Setup the HW Rx Head and Tail Descriptor Pointers.
	 * This needs to be done after enable.
	 */
	for (i = 0; i < dev->data->nb_rx_queues; i++) {
		E1000_WRITE_REG(hw, E1000_RDH(i), 0);
		E1000_WRITE_REG(hw, E1000_RDT(i), rxq->nb_rx_desc - 1);
		printf("========DPDK - WRITE RDH[%d] = 0========\n", i);
		printf("========DPDK - WRITE RDT[%d] = %d========\n", i, rxq->nb_rx_desc - 1);
	}

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
