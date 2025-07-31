#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <assert.h>
#include <signal.h>


#include <rte_launch.h>
#include <rte_eal.h>
#include <rte_ethdev.h>
#include <rte_lcore.h>
#include <rte_byteorder.h>
#include <rte_log.h>
#include <rte_malloc.h>
#include <rte_debug.h>

#include <rte_ether.h>
#include <rte_ip.h>


struct RXTXState 
{
    //EndpointId eid;
    //enso::TxPipe* tx_pipe;

    struct PendingTX {

    uint8_t* start_tx_buffer;
    uint8_t* current_tx_buffer;

    uint16_t count;
    uint64_t oldest_time;
    } pending_tx;
};

uint16_t get_pkt_len(const uint8_t* addr) {
    const struct rte_ether_hdr* l2_hdr = (struct rte_ether_hdr*)addr;
    const struct rte_ipv4_hdr* l3_hdr = (struct rte_ipv4_hdr*)(l2_hdr + 1);
    const uint16_t total_len = be_to_le_16(l3_hdr->total_length + sizeof(struct rte_ether_hdr));
    
    return total_len;
}

uint8_t* getNextPkt(uint8_t* pkt)
{
    uint16_t pkt_len = get_pkt_len(pkt);
    uint16_t nb_flits = (pkt_len - 1) / 64 + 1;
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
        /*
        struct rte_ether_hdr* l2_hdr = (struct rte_ether_hdr*)rxTxState->pending_tx.current_tx_buffer;
        struct rte_ether_addr original_src_mac = l2_hdr->s_addr;
        l2_hdr->s_addr = l2_hdr->d_addr;
        l2_hdr->d_addr = original_src_mac;
        */
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

// for maintain current enso pipe state used by host
typedef struct MicaProcessingUnit
{
    uint32_t avail_bytes;
    uint8_t* rx_buf; // Enso Pipe Start addr for packet processing
    uint8_t* end_of_buffer;

    // maintain state
    int32_t missing_messages; // = burstSize; will be decline
    uint32_t remaining_bytes; // = availByte;, will be decline
    uint32_t received; // count for received packet
    uint32_t end; // flag for current buffer is consumed all

    uint8_t* addr;
    uint8_t* next_addr;

}MicaProcessingUnit_t;

void
setProcessingUnit(RxEnsoPipe_t* rx_pipe, MicaProcessingUnit_t* mica_unit, uint32_t new_rx_bytes, uint8_t* new_rx_buf, int32_t burst_size)
{
    mica_unit->rx_buf = new_rx_buf;
    mica_unit->avail_bytes = new_rx_bytes;
    mica_unit->end_of_buffer = (uint8_t*)rx_pipe->buf + ENSO_BUF_SIZE;

    mica_unit->missing_messages = burst_size;
    mica_unit->remaining_bytes = new_rx_bytes;
    mica_unit->received = 0;
    mica_unit->end = burst_size;

    mica_unit->addr = new_rx_buf;
    mica_unit->next_addr = new_rx_buf;//getNextPkt(mica_unit->addr);

}

struct mehcached_batch_packet* 
getPacketFromRxPipe(MicaProcessingUnit_t* mica_unit)
{
    if((mica_unit->missing_messages > 0) && (mica_unit->remaining_bytes > 0))
    {
        mica_unit->addr = mica_unit->next_addr;
        if(mica_unit->addr >= mica_unit->end_of_buffer)
        {
            mica_unit->end = mica_unit->received;
            return NULL;
        }
        mica_unit->next_addr = getNextPkt(mica_unit->next_addr);
        unit32_t consumed = mica_unit->next_addr - mica_unit->addr;

        mica_unit->remaining_bytes -= consumed;
        --mica_unit->missing_messages;
        ++mica_unit->received;

        return (struct mehcached_batch_packet* )mica_unit->addr;
    }
    else
    {
        mica_unit->end = mica_unit->received;
        return NULL;
    }
}

void
setPacketToTxPipe(struct RXTXState* rxTxState)
{
    struct mehcached_batch_packet *packet = (struct mehcached_batch_packet *)rxTxState->pending_tx.current_tx_buffer;
    
}

while(!exit)
{
    size_t stage0_index = 0;
    size_t stage1_index = 0;
    size_t stage2_index = 0;
    size_t stage3_index = 0;

    while (stage3_index < mica_unit.end)
    {
        if (stage0_index < mica_unit.end && stage0_index - stage3_index < max_pending_packets && stage0_index - stage1_index < stage_gap)
        {
            packets[stage0_index] = getPacketFromRxPipe(&mica_unit);
            //rte_pktmbuf_mtod(mbuf, struct mehcached_batch_packet *);
            if (packets[stage0_index] != NULL)
            {
                printf("  [Stage0] limit pkt %u , prefetch %u pkt\n", mica_unit.end, stage0_index);
                stage0_index++;
            }
            else
            {
                printf("  [Stage0] NULL packet returned! limit %u , current %u\n", mica_unit.end, stage0_index);
                // current stage0 ptr to NULL so, -1
            }   
        }
        else if (stage1_index < stage0_index && stage1_index - stage2_index < stage_gap)
        {
            stage1_index++;
        }
        else if (stage2_index < stage1_index && stage2_index - stage3_index < stage_gap)
        {
            stage2_index++;
        }
        else if (stage3_index < stage2_index)
        {
            stage3_index++;
        }
    }
}

/*============== ENSO MULTI QUEUE VERSION DEMO ========================*/

bool
rxEnsoStateMachine(int queueID)
{
    bool rxTickQueue = rxTick; //rxTickQueue is used to keep track of the rxTick status for the queueID
    if (!regs.rctl.en()) {
        rxTickQueue = false;
        DPRINTF(EthernetENSO, "RXS[%d]: RX disabled, stopping ticking\n", queueID);
        return rxTickQueue;
    }

    // If the packet is done check for interrupts/descriptors/etc
    if (rxNotifBufManager.packetDone()) {
        rxDmaPacket = false;
        DPRINTF(EthernetENSO, "RXS: Packet completed DMA to memory\n");

        // if DMA packet complete & have to notfy, doing DMA notification
        if(rxDmaNotif)
        {
            DPRINTF(EthernetENSO, "RXS: Writing notification into memory!!\n");
            rxNotifBufManager.writeNotification();
            DPRINTF(EthernetENSO,
                "RXS: stopping ticking until notification DMA completes\n");
            rxTick = false;
            return;
        }

        return;
    }

    if(rxNotifBufManager.notificationDone())
    {
        rxDmaNotif = false;
        DPRINTF(EthernetENSO, "RXS: Notification completed DMA to memory\n");
        return;
    }

    // check notfiy after DMA data complete
    if(rxDmaNotif)
    {
        DPRINTF(EthernetENSO,
                "RXS: stopping ticking until notification DMA completes\n");
        rxTick = false;
        return;
    }

    if (rxDmaPacket) {
        DPRINTF(EthernetENSO,
                "RXS: stopping ticking until packet DMA completes\n");
        rxTick = false;
        return;
    }

    if (EnsoRxFifo.empty()) {
        DPRINTF(EthernetENSO, "RXS: RxFIFO empty, stopping ticking\n");
    
        rxTick = false;
        return;
    }

    EnsoRxPtr pkt = EnsoRxFifo.front();
    

    if (rxEnsoPipeManager.isEnsoPipeFull(pkt->length, pkt->meta.pktQueueId)) {
        // Host enso pipe is full, drop packet
        etherDeviceStats.rxEnsoPipeFull++;
        DPRINTF(EthernetENSO, "RXS: Host RX enso pipe is full, stop ticking...\n");
        // need to drop packet?
        EnsoRxFifo.pop();
        rxEnsoPipeManager.setFull(1);
        int rxFifoFull = 0; // no matter what it is
        updateDropFSMEnso(rxFifoFull, rxEnsoPipeManager.isFull());
        rxTick = false;
        return;
    }
    rxEnsoPipeManager.setFull(0);

    // notify logic when host update SWhead, considering multi-enso pipe
    if(pipeHeadUpdated[pkt->meta.pktQueueId])
    {
        rxEnsoPipeManager.onRxUpdate(pkt);
        pipeHeadUpdated[pkt->meta.pktQueueId] = false;
    }

    // need to check notification buffer is full??
    // change to pending??
    if( pkt->meta.isNotify && rxNotifBufManager.isNotifBufFull(pkt->meta.pktQueueId))
    {  
        etherDeviceStats.rxNotifBufferFull++;
        DPRINTF(EthernetENSO, "RXS: Host RX notification buffer is full, stop ticking...\n");
        // etherDeviceStats.rxNotifBufFull++;
        rxTick = false;
        return;
    }

    if(pkt->meta.isNotify)
    {
        rxDmaNotif = true;
    }

    

    // DMA packet (+ Notification)
    rxNotifBufManager.writePacket(pkt);
    DPRINTF(EthernetENSO, "RXS: Writing packet into memory\n");
    // update RxEnsoPipe Tail register value
    // timing is right?
    rxEnsoPipeManager.updateRxPipeTail(pkt->pipe->tail, pkt->meta.pktQueueId);
    // data pop from DataFIFO
    //dataPop();
    //EnsoRxFifo.pop_front();
    EnsoRxFifo.pop();
    DPRINTF(EthernetENSO, "RXS: left %u bytes in fifo\n", EnsoRxFifo.avail());
    DPRINTF(EthernetENSO, "RXS: stopping ticking until packet DMA completes\n");
    rxTick = false;
    rxDmaPacket = true;


    return;
}
