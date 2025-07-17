/*
 * Copyright (c) 2004-2005 The Regents of The University of Michigan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "dev/net/etherdevice.hh"

#include "sim/stats.hh"

namespace gem5
{

EtherDevice::EtherDeviceStats::EtherDeviceStats(statistics::Group *parent)
    : statistics::Group(parent, "EtherDevice"),
      ADD_STAT(dmaDrops, statistics::units::Count::get(),
               "Number of packet drops due to DMA/rxFifo bottleneck"),
      ADD_STAT(coreDrops, statistics::units::Count::get(),
               "Number of packet drops due to Core bottleneck"),
      ADD_STAT(txDrops, statistics::units::Count::get(),
               "Number of packet drops due to TX Ring Buffer bottleneck"),
      ADD_STAT(unknownDrops, statistics::units::Count::get(),
               "Number of packet drops due to either dma, core, or tx bottleneck"),
      ADD_STAT(rxdisabledDrops, statistics::units::Count::get(),
               "Number of packet drops due to rx state machine being disabled"),
      ADD_STAT(rxFifoFullCount, statistics::units::Count::get(),
               "Number of times the rxFifo fills up"),
      ADD_STAT(txFifoFullCount, statistics::units::Count::get(),
               "Number of times the txFifo fills up"),
      ADD_STAT(rxDescCacheFullCount, statistics::units::Count::get(),
               "Number of times the rxDescCache fills up"),
      ADD_STAT(txDescCacheFullCount, statistics::units::Count::get(),
               "Number of times the txDescCache fills up"),
      ADD_STAT(rxRingBufferFull, statistics::units::Count::get(),
               "Number of times the rxRingBuffer fills up"),
      ADD_STAT(txRingBufferFull, statistics::units::Count::get(),
               "Number of times the txRingBuffer fills up"),
      ADD_STAT(m2funcTxFifoMaxLen, statistics::units::Count::get(),
               "Maximum length of the m2func txFifo"),
      ADD_STAT(rxFifoNotEmptyDmaBusy, statistics::units::Count::get(),
               "Number of times the rxFifo is not empty but the DMA is busy"),
      ADD_STAT(rxFifoNotEmptyRSSBad, statistics::units::Count::get(),
               "Number of times the rxFifo is not empty but the RSS queue result is bad (target queue is full, but the non-target is not)"),
    //   ADD_STAT(rxEnd2EndClk, statistics::units::Second::get(), 
    //            "Distribution of end to end ms for received packets"),
    //   ADD_STAT(rxEtherLinkClk, statistics::units::Second::get(), 
    //            "Distribution of time spent in the Ethernet link for received packets"),
    //   ADD_STAT(rxPort2FifoClk, statistics::units::Second::get(), 
    //            "Distribution of time spent in the port to fifo for received packets"),
    //   ADD_STAT(rxFifo2DmaClk, statistics::units::Second::get(), 
    //            "Distribution of time spent in the fifo to start of dma for received packets"),
    //   ADD_STAT(rxDma2CoreClk, statistics::units::Second::get(), 
    //            "Distribution of time spent in the dma to core for received packets"),
      ADD_STAT(rxPacketComeDistance, statistics::units::Tick::get(),
               "Distribution of Tick between consecutive receive packets comming in rxFifo"),
      ADD_STAT(rxM2funcReadDistance, statistics::units::Tick::get(),
               "Distribution of Tick between consecutive reads from M2func"),
      ADD_STAT(rxRingBufferStartDMADistance, statistics::units::Tick::get(),
               "Distribution of Tick between consecutive DMA start from the ring buffer"),
      ADD_STAT(postedInterrupts, statistics::units::Count::get(),
               "Number of posts to CPU"),
      ADD_STAT(txBytes, statistics::units::Byte::get(),
               "Bytes Transmitted through etherlink"),
      ADD_STAT(rxBytes, statistics::units::Byte::get(), "Bytes Received through etherlink"),
      ADD_STAT(txDMABytes, statistics::units::Byte::get(),
               "Bytes Transmitted by DMA"),
      ADD_STAT(rxDMABytes, statistics::units::Byte::get(),
                "Bytes Received by DMA"),
      ADD_STAT(metaDMABytes, statistics::units::Byte::get(),
               "Bytes Transmitted by DMA for metadata (DescFetch + DescWB + TailWrite)"),
      ADD_STAT(rxDescFetchBytes, statistics::units::Byte::get(),
               "RX Bytes DMA for descriptor fetch"),
      ADD_STAT(txDescFetchBytes, statistics::units::Byte::get(),
                "TX Bytes DMA for descriptor fetch"),
      ADD_STAT(rxDescWBBytes, statistics::units::Byte::get(),
               "RX Bytes DMA for descriptor writeback"),
      ADD_STAT(txDescWBBytes, statistics::units::Byte::get(),
                "TX Bytes DMA for descriptor writeback"),
      ADD_STAT(rxTailWriteBytes, statistics::units::Byte::get(),
                "RX Bytes DMA for tail write"),
      ADD_STAT(txTailWriteBytes, statistics::units::Byte::get(),
                "TX Bytes DMA for tail write"),
      
      ADD_STAT(txBytesM2func, statistics::units::Byte::get(),
               "Total Bytes Transmitted by M2func"),
      ADD_STAT(rxBytesM2func, statistics::units::Byte::get(),
                "Total Bytes Received by M2func"),
      ADD_STAT(txBytesM2funcData, statistics::units::Byte::get(),
                "Actual Data Bytes Transmitted by M2func"),
      ADD_STAT(rxBytesM2funcData, statistics::units::Byte::get(),
                "Actual Data Bytes Received by M2func"),
      ADD_STAT(txBytesM2funcDesc, statistics::units::Byte::get(),
                "Actual Descriptor Bytes Transmitted by M2func"),
      ADD_STAT(rxBytesM2funcDesc, statistics::units::Byte::get(),
                "Actual Descriptor Bytes Received by M2func"),
      ADD_STAT(txBytesM2funcBitMask, statistics::units::Byte::get(),
                "Actual BitMask Bytes Transmitted by M2func"),
    
    #ifdef USE_ENSO
      ADD_STAT(rxEnsoPipeFull, statistics::units::Count::get(),
               "Number of times the host rxEnsoPipe fills up"),
      ADD_STAT(rxNotifBufferFull, statistics::units::Count::get(),
               "Number of times the host rxNotificationBuffer fills up"),
      ADD_STAT(rxNotification, statistics::units::Count::get(),
               "Number of Rx Notifications Transmitted by Rx Notif Manager"),
      ADD_STAT(txNotification, statistics::units::Count::get(),
               "Number of Rx Notifications Received by Tx Notif Manager"),
      ADD_STAT(complNotification, statistics::units::Count::get(),
               "Number of Tx Completions Transmitted by Tx Notif Manager"),
      ADD_STAT(rxNotifDMABytes, statistics::units::Byte::get(),
               "RxNotification Bytes DMA wr for notify host"),
      ADD_STAT(txNotifDMABytes, statistics::units::Byte::get(),
               "TxNotification Bytes DMA rd for fetch TX data"),
      ADD_STAT(txComplDMABytes, statistics::units::Byte::get(),
               "TxCompletions Bytes DMA wr for write back"),
    #endif
      ADD_STAT(txPackets, statistics::units::Count::get(),
               "Number of Packets Transmitted"),
      ADD_STAT(rxPackets, statistics::units::Count::get(),
               "Number of Packets Received"),
      ADD_STAT(txBandwidth, statistics::units::Rate<
                    statistics::units::Bit, statistics::units::Second>::get(),
               "Transmit Bandwidth",
               txBytes * statistics::constant(8) / simSeconds),
      ADD_STAT(rxBandwidth, statistics::units::Rate<
                    statistics::units::Bit, statistics::units::Second>::get(),
               "Receive Bandwidth",
               rxBytes * statistics::constant(8) / simSeconds),
      ADD_STAT(txDMABandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
               "Transmit Bandwidth by DMA (MB/s)",
               (txDMABytes / 1000000) / simSeconds),
      ADD_STAT(rxDMABandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
               "Receive Bandwidth by DMA (MB/s)",
               (rxDMABytes / 1000000) / simSeconds),
      ADD_STAT(metaDMABandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
               "Bandwidth by DMA for metadata (MB/s)",
               (metaDMABytes / 1000000) / simSeconds),
      ADD_STAT(rxDescFetchBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
               "RX Bandwidth by DMA for descriptor fetch (MB/s)",
               (rxDescFetchBytes / 1000000) / simSeconds),
      ADD_STAT(txDescFetchBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
                "TX Bandwidth by DMA for descriptor fetch (MB/s)",
                (txDescFetchBytes / 1000000) / simSeconds),
      ADD_STAT(rxDescWBBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
                "RX Bandwidth by DMA for descriptor writeback (MB/s)",
                (rxDescWBBytes / 1000000) / simSeconds),
      ADD_STAT(txDescWBBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
                "TX Bandwidth by DMA for descriptor writeback (MB/s)",
                (txDescWBBytes / 1000000) / simSeconds),
      ADD_STAT(rxTailWriteBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
                "RX Bandwidth by DMA for tail write (MB/s)",
                (rxTailWriteBytes / 1000000) / simSeconds),
      ADD_STAT(txTailWriteBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
                "TX Bandwidth by DMA for tail write (MB/s)",
                (txTailWriteBytes / 1000000) / simSeconds),
      ADD_STAT(txM2funcBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
                "TX Bandwidth by M2func (MB/s)",
                (txBytesM2func / 1000000) / simSeconds),
      ADD_STAT(rxM2funcBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
                "RX Bandwidth by M2func (MB/s)",
                (rxBytesM2func / 1000000) / simSeconds),
      ADD_STAT(txM2funcEffectiveBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
                "TX Effective Bandwidth by M2func (MB/s)",
                ((txBytesM2funcData + txBytesM2funcDesc + txBytesM2funcBitMask) / 1000000) / simSeconds),
      ADD_STAT(rxM2funcEffectiveBandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
                "RX Effective Bandwidth by M2func (MB/s)",
                ((rxBytesM2funcData + rxBytesM2funcDesc) / 1000000) / simSeconds),
      ADD_STAT(txIpChecksums, statistics::units::Count::get(),
               "Number of tx IP Checksums done by device"),
      ADD_STAT(rxIpChecksums, statistics::units::Count::get(),
               "Number of rx IP Checksums done by device"),
      ADD_STAT(txTcpChecksums, statistics::units::Count::get(),
               "Number of tx TCP Checksums done by device"),
      ADD_STAT(rxTcpChecksums, statistics::units::Count::get(),
               "Number of rx TCP Checksums done by device"),
      ADD_STAT(txUdpChecksums, statistics::units::Count::get(),
               "Number of tx UDP Checksums done by device"),
      ADD_STAT(rxUdpChecksums, statistics::units::Count::get(),
               "Number of rx UDP Checksums done by device"),
      ADD_STAT(descDmaReads, statistics::units::Count::get(),
               "Number of descriptors the device read w/ DMA"),
      ADD_STAT(descDmaWrites, statistics::units::Count::get(),
               "Number of descriptors the device wrote w/ DMA"),
      ADD_STAT(descDmaRdBytes, statistics::units::Count::get(),
               "Number of descriptor bytes read w/ DMA"),
      ADD_STAT(descDmaWrBytes, statistics::units::Count::get(),
               "Number of descriptor bytes write w/ DMA"),
      ADD_STAT(totBandwidth, statistics::units::Rate<
                    statistics::units::Bit, statistics::units::Second>::get(),
               "Total Bandwidth",
               txBandwidth + rxBandwidth),
      ADD_STAT(totDMABandwidth, statistics::units::Rate<
                    statistics::units::Byte, statistics::units::Second>::get(),
               "Total Bandwidth (tx + rx + metadata) by DMA (MB/s)",
               txDMABandwidth + rxDMABandwidth + metaDMABandwidth),
      ADD_STAT(totPackets, statistics::units::Count::get(), "Total Packets",
               txPackets + rxPackets),
      ADD_STAT(totBytes, statistics::units::Byte::get(), "Total Bytes",
               txBytes + rxBytes),
      ADD_STAT(totPacketRate, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Second>::get(),
               "Total Packet Tranmission Rate",
               totPackets / simSeconds),
      ADD_STAT(txPacketRate, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Second>::get(),
               "Packet Tranmission Rate",
               txPackets / simSeconds),
      ADD_STAT(rxPacketRate, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Second>::get(),
               "Packet Reception Rate",
               rxPackets / simSeconds),
      ADD_STAT(postedSwi, statistics::units::Count::get(),
               "Number of software interrupts posted to CPU"),
      ADD_STAT(totalSwi, statistics::units::Count::get(),
               "Total number of Swi written to ISR"),
      ADD_STAT(coalescedSwi, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Count>::get(),
               "Average number of Swi's coalesced into each post",
               totalSwi / postedInterrupts),
      ADD_STAT(postedRxIdle, statistics::units::Count::get(),
               "Number of rxIdle interrupts posted to CPU"),
      ADD_STAT(totalRxIdle, statistics::units::Count::get(),
               "Total number of RxIdle written to ISR"),
      ADD_STAT(coalescedRxIdle, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Count>::get(),
               "Average number of RxIdle's coalesced into each post",
               totalRxIdle / postedInterrupts),
      ADD_STAT(postedRxOk, statistics::units::Count::get(),
               "Number of RxOk interrupts posted to CPU"),
      ADD_STAT(totalRxOk, statistics::units::Count::get(),
               "Total number of RxOk written to ISR"),
      ADD_STAT(coalescedRxOk, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Count>::get(),
               "Average number of RxOk's coalesced into each post",
               totalRxOk / postedInterrupts),
      ADD_STAT(postedRxDesc, statistics::units::Count::get(),
               "Number of RxDesc interrupts posted to CPU"),
      ADD_STAT(totalRxDesc, statistics::units::Count::get(),
               "Total number of RxDesc written to ISR"),
      ADD_STAT(coalescedRxDesc, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Count>::get(),
               "Average number of RxDesc's coalesced into each post",
               totalRxDesc / postedInterrupts),
      ADD_STAT(postedTxOk, statistics::units::Count::get(),
               "Number of TxOk interrupts posted to CPU"),
      ADD_STAT(totalTxOk, statistics::units::Count::get(),
               "Total number of TxOk written to ISR"),
      ADD_STAT(coalescedTxOk, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Count>::get(),
               "Average number of TxOk's coalesced into each post",
               totalTxOk / postedInterrupts),
      ADD_STAT(postedTxIdle, statistics::units::Count::get(),
               "Number of TxIdle interrupts posted to CPU"),
      ADD_STAT(totalTxIdle, statistics::units::Count::get(),
               "Total number of TxIdle written to ISR"),
      ADD_STAT(coalescedTxIdle, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Count>::get(),
               "Average number of TxIdle's coalesced into each post",
               totalTxIdle / postedInterrupts),
      ADD_STAT(postedTxDesc, statistics::units::Count::get(),
               "Number of TxDesc interrupts posted to CPU"),
      ADD_STAT(totalTxDesc, statistics::units::Count::get(),
               "Total number of TxDesc written to ISR"),
      ADD_STAT(coalescedTxDesc, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Count>::get(),
               "Average number of TxDesc's coalesced into each post",
               totalTxDesc / postedInterrupts),
      ADD_STAT(postedRxOrn, statistics::units::Count::get(),
               "Number of RxOrn posted to CPU"),
      ADD_STAT(totalRxOrn, statistics::units::Count::get(),
               "Total number of RxOrn written to ISR"),
      ADD_STAT(coalescedRxOrn, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Count>::get(),
               "Average number of RxOrn's coalesced into each post",
               totalRxOrn / postedInterrupts),
      ADD_STAT(coalescedTotal, statistics::units::Rate<
                    statistics::units::Count, statistics::units::Count>::get(),
               "Average number of interrupts coalesced into each post"),
      ADD_STAT(droppedPackets, statistics::units::Count::get(),
               "Number of packets dropped")
{
    rxFifoNotEmptyDmaBusy
        .precision(0);
    rxFifoNotEmptyRSSBad
        .precision(0);
    
    rxPacketComeDistance
        .init(100)
        .flags(statistics::pdf);
    rxM2funcReadDistance
        .init(100)
        .flags(statistics::pdf);
    rxRingBufferStartDMADistance
        .init(100)
        .flags(statistics::pdf);

    // rxEnd2EndClk.init(100);
    // rxEtherLinkClk.init(100);
    // rxPort2FifoClk.init(100);
    // rxFifo2DmaClk.init(100);
    // rxDma2CoreClk.init(100);

    postedInterrupts
        .precision(0);

    txBytes
        .prereq(txBytes);

    rxBytes
        .prereq(rxBytes);

    txDMABytes
        .prereq(txDMABytes);
    
    rxDMABytes
        .prereq(rxDMABytes);

    metaDMABytes
        .prereq(metaDMABytes);
    
    rxDescFetchBytes
        .prereq(rxDescFetchBytes);
    
    txDescFetchBytes
        .prereq(txDescFetchBytes);
    
    rxDescWBBytes
        .prereq(rxDescWBBytes);
    
    txDescWBBytes
        .prereq(txDescWBBytes);
    
    rxTailWriteBytes
        .prereq(rxTailWriteBytes);

    txTailWriteBytes
        .prereq(txTailWriteBytes);
    
    txBytesM2func
        .prereq(txBytesM2func);
    
    rxBytesM2func
        .prereq(rxBytesM2func);

    txBytesM2funcData
        .prereq(txBytesM2funcData);
    
    rxBytesM2funcData
        .prereq(rxBytesM2funcData);
    
    txBytesM2funcDesc
        .prereq(txBytesM2funcDesc);
    
    rxBytesM2funcDesc
        .prereq(rxBytesM2funcDesc);
    
    txBytesM2funcBitMask
        .prereq(txBytesM2funcBitMask);

    #ifdef USE_ENSO
    rxNotification
        .prereq(rxNotifDMABytes);
    
    txNotification
        .prereq(txNotifDMABytes);
    
    complNotification
        .prereq(txComplDMABytes);

    rxNotifDMABytes
        .prereq(rxNotifDMABytes);

    txNotifDMABytes
        .prereq(txNotifDMABytes);

    txComplDMABytes
        .prereq(txComplDMABytes);
    #endif

    txPackets
        .prereq(txBytes);

    rxPackets
        .prereq(rxBytes);

    txIpChecksums
        .precision(0)
        .prereq(txBytes);

    rxIpChecksums
        .precision(0)
        .prereq(rxBytes);

    txTcpChecksums
        .precision(0)
        .prereq(txBytes);

    rxTcpChecksums
        .precision(0)
        .prereq(rxBytes);

    txUdpChecksums
        .precision(0)
        .prereq(txBytes);

    rxUdpChecksums
        .precision(0)
        .prereq(rxBytes);

    descDmaReads
        .precision(0);

    descDmaWrites
        .precision(0);

    descDmaRdBytes
        .precision(0);

    descDmaWrBytes
        .precision(0);

    txBandwidth
        .precision(0)
        .prereq(txBytes)
        ;

    rxBandwidth
        .precision(0)
        .prereq(rxBytes);
    
    txDMABandwidth
        .precision(0)
        .prereq(txDMABytes);
    
    rxDMABandwidth
        .precision(0)
        .prereq(rxDMABytes);
    
    metaDMABandwidth
        .precision(0)
        .prereq(metaDMABytes);
    
    rxDescFetchBandwidth
        .precision(0)
        .prereq(rxDescFetchBytes);
    
    txDescFetchBandwidth
        .precision(0)
        .prereq(txDescFetchBytes);
    
    rxDescWBBandwidth
        .precision(0)
        .prereq(rxDescWBBytes);

    txDescWBBandwidth
        .precision(0)
        .prereq(txDescWBBytes);
    
    rxTailWriteBandwidth
        .precision(0)
        .prereq(rxTailWriteBytes);
    
    txTailWriteBandwidth
        .precision(0)
        .prereq(txTailWriteBytes);
    
    txM2funcBandwidth
        .precision(0)
        .prereq(txBytesM2func);
    
    rxM2funcBandwidth
        .precision(0)
        .prereq(rxBytesM2func);
    
    txM2funcEffectiveBandwidth
        .precision(0)
        .prereq(txBytesM2funcData);
    
    rxM2funcEffectiveBandwidth
        .precision(0)
        .prereq(rxBytesM2funcData);
    
    totBandwidth
        .precision(0)
        .prereq(totBytes);
    
    totDMABandwidth
        .precision(0)
        .prereq(totBytes);

    totPackets
        .precision(0)
        .prereq(totBytes);

    totBytes
        .precision(0)
        .prereq(totBytes);

    totPacketRate
        .precision(0)
        .prereq(totBytes);

    txPacketRate
        .precision(0)
        .prereq(txBytes);

    rxPacketRate
        .precision(0)
        .prereq(rxBytes);

    postedSwi
        .precision(0);

    totalSwi
        .precision(0);

    coalescedSwi
        .precision(0);

    postedRxIdle
        .precision(0);

    totalRxIdle
        .precision(0);

    coalescedRxIdle
        .precision(0);

    postedRxOk
        .precision(0);

    totalRxOk
        .precision(0);

    coalescedRxOk
        .precision(0);

    postedRxDesc
        .precision(0);

    totalRxDesc
        .precision(0);

    coalescedRxDesc
        .precision(0);

    postedTxOk
        .precision(0);

    totalTxOk
        .precision(0);

    coalescedTxOk
        .precision(0);

    postedTxIdle
        .precision(0);

    totalTxIdle
        .precision(0);

    coalescedTxIdle
        .precision(0);

    postedTxDesc
        .precision(0);

    totalTxDesc
        .precision(0);

    coalescedTxDesc
        .precision(0);

    postedRxOrn
        .precision(0);

    totalRxOrn
        .precision(0);

    coalescedRxOrn
        .precision(0);

    coalescedTotal
        .precision(0);

    droppedPackets
        .precision(0);

    coalescedTotal = (totalSwi + totalRxIdle + totalRxOk + totalRxDesc +
                      totalTxOk + totalTxIdle + totalTxDesc +
                      totalRxOrn) / postedInterrupts;
}

} // namespace gem5
