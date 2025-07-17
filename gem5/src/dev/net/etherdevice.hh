/*
 * Copyright (c) 2007 The Regents of The University of Michigan
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

/**
 * @file
 * Base Ethernet Device declaration.
 */

#ifndef __DEV_NET_ETHERDEVICE_HH__
#define __DEV_NET_ETHERDEVICE_HH__

#include "base/statistics.hh"
#include "dev/pci/device.hh"
#include "params/EtherDevBase.hh"
#include "params/EtherDevice.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class EtherInt;

class EtherDevice : public PciDevice
{
  public:
    using Params = EtherDeviceParams;
    EtherDevice(const Params &params)
        : PciDevice(params),
          etherDeviceStats(this)
    {}

  protected:
    struct EtherDeviceStats : public statistics::Group
    {
        EtherDeviceStats(statistics::Group *parent);
        
        statistics::Scalar dmaDrops;
        statistics::Scalar coreDrops;
        statistics::Scalar txDrops;
        statistics::Scalar unknownDrops;
        statistics::Scalar rxdisabledDrops;
        statistics::Scalar rxFifoFullCount;
        statistics::Scalar txFifoFullCount;
        statistics::Scalar rxDescCacheFullCount;
        statistics::Scalar txDescCacheFullCount;
        statistics::Scalar rxRingBufferFull;
        statistics::Scalar txRingBufferFull;
        statistics::Scalar m2funcTxFifoMaxLen;

        statistics::Scalar rxFifoNotEmptyDmaBusy;
        statistics::Scalar rxFifoNotEmptyRSSBad;

        // statistics::Histogram rxEnd2EndClk;
        // statistics::Histogram rxEtherLinkClk;
        // statistics::Histogram rxPort2FifoClk;
        // statistics::Histogram rxFifo2DmaClk;
        // statistics::Histogram rxDma2CoreClk;
        
        statistics::Histogram rxPacketComeDistance; // The time between the time of consecutive packets ready
        statistics::Histogram rxM2funcReadDistance; // The time between the time of consecutive reads from the M2func
        statistics::Histogram rxRingBufferStartDMADistance; // The time between the time of consecutive DMA start from the ring buffer

        statistics::Scalar postedInterrupts;

        statistics::Scalar txBytes;
        statistics::Scalar rxBytes;

        statistics::Scalar txDMABytes;
        statistics::Scalar rxDMABytes;
        statistics::Scalar metaDMABytes;
        statistics::Scalar rxDescFetchBytes;
        statistics::Scalar txDescFetchBytes;
        statistics::Scalar rxDescWBBytes;
        statistics::Scalar txDescWBBytes;
        statistics::Scalar rxTailWriteBytes;
        statistics::Scalar txTailWriteBytes;

        //For M2func
        statistics::Scalar txBytesM2func; // tx Bytes from host to device (total bytes used in M2func - including dummy data within flit) 
        statistics::Scalar rxBytesM2func; // rx Bytes from device to host (total bytes used in M2func)
        statistics::Scalar txBytesM2funcData; // tx Bytes from host to device (actual data bytes used in M2func)
        statistics::Scalar rxBytesM2funcData; // rx Bytes from device to host (actual data bytes used in M2func)
        statistics::Scalar txBytesM2funcDesc; // tx Bytes from host to device (actual descriptor bytes used in M2func)
        statistics::Scalar rxBytesM2funcDesc; // rx Bytes from device to host (actual descriptor bytes used in M2func)
        statistics::Scalar txBytesM2funcBitMask; // tx Bytes from host to device (actual bitmask bytes used in M2func)

        statistics::Scalar txPackets;
        statistics::Scalar rxPackets;

        statistics::Formula txBandwidth;
        statistics::Formula rxBandwidth;

        statistics::Formula txDMABandwidth;
        statistics::Formula rxDMABandwidth;
        statistics::Formula metaDMABandwidth;
        statistics::Formula rxDescFetchBandwidth;
        statistics::Formula txDescFetchBandwidth;
        statistics::Formula rxDescWBBandwidth;
        statistics::Formula txDescWBBandwidth;
        statistics::Formula rxTailWriteBandwidth;
        statistics::Formula txTailWriteBandwidth;

        statistics::Formula txM2funcBandwidth; // tx Bandwidth from host to device (total bandwidth used in M2func)
        statistics::Formula rxM2funcBandwidth; // rx Bandwidth from device to host (total bandwidth used in M2func)
        statistics::Formula txM2funcEffectiveBandwidth; // tx Bandwidth from host to device (actual data bandwidth used in M2func - excluding dummy data within flit)
        statistics::Formula rxM2funcEffectiveBandwidth; // rx Bandwidth from device to host (actual data bandwidth used in M2func)

        statistics::Scalar txIpChecksums;
        statistics::Scalar rxIpChecksums;

        statistics::Scalar txTcpChecksums;
        statistics::Scalar rxTcpChecksums;

        statistics::Scalar txUdpChecksums;
        statistics::Scalar rxUdpChecksums;

        statistics::Scalar descDmaReads;
        statistics::Scalar descDmaWrites;

        statistics::Scalar descDmaRdBytes;
        statistics::Scalar descDmaWrBytes;

        statistics::Formula totBandwidth;
        statistics::Formula totDMABandwidth;
        statistics::Formula totPackets;
        statistics::Formula totBytes;
        statistics::Formula totPacketRate;

        statistics::Formula txPacketRate;
        statistics::Formula rxPacketRate;

        statistics::Scalar postedSwi;
        statistics::Scalar totalSwi;
        statistics::Formula coalescedSwi;

        statistics::Scalar postedRxIdle;
        statistics::Scalar totalRxIdle;
        statistics::Formula coalescedRxIdle;

        statistics::Scalar postedRxOk;
        statistics::Scalar totalRxOk;
        statistics::Formula coalescedRxOk;

        statistics::Scalar postedRxDesc;
        statistics::Scalar totalRxDesc;
        statistics::Formula coalescedRxDesc;

        statistics::Scalar postedTxOk;
        statistics::Scalar totalTxOk;
        statistics::Formula coalescedTxOk;

        statistics::Scalar postedTxIdle;
        statistics::Scalar totalTxIdle;
        statistics::Formula coalescedTxIdle;

        statistics::Scalar postedTxDesc;
        statistics::Scalar totalTxDesc;
        statistics::Formula coalescedTxDesc;

        statistics::Scalar postedRxOrn;
        statistics::Scalar totalRxOrn;
        statistics::Formula coalescedRxOrn;

        statistics::Formula coalescedTotal;
        statistics::Scalar droppedPackets;
        #ifdef USE_ENSO
        statistics::Scalar rxNotification; // sent by NIC
        statistics::Scalar txNotification; // used by NIC
        statistics::Scalar complNotification; // TX compl sent by NIC
        
        statistics::Scalar rxNotifDMABytes; // send
        statistics::Scalar txNotifDMABytes; // fetch
        statistics::Scalar txComplDMABytes; // send

        statistics::Scalar rxEnsoPipeFull;
        statistics::Scalar rxNotifBufferFull;
        //statistics::Scalar txNotifBufferFull;
        #endif

    } etherDeviceStats;
};

/**
 * Dummy class to keep the Python class hierarchy in sync with the C++
 * object hierarchy.
 *
 * The Python object hierarchy includes the EtherDevBase class which
 * is used by some ethernet devices as a way to share common
 * configuration information in the generated param structs. Since the
 * Python hierarchy is used to generate a Python interfaces for all C++
 * SimObjects, we need to reflect this in the C++ object hierarchy.
 */
class EtherDevBase : public EtherDevice
{
  public:
    using Params = EtherDevBaseParams;
    EtherDevBase(const Params &params)
        : EtherDevice(params)
    {}
};

} // namespace gem5

#endif // __DEV_NET_ETHERDEVICE_HH__
