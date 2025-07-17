/*
 * Copyright (c) 2002-2005 The Regents of The University of Michigan
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

/* @file
 * Reference counted class containing ethernet packet data
 */

#ifndef __DEV_NET_ETHERPKT_HH__
#define __DEV_NET_ETHERPKT_HH__

#include <cassert>
#include <iosfwd>
#include <memory>

#include "base/types.hh"
#include "sim/serialize.hh"

namespace gem5
{

/*
 * Reference counted class containing ethernet packet data
 */
class EthPacketData
{
  public:
    /**
     * Pointer to packet data will be deleted
     */
    uint8_t *data;

    /**
     * Total size of the allocated data buffer.
     */
    unsigned bufLength;

    /**
     * Amount of space occupied by the payload in the data buffer
     */
    unsigned length;

    /**
     * Effective length, used for modeling timing in the simulator.
     * This could be different from length if the packets are assumed
     * to use a tightly packed or compressed format, but it's not worth
     * the performance/complexity hit to perform that packing or compression
     * in the simulation.
     */
    unsigned simLength;

    uint32_t rssHash; // RSS hash value - temporal storage -> finally should be moved to the descriptor

    uint64_t rxMadeTick; // Tick when the packet was created from EtherLoadGen
    uint64_t rxPortTick; // Tick when the packet was received by the NIC
    uint64_t rxFifoTick; // Tick when the packet was enqueued in the NIC's receive FIFO
    uint64_t rxDMAStartTick; // Tick when the packet was dequeued from the NIC's receive FIFO
    uint64_t rxDMAEndTick; // Tick when the packet was fully received by the NIC

    bool rxFifoNotEmptyDmaBusyChecked;


    EthPacketData()
        : data(nullptr), bufLength(0), length(0), simLength(0), rssHash(0), rxMadeTick(0), rxPortTick(0), rxFifoTick(0), rxDMAStartTick(0), rxDMAEndTick(0), rxFifoNotEmptyDmaBusyChecked(false)
    { }

    explicit EthPacketData(unsigned size)
        : data(new uint8_t[size]), bufLength(size), length(0), simLength(0), rssHash(0), rxMadeTick(0), rxPortTick(0), rxFifoTick(0), rxDMAStartTick(0), rxDMAEndTick(0), rxFifoNotEmptyDmaBusyChecked(false)
    { }

    ~EthPacketData() { if (data) delete [] data; }

    void serialize(const std::string &base, CheckpointOut &cp) const;
    void unserialize(const std::string &base, CheckpointIn &cp);
};
typedef std::shared_ptr<EthPacketData> EthPacketPtr;

#define USE_ENSO
#ifdef USE_ENSO
  struct MetaData
    {
      // uint16_t pktID; // Data offset in Data FIFO
      uint32_t pktQueueId; // Enso Pipe ID
      bool isNotify; // Is noftification needed after this pkt DMAed?
      MetaData() : pktQueueId(0), isNotify(false) {}
    };

    struct PipeState
    {
      uint64_t physAddr;   // Host Enso Pipe physical addr for DMA
      bool pipeStatus;     // Enso Pipe Status using for notify logic, 1 == busy, 0 == idle
      uint64_t notifBufId; // Notification buffer ID, typically 1 notification buffer per core

      // internal queue_state;
      // uint16_t head;
      uint32_t tail;
    };

    class EnsoRxData
    {
    public:
        /**
         * Pointer to packet data will be deleted
         */
        uint8_t *data;

        /**
         * Total size of the allocated data buffer.
         */
        unsigned bufLength;

        /**
         * Amount of space occupied by the payload in the data buffer
         */
        unsigned length;

        /**
         * number of flit of this pkt
         */
        unsigned flits;
        
        /**
         * metadata for this packet
         */
        MetaData meta;

        /**
         * EnsoPipeState used by this pkt
         */
        PipeState *pipe;

        EnsoRxData()
            : data(nullptr), bufLength(0), length(0), flits(0), meta(), pipe(nullptr)
        { }

        explicit EnsoRxData(unsigned size)
            : data(new uint8_t[size]), bufLength(size), length(0), flits(), meta(), pipe(nullptr)
        { }

        ~EnsoRxData() { if (data) delete [] data; }

        void serialize(const std::string &base, CheckpointOut &cp) const;
        void unserialize(const std::string &base, CheckpointIn &cp);
    };

    typedef std::shared_ptr<EnsoRxData> EnsoRxPtr;
  #endif

} // namespace gem5

#endif // __DEV_NET_ETHERPKT_HH__
