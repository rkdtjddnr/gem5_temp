/*
 * Copyright (c) 2006 The Regents of The University of Michigan
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
 * Device model for Intel's 8254x line of gigabit ethernet controllers.
 */

#ifndef __DEV_NET_I8254XGBE_HH__
#define __DEV_NET_I8254XGBE_HH__

#include <cstdint>
#include <deque>
#include <string>

#include "base/inet.hh"
#include "base/trace.hh"
#include "base/types.hh"
#include "debug/EthernetDesc.hh"
#include "debug/EthernetIntr.hh"
#include "dev/net/etherdevice.hh"
#include "dev/net/etherint.hh"
#include "dev/net/etherpkt.hh"
#include "dev/net/i8254xGBe_defs.hh"
#include "dev/net/pktfifo.hh"
#include "dev/pci/device.hh"
#include "params/IGbE.hh"
#include "sim/eventq.hh"
#include "sim/serialize.hh"

namespace gem5
{

class IGbEInt;
class M2funcPort;

class IGbE : public EtherDevice
{
  private:
    // SHIN
    int adq = -1;
    IGbEInt *etherInt;

    // JM - for the port to connect NIC with DTA
    M2funcPort *m2funcPort;

    // device registers
    igbreg::Regs regs;

    // eeprom data, status and control bits
    int eeOpBits, eeAddrBits, eeDataBits;
    uint8_t eeOpcode, eeAddr;
    uint16_t flash[igbreg::EEPROM_SIZE];

    // packet fifos
    PacketFifo rxFifo;
    PacketFifo txFifo;

    // multi-queue
    int numQueues;

    // Packet that we are currently putting into the txFifo
    //jm - this should be multiple arrays
    // EthPacketPtr txPacket;
    EthPacketPtr txPacketArray[MAX_QUEUE_SIZE];

    //jm rx EthPacketPtr Array 
    //Packet that is currently being put into the RX queue
    //If RSS is enabled, RSS is done and push to the rxPacketArray per cycle.
    //If target rxPacketArray is full, it cannot pop the packet from rxFifo

    EthPacketPtr rxPacketArray[MAX_QUEUE_SIZE]; 

    // Should to Rx/Tx State machine tick?
    bool inTick;
    bool rxTick;
    bool txTick;
    bool txFifoTick;

    enum class CommunicationType : uint8_t {
      RING = 0,
      M2FUNC = 1
    };
    CommunicationType commType = CommunicationType::RING;

    bool enableDTA = false;

    bool is_dpdk_setup_step;

    unsigned flitSize; // Flit size in bytes

    //jm - this should be multiple arrays
    // bool rxDmaPacket;
    bool rxDmaPacketArray[MAX_QUEUE_SIZE];

    //jm - this should be multiple arrays
    // Number of bytes copied from current RX packet
    // unsigned pktOffset;
    unsigned pktOffsetArray[MAX_QUEUE_SIZE];

    // Delays in managaging descriptors
    Tick fetchDelay, wbDelay;
    Tick fetchCompDelay, wbCompDelay;
    Tick rxWriteDelay, txReadDelay;

    // JM - delay for CXL access
    Tick cxlMemDelay;

    // Event and function to deal with RDTR timer expiring
    // void rdtrProcess() {
    //     rxDescCache.writeback(0);
    //     DPRINTF(EthernetIntr,
    //             "Posting RXT interrupt because RDTR timer expired\n");
    //     postInterrupt(igbreg::IT_RXT);
    // }

    // EventFunctionWrapper rdtrEvent;

    // Event and function to deal with RADV timer expiring
    // void radvProcess() {
    //     rxDescCache.writeback(0);
    //     DPRINTF(EthernetIntr,
    //             "Posting RXT interrupt because RADV timer expired\n");
    //     postInterrupt(igbreg::IT_RXT);
    // }

    // EventFunctionWrapper radvEvent;

    // Event and function to deal with TADV timer expiring
    // void tadvProcess() {
    //     txDescCache.writeback(0);
    //     DPRINTF(EthernetIntr,
    //             "Posting TXDW interrupt because TADV timer expired\n");
    //     postInterrupt(igbreg::IT_TXDW);
    // }

    // EventFunctionWrapper tadvEvent;

    // Event and function to deal with TIDV timer expiring
    // void tidvProcess() {
    //     txDescCache.writeback(0);
    //     DPRINTF(EthernetIntr,
    //             "Posting TXDW interrupt because TIDV timer expired\n");
    //     postInterrupt(igbreg::IT_TXDW);
    // }
    // EventFunctionWrapper tidvEvent;

    // Main event to tick the device
    void tick();
    EventFunctionWrapper tickEvent;


    uint64_t macAddr;

    bool rxStateMachine(int queueID);
    // void rxStateMachine();
    bool txStateMachine(int queueID);
    // void txStateMachine();
    // Compute the hash function for the given input - used for RSS
    uint32_t computeHash(uint8_t *input, int length); 
    int doRSS(EthPacketPtr pkt);
    int prevRSSQueue; //It is for the case when RSS is enabled, but not ip, tcp, udp packet. just do round-robin
    // Decide the queue of the packet in front of the rxFifo & pop from rxFifo to the rxPacketArray
    void rxFifotoRxDesc();
    // select the queue to push the packet from txPacketArray to txFifo using round-robin
    void txPackettoTxFifo();
    int candidateTxQueue;
    bool successTxQueueSend;
    void updateDropFSM(int rxFifoFull, int rxRingFull, int txRingFull, int txFifoFull);
    void updateDropFSMM2func(int rxFifoFull, int pcieBusy, int txRingFull);
    double getPCIeUtilization();
    void txWire();

    /** Write an interrupt into the interrupt pending register and check mask
     * and interrupt limit timer before sending interrupt to CPU
     * @param t the type of interrupt we are posting
     * @param now should we ignore the interrupt limiting timer
     */
    void postInterrupt(igbreg::IntTypes t, bool now = false);

    /** Check and see if changes to the mask register have caused an interrupt
     * to need to be sent or perhaps removed an interrupt cause.
     */
    void chkInterrupt();

    /** Send an interrupt to the cpu
     */
    void delayIntEvent();
    void cpuPostInt();
    // Event to moderate interrupts
    EventFunctionWrapper interEvent;

    /** Clear the interupt line to the cpu
     */
    void cpuClearInt();

    Tick intClock() { return sim_clock::as_int::ns * 1024; }

    /** This function is used to restart the clock so it can handle things like
     * draining and resume in one place. */
    void restartClock();


    template<class T>
    class DescCache : public Serializable
    {
      protected:
        virtual Addr descBase() const = 0;
        virtual long descHead() const = 0;
        virtual long descTail() const = 0;
        virtual long descLen() const = 0;
        virtual void updateHead(long h) = 0;
        virtual void enableSm() = 0;
        virtual void actionAfterWb() {}
        virtual void fetchAfterWb() = 0;

        typedef std::deque<T *> CacheType;
        CacheType usedCache;
        CacheType unusedCache;

        T *fetchBuf;
        T *wbBuf;

        // Pointer to the device we cache for
        IGbE *igbe;

        // Name of this  descriptor cache
        std::string _name;

        // How far we've cached
        int cachePnt;

        // The size of the descriptor cache
        int size;

        // How many descriptors we are currently fetching
        int curFetching;

        // How many descriptors we are currently writing back
        int wbOut;

        // if the we wrote back to the end of the descriptor ring and are going
        // to have to wrap and write more
        bool moreToWb;

        // What the alignment is of the next descriptor writeback
        Addr wbAlignment;

        /** The packet that is currently being dmad to memory if any */
        EthPacketPtr pktPtr;

        // JM
        bool isRx;

        /** Shortcut for DMA address translation */
        Addr pciToDma(Addr a) { return igbe->pciToDma(a); }

      public:
        /** Annotate sm*/
        std::string annSmFetch, annSmWb, annUnusedDescQ, annUsedCacheQ,
            annUsedDescQ, annUnusedCacheQ, annDescQ;

        DescCache(IGbE *i, const std::string n, int s, bool _isRx);
        virtual ~DescCache();

        std::string name() { return _name; }

        /** If the address/len/head change when we've got descriptors that are
         * dirty that is very bad. This function checks that we don't and if we
         * do panics.
         */
        void areaChanged();

        void writeback(Addr aMask);
        void writeback1();
        EventFunctionWrapper wbDelayEvent;

        /** Fetch a chunk of descriptors into the descriptor cache.
         * Calls fetchComplete when the memory system returns the data
         */
        void fetchDescriptors();
        void fetchDescriptors1();
        EventFunctionWrapper fetchDelayEvent;

        /** Called by event when dma to read descriptors is completed
         */
        void fetchComplete();
        EventFunctionWrapper fetchEvent;

        /** Called by event when dma to writeback descriptors is completed
         */
        void wbComplete();
        EventFunctionWrapper wbEvent;

        /* Return the number of descriptors left in the ring, so the device has
         * a way to figure out if it needs to interrupt.
         */
        unsigned
        descLeft() const
        {
            unsigned left = unusedCache.size();
            if (cachePnt > descTail())
                left += (descLen() - cachePnt + descTail());
            else
                left += (descTail() - cachePnt);

            return left;
        }

        /* Return the number of descriptors used and not written back.
         */
        unsigned descUsed() const { return usedCache.size(); }

        /* Return the number of cache unused descriptors we have. */
        unsigned descUnused() const { return unusedCache.size(); }

        /* Get into a state where the descriptor address/head/etc colud be
         * changed */
        void reset();


        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;

        virtual bool hasOutstandingEvents() {
            return wbEvent.scheduled() || fetchEvent.scheduled();
        }

    };


    class RxDescCache : public DescCache<igbreg::RxDesc>
    {
      protected:
        Addr descBase() const override { 
          // return igbe->regs.rdba(); 
          return igbe->regs.rdba_array[queueID]();
        }
        long descHead() const override { 
          // return igbe->regs.rdh(); 
          return igbe->regs.rdh_array[queueID]();
        }
        long descLen() const override { 
          // return igbe->regs.rdlen() >> 4; 
          return igbe->regs.rdlen_array[queueID]() >> 4;
        }
        long descTail() const override { 
          // return igbe->regs.rdt(); 
          return igbe->regs.rdt_array[queueID]();
        }
        void updateHead(long h) override { 
          // igbe->regs.rdh(h); 
          igbe->regs.rdh_array[queueID](h);
        }
        void enableSm() override;
        void fetchAfterWb() override {
            if (!igbe->rxTick && igbe->drainState() == DrainState::Running)
                fetchDescriptors();
        }

        bool pktDone;

        /** Variable to head with header/data completion events */
        int splitCount;

        /** Bytes of packet that have been copied, so we know when to
            set EOP */
        unsigned bytesCopied;

        int queueID; // Queue ID for this cache

      public:
        RxDescCache(IGbE *i, std::string n, int s, int qid);

        /** Write the given packet into the buffer(s) pointed to by the
         * descriptor and update the book keeping. Should only be called when
         * there are no dma's pending.
         * @param packet ethernet packet to write
         * @param pkt_offset bytes already copied from the packet to memory
         * @return pkt_offset + number of bytes copied during this call
         */
        int writePacket(EthPacketPtr packet, int pkt_offset);

        /** Called by event when dma to write packet is completed
         */
        void pktComplete();

        /** Check if the dma on the packet has completed and RX state machine
         * can continue
         */
        bool packetDone();

        void unsetPacketDone() { pktDone = false; }

        EventFunctionWrapper pktEvent;

        // Event to handle issuing header and data write at the same time
        // and only callking pktComplete() when both are completed
        void pktSplitDone();
        EventFunctionWrapper pktHdrEvent;
        EventFunctionWrapper pktDataEvent;

        // Event and function to deal with RDTR timer expiring
        void _rdtrProcess() {
            writeback(0);
            DPRINTF(EthernetIntr,
                    "At RX[%d] Posting RXT interrupt because RDTR timer expired\n", queueID);
            igbe->postInterrupt(igbreg::IT_RXT);
        }
        EventFunctionWrapper _rdtrEvent;

        // Event and function to deal with RADV timer expiring
        void _radvProcess() {
            writeback(0);
            DPRINTF(EthernetIntr,
                    "At RX[%d] Posting RXT interrupt because RADV timer expired\n", queueID);
            igbe->postInterrupt(igbreg::IT_RXT);
        }
        EventFunctionWrapper _radvEvent;

        bool hasOutstandingEvents() override;

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };
    friend class RxDescCache;

    // RxDescCache rxDescCache;
    //jm - this should be multiple arrays
    RxDescCache *rxDescCacheArray[MAX_QUEUE_SIZE];

    class TxDescCache  : public DescCache<igbreg::TxDesc>
    {
      protected:
        Addr descBase() const override { 
          // return igbe->regs.tdba(); 
          return igbe->regs.tdba_array[queueID]();
        }
        long descHead() const override { 
          // return igbe->regs.tdh(); 
          return igbe->regs.tdh_array[queueID]();
        }
        long descTail() const override { 
          // return igbe->regs.tdt(); 
          return igbe->regs.tdt_array[queueID]();
        }
        long descLen() const override { 
          // return igbe->regs.tdlen() >> 4; 
          return igbe->regs.tdlen_array[queueID]() >> 4;
        }
        void updateHead(long h) override { 
          // igbe->regs.tdh(h); 
          igbe->regs.tdh_array[queueID](h);
        }
        void enableSm() override;
        void actionAfterWb() override;
        void fetchAfterWb() override {
            if (!igbe->txTick && igbe->drainState() == DrainState::Running)
                fetchDescriptors();
        }



        bool pktDone;
        bool isTcp;
        bool pktWaiting;
        bool pktMultiDesc;
        Addr completionAddress;
        bool completionEnabled;
        uint32_t descEnd;


        // tso variables
        bool useTso;
        Addr tsoHeaderLen;
        Addr tsoMss;
        Addr tsoTotalLen;
        Addr tsoUsedLen;
        Addr tsoPrevSeq;
        Addr tsoPktPayloadBytes;
        bool tsoLoadedHeader;
        bool tsoPktHasHeader;
        uint8_t tsoHeader[256];
        Addr tsoDescBytesUsed;
        Addr tsoCopyBytes;
        int tsoPkts;

        int queueID; // Queue ID for this cache

      public:
        TxDescCache(IGbE *i, std::string n, int s, int qid);

        /** Tell the cache to DMA a packet from main memory into its buffer and
         * return the size the of the packet to reserve space in tx fifo.
         * @return size of the packet
         */
        unsigned getPacketSize(EthPacketPtr p);
        void getPacketData(EthPacketPtr p);
        void processContextDesc();

        /** Return the number of dsecriptors in a cache block for threshold
         * operations.
         */
        unsigned
        descInBlock(unsigned num_desc)
        {
            return num_desc / igbe->cacheBlockSize() / sizeof(igbreg::TxDesc);
        }

        /** Ask if the packet has been transfered so the state machine can give
         * it to the fifo.
         * @return packet available in descriptor cache
         */
        bool packetAvailable();

        /**
         * set packetDone to false -> it is called after push the packet to the fifo
         * 
         */
        void unsetPacketDone() { pktDone = false; }

        /** Ask if we are still waiting for the packet to be transfered.
         * @return packet still in transit.
         */
        bool packetWaiting() { return pktWaiting; }

        /** Ask if this packet is composed of multiple descriptors
         * so even if we've got data, we need to wait for more before
         * we can send it out.
         * @return packet can't be sent out because it's a multi-descriptor
         * packet
         */
        bool packetMultiDesc() { return pktMultiDesc;}

        /** Called by event when dma to write packet is completed
         */
        void pktComplete();
        EventFunctionWrapper pktEvent;

        void headerComplete();
        EventFunctionWrapper headerEvent;


        void completionWriteback(Addr a, bool enabled) {
            DPRINTF(EthernetDesc,
                    "Completion writeback Addr: %#x enabled: %d\n",
                    a, enabled);
            completionAddress = a;
            completionEnabled = enabled;
        }

        bool hasOutstandingEvents() override;

        void nullCallback() {
            DPRINTF(EthernetDesc, "Completion writeback complete\n");
            igbe->etherDeviceStats.metaDMABytes += 4; // write 4 bytes (descEnd)
        }
        EventFunctionWrapper nullEvent;

        void _tadvProcess() {
            writeback(0);
            DPRINTF(EthernetIntr,
                    "At TX[%d] Posting TXDW interrupt because TADV timer expired\n", queueID);
            igbe->postInterrupt(igbreg::IT_TXDW);
        }
        EventFunctionWrapper _tadvEvent;

        void _tidvProcess() {
            writeback(0);
            DPRINTF(EthernetIntr,
                    "At TX[%d] Posting TXDW interrupt because TIDV timer expired\n", queueID);
            igbe->postInterrupt(igbreg::IT_TXDW);
        }
        EventFunctionWrapper _tidvEvent;                     

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };

    friend class TxDescCache;

    // TxDescCache txDescCache;
    //jm - this should be multiple arrays
    TxDescCache *txDescCacheArray[MAX_QUEUE_SIZE];

    class RxM2funcContext : public Serializable
    {
      protected:
        IGbE *igbe;
        int queueID;
        int rxContextFifoSize;
        std::string _name;

        bool enableDTA; // enable Data Transfer Accelerator mode or not. If use DTA, the RD request is coming from DTA, not from host. And use CXLReqBuffer to store the RD request and use the request to make response

        /** To handle the case when the ethernet packet size is larger than CXL flit size
         * and the packet is split into multiple flits. It indicates whether the packet is split or not
         */
        bool remainPacket;

        /** Bytes of packet that have been sent to the host */
        unsigned bytesSent;

        /** Variable to head with header/data completion events */
        int splitCount;        

        /** Number of rx packet received */
        uint64_t rxCount;   

        struct m2funcRxFifoEntry {
          uint64_t rxCount;
          uint64_t packetLength; // Bytes - this includes the dummy bytes to align the packet size to flit size
          uint64_t descLength;
          uint64_t dataLength;
          uint8_t * packetData;
        };
        // M2func Rx Queue - store the packet, that is received from Ethernet and processed by NIC
        std::deque<m2funcRxFifoEntry> m2funcRxFifo;  

        // M2func CXL Request Buffer - store the packet, that is received from the host part and waiting for the response
        // TODO - JM : serialize/unserialize
        std::deque<PacketPtr> m2funcCXLReqBuf;
        uint64_t numCXLReq; // Number of CXL request buffer (accummulated)
        uint64_t numFreeCXLReq; // Number of free CXL request buffer
        uint64_t numFreeCXLReqMax; // Maximum number of free CXL request buffer

        // For stat
        Tick lastRxM2funcReadTick; // last tick when the M2func RD request is comming from host
        

      public:
        RxM2funcContext(IGbE *i, std::string n, int _rxContextFifoSize, int qid, bool _enableDTA, int _cxlReqBufSize);
        ~RxM2funcContext();
        std::string name() { return _name; }

        // Check m2funcRxFifo size is full or not
        bool m2funcRxFifoFull() { return m2funcRxFifo.size() >= rxContextFifoSize; }
        // it corresponds to writePacket in RxDescCache
        void processRxPacket(EthPacketPtr packet); 
        bool rxM2funcStateMachine();
        void readM2funcPacket(PacketPtr pkt); // receive packet from host and make response 

        // DTA
        bool isDTAEnabled() { return enableDTA; }
        bool m2funcCXLReqBufFull() { return m2funcCXLReqBuf.size() >= numFreeCXLReqMax; }
        bool isCXLReqBufEmpty() { return m2funcCXLReqBuf.empty(); }
        // Push new CXL RD request to m2funcCXLReqBuf
        bool pushCXLReqBuf(PacketPtr pkt);
        // Pop the CXL RD request from m2funcCXLReqBuf
        PacketPtr popCXLReqBuf();
        // send the response to the host
        void sendCXLResp(PacketPtr pkt); // This can be tricky. The PIO port is working in atomic mode. So, we need to change the PIO port to non-atomic mode to send the response to the host
        

        void updateRxM2funcReadStat(Tick tick) { 
          if (lastRxM2funcReadTick != 0) {
            Tick diff = tick - lastRxM2funcReadTick;
            igbe->etherDeviceStats.rxM2funcReadDistance.sample(diff);
          }

          lastRxM2funcReadTick = tick;
        }

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;    
    };


    class TxM2funcContext : public Serializable
    {
      protected:
        IGbE *igbe;
        int queueID;
        int txContextFifoSize;
        std::string _name;

        bool enableDTA; // enable Data Transfer Accelerator mode or not. If use DTA, the WR request is coming from DTA, not from host.

        /** Set when the packet is ready to be transmitted through txFifo */
        bool pktDone;
        /** The ethernet packet size that is set within TX descriptor, currently processing */
        uint64_t ethPktSize;
        /** The TX descriptor, currently processing */
        uint64_t txDesc;
        /** The received packet size. We have to connect received packet from host until the length of ethPktSize to send through txFifo */
        uint64_t receivedPktSize;

        /** descriptor size (Bytes) of TX */
        size_t descSize;

        bool isTcp; // ?
        bool pktWaiting; // ?
        bool pktMultiDesc; // ?

        // Make struct for tso variables
        struct TSOEntry {
          bool tsoEnabled;
          bool txPktExists;
          uint32_t mss; //Maximum Segment Size
          uint32_t headerLen; //header len (10 bits)
          uint32_t totalLen; //paylen
          //below is control variables used in NIC
          //set after header packet is sent
          bool loadedHeader;
          //set after header is included in ethpacket
          bool ethPacketHasHeader;
          //updated when each packet is sent
          uint32_t usedLen;
          uint32_t prevSeq;
          int tsoPkts;
        };
        TSOEntry tsoEntry;

        /** Number of tx packet sent */
        uint64_t txCount;

        struct m2funcTxFifoEntry {
          uint64_t txCount;
          uint64_t packetLength; // Bytes
          uint8_t * packetData;
        };

        // Store the packet, that is from host and this will be processed to make it as Ethernet packet, and then push to txFifo
        std::deque<m2funcTxFifoEntry> m2funcTxFifo;

        // bitmask - indicating the packet success/fail to send
        static const int bitmask_expand_factor = 2; // expansion factor for bitmask
        static const int bitmask_window_size_bits = 32; // size of window for each read request (32-bit)
        static const int initial_bitmask_size_bytes = 64; // initial size of bitmask in bytes (64 bytes)
        
        struct bitmaskWrapper {
          uint8_t * bitmask; // bitmask
          uint64_t bitmaskSize; // size of bitmask in bits
          uint64_t updateCursor; // Cursor for update the bitmask
          uint64_t readCursor; // Cursor for read the bitmask
        };
        bitmaskWrapper bitmaskWrap;

      public:
        TxM2funcContext(IGbE *i, std::string n, int _txContextFifoSize, int qid, bool _enableDTA);
        ~TxM2funcContext();
        std::string name() { return _name; }

        // Check m2funcTxFifo size is full or not
        bool m2funcTxFifoFull() { return m2funcTxFifo.size() >= txContextFifoSize; }
        void processTxPacket(EthPacketPtr ethpkt, int dataSize, uint8_t* data, bool isHeader, bool ixsm, bool txsm);
        bool txM2funcStateMachine();
        void writeM2funcPacket(PacketPtr pkt); // receive packet from host and push it to txCXLMemFifo

        bool ethPktDone() { return pktDone; }

        void expandBitmask();
        void updateBitmask(bool success);
        void readBitmask(PacketPtr pkt);

        // DTA
        bool isDTAEnabled() { return enableDTA; }

        void serialize(CheckpointOut &cp) const override;
        void unserialize(CheckpointIn &cp) override;
    };
    

  public:
    PARAMS(IGbE);

    RxM2funcContext *rxM2funcContextArray[MAX_QUEUE_SIZE];
    TxM2funcContext *txM2funcContextArray[MAX_QUEUE_SIZE];

    IGbE(const Params &params);
    ~IGbE();
    void init() override;

    Port &getPort(const std::string &if_name,
                  PortID idx=InvalidPortID) override;

    Tick lastInterrupt;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    Tick writeConfig(PacketPtr pkt) override;

    bool ethRxPkt(EthPacketPtr packet);
    void ethTxDone();

    Tick lastRxM2funcPacketComeTick; // last tick when the ethernet packet is ready to be sent to host
    Tick lastRxDMAStartTick; // last tick when the DMA start

    void updateRxPacketReceiveStat(Tick tick) { 
      if (lastRxM2funcPacketComeTick != 0) {
        Tick diff = tick - lastRxM2funcPacketComeTick;
        etherDeviceStats.rxPacketComeDistance.sample(diff);
      }

      lastRxM2funcPacketComeTick = tick;
    }
    
    void updateRxRingBufferDMAStartStat(Tick tick) {
      if (lastRxDMAStartTick != 0) { 
        Tick diff = tick - lastRxDMAStartTick;
        etherDeviceStats.rxRingBufferStartDMADistance.sample(diff);
      }

      lastRxDMAStartTick = tick;
    }

    int getNumQueues() { return numQueues; }

    void serialize(CheckpointOut &cp) const override;
    void unserialize(CheckpointIn &cp) override;

    /** Check if all the draining things that need to occur have occured and
     * handle the drain event if so.
     */
    void checkDrain();
    DrainState drain() override;
    void drainResume() override;
    // For M2func
    void enableSmTx();

};

class IGbEInt : public EtherInt
{
  private:
    IGbE *dev;

  public:
    IGbEInt(const std::string &name, IGbE *d)
        : EtherInt(name), dev(d)
    { }

    virtual bool recvPacket(EthPacketPtr pkt) { return dev->ethRxPkt(pkt); }
    virtual void sendDone() { dev->ethTxDone(); }
};

// JM
class M2funcPort : public QueuedResponsePort
{
  private:
    IGbE *dev;
    RespPacketQueue _respQueue;
    Tick cxlMemDelay;
    AddrRangeList addrRanges;
    int numQueues;
  protected:
    void recvFunctional(PacketPtr pkt) override;
    bool recvTimingReq(PacketPtr pkt) override; 
    Tick recvAtomic(PacketPtr pkt) override;

  public:
    M2funcPort(const std::string &_name, IGbE *_owner, Tick _delay, int _numQueues)
        : QueuedResponsePort(_name, _owner, respQueue), _respQueue(*_owner, *this), cxlMemDelay(_delay), dev(_owner), numQueues(_numQueues) { setAddrRange(); }

    void setAddrRange() { 
      const Addr base_addr = dev->getBARBaseAddr(0);
      for (int i = 0; i < numQueues; i++) {
        printf("M2funcPort::setAddrRange - i: %d\n", i);
        printf("M2funcPort::setAddrRange - base_addr: %ld\n", base_addr);
        Addr rxM2funcDTAAddr = igbreg::E1000_RXDTA(i) + base_addr;
        Addr txM2funcDTAAddr = igbreg::E1000_TXDTA(i) + base_addr;
        printf("M2funcPort::setAddrRange - rxM2funcDTAAddr: %ld\n", rxM2funcDTAAddr);
        printf("M2funcPort::setAddrRange - txM2funcDTAAddr: %ld\n", txM2funcDTAAddr);
        Addr size = 64; // CXL flit size
        addrRanges.push_back(RangeSize(rxM2funcDTAAddr, size));
        addrRanges.push_back(RangeSize(txM2funcDTAAddr, size));
      }
    }
    AddrRangeList getAddrRanges() const override { return addrRanges; }

};


} // namespace gem5

#endif //__DEV_NET_I8254XGBE_HH__
