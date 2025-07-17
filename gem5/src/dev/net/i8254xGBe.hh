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

#define USE_ENSO
#ifdef USE_ENSO
#include <vector>
#include <array>
#include <cassert>

#define MAX_NB_APPS 1024
#define MAX_NB_FLOWS 8192
#define MAX_NB_MANAGER (MAX_QUEUE_SIZE / 2)     // Max RX Enso Pipe Manager number
#define MAX_NB_NOTIF MAX_NB_MANAGER // Max RX notification buffer number

#define ENSO_PIPE_SIZE 32768 // assume host enso pipe 2MB, 2048*1024B/64B = 32768, 64B flit gran
#define NOTIF_BUF_SIZE 16384 // assume host notif buf 1MB, 1024*1024B/64B = 16384, 64B notif gran
#define MAX_TX_TRANS 131072 // max TX data from host per notification, 128KB
#define NOTIF_SIZE 64
#define BASE_PKT_SIZE 64 // base flit size 64
#define NOTIF_BUF_MASK (NOTIF_BUF_SIZE - 1)
#define ENSO_PIPE_MASK (ENSO_PIPE_SIZE - 1)
#endif
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
    // jm - this should be multiple arrays
    #ifdef USE_ENSO
    EthPacketPtr txPacket;
    #endif
    EthPacketPtr txPacketArray[MAX_QUEUE_SIZE];

    // jm rx EthPacketPtr Array
    // Packet that is currently being put into the RX queue
    // If RSS is enabled, RSS is done and push to the rxPacketArray per cycle.
    // If target rxPacketArray is full, it cannot pop the packet from rxFifo

    EthPacketPtr rxPacketArray[MAX_QUEUE_SIZE];

    #ifdef USE_ENSO
    // using for RxEnsoPipeManager notify logic

    bool pipeHeadUpdated[MAX_NB_MANAGER];

    // DataFIFO
    EnsoPacketFifo EnsoRxFifo;
    bool ensoDataPush(EnsoRxPtr pkt) { return EnsoRxFifo.push(pkt); }

    // std::deque<EnsoRxPtr> EnsoRxFifo; 

    // bool fifoEmpty() { return EnsoRxFifo.empty(); }
    // void ensoDataPush(EnsoRxPtr pkt) { EnsoRxFifo.push_back(pkt); }
    // void dataPop() { EnsoRxFifo.pop_front(); }
    // void fifoClear() { EnsoRxFifo.clear(); }
    // unsigned fifoSize() { return EnsoRxFifo.size(); }

    void updateDropFSMEnso(int rxFifoFull, int ensoPipeFull);
    #endif


    // Should to Rx/Tx State machine tick?
    bool inTick;
    bool rxTick;
    bool txTick;
    bool txFifoTick;

    bool startLoadGen;

    enum class CommunicationType : uint8_t
    {
      RING = 0,
      M2FUNC = 1
    };
    CommunicationType commType = CommunicationType::RING;

    bool enableDTA = false;

    bool is_dpdk_setup_step;

    unsigned flitSize; // Flit size in bytes

    // jm - this should be multiple arrays
    #ifdef USE_ENSO
    bool rxDmaPacket;
    bool rxDmaNotif;
    #endif
    bool rxDmaPacketArray[MAX_QUEUE_SIZE];

    // jm - this should be multiple arrays
    //  Number of bytes copied from current RX packet
    // unsigned pktOffset;
    unsigned pktOffsetArray[MAX_QUEUE_SIZE];

    // Delays in managaging descriptors
    Tick fetchDelay, wbDelay;
    Tick fetchCompDelay, wbCompDelay;
    Tick rxWriteDelay, txReadDelay;

    // JM - delay for CXL access
    Tick cxlMemDelay;

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
    int prevRSSQueue; // It is for the case when RSS is enabled, but not ip, tcp, udp packet. just do round-robin
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

    template <class T>
    class DescCache
    {
    protected:
      virtual long descHead() const = 0;
      virtual long descTail() const = 0;

      // Pointer to the device we cache for
      IGbE *igbe;

      // Name of this  descriptor cache
      std::string _name;

      /** The packet that is currently being dmad to memory if any */
      EthPacketPtr pktPtr;

      /* Address DMA is currently at */
      Addr dmaAddr;

      // JM
      bool isRx;

      // DMA engine idx within DescCacheGlobal
      int dmaEngineIdx;

      /** Shortcut for DMA address translation */
      Addr pciToDma(Addr a) { return igbe->pciToDma(a); }

    public:
      DescCache(IGbE *i, const std::string n, bool _isRx, int engineIdx);
      virtual ~DescCache();

      std::string name() { return _name; }

      virtual bool hasOutstandingEvents()
      {
        return false;
      }
    };

    class DescDMAEngine
    {
    private:
      // Parent DescCacheGlobal
      void *parent;
      // Desc DMA engine idx
      int descDMAEngineIdx;
      // name
      std::string _name;
      // Pointer to the device we cache for
      IGbE *igbe;
      // Start of the address
      Addr descBase;
      // Length of the DMA
      Addr descLen;
      // rd or wr
      bool isRead;
      // number of descriptors currently being processed
      int numDescDMAing;

      void *fetchBuf;
      void *wbBuf;

      bool isRx;

      bool isDescFetchEngine;
      bool isDescWbEngine;

    public:
      DescDMAEngine(void *p, const std::string n, int idx, IGbE *i, bool _isRx,
                    bool _isDescFetchEngine, bool _isDescWbEngine);
      ~DescDMAEngine();
      bool isDescFetch() { return isDescFetchEngine; }
      bool isDescWb() { return isDescWbEngine; }
      bool isWorking() { return numDescDMAing > 0; }
      Addr getDescBase() { return descBase; }
      Addr getDescLen() { return descLen; }
      int getDescDMAEngineIdx() { return descDMAEngineIdx; }

      void writebackRegister(Addr _descBase, Addr _descLen, int numDesc, void *_wbBuf);
      void writebackRegister1();
      EventFunctionWrapper wbDelayEvent;
      void writebackComplete();
      EventFunctionWrapper wbDMAEvent;

      void fetchRegister(Addr _descBase, Addr _descLen, int numDesc, void *_wbBuf);
      void fetchRegister1();
      EventFunctionWrapper fetchDelayEvent;
      void fetchComplete();
      EventFunctionWrapper fetchDMAEvent;

      void parentsPrinter(std::string printStr);

      bool hasOutstandingEvents()
      {
        return wbDMAEvent.scheduled() || fetchDMAEvent.scheduled();
      }
    };

    template <class T>
    class DescCacheGlobal : public Serializable
    { // Global descriptor cache - has DescCache objects inside to do DMA in a parallel way
    protected:
      virtual Addr descBase() const = 0;
      virtual long descHead() const = 0;
      virtual long descTail() const = 0;
      virtual long descLen() const = 0;
      virtual void updateHead(long h) = 0;
      virtual void enableSm() = 0;
      virtual void actionAfterWb() {}
      virtual void fetchAfterWb() = 0;
      virtual std::string wbBufToString(int idx) = 0;
      virtual std::string wbBufArrayToString(int engineIdx, int idx) = 0;
      virtual std::string fetchBufToString(T *desc) = 0;

      typedef std::deque<T *> CacheType;
      CacheType usedCache;
      CacheType unusedCache;
      uint64_t nextSeqToWriteback = 0;
      uint64_t nextSeqToProcess = 0;
      struct ProcessingCacheEntry
      {
        T *desc;
        uint64_t seqIdx;
        bool done;
      };
      ProcessingCacheEntry processingCache[MAX_DMA_ENGINE_SIZE]; // Cache for processing descriptors, that are DMAing data to memory by DMA engines.

      T *fetchBuf;
      T *wbBuf;

      EthPacketPtr processingPktArray[MAX_DMA_ENGINE_SIZE]; // Packet that is currently being DMAing data to memory by DMA engines.

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

      // JM
      bool isRx;
      int numDMAEngines = 0;     // Number of mbuf DMA engines
      int numDescDMAEngines = 0; // Number of desc DMA engines
      // for desc dma engines
      DescDMAEngine *descFetchDMAEngines[MAX_DMA_ENGINE_SIZE];
      DescDMAEngine *descWbDMAEngines[MAX_DMA_ENGINE_SIZE];
      bool descFetchDMAEngineWorking[MAX_DMA_ENGINE_SIZE]; // true if the desc DMA engine is working, false if it is idle
      bool descWbDMAEngineWorking[MAX_DMA_ENGINE_SIZE];    // true if the desc DMA engine is working, false if it is idle
      bool descFetchDMAEngineDone[MAX_DMA_ENGINE_SIZE];    // true if the desc DMA engine has completed the fetch, false if it is still working
      bool descWbDMAEngineDone[MAX_DMA_ENGINE_SIZE];       // true if the desc DMA engine has completed the writeback, false if it is still working
      // for fetch dma tracking
      std::deque<int> fetchDMAJobQueue;          // check this queue when fetchCompleteGlobal is called. // use this queue to update in-order completion
      int curFetchDMAPnt;                        // it is the pointer of the ring buffer, considering in-flight fetch requests
      int curFetchingNum;                        // the number of descriptors that are currently being fetched
      int descFetchBaseIdx[MAX_DMA_ENGINE_SIZE]; // the base index of the descriptor ring for each desc DMA engine
      int descFetchNum[MAX_DMA_ENGINE_SIZE];     // the number of descriptors for each desc DMA engine
      // array of fetch buffers for each desc DMA engine
      typedef T *FetchBufPtr;
      FetchBufPtr fetchBufArray[MAX_DMA_ENGINE_SIZE];

      std::deque<int> wbDMAJobQueue;          // check this queue when wbCompleteGlobal is called. // use this queue to update in-order completion
      int curWbDMAPnt;                        // it is the pointer of the ring buffer, considering in-flight writeback requests
      int curWbingNum;                        // the number of descriptors that are currently being written back
      int descWbBaseIdx[MAX_DMA_ENGINE_SIZE]; // the base index of the descriptor ring for each desc DMA engine
      int descWbNum[MAX_DMA_ENGINE_SIZE];     // the number of descriptors for each desc DMA engine
      // array of writeback buffers for each desc DMA engine
      typedef T *WbBufPtr;
      WbBufPtr wbBufArray[MAX_DMA_ENGINE_SIZE];

      /** Shortcut for DMA address translation */
      Addr pciToDma(Addr a) { return igbe->pciToDma(a); }

    public:
      /** Annotate sm*/
      std::string annSmFetch, annSmWb, annUnusedDescQ, annUsedCacheQ,
          annUsedDescQ, annUnusedCacheQ, annDescQ;

      DescCacheGlobal(IGbE *i, const std::string n, int s, bool _isRx, int _numDMAEngines, int _numDescDMAEngines);
      virtual ~DescCacheGlobal();

      std::string name() { return _name; }

      /** If the address/len/head change when we've got descriptors that are
       * dirty that is very bad. This function checks that we don't and if we
       * do panics.
       */
      void areaChanged();

      bool hasFreeWbDMAEngine();
      int getFreeWbDMAEngine();
      bool hasFreeFetchDMAEngine();
      int getFreeFetchDMAEngine();

      void allocFetchDMAEngineWorking(int idx)
      {
        assert(idx < numDescDMAEngines);
        assert(!descFetchDMAEngineWorking[idx]);
        descFetchDMAEngineWorking[idx] = true;
      }
      void allocWbDMAEngineWorking(int idx)
      {
        assert(idx < numDescDMAEngines);
        assert(!descWbDMAEngineWorking[idx]);
        descWbDMAEngineWorking[idx] = true;
      }
      void freeFetchDMAEngineWorking(int idx)
      {
        assert(idx < numDescDMAEngines);
        assert(descFetchDMAEngineWorking[idx]);
        descFetchDMAEngineWorking[idx] = false;
      }
      void freeWbDMAEngineWorking(int idx)
      {
        assert(idx < numDescDMAEngines);
        assert(descWbDMAEngineWorking[idx]);
        descWbDMAEngineWorking[idx] = false;
      }
      void setFetchDMAEngineDone(int idx)
      {
        assert(idx < numDescDMAEngines);
        assert(descFetchDMAEngineWorking[idx]);
        assert(!descFetchDMAEngineDone[idx]);
        descFetchDMAEngineDone[idx] = true;
      }
      void setWbDMAEngineDone(int idx)
      {
        assert(idx < numDescDMAEngines);
        assert(descWbDMAEngineWorking[idx]);
        assert(!descWbDMAEngineDone[idx]);
        descWbDMAEngineDone[idx] = true;
      }
      void clearFetchDMAEngineDone(int idx)
      {
        assert(idx < numDescDMAEngines);
        descFetchDMAEngineDone[idx] = false;
      }
      void clearWbDMAEngineDone(int idx)
      {
        assert(idx < numDescDMAEngines);
        descWbDMAEngineDone[idx] = false;
      }
      bool isFetchDMAEngineDone(int idx)
      {
        assert(idx < numDescDMAEngines);
        assert(descFetchDMAEngineWorking[idx]);
        return descFetchDMAEngineDone[idx];
      }
      bool isWbDMAEngineDone(int idx)
      {
        assert(idx < numDescDMAEngines);
        assert(descWbDMAEngineWorking[idx]);
        return descWbDMAEngineDone[idx];
      }

      void printer(std::string str)
      {
        DPRINTF(EthernetDesc, "%s\n", str);
      }

      void writebackGlobal(Addr aMask);
      void fetchDescriptorsGlobal();
      // This is called by the DMA engine when it has completed a descriptor
      void wbCompleteGlobal(int dmaEngineIdx);
      void fetchCompleteGlobal(int dmaEngineIdx);

      /* Return the number of !NULL desc in processingCache */
      // This should be counted as unused descriptors
      unsigned descProcessing() const
      {
        unsigned processing = 0;
        for (int i = 0; i < numDMAEngines; i++)
        {
          if (processingCache[i].desc != NULL)
          {
            processing++;
          }
        }
        return processing;
      }

      /* Return the number of descriptors left in the ring, so the device has
       * a way to figure out if it needs to interrupt.
       */
      unsigned
      descLeft() const
      {
        unsigned left = unusedCache.size() + descProcessing();
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
      unsigned descUnused() const { return unusedCache.size() + descProcessing(); }

      /* Get into a state where the descriptor address/head/etc colud be
       * changed */
      void reset();

      void serialize(CheckpointOut &cp) const override;
      void unserialize(CheckpointIn &cp) override;

      virtual bool hasOutstandingEvents()
      {
        return false;
      }
    };

    class RxDescCacheGlobal;
    class RxDescCache : public DescCache<igbreg::RxDesc>
    {
    protected:
      long descHead() const override
      {
        // return igbe->regs.rdh();
        return igbe->regs.rdh_array[queueID]();
      }
      long descTail() const override
      {
        // return igbe->regs.rdt();
        return igbe->regs.rdt_array[queueID]();
      }

      bool pktWaiting;

      igbreg::RxDesc *correspondingDesc; // Corresponding descriptor to use during DMA

      /** Variable to head with header/data completion events */
      int splitCount;

      /** Bytes of packet that have been copied, so we know when to
          set EOP */
      unsigned bytesCopied;

      int queueID; // Queue ID for this cache

      RxDescCacheGlobal *parent;

    public:
      RxDescCache(IGbE *i, std::string n, int qid, int engineIdx, RxDescCacheGlobal *_parent);

      /** Write the given packet into the buffer(s) pointed to by the
       * descriptor and update the book keeping. Should only be called when
       * there are no dma's pending.
       * @param packet ethernet packet to write
       * @param pkt_offset bytes already copied from the packet to memory
       * @param rxDesc the descriptor to write the packet into
       * @return pkt_offset + number of bytes copied during this call
       */
      int writePacket(EthPacketPtr packet, int pkt_offset, igbreg::RxDesc *rxDesc);

      /** Called by event when dma to write packet is completed
       */
      void pktComplete();

      // DMA engine manage functions
      bool packetWaiting() { return pktWaiting; }
      unsigned getBytesCopied() { return bytesCopied; }
      void setBytesCopied(unsigned bytes) { bytesCopied = bytes; }

      EventFunctionWrapper pktEvent;

      // Event to handle issuing header and data write at the same time
      // and only callking pktComplete() when both are completed
      void pktSplitDone();
      EventFunctionWrapper pktHdrEvent;
      EventFunctionWrapper pktDataEvent;

      bool hasOutstandingEvents() override;
    };
    friend class RxDescCache;

    class RxDescCacheGlobal : public DescCacheGlobal<igbreg::RxDesc>
    {
    protected:
      Addr descBase() const override
      {
        // return igbe->regs.rdba();
        return igbe->regs.rdba_array[queueID]();
      }
      long descHead() const override
      {
        // return igbe->regs.rdh();
        return igbe->regs.rdh_array[queueID]();
      }
      long descLen() const override
      {
        // return igbe->regs.rdlen() >> 4;
        return igbe->regs.rdlen_array[queueID]() >> 4;
      }
      long descTail() const override
      {
        // return igbe->regs.rdt();
        return igbe->regs.rdt_array[queueID]();
      }
      void updateHead(long h) override
      {
        // igbe->regs.rdh(h);
        igbe->regs.rdh_array[queueID](h);
      }
      void enableSm() override;
      void fetchAfterWb() override
      {
        if (!igbe->rxTick && igbe->drainState() == DrainState::Running)
          fetchDescriptorsGlobal();
      }
      std::string wbBufToString(int idx) override
      {
        // return the string of the wbBuf[idx] with igbreg::RxDesc format
        igbreg::RxDesc *desc;
        desc = wbBuf + idx;
        std::string descStr = "RxDescWb_pktlen: ";
        descStr += csprintf("%u", desc->adv_wb.pkt_len);
        return descStr;
      }
      std::string wbBufArrayToString(int engineIdx, int idx) override
      {
        // return the string of the wbBufArray[engineIdx][idx] with igbreg::RxDesc format
        igbreg::RxDesc *desc;
        desc = wbBufArray[engineIdx] + idx;
        std::string descStr = "RxDescWb_pktlen: ";
        descStr += csprintf("%u", desc->adv_wb.pkt_len);
        return descStr;
      }
      std::string fetchBufToString(igbreg::RxDesc *desc) override
      {
        std::string descStr = "RxDescFetch_addr: ";
        descStr += csprintf("%#x", desc->adv_read.pkt);
        return descStr;
      }

      int queueID; // Queue ID for this cache
      RxDescCache *dmaEngineArray[MAX_DMA_ENGINE_SIZE];
      unsigned processingPktOffsetArray[MAX_DMA_ENGINE_SIZE]; // Number of bytes copied from current RX packet for each packet
      bool processingPktDoneArray[MAX_DMA_ENGINE_SIZE];       // Flag to indicate if the packet is done for each DMA engine
    public:
      RxDescCacheGlobal(IGbE *i, std::string n, int s, int qid, int _numDMAEngines, int _numDescDMAEngines);

      /** Write the given packet into the buffer(s) pointed to by the
       * descriptor and update the book keeping. Should only be called when
       * there are no dma's pending.
       * @param success or not
       */
      bool writePacketGlobal(EthPacketPtr packet);

      // This function will get the information from the DMA engine & get the RX descriptor to write the descriptor back
      void onDMAComplete(int engineIdx);
      int hasReadyEthPacket();
      void clearDoneEthPacket(int engineIdx);

      bool packetWaitingGlobal()
      {
        // Return true if any of the DMA engines has a packet waiting
        bool waiting = false;
        for (int i = 0; i < numDMAEngines; i++)
        {
          waiting = waiting || dmaEngineArray[i]->packetWaiting();
        }
        return waiting;
      }

      bool hasFreeDMAEngine()
      {
        // Return true if any of the DMA engines has a free DMA engine
        for (int i = 0; i < numDMAEngines; i++)
        {
          if (!dmaEngineArray[i]->packetWaiting() && processingPktDoneArray[i] == false && processingCache[i].desc == NULL && processingPktOffsetArray[i] == 0 && processingPktArray[i] == NULL)
          {
            // free DMA engine: no packet waiting and also, no packet available (if packet is available & not waiting, it is waiting for push to the txFifo)
            return true;
          }
        }
        return false;
      }

      int getFreeDMAEngine()
      {
        for (int i = 0; i < numDMAEngines; i++)
        {
          if (!dmaEngineArray[i]->packetWaiting() && processingPktDoneArray[i] == false && processingCache[i].desc == NULL && processingPktOffsetArray[i] == 0 && processingPktArray[i] == NULL)
          {
            return i;
          }
        }
        return -1;
      }

      // manage processing descriptors in the processingCache to push to usedCache regarding the sequence index
      void manageUsedCache();

      // Event and function to deal with RDTR timer expiring
      void _rdtrProcess()
      {
        writebackGlobal(0);
        DPRINTF(EthernetIntr,
                "At RX[%d] Posting RXT interrupt because RDTR timer expired\n", queueID);
        igbe->postInterrupt(igbreg::IT_RXT);
      }
      EventFunctionWrapper _rdtrEvent;

      // Event and function to deal with RADV timer expiring
      void _radvProcess()
      {
        writebackGlobal(0);
        DPRINTF(EthernetIntr,
                "At RX[%d] Posting RXT interrupt because RADV timer expired\n", queueID);
        igbe->postInterrupt(igbreg::IT_RXT);
      }
      EventFunctionWrapper _radvEvent;

      bool hasOutstandingEvents() override;

      void serialize(CheckpointOut &cp) const override;
      void unserialize(CheckpointIn &cp) override;
    };
    friend class RxDescCacheGlobal;

    // RxDescCache rxDescCache;
    // jm - this should be multiple arrays
    RxDescCacheGlobal *rxDescCacheArray[MAX_QUEUE_SIZE];

    class TxDescCacheGlobal;
    class TxDescCache : public DescCache<igbreg::TxDesc>
    {
    protected:
      long descHead() const override
      {
        // return igbe->regs.tdh();
        return igbe->regs.tdh_array[queueID]();
      }
      long descTail() const override
      {
        // return igbe->regs.tdt();
        return igbe->regs.tdt_array[queueID]();
      }

      bool pktDone;
      bool pktWaiting;

      int queueID; // Queue ID for this cache

      TxDescCacheGlobal *parent;

    public:
      TxDescCache(IGbE *i, std::string n, int qid, int engineIdx, TxDescCacheGlobal *_parent);

      /** Tell the cache to DMA a packet from main memory into its buffer and
       * return the size the of the packet to reserve space in tx fifo.
       * @return size of the packet
       */
      unsigned getPacketSize(EthPacketPtr p);
      void getPacketData(EthPacketPtr p, Addr addr, int size);
      void processContextDesc();

      // manage functions
      void setPktDone(bool flag) { pktDone = flag; }
      void setPktWaiting(bool flag) { pktWaiting = flag; }
      void unsetPacketDone() { pktDone = false; }
      bool packetWaiting() { return pktWaiting; }

      /** Ask if the packet has been transfered so the state machine can give
       * it to the fifo.
       * @return packet available in descriptor cache
       */
      bool packetAvailable();

      /** Called by event when dma to write packet is completed
       */
      void pktComplete();
      EventFunctionWrapper pktEvent;

      bool hasOutstandingEvents() override;
    };

    friend class TxDescCache;

    class TxDescCacheGlobal : public DescCacheGlobal<igbreg::TxDesc>
    {
    protected:
      Addr descBase() const override
      {
        // return igbe->regs.tdba();
        return igbe->regs.tdba_array[queueID]();
      }
      long descHead() const override
      {
        // return igbe->regs.tdh();
        return igbe->regs.tdh_array[queueID]();
      }
      long descTail() const override
      {
        // return igbe->regs.tdt();
        return igbe->regs.tdt_array[queueID]();
      }
      long descLen() const override
      {
        // return igbe->regs.tdlen() >> 4;
        return igbe->regs.tdlen_array[queueID]() >> 4;
      }
      void updateHead(long h) override
      {
        // igbe->regs.tdh(h);
        igbe->regs.tdh_array[queueID](h);
      }
      void enableSm() override;
      void actionAfterWb() override;
      void fetchAfterWb() override
      {
        if (!igbe->txTick && igbe->drainState() == DrainState::Running)
          fetchDescriptorsGlobal();
      }
      std::string wbBufToString(int idx) override;
      std::string wbBufArrayToString(int engineIdx, int idx) override;
      std::string fetchBufToString(igbreg::TxDesc *desc) override;

      bool isTcp;
      bool pktHdrWaiting; // If use TSO, this is used to check if the header is loading or loaded
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
      int tsoDMAEngineIdx; // DMA engine index for doing TSO

      int queueID; // Queue ID for this cache
      TxDescCache *dmaEngineArray[MAX_DMA_ENGINE_SIZE];

      int roundRobinIdx; // Round-robin index for selecting DMA engine

    public:
      TxDescCacheGlobal(IGbE *i, std::string n, int s, int qid, int _numDMAEngines, int _numDescDMAEngines);

      /** Tell the cache to DMA a packet from main memory into its buffer and
       * return the size the of the packet to reserve space in tx fifo.
       * @return size of the packet
       */
      unsigned getPacketSize();
      void getPacketDataGlobal();
      void processContextDesc();
      void onDMAComplete(int engineIdx);
      bool hasReadyEthPacket();         // Check if there is a packet that is ready to be sent to the txFifo
      EthPacketPtr getReadyEthPacket(); // Get the packet that is ready to be sent to the txFifo

      // manage processing descriptors in the processingCache to push to usedCache regarding the sequence index
      void manageUsedCache();

      /** Return the number of dsecriptors in a cache block for threshold
       * operations.
       */
      unsigned
      descInBlock(unsigned num_desc)
      {
        return num_desc / igbe->cacheBlockSize() / sizeof(igbreg::TxDesc);
      }

      /** Ask if we are still waiting for the packet to be transfered.
       * @return packet still in transit.
       */
      bool packetWaitingGlobal()
      {
        // Return true if any of the DMA engines has a packet waiting
        bool waiting = false;
        for (int i = 0; i < numDMAEngines; i++)
        {
          waiting = waiting || dmaEngineArray[i]->packetWaiting();
        }
        return waiting;
      }

      bool packetHdrWaiting()
      {
        // Return true if header DMA is waiting
        // This is meaningful when TSO is enabled
        return pktHdrWaiting;
      }

      bool hasFreeDMAEngine()
      {
        // Have to check packetWaiting() && pktDone() for all DMA engines
        if (!useTso || (useTso && tsoDMAEngineIdx == -1))
        {
          for (int i = 0; i < numDMAEngines; i++)
          {
            if (!dmaEngineArray[i]->packetWaiting() && !dmaEngineArray[i]->packetAvailable() && processingCache[i].desc == NULL)
            {
              // free DMA engine: no packet waiting and also, no packet available (if packet is available & not waiting, it is waiting for push to the txFifo)
              return true;
            }
          }
          return false;
        }
        else
        {
          // if use TSO, then check only TSO DMA engine
          return isTSODMAEngineFree();
        }
      }

      int getFreeDMAEngine()
      {
        for (int i = 0; i < numDMAEngines; i++)
        {
          if (!dmaEngineArray[i]->packetWaiting() && !dmaEngineArray[i]->packetAvailable() && processingCache[i].desc == NULL)
          {
            return i;
          }
        }
        return -1;
      }

      bool isTSODMAEngineFree()
      {
        // if tsoDMAEngineIdx is -1, then return hasFreeDMAEngine()
        // else, return !dmaEngineArray[tsoDMAEngineIdx]->packetWaiting()
        if (tsoDMAEngineIdx == -1)
        {
          return hasFreeDMAEngine();
        }
        else
        {
          assert(tsoDMAEngineIdx < numDMAEngines);
          return !dmaEngineArray[tsoDMAEngineIdx]->packetWaiting() && processingCache[tsoDMAEngineIdx].desc == NULL;
        }
      }

      void headerComplete();
      EventFunctionWrapper headerEvent;

      void completionWriteback(Addr a, bool enabled)
      {
        DPRINTF(EthernetDesc,
                "Completion writeback Addr: %#x enabled: %d\n",
                a, enabled);
        completionAddress = a;
        completionEnabled = enabled;
      }

      bool hasOutstandingEvents() override;

      void nullCallback()
      {
        DPRINTF(EthernetDesc, "Completion writeback complete\n");
        igbe->etherDeviceStats.metaDMABytes += 4; // write 4 bytes (descEnd)
      }
      EventFunctionWrapper nullEvent;

      void _tadvProcess()
      {
        writebackGlobal(0);
        DPRINTF(EthernetIntr,
                "At TX[%d] Posting TXDW interrupt because TADV timer expired\n", queueID);
        igbe->postInterrupt(igbreg::IT_TXDW);
      }
      EventFunctionWrapper _tadvEvent;

      void _tidvProcess()
      {
        writebackGlobal(0);
        DPRINTF(EthernetIntr,
                "At TX[%d] Posting TXDW interrupt because TIDV timer expired\n", queueID);
        igbe->postInterrupt(igbreg::IT_TXDW);
      }
      EventFunctionWrapper _tidvEvent;

      void serialize(CheckpointOut &cp) const override;
      void unserialize(CheckpointIn &cp) override;
    };

    friend class TxDescCacheGlobal;

    // TxDescCache txDescCache;
    // jm - this should be multiple arrays
    TxDescCacheGlobal *txDescCacheArray[MAX_QUEUE_SIZE];

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

      struct m2funcRxFifoEntry
      {
        uint64_t rxCount;
        uint64_t packetLength; // Bytes - this includes the dummy bytes to align the packet size to flit size
        uint64_t descLength;
        uint64_t dataLength;
        uint8_t *packetData;
      };
      // M2func Rx Queue - store the packet, that is received from Ethernet and processed by NIC
      std::deque<m2funcRxFifoEntry> m2funcRxFifo;

      // M2func CXL Request Buffer - store the packet, that is received from the host part and waiting for the response
      // TODO - JM : serialize/unserialize
      std::deque<PacketPtr> m2funcCXLReqBuf;
      uint64_t numCXLReq;        // Number of CXL request buffer (accummulated)
      uint64_t numFreeCXLReq;    // Number of free CXL request buffer
      uint64_t numFreeCXLReqMax; // Maximum number of free CXL request buffer

      // For stat
      Tick lastRxM2funcReadTick; // last tick when the M2func RD request is comming from host

      bool isLoadGenStarted = false;            // To check the LoadGen is started or not
      std::map<uint64_t, bool> dtaRXJobSuccess; // To check the DTA RX job is success or not - to determine zeroed response or not

    public:
      RxM2funcContext(IGbE *i, std::string n, int _rxContextFifoSize, int qid, bool _enableDTA, int _cxlReqBufSize);
      ~RxM2funcContext();
      std::string name() { return _name; }

      // Check m2funcRxFifo size is full or not
      bool m2funcRxFifoFull() { return m2funcRxFifo.size() >= rxContextFifoSize; }
      // it corresponds to writePacket in RxDescCache
      void processRxPacket(EthPacketPtr packet);
      bool rxM2funcStateMachine();
      void readM2funcPacket(PacketPtr pkt);   // receive packet from host and make response
      void makeZeroedResponse(PacketPtr pkt); // make zeroed response to the host

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

      void updateRxM2funcReadStat(Tick tick)
      {
        if (lastRxM2funcReadTick != 0)
        {
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

      bool isTcp;        // ?
      bool pktWaiting;   // ?
      bool pktMultiDesc; // ?

      // Make struct for tso variables
      struct TSOEntry
      {
        bool tsoEnabled;
        bool txPktExists;
        uint32_t mss;       // Maximum Segment Size
        uint32_t headerLen; // header len (10 bits)
        uint32_t totalLen;  // paylen
        // below is control variables used in NIC
        // set after header packet is sent
        bool loadedHeader;
        // set after header is included in ethpacket
        bool ethPacketHasHeader;
        // updated when each packet is sent
        uint32_t usedLen;
        uint32_t prevSeq;
        int tsoPkts;
      };
      TSOEntry tsoEntry;

      /** Number of tx packet sent */
      uint64_t txCount;

      struct m2funcTxFifoEntry
      {
        uint64_t txCount;
        uint64_t packetLength; // Bytes
        uint8_t *packetData;
      };

      // Store the packet, that is from host and this will be processed to make it as Ethernet packet, and then push to txFifo
      std::deque<m2funcTxFifoEntry> m2funcTxFifo;

      // bitmask - indicating the packet success/fail to send
      static const int bitmask_expand_factor = 2;       // expansion factor for bitmask
      static const int bitmask_window_size_bits = 32;   // size of window for each read request (32-bit)
      static const int initial_bitmask_size_bytes = 64; // initial size of bitmask in bytes (64 bytes)

      struct bitmaskWrapper
      {
        uint8_t *bitmask;      // bitmask
        uint64_t bitmaskSize;  // size of bitmask in bits
        uint64_t updateCursor; // Cursor for update the bitmask
        uint64_t readCursor;   // Cursor for read the bitmask
      };
      bitmaskWrapper bitmaskWrap;

    public:
      TxM2funcContext(IGbE *i, std::string n, int _txContextFifoSize, int qid, bool _enableDTA);
      ~TxM2funcContext();
      std::string name() { return _name; }

      // Check m2funcTxFifo size is full or not
      bool m2funcTxFifoFull() { return m2funcTxFifo.size() >= txContextFifoSize; }
      void processTxPacket(EthPacketPtr ethpkt, int dataSize, uint8_t *data, bool isHeader, bool ixsm, bool txsm);
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

#ifdef USE_ENSO

    
    void rxEnsoStateMachine();
    void txEnsoStateMachine();

    /*
        Per RX Enso Pipe Manager can manage MAX 512 Enso Pipe
        - maintain each Enso Pipe state,
        - determinig notify logic
            - Check host MMIO (in notify logic)

    */
    class EnsoPipeManager : public Serializable
    {
    protected:
      // associated with host (MMIO register)
      virtual Addr ensoPipeBase(int queueId) const = 0;
      virtual long ensoPipeHead(int queueId) const = 0;
      virtual long ensoPipeTail(int queueId) const = 0;
      virtual void updateHead(int queueId, long h) = 0;
      virtual void updateTail(int queueId, long t) = 0;

      // Enso NIC data FIFO enqueue/dequeue

      // Pointer to the device
      IGbE *igbe;

      // Name of this  enso pipe manager
      std::string _name;

      // The size of the EnsoPipe
      int size;

    public:
      EnsoPipeManager(IGbE *i, const std::string n, int s);
      virtual ~EnsoPipeManager();

      // check host enso pipe is full
      // considering enso pipe is ring buffer
      bool isEnsoPipeFull(size_t pktLength, int queueId)
      {
        assert(pktLength % igbe->flitSize == 0);
        // Read head and tail positions from MMIO
        uint32_t head = ensoPipeHead(queueId) & ENSO_PIPE_MASK;
        uint32_t tail = ensoPipeTail(queueId) & ENSO_PIPE_MASK;

        // Compute used space in ring buffer
        uint32_t used = (tail - head) & ENSO_PIPE_MASK;

        // Compute free space in ring buffer
        uint32_t freeSpace = ENSO_PIPE_SIZE - used - 1; // -1 to distinguish full vs empty

        // compute space current pkt need, 64B flit gran
        uint32_t neededSpace = pktLength / igbe->flitSize;

        // Return true if packet does not fit
        return neededSpace >= freeSpace;
      }

      std::string name() { return _name; }

      void serialize(CheckpointOut &cp) const override;
      void unserialize(CheckpointIn &cp) override;
    };

    /******************** RX Enso Pipe Manger ********************/

    class RXEnsoPipeManager : public EnsoPipeManager
    {
    protected:
      // EnsoPipeManager using half of the RDT/RDH/RDBA
      Addr ensoPipeBase(int queueId) const override { return igbe->regs.rdba_array[queueId + MAX_NB_MANAGER](); }
      long ensoPipeHead(int queueId) const override { return igbe->regs.rdh_array[queueId + MAX_NB_MANAGER](); }
      long ensoPipeTail(int queueId) const override { return igbe->regs.rdt_array[queueId + MAX_NB_MANAGER](); }
      void updateHead(int queueId, long h) override { igbe->regs.rdh_array[queueId + MAX_NB_MANAGER](h); }
      void updateTail(int queueId, long t) override { igbe->regs.rdt_array[queueId + MAX_NB_MANAGER](t); }

      // each host enso pipe state
      PipeState pipeStates[MAX_NB_MANAGER];

      int stateFull;

    public:
      RXEnsoPipeManager(IGbE *i, const std::string n, int s);

      // determinig notify logic
      // need to check ensoPipeTail is updated
      // only check when DataFIFO is not empty
      void onPktArrival(EnsoRxPtr pkt);
      void onRxUpdate(EnsoRxPtr pkt);

      // Data FIFO push/pop
      bool dataPush(EthPacketPtr pkt);

      // when host MMIO write enso pipe base addr, update PipeState
      void setRxPipeState(int queueId);

      // manage reg value
      void updateRxPipeHead(uint32_t h, int queueId){ updateHead(queueId, h); }
      void updateRxPipeTail(uint32_t t, int queueId){ updateTail(queueId, t); }

      PipeState* relinkDataFifoPipes(int queueId);

      // for DropFSM
      void setFull(int full){
        if(full)
          stateFull = 1;
        else  
          stateFull = 0;
      }
      int isFull() { return stateFull; }

      void serialize(CheckpointOut &cp) const override;
      void unserialize(CheckpointIn &cp) override;
    };
    friend RXEnsoPipeManager;
    RXEnsoPipeManager rxEnsoPipeManager;

    // Serializable for checkpoint
    template <class T>
    class NotifBufManager : public Serializable
    {
    protected:
      virtual Addr notifBufBase(int queueId) const = 0;
      virtual long notifBufHead(int queueId) const = 0;
      virtual long notifBufTail(int queueId) const = 0;
      virtual void updateHead(int queueId, long h) = 0;
      virtual void updateTail(int queueId, long t) = 0;
      virtual void enableSm() = 0;

      // Pointer to the device
      IGbE *igbe;

      // Name of this
      std::string _name;

      // The size of the notification buffer in NIC
      int size;

      
      

    public:
      NotifBufManager(IGbE *i, const std::string n, int s);
      virtual ~NotifBufManager();

      /** Shortcut for DMA address translation */
      Addr pciToDma(Addr a) { return igbe->pciToDma(a); }

      std::string name() { return _name; }

      // check host notification buffer is full
      // considering notification buffer is ring buffer
      bool isNotifBufFull(int queueId)
      {
        // Read head and tail positions from MMIO
        uint32_t head = notifBufHead(queueId) & NOTIF_BUF_MASK;
        uint32_t tail = notifBufTail(queueId) & NOTIF_BUF_MASK;

        // Compute used space in ring buffer
        uint32_t used = (tail - head) & NOTIF_BUF_MASK;

        // Compute free space in ring buffer
        uint32_t freeSpace = NOTIF_BUF_SIZE - used - 1; // -1 to distinguish full vs empty

        // compute space current pkt need, 64B flit gran
        // need to modify, there can be multiple notification ???
        uint32_t neededSpace = 1;

        // Return true if packet does not fit
        return neededSpace >= freeSpace;
      }

      // manage reg value
      void updateNotifHead(uint32_t h, int queueId){ updateHead(queueId, h); }
      void updateNotifTail(uint32_t t, int queueId){ updateTail(queueId, t); }

      virtual bool hasOutstandingEvents() = 0;

      void serialize(CheckpointOut &cp) const override;
      void unserialize(CheckpointIn &cp) override;
    };

    class RXNotifBufManager : public NotifBufManager<igbreg::RxNotification>
    {
    protected:
      Addr notifBufBase(int queueId) const override { return igbe->regs.rdba_array[queueId](); }
      long notifBufHead(int queueId) const override { return igbe->regs.rdh_array[queueId](); }
      long notifBufTail(int queueId) const override { return igbe->regs.rdt_array[queueId](); }
      void updateHead(int queueId, long h) override { igbe->regs.rdh_array[queueId](h); }
      void updateTail(int queueId, long t) override { igbe->regs.rdt_array[queueId](t); }

      void enableSm() override;

      /** Variable to head with packet/notification completion events */
      int pktSplitDone;
      int pktNotifDone;
      int pktSplitNotifDone;

      struct RXNotifState
      {
        uint64_t physAddr;
        uint64_t head;
        uint64_t tail;
      };
      RXNotifState RXNotifStates[MAX_NB_NOTIF];

      EnsoRxPtr pktPtr;

      bool pktDone;
      bool notifDone;

      // using when need to send rx notification
      igbreg::RxNotification *notifRxBuf;

      uint32_t dmaTail;
      uint32_t dmaQueueId;
      uint32_t dmaNotifId;
      

    public:
      RXNotifBufManager(IGbE *i, const std::string n, int s);
      // DMA packet & notification
      void writePacket(EnsoRxPtr pkt);

      void writeNotification();

      /** Called by event when dma to write packet is completed
       */
      void pktComplete();

      void notifComplete();

      void pktNotifComplete();

      void pktSplitComplete();

      void pktSplitNotifComplete();

      /** Check if the dma on the packet has completed and RX state machine
       * can continue
       */
      bool packetDone();
      bool notificationDone();

      EventFunctionWrapper pktEvent;

      //EventFunctionWrapper pktDataEvent; // pktNotifComplete
      EventFunctionWrapper pktNotifEvent; // notifComplete

      EventFunctionWrapper pktFirstEvent; // pktSplitComplete
      EventFunctionWrapper pktSecondEvent; // pktSplitComplete

      //EventFunctionWrapper pktFirstDataEvent; // pktSplitNotifComplete
      //EventFunctionWrapper pktSecondDataEvent; // pktSplitNotifComplete
      //EventFunctionWrapper pktSplitNotifEvent; // pktSplitNotifComplete

      bool hasOutstandingEvents() override;

      // when host setup Notification buffer, need to update status
      void setRxNotifState(int queueId) { RXNotifStates[queueId].physAddr = notifBufBase(queueId); }

      void setDmaNotifVar(uint32_t notifId, uint32_t queueId, uint32_t tail)
      {
        dmaNotifId = notifId;
        dmaQueueId = queueId;
        dmaTail = tail;
      }

      void clearDmaNotifVar()
      {
        dmaNotifId = 0;
        dmaQueueId = 0;
        dmaTail = 0;
      }

      void serialize(CheckpointOut &cp) const override;
      void unserialize(CheckpointIn &cp) override;
    };

    friend RXNotifBufManager;
    RXNotifBufManager rxNotifBufManager;

    class TXNotifBufManager : public NotifBufManager<igbreg::TxNotification>
    {
    protected:
      Addr notifBufBase(int queueId) const override { return igbe->regs.tdba_array[queueId](); }
      long notifBufHead(int queueId) const override { return igbe->regs.tdh_array[queueId](); }
      long notifBufTail(int queueId) const override { return igbe->regs.tdt_array[queueId](); }
      void updateHead(int queueId, long h) override { igbe->regs.tdh_array[queueId](h); }
      void updateTail(int queueId, long t) override { igbe->regs.tdt_array[queueId](t); }
      void enableSm() override;

      struct TxCompletion
      {
        int queueId;
        int wbHead;
        igbreg::TxNotification* txNotif;
      };

      // Notification FIFO used by TxNotifBufManager
      // max = 1024 notification (same as enso)
      std::deque<struct TxCompletion*> notifTxBuf;

      
      // TX completion Notification buffer
      std::deque<struct TxCompletion*> txComplBuf;

      // Host notification buffer addr array
      uint64_t TXNotifBaseAddr[MAX_NB_NOTIF];

      // How many descriptors we are currently fetching
      int curFetching;

      int curQueueId;

      /* TX Completion Notification write back */
      // How many descriptors we are currently writing back
      int wbOut;

      int wbQueueId;

      // if the we wrote back to the end of the descriptor ring and are going
      // to have to wrap and write more
      bool moreToWb;

      bool pktDone;
      bool pktWaiting;

      bool awaitingContinuation;

      /** The packet that is currently being dmad to memory if any */
      EthPacketPtr pktPtr;
      
      igbreg::TxNotification* fetchBuf;
      igbreg::TxNotification* wbBuf;

      

    public:
      TXNotifBufManager(IGbE *i, const std::string n, int s);
      ~TXNotifBufManager();

      // when host setup Notification buffer, need to update status
      void setTxNotifState(int queueId){ TXNotifBaseAddr[queueId] = notifBufBase(queueId); }
      uint16_t getPktLen(uint8_t* currentPkt);
      void splitPacketToFifo(EthPacketPtr pkts);

      unsigned getPacketSize();
      void getPacketData(EthPacketPtr pkt);

      long txNotifBufHead(int queueId) { return notifBufHead(queueId); }
      long txNotifBufTail(int queueId) { return notifBufTail(queueId); }

      /** Ask if the packet has been transfered so the state machine can give
       * it to the fifo.
       * @return packet available in descriptor cache
       */
      bool packetAvailable()
      {
        if (pktDone) {
            pktDone = false;
            return true;
        }
        return false;
      }

      /** Ask if we are still waiting for the packet to be transfered.
       * @return packet still in transit.
       */
      bool packetWaiting() { return pktWaiting; }

      /** Notification fetch event **/ 
      void fetchNotification(int queueId);
      void fetchNotification1();
      EventFunctionWrapper fetchDelayEvent;

      void fetchComplete();
      EventFunctionWrapper fetchEvent;

      /** Data DMA event **/
      void pktComplete();
      EventFunctionWrapper pktEvent;

      /** Write back completion notification, overwrite consumed notification **/
      void wbNotification(int queueId);
      void wbNotification1();
      EventFunctionWrapper wbDelayEvent;

      void wbComplete();
      EventFunctionWrapper wbEvent;

      // return txComplBuf is empty
      bool txComplBufEmpty() { return txComplBuf.empty(); }

      // return queueId of first completion in fifo
      int fisrtComplQueueId() { return txComplBuf.front()->queueId; }

      bool isAwaitingContinuation(){ return awaitingContinuation; }

      bool hasOutstandingEvents() override;

      void serialize(CheckpointOut &cp) const override;
      void unserialize(CheckpointIn &cp) override;

    };
    friend TXNotifBufManager;
    TXNotifBufManager txNotifBufManager;

#endif

  public:
    PARAMS(IGbE);

    RxM2funcContext *rxM2funcContextArray[MAX_QUEUE_SIZE];
    TxM2funcContext *txM2funcContextArray[MAX_QUEUE_SIZE];

    IGbE(const Params &params);
    ~IGbE();
    void init() override;

    Port &getPort(const std::string &if_name,
                  PortID idx = InvalidPortID) override;

    Tick lastInterrupt;

    Tick read(PacketPtr pkt) override;
    Tick write(PacketPtr pkt) override;

    Tick writeConfig(PacketPtr pkt) override;

    bool ethRxPkt(EthPacketPtr packet);
    void ethTxDone();

    Tick lastRxM2funcPacketComeTick; // last tick when the ethernet packet is ready to be sent to host
    Tick lastRxDMAStartTick;         // last tick when the DMA start

    void updateRxPacketReceiveStat(Tick tick)
    {
      if (lastRxM2funcPacketComeTick != 0)
      {
        Tick diff = tick - lastRxM2funcPacketComeTick;
        etherDeviceStats.rxPacketComeDistance.sample(diff);
      }

      lastRxM2funcPacketComeTick = tick;
    }

    void updateRxRingBufferDMAStartStat(Tick tick)
    {
      if (lastRxDMAStartTick != 0)
      {
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
    void enableSmRx();
  };

  class IGbEInt : public EtherInt
  {
  private:
    IGbE *dev;

  public:
    IGbEInt(const std::string &name, IGbE *d)
        : EtherInt(name), dev(d)
    {
    }

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
        : QueuedResponsePort(_name, _owner, _respQueue), _respQueue(*_owner, *this), cxlMemDelay(_delay), dev(_owner), numQueues(_numQueues) { setAddrRange(); }

    void setAddrRange()
    {
      const Addr base_addr = dev->getBARBaseAddr(0);
      for (int i = 0; i < numQueues; i++)
      {
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
