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
 * In particular an 82547 revision 2 (82547GI) MAC because it seems to have the
 * fewest workarounds in the driver. It will probably work with most of the
 * other MACs with slight modifications.
 */

#include "dev/net/i8254xGBe.hh"

/*
 * @todo really there are multiple dma engines.. we should implement them.
 */

#include <algorithm>
#include <memory>

#include <random>

#include "base/inet.hh"
#include "base/trace.hh"
#include "debug/Drain.hh"
#include "debug/EthernetAll.hh"
#include "mem/packet.hh"
#include "mem/packet_access.hh"
#include "params/IGbE.hh"
#include "sim/stats.hh"
#include "sim/system.hh"
// Loadgen debug flag for DPDK
#include "debug/EthernetDpdk.hh"
#include "i8254xGBe.hh"

// Enso debug flag
#include "debug/EthernetENSO.hh"
#include "debug/EthernetEnsoRxPipe.hh"
#include "debug/EthernetEnsoRxNotif.hh"
#include "debug/EthernetEnsoTxNotif.hh"

//0: No log, 1: Ring buffer log, 5: Ring buffer log prepare
#define LOG_LEVEL 0

// SW : more condition to i want to watch
#define WANT_TO_SEE 0

namespace gem5
{

using namespace igbreg;
using namespace networking;

IGbE::IGbE(const Params &p)
    : EtherDevice(p), adq(p.adq_idx), etherInt(NULL), m2funcPort(NULL), enableDTA(p.enable_dta), numQueues(p.num_queues),     // SHIN. add adq // jm. add numQueues
      rxFifo(p.rx_fifo_size, true), txFifo(p.tx_fifo_size, false), inTick(false), startLoadGen(false),
      rxTick(false), txTick(false), txFifoTick(false), flitSize(p.flit_size), // TODO - JM: add flitSize 
      fetchDelay(p.fetch_delay), wbDelay(p.wb_delay),
      fetchCompDelay(p.fetch_comp_delay), wbCompDelay(p.wb_comp_delay),
      rxWriteDelay(p.rx_write_delay), txReadDelay(p.tx_read_delay),
      cxlMemDelay(p.cxl_mem_delay),
    //   rdtrEvent([this]{ rdtrProcess(); }, name()),
    //   radvEvent([this]{ radvProcess(); }, name()),
    //   tadvEvent([this]{ tadvProcess(); }, name()),
    //   tidvEvent([this]{ tidvProcess(); }, name()),
      tickEvent([this]{ tick(); }, name()),
      interEvent([this]{ delayIntEvent(); }, name())
#ifdef USE_ENSO
    // generate of Manager class of ENSO
      ,EnsoRxFifo(p.rx_fifo_size, true)
      ,rxEnsoPipeManager(this, name()+".RxEnsoPipeManager", ENSO_PIPE_SIZE)
      ,rxNotifBufManager(this, name()+".RxNotifBufManager", NOTIF_BUF_SIZE)
      ,txNotifBufManager(this, name()+".TxNotifBufManager", NOTIF_BUF_SIZE)
#endif
{
    //   rxDescCache(this, name()+".RxDesc", p.rx_desc_cache_size),
    //   txDescCache(this, name()+".TxDesc", p.tx_desc_cache_size),
    //   lastInterrupt(0)
    etherInt = new IGbEInt(name() + ".int", this);

    if (enableDTA) {
        m2funcPort = new M2funcPort(name() + ".m2funcPort", this, p.cxl_mem_delay, numQueues);
    }
    //multi-queue sanity check
    if (numQueues > MAX_QUEUE_SIZE) {
        panic("Number of queues exceeds maximum allowed\n");
    }

    DPRINTF(EthernetDpdk, "Number of queues: %d\n", numQueues);

    // Set communication type
    if (p.is_m2func) {
        commType = CommunicationType::M2FUNC;
    } else {
        commType = CommunicationType::RING;
    }
    printf("Communication Type: %s\n", commType == CommunicationType::RING ? "RING" : "M2FUNC");
    printf("Enable DTA: %s\n", enableDTA ? "true" : "false");

    // Initialize rxDescCacheArray and txDescCacheArray
    for (int i = 0; i < numQueues; i++) {
        if (commType == CommunicationType::RING) {
            rxDescCacheArray[i] = new RxDescCacheGlobal(this, name()+".RxDescArray"+std::to_string(i), p.rx_desc_cache_size, i, p.num_dma_engines, p.num_desc_dma_engines); // TODO
            txDescCacheArray[i] = new TxDescCacheGlobal(this, name()+".TxDescArray"+std::to_string(i), p.tx_desc_cache_size, i, p.num_dma_engines, p.num_desc_dma_engines);
        } else if (commType == CommunicationType::M2FUNC) {
            // JM. For now, just set m2funcFifo size to 2. Maybe 1 is enough.
            rxM2funcContextArray[i] = new RxM2funcContext(this, name()+".RxM2funcContextArray"+std::to_string(i), 2, i, enableDTA, p.cxl_req_buf_size);
            txM2funcContextArray[i] = new TxM2funcContext(this, name()+".TxM2funcContextArray"+std::to_string(i), 2, i, enableDTA);
            if (!rxM2funcContextArray[i] || !txM2funcContextArray[i]) {
                panic("Failed to initialize RxM2funcContext or TxM2funcContext for queue %d\n", i);
            } else {
                printf("Successfully initialized RxM2funcContext and TxM2funcContext for queue %d with rxM2funcContextArray[%d]: %p, txM2funcContextArray[%d]: %p\n", i, i, rxM2funcContextArray[i], i, txM2funcContextArray[i]);
            }
        } else {
            panic("Invalid communication type\n");
        }
    }

    if (p.is_dpdk_setup_step) {
        is_dpdk_setup_step = true;
        printf("This is DPDK setup step!!! So RDT, TDT will act as normal DPDK\n");
    } else {
        is_dpdk_setup_step = false;
        printf("This is not DPDK setup step!!! So RDT, TDT will act as M2func\n");
    }

    lastInterrupt = 0;
    lastRxM2funcPacketComeTick = 0;
    lastRxDMAStartTick = 0;
    prevRSSQueue = 0;
    candidateTxQueue = 0;
    successTxQueueSend = false; // set to true when the candidateTxQueue's packet is successfully sent to the txFifo
    // and when it is true, the candidateTxQueue can be incremented to the next queue
    // and when it is false, the candidateTxQueue will be the same queue in the next tick

    // Initialize rxPacketArray & txPacketArray
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        rxPacketArray[i] = nullptr;
        txPacketArray[i] = nullptr;
    }

    // Initialize pktOffsetArray
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        pktOffsetArray[i] = 0;
    }

    // Initialize rxDmaPacketArray
    for (int i = 0; i < MAX_QUEUE_SIZE; i++) {
        rxDmaPacketArray[i] = false;
    }

    // Initialized internal registers per Intel documentation
    // All registers intialized to 0 by per register constructor
    regs.ctrl.fd(1);
    regs.ctrl.lrst(1);
    regs.ctrl.speed(2);
    regs.ctrl.frcspd(1);
    regs.sts.speed(3); // Say we're 1000Mbps
    regs.sts.fd(1); // full duplex
    regs.sts.lu(1); // link up
    regs.eecd.fwe(1);
    regs.eecd.ee_type(1);
    regs.imr = 0;
    regs.iam = 0;
    // regs.rxdctl.gran(1);
    // regs.rxdctl.wthresh(1);
    // initialize rxdctl for the number of queues
    for (int i = 0; i < numQueues; i++) {
        regs.rxdctl_array[i].gran(1);
        regs.rxdctl_array[i].wthresh(1);
    }
    regs.fcrth(1);
    // regs.tdwba = 0;
    for (int i = 0; i < numQueues; i++) {
        regs.tdwba_array[i] = 0;
    }
    regs.rlpml = 0;
    regs.sw_fw_sync = 0;

    regs.pba.rxa(0x30);
    regs.pba.txa(0x10);

    //Initiailize mrqc.en as 1 to enable RSS
    regs.mrqc.en(1);

    // for (int i = 0; i < RETA_SIZE; i++) {
    //     regs.reta_array[i] = 0;
    // }
    //initialize reta_array for the number of queues
    // make random number in the range of 0 to numQueues-1 & set it to reta_array
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, numQueues-1);
    for (int i = 0; i < RETA_SIZE; i++) {
        regs.reta_array[i] = dis(gen);
        // printf("reta_array[%d]: %d\n", i, regs.reta_array[i]);
    }
    eeOpBits            = 0;
    eeAddrBits          = 0;
    eeDataBits          = 0;
    eeOpcode            = 0;

    // clear all 64 16 bit words of the eeprom
    memset(&flash, 0, EEPROM_SIZE*2);

    // Set the MAC address
    memcpy(flash, p.hardware_address.bytes(), ETH_ADDR_LEN);
    for (int x = 0; x < ETH_ADDR_LEN/2; x++)
        flash[x] = htobe(flash[x]);

    uint16_t csum = 0;
    for (int x = 0; x < EEPROM_SIZE; x++)
        csum += htobe(flash[x]);


    // Magic happy checksum value
    flash[EEPROM_SIZE-1] = htobe((uint16_t)(EEPROM_CSUM - csum));

    // Store the MAC address as queue ID
    macAddr = p.hardware_address;

    rxFifo.clear();
    txFifo.clear();

    #ifdef USE_ENSO
    txPacket = nullptr;
    rxDmaPacket = false;
    rxDmaNotif = false;
    for (int i = 0; i < MAX_NB_MANAGER; i++)
    {
        pipeHeadUpdated[i] = false;
    }
    //fifoClear();
    EnsoRxFifo.clear();
    #endif
}

IGbE::~IGbE()
{
    delete etherInt;
    if (enableDTA) {
        assert(m2funcPort);
        delete m2funcPort;
    }
}

void
IGbE::init()
{
    if (enableDTA) {
        assert(m2funcPort);
        if (m2funcPort->isConnected()) {
            printf("IGbE: m2funcPort is connected\n");
            m2funcPort->sendRangeChange();
        } else {
            panic("IGbE: m2funcPort is not connected\n");
        }
    }
    PciDevice::init();
}

Port &
IGbE::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "interface")
        return *etherInt;
    else if (if_name == "m2func_port") { 
        assert(enableDTA);
        assert(m2funcPort);
        return *m2funcPort;
    }
    else
        return EtherDevice::getPort(if_name, idx);
}

Tick
IGbE::writeConfig(PacketPtr pkt)
{
    int offset = pkt->getAddr() & PCI_CONFIG_SIZE;
    if (offset < PCI_DEVICE_SPECIFIC)
        PciDevice::writeConfig(pkt);
    else
        panic("Device specific PCI config space not implemented.\n");

    //
    // Some work may need to be done here based for the pci COMMAND bits.
    //

    return configDelay;
}

// Handy macro for range-testing register access addresses
#define IN_RANGE(val, base, len) (val >= base && val < (base + len))

Tick
IGbE::read(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    // Only 32bit accesses allowed
    // assert(pkt->getSize() == 4);

    DPRINTF(Ethernet, "Read device register addr : %#X daddr: %#X with size: %d\n", pkt->getAddr(), daddr, pkt->getSize());
    // printf("Read device register addr : %#X daddr: %#X with size: %d\n", pkt->getAddr(), daddr, pkt->getSize());

    //
    // Handle read of register here
    //


    switch (daddr) {
      case REG_CTRL:
        pkt->setLE<uint32_t>(regs.ctrl());
        break;
      case REG_STATUS:
        pkt->setLE<uint32_t>(regs.sts());
        break;
      case REG_EECD:
        pkt->setLE<uint32_t>(regs.eecd());
        break;
      case REG_EERD:
        pkt->setLE<uint32_t>(regs.eerd());
        break;
      case REG_CTRL_EXT:
        pkt->setLE<uint32_t>(regs.ctrl_ext());
        break;
      case REG_MDIC:
        pkt->setLE<uint32_t>(regs.mdic());
        break;
      case REG_ICR:
        DPRINTF(Ethernet, "Reading ICR. ICR=%#x IMR=%#x IAM=%#x IAME=%d\n",
                regs.icr(), regs.imr, regs.iam, regs.ctrl_ext.iame());
        pkt->setLE<uint32_t>(regs.icr());
        if (regs.icr.int_assert() || regs.imr == 0) {
            regs.icr = regs.icr() & ~mask(30);
            DPRINTF(Ethernet, "Cleared ICR. ICR=%#x\n", regs.icr());
        }
        if (regs.ctrl_ext.iame() && regs.icr.int_assert())
            regs.imr &= ~regs.iam;
        chkInterrupt();
        break;
      case REG_EICR:
        // This is only useful for MSI, but the driver reads it every time
        // Just don't do anything
        pkt->setLE<uint32_t>(0);
        break;
      case REG_ITR:
        pkt->setLE<uint32_t>(regs.itr());
        break;
      case REG_RCTL:
        pkt->setLE<uint32_t>(regs.rctl());
        break;
      case REG_FCTTV:
        pkt->setLE<uint32_t>(regs.fcttv());
        break;
      case REG_TCTL:
        pkt->setLE<uint32_t>(regs.tctl());
        break;
      case REG_MRQC:
        pkt->setLE<uint32_t>(regs.mrqc());
        printf("Device Reg Read MRQC: %#x\n", regs.mrqc());
        break;
      case REG_PBA:
        pkt->setLE<uint32_t>(regs.pba());
        break;
      case REG_WUC:
      case REG_WUFC:
      case REG_WUS:
      case REG_LEDCTL:
        pkt->setLE<uint32_t>(0); // We don't care, so just return 0
        break;
      case REG_FCRTL:
        pkt->setLE<uint32_t>(regs.fcrtl());
        break;
      case REG_FCRTH:
        pkt->setLE<uint32_t>(regs.fcrth());
        break;
    //   case REG_RDBAL:
    //     pkt->setLE<uint32_t>(regs.rdba.rdbal());
    //     break;
    //   case REG_RDBAH:
    //     pkt->setLE<uint32_t>(regs.rdba.rdbah());
    //     break;
    //   case REG_RDLEN:
    //     pkt->setLE<uint32_t>(regs.rdlen());
    //     break;
    //   case REG_SRRCTL:
    //     pkt->setLE<uint32_t>(regs.srrctl());
    //     break;
    //   case REG_RDH:
    //     pkt->setLE<uint32_t>(regs.rdh());
    //     break;
    //   case REG_RDT:
    //     pkt->setLE<uint32_t>(regs.rdt());
    //     break;
      case REG_RDTR:
        pkt->setLE<uint32_t>(regs.rdtr());
        if (regs.rdtr.fpd()) {
            if (commType == CommunicationType::RING) {
                for (int i = 0; i < numQueues; i++) {
                    rxDescCacheArray[i]->writebackGlobal(0);
                }
            }
            DPRINTF(EthernetIntr,
                    "Posting interrupt because of RDTR.FPD write\n");
            postInterrupt(IT_RXT);
            regs.rdtr.fpd(0);
        }
        break;
    //   case REG_RXDCTL:
    //     pkt->setLE<uint32_t>(regs.rxdctl());
    //     break;
      case REG_RADV:
        pkt->setLE<uint32_t>(regs.radv());
        break;
    //   case REG_TDBAL:
    //     pkt->setLE<uint32_t>(regs.tdba.tdbal());
    //     break;
    //   case REG_TDBAH:
    //     pkt->setLE<uint32_t>(regs.tdba.tdbah());
    //     break;
    //   case REG_TDLEN:
    //     pkt->setLE<uint32_t>(regs.tdlen());
    //     break;
    //   case REG_TDH:
    //     pkt->setLE<uint32_t>(regs.tdh());
    //     break;
    //   case REG_TXDCA_CTL:
    //     pkt->setLE<uint32_t>(regs.txdca_ctl());
    //     break;
    //   case REG_TDT:
    //     pkt->setLE<uint32_t>(regs.tdt());
    //     break;
      case REG_TIDV:
        pkt->setLE<uint32_t>(regs.tidv());
        break;
    //   case REG_TXDCTL:
    //     pkt->setLE<uint32_t>(regs.txdctl());
    //     break;
      case REG_TADV:
        pkt->setLE<uint32_t>(regs.tadv());
        break;
    //   case REG_TDWBAL:
    //     pkt->setLE<uint32_t>(regs.tdwba & mask(32));
    //     break;
    //   case REG_TDWBAH:
    //     pkt->setLE<uint32_t>(regs.tdwba >> 32);
    //     break;
      case REG_RXCSUM:
        pkt->setLE<uint32_t>(regs.rxcsum());
        break;
      case REG_RLPML:
        pkt->setLE<uint32_t>(regs.rlpml);
        break;
      case REG_RFCTL:
        pkt->setLE<uint32_t>(regs.rfctl());
        break;
      case REG_MANC:
        pkt->setLE<uint32_t>(regs.manc());
        break;
      case REG_SWSM:
        pkt->setLE<uint32_t>(regs.swsm());
        regs.swsm.smbi(1);
        break;
      case REG_FWSM:
        pkt->setLE<uint32_t>(regs.fwsm());
        break;
      case REG_SWFWSYNC:
        pkt->setLE<uint32_t>(regs.sw_fw_sync);
        break;
      case REG_IMS:
        pkt->setLE<uint32_t>(regs.imr);
        break;
      default: {
        int queueid = -1;
        int retaIndex = -1;
        int rssrkIndex = -1;
        if (isRegisterAddress<E1000_RDBAL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.rdba_array[queueid].rdbal());
            uint32_t rdbal = regs.rdba_array[queueid].rdbal();
            DPRINTF(EthernetDpdk, "Read RDBAL[%d]: %#x\n", queueid, rdbal);
        } else if (isRegisterAddress<E1000_RDBAH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.rdba_array[queueid].rdbah());
            uint32_t rdbah = regs.rdba_array[queueid].rdbah();
            DPRINTF(EthernetDpdk, "Read RDBAH[%d]: %#x\n", queueid, rdbah);
        } else if (isRegisterAddress<E1000_RDLEN>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.rdlen_array[queueid]());
            uint32_t rdlen = regs.rdlen_array[queueid]();
            DPRINTF(EthernetDpdk, "Read RDLEN[%d]: %#x\n", queueid, rdlen);
        } else if (isRegisterAddress<E1000_SRRCTL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.srrctl_array[queueid]());
        } else if (isRegisterAddress<E1000_RDH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.rdh_array[queueid]());
            uint32_t rdh = regs.rdh_array[queueid]();
            DPRINTF(EthernetDpdk, "Read RDH[%d]: %d\n", queueid, rdh);
            // printf("[LOG], %lu Read RDH[%d]: %d\n", curTick(), queueid, rdh);
        } else if (isRegisterAddress<E1000_RDT>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.rdt_array[queueid]());
            uint32_t rdt = regs.rdt_array[queueid]();
            DPRINTF(EthernetDpdk, "Read RDT[%d]: %d\n", queueid, rdt);
            // printf("[LOG], %lu Read RDT[%d]: %d\n", curTick(), queueid, rdt);
        } else if (isRegisterAddress<E1000_RXM2FUNC>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            if (commType == CommunicationType::M2FUNC) {
                DPRINTF(EthernetDpdk, "RXM2func[%d] Read Comming from host to RXM2FUNC with length: %d\n", queueid, pkt->getSize());
                // printf("RXM2func[%d] Read Comming from host to RXM2FUNC with length: %d\n", queueid, pkt->getSize());
                if (enableDTA) {
                    panic("If DTA is enabled, RXM2func RD request cannot be come over here!\n");
                } else {
                    rxM2funcContextArray[queueid]->readM2funcPacket(pkt);
                }
            } else {
                panic("Invalid communication type with - RXM2FUNC read!!!\n");
            }
        } else if (isRegisterAddress<E1000_RXDCTL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.rxdctl_array[queueid]());
        } else if (isRegisterAddress<E1000_TDBAL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.tdba_array[queueid].tdbal());
            uint32_t tdbal = regs.tdba_array[queueid].tdbal();
            DPRINTF(EthernetDpdk, "Read TDBAL[%d]: %#x\n", queueid, tdbal);
        } else if (isRegisterAddress<E1000_TDBAH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.tdba_array[queueid].tdbah());
            uint32_t tdbah = regs.tdba_array[queueid].tdbah();
            DPRINTF(EthernetDpdk, "Read TDBAH[%d]: %#x\n", queueid, tdbah);
        } else if (isRegisterAddress<E1000_TDLEN>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.tdlen_array[queueid]());
            uint32_t tdlen = regs.tdlen_array[queueid]();
            DPRINTF(EthernetDpdk, "Read TDLEN[%d]: %#x\n", queueid, tdlen);
        } else if (isRegisterAddress<E1000_TDH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.tdh_array[queueid]());
            uint32_t tdh = regs.tdh_array[queueid]();
            DPRINTF(EthernetDpdk, "Read TDH[%d]: %d\n", queueid, tdh);
            // printf("[LOG], %lu Read TDH[%d]: %d\n", curTick(), queueid, tdh);
        } else if (isRegisterAddress<E1000_TXDCA_CTL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.txdca_ctl_array[queueid]());
        } else if (isRegisterAddress<E1000_TDT>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.tdt_array[queueid]());
            uint32_t tdt = regs.tdt_array[queueid]();
            DPRINTF(EthernetDpdk, "Read TDT[%d]: %d\n", queueid, tdt);
            // printf("[LOG], %lu Read TDT[%d]: %d\n", curTick(), queueid, tdt);
        } else if (isRegisterAddress<E1000_TXM2FUNC>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            if (commType == CommunicationType::M2FUNC) {
                if (enableDTA) {
                    panic("If DTA is enabled, TXM2func RD request cannot be come over here!\n");
                } else {
                    DPRINTF(EthernetDpdk, "TXM2func[%d] Read Comming from host to TXM2FUNC with length: %d\n", queueid, pkt->getSize());
                    // printf("TXM2func[%d] Read Comming from host to TXM2FUNC with length: %d\n", queueid, pkt->getSize());
                    // Set bitmask as response
                    txM2funcContextArray[queueid]->readBitmask(pkt);
                }
            } else {
                panic("Invalid communication type with - TXM2FUNC read!!!\n");
            }
        } else if (isRegisterAddress<E1000_TXDCTL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.txdctl_array[queueid]());
        } else if (isRegisterAddress<E1000_TDWBAL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.tdwba_array[queueid] & mask(32));
        } else if (isRegisterAddress<E1000_TDWBAH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.tdwba_array[queueid] >> 32);
        } else if (isRETAAddress(daddr, retaIndex)) {
            assert(retaIndex < (RETA_SIZE / 4));
            union e1000_reta {
                uint32_t dword;
                uint8_t byte[4];
            } reta;
            for (int i = 0; i < 4; i++) {
                reta.byte[i] = regs.reta_array[retaIndex * 4 + i];
            }
            pkt->setLE<uint32_t>(reta.dword);
        } else if (isRSSRKAddress(daddr, rssrkIndex)) {
            assert(rssrkIndex < 10);
            union e1000_rssrk {
                uint32_t dword;
                uint8_t byte[4];
            } rssrk;
            for (int i = 0; i < 4; i++) {
                rssrk.byte[i] = regs.rssrk[rssrkIndex * 4 + i];
            }
            pkt->setLE<uint32_t>(rssrk.dword);
        } else if (!IN_RANGE(daddr, REG_VFTA, VLAN_FILTER_TABLE_SIZE*4) &&
            !IN_RANGE(daddr, REG_RAL, RCV_ADDRESS_TABLE_SIZE*8) &&
            !IN_RANGE(daddr, REG_MTA, MULTICAST_TABLE_SIZE*4) &&
            !IN_RANGE(daddr, REG_CRCERRS, STATS_REGS_SIZE))
            panic("Read request to unknown register number: %#x, size: %d\n", daddr, pkt->getSize());
        else
            pkt->setLE<uint32_t>(0);
      }
    };

   // printf("Read value %X from address %X\n", pkt->getLE<uint32_t>(), daddr);

    pkt->makeAtomicResponse();
    return pioDelay;
}

Tick
IGbE::write(PacketPtr pkt)
{
    int bar;
    Addr daddr;

    if (!getBAR(pkt->getAddr(), bar, daddr))
        panic("Invalid PCI memory access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    // Only 32bit accesses allowed
    // assert(pkt->getSize() == sizeof(uint32_t));

    // DPRINTF(Ethernet, "Wrote device register %#X value %#X\n",
            // daddr, pkt->getLE<uint32_t>());
    // printf("Wrote device register %#X value %#X\n", daddr, pkt->getLE<uint32_t>());

    //printf("Wrote value %X to address %X\n", pkt->getLE<uint32_t>(), daddr);
    // printf("Write to address: %#X, daddr: %#X, size: %d\n", pkt->getAddr(), daddr, pkt->getSize());
    //
    // Handle write of register here
    //
    uint32_t val = 0;
    if (pkt->getSize() == sizeof(uint32_t)) {
        val = pkt->getLE<uint32_t>();
    }

    Regs::RCTL oldrctl;
    Regs::TCTL oldtctl;

    switch (daddr) {
      case REG_CTRL:
        regs.ctrl = val;
        if (regs.ctrl.tfce())
            warn("TX Flow control enabled, should implement\n");
        if (regs.ctrl.rfce())
            warn("RX Flow control enabled, should implement\n");
        break;
      case REG_CTRL_EXT:
        regs.ctrl_ext = val;
        break;
      case REG_STATUS:
        regs.sts = val;
        break;
      case REG_EECD:
        int oldClk;
        oldClk = regs.eecd.sk();
        regs.eecd = val;
        // See if this is a eeprom access and emulate accordingly
        if (!oldClk && regs.eecd.sk()) {
            if (eeOpBits < 8) {
                eeOpcode = eeOpcode << 1 | regs.eecd.din();
                eeOpBits++;
            } else if (eeAddrBits < 8 && eeOpcode == EEPROM_READ_OPCODE_SPI) {
                eeAddr = eeAddr << 1 | regs.eecd.din();
                eeAddrBits++;
            } else if (eeDataBits < 16 && eeOpcode == EEPROM_READ_OPCODE_SPI) {
                assert(eeAddr>>1 < EEPROM_SIZE);
                DPRINTF(EthernetEEPROM, "EEPROM bit read: %d word: %#X\n",
                        flash[eeAddr>>1] >> eeDataBits & 0x1,
                        flash[eeAddr>>1]);
                regs.eecd.dout((flash[eeAddr>>1] >> (15-eeDataBits)) & 0x1);
                eeDataBits++;
            } else if (eeDataBits < 8 && eeOpcode == EEPROM_RDSR_OPCODE_SPI) {
                regs.eecd.dout(0);
                eeDataBits++;
            } else
                panic("What's going on with eeprom interface? opcode:"
                      " %#x:%d addr: %#x:%d, data: %d\n", (uint32_t)eeOpcode,
                      (uint32_t)eeOpBits, (uint32_t)eeAddr,
                      (uint32_t)eeAddrBits, (uint32_t)eeDataBits);

            // Reset everything for the next command
            if ((eeDataBits == 16 && eeOpcode == EEPROM_READ_OPCODE_SPI) ||
                (eeDataBits == 8 && eeOpcode == EEPROM_RDSR_OPCODE_SPI)) {
                eeOpBits = 0;
                eeAddrBits = 0;
                eeDataBits = 0;
                eeOpcode = 0;
                eeAddr = 0;
            }

            DPRINTF(EthernetEEPROM, "EEPROM: opcode: %#X:%d addr: %#X:%d\n",
                    (uint32_t)eeOpcode, (uint32_t) eeOpBits,
                    (uint32_t)eeAddr>>1, (uint32_t)eeAddrBits);
            if (eeOpBits == 8 && !(eeOpcode == EEPROM_READ_OPCODE_SPI ||
                                   eeOpcode == EEPROM_RDSR_OPCODE_SPI ))
                panic("Unknown eeprom opcode: %#X:%d\n", (uint32_t)eeOpcode,
                      (uint32_t)eeOpBits);


        }
        // If driver requests eeprom access, immediately give it to it
        regs.eecd.ee_gnt(regs.eecd.ee_req());
        break;
      case REG_EERD:
        regs.eerd = val;
        if (regs.eerd.start()) {
            regs.eerd.done(1);
            assert(regs.eerd.addr() < EEPROM_SIZE);
            regs.eerd.data(flash[regs.eerd.addr()]);
            regs.eerd.start(0);
            DPRINTF(EthernetEEPROM, "EEPROM: read addr: %#X data %#x\n",
                    regs.eerd.addr(), regs.eerd.data());
        }
        break;
      case REG_MDIC:
        regs.mdic = val;
        if (regs.mdic.i())
            panic("No support for interrupt on mdic complete\n");
        if (regs.mdic.phyadd() != 1)
            panic("No support for reading anything but phy\n");
        DPRINTF(Ethernet, "%s phy address %x\n",
                regs.mdic.op() == 1 ? "Writing" : "Reading",
                regs.mdic.regadd());
        switch (regs.mdic.regadd()) {
          case PHY_PSTATUS:
            regs.mdic.data(0x796D); // link up
            break;
          case PHY_PID:
            regs.mdic.data(params().phy_pid);
            break;
          case PHY_EPID:
            regs.mdic.data(params().phy_epid);
            break;
          case PHY_GSTATUS:
            regs.mdic.data(0x7C00);
            break;
          case PHY_EPSTATUS:
            regs.mdic.data(0x3000);
            break;
          case PHY_AGC:
            regs.mdic.data(0x180); // some random length
            break;
          default:
            regs.mdic.data(0);
        }
        regs.mdic.r(1);
        break;
      case REG_ICR:
        DPRINTF(Ethernet, "Writing ICR. ICR=%#x IMR=%#x IAM=%#x IAME=%d\n",
                regs.icr(), regs.imr, regs.iam, regs.ctrl_ext.iame());
        if (regs.ctrl_ext.iame())
            regs.imr &= ~regs.iam;
        regs.icr = ~bits(val,30,0) & regs.icr();
        chkInterrupt();
        break;
      case REG_ITR:
        regs.itr = val;
        break;
      case REG_ICS:
        DPRINTF(EthernetIntr, "Posting interrupt because of ICS write\n");
        postInterrupt((IntTypes)val);
        break;
      case REG_IMS:
        regs.imr |= val;
        chkInterrupt();
        break;
      case REG_IMC:
        regs.imr &= ~val;
        chkInterrupt();
        break;
      case REG_IAM:
        regs.iam = val;
        break;
      case REG_RCTL:
        oldrctl = regs.rctl;
        regs.rctl = val;
        if (regs.rctl.rst()) {
            if (commType == CommunicationType::RING) {
                for (int i = 0; i < numQueues; i++) {
                    rxDescCacheArray[i]->reset();
                }
            }
            // rxDescCache.reset();
            DPRINTF(EthernetSM, "RXS: Got RESET!\n");
            rxFifo.clear();
            regs.rctl.rst(0);
        }
        if (regs.rctl.en())
            rxTick = true;
        restartClock();
        break;
      case REG_FCTTV:
        regs.fcttv = val;
        break;
      case REG_TCTL:
        regs.tctl = val;
        oldtctl = regs.tctl;
        regs.tctl = val;
        if (regs.tctl.en())
            txTick = true;
        restartClock();
        if (regs.tctl.en() && !oldtctl.en()) {
            for (int i = 0; i < numQueues; i++) {
                if (commType == CommunicationType::RING)
                    txDescCacheArray[i]->reset();
            }
            // txDescCache.reset();
        }
        break;
      case REG_MRQC:
        regs.mrqc = val;
        printf("Device Reg Write MRQC: %#x. Enable: %d\n", regs.mrqc(), regs.mrqc.en());
        if (regs.mrqc.en() == 0) {
            printf("RSS disabled, but we enforce it\n");
            regs.mrqc.en(1);
        }
        break;
      case REG_PBA:
        regs.pba.rxa(val);
        regs.pba.txa(64 - regs.pba.rxa());
        break;
      case REG_WUC:
      case REG_WUFC:
      case REG_WUS:
      case REG_LEDCTL:
      case REG_FCAL:
      case REG_FCAH:
      case REG_FCT:
      case REG_VET:
      case REG_AIFS:
      case REG_TIPG:
        ; // We don't care, so don't store anything
        break;
      case REG_IVAR0:
        warn("Writing to IVAR0, ignoring...\n");
        break;
      case REG_FCRTL:
        regs.fcrtl = val;
        break;
      case REG_FCRTH:
        regs.fcrth = val;
        break;
      case REG_RDTR:
        regs.rdtr = val;
        break;
      case REG_RADV:
        regs.radv = val;
        break;
      case REG_TIDV:
        regs.tidv = val;
        break;
      case REG_TADV:
        regs.tadv = val;
        break;
      case REG_RXCSUM:
        regs.rxcsum = val;
        break;
      case REG_RLPML:
        regs.rlpml = val;
        break;
      case REG_RFCTL:
        regs.rfctl = val;
        if (regs.rfctl.exsten())
            panic("Extended RX descriptors not implemented\n");
        break;
      case REG_MANC:
        regs.manc = val;
        break;
      case REG_SWSM:
        regs.swsm = val;
        if (regs.fwsm.eep_fw_semaphore())
            regs.swsm.swesmbi(0);
        break;
      case REG_SWFWSYNC:
        regs.sw_fw_sync = val;
        break;
      default: {
        int queueid = -1;
        int retaIndex = -1;
        int rssrkIndex = -1;
        if (isRegisterAddress<E1000_RDBAL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            
            #ifndef USE_ENSO
            regs.rdba_array[queueid].rdbal(val & ~mask(4));
            if (commType == CommunicationType::RING) 
                rxDescCacheArray[queueid]->areaChanged();
            uint32_t rdbal = regs.rdba_array[queueid].rdbal();
            DPRINTF(EthernetDpdk, "Write RDBAL[%d]: %#x\n", queueid, rdbal);
            #else
            /*
            * if host setting enso pipe rx_mem_low,
            * LSB is used for Notification buffer id
            */
            if(queueid >= MAX_NB_MANAGER) // target RX enso pipe
                regs.rdba_array[queueid].rdbal(val);
            else
                regs.rdba_array[queueid].rdbal(val & ~mask(4));
            #endif
            
        } else if (isRegisterAddress<E1000_RDBAH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.rdba_array[queueid].rdbah(val);
            #ifndef USE_ENSO
            if (commType == CommunicationType::RING) 
                rxDescCacheArray[queueid]->areaChanged();
            uint32_t rdbah = regs.rdba_array[queueid].rdbah();
            DPRINTF(EthernetDpdk, "Write RDBAH[%d]: %#x\n", queueid, rdbah);
            #else // USE_ENSO
            // when host write RX enso pipe or RX notification `rx_mem_high`, need to update pipe status
            uint32_t rdbah = regs.rdba_array[queueid].rdbah();
            if(queueid >= MAX_NB_MANAGER) // target RX enso pipe
            {
                rxEnsoPipeManager.setRxPipeState(queueid-MAX_NB_MANAGER);
                //printf("Write RX PIPE RDBAH[%d]: %#x\n", queueid, rdbah);
                DPRINTF(EthernetENSO, "Write RX PIPE RDBAH[%d]: %#x\n", queueid, rdbah);
            }
            else // target RX notification buffer
            {
                rxNotifBufManager.setRxNotifState(queueid);
                //printf("Write RX NOTIF RDBAH[%d]: %#x\n", queueid, rdbah);
                DPRINTF(EthernetENSO, "Write RX NOTIF RDBAH[%d]: %#x\n", queueid, rdbah);
            }
            #endif
            
        } else if (isRegisterAddress<E1000_RDLEN>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.rdlen_array[queueid] = val & ~mask(7);
            if (commType == CommunicationType::RING) 
                rxDescCacheArray[queueid]->areaChanged();
            uint32_t rdlen = regs.rdlen_array[queueid]();
            DPRINTF(EthernetDpdk, "Write RDLEN[%d]: %#x\n", queueid, rdlen);
        } else if (isRegisterAddress<E1000_SRRCTL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.srrctl_array[queueid] = val;
        } else if (isRegisterAddress<E1000_RDH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            #ifdef USE_ENSO
            // only for RxEnsoPipeManager 8~15 (when MAX_QUEUE=16)
            if(queueid >= MAX_NB_MANAGER)
            {
                pipeHeadUpdated[queueid-MAX_NB_MANAGER] = true;
                DPRINTF(EthernetENSO, "Host update pipe head\n");
            }
            #else
            if (commType == CommunicationType::RING)
                rxDescCacheArray[queueid]->areaChanged();
            #endif

            regs.rdh_array[queueid] = val;
            
            DPRINTF(EthernetDpdk, "Write RDH[%d]: %d\n", queueid, regs.rdh_array[queueid]());
            DPRINTF(EthernetENSO, "Write RDH[%d]: %d\n", queueid, regs.rdh_array[queueid]());
            // printf("[LOG], %lu Write RDH[%d]: %d\n", curTick(), queueid, val);
        } else if (isRegisterAddress<E1000_RDT>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.rdt_array[queueid] = val;
            DPRINTF(EthernetDpdk, "RXS: RDT Updated.\n");
            DPRINTF(EthernetDpdk, "Write RDT[%d]: %d\n", queueid, regs.rdt_array[queueid]());
            DPRINTF(EthernetENSO, "Write RDT[%d]: %d\n", queueid, regs.rdt_array[queueid]());
            Tick headerDelay = pkt->headerDelay;
            Tick payloadDelay = pkt->payloadDelay;
            #ifndef USE_ENSO
            #if LOG_LEVEL == 1 || LOG_LEVEL == 5
            printf("[LOG], %lu, Write RDT[%d]: %d\n", curTick(), queueid, val);
            #endif
            etherDeviceStats.rxTailWriteBytes += pkt->getSize();
            if (commType == CommunicationType::RING) {
                if (drainState() == DrainState::Running) {
                    DPRINTF(EthernetDpdk, "RXS: RDT Fetching Descriptors! in queue %d\n",
                            queueid);
                    rxDescCacheArray[queueid]->fetchDescriptorsGlobal();
                } else {
                    printf("RXS: RDT NOT Fetching Desc b/c draining! in queue %d\n", queueid);
                }
            }
            #endif
        } else if (isRegisterAddress<E1000_RXM2FUNC>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            if (commType == CommunicationType::M2FUNC) {
                DPRINTF(EthernetDpdk, "RXM2func[%d] Write Comming from host to RXM2FUNC with length: %d But we ignore it\n", queueid, pkt->getSize());
                // printf("RXM2func[%d] Write Comming from host to RXM2FUNC with length: %d But we ignore it\n", queueid, pkt->getSize());
            } else {
                panic("Invalid communication type with - RXM2FUNC write!!!\n");
            }
        } else if (isRegisterAddress<E1000_RXDTA>(daddr, queueid, numQueues)) {
          printf("RXDTA[%d]: RX Write comming from host!! with length: %d This can be captured as PIO if in atomic, checkpoint mode\n", queueid, pkt->getSize());  
        } else if (isRegisterAddress<E1000_RXDCTL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.rxdctl_array[queueid] = val;
        } else if (isRegisterAddress<E1000_TDBAL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.tdba_array[queueid].tdbal(val & ~mask(4));
            #ifndef USE_ENSO
            if (commType == CommunicationType::RING)
                txDescCacheArray[queueid]->areaChanged();
            DPRINTF(EthernetDpdk, "Write TDBAL[%d]: %#x\n", queueid, regs.tdba_array[queueid].tdbal());
            #endif
        } else if (isRegisterAddress<E1000_TDBAH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.tdba_array[queueid].tdbah(val);
            #ifndef USE_ENSO
            if (commType == CommunicationType::RING)
                txDescCacheArray[queueid]->areaChanged();
            #else
            txNotifBufManager.setTxNotifState(queueid);
            #endif
            DPRINTF(EthernetDpdk, "Write TDBAH[%d]: %#x\n", queueid, regs.tdba_array[queueid].tdbah());
            DPRINTF(EthernetENSO, "Write TX NOTIF TDBAH[%d]: %#x\n", queueid, regs.tdba_array[queueid].tdbah());
        } else if (isRegisterAddress<E1000_TDLEN>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.tdlen_array[queueid] = val & ~mask(7);
            if (commType == CommunicationType::RING)
                txDescCacheArray[queueid]->areaChanged();
            DPRINTF(EthernetDpdk, "Write TDLEN[%d]: %#x\n", queueid, regs.tdlen_array[queueid]());
        } else if (isRegisterAddress<E1000_TDH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.tdh_array[queueid] = val;
            #ifndef USE_ENSO
            if (commType == CommunicationType::RING)
                txDescCacheArray[queueid]->areaChanged();
            DPRINTF(EthernetDpdk, "Write TDH[%d]: %d\n", queueid, regs.tdh_array[queueid]());
            #endif
            DPRINTF(EthernetENSO, "Write TDH[%d]: %d\n", queueid, regs.tdh_array[queueid]());
            // printf("[LOG], %lu, Write TDH[%d]: %d\n", curTick(), queueid, val);
        } else if (isRegisterAddress<E1000_TXDCA_CTL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.txdca_ctl_array[queueid] = val;
            if (regs.txdca_ctl_array[queueid].enabled())
                panic("No support for DCA\n");
        } else if (isRegisterAddress<E1000_TDT>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.tdt_array[queueid] = val;
            Tick headerDelay = pkt->headerDelay;
            Tick payloadDelay = pkt->payloadDelay;
            DPRINTF(EthernetDpdk, "TXS: TX Tail pointer updated in queue %d\n", queueid);
            DPRINTF(EthernetDpdk, "Write TDT[%d]: %d, headerDelay: %d, payloadDelay: %d\n", queueid, regs.tdt_array[queueid](),
                   headerDelay, payloadDelay);
            DPRINTF(EthernetENSO, "Write TDT[%d]: %d, headerDelay: %d, payloadDelay: %d\n", queueid, regs.tdt_array[queueid](),
                   headerDelay, payloadDelay);
            #ifndef USE_ENSO
            #if LOG_LEVEL == 1 || LOG_LEVEL == 5
            printf("[LOG], %lu, Write TDT[%d]: %d\n", curTick(), queueid, val);
            #endif
            etherDeviceStats.txTailWriteBytes += pkt->getSize();
            if (commType == CommunicationType::RING) {
                if (drainState() == DrainState::Running) {
                    DPRINTF(EthernetDpdk, "TXS: TDT Fetching Descriptors! in queue %d\n", queueid);  
                    txDescCacheArray[queueid]->fetchDescriptorsGlobal();
                } else {
                    printf("TXS: TDT NOT Fetching Desc b/c draining! in queue %d\n", queueid);
                }
            }
            #else
            if (drainState() == DrainState::Running) {
                DPRINTF(EthernetDpdk, "TXS: TDT Fetching TX Notification! in queue %d\n", queueid);  
                txNotifBufManager.fetchNotification(queueid);
            } else {
                printf("TXS: TDT NOT Fetching Desc b/c draining! in queue %d\n", queueid);
            }
            #endif
        } else if (isRegisterAddress<E1000_TXM2FUNC>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            if (commType == CommunicationType::M2FUNC) {
                if (enableDTA) {
                    panic("If DTA is enabled, TXM2func WR request cannot be come over here!\n");
                } else {
                    DPRINTF(EthernetDpdk, "TXM2func[%d]: TX Write comming from host!! with length: %d\n", queueid, pkt->getSize());
                    // printf("TXM2func[%d]: TX Write comming from host!! with length: %d\n", queueid, pkt->getSize());
                    
                    txM2funcContextArray[queueid]->writeM2funcPacket(pkt);

                    enableSmTx();
                    checkDrain();
                }
            } else {
                panic("Invalid communication type with - TXM2FUNC write!!!\n");
            }
        } else if (isRegisterAddress<E1000_TXDTA>(daddr, queueid, numQueues)) {
            printf("TXDTA[%d]: TX Write comming from host!! with length: %d This can be captured as PIO if in atomic, checkpoint mode\n", queueid, pkt->getSize());
        } else if (isRegisterAddress<E1000_TXDCTL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.txdctl_array[queueid] = val;
        } else if (isRegisterAddress<E1000_TDWBAL>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.tdwba_array[queueid] &= ~mask(32);
            regs.tdwba_array[queueid] |= val;
            if (commType == CommunicationType::RING) {
                txDescCacheArray[queueid]->completionWriteback(regs.tdwba_array[queueid] & ~mask(1),
                                                            regs.tdwba_array[queueid] & mask(1));
            }
        } else if (isRegisterAddress<E1000_TDWBAH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.tdwba_array[queueid] &= mask(32);
            regs.tdwba_array[queueid] |= (uint64_t)val << 32;
            if (commType == CommunicationType::RING) {
                txDescCacheArray[queueid]->completionWriteback(regs.tdwba_array[queueid] & ~mask(1),
                                                            regs.tdwba_array[queueid] & mask(1));
            }
        } else if (isRETAAddress(daddr, retaIndex)) {
            assert(retaIndex < (RETA_SIZE / 4));
            union e1000_reta {
                uint32_t dword;
                uint8_t byte[4];
            } reta;
            reta.dword = val;
            for (int i = 0; i < 4; i++) {
                regs.reta_array[retaIndex * 4 + i] = reta.byte[i];
                DPRINTF(EthernetDpdk, "Write RETA[%d]: %d\n", retaIndex * 4 + i, regs.reta_array[retaIndex * 4 + i]);
            }
        } else if (isRSSRKAddress(daddr, rssrkIndex)) {
            printf("Not support RSSRK write!!!\n");
        } else if (!IN_RANGE(daddr, REG_VFTA, VLAN_FILTER_TABLE_SIZE*4) &&
            !IN_RANGE(daddr, REG_RAL, RCV_ADDRESS_TABLE_SIZE*8) &&
            !IN_RANGE(daddr, REG_MTA, MULTICAST_TABLE_SIZE*4))
            panic("Write request to unknown register number: %#x\n", daddr);
      }
    };

    pkt->makeAtomicResponse();
    return pioDelay;
}

void
IGbE::postInterrupt(IntTypes t, bool now)
{
    assert(t);

    // Interrupt is already pending
    if (t & regs.icr() && !now)
        return;

    regs.icr = regs.icr() | t;

    Tick itr_interval = sim_clock::as_int::ns * 256 * regs.itr.interval();
    DPRINTF(EthernetIntr,
            "EINT: postInterrupt() curTick(): %d itr: %d interval: %d\n",
            curTick(), regs.itr.interval(), itr_interval);

    if (regs.itr.interval() == 0 || now ||
        lastInterrupt + itr_interval <= curTick()) {
        if (interEvent.scheduled()) {
            deschedule(interEvent);
        }
        cpuPostInt();
    } else {
        Tick int_time = lastInterrupt + itr_interval;
        assert(int_time > 0);
        DPRINTF(EthernetIntr, "EINT: Scheduling timer interrupt for tick %d\n",
                int_time);
        if (!interEvent.scheduled()) {
            schedule(interEvent, int_time);
        }
    }
}

void
IGbE::delayIntEvent()
{
    cpuPostInt();
}


void
IGbE::cpuPostInt()
{
    PciCommandRegister command = letoh(PciDevice::config.command);
    //if interrupt masking bit is set
    if (command.interruptDisable)
       return;

    etherDeviceStats.postedInterrupts++;

    if (!(regs.icr() & regs.imr)) {
        DPRINTF(Ethernet, "Interrupt Masked. Not Posting\n");
        return;
    }

    DPRINTF(Ethernet, "Posting Interrupt\n");


    if (interEvent.scheduled()) {
        deschedule(interEvent);
    }

    //jm - this part would not be used. Because of DPDK PMD. Also, rxt0 is for RX queue 0 receiver timer interrupt. 
    //If want to interrupt for other queues, have to use extended interrupt cause (EICR)
    // So, for now, just check the rdtrEvent and radvEvent of queue 0
    if (commType == CommunicationType::RING) {
        if (rxDescCacheArray[0]->_radvEvent.scheduled()) {
            regs.icr.rxt0(1);
            deschedule(rxDescCacheArray[0]->_radvEvent);
        }
        if (rxDescCacheArray[0]->_rdtrEvent.scheduled()) {
            regs.icr.rxt0(1);
            deschedule(rxDescCacheArray[0]->_rdtrEvent);
        }
    }

    //txdw: TX descriptor write-back interrupt
    for (int i = 0; i < numQueues; i++) {
        if (commType == CommunicationType::RING) {
            if (txDescCacheArray[i]->_tadvEvent.scheduled()) {
                regs.icr.txdw(1);
                deschedule(txDescCacheArray[i]->_tadvEvent);
            }
            if (txDescCacheArray[i]->_tidvEvent.scheduled()) {
                regs.icr.txdw(1);
                deschedule(txDescCacheArray[i]->_tidvEvent);
            }
        }
    }

    regs.icr.int_assert(1);
    DPRINTF(EthernetIntr, "EINT: Posting interrupt to CPU now. Vector %#x\n",
            regs.icr());

    intrPost();

    lastInterrupt = curTick();
}

void
IGbE::cpuClearInt()
{
    if (regs.icr.int_assert()) {
        regs.icr.int_assert(0);
        DPRINTF(EthernetIntr,
                "EINT: Clearing interrupt to CPU now. Vector %#x\n",
                regs.icr());
        intrClear();
    }
}

void
IGbE::chkInterrupt()
{
    DPRINTF(Ethernet, "Checking interrupts icr: %#x imr: %#x\n", regs.icr(),
            regs.imr);
    // Check if we need to clear the cpu interrupt
    if (!(regs.icr() & regs.imr)) {
        DPRINTF(Ethernet, "Mask cleaned all interrupts\n");
        if (interEvent.scheduled())
            deschedule(interEvent);
        if (regs.icr.int_assert())
            cpuClearInt();
    }
    DPRINTF(Ethernet, "ITR = %#X itr.interval = %#X\n",
            regs.itr(), regs.itr.interval());

    if (regs.icr() & regs.imr) {
        if (regs.itr.interval() == 0)  {
            cpuPostInt();
        } else {
            DPRINTF(Ethernet,
                    "Possibly scheduling interrupt because of imr write\n");
            if (!interEvent.scheduled()) {
                Tick t = curTick() +
                    sim_clock::as_int::ns * 256 * regs.itr.interval();
                DPRINTF(Ethernet, "Scheduling for %d\n", t);
                schedule(interEvent, t);
            }
        }
    }
}


///////////////////////////// IGbE::DescCache //////////////////////////////

template<class T>
IGbE::DescCache<T>::DescCache(IGbE *i, const std::string n, bool _isRx, int engineIdx)
    : igbe(i), _name(n), isRx(_isRx), dmaEngineIdx(engineIdx),
      pktPtr(NULL), dmaAddr(0)
{
}

template<class T>
IGbE::DescCache<T>::~DescCache()
{
}

///////////////////////////// IGbE::DescDMAEngine ////////////////////////////
IGbE::DescDMAEngine::DescDMAEngine(void *p, const std::string n, int idx, IGbE *i, bool _isRx,
                      bool _isDescFetchEngine, bool _isDescWbEngine)
    : parent(p), descDMAEngineIdx(idx), igbe(i), _name(n),
      isRx(_isRx), isDescFetchEngine(_isDescFetchEngine),
      isDescWbEngine(_isDescWbEngine),
      descBase(0), descLen(0), numDescDMAing(0), fetchBuf(nullptr), wbBuf(nullptr),
      wbDelayEvent([this]{writebackRegister1(); }, n),
      wbDMAEvent([this]{writebackComplete(); }, n),
      fetchDelayEvent([this]{fetchRegister1(); }, n),
      fetchDMAEvent([this]{fetchComplete(); }, n)
{
    
}
IGbE::DescDMAEngine::~DescDMAEngine()
{

}

void
IGbE::DescDMAEngine::parentsPrinter(std::string printStr)
{
    if (isRx) {
        // use parent's printer
        RxDescCacheGlobal *rxDescCache = static_cast<RxDescCacheGlobal*>(parent);
        rxDescCache->printer(printStr);
    } else {
        // use parent's printer
        TxDescCacheGlobal *txDescCache = static_cast<TxDescCacheGlobal*>(parent);
        txDescCache->printer(printStr);
    }
}

void 
IGbE::DescDMAEngine::writebackRegister(Addr _descBase, Addr _descLen, int numDesc, void *_wbBuf)
{
    assert(descBase == 0);
    assert(descLen == 0);
    assert(numDescDMAing == 0);
    assert(!wbDelayEvent.scheduled());
    assert(!wbDMAEvent.scheduled());

    descBase = _descBase;
    descLen = _descLen;
    wbBuf = _wbBuf;
    numDescDMAing = numDesc;

    std::string printStr = "DMA Engine " + _name + ": Writing back " + std::to_string(numDescDMAing) + " descriptors\n";
    parentsPrinter(printStr);
    
    igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);

}

void
IGbE::DescDMAEngine::writebackRegister1()
{
    // If we're draining delay issuing this DMA
    if (igbe->drainState() != DrainState::Running) {
        igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);
        return;
    }

    std::string printStr = "Begining DMA of " + std::to_string(numDescDMAing) + " descriptors\n";
    parentsPrinter(printStr);

    assert(numDescDMAing);

    #if LOG_LEVEL == 1
    if (isRx) {
        printf("[LOG], %llu, RXDescWB_DMAE[%d], Addr: %lx, Size: %ld\n",
            curTick(), descDMAEngineIdx, descBase, descLen);
    } else {
        printf("[LOG], %llu, TXDescWB_DMAE[%d], Addr: %lx, Size: %ld\n",
            curTick(), descDMAEngineIdx, descBase, descLen);
    }
    #endif

    igbe->IdioWrite(descBase,
                    descLen, &wbDMAEvent, (uint8_t*)wbBuf,
                    igbe->wbCompDelay, 0, igbe->adq);
}

void
IGbE::DescDMAEngine::writebackComplete()
{
    #if LOG_LEVEL == 1
    if (isRx) {
        printf("[LOG], %llu, WBRXDescComplete_DMAE[%d], Addr: %lx, Size: %ld\n",
            curTick(), descDMAEngineIdx, descBase, descLen);
    } else {
        printf("[LOG], %llu, WBTXDescComplete_DMAE[%d], Addr: %lx, Size: %ld\n",
            curTick(), descDMAEngineIdx, descBase, descLen);
    }
    #endif

    // clear the DMA engine
    descBase = 0;
    descLen = 0;
    numDescDMAing = 0;
    wbBuf = nullptr;

    //call wbCompleteGlobal
    if (isRx) {
        RxDescCacheGlobal *rxDescCache = static_cast<RxDescCacheGlobal*>(parent);
        rxDescCache->wbCompleteGlobal(descDMAEngineIdx);
    } else {
        TxDescCacheGlobal *txDescCache = static_cast<TxDescCacheGlobal*>(parent);
        txDescCache->wbCompleteGlobal(descDMAEngineIdx);
    }
}

void
IGbE::DescDMAEngine::fetchRegister(Addr _descBase, Addr _descLen, int numDesc, void *_wbBuf)
{
    assert(descBase == 0);
    assert(descLen == 0);
    assert(numDescDMAing == 0);
    assert(!fetchDelayEvent.scheduled());
    assert(!fetchDMAEvent.scheduled());

    descBase = _descBase;
    descLen = _descLen;
    fetchBuf = _wbBuf;
    numDescDMAing = numDesc;

    std::string printStr = "DMA Engine " + _name + ": Fetching " + std::to_string(numDescDMAing) + " descriptors\n";
    parentsPrinter(printStr);

    igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
}

void
IGbE::DescDMAEngine::fetchRegister1()
{
    // If we're draining delay issuing this DMA
    if (igbe->drainState() != DrainState::Running) {
        igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
        return;
    }

    std::string printStr = "Fetching descriptors at " + std::to_string(descBase) + ", size: " + std::to_string(descLen) + "\n";
    parentsPrinter(printStr);
    
    #if LOG_LEVEL == 1
    if (isRx) {
        printf("[LOG], %llu, RXDescFetch_DMAE[%d], Addr: %lx, Size: %ld\n",
            curTick(), descDMAEngineIdx, descBase, descLen);
    } else {
        printf("[LOG], %llu, TXDescFetch_DMAE[%d], Addr: %lx, Size: %ld\n",
            curTick(), descDMAEngineIdx, descBase, descLen);
    }
    #endif
    assert(numDescDMAing);
    igbe->dmaRead(descBase,
                  descLen, &fetchDMAEvent, (uint8_t*)fetchBuf,
                  igbe->fetchCompDelay);

}

void
IGbE::DescDMAEngine::fetchComplete()
{
    #if LOG_LEVEL == 1 || LOG_LEVEL == 5
    if (isRx) {
        printf("[LOG], %lu, RXDescFetchComplete_DMAE[%d], Addr: %lx, Size: %ld\n",
            curTick(), descDMAEngineIdx, descBase, descLen);
    } else {
        printf("[LOG], %lu, TXDescFetchComplete_DMAE[%d], Addr: %lx, Size: %ld\n",
            curTick(), descDMAEngineIdx, descBase, descLen);
    }
    #endif

    // clear the DMA engine
    descBase = 0;
    descLen = 0;
    numDescDMAing = 0;
    fetchBuf = nullptr;

    //call fetchCompleteGlobal
    if (isRx) {
        RxDescCacheGlobal *rxDescCache = static_cast<RxDescCacheGlobal*>(parent);
        rxDescCache->fetchCompleteGlobal(descDMAEngineIdx);
    } else {
        TxDescCacheGlobal *txDescCache = static_cast<TxDescCacheGlobal*>(parent);
        txDescCache->fetchCompleteGlobal(descDMAEngineIdx);
    }
}

template<class T>
IGbE::DescCacheGlobal<T>::DescCacheGlobal(IGbE *i, const std::string n, int s, bool _isRx, int _numDMAEngines, int _numDescDMAEngines)
    : igbe(i), _name(n), cachePnt(0), size(s), curFetching(0), isRx(_isRx), numDMAEngines(_numDMAEngines), numDescDMAEngines(_numDescDMAEngines),
      wbOut(0), moreToWb(false), wbAlignment(0)
{
    fetchBuf = new T[size];
    wbBuf = new T[size];
    // Initialize processingCache
    for (int idx = 0; idx < MAX_DMA_ENGINE_SIZE; idx++) {
        processingCache[idx] = {NULL, 0, false};
        processingPktArray[idx] = NULL;
    }
    // Initialize descriptor dma engines
    for (int idx = 0; idx < MAX_DMA_ENGINE_SIZE; idx++) {
        std::string descFetchDMAEngineName = _name + ".descFetchDMAEngine" + std::to_string(idx);
        std::string descWBDMAEngineName = _name + ".descWBDMAEngine" + std::to_string(idx);
        descFetchDMAEngines[idx] = new DescDMAEngine(this, descFetchDMAEngineName, idx, igbe, isRx, true, false);
        descWbDMAEngines[idx] = new DescDMAEngine(this, descWBDMAEngineName, idx, igbe, isRx, false, true);
        
        descFetchDMAEngineWorking[idx] = false;
        descWbDMAEngineWorking[idx] = false;
        descFetchDMAEngineDone[idx] = false;
        descWbDMAEngineDone[idx] = false;

        descFetchBaseIdx[idx] = -1;
        descFetchNum[idx] = -1;

        fetchBufArray[idx] = new T[size];

        descWbBaseIdx[idx] = -1;
        descWbNum[idx] = -1;

        wbBufArray[idx] = new T[size];
        
    }
    // fetch/Wb tracking
    curFetchDMAPnt = 0;
    curWbDMAPnt = 0;
    curFetchingNum = 0;
    curWbingNum = 0;
}

template<class T>
IGbE::DescCacheGlobal<T>::~DescCacheGlobal()
{
    reset();
    delete[] fetchBuf;
    delete[] wbBuf;
    for (int idx = 0; idx < MAX_DMA_ENGINE_SIZE; idx++) {
        delete[] fetchBufArray[idx];
        delete[] wbBufArray[idx];
        delete descFetchDMAEngines[idx];
        delete descWbDMAEngines[idx];
    }   
}

template<class T>
void
IGbE::DescCacheGlobal<T>::areaChanged()
{
    if (usedCache.size() > 0 || curFetching || wbOut)
        panic("Descriptor Address, Length or Head changed. Bad\n");
    reset();

}


template<class T>
void
IGbE::DescCacheGlobal<T>::reset()
{
    DPRINTF(EthernetDesc, "Reseting descriptor cache\n");
    for (typename CacheType::size_type x = 0; x < usedCache.size(); x++)
        delete usedCache[x];
    for (typename CacheType::size_type x = 0; x < unusedCache.size(); x++)
        delete unusedCache[x];
    for (int i = 0; i < MAX_DMA_ENGINE_SIZE; i++) {
        if (processingCache[i].desc != NULL) {
            delete processingCache[i].desc;
            processingCache[i].desc = NULL;
        }
    }
    for (int i = 0; i < MAX_DMA_ENGINE_SIZE; i++) {
        processingPktArray[i] = NULL;
    }
    for (int i = 0; i < MAX_DMA_ENGINE_SIZE; i++) {
        descFetchDMAEngineWorking[i] = false;
        descWbDMAEngineWorking[i] = false;
        descFetchDMAEngineDone[i] = false;
        descWbDMAEngineDone[i] = false;

        descFetchBaseIdx[i] = -1;
        descFetchNum[i] = -1;

        descWbBaseIdx[i] = -1;
        descWbNum[i] = -1;
    }
    fetchDMAJobQueue.clear();
    wbDMAJobQueue.clear();
    curFetchDMAPnt = 0;
    curWbDMAPnt = 0;
    curFetchingNum = 0;
    curWbingNum = 0;


    usedCache.clear();
    unusedCache.clear();

    cachePnt = 0;

}

template<class T>
bool
IGbE::DescCacheGlobal<T>::hasFreeWbDMAEngine()
{
    for (int i = 0; i < numDescDMAEngines; i++) {
        if (!descWbDMAEngineWorking[i]) {
            assert(!descWbDMAEngineDone[i]);
            return true;
        }
    }
    return false;
}

template<class T>
int
IGbE::DescCacheGlobal<T>::getFreeWbDMAEngine()
{
    for (int i = 0; i < numDescDMAEngines; i++) {
        if (!descWbDMAEngineWorking[i]) {
            assert(!descWbDMAEngineDone[i]);
            return i;
        }
    }
    return -1;
}

template<class T>
bool
IGbE::DescCacheGlobal<T>::hasFreeFetchDMAEngine()
{
    for (int i = 0; i < numDescDMAEngines; i++) {
        if (!descFetchDMAEngineWorking[i]) {
            assert(!descFetchDMAEngineDone[i]);
            return true;
        }
    }
    return false;
}

template<class T>
int
IGbE::DescCacheGlobal<T>::getFreeFetchDMAEngine()
{
    for (int i = 0; i < numDescDMAEngines; i++) {
        if (!descFetchDMAEngineWorking[i]) {
            assert(!descFetchDMAEngineDone[i]);
            return i;
        }
    }
    return -1;
}

template<class T>
void
IGbE::DescCacheGlobal<T>::writebackGlobal(Addr aMask)
{
    int curHead = descHead();
    int max_to_wb = usedCache.size();
    // Consider curWbDMAPnt as the head of the descriptor cache & curWbingNum
    assert(max_to_wb >= curWbingNum);
    max_to_wb -= curWbingNum;

    moreToWb = false;
    wbAlignment = aMask;

    DPRINTF(EthernetDesc, "Writing back descriptors head: %d tail: "
            "%d len: %d cachePnt: %d curWbDMAPnt: %d max_to_wb: %d usedCache: %d descleft: %d\n",
            curHead, descTail(), descLen(), cachePnt, curWbDMAPnt, max_to_wb,
            usedCache.size(), descLeft());
    
    if (max_to_wb + curWbDMAPnt >= descLen()) {
        max_to_wb = descLen() - curWbDMAPnt;
        moreToWb = true;
    } else if (wbAlignment != 0) {
        // align the wb point to the mask
        max_to_wb = max_to_wb & ~wbAlignment;
    }

    DPRINTF(EthernetDesc, "Writing back %d descriptors\n", max_to_wb);

    if (max_to_wb <= 0)
        return;
    
    // Find the free wb DMA engine
    if (!hasFreeWbDMAEngine()) {
        DPRINTF(EthernetDesc, "No free wb DMA engine\n");
        moreToWb = true;
        return;
    }
    int wbDMAEngineIdx = getFreeWbDMAEngine();
    assert(wbDMAEngineIdx != -1);

    // Set the wb DMA engine working
    descWbDMAEngineWorking[wbDMAEngineIdx] = true;
    descWbDMAEngineDone[wbDMAEngineIdx] = false;
    descWbBaseIdx[wbDMAEngineIdx] = curWbDMAPnt;
    descWbNum[wbDMAEngineIdx] = max_to_wb;
    // Copy usedCache to wbBufArray
    assert(curWbingNum + max_to_wb <= usedCache.size());
    for (int x = 0; x < max_to_wb; x++) {
        assert(usedCache.size());
        memcpy(&wbBufArray[wbDMAEngineIdx][x], usedCache[x + curWbingNum], sizeof(T));
    }

    #if LOG_LEVEL == 1 || WANT_TO_SEE == 1
    if (isRx) {
        printf("[LOG], %lu, RXDescWB_DMAE[%d]_R, Addr: %lx, Size: %d, (Head: %ld, Tail: %ld, WbDMAPnt: %d, WbDMAingNum: %d)\n",
            curTick(), wbDMAEngineIdx, pciToDma(descBase() + curWbDMAPnt * sizeof(T)), max_to_wb * sizeof(T), descHead(), descTail(), curWbDMAPnt, curWbingNum);
    } else {
        printf("[LOG], %lu, TXDescWB_DMAE[%d]_R, Addr: %lx, Size: %d, (Head: %ld, Tail: %ld, WbDMAPnt: %d, WbDMAingNum: %d)\n",
            curTick(), wbDMAEngineIdx, pciToDma(descBase() + curWbDMAPnt * sizeof(T)), max_to_wb * sizeof(T), descHead(), descTail(), curWbDMAPnt, curWbingNum);
    }
    for (int i = 0; i < max_to_wb; i++) {
        printf("wbBuf[%d]: %s ", i, wbBufArrayToString(wbDMAEngineIdx, i).c_str());
    }
    printf("\n");
    #endif

    descWbDMAEngines[wbDMAEngineIdx]->writebackRegister(pciToDma(descBase() + curWbDMAPnt * sizeof(T)),
            max_to_wb * sizeof(T), max_to_wb, wbBufArray[wbDMAEngineIdx]);
    
    // push to the wbDMAJobQueue
    // There should be no same DMA engine in the wbDMAJobQueue
    for (int i = 0; i < wbDMAJobQueue.size(); i++) {
        if (wbDMAJobQueue[i] == wbDMAEngineIdx) {
            DPRINTF(EthernetDesc, "WbDMAJobQueue already has the same DMA engine\n");
            assert(0 && "WbDMAJobQueue already has the same DMA engine");
        }
    }
    wbDMAJobQueue.push_back(wbDMAEngineIdx);

    curWbingNum += max_to_wb;
    curWbDMAPnt += max_to_wb;
    if (curWbDMAPnt >= descLen())
        curWbDMAPnt -= descLen();

}

template<class T>
void
IGbE::DescCacheGlobal<T>::fetchDescriptorsGlobal()
{
    size_t max_to_fetch;

    // Use curFetchDMAPnt instead of cachePnt
    if (descTail() >= curFetchDMAPnt)
        max_to_fetch = descTail() - curFetchDMAPnt;
    else {
        assert(descLen() >= curFetchDMAPnt);
        max_to_fetch = descLen() - curFetchDMAPnt;
    }

    // Also, add curFetchingNum to the totalUsed (it is reserved size of the cache)
    size_t totalUsed = usedCache.size() + unusedCache.size() + descProcessing() +
                       curFetchingNum;
    
    size_t free_cache = (static_cast<size_t>(size) >= totalUsed) ? (static_cast<size_t>(size) - totalUsed) : 0;

    max_to_fetch = std::min(max_to_fetch, free_cache);

    DPRINTF(EthernetDesc, "Fetching descriptors head: %d tail: "
            "%d len: %d cachePnt: %d curFetchDMAPnt: %d max_to_fetch: %d descleft: %d free_cache: %d\n",
            descHead(), descTail(), descLen(), cachePnt, curFetchDMAPnt,
            max_to_fetch, descLeft(), free_cache);
    DPRINTF(EthernetDpdk, "Fetching descriptors head: %d tail: "
            "%d len: %d cachePnt: %d curFetchDMAPnt: %d max_to_fetch: %d descleft: %d free_cache: %d\n",
            descHead(), descTail(), descLen(), cachePnt, curFetchDMAPnt,
            max_to_fetch, descLeft(), free_cache);
    // Nothing to do
    if (max_to_fetch == 0)
        return;
    
    // Check if we have a free fetch DMA engine
    if (!hasFreeFetchDMAEngine()) {
        DPRINTF(EthernetDesc, "No free fetch DMA engine\n");
        return;
    }
    int fetchDMAEngineIdx = getFreeFetchDMAEngine();
    assert(fetchDMAEngineIdx != -1);

    // Set the fetch DMA engine working
    descFetchDMAEngineWorking[fetchDMAEngineIdx] = true;
    descFetchDMAEngineDone[fetchDMAEngineIdx] = false;
    descFetchBaseIdx[fetchDMAEngineIdx] = curFetchDMAPnt;
    descFetchNum[fetchDMAEngineIdx] = max_to_fetch;
        
    DPRINTF(EthernetDesc, "Fetching %d descriptors from %d to %d\n",
            max_to_fetch, curFetchDMAPnt, descTail());
    #if LOG_LEVEL == 1 || LOG_LEVEL == 5
    if (isRx) {
        printf("[LOG], %lu, RXDescFetch_DMAE[%d]_R, Addr: %lx, Size: %d, (Head: %ld, Tail: %ld, FetchDMAPnt: %d, cachePnt: %d)\n",
            curTick(), fetchDMAEngineIdx, pciToDma(descBase() + curFetchDMAPnt * sizeof(T)), max_to_fetch * sizeof(T), descHead(), descTail(), curFetchDMAPnt, cachePnt);
    } else {
        printf("[LOG], %lu, TXDescFetch_DMAE[%d]_R, Addr: %lx, Size: %d, (Head: %ld, Tail: %ld, FetchDMAPnt: %d, cachePnt: %d)\n",
            curTick(), fetchDMAEngineIdx, pciToDma(descBase() + curFetchDMAPnt * sizeof(T)), max_to_fetch * sizeof(T), descHead(), descTail(), curFetchDMAPnt, cachePnt);
    }
    #endif

    // Register the fetch
    descFetchDMAEngines[fetchDMAEngineIdx]->fetchRegister(pciToDma(descBase() + curFetchDMAPnt * sizeof(T)),
            max_to_fetch * sizeof(T), max_to_fetch, fetchBufArray[fetchDMAEngineIdx]);
    
    // push to the fetchDMAJobQueue
    // There should be no same DMA engine in the fetchDMAJobQueue
    for (int i = 0; i < fetchDMAJobQueue.size(); i++) {
        if (fetchDMAJobQueue[i] == fetchDMAEngineIdx) {
            DPRINTF(EthernetDesc, "FetchDMAJobQueue already has the same DMA engine\n");
            assert(0 && "FetchDMAJobQueue already has the same DMA engine");
        }
    }
    fetchDMAJobQueue.push_back(fetchDMAEngineIdx);

    curFetchingNum += max_to_fetch;
    curFetchDMAPnt += max_to_fetch;

    if (curFetchDMAPnt >= descLen())
        curFetchDMAPnt -= descLen();

}

template<class T>
void
IGbE::DescCacheGlobal<T>::wbCompleteGlobal(int dmaEngineIdx)
{
    assert(dmaEngineIdx >= 0 && dmaEngineIdx < numDescDMAEngines);
    // Update the wb DMA engine status
    descWbDMAEngineDone[dmaEngineIdx] = true;

    igbe->etherDeviceStats.metaDMABytes += descWbNum[dmaEngineIdx] * sizeof(T);
    if (isRx) {
        igbe->etherDeviceStats.rxDescWBBytes += descWbNum[dmaEngineIdx] * sizeof(T);
    } else {
        igbe->etherDeviceStats.rxDescWBBytes += descWbNum[dmaEngineIdx] * sizeof(T);
    }

    // Update the global status by checking the wbDMAJobQueue from the front
    // Have to wait until front of the queue is done (in-order)
    assert(wbDMAJobQueue.size() > 0);
    if (wbDMAJobQueue.front() == dmaEngineIdx) {
        do {
            long curHead = descHead();
            long oldHead = curHead; 
            // pop the front of the queue
            int popIdx = wbDMAJobQueue.front();
            wbDMAJobQueue.pop_front();
            
            assert(descWbNum[popIdx] != -1);
            assert(curWbingNum >= descWbNum[popIdx]);
            curWbingNum -= descWbNum[popIdx];

            // pop usedCache & update the cachePnt
            for (int x = 0; x < descWbNum[popIdx]; x++) {
                assert(usedCache.size());
                delete usedCache[0];
                usedCache.pop_front();
            }

            // Update head
            curHead += descWbNum[popIdx];
            if (curHead >= descLen())
                curHead -= descLen();
            updateHead(curHead);

            DPRINTF(EthernetDesc, "Writeback complete curHead %d -> %d\n",
                    oldHead, curHead);
            
            #if LOG_LEVEL == 1 || WANT_TO_SEE == 1
            if (isRx) {
                printf("[LOG], %lu, WBRXDescComplete_DMAE[%d], OldHead: %ld, UpdatedHead: %ld, Tail: %ld, curWbDMAPnt: %ld\n", curTick(), popIdx, oldHead, descHead(), descTail(), curWbDMAPnt);
            } else {
                printf("[LOG], %lu, WBTXDescComplete_DMAE[%d], OldHead: %ld, UpdatedHead: %ld, Tail: %ld, curWbDMAPnt: %ld\n", curTick(), popIdx, oldHead, descHead(), descTail(), curWbDMAPnt);
            }
            #endif

            // update the wb DMA engine status
            descWbDMAEngineWorking[popIdx] = false;
            descWbDMAEngineDone[popIdx] = false;
            descWbBaseIdx[popIdx] = -1;
            descWbNum[popIdx] = -1;
            
        } while (wbDMAJobQueue.size() > 0 &&
                 descWbDMAEngineDone[wbDMAJobQueue.front()] == true);
    } else {
        DPRINTF(EthernetDesc, "WbDMAJobQueue front (%d) is not the same as the current DMA engine (%d)\n",
                wbDMAJobQueue.front(), dmaEngineIdx);
        // If the current DMA engine is not the same as the front of the queue,
    }

    // If we still have more to wb, call wb now
    actionAfterWb();
    if (moreToWb) {
        moreToWb = false;
        DPRINTF(EthernetDesc, "Writeback has more todo\n");
        writebackGlobal(wbAlignment);
    }

    igbe->checkDrain();
    fetchAfterWb();

}

template<class T>
void
IGbE::DescCacheGlobal<T>::fetchCompleteGlobal(int dmaEngineIdx)
{
    assert(dmaEngineIdx >= 0 && dmaEngineIdx < numDescDMAEngines);
    // Update the fetch DMA engine status
    descFetchDMAEngineDone[dmaEngineIdx] = true;

    igbe->etherDeviceStats.metaDMABytes += descFetchNum[dmaEngineIdx] * sizeof(T);
    if (isRx) {
        igbe->etherDeviceStats.rxDescFetchBytes += descFetchNum[dmaEngineIdx] * sizeof(T);
    } else {
        igbe->etherDeviceStats.txDescFetchBytes += descFetchNum[dmaEngineIdx] * sizeof(T);
    }

    // Update the global status by checking the fetchDMAJobQueue from the front
    // Have to wait until front of the queue is done (in-order)
    assert(fetchDMAJobQueue.size() > 0);
    if (fetchDMAJobQueue.front() == dmaEngineIdx) {
        do {
            int oldCp = cachePnt;
            // pop the front of the queue
            int popIdx = fetchDMAJobQueue.front();
            fetchDMAJobQueue.pop_front();

            assert(descFetchNum[popIdx] != -1);
            assert(curFetchingNum >= descFetchNum[popIdx]);
            curFetchingNum -= descFetchNum[popIdx];

            // push fetchBuf to unusedCache
            std::string printStr = "";
            T *newDesc;
            for (int x = 0; x < descFetchNum[popIdx]; x++) {
                newDesc = new T;
                memcpy(newDesc, &fetchBufArray[popIdx][x], sizeof(T));
                unusedCache.push_back(newDesc);
                printStr += "NewDesc[" + std::to_string(x) + "]: " + fetchBufToString(newDesc) + " | ";
            }

            // Update cachePnt
            cachePnt += descFetchNum[popIdx];
            assert(cachePnt <= descLen());
            if (cachePnt == descLen())
                cachePnt = 0;
            
            DPRINTF(EthernetDesc, "Fetching complete cachePnt %d -> %d\n",
                    oldCp, cachePnt);
            
            #if LOG_LEVEL == 1 || LOG_LEVEL == 5
            if (isRx) {
                printf("[LOG], %lu, RXDescFetchComplete_DMAE[%d], cachePnt: %d -> %d, Head: %ld, Tail: %ld, unusedDesc: %d\n %s\n",
                    curTick(), popIdx, oldCp, cachePnt, descHead(), descTail(), descUnused(), printStr.c_str());
            } else {
                printf("[LOG], %lu, TXDescFetchComplete_DMAE[%d], cachePnt: %d -> %d, Head: %ld, Tail: %ld, unusedDesc: %d\n %s\n",
                    curTick(), popIdx, oldCp, cachePnt, descHead(), descTail(), descUnused(), printStr.c_str());
            }
            #endif

            // update the fetch DMA engine status
            descFetchDMAEngineWorking[popIdx] = false;
            descFetchDMAEngineDone[popIdx] = false;
            descFetchBaseIdx[popIdx] = -1;
            descFetchNum[popIdx] = -1;

        } while (fetchDMAJobQueue.size() > 0 &&
                 descFetchDMAEngineDone[fetchDMAJobQueue.front()] == true);
    } else {
        DPRINTF(EthernetDesc, "FetchDMAJobQueue front (%d) is not the same as the current DMA engine (%d)\n",
                fetchDMAJobQueue.front(), dmaEngineIdx);
    }

    enableSm();
    igbe->checkDrain();
}

template<class T>
void
IGbE::DescCacheGlobal<T>::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(cachePnt);
    SERIALIZE_SCALAR(curFetching);
    SERIALIZE_SCALAR(wbOut);
    SERIALIZE_SCALAR(moreToWb);
    SERIALIZE_SCALAR(wbAlignment);

    typename CacheType::size_type usedCacheSize = usedCache.size();
    SERIALIZE_SCALAR(usedCacheSize);
    for (typename CacheType::size_type x = 0; x < usedCacheSize; x++) {
        arrayParamOut(cp, csprintf("usedCache_%d", x),
                      (uint8_t*)usedCache[x],sizeof(T));
    }

    // push back the processing cache to unused cache, regarding the sequence number
    // Sort the processing cache by sequence number
    CacheType mergedCache;

    std::vector<std::pair<int, T*>> processingCacheVec;
    for (int i = 0; i < MAX_DMA_ENGINE_SIZE; i++) {
        if (processingCache[i].desc != NULL) {
            processingCacheVec.push_back(std::make_pair(processingCache[i].seqIdx, processingCache[i].desc));
        }
    }
    std::sort(processingCacheVec.begin(), processingCacheVec.end(),
          [](const std::pair<int, T*>& a, const std::pair<int, T*>& b) {
              return a.first < b.first;
          });

    for (const auto& pair : processingCacheVec) {
        mergedCache.push_back(pair.second);
    }

    // merge unusedCache
    for (const auto& desc : unusedCache) {
        mergedCache.push_back(desc);
    }

    // Step 5: mergedCache serialize
    typename CacheType::size_type unusedCacheSize = mergedCache.size();
    SERIALIZE_SCALAR(unusedCacheSize);
    for (typename CacheType::size_type x = 0; x < unusedCacheSize; x++) {
        arrayParamOut(cp, csprintf("unusedCache_%d", x),
                    (uint8_t*)mergedCache[x], sizeof(T));
    }

    Tick fetch_delay = 0, wb_delay = 0;
    // if (fetchDelayEvent.scheduled())
    //     fetch_delay = fetchDelayEvent.when();
    SERIALIZE_SCALAR(fetch_delay);
    // if (wbDelayEvent.scheduled())
    //     wb_delay = wbDelayEvent.when();
    SERIALIZE_SCALAR(wb_delay);


}

template<class T>
void
IGbE::DescCacheGlobal<T>::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(cachePnt);
    UNSERIALIZE_SCALAR(curFetching);
    UNSERIALIZE_SCALAR(wbOut);
    UNSERIALIZE_SCALAR(moreToWb);
    UNSERIALIZE_SCALAR(wbAlignment);

    curFetchDMAPnt = cachePnt;

    typename CacheType::size_type usedCacheSize;
    UNSERIALIZE_SCALAR(usedCacheSize);
    T *temp;
    for (typename CacheType::size_type x = 0; x < usedCacheSize; x++) {
        temp = new T;
        arrayParamIn(cp, csprintf("usedCache_%d", x),
                     (uint8_t*)temp,sizeof(T));
        usedCache.push_back(temp);
    }

    typename CacheType::size_type unusedCacheSize;
    UNSERIALIZE_SCALAR(unusedCacheSize);
    for (typename CacheType::size_type x = 0; x < unusedCacheSize; x++) {
        temp = new T;
        arrayParamIn(cp, csprintf("unusedCache_%d", x),
                     (uint8_t*)temp,sizeof(T));
        unusedCache.push_back(temp);
    }
    Tick fetch_delay = 0, wb_delay = 0;
    UNSERIALIZE_SCALAR(fetch_delay);
    UNSERIALIZE_SCALAR(wb_delay);
    // if (fetch_delay)
    //     igbe->schedule(fetchDelayEvent, fetch_delay);
    // if (wb_delay)
    //     igbe->schedule(wbDelayEvent, wb_delay);


}

///////////////////////////// IGbE::RxDescCache //////////////////////////////

IGbE::RxDescCache::RxDescCache(IGbE *i, const std::string n, int qid, int engineIdx, RxDescCacheGlobal *_parent)
    : DescCache<RxDesc>(i, n, true, engineIdx), pktWaiting(false),
    splitCount(0), queueID(qid), parent(_parent), bytesCopied(0), correspondingDesc(NULL),
    pktEvent([this]{ pktComplete(); }, n),
    pktHdrEvent([this]{ pktSplitDone(); }, n),
    pktDataEvent([this]{ pktSplitDone(); }, n)
{
}

void
IGbE::RxDescCache::pktSplitDone()
{
    splitCount++;
    DPRINTF(EthernetDesc,
            "Part of split packet done: splitcount now %d\n", splitCount);
    assert(splitCount <= 2);
    if (splitCount != 2)
        return;
    splitCount = 0;
    DPRINTF(EthernetDesc,
            "Part of split packet done: calling pktComplete()\n");
    pktComplete();
}

int
IGbE::RxDescCache::writePacket(EthPacketPtr packet, int pkt_offset, igbreg::RxDesc* rxDesc)
{
    assert(correspondingDesc == NULL);
    correspondingDesc = rxDesc;

    pktPtr = packet;
    pktWaiting = true;

    unsigned buf_len, hdr_len;

    if (pktPtr->rxDMAStartTick == 0)
        pktPtr->rxDMAStartTick = curTick();

    switch (igbe->regs.srrctl_array[queueID].desctype()) {
      case RXDT_LEGACY:
        assert(pkt_offset == 0);
        bytesCopied = packet->length;
        DPRINTF(EthernetDesc, "LEGACY Packet Length: %d Desc Size: %d\n",
                packet->length, igbe->regs.rctl.descSize());
        DPRINTF(EthernetDpdk, "RXD[%d] LEGACY Packet Length: %d Desc Size: %d\n",
                queueID, packet->length, igbe->regs.rctl.descSize());
        assert(packet->length < igbe->regs.rctl.descSize());
        igbe->dmaWrite(pciToDma(correspondingDesc->legacy.buf),
                       packet->length, &pktEvent, packet->data,
                       igbe->rxWriteDelay);
        break;
      case RXDT_ADV_ONEBUF:
        assert(pkt_offset == 0);
        bytesCopied = packet->length;
        buf_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl_array[queueID].bufLen() :
            igbe->regs.rctl.descSize();
        DPRINTF(EthernetDesc, "ADV_ONEBUF LPE: %d Packet Length: %d srrctl: %#x Desc Size: %d\n",
                igbe->regs.rctl.lpe(), packet->length, igbe->regs.srrctl_array[queueID](), buf_len);
        DPRINTF(EthernetDpdk, "RXD[%d] ADV_ONEBUF LPE: %d Packet Length: %d srrctl: %#x Desc Size: %d\n",
                queueID, igbe->regs.rctl.lpe(), packet->length, igbe->regs.srrctl_array[queueID](), buf_len);
        assert(packet->length < buf_len);
        // SHIN. Change to IDIO
        // igbe->dmaWrite(pciToDma(correspondingDesc->adv_read.pkt),
        //                packet->length, &pktEvent, packet->data,
        //                igbe->rxWriteDelay);
        // printf("RXD[%d] At clk: %ld IdioWrite pktPtr: %p, Addr: %lx, Size: %d, Delay: %ld\n",
        //         queueID, curTick(), pktPtr, pciToDma(correspondingDesc->adv_read.pkt),
        //         packet->length, igbe->rxWriteDelay);
        
        #if LOG_LEVEL == 1
        if (descTail() == 63 || descTail() == 127 || descTail() == 191 || descTail() == 255) {
            printf("[LOG], %lu, NIC_WR_RX_PKT_TO_MBUF_DMAE[%d], %lx, %d, Head: %d, Tail: %d\n", curTick(), dmaEngineIdx, pciToDma(correspondingDesc->adv_read.pkt), packet->length, descHead(), descTail());
        }
        #endif

        igbe->IdioWrite(pciToDma(correspondingDesc->adv_read.pkt),
                       packet->length, &pktEvent, packet->data,
                       igbe->rxWriteDelay, 0, igbe->adq);

        correspondingDesc->adv_wb.header_len = htole(0);
        correspondingDesc->adv_wb.sph = htole(0);
        correspondingDesc->adv_wb.pkt_len = htole((uint16_t)(pktPtr->length));
        break;
      case RXDT_ADV_SPLIT_A:
        int split_point;

        buf_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl_array[queueID].bufLen() :
            igbe->regs.rctl.descSize();
        hdr_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl_array[queueID].hdrLen() : 0;
        DPRINTF(EthernetDesc,
                "lpe: %d Packet Length: %d offset: %d srrctl: %#x "
                "hdr addr: %#x Hdr Size: %d correspondingDesc addr: %#x Desc Size: %d\n",
                igbe->regs.rctl.lpe(), packet->length, pkt_offset,
                igbe->regs.srrctl_array[queueID](), correspondingDesc->adv_read.hdr, hdr_len,
                correspondingDesc->adv_read.pkt, buf_len);
        DPRINTF(EthernetDpdk,
                "RXD[%d] lpe: %d Packet Length: %d offset: %d srrctl: %#x "
                "hdr addr: %#x Hdr Size: %d correspondingDesc addr: %#x Desc Size: %d\n",
                queueID, igbe->regs.rctl.lpe(), packet->length, pkt_offset,
                igbe->regs.srrctl_array[queueID](), correspondingDesc->adv_read.hdr, hdr_len,
                correspondingDesc->adv_read.pkt, buf_len);

        split_point = hsplit(pktPtr);

        if (packet->length <= hdr_len) {
            bytesCopied = packet->length;
            assert(pkt_offset == 0);
            DPRINTF(EthernetDesc, "Hdr split: Entire packet in header\n");
            // SHIN. Change To IDIO
            // igbe->dmaWrite(pciToDma(correspondingDesc->adv_read.hdr),
            //                packet->length, &pktEvent, packet->data,
            //                igbe->rxWriteDelay);
            igbe->IdioWrite(pciToDma(correspondingDesc->adv_read.hdr),
                           packet->length, &pktEvent, packet->data,
                           igbe->rxWriteDelay, 0, igbe->adq);

            correspondingDesc->adv_wb.header_len = htole((uint16_t)packet->length);
            correspondingDesc->adv_wb.sph = htole(0);
            correspondingDesc->adv_wb.pkt_len = htole(0);
        } else if (split_point) {
            if (pkt_offset) {
                // we are only copying some data, header/data has already been
                // copied
                int max_to_copy =
                    std::min(packet->length - pkt_offset, buf_len);
                bytesCopied += max_to_copy;
                DPRINTF(EthernetDesc,
                        "Hdr split: Continuing data buffer copy\n");
                
                // SHIN
                // igbe->dmaWrite(pciToDma(correspondingDesc->adv_read.pkt),
                //                max_to_copy, &pktEvent,
                //                packet->data + pkt_offset, igbe->rxWriteDelay);

                igbe->IdioWrite(pciToDma(correspondingDesc->adv_read.pkt),
                               max_to_copy, &pktEvent,
                               packet->data + pkt_offset, igbe->rxWriteDelay,
                               0, igbe->adq);

                correspondingDesc->adv_wb.header_len = htole(0);
                correspondingDesc->adv_wb.pkt_len = htole((uint16_t)max_to_copy);
                correspondingDesc->adv_wb.sph = htole(0);
            } else {
                int max_to_copy =
                    std::min(packet->length - split_point, buf_len);
                bytesCopied += max_to_copy + split_point;

                DPRINTF(EthernetDesc, "Hdr split: splitting at %d\n",
                        split_point);
                // SHIN
                // igbe->dmaWrite(pciToDma(correspondingDesc->adv_read.hdr),
                //                split_point, &pktHdrEvent,
                //                packet->data, igbe->rxWriteDelay);
                // igbe->dmaWrite(pciToDma(correspondingDesc->adv_read.pkt),
                //                max_to_copy, &pktDataEvent,
                //                packet->data + split_point, igbe->rxWriteDelay);
                igbe->IdioWrite(pciToDma(correspondingDesc->adv_read.hdr),
                               split_point, &pktHdrEvent,
                               packet->data, igbe->rxWriteDelay,
                               0, igbe->adq);
                igbe->IdioWrite(pciToDma(correspondingDesc->adv_read.pkt),
                               max_to_copy, &pktDataEvent,
                               packet->data + split_point, igbe->rxWriteDelay,
                               0, igbe->adq);
                correspondingDesc->adv_wb.header_len = htole(split_point);
                correspondingDesc->adv_wb.sph = 1;
                correspondingDesc->adv_wb.pkt_len = htole((uint16_t)(max_to_copy));
            }
        } else {
            panic("Header split not fitting within header buffer or "
                  "undecodable packet not fitting in header unsupported\n");
        }
        break;
      default:
        panic("Unimplemnted RX receive buffer type: %d\n",
              igbe->regs.srrctl_array[queueID].desctype());
    }
    return bytesCopied; // TODO: have to deal with this bytesCopied for each DMA engine within RxDescCacheGlobal - make array or..

}

void
IGbE::RxDescCache::pktComplete()
{
    assert(correspondingDesc!=NULL);

    uint16_t crcfixup = igbe->regs.rctl.secrc() ? 0 : 4 ;
    DPRINTF(EthernetDesc, "RXD[%d] pktPtr->length: %d bytesCopied: %d "
            "stripcrc offset: %d value written: %d %d\n",
            queueID, pktPtr->length, bytesCopied, crcfixup,
            htole((uint16_t)(pktPtr->length + crcfixup)),
            (uint16_t)(pktPtr->length + crcfixup));
    
    igbe->etherDeviceStats.rxDMABytes += bytesCopied;

    // no support for anything but starting at 0
    assert(igbe->regs.rxcsum.pcss() == 0);

    DPRINTF(EthernetDesc, "RXD[%d] Packet written to memory updating Descriptor\n", queueID);
    DPRINTF(EthernetDpdk, "RXD[%d] Packet written to memory updating Descriptor\n", queueID);

    #if LOG_LEVEL == 1
    if (descTail() == 63 || descTail() == 127 || descTail() == 191 || descTail() == 255) {
        printf("[LOG], %lu, NIC_WR_RESP_RX_PKT_TO_MBUF_DMAE[%d], Head: %d, Tail: %d\n", curTick(), dmaEngineIdx, descHead(), descTail());
    }
    #endif

    // printf("RXD[%d] At clk: %ld pktComplete() pktPtr: %p\n", queueID, curTick(), pktPtr);

    uint16_t status = RXDS_DD;
    uint8_t err = 0;
    uint16_t ext_err = 0;
    uint16_t csum = 0;
    uint16_t ptype = 0;
    uint16_t ip_id = 0;

    assert(bytesCopied <= pktPtr->length);
    if (bytesCopied == pktPtr->length)
        status |= RXDS_EOP;

    IpPtr ip(pktPtr);
    Ip6Ptr ip6(pktPtr);

    if (ip || ip6) {
        if (ip) {
            DPRINTF(EthernetDesc, "RXD[%d] Proccesing Ip packet with Id=%d\n",
                    queueID, ip->id());
            ptype |= RXDP_IPV4;
            ip_id = ip->id();
        }
        if (ip6)
            ptype |= RXDP_IPV6;

        if (ip && igbe->regs.rxcsum.ipofld()) {
            DPRINTF(EthernetDesc, "RXD[%d] Checking IP checksum\n", queueID);
            status |= RXDS_IPCS;
            csum = htole(cksum(ip));
            igbe->etherDeviceStats.rxIpChecksums++;
            if (cksum(ip) != 0) {
                err |= RXDE_IPE;
                ext_err |= RXDEE_IPE;
                DPRINTF(EthernetDesc, "RXD[%d] Checksum is bad!!\n", queueID);
            }
        }
        TcpPtr tcp = ip ? TcpPtr(ip) : TcpPtr(ip6);
        if (tcp && igbe->regs.rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "RXD[%d] Checking TCP checksum\n", queueID);
            status |= RXDS_TCPCS;
            ptype |= RXDP_TCP;
            csum = htole(cksum(tcp));
            igbe->etherDeviceStats.rxTcpChecksums++;
            if (cksum(tcp) != 0) {
                DPRINTF(EthernetDesc, "RXD[%d] Checksum is bad!!\n", queueID);
                err |= RXDE_TCPE;
                ext_err |= RXDEE_TCPE;
            }
        }

        UdpPtr udp = ip ? UdpPtr(ip) : UdpPtr(ip6);
        if (udp && igbe->regs.rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "RXD[%d] Checking UDP checksum\n", queueID);
            status |= RXDS_UDPCS;
            ptype |= RXDP_UDP;
            csum = htole(cksum(udp));
            igbe->etherDeviceStats.rxUdpChecksums++;
            if (cksum(udp) != 0) {
                DPRINTF(EthernetDesc, "RXD[%d] Checksum is bad!!\n", queueID);
                ext_err |= RXDEE_TCPE;
                err |= RXDE_TCPE;
            }
        }
    } else { // if ip
        DPRINTF(EthernetSM, "Proccesing Non-Ip packet\n");
    }

    switch (igbe->regs.srrctl_array[queueID].desctype()) {
      case RXDT_LEGACY:
        correspondingDesc->legacy.len = htole((uint16_t)(pktPtr->length + crcfixup));
        correspondingDesc->legacy.status = htole(status);
        correspondingDesc->legacy.errors = htole(err);
        // No vlan support at this point... just set it to 0
        correspondingDesc->legacy.vlan = 0;
        DPRINTF(EthernetDesc, "RXD[%d] Descriptor LEGACY complete len: %#x status: %#x\n",
                queueID, correspondingDesc->legacy.len, correspondingDesc->legacy.status);
        break;
      case RXDT_ADV_SPLIT_A:
      case RXDT_ADV_ONEBUF:
        correspondingDesc->adv_wb.rss_type = htole(0);
        correspondingDesc->adv_wb.pkt_type = htole(ptype);
        if (igbe->regs.rxcsum.pcsd()) {
            // no rss support right now
            correspondingDesc->adv_wb.rss_hash = htole(0);
        } else {
            correspondingDesc->adv_wb.id = htole(ip_id);
            correspondingDesc->adv_wb.csum = htole(csum);
        }
        correspondingDesc->adv_wb.status = htole(status);
        correspondingDesc->adv_wb.errors = htole(ext_err);
        // no vlan support
        correspondingDesc->adv_wb.vlan_tag = htole(0);
        break;
      default:
        panic("Unimplemnted RX receive buffer type %d\n", queueID,
              igbe->regs.srrctl_array[queueID].desctype());
    }

    DPRINTF(EthernetDesc, "RXD[%d] Descriptor complete w0: %#x w1: %#x\n",
            queueID, correspondingDesc->adv_read.pkt, correspondingDesc->adv_read.hdr);

    if (bytesCopied == pktPtr->length) {
        DPRINTF(EthernetDesc,
                "RXD[%d] Packet completely written to descriptor buffers\n", queueID);
        DPRINTF(EthernetDpdk,
                "RXD[%d] Packet completely written to descriptor buffers\n", queueID);
        
        if (pktPtr->rxDMAEndTick == 0)
            pktPtr->rxDMAEndTick = curTick();
        
        if (pktPtr->rxMadeTick != 0 && pktPtr->rxPortTick != 0 && pktPtr->rxFifoTick != 0 && pktPtr->rxDMAStartTick != 0 && pktPtr->rxDMAEndTick != 0) {
            uint64_t rxTotalTime = pktPtr->rxDMAEndTick - pktPtr->rxMadeTick;
            uint64_t rxEtherLinkTime = pktPtr->rxPortTick - pktPtr->rxMadeTick;
            uint64_t rxPort2FifoTime = pktPtr->rxFifoTick - pktPtr->rxPortTick;
            uint64_t rxFifo2DMAStartTime = pktPtr->rxDMAStartTick - pktPtr->rxFifoTick;
            uint64_t rxDMATime = pktPtr->rxDMAEndTick - pktPtr->rxDMAStartTick;
            // printf("RXD[%d] RX Total Time: %ld, EtherLink Time: %ld, Port2Fifo Time: %ld, Fifo2DMAStart Time: %ld, DMA Time: %ld\n", queueID, rxTotalTime, rxEtherLinkTime, rxPort2FifoTime, rxFifo2DMAStartTime, rxDMATime);
            float rxTotalTimeInSec = (float)rxTotalTime / 10.0e12;
            float rxEtherLinkTimeInSec = (float)rxEtherLinkTime / 10.0e12;
            float rxPort2FifoTimeInSec = (float)rxPort2FifoTime / 10.0e12;
            float rxFifo2DMAStartTimeInSec = (float)rxFifo2DMAStartTime / 10.0e12;
            float rxDMATimeInSec = (float)rxDMATime / 10.0e12;
        }
    }

    pktPtr = NULL;
    pktWaiting = false;
    correspondingDesc = NULL;

    DPRINTF(EthernetDesc, "RXD[%d] Processing of this descriptor complete\n", queueID);
    DPRINTF(EthernetDpdk, "RXD[%d] Processing of this descriptor complete\n", queueID);
    
    // call RxDescCacheGlobal to return the descriptor
    parent->onDMAComplete(dmaEngineIdx);
}

bool
IGbE::RxDescCache::hasOutstandingEvents()
{
    return pktEvent.scheduled() || pktHdrEvent.scheduled() ||
        pktDataEvent.scheduled();

}

///////////////////////////// IGbE::RxDescCacheGlobal //////////////////////////////

IGbE::RxDescCacheGlobal::RxDescCacheGlobal(IGbE *i, const std::string n, int s, int qid, int _numDMAEngines, int _numDescDMAEngines)
    : DescCacheGlobal<RxDesc>(i, n, s, true, _numDMAEngines, _numDescDMAEngines), queueID(qid),
    _rdtrEvent([this]{ _rdtrProcess(); }, n),
    _radvEvent([this]{ _radvProcess(); }, n)

{
    annSmFetch = "RX Desc Fetch";
    annSmWb = "RX Desc Writeback";
    annUnusedDescQ = "RX Unused Descriptors";
    annUnusedCacheQ = "RX Unused Descriptor Cache";
    annUsedCacheQ = "RX Used Descriptor Cache";
    annUsedDescQ = "RX Used Descriptors";
    annDescQ = "RX Descriptors";

    // initialize the DMA engine
    assert(numDMAEngines > 0);
    printf("==================RxDescCacheGlobal with %d DMA Engines %d desc DMA Engines==================\n", numDMAEngines, numDescDMAEngines);
    for (int engine_idx = 0; engine_idx < numDMAEngines; engine_idx++) {
        // Treat original RxDescCache as the DMA engine
        std::string dmaEngineName = n + ".DMAEngine" + std::to_string(engine_idx);
        dmaEngineArray[engine_idx] = new RxDescCache(i, dmaEngineName, qid, engine_idx, this);
        processingPktOffsetArray[engine_idx] = 0;
        processingPktDoneArray[engine_idx] = false;
    }
    
}

bool 
IGbE::RxDescCacheGlobal::writePacketGlobal(EthPacketPtr packet)
{
    // First receive ethpacket and will choose the DMA engine to write the packet
    if (!unusedCache.size()) {
        // This can happen, because DMA engine will use the descriptors in the unusedCache, 
        // and it is counted as descUnused(), but new DMA engine cannot use. 
        // So we have to wait until descUnused() becomes 0, which means previous work is done.
        return false;
    }

    // Choose the DMA engine to write the packet
    assert(hasFreeDMAEngine());
    int idx = getFreeDMAEngine();
    assert(idx != -1);
    assert(processingPktArray[idx] == NULL);
    assert(processingPktOffsetArray[idx] == 0);
    assert(processingPktDoneArray[idx] == false);
    assert(processingCache[idx].desc == NULL);
    processingPktArray[idx] = packet;
    RxDesc *desc = unusedCache.front();
    processingCache[idx].desc = desc;
    processingCache[idx].seqIdx = nextSeqToProcess++;
    processingCache[idx].done = false;
    unusedCache.pop_front();

    // printf("[LOG], %lu, RXD[%d]_Use_DMAEngine[%d]_For_Packet_with_seqIdx_%d\n"
    //         , curTick(), queueID, idx, processingCache[idx].seqIdx);

    // Call the writePacket function of the DMA engine
    processingPktOffsetArray[idx] = dmaEngineArray[idx]->writePacket(packet, 0, desc);
    DPRINTF(EthernetSM, "RXS[%d]: Writing packet into memory\n", queueID);
    DPRINTF(EthernetDpdk, "RXS[%d]: Writing packet into memory\n", queueID);

    return true;
}

void
IGbE::RxDescCacheGlobal::onDMAComplete(int engineIdx)
{
    assert(engineIdx < numDMAEngines);
    RxDescCache *dmaEngine = dmaEngineArray[engineIdx];
    assert(dmaEngine != NULL);

    // Get the info from the DMA engine
    EthPacketPtr pktPtr = processingPktArray[engineIdx];
    assert(pktPtr != NULL);
    unsigned bytesCopied = dmaEngine->getBytesCopied();
    assert(bytesCopied > 0);

    if (bytesCopied == pktPtr->length) {
        // This is end of processing ethPacket
        DPRINTF(EthernetSM, "RXS[%d]: Packet completely written to memory\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: Packet completely written to memory\n", queueID);
        
        // Update the info of DMA engine
        processingPktOffsetArray[engineIdx] = 0;
        processingPktDoneArray[engineIdx] = true;
        processingPktArray[engineIdx] = NULL;
        // Reset the bytes copied
        dmaEngine->setBytesCopied(0);

        // Set done the processing descriptor
        processingCache[engineIdx].done = true;
        RxDesc *desc = processingCache[engineIdx].desc;
        assert(desc != NULL);

        // printf("[LOG], %lu, RXD[%d]_DMAEngine[%d]_Done_with_seqIdx_%d\n"
        //     , curTick(), queueID, engineIdx, processingCache[engineIdx].seqIdx);

        // Call to the manage the used cache and the processingCache to meet the sequence
        manageUsedCache();
        

        // Deal with the rx timer interrupts
        if (igbe->regs.rdtr.delay()) {
            Tick delay = igbe->regs.rdtr.delay() * igbe->intClock();
            DPRINTF(EthernetSM, "RXS[%d]: Scheduling DTR for %d\n", queueID, delay);
            igbe->reschedule(_rdtrEvent, curTick() + delay);
        }

        if (igbe->regs.radv.idv()) {
            Tick delay = igbe->regs.radv.idv() * igbe->intClock();
            DPRINTF(EthernetSM, "RXS[%d]: Scheduling ADV for %d\n", queueID, delay);
            if (!_radvEvent.scheduled()) {
                igbe->schedule(_radvEvent, curTick() + delay);
            }
        }

        // if neither radv or rdtr, maybe itr is set...
        if (!igbe->regs.rdtr.delay() && !igbe->regs.radv.idv()) {
            DPRINTF(EthernetSM,
                    "RXS: Receive interrupt delay disabled, posting IT_RXT\n");
            igbe->postInterrupt(IT_RXT);
        }

        // If the packet is small enough, interrupt appropriately
        // I wonder if this is delayed or not?!
        if (pktPtr->length <= igbe->regs.rsrpd.idv()) {
            DPRINTF(EthernetSM,
                    "RXS: Posting IT_SRPD beacuse small packet received\n");
            igbe->postInterrupt(IT_SRPD);
        }        
    } else {
        // This is not end of processing ethPacket
        DPRINTF(EthernetSM, "RXS[%d]: Packet partially written to memory\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: Packet partially written to memory\n", queueID);
        // Update the info of DMA engine
        assert(0 && "Not implemented yet");
    }

    igbe->checkDrain();
    enableSm();
}

int 
IGbE::RxDescCacheGlobal::hasReadyEthPacket()
{
    // Check each DMA engine has ready packet
    // Ready packet: processingPktDoneArray[engineIdx] == true
    for (int engine_idx = 0; engine_idx < numDMAEngines; engine_idx++) {
        if (processingPktDoneArray[engine_idx]) {
            return engine_idx;
        }
    }
    return -1;
}
        
void 
IGbE::RxDescCacheGlobal::clearDoneEthPacket(int engineIdx)
{   
    processingPktDoneArray[engineIdx] = false;
    assert(processingPktArray[engineIdx] == NULL);
    assert(processingPktOffsetArray[engineIdx] == 0);
    // assert(processingCache[engineIdx].desc == NULL); // This can be not NULL if the mbuf DMA is not done sequentially

}

void
IGbE::RxDescCacheGlobal::manageUsedCache()
{
    bool progress = true;
    while (progress) {
        progress = false;

        for (int engine_idx = 0; engine_idx < numDMAEngines; engine_idx++) {
            if (processingCache[engine_idx].done &&
                processingCache[engine_idx].seqIdx == nextSeqToWriteback) {

                RxDesc *desc = processingCache[engine_idx].desc;
                assert(desc != NULL);

                usedCache.push_back(desc);
                processingCache[engine_idx].desc = NULL;
                processingCache[engine_idx].seqIdx = 0;
                processingCache[engine_idx].done = false;

                // printf("[LOG], %lu, RXD[%d]_Push_ProcessingCache[%d]_To_UsedCache_with_seqIdx_%d\n"
                //     , curTick(), queueID, engine_idx, nextSeqToWriteback);
                nextSeqToWriteback++;
                progress = true; // made progress, need to scan again
                break; // restart loop to recheck all engines
            }
        }
    }
}

void
IGbE::RxDescCacheGlobal::enableSm()
{
    if (igbe->drainState() != DrainState::Draining) {
        igbe->rxTick = true;
        igbe->restartClock();
    }
}

bool
IGbE::RxDescCacheGlobal::hasOutstandingEvents()
{
    bool descCacheGlobalOutstanding = false;
    // Check all desc DMA engines
    for (int engine_idx = 0; engine_idx < numDescDMAEngines; engine_idx++) {
        descCacheGlobalOutstanding |= descFetchDMAEngines[engine_idx]->hasOutstandingEvents();
        descCacheGlobalOutstanding |= descWbDMAEngines[engine_idx]->hasOutstandingEvents();
    }
    // Have to check all DMA engines
    for (int engine_idx = 0; engine_idx < numDMAEngines; engine_idx++) {
        descCacheGlobalOutstanding |= dmaEngineArray[engine_idx]->hasOutstandingEvents();
    }
    return descCacheGlobalOutstanding;

}

void
IGbE::RxDescCacheGlobal::serialize(CheckpointOut &cp) const
{
    DescCacheGlobal<RxDesc>::serialize(cp);

    Tick _rdtr_time = 0, _radv_time = 0;

    if (_rdtrEvent.scheduled())
        _rdtr_time = _rdtrEvent.when();
    SERIALIZE_SCALAR(_rdtr_time);
    if (_radvEvent.scheduled())
        _radv_time = _radvEvent.when();
    SERIALIZE_SCALAR(_radv_time);
}

void
IGbE::RxDescCacheGlobal::unserialize(CheckpointIn &cp)
{
    DescCacheGlobal<RxDesc>::unserialize(cp);

    Tick _rdtr_time = 0, _radv_time = 0;
    UNSERIALIZE_SCALAR(_rdtr_time);
    UNSERIALIZE_SCALAR(_radv_time);
    if (_rdtr_time)
        igbe->schedule(_rdtrEvent, _rdtr_time);
    if (_radv_time)
        igbe->schedule(_radvEvent, _radv_time);
}


///////////////////////////// IGbE::TxDescCache //////////////////////////////

IGbE::TxDescCache::TxDescCache(IGbE *i, const std::string n, int qid, int engineIdx, TxDescCacheGlobal *_parent)
    : DescCache<TxDesc>(i,n, false, engineIdx), pktDone(false), parent(_parent),
      pktWaiting(false), queueID(qid),
    pktEvent([this]{ pktComplete(); }, n)
{

}

void
IGbE::TxDescCache::getPacketData(EthPacketPtr p, Addr addr, int size)
{
    assert(pktPtr == NULL);
    assert(pktWaiting == false);

    pktPtr = p;
    dmaAddr = addr;

    pktWaiting = true;

    DPRINTF(EthernetDesc, "TXD[%d] Starting DMA of packet at offset %d\n", queueID, p->length);

    #if LOG_LEVEL == 1
    if (descTail() == 128 || descTail() == 192 || descTail() == 256 || descTail() == 320) {
        printf("[LOG], %lu, NIC_RD_REQ_TX_MBUF_DMAE[%d], %lx, %d, Head: %d, Tail: %d\n", curTick(), dmaEngineIdx, addr, size, descHead(), descTail());
    }
    #endif
    igbe->dmaRead(addr,
                    size, &pktEvent, p->data + p->length,
                    igbe->txReadDelay);

}

void
IGbE::TxDescCache::pktComplete()
{
    assert(pktPtr);

    DPRINTF(EthernetDesc, "TXD[%d] DMAE[%d] of packet complete\n", queueID, dmaEngineIdx);
    DPRINTF(EthernetDpdk, "TXD[%d] DMAE[%d] of packet complete\n", queueID, dmaEngineIdx);

    // printf("TXD[%d] At clk: %ld pktComplete() pktPtr: %p\n", queueID, curTick(), pktPtr);

    #if LOG_LEVEL == 1
    if (descTail() == 128 || descTail() == 192 || descTail() == 256 || descTail() == 320) {
        printf("[LOG], %lu, NIC_RD_RESP_TX_MBUF_DMAE[%d], %lx, Head: %d, Tail: %d\n", curTick(), dmaEngineIdx, dmaAddr, descHead(), descTail());
    }
    #endif
    
    // Call the global level to process the descriptor
    parent->onDMAComplete(dmaEngineIdx);
    pktPtr = NULL;
    dmaAddr = 0;
    return;
}

bool
IGbE::TxDescCache::packetAvailable()
{
    if (pktDone) {
        // pktDone = false; -> it is set through unsetPacketDone()
        return true;
    }
    return false;
}

bool
IGbE::TxDescCache::hasOutstandingEvents()
{
    return pktEvent.scheduled();
}

////////////////////////////// IGbE::TxDescCacheGlobal //////////////////////////////
IGbE::TxDescCacheGlobal::TxDescCacheGlobal(IGbE *i, const std::string n, int s, int qid, int _numDMAEngines, int _numDescDMAEngines)
    : DescCacheGlobal<TxDesc>(i,n, s, false, _numDMAEngines, _numDescDMAEngines), queueID(qid), roundRobinIdx(0),
      isTcp(false), pktHdrWaiting(false), pktMultiDesc(false),
      completionAddress(0), completionEnabled(false),
      useTso(false), tsoHeaderLen(0), tsoMss(0), tsoTotalLen(0), tsoUsedLen(0),
      tsoPrevSeq(0), tsoPktPayloadBytes(0), tsoLoadedHeader(false),
      tsoPktHasHeader(false), tsoDescBytesUsed(0), tsoCopyBytes(0), tsoPkts(0), tsoDMAEngineIdx(-1),
    headerEvent([this]{ headerComplete(); }, n),
    nullEvent([this]{ nullCallback(); }, n),
    _tadvEvent([this]{ _tadvProcess(); }, n),
    _tidvEvent([this]{ _tidvProcess(); }, n)
{
    annSmFetch = "TX Desc Fetch";
    annSmWb = "TX Desc Writeback";
    annUnusedDescQ = "TX Unused Descriptors";
    annUnusedCacheQ = "TX Unused Descriptor Cache";
    annUsedCacheQ = "TX Used Descriptor Cache";
    annUsedDescQ = "TX Used Descriptors";
    annDescQ = "TX Descriptors";

    // initialize the DMA engine
    assert(numDMAEngines > 0);
    printf("==================TxDescCacheGlobal with %d DMA Engines %d desc DMA Engines==================\n", numDMAEngines, numDescDMAEngines);
    for (int engine_idx = 0; engine_idx < numDMAEngines; engine_idx++) {
        // Treat original TxDescCache as the DMA engine
        std::string dmaEngineName = n + ".DMAEngine" + std::to_string(engine_idx);
        dmaEngineArray[engine_idx] = new TxDescCache(i, dmaEngineName, qid, engine_idx, this);
    }
}

void
IGbE::TxDescCacheGlobal::processContextDesc()
{
    TxDesc *desc;

    DPRINTF(EthernetDesc, "TXD[%d] Checking and  processing context descriptors\n", queueID);

    while (!useTso && unusedCache.size() &&
           txd_op::isContext(unusedCache.front())) {
        DPRINTF(EthernetDesc, "TXD[%d] Got context descriptor type...\n", queueID);

        desc = unusedCache.front();
        DPRINTF(EthernetDesc, "TXD[%d] Descriptor upper: %#x lower: %#X\n", queueID,
                desc->d1, desc->d2);


        // is this going to be a tcp or udp packet?
        isTcp = txd_op::tcp(desc) ? true : false;

        // setup all the TSO variables, they'll be ignored if we don't use
        // tso for this connection
        tsoHeaderLen = txd_op::hdrlen(desc);
        tsoMss  = txd_op::mss(desc);

        if (txd_op::isType(desc, txd_op::TXD_CNXT) && txd_op::tse(desc)) {
            DPRINTF(EthernetDesc, "TXD[%d] TCP offload enabled for packet hdrlen: "
                    "%d mss: %d paylen %d\n", queueID, txd_op::hdrlen(desc),
                    txd_op::mss(desc), txd_op::getLen(desc));
            useTso = true;
            tsoTotalLen = txd_op::getLen(desc);
            tsoLoadedHeader = false;
            tsoDescBytesUsed = 0;
            tsoUsedLen = 0;
            tsoPrevSeq = 0;
            tsoPktHasHeader = false;
            tsoPkts = 0;
            tsoCopyBytes = 0;
        }

        txd_op::setDd(desc);
        unusedCache.pop_front();
        usedCache.push_back(desc);
    }

    if (!unusedCache.size())
        return;

    desc = unusedCache.front();
    if (!useTso && txd_op::isType(desc, txd_op::TXD_ADVDATA) &&
        txd_op::tse(desc)) {
        DPRINTF(EthernetDesc, "TXD[%d] TCP offload(adv) enabled for packet "
                "hdrlen: %d mss: %d paylen %d\n", queueID,
                tsoHeaderLen, tsoMss, txd_op::getTsoLen(desc));
        useTso = true;
        tsoTotalLen = txd_op::getTsoLen(desc);
        tsoLoadedHeader = false;
        tsoDescBytesUsed = 0;
        tsoUsedLen = 0;
        tsoPrevSeq = 0;
        tsoPktHasHeader = false;
        tsoPkts = 0;
    }

    if (useTso && !tsoLoadedHeader) {
        // we need to fetch a header
        DPRINTF(EthernetDesc, "TXD[%d] Starting DMA of TSO header\n", queueID);
        assert(txd_op::isData(desc) && txd_op::getLen(desc) >= tsoHeaderLen);
        pktHdrWaiting = true;
        assert(tsoHeaderLen <= 256);
        igbe->dmaRead(pciToDma(txd_op::getBuf(desc)),
                      tsoHeaderLen, &headerEvent, tsoHeader, 0);
    }
}

void
IGbE::TxDescCacheGlobal::headerComplete()
{
    DPRINTF(EthernetDesc, "TXD[%d] TSO: Fetching TSO header complete\n", queueID);
    pktHdrWaiting = false;

    assert(unusedCache.size());
    TxDesc *desc = unusedCache.front();
    DPRINTF(EthernetDesc, "TXD[%d] TSO: len: %d tsoHeaderLen: %d\n", queueID,
            txd_op::getLen(desc), tsoHeaderLen);

    if (txd_op::getLen(desc) == tsoHeaderLen) {
        tsoDescBytesUsed = 0;
        tsoLoadedHeader = true;
        unusedCache.pop_front();
        usedCache.push_back(desc);
    } else {
        DPRINTF(EthernetDesc, "TXD[%d] TSO: header part of larger payload\n", queueID);
        tsoDescBytesUsed = tsoHeaderLen;
        tsoLoadedHeader = true;
    }
    enableSm();
    igbe->checkDrain();
}

unsigned
IGbE::TxDescCacheGlobal::getPacketSize()
{
    if (!unusedCache.size())
        return 0;

    DPRINTF(EthernetDesc, "TXD[%d] Starting processing of descriptor\n", queueID);
    DPRINTF(EthernetDpdk, "TXD[%d] Starting processing of descriptor\n", queueID);
    assert(!useTso || tsoLoadedHeader);
    TxDesc *desc = unusedCache.front();

    if (useTso) {
        DPRINTF(EthernetDesc, "TXD[%d] getPacket(): TxDescriptor data "
                "d1: %#llx d2: %#llx\n", queueID, desc->d1, desc->d2);
        DPRINTF(EthernetDesc, "TXD[%d] TSO: use: %d hdrlen: %d mss: %d total: %d "
                "used: %d loaded hdr: %d\n", queueID, useTso, tsoHeaderLen, tsoMss,
                tsoTotalLen, tsoUsedLen, tsoLoadedHeader);

        if (tsoPktHasHeader) {
            assert(tsoDMAEngineIdx != -1);
            EthPacketPtr p = processingPktArray[tsoDMAEngineIdx];
            assert(p != NULL);
            tsoCopyBytes =  std::min((tsoMss + tsoHeaderLen) - p->length,
                                     txd_op::getLen(desc) - tsoDescBytesUsed);
        }
        else {
            tsoCopyBytes =  std::min(tsoMss,
                                     txd_op::getLen(desc) - tsoDescBytesUsed);
        }
        unsigned pkt_size =
            tsoCopyBytes + (tsoPktHasHeader ? 0 : tsoHeaderLen);

        DPRINTF(EthernetDesc, "TXD[%d] TSO: descBytesUsed: %d copyBytes: %d "
                "this descLen: %d\n", queueID,
                tsoDescBytesUsed, tsoCopyBytes, txd_op::getLen(desc));
        DPRINTF(EthernetDesc, "TXD[%d] TSO: pktHasHeader: %d\n", queueID, tsoPktHasHeader);
        DPRINTF(EthernetDesc, "TXD[%d] TSO: Next packet is %d bytes\n", queueID, pkt_size);
        return pkt_size;
    }

    DPRINTF(EthernetDesc, "TXD[%d] Next TX packet is %d bytes\n", queueID,
            txd_op::getLen(unusedCache.front()));
    return txd_op::getLen(desc);
}

void
IGbE::TxDescCacheGlobal::getPacketDataGlobal()
{   
    assert(unusedCache.size());

    TxDesc *desc;
    desc = unusedCache.front();

    DPRINTF(EthernetDesc, "TXD[%d] getPacketData(): TxDescriptor data "
            "d1: %#llx d2: %#llx\n", queueID, desc->d1, desc->d2);
    assert((txd_op::isLegacy(desc) || txd_op::isData(desc)) &&
           txd_op::getLen(desc));

    if (!useTso) {
        EthPacketPtr p = std::make_shared<EthPacketData>(16384);
        assert(hasFreeDMAEngine());
        int idx = getFreeDMAEngine();
        assert(idx != -1);
        assert(processingPktArray[idx] == NULL);
        assert(processingCache[idx].desc == NULL);
        processingPktArray[idx] = p;
        processingCache[idx].desc = desc;
        processingCache[idx].seqIdx = nextSeqToProcess++;
        processingCache[idx].done = false;
        unusedCache.pop_front();

        // printf("[LOG], %lu, TXD[%d]_Use_DMAEngine[%d]_For_Packet_with_seqIdx_%d\n"
        //     , curTick(), queueID, idx, processingCache[idx].seqIdx);

        // Call dmaEngine's getPacketData
        dmaEngineArray[idx]->getPacketData(p, pciToDma(txd_op::getBuf(desc)),
                        txd_op::getLen(desc));
    } else {
        // useTso
        if (tsoDMAEngineIdx == -1) {
            // First descriptor of TSO packet
            EthPacketPtr p = std::make_shared<EthPacketData>(16384);
            assert(hasFreeDMAEngine());
            tsoDMAEngineIdx = getFreeDMAEngine();
            assert(tsoDMAEngineIdx != -1);
            assert(processingPktArray[tsoDMAEngineIdx] == NULL);
            processingPktArray[tsoDMAEngineIdx] = p;
        }
        assert(processingPktArray[tsoDMAEngineIdx] != NULL);
        assert(tsoLoadedHeader);
        if (!tsoPktHasHeader) {
            DPRINTF(EthernetDesc,
                    "TXD[%d] Loading TSO header (%d bytes) into start of packet\n", queueID,
                    tsoHeaderLen);
            memcpy(processingPktArray[tsoDMAEngineIdx]->data, &tsoHeader,
                   tsoHeaderLen);
            processingPktArray[tsoDMAEngineIdx]->length += tsoHeaderLen;
            tsoPktHasHeader = true;
        }
        assert(processingCache[tsoDMAEngineIdx].desc == NULL);
        processingCache[tsoDMAEngineIdx].desc = desc;
        processingCache[tsoDMAEngineIdx].seqIdx = nextSeqToProcess++;
        processingCache[tsoDMAEngineIdx].done = false;
        unusedCache.pop_front();

        // Call dmaEngine's getPacketData
        assert(tsoCopyBytes > 0);
        dmaEngineArray[tsoDMAEngineIdx]->getPacketData(processingPktArray[tsoDMAEngineIdx], 
                        pciToDma(txd_op::getBuf(desc)) + tsoDescBytesUsed,
                        tsoCopyBytes);
        tsoDescBytesUsed += tsoCopyBytes;
        assert(tsoDescBytesUsed <= txd_op::getLen(desc));
    }
}

void
IGbE::TxDescCacheGlobal::onDMAComplete(int engineIdx)
{
    assert(engineIdx < numDMAEngines);
    TxDescCache *dmaEngine = dmaEngineArray[engineIdx];
    assert(dmaEngine != NULL);

    // Get the info from the DMA engine
    EthPacketPtr pktPtr = processingPktArray[engineIdx];
    assert(pktPtr != NULL);
    TxDesc *desc = processingCache[engineIdx].desc;
    assert(desc != NULL);

    assert((txd_op::isLegacy(desc) || txd_op::isData(desc)) &&
           txd_op::getLen(desc));
    
    DPRINTF(EthernetDesc, "TXD[%d] TxDescriptor data d1: %#llx d2: %#llx\n", queueID,
            desc->d1, desc->d2);

    // Set the length of the data in the EtherPacket
    if (useTso) {
        DPRINTF(EthernetDesc, "TXD[%d] TSO: use: %d hdrlen: %d mss: %d total: %d "
            "used: %d loaded hdr: %d\n", queueID, useTso, tsoHeaderLen, tsoMss,
            tsoTotalLen, tsoUsedLen, tsoLoadedHeader);
        pktPtr->simLength += tsoCopyBytes;
        pktPtr->length += tsoCopyBytes;
        tsoUsedLen += tsoCopyBytes;
        DPRINTF(EthernetDesc, "TXD[%d] TSO: descBytesUsed: %d copyBytes: %d\n", queueID,
            tsoDescBytesUsed, tsoCopyBytes);
    } else {
        pktPtr->simLength += txd_op::getLen(desc);
        pktPtr->length += txd_op::getLen(desc);
    }

    if ((!txd_op::eop(desc) && !useTso) ||
        (pktPtr->length < ( tsoMss + tsoHeaderLen) &&
         tsoTotalLen != tsoUsedLen && useTso)) {
        assert(!useTso || (tsoDescBytesUsed == txd_op::getLen(desc)));

        processingCache[engineIdx].done = true;

        // Call to the manage the used cache and the processingCache to meet the sequence
        manageUsedCache();

        tsoDescBytesUsed = 0;
        dmaEngine->setPktDone(true);
        dmaEngine->setPktWaiting(false);

        pktMultiDesc = true;

        DPRINTF(EthernetDesc, "TXD[%d] Partial Packet Descriptor of %d bytes Done\n", queueID,
                pktPtr->length);

        enableSm();
        igbe->checkDrain();
        return;
    }

    pktMultiDesc = false;
    // no support for vlans
    assert(!txd_op::vle(desc));

    // we only support single packet descriptors at this point
    if (!useTso)
        assert(txd_op::eop(desc));

    // set that this packet is done
    if (txd_op::rs(desc))
        txd_op::setDd(desc);

    DPRINTF(EthernetDesc, "TXD[%d] TxDescriptor data d1: %#llx d2: %#llx\n", queueID,
            desc->d1, desc->d2);
    
    if (useTso) {
        IpPtr ip(pktPtr);
        Ip6Ptr ip6(pktPtr);
        if (ip) {
            DPRINTF(EthernetDesc, "TXD[%d] TSO: Modifying IP header. Id + %d\n", queueID,
                    tsoPkts);
            ip->id(ip->id() + tsoPkts++);
            ip->len(pktPtr->length - EthPtr(pktPtr)->size());
        }
        if (ip6)
            ip6->plen(pktPtr->length - EthPtr(pktPtr)->size());
        TcpPtr tcp = ip ? TcpPtr(ip) : TcpPtr(ip6);
        if (tcp) {
            DPRINTF(EthernetDesc,
                    "TXD[%d] TSO: Modifying TCP header. old seq %d + %d\n", queueID,
                    tcp->seq(), tsoPrevSeq);
            tcp->seq(tcp->seq() + tsoPrevSeq);
            if (tsoUsedLen != tsoTotalLen)
                tcp->flags(tcp->flags() & ~9); // clear fin & psh
        }
        UdpPtr udp = ip ? UdpPtr(ip) : UdpPtr(ip6);
        if (udp) {
            DPRINTF(EthernetDesc, "TXD[%d] TSO: Modifying UDP header.\n", queueID);
            udp->len(pktPtr->length - EthPtr(pktPtr)->size());
        }
        tsoPrevSeq = tsoUsedLen;
    }

    if (debug::EthernetDesc) {
        IpPtr ip(pktPtr);
        if (ip)
            DPRINTF(EthernetDesc, "TXD[%d] Proccesing Ip packet with Id=%d\n", queueID,
                    ip->id());
        else
            DPRINTF(EthernetSM, "TXD[%d] Proccesing Non-Ip packet\n", queueID);
    }

    // Checksums are only ofloaded for new descriptor types
    if (txd_op::isData(desc) && (txd_op::ixsm(desc) || txd_op::txsm(desc))) {
        DPRINTF(EthernetDesc, "TXD[%d] Calculating checksums for packet\n", queueID);
        IpPtr ip(pktPtr);
        Ip6Ptr ip6(pktPtr);
        assert(ip || ip6);
        if (ip && txd_op::ixsm(desc)) {
            ip->sum(0);
            ip->sum(cksum(ip));
            igbe->etherDeviceStats.txIpChecksums++;
            DPRINTF(EthernetDesc, "TXD[%d] Calculated IP checksum\n", queueID);
        }
        if (txd_op::txsm(desc)) {
            TcpPtr tcp = ip ? TcpPtr(ip) : TcpPtr(ip6);
            UdpPtr udp = ip ? UdpPtr(ip) : UdpPtr(ip6);
            if (tcp) {
                tcp->sum(0);
                tcp->sum(cksum(tcp));
                igbe->etherDeviceStats.txTcpChecksums++;
                DPRINTF(EthernetDesc, "TXD[%d] Calculated TCP checksum\n", queueID);
            } else if (udp) {
                assert(udp);
                udp->sum(0);
                udp->sum(cksum(udp));
                igbe->etherDeviceStats.txUdpChecksums++;
                DPRINTF(EthernetDesc, "TXD[%d] Calculated UDP checksum\n", queueID);
            } else {
                panic("Told to checksum, but don't know how\n");
            }
        }
    }

    if (txd_op::ide(desc)) {
        // Deal with the rx timer interrupts
        DPRINTF(EthernetDesc, "TXD[%d] Descriptor had IDE set\n", queueID);
        if (igbe->regs.tidv.idv()) {
            Tick delay = igbe->regs.tidv.idv() * igbe->intClock();
            DPRINTF(EthernetDesc, "TXD[%d] setting tidv\n", queueID);
            // igbe->reschedule(igbe->tidvEvent, curTick() + delay, true);
            igbe->reschedule(_tidvEvent, curTick() + delay, true);
        }

        if (igbe->regs.tadv.idv() && igbe->regs.tidv.idv()) {
            Tick delay = igbe->regs.tadv.idv() * igbe->intClock();
            DPRINTF(EthernetDesc, "TXD[%d] setting tadv\n", queueID);
            // if (!igbe->tadvEvent.scheduled()) {
            //     igbe->schedule(igbe->tadvEvent, curTick() + delay);
            // }
            if (!_tadvEvent.scheduled()) {
                igbe->schedule(_tadvEvent, curTick() + delay);
            }
        }
    }

    if (!useTso ||  txd_op::getLen(desc) == tsoDescBytesUsed) {
        DPRINTF(EthernetDesc, "TXD[%d] Descriptor Done\n", queueID);
        processingCache[engineIdx].done = true;

        // printf("[LOG], %lu, TXD[%d]_DMAEngine[%d]_Done_with_seqIdx_%d\n"
        //     , curTick(), queueID, engineIdx, processingCache[engineIdx].seqIdx);
        
        // Call to the manage the used cache and the processingCache to meet the sequence
        manageUsedCache();

        tsoDescBytesUsed = 0;
    }

    if (useTso && tsoUsedLen == tsoTotalLen) {
        useTso = false;
        tsoDMAEngineIdx = -1;
    }

    DPRINTF(EthernetDesc,
            "TXD[%d] ------Packet of %d bytes ready for transmission-------\n", queueID,
            pktPtr->length);
    DPRINTF(EthernetDpdk,
            "TXD[%d] ------Packet of %d bytes ready for transmission-------\n", queueID,
            pktPtr->length);
    
    igbe->etherDeviceStats.txDMABytes += pktPtr->length;

    dmaEngine->setPktDone(true);
    dmaEngine->setPktWaiting(false);
    tsoPktHasHeader = false;

    if (igbe->regs.txdctl_array[queueID].wthresh() == 0) {
        DPRINTF(EthernetDesc, "TXD[%d] WTHRESH == 0, writing back descriptor\n", queueID);
        writebackGlobal(0);
    } else if (!igbe->regs.txdctl_array[queueID].gran() && igbe->regs.txdctl_array[queueID].wthresh() <=
               descInBlock(usedCache.size())) {
        DPRINTF(EthernetDesc, "TXD[%d] used > WTHRESH, writing back descriptor\n", queueID);
        writebackGlobal((igbe->cacheBlockSize()-1)>>4);
    } else if (igbe->regs.txdctl_array[queueID].wthresh() <= usedCache.size()) {
        DPRINTF(EthernetDesc, "TXD[%d] used > WTHRESH, writing back descriptor\n", queueID);
        writebackGlobal((igbe->cacheBlockSize()-1)>>4);
    }

    enableSm();
    igbe->checkDrain();
    return;
}

bool
IGbE::TxDescCacheGlobal::hasReadyEthPacket()
{
    // Check each DMA engine has ready packet
    // Ready packet: pktDone == true && pktWaiting == false && pktPtr != NULL && pktMultiDesc == false && pktPtr->length > 0
    for (int engine_idx = 0; engine_idx < numDMAEngines; engine_idx++) {
        if (!pktMultiDesc &&
            dmaEngineArray[engine_idx]->packetAvailable() && 
            processingPktArray[engine_idx] != NULL &&
            processingPktArray[engine_idx]->length > 0) {
            
            return true;
        }
    }
    return false;
}

EthPacketPtr 
IGbE::TxDescCacheGlobal::getReadyEthPacket()
{
    assert(hasReadyEthPacket());
    EthPacketPtr pktPtr = NULL;
    for (int i = 0; i < numDMAEngines; i++) {
        int engine_idx = (roundRobinIdx + i) % numDMAEngines;
        if (!pktMultiDesc &&
            dmaEngineArray[engine_idx]->packetAvailable() && 
            processingPktArray[engine_idx] != NULL &&
            processingPktArray[engine_idx]->length > 0) {
            
            pktPtr = processingPktArray[engine_idx];
            // Reset the DMA engine
            processingPktArray[engine_idx] = NULL;
            dmaEngineArray[engine_idx]->unsetPacketDone();
            assert(dmaEngineArray[engine_idx]->packetWaiting() == false);
            
            roundRobinIdx = (engine_idx + 1) % numDMAEngines;
            break;
        }   
    }
    return pktPtr;
}

void
IGbE::TxDescCacheGlobal::manageUsedCache()
{
    bool progress = true;
    while (progress) {
        progress = false;

        for (int engine_idx = 0; engine_idx < numDMAEngines; engine_idx++) {
            if (processingCache[engine_idx].done &&
                processingCache[engine_idx].seqIdx == nextSeqToWriteback) {

                TxDesc *desc = processingCache[engine_idx].desc;
                assert(desc != NULL);

                usedCache.push_back(desc);
                processingCache[engine_idx].desc = NULL;
                processingCache[engine_idx].seqIdx = 0;
                processingCache[engine_idx].done = false;

                // printf("[LOG], %lu, TXD[%d]_Push_ProcessingCache[%d]_To_UsedCache_with_seqIdx_%d\n"
                //     , curTick(), queueID, engine_idx, nextSeqToWriteback);
                nextSeqToWriteback++;
                progress = true; // made progress, need to scan again
                break; // restart loop to recheck all engines
            }
        }
    }
}

void
IGbE::TxDescCacheGlobal::actionAfterWb()
{
    DPRINTF(EthernetDesc, "TXD[%d] actionAfterWb() completionEnabled: %d\n", queueID,
            completionEnabled);
    igbe->postInterrupt(igbreg::IT_TXDW);
    if (completionEnabled) {
        descEnd = igbe->regs.tdh_array[queueID]();
        DPRINTF(EthernetDesc,
                "TXD[%d] Completion writing back value: %d to addr: %#x\n", queueID, descEnd,
                completionAddress);
        // SHIN.
        // igbe->dmaWrite(pciToDma(mbits(completionAddress, 63, 2)),
        //                sizeof(descEnd), &nullEvent, (uint8_t*)&descEnd, 0);
        igbe->IdioWrite(pciToDma(mbits(completionAddress, 63, 2)),
                       sizeof(descEnd), &nullEvent, (uint8_t*)&descEnd, 0, 0, igbe->adq);
    }
}

void
IGbE::TxDescCacheGlobal::serialize(CheckpointOut &cp) const
{   
    DescCacheGlobal<TxDesc>::serialize(cp);

    SERIALIZE_SCALAR(isTcp);
    SERIALIZE_SCALAR(pktMultiDesc);

    SERIALIZE_SCALAR(useTso);
    SERIALIZE_SCALAR(tsoHeaderLen);
    SERIALIZE_SCALAR(tsoMss);
    SERIALIZE_SCALAR(tsoTotalLen);
    SERIALIZE_SCALAR(tsoUsedLen);
    SERIALIZE_SCALAR(tsoPrevSeq);;
    SERIALIZE_SCALAR(tsoPktPayloadBytes);
    SERIALIZE_SCALAR(tsoLoadedHeader);
    SERIALIZE_SCALAR(tsoPktHasHeader);
    SERIALIZE_ARRAY(tsoHeader, 256);
    SERIALIZE_SCALAR(tsoDescBytesUsed);
    SERIALIZE_SCALAR(tsoCopyBytes);
    SERIALIZE_SCALAR(tsoPkts);

    SERIALIZE_SCALAR(completionAddress);
    SERIALIZE_SCALAR(completionEnabled);
    SERIALIZE_SCALAR(descEnd);

    Tick _tidv_time = 0, _tadv_time = 0;
    if (_tidvEvent.scheduled())
        _tidv_time = _tidvEvent.when();
    SERIALIZE_SCALAR(_tidv_time);
    if (_tadvEvent.scheduled())
        _tadv_time = _tadvEvent.when();
    SERIALIZE_SCALAR(_tadv_time);

}

void
IGbE::TxDescCacheGlobal::unserialize(CheckpointIn &cp)
{   
    DescCacheGlobal<TxDesc>::unserialize(cp);

    UNSERIALIZE_SCALAR(isTcp);
    UNSERIALIZE_SCALAR(pktMultiDesc);

    UNSERIALIZE_SCALAR(useTso);
    UNSERIALIZE_SCALAR(tsoHeaderLen);
    UNSERIALIZE_SCALAR(tsoMss);
    UNSERIALIZE_SCALAR(tsoTotalLen);
    UNSERIALIZE_SCALAR(tsoUsedLen);
    UNSERIALIZE_SCALAR(tsoPrevSeq);;
    UNSERIALIZE_SCALAR(tsoPktPayloadBytes);
    UNSERIALIZE_SCALAR(tsoLoadedHeader);
    UNSERIALIZE_SCALAR(tsoPktHasHeader);
    UNSERIALIZE_ARRAY(tsoHeader, 256);
    UNSERIALIZE_SCALAR(tsoDescBytesUsed);
    UNSERIALIZE_SCALAR(tsoCopyBytes);
    UNSERIALIZE_SCALAR(tsoPkts);

    UNSERIALIZE_SCALAR(completionAddress);
    UNSERIALIZE_SCALAR(completionEnabled);
    UNSERIALIZE_SCALAR(descEnd);

    Tick _tidv_time = 0, _tadv_time = 0;
    UNSERIALIZE_SCALAR(_tidv_time);
    UNSERIALIZE_SCALAR(_tadv_time);
    if (_tidv_time)
        igbe->schedule(_tidvEvent, _tidv_time);
    if (_tadv_time)
        igbe->schedule(_tadvEvent, _tadv_time);
}

void
IGbE::TxDescCacheGlobal::enableSm()
{
    if (igbe->drainState() != DrainState::Draining) {
        igbe->txTick = true;
        igbe->restartClock();
    }
}

bool
IGbE::TxDescCacheGlobal::hasOutstandingEvents()
{
    bool descCacheGlobalOutstanding = false;
    // Check all desc DMA engines
    for (int engine_idx = 0; engine_idx < numDescDMAEngines; engine_idx++) {
        descCacheGlobalOutstanding |= descFetchDMAEngines[engine_idx]->hasOutstandingEvents();
        descCacheGlobalOutstanding |= descWbDMAEngines[engine_idx]->hasOutstandingEvents();
    }
    // Have to check all DMA engines
    for (int engine_idx = 0; engine_idx < numDMAEngines; engine_idx++) {
        descCacheGlobalOutstanding |= dmaEngineArray[engine_idx]->hasOutstandingEvents();
    }
    return descCacheGlobalOutstanding;
    
}

std::string 
IGbE::TxDescCacheGlobal::wbBufToString(int idx) {
    igbreg::TxDesc *desc;
    desc = wbBuf + idx;
    uint8_t Dd = txd_op::getDd(desc);
    std::string descStr = "TxDescWb_DD: ";
    descStr += csprintf("%u", Dd);
    return descStr;
}
std::string
IGbE::TxDescCacheGlobal::wbBufArrayToString(int engineIdx, int idx) {
    igbreg::TxDesc *desc;
    desc = wbBufArray[engineIdx] + idx;
    uint8_t Dd = txd_op::getDd(desc);
    std::string descStr = "TxDescWb_DD: ";
    descStr += csprintf("%u", Dd);
    return descStr;
}
std::string 
IGbE::TxDescCacheGlobal::fetchBufToString(igbreg::TxDesc* desc) {
    std::string descStr = "TxDescFetch_addr: ";
    descStr += csprintf("%#x", txd_op::getBuf(desc));
    descStr += " TxDescFetch_len: ";
    descStr += csprintf("%u", txd_op::getLen(desc));
    return descStr;
}

///////////////////////////// IGbE::RxM2funcContext //////////////////////////////

IGbE::RxM2funcContext::RxM2funcContext(IGbE *i, std::string n, int _rxContextFifoSize, int qid, bool _enableDTA, int _cxlReqBufSize)
    : igbe(i), _name(n), rxContextFifoSize(_rxContextFifoSize), queueID(qid),
      remainPacket(false), splitCount(0), bytesSent(0), rxCount(0), enableDTA(_enableDTA), numFreeCXLReqMax(_cxlReqBufSize)
{
    m2funcRxFifo.clear();
    m2funcCXLReqBuf.clear();
    lastRxM2funcReadTick = 0;
    numCXLReq = 0;
    numFreeCXLReq = 0;

}

IGbE::RxM2funcContext::~RxM2funcContext()
{
    // Clear the m2funcRxFifo
    for (auto it = m2funcRxFifo.begin(); it != m2funcRxFifo.end(); it++) {
        delete[] it->packetData;
    }
    m2funcRxFifo.clear();
    
    m2funcCXLReqBuf.clear();
}

void
IGbE::RxM2funcContext::processRxPacket(EthPacketPtr packet)
{
    /*
     * This function processes the packet in the M2func context. If offload is needed,
     * it will be done here. Make a data packet from the EthPacket 
     */
    DPRINTF(EthernetDpdk, "RXM2func[%d]: Processing packet %p of length: %d\n", queueID, packet.get(), packet->length);

    unsigned buf_len, hdr_len;

    // do pktComplete() things - setting descriptor 
    uint16_t crcfixup = igbe->regs.rctl.secrc() ? 0 : 4 ;
    DPRINTF(EthernetDpdk, "RxM2func[%d] CRC offset: %d total packet length: %d %d\n", 
    queueID, crcfixup, htole((uint16_t)(packet->length + crcfixup)), (uint16_t)(packet->length + crcfixup));

    uint16_t status = RXDS_DD;
    uint8_t err = 0;
    uint16_t ext_err = 0;
    uint16_t csum = 0;
    uint16_t ptype = 0;
    uint16_t ip_id = 0;

    // Assume ethpacket is within MTU size. So always set EOP
    status |= RXDS_EOP;

    IpPtr ip(packet);
    Ip6Ptr ip6(packet);

    if (ip || ip6) {
        if (ip) {
            DPRINTF(EthernetDesc, "Proccesing Ip packet with Id=%d\n",
                    ip->id());
            ptype |= RXDP_IPV4;
            ip_id = ip->id();
        }
        if (ip6)
            ptype |= RXDP_IPV6;

        if (ip && igbe->regs.rxcsum.ipofld()) {
            DPRINTF(EthernetDesc, "Checking IP checksum\n");
            status |= RXDS_IPCS;
            csum = htole(cksum(ip));
            igbe->etherDeviceStats.rxIpChecksums++;
            if (cksum(ip) != 0) {
                err |= RXDE_IPE;
                ext_err |= RXDEE_IPE;
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
            }
        }
        TcpPtr tcp = ip ? TcpPtr(ip) : TcpPtr(ip6);
        if (tcp && igbe->regs.rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "Checking TCP checksum\n");
            status |= RXDS_TCPCS;
            ptype |= RXDP_TCP;
            csum = htole(cksum(tcp));
            igbe->etherDeviceStats.rxTcpChecksums++;
            if (cksum(tcp) != 0) {
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
                err |= RXDE_TCPE;
                ext_err |= RXDEE_TCPE;
            }
        }

        UdpPtr udp = ip ? UdpPtr(ip) : UdpPtr(ip6);
        if (udp && igbe->regs.rxcsum.tuofld()) {
            DPRINTF(EthernetDesc, "Checking UDP checksum\n");
            status |= RXDS_UDPCS;
            ptype |= RXDP_UDP;
            csum = htole(cksum(udp));
            igbe->etherDeviceStats.rxUdpChecksums++;
            if (cksum(udp) != 0) {
                DPRINTF(EthernetDesc, "Checksum is bad!!\n");
                ext_err |= RXDEE_TCPE;
                err |= RXDE_TCPE;
            }
        }
    } else {
        DPRINTF(EthernetDpdk, "RxM2func[%d]: Non-IP packet\n", queueID);
    }

    RxM2funcDesc rxDesc;
    unsigned desc_len = 8;
    switch (igbe->regs.srrctl_array[queueID].desctype()) {
      case RXDT_LEGACY:
      {
        DPRINTF(EthernetDpdk, "RxM2func[%d]: processPacket with LEGACY\n", queueID);
        
        desc_len = 8;

        rxDesc.legacy.len = htole((uint16_t)(packet->length + crcfixup));
        rxDesc.legacy.csum = htole(0);
        rxDesc.legacy.status = htole(status);
        rxDesc.legacy.errors = htole(err);
        // No vlan support at this point... just set it to 0
        rxDesc.legacy.vlan = 0;
        break;
      }
      case RXDT_ADV_ONEBUF:
      { 
        DPRINTF(EthernetDpdk, "RxM2func[%d]: processPacket with ADV_ONEBUF\n", queueID);

        desc_len = 16;

        buf_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl_array[queueID].bufLen() :
            igbe->regs.rctl.descSize();
        DPRINTF(EthernetDpdk, "RxM2func[%d] ADV_ONEBUF LPE: %d Packet Length: %d srrctl: %#x Desc Size: %d\n",
                queueID, igbe->regs.rctl.lpe(), packet->length, igbe->regs.srrctl_array[queueID](), buf_len);
        assert(packet->length < buf_len);

        rxDesc.adv_wb.rss_type = htole(0);
        rxDesc.adv_wb.pkt_type = htole(ptype);
        rxDesc.adv_wb.__reserved1 = htole(0);
        rxDesc.adv_wb.header_len = htole(0);
        rxDesc.adv_wb.sph = htole(0);
        rxDesc.adv_wb.pkt_len = htole((uint16_t)(packet->length));
        if (igbe->regs.rxcsum.pcsd()) {
            // no rss support right now
            rxDesc.adv_wb.rss_hash = htole(0);
        } else {
            rxDesc.adv_wb.id = htole(ip_id);
            rxDesc.adv_wb.csum = htole(csum);
        }
        rxDesc.adv_wb.status = htole(status);
        rxDesc.adv_wb.errors = htole(ext_err);
        // no vlan support
        rxDesc.adv_wb.vlan_tag = htole(0);
        break;
      }
      case RXDT_ADV_SPLIT_A:
      {
        DPRINTF(EthernetDpdk, "RxM2func[%d]: processPacket with ADV_SPLIT_A\n", queueID);
        
        desc_len = 16;

        buf_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl_array[queueID].bufLen() :
            igbe->regs.rctl.descSize();
        hdr_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl_array[queueID].hdrLen() : 0;

        rxDesc.adv_wb.rss_type = htole(0);
        rxDesc.adv_wb.pkt_type = htole(0);
        rxDesc.adv_wb.__reserved1 = htole(0);
        rxDesc.adv_wb.header_len = htole(0);
        rxDesc.adv_wb.sph = htole(0);
        rxDesc.adv_wb.rss_hash = htole(0);
        rxDesc.adv_wb.status = htole(0);
        rxDesc.adv_wb.errors = htole(0);
        rxDesc.adv_wb.pkt_len = htole(0);
        rxDesc.adv_wb.vlan_tag = htole(0); 
        panic("Not implemented yet\n");
        break;
      }
      default:
        panic("Unimplemnted RX receive buffer type %d\n",
              igbe->regs.srrctl_array[queueID].desctype());
    }

    // Pack descriptor & EthPacket into a single packet
    uint8_t *src = packet->data;
    int ethPkt_size = packet->length;
    int cxlPkt_size = desc_len + ethPkt_size;
    // Make cxlPkt_size multiple of flitSize
    cxlPkt_size = ((cxlPkt_size + igbe->flitSize - 1) / igbe->flitSize) * igbe->flitSize;
    uint8_t * cxl_packet = new uint8_t[cxlPkt_size];
    memset(cxl_packet, 0, cxlPkt_size);
    DPRINTF(EthernetDpdk, "RxM2func[%d]: ethPkt_size: %d desc_len: %d cxlPkt_size: %d\n", queueID, ethPkt_size, desc_len, cxlPkt_size);
    
    // Put rxDesc to the beginning of the packet
    if (igbe->regs.srrctl_array[queueID].desctype() == RXDT_LEGACY) {
        DPRINTF(EthernetDpdk, "RxM2func[%d]: Copying legacy descriptor\n", queueID);
        memcpy(cxl_packet, &rxDesc.legacy, (size_t) desc_len);
    } else {
        DPRINTF(EthernetDpdk, "RxM2func[%d]: Copying advanced descriptor\n", queueID);
        memcpy(cxl_packet, &rxDesc.adv_wb, (size_t) desc_len);
    }

    // Put the EthPacket to the end of the packet
    memcpy(cxl_packet + desc_len, src, (size_t) ethPkt_size);

    // Push to rxM2funcFifo
    // Make m2funcRXEntry
    m2funcRxFifoEntry entry;
    entry.packetData = cxl_packet;
    entry.packetLength = cxlPkt_size;
    entry.descLength = desc_len;
    entry.dataLength = ethPkt_size;
    entry.rxCount = rxCount;
    m2funcRxFifo.push_back(entry);
    DPRINTF(EthernetDpdk, "RxM2func[%d]: Packet pushed to m2funcRxFifo, with rxCount: %ld\n", queueID, rxCount);
    
    rxCount++;
    
    igbe->checkDrain();

}

bool
IGbE::RxM2funcContext::rxM2funcStateMachine()
{
    /*
     * This function is the main state machine for the M2func context. It
     * processes the packets in the M2func context and moves them to the
     * m2funcRxFifo.
     * Get the EthPacket from rxPacketArray[queueID] and process it.
     * 
     */
    if (isDTAEnabled()) {
        // If use DTA, we have to make CXL response packet corresponding to the request in m2funcCXLReqBuf
        // Call readM2funcPacket() here to make CXL response packet
        if (m2funcCXLReqBuf.empty()) {
            DPRINTF(EthernetDpdk, "RXM2func[%d]: No CXL request to process\n", queueID);
        } else {
            // If m2funcCXLReqBuf is not empty, we have to check dtaRXJobSuccess map to see if the job is already sent to DTA with zero packet or not
            // If it is already sent as zeroed packet (map is false), we have to skip the process and make a response with zero packet
            // If it is already sent as non-zeroed packet (map is true), we have to process the packet and make a response with the data in the m2funcRxFifo. 
              // So, in this case, if m2funcRxFifo is empty, we have to wait until the next RX packet comes from ethernet.
            // If dtaRXJobSuccess is not found in the map, we have to check m2funcRxFifo. If it is empty, set the map with false and make a response with zero packet
            // If m2funcRxFifo is not empty, set the map with true and process the packet and make a response with the data in the m2funcRxFifo
            
            PacketPtr cxlReq = m2funcCXLReqBuf.front();
            if (dtaRXJobSuccess.find(cxlReq->getJobID()) == dtaRXJobSuccess.end()) {
                if (m2funcRxFifo.empty()) {
                    if (!isLoadGenStarted) {
                        // When loadgen is not started, just return zeroed packet.
                        DPRINTF(EthernetDpdk, "RXM2func[%d]: No packet to process. Set dtaRXJobSuccess with false & Make zeroed packet\n", queueID);
                        // printf("RXM2func[%d]: No packet to process. Set dtaRXJobSuccess with false & Make zeroed packet for job id: %llu\n", queueID, cxlReq->getJobID());
                        dtaRXJobSuccess[cxlReq->getJobID()] = false;
                        popCXLReqBuf();
                        makeZeroedResponse(cxlReq);
                        sendCXLResp(cxlReq);
                    } else {
                        // When loadgen is started, just wait until the next RX packet comes from ethernet.
                        // printf("RXM2func[%d]: No packet to process. But, wait until the next RX packet comes from ethernet\n", queueID);
                    }
                } else {
                    if (!isLoadGenStarted) {
                        isLoadGenStarted = true;
                        printf("RXM2func[%d]: LoadGen started at %ld\n", queueID, curTick());
                    }
                    DPRINTF(EthernetDpdk, "RXM2func[%d]: Received RX eth. packet. Set dtaRXJobSuccess with true & Make a response for job id: %llu\n", queueID, cxlReq->getJobID());
                    // printf("RXM2func[%d]: Received RX eth. packet. Set dtaRXJobSuccess with true & Make a response for job id: %llu\n", queueID, cxlReq->getJobID());
                    dtaRXJobSuccess[cxlReq->getJobID()] = true;
                    popCXLReqBuf();
                    readM2funcPacket(cxlReq);
                    DPRINTF(EthernetDpdk, "RXM2func[%d]: CXL request processed & Make a response using data in the m2funcRxFifo. Pop out from m2funcCXLReqBuf and return to DTA\n", queueID);
                    DPRINTF(EthernetDpdk, "Current free CXLReqBuf: %ld, numCXLReq: %ld\n", numFreeCXLReq, numCXLReq);
                    sendCXLResp(cxlReq);
                }
            } else {
                if (dtaRXJobSuccess[cxlReq->getJobID()]) {
                    if (!m2funcRxFifo.empty()) {
                        DPRINTF(EthernetDpdk, "RXM2func[%d]: Processing CXL request with m2funcRxFifo. Because dtaRXJobSuccess[%llu] is true\n", queueID, cxlReq->getJobID());
                        // printf("RXM2func[%d]: Processing CXL request with m2funcRxFifo. Because dtaRXJobSuccess[%llu] is true\n", queueID, cxlReq->getJobID());
                        popCXLReqBuf();
                        readM2funcPacket(cxlReq);
                        DPRINTF(EthernetDpdk, "RXM2func[%d]: CXL request processed & Make a response using data in the m2funcRxFifo. Pop out from m2funcCXLReqBuf and return to DTA\n", queueID);
                        DPRINTF(EthernetDpdk, "Current free CXLReqBuf: %ld, numCXLReq: %ld\n", numFreeCXLReq, numCXLReq);
                        sendCXLResp(cxlReq);
                    } else {
                        // printf("RXM2func[%d]: No packet to process. But, dtaRXJobSuccess[%llu] is true. So wait until the next RX packet comes from ethernet\n", queueID, cxlReq->getJobID());
                    }
                } else {
                    DPRINTF(EthernetDpdk, "RXM2func[%d]: No packet to process. Because dtaRXJobSuccess[%llu] is false. So Make Zeroed packet\n", queueID, cxlReq->getJobID());
                    // printf("RXM2func[%d]: No packet to process. Because dtaRXJobSuccess[%llu] is false. So Make Zeroed packet\n", queueID, cxlReq->getJobID());
                    popCXLReqBuf();
                    makeZeroedResponse(cxlReq);
                    sendCXLResp(cxlReq);
                }

            }
        }
    }

    if (igbe->rxPacketArray[queueID] == nullptr) {
        DPRINTF(EthernetDpdk, "RXM2func[%d]: No packet to process\n", queueID);

        return true; // It means this context may need to keep ticking
    }
    // Check if m2funcRxFifo is full or not
    if (m2funcRxFifoFull()) {
        DPRINTF(EthernetDpdk, "RXM2func[%d]: m2funcRxFifo is full\n", queueID);

        return true; // It means this context may need to keep ticking
    } else {
        EthPacketPtr pkt = igbe->rxPacketArray[queueID];
        processRxPacket(pkt);
        DPRINTF(EthernetDpdk, "RXM2func[%d]: Processing packet of length: %d\n", queueID, pkt->length);
        DPRINTF(EthernetDpdk, "RXM2func[%d]: Packet processed & moved to m2funcRxFifo. pop out from rxPacketArray\n", queueID);
        igbe->rxPacketArray[queueID] = nullptr;

        return true; // It means this context may need to keep ticking
    }

    return true;
}

void
IGbE::RxM2funcContext::readM2funcPacket(PacketPtr pkt)
{   
    assert(pkt->getSize() == igbe->flitSize);
    updateRxM2funcReadStat(curTick());
    if (m2funcRxFifo.empty()) {
        DPRINTF(EthernetDpdk, "RXM2func[%d]: No packet to read! set packet with size: %d\n", queueID, igbe->flitSize);
        // Make DD bit 0 in the descriptor and set the packet's data to 0
        uint8_t *cxl_packet = new uint8_t[igbe->flitSize];
        memset(cxl_packet, 0, igbe->flitSize);
        RxM2funcDesc rxDesc;
        unsigned desc_len = 8;
        switch (igbe->regs.srrctl_array[queueID].desctype()) {
            case RXDT_LEGACY:
            {
                desc_len = 8;
                rxDesc.legacy.len = htole(0);
                rxDesc.legacy.csum = htole(0);
                rxDesc.legacy.status = htole(0);
                rxDesc.legacy.errors = htole(0);
                rxDesc.legacy.vlan = htole(0);

                memcpy(cxl_packet, &rxDesc.legacy, (size_t) desc_len);
                break;
            }
            case RXDT_ADV_ONEBUF:
            case RXDT_ADV_SPLIT_A:
            { 
                desc_len = 16;
                rxDesc.adv_wb.status = htole(0); // Set DD bit to 0
                rxDesc.adv_wb.rss_type = htole(0);
                rxDesc.adv_wb.pkt_type = htole(0);
                rxDesc.adv_wb.__reserved1 = htole(0);
                rxDesc.adv_wb.header_len = htole(0);
                rxDesc.adv_wb.sph = htole(0);
                rxDesc.adv_wb.rss_hash = htole(0);
                rxDesc.adv_wb.errors = htole(0);
                rxDesc.adv_wb.pkt_len = htole(0);
                rxDesc.adv_wb.vlan_tag = htole(0);

                memcpy(cxl_packet, &rxDesc.adv_wb, (size_t) desc_len);
                break;
            }
            default:
                panic("Unimplemnted RX receive buffer type %d\n",
                    igbe->regs.srrctl_array[queueID].desctype());
        }
        pkt->setData(cxl_packet);

        // Update stat
        igbe->etherDeviceStats.rxBytesM2func += igbe->flitSize;
        igbe->etherDeviceStats.rxBytesM2funcDesc += desc_len;


        // Free the memory
        delete[] cxl_packet;
    } else {
        // Pop the packet from m2funcRxFifo
        m2funcRxFifoEntry entry = m2funcRxFifo.front();
        uint64_t packetLength = entry.packetLength;
        uint64_t descLength = entry.descLength; // For stat
        uint64_t dataLength = entry.dataLength; // For stat
        uint8_t *packetData = entry.packetData;

        // Check remainPacket to check if the packet is remained
        if (remainPacket) {
            DPRINTF(EthernetDpdk, "RXM2func[%d]: Remain packet is true\n", queueID);
            // Copy the remaining packet
            if (packetLength > bytesSent) {
                DPRINTF(EthernetDpdk, "RXM2func[%d]: Ethernet Packet length: %ld bytesSent: %ld\n", queueID, packetLength, bytesSent);
                uint64_t remainLength = packetLength - bytesSent;
                uint64_t copyLength = remainLength > igbe->flitSize ? igbe->flitSize : remainLength;
                DPRINTF(EthernetDpdk, "RXM2func[%d]: Set Packet response with remaining packet with length: %ld\n", queueID, copyLength);
                pkt->setData(packetData + bytesSent, copyLength);
                bytesSent += copyLength;

                // Update stat
                igbe->etherDeviceStats.rxBytesM2func += copyLength;

                if (bytesSent == packetLength) {
                    remainPacket = false;
                    bytesSent = 0;
                    delete[] packetData;
                    m2funcRxFifo.pop_front();

                    // Update stat
                    igbe->etherDeviceStats.rxBytesM2funcData += dataLength;
                }
            } else {
                panic("RXM2func[%d]: This case should not happen. if packetLength <= bytesSent, remainPacket should be false\n", queueID);
            }
        } else {
            DPRINTF(EthernetDpdk, "RXM2func[%d]: Remain packet is false. Send new ethernet packet. Set packet as DDIO header\n", queueID);
            // As this is the first time to send the packet, we have to mark the packet as header
            pkt->setDdioHeader();
            // Copy the packet
            if (packetLength > igbe->flitSize) {
                DPRINTF(EthernetDpdk, "RXM2func[%d]: Ethernet Packet length: %ld\n", queueID, packetLength);
                pkt->setData(packetData, igbe->flitSize);
                remainPacket = true;
                bytesSent = igbe->flitSize;

                // Update stat
                igbe->etherDeviceStats.rxBytesM2func += igbe->flitSize;
                igbe->etherDeviceStats.rxBytesM2funcDesc += descLength;
            } else {
                DPRINTF(EthernetDpdk, "RXM2func[%d]: Set Packet response with packet with length: %ld\n", queueID, packetLength);
                pkt->setData(packetData, packetLength);
                delete[] packetData;
                m2funcRxFifo.pop_front();

                // Update stat
                igbe->etherDeviceStats.rxBytesM2func += packetLength;
                igbe->etherDeviceStats.rxBytesM2funcDesc += descLength;
                igbe->etherDeviceStats.rxBytesM2funcData += dataLength;
            }
        }

    }
}

void 
IGbE::RxM2funcContext::makeZeroedResponse(PacketPtr pkt)
{
    assert(pkt->getSize() == igbe->flitSize);
    DPRINTF(EthernetDpdk, "RXM2func[%d]: No packet to read! set packet with size: %d\n", queueID, igbe->flitSize);
    // printf("RXM2func[%d]: No packet to read! set packet with size: %d\n", queueID, igbe->flitSize);
    // Make DD bit 0 in the descriptor and set the packet's data to 0
    uint8_t *cxl_packet = new uint8_t[igbe->flitSize];
    memset(cxl_packet, 0, igbe->flitSize);
    RxM2funcDesc rxDesc;
    unsigned desc_len = 8;
    switch (igbe->regs.srrctl_array[queueID].desctype()) {
        case RXDT_LEGACY:
        {
            desc_len = 8;
            rxDesc.legacy.len = htole(0);
            rxDesc.legacy.csum = htole(0);
            rxDesc.legacy.status = htole(0);
            rxDesc.legacy.errors = htole(0);
            rxDesc.legacy.vlan = htole(0);

            memcpy(cxl_packet, &rxDesc.legacy, (size_t) desc_len);
            break;
        }
        case RXDT_ADV_ONEBUF:
        case RXDT_ADV_SPLIT_A:
        { 
            desc_len = 16;
            rxDesc.adv_wb.status = htole(0); // Set DD bit to 0
            rxDesc.adv_wb.rss_type = htole(0);
            rxDesc.adv_wb.pkt_type = htole(0);
            rxDesc.adv_wb.__reserved1 = htole(0);
            rxDesc.adv_wb.header_len = htole(0);
            rxDesc.adv_wb.sph = htole(0);
            rxDesc.adv_wb.rss_hash = htole(0);
            rxDesc.adv_wb.errors = htole(0);
            rxDesc.adv_wb.pkt_len = htole(0);
            rxDesc.adv_wb.vlan_tag = htole(0);

            memcpy(cxl_packet, &rxDesc.adv_wb, (size_t) desc_len);
            break;
        }
        default:
            panic("Unimplemnted RX receive buffer type %d\n",
                igbe->regs.srrctl_array[queueID].desctype());
    }
    pkt->setData(cxl_packet);

    // Update stat
    igbe->etherDeviceStats.rxBytesM2func += igbe->flitSize;
    igbe->etherDeviceStats.rxBytesM2funcDesc += desc_len;


    // Free the memory
    delete[] cxl_packet;
}

bool 
IGbE::RxM2funcContext::pushCXLReqBuf(PacketPtr pkt) {
    /**
     * Receive a packet from DTA and push it to the CXLReqBuf
     * Check the size of the CXLReqBuf and push the packet to the CXLReqBuf
     * Return true if the packet is pushed successfully, otherwise return false
     */
    if (m2funcCXLReqBufFull()) {
        DPRINTF(EthernetDpdk, "RXM2func[%d]: CXLReqBuf is full\n", queueID);
        return false;
    } else {
        // Push the packet to the CXLReqBuf
        m2funcCXLReqBuf.push_back(pkt);
        numCXLReq++;
        numFreeCXLReq++;
        DPRINTF(EthernetDpdk, "RXM2func[%d]: Packet pushed to CXLReqBuf with numCXLReq: %ld, numFreeCXLReq: %ld\n", queueID, numCXLReq, numFreeCXLReq);
        return true;
    }
}

PacketPtr 
IGbE::RxM2funcContext::popCXLReqBuf() {
    /**
     * Pop a packet from the CXLReqBuf and return it
     */
    assert(!m2funcCXLReqBuf.empty());
    
    PacketPtr pkt = m2funcCXLReqBuf.front();
    m2funcCXLReqBuf.pop_front();
    numFreeCXLReq--;
    DPRINTF(EthernetDpdk, "RXM2func[%d]: Packet popped from CXLReqBuf with numFreeCXLReq: %ld\n", queueID, numFreeCXLReq);
    return pkt;
    
}

void 
IGbE::RxM2funcContext::sendCXLResp(PacketPtr pkt) {
    /**
     * Send a response packet to DTA using port
     */
    DPRINTF(EthernetDpdk, "RXM2func[%d]: Send a response packet to DTA\n", queueID);
    
    pkt->makeResponse();

    Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;
    const Tick delay = receive_delay + igbe->cxlMemDelay; 
    
    assert(pkt->isResponse());
    igbe->m2funcPort->schedTimingResp(pkt, curTick() + delay);
}



void
IGbE::RxM2funcContext::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(queueID);
    SERIALIZE_SCALAR(rxContextFifoSize);
    SERIALIZE_SCALAR(remainPacket);
    SERIALIZE_SCALAR(bytesSent);
    SERIALIZE_SCALAR(splitCount);
    SERIALIZE_SCALAR(rxCount);

    // Serialize m2funcRxFifo
    uint64_t m2funcRxFifoSize = m2funcRxFifo.size();
    SERIALIZE_SCALAR(m2funcRxFifoSize);
    for (uint64_t i = 0; i < m2funcRxFifoSize; i++) {
        const m2funcRxFifoEntry &entry = m2funcRxFifo[i];
        paramOut(cp, csprintf("m2funcRxFifo[%d].rxCount", i), entry.rxCount);
        paramOut(cp, csprintf("m2funcRxFifo[%d].packetLength", i), entry.packetLength);
        paramOut(cp, csprintf("m2funcRxFifo[%d].descLength", i), entry.descLength);
        paramOut(cp, csprintf("m2funcRxFifo[%d].dataLength", i), entry.dataLength);
        arrayParamOut(cp, csprintf("m2funcRxFifo[%d].packetData", i), entry.packetData, entry.packetLength);
    }
}

void
IGbE::RxM2funcContext::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(queueID);
    UNSERIALIZE_SCALAR(rxContextFifoSize);
    UNSERIALIZE_SCALAR(remainPacket);
    UNSERIALIZE_SCALAR(bytesSent);
    UNSERIALIZE_SCALAR(splitCount);
    UNSERIALIZE_SCALAR(rxCount);

    // Unserialize m2funcRxFifo
    uint64_t m2funcRxFifoSize = 0;
    UNSERIALIZE_SCALAR(m2funcRxFifoSize);
    m2funcRxFifo.resize(m2funcRxFifoSize);
    for (uint64_t i = 0; i < m2funcRxFifoSize; i++) {
        m2funcRxFifoEntry &entry = m2funcRxFifo[i];
        paramIn(cp, csprintf("m2funcRxFifo[%d].rxCount", i), entry.rxCount);
        paramIn(cp, csprintf("m2funcRxFifo[%d].packetLength", i), entry.packetLength);
        paramIn(cp, csprintf("m2funcRxFifo[%d].descLength", i), entry.descLength);
        paramIn(cp, csprintf("m2funcRxFifo[%d].dataLength", i), entry.dataLength);
        arrayParamIn(cp, csprintf("m2funcRxFifo[%d].packetData", i), entry.packetData, entry.packetLength);
    }
}
///////////////////////////// IGbE::TxM2funcContext //////////////////////////////

IGbE::TxM2funcContext::TxM2funcContext(IGbE *i, std::string n, int _txContextFifoSize, int qid, bool _enableDTA)
    : igbe(i), _name(n), txContextFifoSize(_txContextFifoSize), queueID(qid),
      pktDone(false), ethPktSize(0), txDesc(0), receivedPktSize(0), descSize(8), isTcp(false), pktWaiting(false), pktMultiDesc(false), txCount(0), enableDTA(_enableDTA)
{    
    tsoEntry.tsoEnabled = false;
    tsoEntry.txPktExists = false;
    tsoEntry.headerLen = 0;
    tsoEntry.mss = 0;
    tsoEntry.totalLen = 0;
    tsoEntry.usedLen = 0;
    tsoEntry.prevSeq = 0;
    tsoEntry.loadedHeader = false;
    tsoEntry.ethPacketHasHeader = false;
    tsoEntry.tsoPkts = 0;

    // Initialize bitmask
    bitmaskWrap.bitmaskSize = (initial_bitmask_size_bytes * 8);
    bitmaskWrap.bitmask = (uint8_t*)malloc(initial_bitmask_size_bytes);
    if (bitmaskWrap.bitmask == NULL) {
        panic("Failed to allocate bitmask\n");
    }
    memset(bitmaskWrap.bitmask, 0, initial_bitmask_size_bytes);
    bitmaskWrap.readCursor = 0;
    bitmaskWrap.updateCursor = 0;

    m2funcTxFifo.clear();
}

IGbE::TxM2funcContext::~TxM2funcContext()
{
    // Free bitmask
    free(bitmaskWrap.bitmask);

    // Free m2funcTxFifo's packetData
    for (auto it = m2funcTxFifo.begin(); it != m2funcTxFifo.end(); it++) {
        delete[] it->packetData;
    }
    m2funcTxFifo.clear();
}

void 
IGbE::TxM2funcContext::processTxPacket(EthPacketPtr ethpkt, int dataSize, uint8_t* data, bool isHeader, bool ixsm, bool txsm)
{
    /**
     * This function processes the packet in the M2func context. If offload is needed,
     * it will be done here. Make a EthPacket from the cxl_packet
     */
    DPRINTF(EthernetDpdk, "TXM2func[%d]: Processing Ethpacket %p of length: %d\n", queueID, ethpkt.get(), dataSize);
    if (isHeader) {
        panic("TSO header processing is not implemented yet\n");
    } else {
        uint8_t *dest = ethpkt->data + ethpkt->length;
        memcpy(dest, data, (size_t) dataSize);

        // Update the ethpkt length
        ethpkt->length += dataSize;
        ethpkt->simLength += dataSize;

        DPRINTF(EthernetDpdk, "TXM2func[%d]: Ethpacket length updated to: %d\n", queueID, ethpkt->length);
        if (tsoEntry.tsoEnabled) {
            panic("TSO processing is not implemented yet\n");
        }
        
        // Offload checksums
        if (ixsm || txsm) {
            DPRINTF(EthernetDpdk, "TXM2func[%d]: Offloading checksums\n", queueID);
            IpPtr ip(ethpkt);
            Ip6Ptr ip6(ethpkt);
            assert(ip || ip6);      
            if (ip && ixsm) {
                ip->sum(0);
                ip->sum(cksum(ip));
                igbe->etherDeviceStats.txIpChecksums++;
                DPRINTF(EthernetDpdk, "TXM2func[%d]: Calculated IP checksum\n", queueID);
            }
            if (txsm) {
                TcpPtr tcp = ip ? TcpPtr(ip) : TcpPtr(ip6);
                UdpPtr udp = ip ? UdpPtr(ip) : UdpPtr(ip6);
                if (tcp) {
                    tcp->sum(0);
                    tcp->sum(cksum(tcp));
                    igbe->etherDeviceStats.txTcpChecksums++;
                    DPRINTF(EthernetDpdk, "TXM2func[%d]: Calculated TCP checksum\n", queueID);
                } else if (udp) {
                    assert(udp);
                    udp->sum(0);
                    udp->sum(cksum(udp));
                    igbe->etherDeviceStats.txUdpChecksums++;
                    DPRINTF(EthernetDpdk, "TXM2func[%d]: Calculated UDP checksum\n", queueID);
                } else {
                    panic("TXM2func: Told to checksum, but don't know how\n");
                }
            }
        }

        igbe->checkDrain();
        return;
    }

}
        
bool 
IGbE::TxM2funcContext::txM2funcStateMachine()
{
    //Check m2funcTxFifo and process the packets in the m2funcTxFifo & push to the txFifo
    
    // Check if txPacket is available to push to txFifo
    if (igbe->txPacketArray[queueID] && pktDone && receivedPktSize) {
        assert(ethPktSize == receivedPktSize);
        if (igbe->candidateTxQueue != queueID) {
            DPRINTF(EthernetDpdk, "TXM2func[%d]: Packet available to push FIFO, but not for this queue, candidate is %d\n", queueID, igbe->candidateTxQueue);

            return true; // It means this context needs to keep ticking
        } else {
            DPRINTF(EthernetDpdk, "TXM2func[%d]: packet placed in TX FIFO\n", queueID);
            bool success =
                (igbe->txFifo).push(igbe->txPacketArray[queueID]);
            igbe->txFifoTick = true && igbe->drainState() != DrainState::Draining;
            assert(success);
            
            // set successTxQueueSend to true
            igbe->successTxQueueSend = true;

            // unset processed packet
            igbe->txPacketArray[queueID] = nullptr;
            pktDone = false;
            ethPktSize = 0;
            receivedPktSize = 0;
            txDesc = 0;

            return true; // It means this context needs to keep ticking
        }
    }

    if (m2funcTxFifo.empty()) {
        DPRINTF(EthernetDpdk, "TXM2func[%d]: No packet to process\n", queueID);

        return true; // It means this context may need to keep ticking
    } else {
        // Pop the packet from m2funcTxFifo
        // Check if there is a packet, which is partially processed
        if (igbe->txPacketArray[queueID]) {
            // There is a EthPacket, which is currently being processed
            // If this condition is true, the corresponding txDesc and ethPktSize should be set
            DPRINTF(EthernetDpdk, "TXM2func[%d]: Partially processed packet exists\n", queueID);
            // Check if the packet is partially processed
            assert(ethPktSize > 0);
            assert(receivedPktSize > 0);
            assert(!pktDone);
            assert(txDesc != 0);
            assert(ethPktSize != receivedPktSize);

            // Process the packet
            m2funcTxFifoEntry entry = m2funcTxFifo.front();
            uint64_t cxl_pkt_size = entry.packetLength;
            uint64_t total_cxl_pkt_size = cxl_pkt_size; // For stat
            uint8_t *cxl_pkt_data = entry.packetData;
            // Manage cxl_pkt_size to not exceed the remaining packet size
            cxl_pkt_size = cxl_pkt_size > (ethPktSize - receivedPktSize) ? (ethPktSize - receivedPktSize) : cxl_pkt_size;
            DPRINTF(EthernetDpdk, "TXM2func[%d]: Processing cxl_packet of length: %ld, current receivedPktSize: %ld, ethPktSize: %ld\n", queueID, cxl_pkt_size, receivedPktSize, ethPktSize);
            processTxPacket(igbe->txPacketArray[queueID], cxl_pkt_size, cxl_pkt_data, 
                false, cxlMemTxdOp::cxlIxsm(txDesc), cxlMemTxdOp::cxlTxsm(txDesc));
            
            // Update stats
            igbe->etherDeviceStats.txBytesM2func += total_cxl_pkt_size;
            igbe->etherDeviceStats.txBytesM2funcData += cxl_pkt_size;
            
            m2funcTxFifo.pop_front();
            delete[] cxl_pkt_data;
            // Update the current processing ethpkt stats
            receivedPktSize += cxl_pkt_size;
            // Check if the packet is fully processed
            if (receivedPktSize == ethPktSize) {
                // The packet is fully processed
                pktDone = true;
                // Update bitmask
                updateBitmask(1);
                DPRINTF(EthernetDpdk, "TXM2func[%d]: Packet is fully processed and update bitmask\n", queueID);
            }

            return true; // It means this context may need to keep ticking
        } else {
            // There is no EthPacket, which is currently being processed
            
            m2funcTxFifoEntry entry = m2funcTxFifo.front();
            uint64_t cxl_pkt_size = entry.packetLength;
            uint64_t total_cxl_pkt_size = cxl_pkt_size; // For stat
            uint8_t *cxl_pkt_data = entry.packetData;
            assert(cxl_pkt_size > descSize);
            assert(txDesc == 0);
            assert(ethPktSize == 0);
            assert(receivedPktSize == 0);
            assert(!pktDone);
            // for CXL, place d2 part at first and d1 part at second - to match with not context case
            // Get first 8 bytes of the cxl_pkt_data
            txDesc = *(uint64_t*)cxl_pkt_data;
            // First check if this cxl_packet is for context packet
            if (cxlMemTxdOp::cxlIsContext(txDesc)) {
                // This is a context descriptor
                // Pkt size is 16 bytes
                assert(cxl_pkt_size >= 16);
                // TSO is used for the context descriptor
                // Set TSO entry
                uint64_t descData2 = *(uint64_t*)(cxl_pkt_data); //d2 part is at first for m2func
                uint64_t descData1 = *(uint64_t*)(cxl_pkt_data + 8); //d1 part is at second for m2func

                DPRINTF(EthernetDpdk, "TXM2func[%d]: Context descriptor received, descData2: %#X, descData1: %#X\n", queueID, descData2, descData1);

                if (!tsoEntry.tsoEnabled) {
                    tsoEntry.headerLen = cxlMemTxdOp::cxlHdrlen(descData1, descData2);
                    tsoEntry.mss = cxlMemTxdOp::cxlMss(descData2);
                    DPRINTF(EthernetDpdk, "TXM2func[%d]: TCP offload enabled for packet hdrlen: "
                            "%d mss: %d\n", queueID, tsoEntry.headerLen, tsoEntry.mss);
                    if (cxlMemTxdOp::cxlIsType(descData2, cxlMemTxdOp::CXL_M_TXD_CNXT) && 
                        cxlMemTxdOp::cxlTse(descData2)) {
                        DPRINTF(EthernetDpdk, "TXM2func[%d]: TCP offload enabled for packet hdrlen: "
                                "%d mss: %d paylen %d\n", queueID, tsoEntry.headerLen,
                                tsoEntry.mss, cxlMemTxdOp::cxlGetLen(descData2));
                        tsoEntry.tsoEnabled = true;
                        tsoEntry.totalLen = cxlMemTxdOp::cxlGetLen(descData2);
                        tsoEntry.loadedHeader = false;
                        tsoEntry.usedLen = 0;
                        tsoEntry.prevSeq = 0;
                        tsoEntry.txPktExists = false;
                        tsoEntry.ethPacketHasHeader = false;
                        tsoEntry.tsoPkts = 0;
                    } 
                } else {
                    DPRINTF(EthernetDpdk, "TXM2func[%d]: TCP offload already enabled\n", queueID);
                }
                m2funcTxFifo.pop_front();
                delete[] cxl_pkt_data;
                return true; // It means this context may need to keep ticking
            } else {
                // This is a cxl_packet that is with TX descriptor and payload
                // descriptor comes first and payload comes later
                // descriptor size is 8 bytes

                // Parse the descriptor to know the ethPktSize
                ethPktSize = cxlMemTxdOp::cxlGetLen(txDesc);
                
                // Check txFifo space
                if (ethPktSize > 0 && (igbe->txFifo.avail() > (ethPktSize))) {
                    // Can process the packet
                    // Make a new EthPacket
                    igbe->txPacketArray[queueID] = std::make_shared<EthPacketData>(16384);

                    // Check TSO is enabled
                    if (tsoEntry.tsoEnabled) {
                        panic("TSO is not supported yet\n");
                    } else {
                        // Not using TSO. Just copy the data to the EthPacket
                        // Reserve txFifo
                        igbe->txFifo.reserve(ethPktSize);
                        DPRINTF(EthernetDpdk, "TXM2func[%d]: Reserving %ld bytes in FIFO and "
                                "beginning processing of new EthPacket\n", queueID, ethPktSize);
                        // Set the current processing packet status
                        receivedPktSize = 0;
                        pktDone = false;
                        
                        // process the packet
                        cxl_pkt_size = cxl_pkt_size - descSize;
                        cxl_pkt_size = std::min(cxl_pkt_size, ethPktSize);
                        DPRINTF(EthernetDpdk, "TXM2func[%d]: Processing cxl_packet of length: %ld, current receivedPktSize: %ld, ethPktSize: %ld\n", queueID, cxl_pkt_size, receivedPktSize, ethPktSize);
                        processTxPacket(igbe->txPacketArray[queueID], cxl_pkt_size, (cxl_pkt_data + descSize), 
                            false, cxlMemTxdOp::cxlIxsm(txDesc), cxlMemTxdOp::cxlTxsm(txDesc));
                        
                        // Update stat
                        igbe->etherDeviceStats.txBytesM2func += total_cxl_pkt_size;
                        igbe->etherDeviceStats.txBytesM2funcDesc += descSize;
                        igbe->etherDeviceStats.txBytesM2funcData += cxl_pkt_size;
                        
                        m2funcTxFifo.pop_front();
                        delete[] cxl_pkt_data;

                        // Update the current processing ethpkt stats
                        receivedPktSize += cxl_pkt_size;
                        // Check if the packet is fully processed
                        if (receivedPktSize == ethPktSize) {
                            // The packet is fully processed
                            pktDone = true;
                            // Update bitmask
                            updateBitmask(1);
                            DPRINTF(EthernetDpdk, "TXM2func[%d]: Packet is fully processed and update bitmask\n", queueID);
                        }

                        return true; // It means this context may need to keep ticking                        
                    }

                } else if (ethPktSize > 0) {
                    // Cannot process the packet. FIFO is full
                    igbe->etherDeviceStats.txFifoFullCount++;
                    DPRINTF(EthernetDpdk, "TXM2func[%d]: FIFO full, stopping ticking until space "
                            "available in FIFO\n", queueID);
                    // Reset the context
                    txDesc = 0;
                    ethPktSize = 0;
                    receivedPktSize = 0;
                    pktDone = false;

                    return true; // It means this context may need to keep ticking
                }
            }

        }
    }
}

void 
IGbE::TxM2funcContext::writeM2funcPacket(PacketPtr pkt)
{
    /**
     * Receive packet from the host (TX) and push it to the m2funcTxFifo
     */
    assert(pkt->getSize() <= igbe->flitSize);

    m2funcTxFifoEntry entry;
    entry.packetLength = pkt->getSize();
    entry.packetData = new uint8_t[entry.packetLength];
    entry.txCount = txCount;
    memcpy(entry.packetData, pkt->getPtr<uint8_t>(), (size_t) (entry.packetLength));
    // TODO - JM : have to check m2funcTxFifo size?
    // Just make stats for the max size of m2funcTxFifo
    m2funcTxFifo.push_back(entry);
    uint64_t currM2funcTxFifoSize = m2funcTxFifo.size();
    uint64_t maxM2funcTxFifoSize = igbe->etherDeviceStats.m2funcTxFifoMaxLen.value();
    if (currM2funcTxFifoSize > maxM2funcTxFifoSize) {
        igbe->etherDeviceStats.m2funcTxFifoMaxLen = currM2funcTxFifoSize;
    }
    DPRINTF(EthernetDpdk, "TXM2func[%d]: Packet %p pushed to m2funcTxFifo with length: %ld with txCount: %ld. Current m2funcTxFifo max len: %ld\n", queueID, pkt, entry.packetLength, txCount, maxM2funcTxFifoSize);
    txCount++;

}

void 
IGbE::TxM2funcContext::expandBitmask()
{
    /**
     * Expand the bitmask when the updateCursor exceeds the current size
     * 
     */
    bitmaskWrap.bitmaskSize *= bitmask_expand_factor; // Double the size

    // Reallocate the bitmask
    uint8_t* new_bitmask = reinterpret_cast<uint8_t*>(realloc(bitmaskWrap.bitmask, (bitmaskWrap.bitmaskSize / 8)));
    if (new_bitmask == NULL) {
        panic("Failed to expand bitmask\n");
    }

    // Update the bitmask pointer only after realloc succeeds
    bitmaskWrap.bitmask = new_bitmask;

    // Initialize the newly allocated portion
    memset(
        bitmaskWrap.bitmask + (bitmaskWrap.bitmaskSize / (bitmask_expand_factor * 8)), // Start of the newly allocated part
        0,
        (bitmaskWrap.bitmaskSize / (bitmask_expand_factor * 8))  // Newly allocated size in bytes
    );
}

void 
IGbE::TxM2funcContext::updateBitmask(bool success)
{
    /**
     * Update the bitmask with success or failure for the current TX packet
     * 
     */

    // Check if the updateCursor exceeds the current size
    if (bitmaskWrap.updateCursor >= bitmaskWrap.bitmaskSize) {
        expandBitmask();
    }

    // Set the corresponding bit in the bitmask
    if (success) {
        bitmaskWrap.bitmask[bitmaskWrap.updateCursor / 8] |= (1 << (bitmaskWrap.updateCursor % 8));
    } else {
        bitmaskWrap.bitmask[bitmaskWrap.updateCursor / 8] &= ~(1 << (bitmaskWrap.updateCursor % 8));
    }

    // Increment the updateCursor
    bitmaskWrap.updateCursor++;
}

void 
IGbE::TxM2funcContext::readBitmask(PacketPtr pkt)
{
    /**
     * Set the window size bitmask to pkt
     * 
     */
    uint64_t pktSize = pkt->getSize();

    uint64_t startIdx = bitmaskWrap.readCursor;
    uint8_t response[pktSize];
    memset(response, 0, pktSize);

    assert(bitmaskWrap.readCursor < bitmaskWrap.updateCursor);
    assert((pktSize * 8) >= bitmask_window_size_bits);

    // Check whether bit is zero within the window size
    bool has_zero = false;
    uint64_t zero_idx = 0;
    for (uint64_t i = 0; i < bitmask_window_size_bits; i++) {
        uint64_t bitIdx = startIdx + i;

        if (bitmaskWrap.bitmask[bitIdx / 8] & (1 << (bitIdx % 8))) {
            // Bit is 1
            response[i / 8] |= (1 << (i % 8));
        } else {
            // Bit is 0
            if (!has_zero) {
                // First zero bit
                has_zero = true;
                zero_idx = bitIdx;
            }
        }
    } 

    pkt->setData(response);

    // Update stat
    igbe->etherDeviceStats.txBytesM2func += pktSize;
    igbe->etherDeviceStats.txBytesM2funcBitMask += (bitmask_window_size_bits / 8);

    // Update the readCursor
    if (has_zero) {
        bitmaskWrap.readCursor = zero_idx;
    } else {
        bitmaskWrap.readCursor += bitmask_window_size_bits;
    }

    if (bitmaskWrap.readCursor == startIdx) {
        // Warning: No progress
        DPRINTF(EthernetDpdk, "TXM2func[%d]: No progress in reading bitmask\n", queueID);
        printf("TXM2func[%d]: No progress in reading bitmask\n", queueID);
    }

}

void 
IGbE::TxM2funcContext::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(queueID);
    SERIALIZE_SCALAR(txContextFifoSize);
    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(ethPktSize);
    SERIALIZE_SCALAR(txDesc);
    SERIALIZE_SCALAR(receivedPktSize);
    SERIALIZE_SCALAR(descSize);
    SERIALIZE_SCALAR(isTcp);
    SERIALIZE_SCALAR(pktWaiting);
    SERIALIZE_SCALAR(pktMultiDesc);

    paramOut(cp, csprintf("tsoEntry.tsoEnabled"), tsoEntry.tsoEnabled);
    paramOut(cp, csprintf("tsoEntry.txPktExists"), tsoEntry.txPktExists);
    paramOut(cp, csprintf("tsoEntry.mss"), tsoEntry.mss);
    paramOut(cp, csprintf("tsoEntry.headerLen"), tsoEntry.headerLen);
    paramOut(cp, csprintf("tsoEntry.totalLen"), tsoEntry.totalLen);
    paramOut(cp, csprintf("tsoEntry.loadedHeader"), tsoEntry.loadedHeader);
    paramOut(cp, csprintf("tsoEntry.ethPacketHasHeader"), tsoEntry.ethPacketHasHeader);
    paramOut(cp, csprintf("tsoEntry.usedLen"), tsoEntry.usedLen);
    paramOut(cp, csprintf("tsoEntry.prevSeq"), tsoEntry.prevSeq);
    paramOut(cp, csprintf("tsoEntry.tsoPkts"), tsoEntry.tsoPkts);

    SERIALIZE_SCALAR(txCount);

    // Serialize m2funcTxFifo
    uint64_t m2funcTxFifoSize = m2funcTxFifo.size();
    SERIALIZE_SCALAR(m2funcTxFifoSize);
    for (uint64_t i = 0; i < m2funcTxFifoSize; i++) {
        const m2funcTxFifoEntry &entry = m2funcTxFifo[i];
        paramOut(cp, csprintf("m2funcTxFifo[%d].txCount", i), entry.txCount);
        paramOut(cp, csprintf("m2funcTxFifo[%d].packetLength", i), entry.packetLength);
        arrayParamOut(cp, csprintf("m2funcTxFifo[%d].packetData", i), entry.packetData, entry.packetLength);
    }

    // Serialize bitmask
    SERIALIZE_SCALAR(bitmaskWrap.bitmaskSize);
    SERIALIZE_SCALAR(bitmaskWrap.updateCursor);
    SERIALIZE_SCALAR(bitmaskWrap.readCursor);
    arrayParamOut(cp, "bitmaskWrap.bitmask", bitmaskWrap.bitmask, bitmaskWrap.bitmaskSize / 8);
}

void
IGbE::TxM2funcContext::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(queueID);
    UNSERIALIZE_SCALAR(txContextFifoSize);
    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(ethPktSize);
    UNSERIALIZE_SCALAR(txDesc);
    UNSERIALIZE_SCALAR(receivedPktSize);
    UNSERIALIZE_SCALAR(descSize);
    UNSERIALIZE_SCALAR(isTcp);
    UNSERIALIZE_SCALAR(pktWaiting);
    UNSERIALIZE_SCALAR(pktMultiDesc);

    paramIn(cp, csprintf("tsoEntry.tsoEnabled"), tsoEntry.tsoEnabled);
    paramIn(cp, csprintf("tsoEntry.txPktExists"), tsoEntry.txPktExists);
    paramIn(cp, csprintf("tsoEntry.mss"), tsoEntry.mss);
    paramIn(cp, csprintf("tsoEntry.headerLen"), tsoEntry.headerLen);
    paramIn(cp, csprintf("tsoEntry.totalLen"), tsoEntry.totalLen);
    paramIn(cp, csprintf("tsoEntry.loadedHeader"), tsoEntry.loadedHeader);
    paramIn(cp, csprintf("tsoEntry.ethPacketHasHeader"), tsoEntry.ethPacketHasHeader);
    paramIn(cp, csprintf("tsoEntry.usedLen"), tsoEntry.usedLen);
    paramIn(cp, csprintf("tsoEntry.prevSeq"), tsoEntry.prevSeq);
    paramIn(cp, csprintf("tsoEntry.tsoPkts"), tsoEntry.tsoPkts);

    UNSERIALIZE_SCALAR(txCount);

    // Unserialize m2funcTxFifo
    uint64_t m2funcTxFifoSize = 0;
    UNSERIALIZE_SCALAR(m2funcTxFifoSize);
    m2funcTxFifo.resize(m2funcTxFifoSize);
    for (uint64_t i = 0; i < m2funcTxFifoSize; i++) {
        m2funcTxFifoEntry &entry = m2funcTxFifo[i];
        paramIn(cp, csprintf("m2funcTxFifo[%d].txCount", i), entry.txCount);
        paramIn(cp, csprintf("m2funcTxFifo[%d].packetLength", i), entry.packetLength);
        arrayParamIn(cp, csprintf("m2funcTxFifo[%d].packetData", i), entry.packetData, entry.packetLength);
    }

    // Unserialize bitmask
    UNSERIALIZE_SCALAR(bitmaskWrap.bitmaskSize);
    UNSERIALIZE_SCALAR(bitmaskWrap.updateCursor);
    UNSERIALIZE_SCALAR(bitmaskWrap.readCursor);
    bitmaskWrap.bitmask = (uint8_t*)malloc((bitmaskWrap.bitmaskSize / 8));
    arrayParamIn(cp, "bitmaskWrap.bitmask", bitmaskWrap.bitmask, bitmaskWrap.bitmaskSize / 8);
}

///////////////////////////////////// IGbE /////////////////////////////////

void
IGbE::restartClock()
{
    if (!tickEvent.scheduled() && (rxTick || txTick || txFifoTick) &&
        drainState() == DrainState::Running)
        schedule(tickEvent, clockEdge(Cycles(1)));
}

DrainState
IGbE::drain()
{
    unsigned int count(0);
    // if (rxDescCache.hasOutstandingEvents() ||
    //     txDescCache.hasOutstandingEvents()) {
    //     count++;
    // }
    //Check rxDescCacheArray and txDescCacheArray
    #ifndef USE_ENSO
    if (commType == CommunicationType::RING) {
        bool rxDescCacheOutstanding = false;
        bool txDescCacheOutstanding = false;
        for (int i = 0; i < numQueues; i++) {
            if (rxDescCacheArray[i]->hasOutstandingEvents()) {
                rxDescCacheOutstanding = true;
            }
            if (txDescCacheArray[i]->hasOutstandingEvents()) {
                txDescCacheOutstanding = true;
            }
        }
        if (rxDescCacheOutstanding || txDescCacheOutstanding) {
            count++;
        }
    }
    #else
    if(rxNotifBufManager.hasOutstandingEvents() || txNotifBufManager.hasOutstandingEvents())
        count++;

    #endif

    txFifoTick = false;
    txTick = false;
    rxTick = false;

    if (tickEvent.scheduled())
        deschedule(tickEvent);

    if (count) {
        DPRINTF(Drain, "IGbE not drained\n");
        return DrainState::Draining;
    } else
        return DrainState::Drained;
}

void
IGbE::drainResume()
{
    Drainable::drainResume();

    txFifoTick = true;
    txTick = true;
    rxTick = true;

    restartClock();
    DPRINTF(EthernetSM, "resuming from drain");
}

void
IGbE::checkDrain()
{
    if (drainState() != DrainState::Draining)
        return;

    txFifoTick = false;
    txTick = false;
    rxTick = false;

    // if (!rxDescCache.hasOutstandingEvents() &&
    //     !txDescCache.hasOutstandingEvents()) {
    //     DPRINTF(Drain, "IGbE done draining, processing drain event\n");
    //     signalDrainDone();
    // }
    //Check rxDescCacheArray and txDescCacheArray
    #ifndef USE_ENSO
    if (commType == CommunicationType::RING) {
        bool rxDescCacheOutstanding = false;
        bool txDescCacheOutstanding = false;
        for (int i = 0; i < numQueues; i++) {
            if (rxDescCacheArray[i]->hasOutstandingEvents()) {
                rxDescCacheOutstanding = true;
            }
            if (txDescCacheArray[i]->hasOutstandingEvents()) {
                txDescCacheOutstanding = true;
            }
        }
        if (!rxDescCacheOutstanding && !txDescCacheOutstanding) {
            DPRINTF(Drain, "IGbE done draining, processing drain event\n");
            signalDrainDone();
        }
    }
    #else
    if(!rxNotifBufManager.hasOutstandingEvents() && !txNotifBufManager.hasOutstandingEvents())
    {
        DPRINTF(Drain, "IGbE done draining, processing drain event\n");
        signalDrainDone();
    }
    #endif
}

void 
IGbE::enableSmTx() 
{
    if (drainState() != DrainState::Draining) {
        txTick = true;
        restartClock();
    }
}

void 
IGbE::enableSmRx() 
{
    if (drainState() != DrainState::Draining) {
        rxTick = true;
        restartClock();
    }
}

// void
bool
IGbE::txStateMachine(int queueID)
{   
    bool txTickQueue = txTick;
    if (!regs.tctl.en()) {
        // txTick = false;
        txTickQueue = false;
        DPRINTF(EthernetSM, "TXD[%d]: TX disabled, stopping ticking\n", queueID);
        DPRINTF(EthernetDpdk, "TXD[%d]: TX disabled, stopping ticking\n", queueID);
        // return;
        return txTickQueue;
    }

    // If we have a packet available and it's length is not 0 (meaning it's not
    // a multidescriptor packet) put it in the fifo, otherwise an the next
    // iteration we'll get the rest of the data
    // if (txPacket && txDescCache.packetAvailable()
    //     && !txDescCache.packetMultiDesc() && txPacket->length) {
    // Check txDescCacheArray has ready ethPacket
    if (txDescCacheArray[queueID]->hasReadyEthPacket()) {
        if (candidateTxQueue != queueID) {
            DPRINTF(EthernetSM, "TXD[%d]: Packet available to push FIFO, but not for this queue, candidate is %d\n", queueID, candidateTxQueue);
            DPRINTF(EthernetDpdk, "TXD[%d]: Packet available to push FIFO, but not for this queue, candidate is %d\n", queueID, candidateTxQueue);

            return txTickQueue;
        } else {
            DPRINTF(EthernetSM, "TXD[%d]: packet placed in TX FIFO\n", queueID);
            DPRINTF(EthernetDpdk, "TXD[%d]: packet placed in TX FIFO\n", queueID);

            txPacketArray[queueID] = txDescCacheArray[queueID]->getReadyEthPacket();
            assert(txPacketArray[queueID]);
            bool success =
                txFifo.push(txPacketArray[queueID]);
            txFifoTick = true && drainState() != DrainState::Draining;
            assert(success);

            txPacketArray[queueID] = nullptr;
            txDescCacheArray[queueID]->writebackGlobal((cacheBlockSize()-1)>>4);
            // set successTxQueueSend to true
            successTxQueueSend = true;
            
            return txTickQueue;
        }
    }

    // Only support descriptor granularity
    if (regs.txdctl_array[queueID].lwthresh() &&
        txDescCacheArray[queueID]->descLeft() < (regs.txdctl_array[queueID].lwthresh() * 8)) {
        DPRINTF(EthernetSM, "TXD[%d]: LWTHRESH caused posting of TXDLOW\n", queueID);
        postInterrupt(IT_TXDLOW);
    }

    // if (!txDescCache.packetWaiting()) {
    if (!txDescCacheArray[queueID]->packetHdrWaiting() && txDescCacheArray[queueID]->hasFreeDMAEngine()) {
        // Check DMA is doing or not.
        //  1. Check txDescCacheGlobal's waiting for packet header (packetHdrWaiting). Meaningful only when useTso
        //  2. Check txDescCacheGlobal's DMA engine are all busy.
        // If none of the above, we have to check descriptor is sufficient or not.
        if (txDescCacheArray[queueID]->descLeft() == 0) {
            etherDeviceStats.txRingBufferFull++; //TODO - jm: make stat as array
            postInterrupt(IT_TXQE);
            txDescCacheArray[queueID]->writebackGlobal(0);
            txDescCacheArray[queueID]->fetchDescriptorsGlobal();
            DPRINTF(EthernetSM, "TXD[%d]: No descriptors left in ring, forcing "
                    "writeback stopping ticking and posting TXQE\n", queueID);
            DPRINTF(EthernetDpdk, "TXD[%d]: No descriptors left in ring, forcing "
                    "writeback stopping ticking and posting TXQE\n", queueID);
            txTickQueue = false;
            return txTickQueue;
        }

        if (!(txDescCacheArray[queueID]->descUnused())) {
            txDescCacheArray[queueID]->fetchDescriptorsGlobal();
            etherDeviceStats.txDescCacheFullCount++; //TODO - jm: make stat as array
            DPRINTF(EthernetSM, "TXD[%d]: No descriptors available in cache, "
                    "fetching and stopping ticking\n", queueID);
            DPRINTF(EthernetDpdk, "TXD[%d]: No descriptors available in cache, "
                    "fetching and stopping ticking\n", queueID);
            txTickQueue = false;
            return txTickQueue;
        }


        // txDescCache.processContextDesc();
        txDescCacheArray[queueID]->processContextDesc();
        // if (txDescCache.packetWaiting()) {
        if (txDescCacheArray[queueID]->packetHdrWaiting()) {
            DPRINTF(EthernetSM,
                    "TXD[%d]: Fetching TSO header, stopping ticking\n", queueID);
            DPRINTF(EthernetDpdk,
                    "TXD[%d]: Fetching TSO header, stopping ticking\n", queueID);
            // txTick = false;
            // return;
            txTickQueue = false;
            return txTickQueue;
        }

        // unsigned size = txDescCache.getPacketSize(txPacket);
        unsigned size = txDescCacheArray[queueID]->getPacketSize();
        if (size > 0 && txFifo.avail() > size) {
            DPRINTF(EthernetSM, "TXD[%d]: Reserving %d bytes in FIFO and "
                    "beginning DMA of next packet\n", queueID, size);
            DPRINTF(EthernetDpdk, "TXD[%d]: Reserving %d bytes in FIFO and "
                    "beginning DMA of next packet\n", queueID, size);
            txFifo.reserve(size);
            // txDescCache.getPacketData(txPacket);
            txDescCacheArray[queueID]->getPacketDataGlobal();
        } else if (size == 0) {
            DPRINTF(EthernetSM, "TXD[%d]: getPacketSize returned: %d\n", queueID, size);
            DPRINTF(EthernetSM,
                    "TXD[%d]: No packets to get, writing back used descriptors\n", queueID);
            DPRINTF(EthernetDpdk, "TXD[%d]: getPacketSize returned: %d\n", queueID, size);
            DPRINTF(EthernetDpdk,
                    "TXD[%d]: No packets to get, writing back used descriptors\n", queueID);
            txDescCacheArray[queueID]->writebackGlobal(0);
        } else {
            etherDeviceStats.txFifoFullCount++; //TODO - jm: make stat as array
            DPRINTF(EthernetSM, "TXD[%d]: FIFO full, stopping ticking until space "
                    "available in FIFO\n", queueID);
            DPRINTF(EthernetDpdk, "TXD[%d]: FIFO full, stopping ticking until space "
                    "available in FIFO\n", queueID);
            // txTick = false;
            txTickQueue = false;
        }


        // return;
        return txTickQueue;
    }
    DPRINTF(EthernetSM, "TXD[%d]: Nothing to do, stopping ticking\n", queueID);
    DPRINTF(EthernetDpdk, "TXD[%d]: Nothing to do, stopping ticking\n", queueID);
    // txTick = false;
    txTickQueue = false;
    return txTickQueue;
}

uint32_t 
IGbE::computeHash(uint8_t *input, int length) {
    /* Below is the pseudo code of the algorithm in Intel 8257x datasheet
    * ComputeHash(input[], N)
    *   For hash-input input[] of length N bytes (8N bits) and a random secret key K of 320 
    *   bits
    *   Result = 0;
    *   For each bit b in input[] {
    *   if (b == 1) then Result ^= (left-most 32 bits of K);
    *   shift K left 1 bit position;
    *   }
    *   return Result;
    *
    *  K is a 320-bit random secret key, which is stored in the RSSRK register.
    *  K[0] is the left-most byte, MSB of K[0] is the left-most bit of K.
    *  K[0] = rssrk[0] -> 8 bits
    *  K[1] = rssrk[1] -> 8 bits
    * */
    uint32_t result = 0;
    uint64_t keyStream = 0; // 64-bit buffer to hold the sliding window of key bits
    int bitPos = 0;

    union e1000_rssrk_reg {
        uint64_t dword;
        uint8_t rssrk[8];
    } rssrkReg;

    // Load the initial 64 bits of the key stream
    for (int i = 0; i < 8; i++) {
        rssrkReg.rssrk[i] = regs.rssrk[i];
    }
    keyStream = rssrkReg.dword;

    // Load the initial 64 bits of the key stream
    // for (int i = 0; i < 8; i++) {
    //     keyStream = (keyStream << 8) | uint64_t(regs.rssrk[i]);
    // }

    // Iterate over the input bits
    for (int i = 0; i < length; i++) {
        for (int bit = 0; bit < 8; bit++) {
            if (input[i] & (1 << bit)) {
                result ^= uint32_t(keyStream >> 32); // XOR with the left-most 32 bits of the key
            }
            // Shift the key stream left by 1 bit
            keyStream = (keyStream << 1) & 0xFFFFFFFFFFFFFFFF;
            
            if (bitPos < 320) {
                // Load the next bit from the key into the right-most bit of the key stream
                int keyIndex = (bitPos + 64) / 8; // 64 bits are already loaded
                int bitIndex = 7 - ((bitPos + 64) % 8); // 7 is the MSB
                uint64_t newBit = (uint64_t((regs.rssrk[keyIndex % 40] >> bitIndex) & 1));
                keyStream |= newBit;
            }
            bitPos++;
        }
    }

    return result;
}


int
IGbE::doRSS(EthPacketPtr pkt)
{
    /* 
        1. Make input for computeHash corresponding to mrqc's tcpipv4, ipv4, tcpipv6, ipv6ex, ipv6
        @x-y denotes bytes x through y of the packet, where byte 0 is the first byte of the IP header
        If tcpipv4:
            Concatenate SourceAddress, DestinationAddress, SourcePort, DestinationPort into one single byte-array, preserving the order in which they occurred in the packet: 
            Input[12] = @12-15, @16-19, @20-21, @22-23.
            Result = ComputeHash(Input, 12);
        If ipv4:
            Concatenate SourceAddress and DestinationAddress into one single byte-array
            Input[8] = @12-15, @16-19
            Result = ComputeHash(Input, 8)
        If tcpipv6:
            Concatenate SourceAddress, DestinationAddress, SourcePort, DestinationPort into one single byte-array, preserving the order in which they occurred in the packet
            Input[36] = @8-23, @24-39, @40-41, @42-43
            Result = ComputeHash(Input, 36)
        If ipv6:
            Input[32] = @8-23, @24-39
            Result = ComputeHash(Input, 32) 
        
        2. use Input to get hash value from computeHash function.
        3. store the hash value to Packet's RSS hash field.
        4. access reta table with hash value[7:0] and get the index.
        5. return the index.
    
    */
    uint8_t *input = new uint8_t[40];
    int length = 0;
    uint32_t hash = 0;

    // Check if the packet is an IPv6 packet
    IpPtr ip = IpPtr(pkt);
    Ip6Ptr ip6 = Ip6Ptr(pkt);
    if (ip || ip6) {
        if (ip6) {
            TcpPtr tcp = TcpPtr(ip6);
            if (tcp) {
                if (regs.mrqc.tcpipv6()) {
                    // tcpipv6
                    int ipHeaderLen = IP6_ADDR_LEN;
                    const uint8_t *srcAddr = ip6->src();
                    const uint8_t *dstAddr = ip6->dst();
                    uint16_t srcPort = tcp->sport();
                    uint16_t dstPort = tcp->dport();
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = srcAddr[i];
                    }
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = dstAddr[i];
                    }
                    input[length++] = (srcPort >> 8) & 0xFF;
                    input[length++] = srcPort & 0xFF;
                    input[length++] = (dstPort >> 8) & 0xFF;
                    input[length++] = dstPort & 0xFF;
                } else if (regs.mrqc.ipv6()) {
                    // ipv6
                    int ipHeaderLen = IP6_ADDR_LEN;
                    const uint8_t *srcAddr = ip6->src();
                    const uint8_t *dstAddr = ip6->dst();
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = srcAddr[i];
                    }
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = dstAddr[i];
                    }
                }
            } else {
                if (regs.mrqc.ipv6()) {
                    // ipv6
                    int ipHeaderLen = IP6_ADDR_LEN;
                    const uint8_t *srcAddr = ip6->src();
                    const uint8_t *dstAddr = ip6->dst();
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = srcAddr[i];
                    }
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = dstAddr[i];
                    }
                }
            }
        } else {
            TcpPtr tcp = TcpPtr(ip);
            if (tcp) {
                if (regs.mrqc.tcpipv4()) {
                    // tcpipv4
                    int ipHeaderLen = IP_ADDR_LEN;
                    uint32_t srcAddr = ip->src();
                    uint32_t dstAddr = ip->dst();
                    uint16_t srcPort = tcp->sport();
                    uint16_t dstPort = tcp->dport();
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = (srcAddr >> (8 * (ipHeaderLen - i - 1))) & 0xFF;
                    }
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = (dstAddr >> (8 * (ipHeaderLen - i - 1))) & 0xFF;
                    }
                    input[length++] = (srcPort >> 8) & 0xFF;
                    input[length++] = srcPort & 0xFF;
                    input[length++] = (dstPort >> 8) & 0xFF;
                    input[length++] = dstPort & 0xFF;
                } else if (regs.mrqc.ipv4()) {
                    // ipv4
                    int ipHeaderLen = IP_ADDR_LEN;
                    uint32_t srcAddr = ip->src();
                    uint32_t dstAddr = ip->dst();
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = (srcAddr >> (8 * (ipHeaderLen - i - 1))) & 0xFF;
                    }
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = (dstAddr >> (8 * (ipHeaderLen - i - 1))) & 0xFF;
                    }
                }
            } else {
                if (regs.mrqc.ipv4()) {
                    // ipv4
                    int ipHeaderLen = IP_ADDR_LEN;
                    uint32_t srcAddr = ip->src();
                    uint32_t dstAddr = ip->dst();
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = (srcAddr >> (8 * (ipHeaderLen - i - 1))) & 0xFF;
                    }
                    for (int i = 0; i < ipHeaderLen; i++) {
                        input[length++] = (dstAddr >> (8 * (ipHeaderLen - i - 1))) & 0xFF;
                    }
                }
            }

        }

        if (length != 0) {
            hash = computeHash(input, length);
            pkt->rssHash = hash; 
            // Access RETA table with hash value[7:0] and get the rx queue idx
            int RETAIdx = hash & 0xFF;
            assert(RETAIdx < 128);
            int rxQueueIdx = regs.reta_array[RETAIdx];
            delete[] input;
            DPRINTF(EthernetDpdk, "At doRSS: Packet(%p) RSS hash is %d, RETAIdx is %d, rxQueueIdx is %d\n", pkt.get(), hash, RETAIdx, rxQueueIdx);
            return rxQueueIdx;
        } else {
            DPRINTF(EthernetDpdk, "At doRSS: Length is 0\n");
            delete[] input;
            return 0;
        }
    } else {
        // Not an IP packet
        delete[] input;
        // do round-robin using prevRSSQueue value
        if (prevRSSQueue < numQueues - 1) {
            prevRSSQueue++;
        } else {
            prevRSSQueue = 0;
        }
        DPRINTF(EthernetDpdk, "At doRSS: Packet(%p) is not an IP packet, using round-robin, prevRSSQueue is %d\n", pkt.get(), prevRSSQueue);

        // Check target queue is empty. If not, find the next empty queue
        if (rxPacketArray[prevRSSQueue] != nullptr) {
            for (int i = 0; i < numQueues; i++) {
                if (rxPacketArray[i] == nullptr) {
                    // Find the empty queue
                    // set prevRSSQueue to the empty queue
                    prevRSSQueue = i;
                    DPRINTF(EthernetDpdk, "At doRSS: Packet(%p) find new empty prevRSSQueue is %d\n", pkt.get(), prevRSSQueue);
                    break;
                }
            }
        }

        return prevRSSQueue;
    }
}

void
IGbE::rxFifotoRxDesc() {
    // check if rxFifo has any packet
    // decide the queueID based on the packet (apply RSS if enabled)
    // push the packet to the rxPacketArray[queueID] if it is null 

    if (rxFifo.empty()) {
        DPRINTF(EthernetDpdk, "RXF: rxFifo is empty\n");
        return;
    }

    EthPacketPtr pkt = rxFifo.front();
    
    if (regs.mrqc.en() == MRQC_ENABLE_RSS_4Q) {
        // RSS enabled
        DPRINTF(EthernetDpdk, "RXF: RSS enabled\n");
        int queueID = doRSS(pkt);
        assert(queueID < numQueues);
        assert(queueID >= 0);
        if (rxPacketArray[queueID] == nullptr) {
            rxPacketArray[queueID] = pkt;
            rxFifo.pop();
            DPRINTF(EthernetDpdk, "RXF: Packet(%p) pushed to rxPacketArray[%d]\n", pkt.get(), queueID);
        } else {
            DPRINTF(EthernetDpdk, "RXF: Packet(%p) RSS result is %d, but rxPacketArray[%d] is not null\n", pkt.get(), queueID, queueID);

            for (int i = 0; i < numQueues; i++) {
                if (i != queueID && rxPacketArray[i] == nullptr) {
                    DPRINTF(EthernetDpdk, "RXF: Packet(%p) RSS result is %d, but rxPacketArray[%d] is not null", pkt.get(), queueID, i);
                    etherDeviceStats.rxFifoNotEmptyRSSBad++;
                    break;
                }
            }
        }
    } else {
        // RSS disabled
        DPRINTF(EthernetDpdk, "RXF: RSS disabled\n");
        if (rxPacketArray[0] == nullptr) {
            rxPacketArray[0] = pkt;
            rxFifo.pop();
            DPRINTF(EthernetDpdk, "RXF: Packet(%p) pushed to rxPacketArray[0]\n", pkt.get());
        } else {
            DPRINTF(EthernetDpdk, "RXF: Packet(%p) pushed to rxPacketArray[0] is not null\n", pkt.get());
        }
    }  
}

void
IGbE::txPackettoTxFifo() {
    // If we have a packet available and it's length is not 0 (meaning it's not 
    // a multidescriptor packet) just give the credit
    for (int i = 0; i < numQueues; i++) {
        int queueID = (candidateTxQueue + i) % numQueues;
        if (commType == CommunicationType::RING) {
            if (txDescCacheArray[queueID]->hasReadyEthPacket()) {
                DPRINTF(EthernetSM, "TXF[%d]: packet can be placed in TX FIFO, so get the credit!\n", queueID);
                DPRINTF(EthernetDpdk, "TXF[%d]: packet can be placed in TX FIFO, so get the credit!\n", queueID);

                candidateTxQueue = queueID;
                //just select the queue, not push the packet to the fifo here
                return;
            }
        } else if (commType == CommunicationType::M2FUNC) {
            if (txPacketArray[queueID] && txM2funcContextArray[queueID]->ethPktDone()) {
                DPRINTF(EthernetDpdk, "TXF[%d]: packet can be placed in TX FIFO, so get the credit!\n", queueID);

                candidateTxQueue = queueID;
                //just select the queue, not push the packet to the fifo here
                return;
            }
        } else {
            panic("Communication type not supported - txPackettoTxFifo\n");
        }
    }
}

char currState = 'A', nextState;
void
IGbE::updateDropFSM(int rxFifoFull, int rxRingFull, int txRingFull, int txFifoFull)
{
    // State Encoding: rxFifoFull,rxRingFull,txRingFull
    // 000 -> "A" || 001 -> "B" || 010 -> "C" || 011 = "D"
    // 100 -> "E" || 101 -> "F" || 110 -> "G" || 111 = "H"
    
    switch(currState) {
        case 'A':
            if(!rxFifoFull && !rxRingFull && txRingFull) {
                nextState = 'B';
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && !txRingFull){
                nextState = 'C';
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(rxFifoFull && !rxRingFull && !txRingFull){
                nextState = 'E';
                etherDeviceStats.dmaDrops++;
            }
            else if(rxFifoFull && !rxRingFull && txRingFull){
                nextState = 'F';
                etherDeviceStats.dmaDrops++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'G';
                etherDeviceStats.coreDrops++;
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'D';
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'H';
                etherDeviceStats.txDrops++;
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else
                nextState = 'A';
            break;
        case 'B':
            if(!rxFifoFull && !rxRingFull && !txRingFull)
                nextState = 'A';
            else if(!rxFifoFull && rxRingFull && !txRingFull){
                nextState = 'C';
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(rxFifoFull && !rxRingFull && !txRingFull){
                nextState = 'E';
                etherDeviceStats.dmaDrops++; //confirm-done
            }
            else if(rxFifoFull && !rxRingFull && txRingFull){
                nextState = 'F';
                etherDeviceStats.dmaDrops++; //confirm-done
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'G';
                etherDeviceStats.coreDrops++;
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'D';
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'H';
                etherDeviceStats.txDrops++; //initially txDrops
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else {
                nextState = 'B';
                // etherDeviceStats.txRingBufferFull++;
            }
            break;
        case 'C':
            if(!rxFifoFull && !rxRingFull && !txRingFull)
                nextState = 'A';
            else if(!rxFifoFull && !rxRingFull && txRingFull) {
                nextState = 'B';
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && !rxRingFull && !txRingFull){
                nextState = 'E';
                etherDeviceStats.coreDrops++;
            }
            else if(rxFifoFull && !rxRingFull && txRingFull){
                nextState = 'F';
                etherDeviceStats.coreDrops++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'G';
                // if(txDescCache.descLeft() == 1024 || (!txDescCache.descUnused() && txDescCache.descLeft()))
                //     etherDeviceStats.txDrops++;
                // else
                etherDeviceStats.coreDrops++;
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'D';
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'H';
                etherDeviceStats.coreDrops++;
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }   
            else {
                nextState = 'C';
                // etherDeviceStats.rxRingBufferFull++;
            }
            break;
        case 'D':
            if(!rxFifoFull && !rxRingFull && !txRingFull)
                nextState = 'A';
            else if(!rxFifoFull && !rxRingFull && txRingFull) {
                nextState = 'B';
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && !rxRingFull && !txRingFull){
                nextState = 'E';
                etherDeviceStats.coreDrops++; //confirm-done (initially txDrops)
            }
            else if(rxFifoFull && !rxRingFull && txRingFull){
                nextState = 'F';
                etherDeviceStats.coreDrops++; //confirm-done (initially txDrops)
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'G';
                etherDeviceStats.coreDrops++; //confirm-done (initially txDrops)
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'C';
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'H';
                etherDeviceStats.txDrops++;
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }   
            else {
                nextState = 'D';
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            break;
        case 'E':
            if(!rxFifoFull && !rxRingFull && !txRingFull)
                nextState = 'A';
            else if(!rxFifoFull && !rxRingFull && txRingFull) {
                nextState = 'B';
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && txRingFull){
                nextState = 'D';
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && !rxRingFull && txRingFull){
                nextState = 'F';
                etherDeviceStats.dmaDrops++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'G';
                etherDeviceStats.coreDrops++;
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'C';
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'H';
                etherDeviceStats.txDrops++; //confirm-done (initially txDrops)
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }   
            else{
                nextState = 'E';
                etherDeviceStats.dmaDrops++;
            }  
            break;
        case 'F':
            if(!rxFifoFull && !rxRingFull && !txRingFull)
                nextState = 'A';
            else if(!rxFifoFull && !rxRingFull && txRingFull) {
                nextState = 'B';
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && txRingFull){
                nextState = 'D';
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && !rxRingFull && !txRingFull){
                nextState = 'E';
                etherDeviceStats.dmaDrops++;
            }
            else if(rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'G';
                etherDeviceStats.coreDrops++;
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'C';
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'H';
                etherDeviceStats.txDrops++; //confirm-done (initially txDrops)
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }   
            else{
                nextState = 'F';
                etherDeviceStats.dmaDrops++;
                // etherDeviceStats.txRingBufferFull++;
            }  
            break;
        case 'G':
            if(!rxFifoFull && !rxRingFull && !txRingFull)
                nextState = 'A';
            else if(!rxFifoFull && !rxRingFull && txRingFull) {
                nextState = 'B';
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && txRingFull){
                nextState = 'D';
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && !rxRingFull && !txRingFull){
                nextState = 'E';
                etherDeviceStats.dmaDrops++; //confirm-done (initially dmaDrops++)
            }
            else if(rxFifoFull && !rxRingFull && txRingFull) {
                nextState = 'F';
                etherDeviceStats.dmaDrops++; //confirm-done (initially dmaDrops++)
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'C';
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && txRingFull) {
                nextState = 'H';
                etherDeviceStats.txDrops++;
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }   
            else{
                nextState = 'G';
                etherDeviceStats.coreDrops++;
                // etherDeviceStats.rxRingBufferFull++;
            }  
            break;
        case 'H':
            if(!rxFifoFull && !rxRingFull && !txRingFull)
                nextState = 'A';
            else if(!rxFifoFull && !rxRingFull && txRingFull) {
                nextState = 'B';
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && txRingFull){
                nextState = 'D';
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(rxFifoFull && !rxRingFull && !txRingFull){
                nextState = 'E';
                etherDeviceStats.dmaDrops++;
            }
            else if(rxFifoFull && !rxRingFull && txRingFull) {
                nextState = 'F';
                etherDeviceStats.dmaDrops++; //confirm-done (initially dmaDrops)
                // etherDeviceStats.txRingBufferFull++;
            }
            else if(!rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'C';
                // etherDeviceStats.rxRingBufferFull++;
            }
            else if(rxFifoFull && rxRingFull && !txRingFull) {
                nextState = 'G';
                etherDeviceStats.coreDrops++; //confirm-done
                // etherDeviceStats.rxRingBufferFull++;
            }   
            else{
                nextState = 'H';
                etherDeviceStats.txDrops++;
                // etherDeviceStats.rxRingBufferFull++;
                // etherDeviceStats.txRingBufferFull++;
            }  
            break;
    }
    currState = nextState;
}

char currStateM2func = 'A', nextStateM2func;
void
IGbE::updateDropFSMM2func(int rxFifoFull, int pcieBusy, int txRingFull)
{
    // State Encoding: rxFifoFull,pcieBusy,txRingFull
    // 000 -> "A" || 001 -> "B" || 010 -> "C" || 011 = "D"
    // 100 -> "E" || 101 -> "F" || 110 -> "G" || 111 = "H"
    // rxFifoFull==1 && pcieBusy==1 -> DMA Drop (PCIe Drop)
    // rxFifoFull==1 && pcieBusy==0 -> Core Drop (Core Drop). (Because pcie is not busy, but rxFifo is full)
    // rxFifoFull==0 && pcieBusy==1 -> potential DMA Drop (PCIe Drop)

    switch(currStateM2func) {
        case 'A': {
           if (!rxFifoFull && !pcieBusy && txRingFull) { // 001
            nextStateM2func = 'B';
           }
           else if (!rxFifoFull && pcieBusy && !txRingFull) { //010
            // Potential DMA Drop
            nextStateM2func = 'C';
           }
           else if (!rxFifoFull && pcieBusy && txRingFull) { //011
            // Potential DMA Drop
            nextStateM2func = 'D';           
           } else if (rxFifoFull && !pcieBusy && !txRingFull) { //100
            // Core Drop
            nextStateM2func = 'E';
            etherDeviceStats.coreDrops++; 
           } else if (rxFifoFull && !pcieBusy && txRingFull) { //101
            // Core Drop
            nextStateM2func = 'F';
            etherDeviceStats.coreDrops++; 
           } else if (rxFifoFull && pcieBusy && !txRingFull) { //110
            // DMA Drop
            nextStateM2func = 'G';
            etherDeviceStats.dmaDrops++; 
           } else if (rxFifoFull && pcieBusy && txRingFull) { //111
            // TX Drop
            nextStateM2func = 'H';
            etherDeviceStats.txDrops++; 
           } else {
            nextStateM2func = 'A';
           }
           break; 
        }
        case 'B': {
            if (!rxFifoFull && !pcieBusy && !txRingFull) { // 000
               nextStateM2func = 'A';
            } else if (!rxFifoFull && pcieBusy && !txRingFull) { // 010
                nextStateM2func = 'C';
            } else if (!rxFifoFull && pcieBusy && txRingFull) { // 011
                nextStateM2func = 'D';
            } else if (rxFifoFull && !pcieBusy && !txRingFull) { // 100
                nextStateM2func = 'E';
                etherDeviceStats.coreDrops++;
            } else if (rxFifoFull && !pcieBusy && txRingFull) { // 101
                nextStateM2func = 'F';
                etherDeviceStats.coreDrops++;
            } else if (rxFifoFull && pcieBusy && !txRingFull) { // 110
                nextStateM2func = 'G';
                etherDeviceStats.dmaDrops++;
            } else if (rxFifoFull && pcieBusy && txRingFull) { // 111
                nextStateM2func = 'H';
                etherDeviceStats.txDrops++;
            } else {
                nextStateM2func = 'B';
            }
            break;
        }
        case 'C': { //potential DMA Drop
            if (!rxFifoFull && !pcieBusy && !txRingFull) { // 000
                nextStateM2func = 'A';
            } else if (!rxFifoFull && !pcieBusy && txRingFull) { // 001
                nextStateM2func = 'B';
            } else if (!rxFifoFull && pcieBusy && txRingFull) { // 011
                nextStateM2func = 'D';
            } else if (rxFifoFull && !pcieBusy && !txRingFull) { // 100
                // As current state is C, it is potential DMA Drop
                nextStateM2func = 'E';
                etherDeviceStats.dmaDrops++;
            } else if (rxFifoFull && !pcieBusy && txRingFull) { // 101
                // As current state is C, it is potential DMA Drop
                nextStateM2func = 'F';
                etherDeviceStats.dmaDrops++;
            } else if (rxFifoFull && pcieBusy && !txRingFull) { // 110
                nextStateM2func = 'G';
                etherDeviceStats.dmaDrops++;
            } else if (rxFifoFull && pcieBusy && txRingFull) { // 111
                nextStateM2func = 'H';
                etherDeviceStats.dmaDrops++;
            } else {
                nextStateM2func = 'C';
            }
            break;
        }
        case 'D': { //potential DMA Drop
            if (!rxFifoFull && !pcieBusy && !txRingFull) { // 000
                nextStateM2func = 'A';
            } else if (!rxFifoFull && !pcieBusy && txRingFull) { //001
                nextStateM2func = 'B';
            } else if (!rxFifoFull && pcieBusy && !txRingFull) { //010
                nextStateM2func = 'C';
            } else if (rxFifoFull && !pcieBusy && !txRingFull) { // 100
                // As current state is D, it is potential DMA Drop
                nextStateM2func = 'E';
                etherDeviceStats.dmaDrops++;
            } else if (rxFifoFull && !pcieBusy && txRingFull) { // 101
                // As current state is D, it is potential DMA Drop
                nextStateM2func = 'F';
                etherDeviceStats.dmaDrops++;
            } else if (rxFifoFull && pcieBusy && !txRingFull) { // 110
                nextStateM2func = 'G';
                etherDeviceStats.dmaDrops++;
            } else if (rxFifoFull && pcieBusy && txRingFull) { // 111
                nextStateM2func = 'H';
                etherDeviceStats.txDrops++;
            } else {
                nextStateM2func = 'D';
            }
            break;
        }
        case 'E': {
            if (!rxFifoFull && !pcieBusy && !txRingFull) { // 000
                nextStateM2func = 'A';
            } else if (!rxFifoFull && !pcieBusy && txRingFull) { //001
                nextStateM2func = 'B';
            } else if (!rxFifoFull && pcieBusy && !txRingFull) { // 010
                nextStateM2func = 'C';
            } else if (!rxFifoFull && pcieBusy && txRingFull) { // 011
                nextStateM2func = 'D';
            } else if (rxFifoFull && !pcieBusy && txRingFull) { // 101
                nextStateM2func = 'F';
                etherDeviceStats.coreDrops++;
            } else if (rxFifoFull && pcieBusy && !txRingFull) { // 110
                nextStateM2func = 'G';
                etherDeviceStats.dmaDrops++;
            } else if (rxFifoFull && pcieBusy && txRingFull) { // 111
                nextStateM2func = 'H';
                etherDeviceStats.txDrops++;
            } else {
                nextStateM2func = 'E';
                etherDeviceStats.coreDrops++;
            }
            break;
        }
        case 'F': {
            if (!rxFifoFull && !pcieBusy && !txRingFull) { // 000
                nextStateM2func = 'A';
            } else if (!rxFifoFull && !pcieBusy && txRingFull) { //001
                nextStateM2func = 'B';
            } else if (!rxFifoFull && pcieBusy && !txRingFull) { // 010
                nextStateM2func = 'C';
            } else if (!rxFifoFull && pcieBusy && txRingFull) { // 011
                nextStateM2func = 'D';
            } else if (rxFifoFull && !pcieBusy && !txRingFull) { // 100
                nextStateM2func = 'E';
                etherDeviceStats.coreDrops++;
            } else if (rxFifoFull && pcieBusy && !txRingFull) { // 110
                nextStateM2func = 'G';
                etherDeviceStats.dmaDrops++;
            } else if (rxFifoFull && pcieBusy && txRingFull) { // 111
                nextStateM2func = 'H';
                etherDeviceStats.txDrops++;
            } else { //101
                nextStateM2func = 'F';
                etherDeviceStats.coreDrops++;
            }
            break;
        }
        case 'G': {
            if (!rxFifoFull && !pcieBusy && !txRingFull) { // 000
               nextStateM2func = 'A';
            } else if (!rxFifoFull && !pcieBusy && txRingFull) { //001
                nextStateM2func = 'B';
            } else if (!rxFifoFull && pcieBusy && !txRingFull) { // 010
                nextStateM2func = 'C';
            } else if (!rxFifoFull && pcieBusy && txRingFull) { // 011
                nextStateM2func = 'D';
            } else if (rxFifoFull && !pcieBusy && !txRingFull) { // 100
                nextStateM2func = 'E';
                etherDeviceStats.coreDrops++;
            } else if (rxFifoFull && !pcieBusy && txRingFull) { // 101
                nextStateM2func = 'F';
                etherDeviceStats.coreDrops++;
            } else if (rxFifoFull && pcieBusy && txRingFull) { // 111
                nextStateM2func = 'H';
                etherDeviceStats.txDrops++;
            } else {
                nextStateM2func = 'G';
                etherDeviceStats.dmaDrops++;
            }
            break;
        }
        case 'H': {
            if (!rxFifoFull && !pcieBusy && !txRingFull) { // 000
                nextStateM2func = 'A';
            } else if (!rxFifoFull && !pcieBusy && txRingFull) { //001
                nextStateM2func = 'B';
            } else if (!rxFifoFull && pcieBusy && !txRingFull) { // 010
                nextStateM2func = 'C';
            } else if (!rxFifoFull && pcieBusy && txRingFull) { // 011
                nextStateM2func = 'D';
            } else if (rxFifoFull && !pcieBusy && !txRingFull) { // 100
                // Core Drop
                nextStateM2func = 'E';
                etherDeviceStats.coreDrops++;
            } else if (rxFifoFull && !pcieBusy && txRingFull) { // 101
                // Core Drop
                nextStateM2func = 'F';
                etherDeviceStats.coreDrops++;
            } else if (rxFifoFull && pcieBusy && !txRingFull) { // 110
                nextStateM2func = 'G';
                etherDeviceStats.dmaDrops++;
            } else {
                nextStateM2func = 'H';
                etherDeviceStats.txDrops++;
            }
            break;
        }
    }
    currStateM2func = nextStateM2func;
}
#ifndef USE_ENSO
bool
IGbE::ethRxPkt(EthPacketPtr pkt)
{
    if (etherDeviceStats.rxBytes.value() == 0) {
        printf("IGbE::ethRxPkt:: Received first packet. startLoadGen: %d. -> Change to startLoadGen as true\n", startLoadGen);
        startLoadGen = true;
    }
    etherDeviceStats.rxBytes += pkt->length;
    etherDeviceStats.rxPackets++;
    
    updateRxPacketReceiveStat(curTick());

    DPRINTF(Ethernet, "RxFIFO: Receiving packet from wire\n");
    DPRINTF(EthernetDpdk, "RxFIFO: Receiving packet from wire\n");

    if (pkt->rxPortTick == 0) {
        pkt->rxPortTick = curTick();
    }

    // set rxFifoFull, txRingFull, rxRingFull
    // int rxRingFull = (rxDescCache.descLeft() == 0 ? 1 : 0);
    // int txRingFull = ((!txDescCache.packetWaiting() && txDescCache.descLeft() == 0) ? 1 : 0);
    // int rxFifoFull = (rxFifo.full() ? 1 : 0);
    // int rxFifoFull = (!rxFifo.push(pkt) ? 1 : 0);
    // updateDropFSM(rxFifoFull, rxRingFull, txRingFull);


    if (!regs.rctl.en()) {
        etherDeviceStats.rxdisabledDrops++;
        DPRINTF(Ethernet, "RxFIFO: RX not enabled, dropping\n");
        DPRINTF(EthernetDpdk, "RxFIFO: RX not enabled, dropping\n");
        return true;
    }

    // restart the state machines if they are stopped
    rxTick = true && drainState() != DrainState::Draining;
    if ((rxTick || txTick) && !tickEvent.scheduled()) {
        DPRINTF(EthernetSM,
                "RXS: received packet into fifo, starting ticking\n");
        DPRINTF(EthernetDpdk,
                "RXS: received packet into fifo, starting ticking\n");
        restartClock();
    }
    
    // int rxRingFull = ((rxDescCache.descLeft() == 0) ? 1 : 0); // RX Path: CPU Produces Descriptors and NIC Consumes/Uses
    // int txRingFull = ((!txDescCache.packetWaiting() && txDescCache.descLeft() == 1024) ? 1 : 0); // TX Path: CPU Produces Packets and NIC Consumes/Uses
    
    // check rxDescCacheArray and txDescCacheArray
    int rxRingFull = 0;
    int txRingFull = 0;
    if (commType == CommunicationType::RING) {
        bool rxDescCacheFull = true;
        bool txDescCacheFull = true;
        for (int i = 0; i < numQueues; i++) {
            if (rxDescCacheArray[i]->descLeft() > 0) {
                rxDescCacheFull = false;
            }
            if (txDescCacheArray[i]->packetWaitingGlobal() || txDescCacheArray[i]->descLeft() < 1024) {
                txDescCacheFull = false;
            }
        }
        if (rxDescCacheFull) {
            rxRingFull = 1; // Have to check - Maybe, we can only check target RX queue (results of RSS)
        }
        if (txDescCacheFull) {
            txRingFull = 1;
        }
    }

    // Set PCIe busy 
    int pcieBusy = 0;
    if (commType == CommunicationType::M2FUNC) {
        double pcieUtil = getPCIeUtilization();
        if (pcieUtil > 0.8) {
            DPRINTF(EthernetDpdk, "ethRXPkt: PCIe is busy. pcieUtil: %f\n", pcieUtil);
            pcieBusy = 1;
        } else {
            DPRINTF(EthernetDpdk, "ethRXPkt: PCIe is not busy. pcieUtil: %f\n", pcieUtil);
        }
    }

    // Set rxFifoFull and txFifoFull
    int rxFifoFull = 0;
    int txFifoFull = 0;
    if (!rxFifo.push(pkt)) {
        rxFifoFull = 1;
        etherDeviceStats.rxFifoFullCount++;
        if (commType == CommunicationType::RING) {
            updateDropFSM(rxFifoFull, rxRingFull, txRingFull, txFifoFull);
        } else if (commType == CommunicationType::M2FUNC) {
            //M2FUNC, now don't care txRingFull. It will always 0
            updateDropFSMM2func(rxFifoFull, pcieBusy, 0);
        } else {
            panic("Communication type not supported - ethRxPkt\n");
        }
        DPRINTF(Ethernet, "RxFIFO: Packet won't fit in fifo... dropped\n");
        DPRINTF(EthernetDpdk, "RxFIFO: Packet won't fit in fifo... dropped\n");
        postInterrupt(IT_RXO, true);
        return false;
    }

    if (commType == CommunicationType::RING) {
        updateDropFSM(rxFifoFull, rxRingFull, txRingFull, txFifoFull);
    } else if (commType == CommunicationType::M2FUNC) {
        //M2FUNC, now don't care txRingFull. It will always 0
        updateDropFSMM2func(rxFifoFull, pcieBusy, 0);
    } else {
        panic("Communication type not supported - ethRxPkt\n");
    }

    if (pkt->rxFifoTick == 0) {
        pkt->rxFifoTick = curTick();
    }

    return true;
}
#else

char curStateEnso = 'A', nextStateEnso;
void
IGbE::updateDropFSMEnso(int rxFifoFull, int ensoPipeFull)
{
    // State Encoding: rxFifoFull,ensoPipeFull
    // 00 -> "A" || 10 -> "B" || x1 -> "C"
    // rxFifoFull==1 && ensoPipeFull==0 -> DMA Drop (PCIe Drop)
    // rxFifoFull==x && ensoPipeFull==1 -> Core Drop (Core Drop) (Becaus enso pipe is full)
    switch(curStateEnso) 
    {
        case 'A': 
        {
            if(rxFifoFull && !ensoPipeFull) // 10
            {
                nextStateEnso = 'B';
                etherDeviceStats.dmaDrops++;
            }
            else if(ensoPipeFull) // x1
            {
                nextStateEnso = 'C';
                etherDeviceStats.coreDrops++;
            }
            else
                nextStateEnso = 'A';
            
            break;
        }
        case 'B': 
        {
            if(rxFifoFull && !ensoPipeFull) // 10
            {
                nextStateEnso = 'B';
                etherDeviceStats.dmaDrops++;
            }
            else if(ensoPipeFull) // x1
            {
                nextStateEnso = 'C';
                etherDeviceStats.coreDrops++;
            }
            else
                nextStateEnso = 'A';
            
            break;
        }
        case 'C': 
        {
            if(rxFifoFull && !ensoPipeFull) // 10
            {
                nextStateEnso = 'B';
                etherDeviceStats.dmaDrops++;
            }
            else if(ensoPipeFull) // x1
            {
                nextStateEnso = 'C';
                etherDeviceStats.coreDrops++;
            }
            else
                nextStateEnso = 'A';
            
            break;
        }
    }
    curStateEnso = nextStateEnso;
}

bool IGbE::ethRxPkt(EthPacketPtr pkt)
{   
    etherDeviceStats.rxBytes += pkt->length;
    etherDeviceStats.rxPackets++;

    DPRINTF(EthernetENSO, "RxFIFO: Receiving packet from wire\n");

    if (!regs.rctl.en()) {
        etherDeviceStats.rxdisabledDrops++;
        DPRINTF(EthernetENSO, "RxFIFO: RX not enabled, dropping\n");
        return true;
    }

    // restart the state machines if they are stopped
    rxTick = true && drainState() != DrainState::Draining;
    if ((rxTick || txTick) && !tickEvent.scheduled()) {
        DPRINTF(EthernetENSO,
                "RXS: received packet into fifo, starting ticking\n");
        restartClock();
    }

    int rxFifoFull = 0;
    int ensoPipeFull = rxEnsoPipeManager.isFull();

    if(!rxEnsoPipeManager.dataPush(pkt))
    {
        rxFifoFull = 1;
        etherDeviceStats.rxFifoFullCount++;
        // NIC RX FIFO is full
        updateDropFSMEnso(rxFifoFull, ensoPipeFull);
        DPRINTF(EthernetENSO, "RXS: rx fifo is full!!\n");
        return false;
    }

    updateDropFSMEnso(rxFifoFull, ensoPipeFull);
    return true;
}
#endif

double 
IGbE::getPCIeUtilization()
{
    PortID reqID = 18; 
    PortID respID = 2;
    double pcieReqUtil = getPCIeReqLayerUtil(reqID);
    double pcieRespUtil = getPCIeRespLayerUtil(respID);
    double pcieUtil = (pcieReqUtil + pcieRespUtil) / 2;

    return pcieUtil;
}


// void
bool
IGbE::rxStateMachine(int queueID)
{
    bool rxTickQueue = rxTick; //rxTickQueue is used to keep track of the rxTick status for the queueID
    if (!regs.rctl.en()) {
        rxTickQueue = false;
        DPRINTF(EthernetSM, "RXS[%d]: RX disabled, stopping ticking\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: RX disabled, stopping ticking\n", queueID);
        return rxTickQueue;
    }
    // If the packet is done check for interrupts/descriptors/etc
    int readyDMAEngineIdx = rxDescCacheArray[queueID]->hasReadyEthPacket();
    if (readyDMAEngineIdx != -1) {
        
        rxDescCacheArray[queueID]->clearDoneEthPacket(readyDMAEngineIdx);

        DPRINTF(EthernetSM, "RXS[%d]: Packet completed DMA to memory\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: Packet completed DMA to memory\n", queueID);
        int descLeft = rxDescCacheArray[queueID]->descLeft();
        DPRINTF(EthernetSM, "RXS[%d]: descLeft: %d rdmts: %d rdlen: %d\n", queueID,
                descLeft, regs.rctl.rdmts(), regs.rdlen_array[queueID]()); //regs.rdlen());
        DPRINTF(EthernetDpdk, "RXS[%d]: descLeft: %d rdmts: %d rdlen: %d\n", queueID,
                descLeft, regs.rctl.rdmts(), regs.rdlen_array[queueID]()); //regs.rdlen());

        // rdmts 2->1/8, 1->1/4, 0->1/2
        int descSize = 16;
        int numDescRX = (regs.rdlen_array[queueID]()) / descSize;
        int ratio = (1ULL << (regs.rctl.rdmts() + 1));
        if (descLeft * ratio <= numDescRX) {
            DPRINTF(EthernetSM, "RXS[%d]: Interrupting (RXDMT) "
                    "because of descriptors left descLeft: %d, ratio: %d, rdlen_array: %d, numDescRX: %d\n", queueID,
                    descLeft, ratio, regs.rdlen_array[queueID](), numDescRX);
            DPRINTF(EthernetDpdk, "RXS[%d]: Interrupting (RXDMT) "
                    "because of descriptors left\n", queueID);
            rxDescCacheArray[queueID]->writebackGlobal(0);
         }

        if (descLeft < 32)
        {
            DPRINTF(EthernetSM, "RXS[%d]: writebackGlobal(0) because descLeft < 32\n", queueID);
            rxDescCacheArray[queueID]->writebackGlobal(0);
        }

        if (rxFifo.empty()) {
            DPRINTF(EthernetSM, "RXS[%d]: writebackGlobal(0) because rxFifo is empty\n", queueID);
            rxDescCacheArray[queueID]->writebackGlobal(0);
        }

        if (descLeft == 0) { 
            etherDeviceStats.rxRingBufferFull++; //TODO - jm : make this as array 
            DPRINTF(EthernetSM, "RXS[%d]: No descriptors left in ring, forcing"
                    " writeback and stopping ticking\n", queueID);
            DPRINTF(EthernetDpdk, "RXS[%d]: No descriptors left in ring, forcing"
                    " writeback and stopping ticking\n", queueID);
            rxDescCacheArray[queueID]->writebackGlobal(0);
            rxTickQueue = false;
        }

        // only support descriptor granulaties
        assert(regs.rxdctl_array[queueID].gran());

        if (regs.rxdctl_array[queueID].wthresh() <= rxDescCacheArray[queueID]->descUsed()) {
            DPRINTF(EthernetSM,
                    "RXS[%d]: Writing back because WTHRESH (%d) <= descUsed (%d)\n", queueID, 
                    regs.rxdctl_array[queueID].wthresh(), rxDescCacheArray[queueID]->descUsed());
            DPRINTF(EthernetDpdk,
                    "RXS[%d]: Writing back because WTHRESH (%d) <= descUsed (%d)\n", queueID,
                    regs.rxdctl_array[queueID].wthresh(), rxDescCacheArray[queueID]->descUsed());
            
            if (regs.rxdctl_array[queueID].wthresh() < (cacheBlockSize()>>4))
                rxDescCacheArray[queueID]->writebackGlobal(regs.rxdctl_array[queueID].wthresh()-1);
            else
                rxDescCacheArray[queueID]->writebackGlobal((cacheBlockSize()-1)>>4);
           
        }

        if ((rxDescCacheArray[queueID]->descUnused() < regs.rxdctl_array[queueID].pthresh()) &&
            ((rxDescCacheArray[queueID]->descLeft() - rxDescCacheArray[queueID]->descUnused()) >
             regs.rxdctl_array[queueID].hthresh())) {
            DPRINTF(EthernetSM, "RXS[%d]: Fetching descriptors because "
                    "descUnused < PTHRESH\n", queueID);
            DPRINTF(EthernetDpdk, "RXS[%d]: Fetching descriptors because "
                    "descUnused < PTHRESH\n", queueID);
            rxDescCacheArray[queueID]->fetchDescriptorsGlobal();
        }
        
        if (rxDescCacheArray[queueID]->descUnused() == 0) {
            rxDescCacheArray[queueID]->fetchDescriptorsGlobal();
            etherDeviceStats.rxDescCacheFullCount++;
            DPRINTF(EthernetSM, "RXS[%d]: No descriptors available in cache, "
                    "fetching descriptors and stopping ticking\n", queueID);
            DPRINTF(EthernetDpdk, "RXS[%d]: No descriptors available in cache, "
                    "fetching descriptors and stopping ticking\n", queueID);
            rxTickQueue = false;
        }
        return rxTickQueue;
    }

    if (!rxDescCacheArray[queueID]->hasFreeDMAEngine()) {
        DPRINTF(EthernetSM,
                "RXS[%d]: stopping ticking until packet DMA completes. No free DMA engines\n", queueID);
        DPRINTF(EthernetDpdk,
                "RXS[%d]: stopping ticking until packet DMA completes. No free DMA engines\n", queueID);
        
        rxTickQueue = false;
        if (rxPacketArray[queueID] != nullptr) {
            if (rxPacketArray[queueID]->rxFifoNotEmptyDmaBusyChecked == false) {
                etherDeviceStats.rxFifoNotEmptyDmaBusy++;
                rxPacketArray[queueID]->rxFifoNotEmptyDmaBusyChecked = true;
            }
        }
        return rxTickQueue;
    }

    if (!rxDescCacheArray[queueID]->descUnused()) {
        rxDescCacheArray[queueID]->fetchDescriptorsGlobal();
        etherDeviceStats.rxDescCacheFullCount++;
        DPRINTF(EthernetSM, "RXS[%d]: No descriptors available in cache, "
                "stopping ticking\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: No descriptors available in cache, "
                "stopping ticking\n", queueID);
        // rxTick = false;
        rxTickQueue = false;
        DPRINTF(EthernetSM, "RXS[%d]: No descriptors available, fetching\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: No descriptors available, fetching\n", queueID);
        return rxTickQueue;
    }

    // Access rxPacketArray of each queueID
    EthPacketPtr pkt;
    pkt = rxPacketArray[queueID];

    if (pkt == nullptr) {
        DPRINTF(EthernetSM, "RXS[%d]: No packet available at rxPacketArray\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: No packet available at rxPacketArray\n", queueID);
        rxTickQueue = false;
        return rxTickQueue;
    }

    bool success = rxDescCacheArray[queueID]->writePacketGlobal(pkt);
    if (success) {
        rxPacketArray[queueID] = nullptr;
        updateRxRingBufferDMAStartStat(curTick());
    }
    rxTickQueue = true; // Because new DMA engine can be started
    return rxTickQueue;
}

void
IGbE::txWire()
{
    txFifoTick = false;

    if (txFifo.empty())
        return;


    if (etherInt->sendPacket(txFifo.front())) {
        if (debug::EthernetSM) {
            IpPtr ip(txFifo.front());
            if (ip)
                DPRINTF(EthernetSM, "Transmitting Ip packet with Id=%d\n",
                        ip->id());
            else
                DPRINTF(EthernetSM, "Transmitting Non-Ip packet\n");
        }
        DPRINTF(EthernetSM,
                "TxFIFO: Successful transmit, bytes available in fifo: %d\n",
                txFifo.avail());
        DPRINTF(EthernetDpdk,
                "TxFIFO: Successful transmit, bytes available in fifo: %d\n",
                txFifo.avail());
        DPRINTF(EthernetENSO,
                "TxFIFO: Successful transmit, bytes available in fifo: %d\n",
                txFifo.avail());

        etherDeviceStats.txBytes += txFifo.front()->length;
        etherDeviceStats.txPackets++;

        txFifo.pop();
    }
}

#ifndef USE_ENSO
void
IGbE::tick()
{
    DPRINTF(EthernetSM, "IGbE: -------------- Cycle --------------\n");
    DPRINTF(EthernetDpdk, "IGbE: -------------- Cycle --------------\n");
    inTick = true;

    if (rxTick) {
        // rxStateMachine();
        bool rxTickQueue = rxTick;
        for (int i = 0; i < numQueues; i++) {
            if (commType == CommunicationType::RING) {
                rxTickQueue |= rxStateMachine(i);
            } else if (commType == CommunicationType::M2FUNC) {
                rxTickQueue |= rxM2funcContextArray[i]->rxM2funcStateMachine();
            } else {
                panic("Communication type not supported - tick\n");
            }
        }
        // Move pkt from rxFifo to rxPacketArray
        rxFifotoRxDesc();

        rxTick = rxTickQueue;
    } else {
        if (commType == CommunicationType::RING) {
            for (int i = 0; i < numQueues; i++) {
                if (rxDmaPacketArray[i] && rxPacketArray[i] != nullptr) {
                    if (rxPacketArray[i]->rxFifoNotEmptyDmaBusyChecked == false) {
                        rxPacketArray[i]->rxFifoNotEmptyDmaBusyChecked = true;
                        etherDeviceStats.rxFifoNotEmptyDmaBusy++;
                    }
                }
            }
        }
    }

    if (txTick) {
        // txStateMachine();
        //Move pkt from txPacketArray to txFifo
        txPackettoTxFifo();
        bool txTickQueue = txTick;
        for (int i = 0; i < numQueues; i++) {
            if (commType == CommunicationType::RING) {
                txTickQueue |= txStateMachine(i);
            } else if (commType == CommunicationType::M2FUNC) {
                txTickQueue |= txM2funcContextArray[i]->txM2funcStateMachine();
            } else {
                panic("Communication type not supported - tick\n");
            }
        }
        txTick = txTickQueue;
        if (successTxQueueSend) {
            // update the candidateTxQueue to the next queue (round-robin)
            candidateTxQueue = (candidateTxQueue + 1) % numQueues;
            successTxQueueSend = false;
        }
    }

    // If txWire returns and txFifoTick is still set, that means the data we
    // sent to the other end was already accepted and we can send another
    // frame right away. This is consistent with the previous behavior which
    // would send another frame if one was ready in ethTxDone. This version
    // avoids growing the stack with each frame sent which can cause stack
    // overflow.
    while (txFifoTick)
        txWire();

    if (rxTick || txTick || txFifoTick)
        schedule(tickEvent, curTick() + clockPeriod());

    inTick = false;
}

void
IGbE::ethTxDone()
{
    // restart the tx state machines if they are stopped
    // fifo to send another packet
    // tx sm to put more data into the fifo
    txFifoTick = true && drainState() != DrainState::Draining;
    // if (txDescCache.descLeft() != 0 && drainState() != DrainState::Draining)
    //     txTick = true;
    // check txDescCacheArray
    for (int i = 0; i < numQueues; i++) {
        if (commType == CommunicationType::RING) {
            if (txDescCacheArray[i]->descLeft() != 0 && drainState() != DrainState::Draining)
                txTick = true;
        } else if (commType == CommunicationType::M2FUNC) {
            // TODO - JM: have to check
            if (drainState() != DrainState::Draining)
                txTick = true;
        } else {
            panic("Communication type not supported - ethTxDone\n");
        }
    }

    if (!inTick)
        restartClock();
    DPRINTF(EthernetSM, "TxFIFO: Transmission complete\n");
}
#else
void
IGbE::tick()
{
    inTick = true;

    if (rxTick)
        rxEnsoStateMachine();
    
    if (txTick)
        txEnsoStateMachine();

    // If txWire returns and txFifoTick is still set, that means the data we
    // sent to the other end was already accepted and we can send another
    // frame right away. This is consistent with the previous behavior which
    // would send another frame if one was ready in ethTxDone. This version
    // avoids growing the stack with each frame sent which can cause stack
    // overflow.
    while (txFifoTick)
        txWire();

    if (rxTick || txTick || txFifoTick)
        schedule(tickEvent, curTick() + clockPeriod());

    inTick = false;
}

void
IGbE::ethTxDone()
{
    // restart the tx state machines if they are stopped
    // fifo to send another packet
    // tx sm to put more data into the fifo
    txFifoTick = true && drainState() != DrainState::Draining;
    if (drainState() != DrainState::Draining)
        txTick = true;

    if (!inTick)
        restartClock();
    DPRINTF(EthernetENSO, "TxFIFO: Transmission complete\n");
}
#endif

void
IGbE::serialize(CheckpointOut &cp) const
{
    PciDevice::serialize(cp);

    regs.serialize(cp);
    SERIALIZE_SCALAR(eeOpBits);
    SERIALIZE_SCALAR(eeAddrBits);
    SERIALIZE_SCALAR(eeDataBits);
    SERIALIZE_SCALAR(eeOpcode);
    SERIALIZE_SCALAR(eeAddr);
    SERIALIZE_SCALAR(lastInterrupt);
    SERIALIZE_ARRAY(flash,igbreg::EEPROM_SIZE);

    rxFifo.serialize("rxfifo", cp);
    txFifo.serialize("txfifo", cp);

    SERIALIZE_SCALAR(numQueues);

    for (int i = 0; i < numQueues; i++) {
        bool txPktExists = txPacketArray[i] != nullptr;
        if (txPktExists) {
            paramOut(cp, csprintf("txpacketexist%d", i), 1);
            txPacketArray[i]->serialize(csprintf("txpacket%d", i), cp);
        } else {
            paramOut(cp, csprintf("txpacketexist%d", i), 0);
        }
    }
    //Serialize rxPacketArray
    for (int i = 0; i < numQueues; i++) {
        bool rxPktExists = rxPacketArray[i] != nullptr;
        if (rxPktExists) {
            paramOut(cp, csprintf("rxpacketexist%d", i), 1);
            rxPacketArray[i]->serialize(csprintf("rxpacket%d", i), cp);
        } else {
            paramOut(cp, csprintf("rxpacketexist%d", i), 0);
        }
    }

    // Tick rdtr_time = 0, radv_time = 0, tidv_time = 0, tadv_time = 0,
    Tick inter_time = 0;

    if (interEvent.scheduled())
        inter_time = interEvent.when();
    SERIALIZE_SCALAR(inter_time);

    // SERIALIZE_SCALAR(pktOffset);
    // Serialize pktOffsetArray
    for (int i = 0; i < numQueues; i++) {
        paramOut(cp, csprintf("pktOffset%d", i), pktOffsetArray[i]);
    }

    
    #ifdef USE_ENSO

    for (int i = 0; i < MAX_NB_MANAGER; i++) {
        paramOut(cp, csprintf("pipeHeadUpdated%d", i), pipeHeadUpdated[i]);
    }
    EnsoRxFifo.serialize("EnsoRxFifo", cp);
    #endif

    /*======= end of the system.nics serializeSection ==========*/

    // txDescCache.serializeSection(cp, "TxDescCache");
    // rxDescCache.serializeSection(cp, "RxDescCache");
    //Serialize rxDescCacheArray and txDescCacheArray
    for (int i = 0; i < numQueues; i++) {
        if (commType == CommunicationType::RING) {
            txDescCacheArray[i]->serializeSection(cp, csprintf("TxDescCache%d", i));
            rxDescCacheArray[i]->serializeSection(cp, csprintf("RxDescCache%d", i));
        } else if (commType == CommunicationType::M2FUNC) {
            txM2funcContextArray[i]->serializeSection(cp, csprintf("TxM2funcContext%d", i));
            rxM2funcContextArray[i]->serializeSection(cp, csprintf("RxM2funcContext%d", i));
        } else {
            panic("Communication type not supported - serialize\n");
        }
    }

    #ifdef USE_ENSO
    rxEnsoPipeManager.serializeSection(cp, "RxEnsoPipeManager");
    rxNotifBufManager.serializeSection(cp, "RxNotifBufManager");
    txNotifBufManager.serializeSection(cp, "TxNotifBufManager");
    #endif
}

void
IGbE::unserialize(CheckpointIn &cp)
{
    PciDevice::unserialize(cp);

    regs.unserialize(cp);
    UNSERIALIZE_SCALAR(eeOpBits);
    UNSERIALIZE_SCALAR(eeAddrBits);
    UNSERIALIZE_SCALAR(eeDataBits);
    UNSERIALIZE_SCALAR(eeOpcode);
    UNSERIALIZE_SCALAR(eeAddr);
    UNSERIALIZE_SCALAR(lastInterrupt);
    UNSERIALIZE_ARRAY(flash,igbreg::EEPROM_SIZE);

    rxFifo.unserialize("rxfifo", cp);
    txFifo.unserialize("txfifo", cp);

    UNSERIALIZE_SCALAR(numQueues);

    // bool txPktExists;
    // UNSERIALIZE_SCALAR(txPktExists);
    // if (txPktExists) {
    //     txPacket = std::make_shared<EthPacketData>(16384);
    //     txPacket->unserialize("txpacket", cp);
    // }
    //Unserialize txPacketArray
    for(int i = 0; i < numQueues; i++) {
        int txPktExists;
        paramIn(cp, csprintf("txpacketexist%d", i), txPktExists);
        if (txPktExists == 1) {
            txPacketArray[i] = std::make_shared<EthPacketData>(16384);
            txPacketArray[i]->unserialize(csprintf("txpacket%d", i), cp);
        } else {
            txPacketArray[i] = nullptr;
        }
    }

    //Unserialize rxPacketArray
    for(int i = 0; i < numQueues; i++) {
        int rxPktExists;
        paramIn(cp, csprintf("rxpacketexist%d", i), rxPktExists);
        if (rxPktExists == 1) {
            rxPacketArray[i] = std::make_shared<EthPacketData>(16384);
            rxPacketArray[i]->unserialize(csprintf("rxpacket%d", i), cp);
        } else {
            rxPacketArray[i] = nullptr;
        }
    }
        

    rxTick = true;
    txTick = true;
    txFifoTick = true;

    // Tick rdtr_time, radv_time, tidv_time, tadv_time, inter_time;
    Tick inter_time;
    // UNSERIALIZE_SCALAR(rdtr_time);
    // UNSERIALIZE_SCALAR(radv_time);
    // UNSERIALIZE_SCALAR(tidv_time);
    // UNSERIALIZE_SCALAR(tadv_time);
    UNSERIALIZE_SCALAR(inter_time);

    // if (rdtr_time)
    //     schedule(rdtrEvent, rdtr_time);

    // if (radv_time)
    //     schedule(radvEvent, radv_time);

    // if (tidv_time)
    //     schedule(tidvEvent, tidv_time);

    // if (tadv_time)
    //     schedule(tadvEvent, tadv_time);

    if (inter_time)
        schedule(interEvent, inter_time);

    // UNSERIALIZE_SCALAR(pktOffset);
    // Unserialize pktOffsetArray
    for (int i = 0; i < numQueues; i++) {
        paramIn(cp, csprintf("pktOffset%d", i), pktOffsetArray[i]);
    }

    #ifdef USE_ENSO

    for (int i = 0; i < MAX_NB_MANAGER; i++) {
        paramIn(cp, csprintf("pipeHeadUpdated%d", i), pipeHeadUpdated[i]);
    }
    EnsoRxFifo.unserialize("EnsoRxFifo", cp);
    
    #endif

    // txDescCache.unserializeSection(cp, "TxDescCache");
    // rxDescCache.unserializeSection(cp, "RxDescCache");
    //Unserialize rxDescCacheArray and txDescCacheArray
    for (int i = 0; i < numQueues; i++) {
        if (commType == CommunicationType::RING) {
            txDescCacheArray[i]->unserializeSection(cp, csprintf("TxDescCache%d", i));
            rxDescCacheArray[i]->unserializeSection(cp, csprintf("RxDescCache%d", i));
        } else if (commType == CommunicationType::M2FUNC) {
            txM2funcContextArray[i]->unserializeSection(cp, csprintf("TxM2funcContext%d", i));
            rxM2funcContextArray[i]->unserializeSection(cp, csprintf("RxM2funcContext%d", i));
        }
    }

    #ifdef USE_ENSO
    rxEnsoPipeManager.unserializeSection(cp, "RxEnsoPipeManager");
    rxNotifBufManager.unserializeSection(cp, "RxNotifBufManager");
    txNotifBufManager.unserializeSection(cp, "TxNotifBufManager");

    #endif
}

void M2funcPort::recvFunctional(PacketPtr pkt)
{
    recvAtomic(pkt); // Just throw away the latency returned
}

bool M2funcPort::recvTimingReq(PacketPtr pkt)
{
    assert(pkt->isRequest());

    int bar;
    Addr daddr;

    if (!(dev->getBARDTA(pkt->getAddr(), bar, daddr)))
        panic("Invalid m2func access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    int queueID = -1;
    if (isRegisterAddress<E1000_RXDTA>(daddr, queueID, numQueues)) {
        assert(queueID < numQueues);
        assert(queueID >= 0);
        if (dev->rxM2funcContextArray[queueID]->isDTAEnabled()) {
            bool success = dev->rxM2funcContextArray[queueID]->pushCXLReqBuf(pkt);
            dev->enableSmRx();
            return success;
        } else {
            panic("DTA is not enabled for RX queue %d\n", queueID);
        }
    } else if (isRegisterAddress<E1000_TXDTA>(daddr, queueID, numQueues)) {
        assert(queueID < numQueues);
        assert(queueID >= 0);
        if (dev->txM2funcContextArray[queueID]->isDTAEnabled()) {
            if (pkt->isRead()) {
                dev->txM2funcContextArray[queueID]->readBitmask(pkt);
                pkt->makeResponse();

                Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
                pkt->headerDelay = pkt->payloadDelay = 0;
                const Tick delay = receive_delay + cxlMemDelay; 
                
                assert(pkt->isResponse());
                schedTimingResp(pkt, curTick() + delay);
                return true;
            } else if (pkt->isWrite()) {
                dev->txM2funcContextArray[queueID]->writeM2funcPacket(pkt);
                pkt->makeResponse();

                dev->enableSmTx();
                dev->checkDrain();

                Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
                pkt->headerDelay = pkt->payloadDelay = 0;
                const Tick delay = receive_delay + cxlMemDelay; 
                
                assert(pkt->isResponse());
                schedTimingResp(pkt, curTick() + delay);
                return true;
            }
        } else {
            panic("DTA is not enabled for TX queue %d\n", queueID);
        }
    } else {
        panic("Invalid PCI memory access to unmapped memory.\n");
    }

    return false;
}

Tick M2funcPort::recvAtomic(PacketPtr pkt)
{
    assert(pkt->isRequest());

    int bar;
    Addr daddr;

    if (!(dev->getBARDTA(pkt->getAddr(), bar, daddr)))
        panic("Invalid m2func access to unmapped memory.\n");

    // Only Memory register BAR is allowed
    assert(bar == 0);

    int queueID = -1;
    if (isRegisterAddress<E1000_RXDTA>(daddr, queueID, numQueues)) {
        assert(queueID < numQueues);
        assert(queueID >= 0);
        if (dev->rxM2funcContextArray[queueID]->isDTAEnabled()) {
            // Not push to the buffer. Just fill the packet with response and then return it
            // So, with atomic access, response can be 0 only.            
            dev->rxM2funcContextArray[queueID]->readM2funcPacket(pkt);
            pkt->makeResponse();

            Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
            pkt->headerDelay = pkt->payloadDelay = 0;
            const Tick delay = receive_delay + cxlMemDelay;
            
            return delay;
        } else {
            panic("DTA is not enabled for RX queue %d\n", queueID);
        }
    } else if (isRegisterAddress<E1000_TXDTA>(daddr, queueID, numQueues)) {
        assert(queueID < numQueues);
        assert(queueID >= 0);
        if (dev->txM2funcContextArray[queueID]->isDTAEnabled()) {
            if (pkt->isRead()) {
                dev->txM2funcContextArray[queueID]->readBitmask(pkt);
                pkt->makeResponse();

                Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
                pkt->headerDelay = pkt->payloadDelay = 0;
                const Tick delay = receive_delay + cxlMemDelay; 

                return delay;
            } else if (pkt->isWrite()) {
                dev->txM2funcContextArray[queueID]->writeM2funcPacket(pkt);
                pkt->makeResponse();

                dev->enableSmTx();
                dev->checkDrain();

                Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
                pkt->headerDelay = pkt->payloadDelay = 0;
                const Tick delay = receive_delay + cxlMemDelay; 
                
                return delay;
            }
        } else {
            panic("DTA is not enabled for TX queue %d\n", queueID);
        }
    } else {
        panic("Invalid PCI memory access to unmapped memory.\n");
    }
}

#ifdef USE_ENSO


/******************** Enso Pipe Manager ***********************/
IGbE::EnsoPipeManager::EnsoPipeManager(IGbE *i, const std::string n, int s)
    : igbe(i), _name(n), size(s)
{}

IGbE::EnsoPipeManager::~EnsoPipeManager()
{}

void
IGbE::EnsoPipeManager::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(size);
}

void 
IGbE::EnsoPipeManager::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(size);
}

/******************** RX Enso Pipe Manger ********************/
// need to modify after connect MMIO
IGbE::RXEnsoPipeManager::RXEnsoPipeManager(IGbE *i, const std::string n, int s)
    : EnsoPipeManager(i, n, s)
{
    for(int i = 0; i < MAX_NB_MANAGER; i++)
    {
        pipeStates[i].notifBufId = 0; // Todo : multicore
        pipeStates[i].pipeStatus = false;
        // Todo : enso pipe number is less than max, phys_addr = NULL?
        pipeStates[i].physAddr = 0;
        //pipe_states[i].head = 0;
        pipeStates[i].tail = 0;
    }
    stateFull = 0;
    
}



bool
IGbE::RXEnsoPipeManager::dataPush(EthPacketPtr pkt)
{
    assert(pkt);
    assert(pkt->data);

    // // check DataFIFO is full
    // if(igbe->fifoSize() >= ENSO_PIPE_SIZE)
    //     return false;

    // new logic for smart ptr
    int pktLen = pkt->length;
    // algined for flit size (64B)
    pktLen = ((pktLen + igbe->flitSize - 1) / igbe->flitSize) * igbe->flitSize;
    EnsoRxPtr rxPkt = std::make_shared<EnsoRxData>(pktLen);
    assert(rxPkt);
    rxPkt->length = pktLen;
    rxPkt->flits = pktLen / igbe->flitSize;
    // flow_table logic, Ideal
    // all packet to pipe0
    // Todo : multicore
    rxPkt->meta.pktQueueId = 0;
    rxPkt->meta.isNotify = false;

    rxPkt->pipe = &pipeStates[rxPkt->meta.pktQueueId];

    // data copy
    memset(rxPkt->data, 0, pktLen);
    memcpy(rxPkt->data, pkt->data, pkt->length);

    // free previous pkt ptr
    pkt = nullptr;

    // Notify logic -> OnPktArrival, set is_notify variable
    onPktArrival(rxPkt);

    //DataFIFO.push_back(newPacket);
    // igbe->ensoDataPush(rxPkt);
    // return true;

    return igbe->ensoDataPush(rxPkt);
}



void
IGbE::RXEnsoPipeManager::setRxPipeState(int queueId)
{
    pipeStates[queueId].physAddr = ensoPipeBase(queueId) & ~0xFF ;
    pipeStates[queueId].notifBufId = ensoPipeBase(queueId) & 0xFF;

    DPRINTF(EthernetENSO, "PIPE STATE - SET physAddr[%u] = 0x%lx\n", queueId, pipeStates[queueId].physAddr);
    DPRINTF(EthernetENSO, "PIPE STATE - SET notifBufId[%u] = 0x%lx\n", queueId, pipeStates[queueId].notifBufId);
}

// RXEnsoPipeManager notify logic
void
IGbE::RXEnsoPipeManager::onPktArrival(EnsoRxPtr pkt)
{
    if(!pkt->pipe->pipeStatus)
    {
        pkt->meta.isNotify = true;
        pkt->pipe->pipeStatus = true;
    }
}

// notify logic, only executing when host update SWhead of EnsoPipe
void
IGbE::RXEnsoPipeManager::onRxUpdate(EnsoRxPtr pkt)
{
    uint32_t queueId = pkt->meta.pktQueueId;

    if(pkt->meta.isNotify)
        return;
    
    // check if host update register(MMIO)
    // OnRxUpdate
    if(ensoPipeHead(queueId) != ensoPipeTail(queueId))
        pkt->meta.isNotify = true;
    else
        pkt->pipe->pipeStatus = false;
}

PipeState*
IGbE::RXEnsoPipeManager::relinkDataFifoPipes(int queueId)
{
    return &pipeStates[queueId];
}


void
IGbE::RXEnsoPipeManager::serialize(CheckpointOut &cp) const
{
    EnsoPipeManager::serialize(cp);
    // Serialize pipeStates array
    SERIALIZE_SCALAR(stateFull);
    for (uint64_t i = 0; i < MAX_NB_MANAGER; i++) {
        paramOut(cp, csprintf("pipeStates[%d].physAddr", i), pipeStates[i].physAddr);
        paramOut(cp, csprintf("pipeStates[%d].pipeStatus", i), pipeStates[i].pipeStatus);
        paramOut(cp, csprintf("pipeStates[%d].notifBufId", i), pipeStates[i].notifBufId);
        paramOut(cp, csprintf("pipeStates[%d].tail", i), pipeStates[i].tail);
    }

}

void
IGbE::RXEnsoPipeManager::unserialize(CheckpointIn &cp)
{
    EnsoPipeManager::unserialize(cp);
    // Unserialize pipeStates array
    UNSERIALIZE_SCALAR(stateFull);
    for (uint64_t i = 0; i < MAX_NB_MANAGER; i++) {
        paramIn(cp, csprintf("pipeStates[%d].physAddr", i), pipeStates[i].physAddr);
        paramIn(cp, csprintf("pipeStates[%d].pipeStatus", i), pipeStates[i].pipeStatus);
        paramIn(cp, csprintf("pipeStates[%d].notifBufId", i), pipeStates[i].notifBufId);
        paramIn(cp, csprintf("pipeStates[%d].tail", i), pipeStates[i].tail);
    }

}



/********************* Notification Buffer Manager *******************/
template<class T>
IGbE::NotifBufManager<T>::NotifBufManager(IGbE *i, const std::string n, int s)
: igbe(i), _name(n), size(s)
{}

template<class T>
IGbE::NotifBufManager<T>::~NotifBufManager()
{}

template<class T>
void
IGbE::NotifBufManager<T>::serialize(CheckpointOut &cp) const
{
    SERIALIZE_SCALAR(size);
}

template<class T>
void
IGbE::NotifBufManager<T>::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(size);
}


/********************* RX Notification Buffer Manager *******************/
IGbE::RXNotifBufManager::RXNotifBufManager(IGbE *i, const std::string n, int s)
: NotifBufManager<igbreg::RxNotification>(i, n, s), pktDone(false), pktSplitDone(0), pktNotifDone(0), pktSplitNotifDone(0), pktPtr(nullptr),
    dmaNotifId(0), dmaQueueId(0), dmaTail(0), notifDone(false),
    pktEvent([this]{ pktComplete(); }, n),
   // pktDataEvent([this]{ pktNotifComplete(); }, n),
    pktNotifEvent([this]{ notifComplete(); }, n),
    pktFirstEvent([this]{ pktSplitComplete(); }, n),
    pktSecondEvent([this]{ pktSplitComplete(); }, n)
    //pktFirstDataEvent([this]{ pktSplitNotifComplete(); }, n),
    //pktSecondDataEvent([this]{ pktSplitNotifComplete(); }, n),
    //pktSplitNotifEvent([this]{ pktSplitNotifComplete(); }, n)
{
    for(int i = 0; i < MAX_NB_NOTIF; i++)
    {
        RXNotifStates[i].physAddr = 0;
        RXNotifStates[i].head = 0;
        RXNotifStates[i].tail = 0;
    }

}


/*
void
IGbE::RXNotifBufManager::writePacket(EnsoRxPtr pkt)
{
    assert(pkt);

    pktPtr = pkt;
    pktDone = false;

    uint32_t pktQueueId = pkt->meta.pktQueueId;
    uint64_t notifBufId = pkt->pipe->notifBufId;
    struct RXNotifState* RXNotifState = &RXNotifStates[notifBufId];

    uint32_t pipeTail = pkt->pipe->tail;
    uint32_t flits = pkt->flits;
    uint32_t pipeSize = ENSO_PIPE_SIZE;  // 32768 => 2MB/64B
    uint32_t pipe_bytes = BASE_PKT_SIZE * pipeSize; // 2MB
    uint32_t tailOffset = BASE_PKT_SIZE * pipeTail;

    uint32_t dmaLen = pkt->length;
    uint8_t* dmaData = pkt->data;

    // for logging
    bool isNotif = false;

    // update RX enso pipe tail
    pkt->pipe->tail = (pipeTail + flits) & ENSO_PIPE_MASK;
    printf("[RXEnsopipe] RX enso pipe base addr %lx, tail offset %u \n", pkt->pipe->physAddr, tailOffset);

    // prepare RX Notification
    notifRxBuf = new igbreg::RxNotification;
    assert(notifRxBuf);

    notifRxBuf->queue_id = pktQueueId;
    notifRxBuf->signal = 1;
    notifRxBuf->tail = pkt->pipe->tail;

    // Enso pipe head, tail -> 0 ~ 32767 
    if (pipeTail + flits < pipeSize) {
        if (!pkt->meta.isNotify) 
        {
            igbe->dmaWrite(pciToDma(pkt->pipe->physAddr + tailOffset),
                        dmaLen, &pktEvent, dmaData,
                        igbe->rxWriteDelay);

            delete notifRxBuf;
            notifRxBuf = NULL;
        }
        else
        {
            igbe->dmaWrite(pciToDma(pkt->pipe->physAddr + tailOffset),
                        dmaLen, &pktDataEvent, dmaData,
                        igbe->rxWriteDelay);

            // DMA notification
            igbe->dmaWrite(pciToDma(RXNotifState->physAddr + NOTIF_SIZE*RXNotifState->tail),
                        NOTIF_SIZE, &pktNotifEvent, (uint8_t*)notifRxBuf,
                        igbe->rxWriteDelay);

            // increment notification buffer tailPtr
            RXNotifState->tail = (RXNotifState->tail + 1) & NOTIF_BUF_MASK;
            updateNotifTail(RXNotifState->tail, notifBufId);

            isNotif = true;
        }
    } else {
        // wrap-around -> split packet & send
        uint32_t firstFlit = pipeSize - pipeTail;
        uint32_t firstFlitBytes = igbe->flitSize * firstFlit;

        uint32_t secondFlitBytes = dmaLen - firstFlitBytes;

        if (!pkt->meta.isNotify) 
        {
            igbe->dmaWrite(pciToDma(pkt->pipe->physAddr + tailOffset),
                       firstFlitBytes, &pktFirstEvent, dmaData,
                       igbe->rxWriteDelay);

            igbe->dmaWrite(pciToDma(pkt->pipe->physAddr),  // pipe base addr -> wrap-around
                        secondFlitBytes, &pktSecondEvent, dmaData + firstFlitBytes,
                        igbe->rxWriteDelay);

            delete notifRxBuf;
            notifRxBuf = NULL;
        }
        else
        {
            igbe->dmaWrite(pciToDma(pkt->pipe->physAddr + tailOffset),
                       firstFlitBytes, &pktFirstDataEvent, dmaData,
                       igbe->rxWriteDelay);

            igbe->dmaWrite(pciToDma(pkt->pipe->physAddr),  // pipe base addr -> wrap-around
                        secondFlitBytes, &pktSecondDataEvent, dmaData + firstFlitBytes,
                        igbe->rxWriteDelay);

            // DMA notification
            igbe->dmaWrite(pciToDma(RXNotifState->physAddr + NOTIF_SIZE*RXNotifState->tail),
                        NOTIF_SIZE, &pktSplitNotifEvent, (uint8_t*)notifRxBuf,
                        igbe->rxWriteDelay);

            // increment notification buffer tailPtr
            RXNotifState->tail = (RXNotifState->tail + 1) & NOTIF_BUF_MASK;
            updateNotifTail(RXNotifState->tail, notifBufId);

            isNotif = true;
        }   
    }

    // pipe tail ptr update
    DPRINTF(EthernetENSO, "PIPE_TAIL update: pipe_id=%u, old_tail=%u, flits=%u, new_tail=%u\n",
        pktQueueId, pipeTail, flits, pkt->pipe->tail);
    if(isNotif)
        DPRINTF(EthernetENSO, "RXNOTIF_TAIL update: pipe_id=%u, new_tail=%u\n", notifBufId, RXNotifState->tail);
    
}
*/
void
IGbE::RXNotifBufManager::writePacket(EnsoRxPtr pkt)
{
    assert(pkt);

    pktPtr = pkt;
    pktDone = false;

    uint32_t pktQueueId = pkt->meta.pktQueueId;
    uint64_t notifBufId = pkt->pipe->notifBufId;
    

    uint32_t pipeTail = pkt->pipe->tail;
    uint32_t flits = pkt->flits;
    uint32_t pipeSize = ENSO_PIPE_SIZE;  // 32768 => 2MB/64B
    uint32_t pipe_bytes = BASE_PKT_SIZE * pipeSize; // 2MB
    uint32_t tailOffset = BASE_PKT_SIZE * pipeTail;

    uint32_t dmaLen = pkt->length;
    uint8_t* dmaData = pkt->data;
    assert(dmaLen > 0);
    assert(dmaData);

    // update RX enso pipe tail
    pkt->pipe->tail = (pipeTail + flits) & ENSO_PIPE_MASK;
    //DPRINTF(EthernetEnsoRxNotif, "RxNotif : RX enso pipe base addr %lx, tail offset %u \n", pkt->pipe->physAddr, tailOffset);


    if(pkt->meta.isNotify)
    {
        setDmaNotifVar(notifBufId, pktQueueId, pkt->pipe->tail);
    }

    // Enso pipe head, tail -> 0 ~ 32767 
    if (pipeTail + flits < pipeSize) 
    {
        
        igbe->dmaWrite(pciToDma(pkt->pipe->physAddr + tailOffset),
                        dmaLen, &pktEvent, dmaData,
                        igbe->rxWriteDelay);

    }
    else 
    {
        // wrap-around -> split packet & send
        uint32_t firstFlit = pipeSize - pipeTail;
        uint32_t firstFlitBytes = igbe->flitSize * firstFlit;

        uint32_t secondFlitBytes = dmaLen - firstFlitBytes;

        igbe->dmaWrite(pciToDma(pkt->pipe->physAddr + tailOffset),
                       firstFlitBytes, &pktFirstEvent, dmaData,
                       igbe->rxWriteDelay);

        igbe->dmaWrite(pciToDma(pkt->pipe->physAddr),  // pipe base addr -> wrap-around
                        secondFlitBytes, &pktSecondEvent, dmaData + firstFlitBytes,
                        igbe->rxWriteDelay);


    }

    DPRINTF(EthernetEnsoRxNotif, "RxNotif :DMA packet %u bytes\n", dmaLen);
    // pipe tail ptr update
    DPRINTF(EthernetEnsoRxNotif, "RxNotif : Rx Enso Pipe tail update pipe_id=%u, old_tail=%u, flits=%u, new_tail=%u\n",
        pktQueueId, pipeTail, flits, pkt->pipe->tail);
    
    
}

void
IGbE::RXNotifBufManager::writeNotification()
{

    notifDone = false;
    struct RXNotifState* rxNotifState = &RXNotifStates[dmaNotifId];
    // prepare RX Notification
    notifRxBuf = new igbreg::RxNotification;
    assert(notifRxBuf);

    notifRxBuf->queue_id = dmaQueueId;
    notifRxBuf->signal = 1;
    notifRxBuf->tail = dmaTail;

    // DMA notification
    igbe->dmaWrite(pciToDma(rxNotifState->physAddr + NOTIF_SIZE*rxNotifState->tail),
                NOTIF_SIZE, &pktNotifEvent, (uint8_t*)notifRxBuf,
                igbe->rxWriteDelay);

    // increment notification buffer tailPtr
    rxNotifState->tail = (rxNotifState->tail + 1) & NOTIF_BUF_MASK;
    updateNotifTail(rxNotifState->tail, dmaNotifId);

    DPRINTF(EthernetEnsoRxNotif, "RxNotif : Rx Notif tail update: pipe_id=%u, new_tail=%u\n", dmaNotifId, rxNotifState->tail);
}

// ensure Notification & Packet DMAed
void
IGbE::RXNotifBufManager::pktNotifComplete()
{
    pktNotifDone++;
    assert(pktNotifDone <= 2);
    if (pktNotifDone != 2)
        return;
    pktNotifDone = 0;

    delete notifRxBuf;
    notifRxBuf = NULL;

    pktComplete();
}

// ensure Notification & Packet DMAed
void
IGbE::RXNotifBufManager::pktSplitNotifComplete()
{
    pktNotifDone++;
    assert(pktNotifDone <= 3);
    if (pktNotifDone != 3)
        return;
    pktNotifDone = 0;

    delete notifRxBuf;
    notifRxBuf = NULL;

    pktComplete();
}

void
IGbE::RXNotifBufManager::pktSplitComplete()
{
    pktSplitDone++;
    assert(pktSplitDone <= 2);
    if (pktSplitDone != 2)
        return;
    pktSplitDone = 0;

    pktComplete();
}



void
IGbE::RXNotifBufManager::pktComplete()
{
    // There is no need to post-process for notification buffer
    igbe->etherDeviceStats.rxDMABytes += pktPtr->length;

    pktPtr = NULL;
    igbe->checkDrain();
    enableSm();
    pktDone = true;
}

void
IGbE::RXNotifBufManager::notifComplete()
{
    igbe->etherDeviceStats.rxNotification += 1;
    igbe->etherDeviceStats.rxNotifDMABytes += NOTIF_SIZE;

    delete notifRxBuf;
    notifRxBuf = nullptr;
    clearDmaNotifVar();
    igbe->checkDrain();
    enableSm();
    notifDone = true;
}

bool
IGbE::RXNotifBufManager::packetDone()
{
    if (pktDone) {
        pktDone = false;
        return true;
    }
    return false;
}

bool
IGbE::RXNotifBufManager::notificationDone()
{
    if (notifDone) {
        notifDone = false;
        return true;
    }
    return false;
}

void
IGbE::RXNotifBufManager::enableSm()
{
    if (igbe->drainState() != DrainState::Draining) {
        igbe->rxTick = true;
        igbe->restartClock();
    }
}

bool
IGbE::RXNotifBufManager::hasOutstandingEvents()
{
    return pktEvent.scheduled() ||
           //pktDataEvent.scheduled() ||
           pktNotifEvent.scheduled() ||
           pktFirstEvent.scheduled() ||
           pktSecondEvent.scheduled();
           //pktFirstDataEvent.scheduled() ||
           //pktSecondDataEvent.scheduled() ||
           //pktSplitNotifEvent.scheduled();
}

void 
IGbE::RXNotifBufManager::serialize(CheckpointOut &cp) const {
    // Serialize RXNotifStates
    NotifBufManager<igbreg::RxNotification>::serialize(cp);

    SERIALIZE_SCALAR(pktSplitDone);
    SERIALIZE_SCALAR(pktNotifDone);
    SERIALIZE_SCALAR(pktSplitNotifDone);
    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(notifDone);
    SERIALIZE_SCALAR(dmaTail);
    SERIALIZE_SCALAR(dmaQueueId);
    SERIALIZE_SCALAR(dmaNotifId);

    for (uint64_t i = 0; i < MAX_NB_NOTIF; i++) {
        paramOut(cp, csprintf("RXNotifStates[%d].physAddr", i), RXNotifStates[i].physAddr);
        paramOut(cp, csprintf("RXNotifStates[%d].head", i), RXNotifStates[i].head);
        paramOut(cp, csprintf("RXNotifStates[%d].tail", i), RXNotifStates[i].tail);
    }


}

void 
IGbE::RXNotifBufManager::unserialize(CheckpointIn &cp) {
    // Unserialize RXNotifStates
    NotifBufManager<igbreg::RxNotification>::unserialize(cp);
    UNSERIALIZE_SCALAR(pktSplitDone);
    UNSERIALIZE_SCALAR(pktNotifDone);
    UNSERIALIZE_SCALAR(pktSplitNotifDone);
    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(notifDone);
    UNSERIALIZE_SCALAR(dmaTail);
    UNSERIALIZE_SCALAR(dmaQueueId);
    UNSERIALIZE_SCALAR(dmaNotifId);

    // Unserialize RXNotifStates array

    for (uint64_t i = 0; i < MAX_NB_NOTIF; i++) {
        paramIn(cp, csprintf("RXNotifStates[%d].physAddr", i), RXNotifStates[i].physAddr);
        paramIn(cp, csprintf("RXNotifStates[%d].head", i), RXNotifStates[i].head);
        paramIn(cp, csprintf("RXNotifStates[%d].tail", i), RXNotifStates[i].tail);
    }


    
}

/********************* TX Notification Buffer Manager *******************/

IGbE::TXNotifBufManager::TXNotifBufManager(IGbE *i, const std::string n, int s)
: NotifBufManager<igbreg::TxNotification>(i, n, s), pktDone(false), pktWaiting(false), pktPtr(NULL),
curFetching(0), curQueueId(0), wbOut(0), wbQueueId(0), moreToWb(false), awaitingContinuation(false),
wbDelayEvent([this]{ wbNotification1(); }, n),
fetchDelayEvent([this]{ fetchNotification1(); }, n),
fetchEvent([this]{ fetchComplete(); }, n),
wbEvent([this]{ wbComplete(); }, n),
pktEvent([this]{ pktComplete(); }, n)
{
    notifTxBuf.clear();
    txComplBuf.clear();
    
    fetchBuf = new igbreg::TxNotification[NOTIF_BUF_SIZE];
    assert(fetchBuf);
    wbBuf = new igbreg::TxNotification;
    assert(wbBuf);

    for(int i = 0; i < MAX_NB_NOTIF; i++)
        TXNotifBaseAddr[i] = 0;

}


IGbE::TXNotifBufManager::~TXNotifBufManager()
{
    notifTxBuf.clear();
    txComplBuf.clear();

    delete[] fetchBuf;
    delete wbBuf;
}

void
IGbE::TXNotifBufManager::enableSm()
{
    if (igbe->drainState() != DrainState::Draining) {
        igbe->txTick = true;
        igbe->restartClock();
    }
}


uint16_t 
IGbE::TXNotifBufManager::getPktLen(uint8_t* currentPkt)
{
    assert(currentPkt);
    const eth_hdr* etherHdr = reinterpret_cast<const eth_hdr*>(currentPkt);
    const ip_hdr* ipHdr = reinterpret_cast<const ip_hdr*>(etherHdr + 1);
    uint16_t pktLen = ntohs(ipHdr->ip_len) + sizeof(eth_hdr);
    // align flit size
    pktLen = ((pktLen + igbe->flitSize - 1) / igbe->flitSize) * igbe->flitSize;

    return pktLen;
}

void
IGbE::TXNotifBufManager::splitPacketToFifo(EthPacketPtr pkts)
{
    assert(pkts && pkts->data);

    uint8_t* currentPkt = pkts->data;
    uint32_t bytesRemaining = pkts->length;

    while (bytesRemaining > 0) {
        assert(currentPkt != nullptr);
        uint16_t pktLen = getPktLen(currentPkt);
        // assume pktLen can't be over DPDK MTU-size 
        if (pktLen == 0 || pktLen > 2048) {
            panic("splitPacketToFifo: Invalid pktLen (%u), possible corruption", pktLen);
        }

        // check pkt len is same as bytesRemainig
        if (pktLen > bytesRemaining) {
            DPRINTF(EthernetEnsoTxNotif, "TxNotif : invalid pktLen (%u) with bytesRemaining (%u)\n",
                    pktLen, bytesRemaining);
            // set flag of wrap-around data
            uint8_t* newData = nullptr;
            try {
                newData = new uint8_t[MAX_TX_TRANS + bytesRemaining];
            } catch (const std::bad_alloc& e) {
                panic("splitPacketToFifo: failed to allocate memory: %s", e.what());
            }
            memcpy(newData, currentPkt, bytesRemaining);

            delete[] pkts->data;
            pkts->data = newData;
            pkts->length = bytesRemaining;
            pkts->bufLength = MAX_TX_TRANS + bytesRemaining;

            awaitingContinuation = true;
            DPRINTF(EthernetEnsoTxNotif, "TxNotif : awaitingContinuation set to true\n");

            return;  
        }
            
        // need to make flit-alinged size??
        EthPacketPtr split_pkt = std::make_shared<EthPacketData>(pktLen);
        split_pkt->length = pktLen;
        memcpy(split_pkt->data, currentPkt, pktLen);

        // change to event handling ?? 
        // ex) igbe->schedule(&fifoevent, pktDelay*flits);
        igbe->txFifo.push(split_pkt);
     
        currentPkt += pktLen;
        bytesRemaining -= pktLen;

        
    }
    
    // flag setting == false
    awaitingContinuation = false;
    DPRINTF(EthernetEnsoTxNotif, "TxNotif : complete send packet to fifo \n");
}

unsigned
IGbE::TXNotifBufManager::getPacketSize()
{
    if(notifTxBuf.empty())
        return 0;
    
    TxCompletion* currentNoti = notifTxBuf.front();
    return currentNoti->txNotif->length;
}

void
IGbE::TXNotifBufManager::getPacketData(EthPacketPtr pkt)
{
    assert(notifTxBuf.size());

    TxCompletion* notif;
    notif = notifTxBuf.front();

    pktPtr = pkt;
    pktWaiting = true;

    igbe->dmaRead(pciToDma(notif->txNotif->phys_addr),
                    notif->txNotif->length, &pktEvent, pkt->data + pkt->length,
                    igbe->txReadDelay);

    DPRINTF(EthernetEnsoTxNotif, "TxNotif : dma packet %lu bytes from %lx\n", notif->txNotif->length, notif->txNotif->phys_addr);

}

void
IGbE::TXNotifBufManager::pktComplete()
{
    TxCompletion* complNotif;
    complNotif = notifTxBuf.front();

    igbe->etherDeviceStats.txDMABytes += complNotif->txNotif->length;

    // pkt->length increment
    pktPtr->length += complNotif->txNotif->length;

    complNotif->txNotif->signal = 0;
    txComplBuf.push_back(complNotif);

    notifTxBuf.pop_front();

    pktDone = true;
    pktWaiting = false;
    pktPtr = NULL;
    enableSm();
    igbe->checkDrain();
}

void
IGbE::TXNotifBufManager::fetchNotification(int queueId)
{
    size_t maxToFetch;
    if(curFetching)
        return;

    int curHead = notifBufHead(queueId);
    int curTail = notifBufTail(queueId);

    if (curTail >= curHead)
        maxToFetch = curTail - curHead;
    else
        maxToFetch = NOTIF_BUF_SIZE - curHead;
    
    // check free buffer size
    size_t freeSize = NOTIF_BUF_SIZE - notifTxBuf.size();

    maxToFetch = std::min(maxToFetch, freeSize);

    if (maxToFetch == 0)
        return;

    curFetching = maxToFetch;
    curQueueId = queueId;


    assert(!fetchDelayEvent.scheduled());
    igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
}


void
IGbE::TXNotifBufManager::fetchNotification1()
{
    if (igbe->drainState() != DrainState::Running) {
        igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
        return;
    }

    Addr base = notifBufBase(curQueueId);
    int curHead = notifBufHead(curQueueId);
    Addr dmaAddr = base + curHead*sizeof(igbreg::TxNotification);

    // DMA from physAddr + offset (cur head)
    igbe->dmaRead(pciToDma(dmaAddr),
                  curFetching * sizeof(igbreg::TxNotification), &fetchEvent, (uint8_t*)fetchBuf,
                  igbe->fetchCompDelay);

    DPRINTF(EthernetEnsoTxNotif, "TxNotif : fetch %d notification from %lx\n", curFetching, dmaAddr);
                  
}

void
IGbE::TXNotifBufManager::fetchComplete()
{   
    igbe->etherDeviceStats.txNotifDMABytes += (curFetching * sizeof(igbreg::TxNotification));
    igbe->etherDeviceStats.txNotification += curFetching;
    TxCompletion *newNotif;
    for (int x = 0; x < curFetching; x++) {
        newNotif = new TxCompletion;
        newNotif->txNotif = new igbreg::TxNotification; 

        assert(newNotif);
        assert(newNotif->txNotif);

        memcpy(newNotif->txNotif, &fetchBuf[x], sizeof(igbreg::TxNotification));
        newNotif->queueId = curQueueId;
        newNotif->wbHead = notifBufHead(curQueueId) + x;
        notifTxBuf.push_back(newNotif);
    }

    // update notification head with the number of fetched notification
    // need to accumulate 
    // updateHead(curQueueId, curFetching);
    updateHead(curQueueId, notifBufHead(curQueueId) + curFetching);
    DPRINTF(EthernetEnsoTxNotif, "TxNotif : Tx Notif head update: pipe_id=%u, new_tail=%u\n", curQueueId, curFetching);

    curFetching = 0;
    curQueueId = 0;

    enableSm();
    igbe->checkDrain();
}

/*
 Write back Notification logic
 write back 1 Completion Notification per req
*/
void
IGbE::TXNotifBufManager::wbNotification(int queueId)
{

    if (wbOut)
        return;

    wbOut = 1;

    wbQueueId = queueId;

    assert(!wbDelayEvent.scheduled());
    igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);

    

}

void
IGbE::TXNotifBufManager::wbNotification1()
{
    // If we're draining delay issuing this DMA
    if (igbe->drainState() != DrainState::Running) 
    {
        igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);
        return;
    }

    Addr base = notifBufBase(wbQueueId);
    int firstHead = txComplBuf.front()->wbHead;
    Addr dmaAddr = base + firstHead * sizeof(igbreg::TxNotification);

    // batch process
    // if error need to change DMA per Completion??
    assert(txComplBuf.size());
    assert(wbBuf);

    memcpy(wbBuf, txComplBuf.front()->txNotif, sizeof(igbreg::TxNotification));
    

    igbe->dmaWrite(pciToDma(dmaAddr),
                   wbOut * sizeof(igbreg::TxNotification), &wbEvent, (uint8_t*)wbBuf,
                   igbe->wbCompDelay);

    DPRINTF(EthernetEnsoTxNotif, "TxNotif : Write back TX Completion to %lx \n", dmaAddr);

}


void
IGbE::TXNotifBufManager::wbComplete()
{
    assert(txComplBuf.size());
    // need to freeing memory of notification
    igbe->etherDeviceStats.complNotification += wbOut;
    igbe->etherDeviceStats.txComplDMABytes += wbOut * sizeof(igbreg::TxNotification);

    delete txComplBuf.front()->txNotif;
    delete txComplBuf.front();
    txComplBuf.pop_front();
    

    // curHead += wbOut;
    wbOut = 0;
    wbQueueId = 0;

    if (!wbOut)
        igbe->checkDrain();

}

bool
IGbE::TXNotifBufManager::hasOutstandingEvents()
{
    return fetchEvent.scheduled() || pktEvent.scheduled() || wbEvent.scheduled();
}

void
IGbE::TXNotifBufManager::serialize(CheckpointOut &cp) const
{
    NotifBufManager<igbreg::TxNotification>::serialize(cp);

    SERIALIZE_SCALAR(curFetching);
    SERIALIZE_SCALAR(curQueueId);
    SERIALIZE_SCALAR(wbOut);
    SERIALIZE_SCALAR(wbQueueId);
    SERIALIZE_SCALAR(moreToWb);
    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(pktWaiting);
    SERIALIZE_SCALAR(awaitingContinuation);
    /*
    // Serialize notifTxBuf
    uint64_t notifTxBufSize = notifTxBuf.size();
    SERIALIZE_SCALAR(notifTxBufSize);
    for (uint64_t i = 0; i < notifTxBufSize; i++) {
        const struct TxCompletion* entry_n = notifTxBuf[i];
        const struct igbreg::TxNotification* entry_n_ = entry_n->txNotif;
        paramOut(cp, csprintf("notifTxBuf[%d]->queueId", i), entry_n->queueId);
        paramOut(cp, csprintf("notifTxBuf[%d]->wbHead", i), entry_n->wbHead);

        paramOut(cp, csprintf("notifTxBuf[%d]->txNotif->length", i), (uint64_t&)entry_n_->length);
        paramOut(cp, csprintf("notifTxBuf[%d]->txNotif->phys_addr", i), (uint64_t&)entry_n_->phys_addr);
        paramOut(cp, csprintf("notifTxBuf[%d]->txNotif->signal", i), (uint64_t&)entry_n_->signal);
    }

    // Serialize txComplBuf
    uint64_t txComplBufSize = txComplBuf.size();
    SERIALIZE_SCALAR(txComplBufSize);
    for (uint64_t i = 0; i < txComplBufSize; i++) {
        const struct TxCompletion* entry_t = txComplBuf[i];
        const struct igbreg::TxNotification* entry_t_ = entry_t->txNotif;
        paramOut(cp, csprintf("txComplBuf[%d]->queueId", i), entry_t->queueId);
        paramOut(cp, csprintf("txComplBuf[%d]->wbHead", i), entry_t->wbHead);

        paramOut(cp, csprintf("txComplBuf[%d]->txNotif->length", i), (uint64_t&)entry_t_->length);
        paramOut(cp, csprintf("txComplBuf[%d]->txNotif->phys_addr", i), (uint64_t&)entry_t_->phys_addr);
        paramOut(cp, csprintf("txComplBuf[%d]->txNotif->signal", i), (uint64_t&)entry_t_->signal);
    }
    */
}

void
IGbE::TXNotifBufManager::unserialize(CheckpointIn &cp)
{
    NotifBufManager<igbreg::TxNotification>::unserialize(cp);
    UNSERIALIZE_SCALAR(curFetching);
    UNSERIALIZE_SCALAR(curQueueId);
    UNSERIALIZE_SCALAR(wbOut);
    UNSERIALIZE_SCALAR(wbQueueId);
    UNSERIALIZE_SCALAR(moreToWb);
    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(pktWaiting);
    UNSERIALIZE_SCALAR(awaitingContinuation);

    /*
    // Unserialize notifTxBuf
    uint64_t notifTxBufSize = 0;
    UNSERIALIZE_SCALAR(notifTxBufSize);
    notifTxBuf.resize(notifTxBufSize);  // Resize the deque to match the serialized size

    for (uint64_t i = 0; i < notifTxBufSize; i++) {
        struct TxCompletion *entry_n = new struct TxCompletion;  // Allocate memory for TxCompletion
        notifTxBuf[i] = entry_n;  // Assign the newly allocated TxCompletion object

        // Unserialize individual fields
        paramIn(cp, csprintf("notifTxBuf[%d]->queueId", i), entry_n->queueId);
        paramIn(cp, csprintf("notifTxBuf[%d]->wbHead", i), entry_n->wbHead);

        // Unserialize the TxNotification structure
        struct igbreg::TxNotification *entry_txNotif = new igbreg::TxNotification;  // Allocate memory for TxNotification
        entry_n->txNotif = entry_txNotif;  // Assign the TxNotification object to txNotif pointer

        paramIn(cp, csprintf("notifTxBuf[%d]->txNotif->length", i), (uint64_t&)entry_txNotif->length);
        paramIn(cp, csprintf("notifTxBuf[%d]->txNotif->phys_addr", i), (uint64_t&)entry_txNotif->phys_addr);
        paramIn(cp, csprintf("notifTxBuf[%d]->txNotif->signal", i), (uint64_t&)entry_txNotif->signal);
    }

    // Unserialize notifTxBuf
    uint64_t txComplBufSize = 0;
    UNSERIALIZE_SCALAR(txComplBufSize);
    notifTxBuf.resize(txComplBufSize);  // Resize the deque to match the serialized size

    for (uint64_t i = 0; i < txComplBufSize; i++) {
        struct TxCompletion *entry_t = new struct TxCompletion;  // Allocate memory for TxCompletion
        txComplBuf[i] = entry_t;  // Assign the newly allocated TxCompletion object

        // Unserialize individual fields
        paramIn(cp, csprintf("txComplBuf[%d]->queueId", i), entry_t->queueId);
        paramIn(cp, csprintf("txComplBuf[%d]->wbHead", i), entry_t->wbHead);

        // Unserialize the TxNotification structure
        struct igbreg::TxNotification *entry_txNotif = new igbreg::TxNotification;  // Allocate memory for TxNotification
        entry_t->txNotif = entry_txNotif;  // Assign the TxNotification object to txNotif pointer

        paramIn(cp, csprintf("txComplBuf[%d]->txNotif->length", i), (uint64_t&)entry_txNotif->length);
        paramIn(cp, csprintf("txComplBuf[%d]->txNotif->phys_addr", i), (uint64_t&)entry_txNotif->phys_addr);
        paramIn(cp, csprintf("txComplBuf[%d]->txNotif->signal", i), (uint64_t&)entry_txNotif->signal);
    }
    */
}

// rxStateMachine, txStateMachine, ethRxPkt, txWire function for ENSO
// demo version of ethRxPkt() function, will be erased


void IGbE::rxEnsoStateMachine()
{
    if (!regs.rctl.en()) {
        rxTick = false;
        DPRINTF(EthernetENSO, "RXS: RX disabled, stopping ticking\n");
        return;
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

// Todo : need to check TX process
void IGbE::txEnsoStateMachine()
{
    if (!regs.tctl.en()) {
        txTick = false;
        DPRINTF(EthernetENSO, "TXS: TX disabled, stopping ticking\n");
        return;
    }

    // check need to send completion notification
    // Todo,, considering multiple notification buffer
    // considering DMA etc...
    // but if TX notifManager processing wrap-around data, skip this
    if (!txNotifBufManager.isAwaitingContinuation())
    {
        if(!txNotifBufManager.txComplBufEmpty())
        {
            // need to check host TX notification buffer is full??
            int queueId = txNotifBufManager.fisrtComplQueueId();
            txNotifBufManager.wbNotification(queueId);
            return;
        }
    }
    

    // If we have a packet available and it's length is not 0 (meaning it's not
    // a multidescriptor packet) put it in the fifo, otherwise an the next
    // iteration we'll get the rest of the data
    if (txPacket && txNotifBufManager.packetAvailable()
        && txPacket->length) {

        if(txNotifBufManager.packetWaiting())
        {
            // check condition for splitted notification
            DPRINTF(EthernetENSO, "TXS: Waiting remained packet \n");
            return;
        }

        DPRINTF(EthernetENSO, "TXS: packets placed in TX FIFO\n");

        // split multiple packets
        txNotifBufManager.splitPacketToFifo(txPacket);
        
        txFifoTick = true && drainState() != DrainState::Draining;
        
        if (!txNotifBufManager.isAwaitingContinuation())
        {
            txPacket = NULL;
            return;
        }
    }

    if (!txPacket) {
        txPacket = std::make_shared<EthPacketData>(MAX_TX_TRANS);
    }

    if (!txNotifBufManager.packetWaiting()) 
    {
        unsigned size = txNotifBufManager.getPacketSize();
        if (size > 0 && txFifo.avail() > size) {
            DPRINTF(EthernetENSO, "TXS: Reserving %d bytes in FIFO and "
                    "beginning DMA of next packet\n", size);
            txFifo.reserve(size);
            txNotifBufManager.getPacketData(txPacket);
        } else if (size == 0) {
            // pop notification?? writeback notification?

        } else {
            DPRINTF(EthernetENSO, "TXS: TX fifo full, stop ticking...\n");
            etherDeviceStats.txFifoFullCount++;
            txTick = false;
        }
        return;
    }

    // when nothing to do, check notification Head != Tail
    // if different, fetchNotification
    /*
    for(int i = 0; i < MAX_NB_NOTIF; i++)
    {
        if(txNotifBufManager.txNotifBufHead(i) != txNotifBufManager.txNotifBufTail(i))
        {
            DPRINTF(EthernetENSO, "TXS: beginning fetch notification from %d queue", i);
            txNotifBufManager.fetchNotification(i);
            txTick = false;
            return;
        }
            
    }
    */

    DPRINTF(EthernetENSO, "TXS: Current TX notif[0] head %u tail %u \n", txNotifBufManager.txNotifBufHead(0),txNotifBufManager.txNotifBufTail(0) );

    DPRINTF(EthernetENSO, "TXS: Nothing to do, stopping ticking\n");
    txTick = false;
}

#endif



} // namespace gem5