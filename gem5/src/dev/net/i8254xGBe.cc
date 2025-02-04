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

namespace gem5
{

using namespace igbreg;
using namespace networking;

IGbE::IGbE(const Params &p)
    : EtherDevice(p), adq(p.adq_idx), etherInt(NULL), m2funcPort(NULL), enableDTA(p.enable_dta), numQueues(p.num_queues),     // SHIN. add adq // jm. add numQueues
      rxFifo(p.rx_fifo_size, true), txFifo(p.tx_fifo_size, false), inTick(false),
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
            rxDescCacheArray[i] = new RxDescCache(this, name()+".RxDescArray"+std::to_string(i), p.rx_desc_cache_size, i);
            txDescCacheArray[i] = new TxDescCache(this, name()+".TxDescArray"+std::to_string(i), p.tx_desc_cache_size, i);
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
                    rxDescCacheArray[i]->writeback(0);
                }
            }
            // rxDescCache.writeback(0);
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
        } else if (isRegisterAddress<E1000_RDT>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            pkt->setLE<uint32_t>(regs.rdt_array[queueid]());
            uint32_t rdt = regs.rdt_array[queueid]();
            DPRINTF(EthernetDpdk, "Read RDT[%d]: %d\n", queueid, rdt);
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
    //   case REG_RDBAL:
    //     regs.rdba.rdbal( val & ~mask(4));
    //     rxDescCache.areaChanged();
    //     break;
    //   case REG_RDBAH:
    //     regs.rdba.rdbah(val);
    //     rxDescCache.areaChanged();
    //     break;
    //   case REG_RDLEN:
    //     regs.rdlen = val & ~mask(7);
    //     rxDescCache.areaChanged();
    //     break;
    //   case REG_SRRCTL:
    //     regs.srrctl = val;
    //     break;
    //   case REG_RDH:
    //     regs.rdh = val;
    //     rxDescCache.areaChanged();
    //     break;
    //   case REG_RDT:
    //     regs.rdt = val;
    //     DPRINTF(EthernetSM, "RXS: RDT Updated.\n");
    //     if (drainState() == DrainState::Running) {
    //         DPRINTF(EthernetSM, "RXS: RDT Fetching Descriptors!\n");
    //         rxDescCache.fetchDescriptors();
    //     } else {
    //         DPRINTF(EthernetSM, "RXS: RDT NOT Fetching Desc b/c draining!\n");
    //     }
    //     break;
      case REG_RDTR:
        regs.rdtr = val;
        break;
      case REG_RADV:
        regs.radv = val;
        break;
    //   case REG_RXDCTL:
    //     regs.rxdctl = val;
    //     break;
    //   case REG_TDBAL:
    //     regs.tdba.tdbal( val & ~mask(4));
    //     txDescCache.areaChanged();
    //     break;
    //   case REG_TDBAH:
    //     regs.tdba.tdbah(val);
    //     txDescCache.areaChanged();
    //     break;
    //   case REG_TDLEN:
    //     regs.tdlen = val & ~mask(7);
    //     txDescCache.areaChanged();
    //     break;
    //   case REG_TDH:
    //     regs.tdh = val;
    //     txDescCache.areaChanged();
    //     break;
    //   case REG_TXDCA_CTL:
    //     regs.txdca_ctl = val;
    //     if (regs.txdca_ctl.enabled())
    //         panic("No support for DCA\n");
    //     break;
    //   case REG_TDT:
    //     regs.tdt = val;
    //     DPRINTF(EthernetSM, "TXS: TX Tail pointer updated\n");
    //     if (drainState() == DrainState::Running) {
    //         DPRINTF(EthernetSM, "TXS: TDT Fetching Descriptors!\n");
    //         txDescCache.fetchDescriptors();
    //     } else {
    //         DPRINTF(EthernetSM, "TXS: TDT NOT Fetching Desc b/c draining!\n");
    //     }
    //     break;
      case REG_TIDV:
        regs.tidv = val;
        break;
    //   case REG_TXDCTL:
    //     regs.txdctl = val;
    //     break;
      case REG_TADV:
        regs.tadv = val;
        break;
    //   case REG_TDWBAL:
    //     regs.tdwba &= ~mask(32);
    //     regs.tdwba |= val;
    //     txDescCache.completionWriteback(regs.tdwba & ~mask(1),
    //                                     regs.tdwba & mask(1));
    //     break;
    //   case REG_TDWBAH:
    //     regs.tdwba &= mask(32);
    //     regs.tdwba |= (uint64_t)val << 32;
    //     txDescCache.completionWriteback(regs.tdwba & ~mask(1),
    //                                     regs.tdwba & mask(1));
    //     break;
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
            regs.rdba_array[queueid].rdbal(val & ~mask(4));
            if (commType == CommunicationType::RING) 
                rxDescCacheArray[queueid]->areaChanged();
            uint32_t rdbal = regs.rdba_array[queueid].rdbal();
            DPRINTF(EthernetDpdk, "Write RDBAL[%d]: %#x\n", queueid, rdbal);
        } else if (isRegisterAddress<E1000_RDBAH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.rdba_array[queueid].rdbah(val);
            if (commType == CommunicationType::RING) 
                rxDescCacheArray[queueid]->areaChanged();
            uint32_t rdbah = regs.rdba_array[queueid].rdbah();
            DPRINTF(EthernetDpdk, "Write RDBAH[%d]: %#x\n", queueid, rdbah);
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
            regs.rdh_array[queueid] = val;
            if (commType == CommunicationType::RING)
                rxDescCacheArray[queueid]->areaChanged();
            DPRINTF(EthernetDpdk, "Write RDH[%d]: %d\n", queueid, regs.rdh_array[queueid]());
        } else if (isRegisterAddress<E1000_RDT>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.rdt_array[queueid] = val;
            DPRINTF(EthernetDpdk, "RXS: RDT Updated.\n");
            DPRINTF(EthernetDpdk, "Write RDT[%d]: %d\n", queueid, regs.rdt_array[queueid]());
            etherDeviceStats.rxTailWriteBytes += pkt->getSize();
            if (commType == CommunicationType::RING) {
                if (drainState() == DrainState::Running) {
                    DPRINTF(EthernetDpdk, "RXS: RDT Fetching Descriptors! in queue %d\n",
                            queueid);
                    rxDescCacheArray[queueid]->fetchDescriptors();
                } else {
                    printf("RXS: RDT NOT Fetching Desc b/c draining! in queue %d\n", queueid);
                }
            }
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
            if (commType == CommunicationType::RING)
                txDescCacheArray[queueid]->areaChanged();
            DPRINTF(EthernetDpdk, "Write TDBAL[%d]: %#x\n", queueid, regs.tdba_array[queueid].tdbal());
        } else if (isRegisterAddress<E1000_TDBAH>(daddr, queueid, numQueues)) {
            assert(queueid < numQueues);
            assert(queueid >= 0);
            regs.tdba_array[queueid].tdbah(val);
            if (commType == CommunicationType::RING)
                txDescCacheArray[queueid]->areaChanged();
            DPRINTF(EthernetDpdk, "Write TDBAH[%d]: %#x\n", queueid, regs.tdba_array[queueid].tdbah());
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
            if (commType == CommunicationType::RING)
                txDescCacheArray[queueid]->areaChanged();
            DPRINTF(EthernetDpdk, "Write TDH[%d]: %d\n", queueid, regs.tdh_array[queueid]());
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
            DPRINTF(EthernetDpdk, "TXS: TX Tail pointer updated in queue %d\n", queueid);
            DPRINTF(EthernetDpdk, "Write TDT[%d]: %d\n", queueid, regs.tdt_array[queueid]());
            etherDeviceStats.txTailWriteBytes += pkt->getSize();
            if (commType == CommunicationType::RING) {
                if (drainState() == DrainState::Running) {
                    DPRINTF(EthernetDpdk, "TXS: TDT Fetching Descriptors! in queue %d\n", queueid);  
                    txDescCacheArray[queueid]->fetchDescriptors();
                } else {
                    printf("TXS: TDT NOT Fetching Desc b/c draining! in queue %d\n", queueid);
                }
            }
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

    // if (rdtrEvent.scheduled()) {
    //     regs.icr.rxt0(1);
    //     deschedule(rdtrEvent);
    // }
    // if (radvEvent.scheduled()) {
    //     regs.icr.rxt0(1);
    //     deschedule(radvEvent);
    // }
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
    // if (tadvEvent.scheduled()) {
    //     regs.icr.txdw(1);
    //     deschedule(tadvEvent);
    // }
    // if (tidvEvent.scheduled()) {
    //     regs.icr.txdw(1);
    //     deschedule(tidvEvent);
    // }
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
IGbE::DescCache<T>::DescCache(IGbE *i, const std::string n, int s, bool _isRx)
    : igbe(i), _name(n), cachePnt(0), size(s), curFetching(0), isRx(_isRx),
      wbOut(0), moreToWb(false), wbAlignment(0), pktPtr(NULL),
      wbDelayEvent([this]{ writeback1(); }, n),
      fetchDelayEvent([this]{ fetchDescriptors1(); }, n),
      fetchEvent([this]{ fetchComplete(); }, n),
      wbEvent([this]{ wbComplete(); }, n)
{
    fetchBuf = new T[size];
    wbBuf = new T[size];
}

template<class T>
IGbE::DescCache<T>::~DescCache()
{
    reset();
    delete[] fetchBuf;
    delete[] wbBuf;
}

template<class T>
void
IGbE::DescCache<T>::areaChanged()
{
    if (usedCache.size() > 0 || curFetching || wbOut)
        panic("Descriptor Address, Length or Head changed. Bad\n");
    reset();

}

template<class T>
void
IGbE::DescCache<T>::writeback(Addr aMask)
{
    int curHead = descHead();
    int max_to_wb = usedCache.size();

    // Check if this writeback is less restrictive that the previous
    // and if so setup another one immediately following it
    if (wbOut) {
        if (aMask < wbAlignment) {
            moreToWb = true;
            wbAlignment = aMask;
        }
        DPRINTF(EthernetDesc,
                "Writing back already in process, returning\n");
        return;
    }

    moreToWb = false;
    wbAlignment = aMask;


    DPRINTF(EthernetDesc, "Writing back descriptors head: %d tail: "
            "%d len: %d cachePnt: %d max_to_wb: %d descleft: %d\n",
            curHead, descTail(), descLen(), cachePnt, max_to_wb,
            descLeft());

    if (max_to_wb + curHead >= descLen()) {
        max_to_wb = descLen() - curHead;
        moreToWb = true;
        // this is by definition aligned correctly
    } else if (wbAlignment != 0) {
        // align the wb point to the mask
        max_to_wb = max_to_wb & ~wbAlignment;
    }

    DPRINTF(EthernetDesc, "Writing back %d descriptors\n", max_to_wb);

    if (max_to_wb <= 0)
        return;

    wbOut = max_to_wb;

    assert(!wbDelayEvent.scheduled());
    igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);
}

template<class T>
void
IGbE::DescCache<T>::writeback1()
{
    // If we're draining delay issuing this DMA
    if (igbe->drainState() != DrainState::Running) {
        igbe->schedule(wbDelayEvent, curTick() + igbe->wbDelay);
        return;
    }

    DPRINTF(EthernetDesc, "Begining DMA of %d descriptors\n", wbOut);
    DPRINTF(EthernetDpdk, "Begining DMA of %d descriptors\n", wbOut);

    for (int x = 0; x < wbOut; x++) {
        assert(usedCache.size());
        memcpy(&wbBuf[x], usedCache[x], sizeof(T));
    }


    assert(wbOut);

    // SHIN. Change to IDIO
    // igbe->dmaWrite(pciToDma(descBase() + descHead() * sizeof(T)),
    //                wbOut * sizeof(T), &wbEvent, (uint8_t*)wbBuf,
    //                igbe->wbCompDelay);
    // printf("WritebackDesc At clk: %ld IdioWrite Addr: %lx, Size: %d, Delay: %ld\n",
    //         curTick(), pciToDma(descBase() + descHead() * sizeof(T)),
    //         wbOut * sizeof(T), igbe->wbCompDelay);
    igbe->IdioWrite(pciToDma(descBase() + descHead() * sizeof(T)),
                    wbOut * sizeof(T), &wbEvent, (uint8_t*)wbBuf,
                    igbe->wbCompDelay, 0, igbe->adq);
}

template<class T>
void
IGbE::DescCache<T>::fetchDescriptors()
{
    size_t max_to_fetch;

    if (curFetching) {
        DPRINTF(EthernetDesc,
                "Currently fetching %d descriptors, returning\n",
                curFetching);
        DPRINTF(EthernetDpdk,
                "Currently fetching %d descriptors, returning\n",
                curFetching);
        return;
    }

    if (descTail() >= cachePnt)
        max_to_fetch = descTail() - cachePnt;
    else {
        assert(descLen() >= cachePnt);
        max_to_fetch = descLen() - cachePnt;
    }

    // Fix the underflow
    size_t totalUsed = usedCache.size() + unusedCache.size();
    size_t free_cache = (static_cast<size_t>(size) >= totalUsed) ? (static_cast<size_t>(size) - totalUsed) : 0;


    max_to_fetch = std::min(max_to_fetch, free_cache);


    DPRINTF(EthernetDesc, "Fetching descriptors head: %d tail: "
            "%d len: %d cachePnt: %d max_to_fetch: %d descleft: %d\n",
            descHead(), descTail(), descLen(), cachePnt,
            max_to_fetch, descLeft());
    DPRINTF(EthernetDpdk, "Fetching descriptors head: %d tail: "
            "%d len: %d cachePnt: %d max_to_fetch: %d descleft: %d\n",
            descHead(), descTail(), descLen(), cachePnt,
            max_to_fetch, descLeft());
    // Nothing to do
    if (max_to_fetch == 0)
        return;

    // So we don't have two descriptor fetches going on at once
    curFetching = max_to_fetch;

    assert(!fetchDelayEvent.scheduled());
    igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
}

template<class T>
void
IGbE::DescCache<T>::fetchDescriptors1()
{
    // If we're draining delay issuing this DMA
    if (igbe->drainState() != DrainState::Running) {
        igbe->schedule(fetchDelayEvent, curTick() + igbe->fetchDelay);
        return;
    }

    DPRINTF(EthernetDesc, "Fetching descriptors at %#x (%#x), size: %#x\n",
            descBase() + cachePnt * sizeof(T),
            pciToDma(descBase() + cachePnt * sizeof(T)),
            curFetching * sizeof(T));
    // printf("FetchDesc At clk: %ld DmaRead Addr: %lx, Size: %d, Delay: %ld\n",
    //         curTick(), pciToDma(descBase() + cachePnt * sizeof(T)),
    //         curFetching * sizeof(T), igbe->fetchCompDelay);
    assert(curFetching);
    igbe->dmaRead(pciToDma(descBase() + cachePnt * sizeof(T)),
                  curFetching * sizeof(T), &fetchEvent, (uint8_t*)fetchBuf,
                  igbe->fetchCompDelay);
}

template<class T>
void
IGbE::DescCache<T>::fetchComplete()
{
    T *newDesc;
    for (int x = 0; x < curFetching; x++) {
        newDesc = new T;
        memcpy(newDesc, &fetchBuf[x], sizeof(T));
        unusedCache.push_back(newDesc);
    }

    igbe->etherDeviceStats.metaDMABytes += curFetching * sizeof(T);
    if (isRx) {
        igbe->etherDeviceStats.rxDescFetchBytes += curFetching * sizeof(T);
    } else {
        igbe->etherDeviceStats.txDescFetchBytes += curFetching * sizeof(T);
    }


    int oldCp = cachePnt;

    cachePnt += curFetching;
    assert(cachePnt <= descLen());
    if (cachePnt == descLen())
        cachePnt = 0;

    curFetching = 0;

    DPRINTF(EthernetDesc, "Fetching complete cachePnt %d -> %d\n",
            oldCp, cachePnt);

    enableSm();
    igbe->checkDrain();
}

template<class T>
void
IGbE::DescCache<T>::wbComplete()
{

    long curHead = descHead();
    long oldHead = curHead;

    for (int x = 0; x < wbOut; x++) {
        assert(usedCache.size());
        delete usedCache[0];
        usedCache.pop_front();
    }

    igbe->etherDeviceStats.metaDMABytes += wbOut * sizeof(T);
    if (isRx) {
        igbe->etherDeviceStats.rxDescWBBytes += wbOut * sizeof(T);
    } else {
        igbe->etherDeviceStats.txDescWBBytes += wbOut * sizeof(T);
    }

    curHead += wbOut;
    wbOut = 0;

    if (curHead >= descLen())
        curHead -= descLen();

    // Update the head
    updateHead(curHead);

    DPRINTF(EthernetDesc, "Writeback complete curHead %d -> %d\n",
            oldHead, curHead);

    // If we still have more to wb, call wb now
    actionAfterWb();
    if (moreToWb) {
        moreToWb = false;
        DPRINTF(EthernetDesc, "Writeback has more todo\n");
        writeback(wbAlignment);
    }

    if (!wbOut)
        igbe->checkDrain();
    fetchAfterWb();
}

template<class T>
void
IGbE::DescCache<T>::reset()
{
    DPRINTF(EthernetDesc, "Reseting descriptor cache\n");
    for (typename CacheType::size_type x = 0; x < usedCache.size(); x++)
        delete usedCache[x];
    for (typename CacheType::size_type x = 0; x < unusedCache.size(); x++)
        delete unusedCache[x];

    usedCache.clear();
    unusedCache.clear();

    cachePnt = 0;

}

template<class T>
void
IGbE::DescCache<T>::serialize(CheckpointOut &cp) const
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

    typename CacheType::size_type unusedCacheSize = unusedCache.size();
    SERIALIZE_SCALAR(unusedCacheSize);
    for (typename CacheType::size_type x = 0; x < unusedCacheSize; x++) {
        arrayParamOut(cp, csprintf("unusedCache_%d", x),
                      (uint8_t*)unusedCache[x],sizeof(T));
    }

    Tick fetch_delay = 0, wb_delay = 0;
    if (fetchDelayEvent.scheduled())
        fetch_delay = fetchDelayEvent.when();
    SERIALIZE_SCALAR(fetch_delay);
    if (wbDelayEvent.scheduled())
        wb_delay = wbDelayEvent.when();
    SERIALIZE_SCALAR(wb_delay);


}

template<class T>
void
IGbE::DescCache<T>::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_SCALAR(cachePnt);
    UNSERIALIZE_SCALAR(curFetching);
    UNSERIALIZE_SCALAR(wbOut);
    UNSERIALIZE_SCALAR(moreToWb);
    UNSERIALIZE_SCALAR(wbAlignment);

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
    if (fetch_delay)
        igbe->schedule(fetchDelayEvent, fetch_delay);
    if (wb_delay)
        igbe->schedule(wbDelayEvent, wb_delay);


}

///////////////////////////// IGbE::RxDescCache //////////////////////////////

IGbE::RxDescCache::RxDescCache(IGbE *i, const std::string n, int s, int qid)
    : DescCache<RxDesc>(i, n, s, true), pktDone(false), splitCount(0), queueID(qid),
    pktEvent([this]{ pktComplete(); }, n),
    pktHdrEvent([this]{ pktSplitDone(); }, n),
    pktDataEvent([this]{ pktSplitDone(); }, n),
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
IGbE::RxDescCache::writePacket(EthPacketPtr packet, int pkt_offset)
{
    assert(unusedCache.size());
    //if (!unusedCache.size())
    //    return false;

    pktPtr = packet;
    pktDone = false;
    unsigned buf_len, hdr_len;

    if (pktPtr->rxDMAStartTick == 0)
        pktPtr->rxDMAStartTick = curTick();

    RxDesc *desc = unusedCache.front();
    switch (igbe->regs.srrctl_array[queueID].desctype()) {
      case RXDT_LEGACY:
        assert(pkt_offset == 0);
        bytesCopied = packet->length;
        DPRINTF(EthernetDesc, "LEGACY Packet Length: %d Desc Size: %d\n",
                packet->length, igbe->regs.rctl.descSize());
        DPRINTF(EthernetDpdk, "RXD[%d] LEGACY Packet Length: %d Desc Size: %d\n",
                queueID, packet->length, igbe->regs.rctl.descSize());
        assert(packet->length < igbe->regs.rctl.descSize());
        igbe->dmaWrite(pciToDma(desc->legacy.buf),
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
        // igbe->dmaWrite(pciToDma(desc->adv_read.pkt),
        //                packet->length, &pktEvent, packet->data,
        //                igbe->rxWriteDelay);
        // printf("RXD[%d] At clk: %ld IdioWrite pktPtr: %p, Addr: %lx, Size: %d, Delay: %ld\n",
        //         queueID, curTick(), pktPtr, pciToDma(desc->adv_read.pkt),
        //         packet->length, igbe->rxWriteDelay);
        igbe->IdioWrite(pciToDma(desc->adv_read.pkt),
                       packet->length, &pktEvent, packet->data,
                       igbe->rxWriteDelay, 0, igbe->adq);

        desc->adv_wb.header_len = htole(0);
        desc->adv_wb.sph = htole(0);
        desc->adv_wb.pkt_len = htole((uint16_t)(pktPtr->length));
        break;
      case RXDT_ADV_SPLIT_A:
        int split_point;

        buf_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl_array[queueID].bufLen() :
            igbe->regs.rctl.descSize();
        hdr_len = igbe->regs.rctl.lpe() ? igbe->regs.srrctl_array[queueID].hdrLen() : 0;
        DPRINTF(EthernetDesc,
                "lpe: %d Packet Length: %d offset: %d srrctl: %#x "
                "hdr addr: %#x Hdr Size: %d desc addr: %#x Desc Size: %d\n",
                igbe->regs.rctl.lpe(), packet->length, pkt_offset,
                igbe->regs.srrctl_array[queueID](), desc->adv_read.hdr, hdr_len,
                desc->adv_read.pkt, buf_len);
        DPRINTF(EthernetDpdk,
                "RXD[%d] lpe: %d Packet Length: %d offset: %d srrctl: %#x "
                "hdr addr: %#x Hdr Size: %d desc addr: %#x Desc Size: %d\n",
                queueID, igbe->regs.rctl.lpe(), packet->length, pkt_offset,
                igbe->regs.srrctl_array[queueID](), desc->adv_read.hdr, hdr_len,
                desc->adv_read.pkt, buf_len);

        split_point = hsplit(pktPtr);

        if (packet->length <= hdr_len) {
            bytesCopied = packet->length;
            assert(pkt_offset == 0);
            DPRINTF(EthernetDesc, "Hdr split: Entire packet in header\n");
            // SHIN. Change To IDIO
            // igbe->dmaWrite(pciToDma(desc->adv_read.hdr),
            //                packet->length, &pktEvent, packet->data,
            //                igbe->rxWriteDelay);
            igbe->IdioWrite(pciToDma(desc->adv_read.hdr),
                           packet->length, &pktEvent, packet->data,
                           igbe->rxWriteDelay, 0, igbe->adq);

            desc->adv_wb.header_len = htole((uint16_t)packet->length);
            desc->adv_wb.sph = htole(0);
            desc->adv_wb.pkt_len = htole(0);
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
                // igbe->dmaWrite(pciToDma(desc->adv_read.pkt),
                //                max_to_copy, &pktEvent,
                //                packet->data + pkt_offset, igbe->rxWriteDelay);

                igbe->IdioWrite(pciToDma(desc->adv_read.pkt),
                               max_to_copy, &pktEvent,
                               packet->data + pkt_offset, igbe->rxWriteDelay,
                               0, igbe->adq);

                desc->adv_wb.header_len = htole(0);
                desc->adv_wb.pkt_len = htole((uint16_t)max_to_copy);
                desc->adv_wb.sph = htole(0);
            } else {
                int max_to_copy =
                    std::min(packet->length - split_point, buf_len);
                bytesCopied += max_to_copy + split_point;

                DPRINTF(EthernetDesc, "Hdr split: splitting at %d\n",
                        split_point);
                // SHIN
                // igbe->dmaWrite(pciToDma(desc->adv_read.hdr),
                //                split_point, &pktHdrEvent,
                //                packet->data, igbe->rxWriteDelay);
                // igbe->dmaWrite(pciToDma(desc->adv_read.pkt),
                //                max_to_copy, &pktDataEvent,
                //                packet->data + split_point, igbe->rxWriteDelay);
                igbe->IdioWrite(pciToDma(desc->adv_read.hdr),
                               split_point, &pktHdrEvent,
                               packet->data, igbe->rxWriteDelay,
                               0, igbe->adq);
                igbe->IdioWrite(pciToDma(desc->adv_read.pkt),
                               max_to_copy, &pktDataEvent,
                               packet->data + split_point, igbe->rxWriteDelay,
                               0, igbe->adq);
                desc->adv_wb.header_len = htole(split_point);
                desc->adv_wb.sph = 1;
                desc->adv_wb.pkt_len = htole((uint16_t)(max_to_copy));
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
    return bytesCopied;

}

void
IGbE::RxDescCache::pktComplete()
{
    assert(unusedCache.size());
    RxDesc *desc;
    desc = unusedCache.front();

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
        desc->legacy.len = htole((uint16_t)(pktPtr->length + crcfixup));
        desc->legacy.status = htole(status);
        desc->legacy.errors = htole(err);
        // No vlan support at this point... just set it to 0
        desc->legacy.vlan = 0;
        DPRINTF(EthernetDesc, "RXD[%d] Descriptor LEGACY complete len: %#x status: %#x\n",
                queueID, desc->legacy.len, desc->legacy.status);
        break;
      case RXDT_ADV_SPLIT_A:
      case RXDT_ADV_ONEBUF:
        desc->adv_wb.rss_type = htole(0);
        desc->adv_wb.pkt_type = htole(ptype);
        if (igbe->regs.rxcsum.pcsd()) {
            // no rss support right now
            desc->adv_wb.rss_hash = htole(0);
        } else {
            desc->adv_wb.id = htole(ip_id);
            desc->adv_wb.csum = htole(csum);
        }
        desc->adv_wb.status = htole(status);
        desc->adv_wb.errors = htole(ext_err);
        // no vlan support
        desc->adv_wb.vlan_tag = htole(0);
        break;
      default:
        panic("Unimplemnted RX receive buffer type %d\n", queueID,
              igbe->regs.srrctl_array[queueID].desctype());
    }

    DPRINTF(EthernetDesc, "RXD[%d] Descriptor complete w0: %#x w1: %#x\n",
            queueID, desc->adv_read.pkt, desc->adv_read.hdr);

    if (bytesCopied == pktPtr->length) {
        DPRINTF(EthernetDesc,
                "RXD[%d] Packet completely written to descriptor buffers\n", queueID);
        DPRINTF(EthernetDpdk,
                "RXD[%d] Packet completely written to descriptor buffers\n", queueID);    
        // Deal with the rx timer interrupts
        if (igbe->regs.rdtr.delay()) {
            Tick delay = igbe->regs.rdtr.delay() * igbe->intClock();
            DPRINTF(EthernetSM, "RXS[%d]: Scheduling DTR for %d\n", queueID, delay);
            // igbe->reschedule(igbe->rdtrEvent, curTick() + delay);
            igbe->reschedule(_rdtrEvent, curTick() + delay);
        }

        if (igbe->regs.radv.idv()) {
            Tick delay = igbe->regs.radv.idv() * igbe->intClock();
            DPRINTF(EthernetSM, "RXS[%d]: Scheduling ADV for %d\n", queueID, delay);
            // if (!igbe->radvEvent.scheduled()) {
            //     igbe->schedule(igbe->radvEvent, curTick() + delay);
            // }
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
        bytesCopied = 0;
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
            // igbe->etherDeviceStats.rxEnd2EndClk.sample(rxTotalTimeInSec);
            // igbe->etherDeviceStats.rxEtherLinkClk.sample(rxEtherLinkTimeInSec);
            // igbe->etherDeviceStats.rxPort2FifoClk.sample(rxPort2FifoTimeInSec);
            // igbe->etherDeviceStats.rxFifo2DmaClk.sample(rxFifo2DMAStartTimeInSec);
            // igbe->etherDeviceStats.rxDma2CoreClk.sample(rxDMATimeInSec);
        }
    }

    pktPtr = NULL;
    igbe->checkDrain();
    enableSm();
    pktDone = true;

    DPRINTF(EthernetDesc, "RXD[%d] Processing of this descriptor complete\n", queueID);
    DPRINTF(EthernetDpdk, "RXD[%d] Processing of this descriptor complete\n", queueID);
    unusedCache.pop_front();
    usedCache.push_back(desc);
}

void
IGbE::RxDescCache::enableSm()
{
    if (igbe->drainState() != DrainState::Draining) {
        igbe->rxTick = true;
        igbe->restartClock();
    }
}

bool
IGbE::RxDescCache::packetDone()
{
    if (pktDone) {
        // pktDone = false;
        return true;
    }
    return false;
}

bool
IGbE::RxDescCache::hasOutstandingEvents()
{
    return pktEvent.scheduled() || wbEvent.scheduled() ||
        fetchEvent.scheduled() || pktHdrEvent.scheduled() ||
        pktDataEvent.scheduled();

}

void
IGbE::RxDescCache::serialize(CheckpointOut &cp) const
{
    DescCache<RxDesc>::serialize(cp);
    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(splitCount);
    SERIALIZE_SCALAR(bytesCopied);

    Tick _rdtr_time = 0, _radv_time = 0;

    if (_rdtrEvent.scheduled())
        _rdtr_time = _rdtrEvent.when();
    SERIALIZE_SCALAR(_rdtr_time);
    if (_radvEvent.scheduled())
        _radv_time = _radvEvent.when();
    SERIALIZE_SCALAR(_radv_time);
}

void
IGbE::RxDescCache::unserialize(CheckpointIn &cp)
{
    DescCache<RxDesc>::unserialize(cp);
    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(splitCount);
    UNSERIALIZE_SCALAR(bytesCopied);

    Tick _rdtr_time = 0, _radv_time = 0;
    UNSERIALIZE_SCALAR(_rdtr_time);
    UNSERIALIZE_SCALAR(_radv_time);
    if (_rdtr_time)
        igbe->schedule(_rdtrEvent, _rdtr_time);
    if (_radv_time)
        igbe->schedule(_radvEvent, _radv_time);
}


///////////////////////////// IGbE::TxDescCache //////////////////////////////

IGbE::TxDescCache::TxDescCache(IGbE *i, const std::string n, int s, int qid)
    : DescCache<TxDesc>(i,n, s, false), pktDone(false), isTcp(false),
      pktWaiting(false), pktMultiDesc(false),
      completionAddress(0), completionEnabled(false),
      useTso(false), tsoHeaderLen(0), tsoMss(0), tsoTotalLen(0), tsoUsedLen(0),
      tsoPrevSeq(0), tsoPktPayloadBytes(0), tsoLoadedHeader(false),
      tsoPktHasHeader(false), tsoDescBytesUsed(0), tsoCopyBytes(0), tsoPkts(0), queueID(qid),
    pktEvent([this]{ pktComplete(); }, n),
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
}

void
IGbE::TxDescCache::processContextDesc()
{
    assert(unusedCache.size());
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
        pktWaiting = true;
        assert(tsoHeaderLen <= 256);
        igbe->dmaRead(pciToDma(txd_op::getBuf(desc)),
                      tsoHeaderLen, &headerEvent, tsoHeader, 0);
    }
}

void
IGbE::TxDescCache::headerComplete()
{
    DPRINTF(EthernetDesc, "TXD[%d] TSO: Fetching TSO header complete\n", queueID);
    pktWaiting = false;

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
IGbE::TxDescCache::getPacketSize(EthPacketPtr p)
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

        if (tsoPktHasHeader)
            tsoCopyBytes =  std::min((tsoMss + tsoHeaderLen) - p->length,
                                     txd_op::getLen(desc) - tsoDescBytesUsed);
        else
            tsoCopyBytes =  std::min(tsoMss,
                                     txd_op::getLen(desc) - tsoDescBytesUsed);
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
IGbE::TxDescCache::getPacketData(EthPacketPtr p)
{
    assert(unusedCache.size());

    TxDesc *desc;
    desc = unusedCache.front();

    DPRINTF(EthernetDesc, "TXD[%d] getPacketData(): TxDescriptor data "
            "d1: %#llx d2: %#llx\n", queueID, desc->d1, desc->d2);
    assert((txd_op::isLegacy(desc) || txd_op::isData(desc)) &&
           txd_op::getLen(desc));

    pktPtr = p;

    pktWaiting = true;

    DPRINTF(EthernetDesc, "TXD[%d] Starting DMA of packet at offset %d\n", queueID, p->length);

    if (useTso) {
        assert(tsoLoadedHeader);
        if (!tsoPktHasHeader) {
            DPRINTF(EthernetDesc,
                    "TXD[%d] Loading TSO header (%d bytes) into start of packet\n", queueID,
                    tsoHeaderLen);
            memcpy(p->data, &tsoHeader,tsoHeaderLen);
            p->length +=tsoHeaderLen;
            tsoPktHasHeader = true;
        }
    }

    if (useTso) {
        DPRINTF(EthernetDesc,
                "TXD[%d] Starting DMA of packet at offset %d length: %d\n", queueID,
                p->length, tsoCopyBytes);
        // printf("TXD[%d] At clk: %ld DmaRead pktPtr: %p, Addr: %lx, Size: %d, Delay: %ld\n", 
        //         queueID, curTick(), p, pciToDma(txd_op::getBuf(desc)) + tsoDescBytesUsed, tsoCopyBytes, igbe->txReadDelay);

        igbe->dmaRead(pciToDma(txd_op::getBuf(desc))
                      + tsoDescBytesUsed,
                      tsoCopyBytes, &pktEvent, p->data + p->length,
                      igbe->txReadDelay);
        tsoDescBytesUsed += tsoCopyBytes;
        assert(tsoDescBytesUsed <= txd_op::getLen(desc));
    } else {
        // printf("TXD[%d] At clk: %ld DmaRead pktPtr: %p, Addr: %lx, Size: %d, Delay: %ld\n", 
        //         queueID, curTick(), p, pciToDma(txd_op::getBuf(desc)), txd_op::getLen(desc), igbe->txReadDelay);
        igbe->dmaRead(pciToDma(txd_op::getBuf(desc)),
                      txd_op::getLen(desc), &pktEvent, p->data + p->length,
                      igbe->txReadDelay);
    }
}

void
IGbE::TxDescCache::pktComplete()
{

    TxDesc *desc;
    assert(unusedCache.size());
    assert(pktPtr);

    DPRINTF(EthernetDesc, "TXD[%d] DMA of packet complete\n", queueID);
    DPRINTF(EthernetDpdk, "TXD[%d] DMA of packet complete\n", queueID);

    // printf("TXD[%d] At clk: %ld pktComplete() pktPtr: %p\n", queueID, curTick(), pktPtr);


    desc = unusedCache.front();
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
        unusedCache.pop_front();
        usedCache.push_back(desc);

        tsoDescBytesUsed = 0;
        pktDone = true;
        pktWaiting = false;
        pktMultiDesc = true;

        DPRINTF(EthernetDesc, "TXD[%d] Partial Packet Descriptor of %d bytes Done\n", queueID,
                pktPtr->length);
        pktPtr = NULL;

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
        unusedCache.pop_front();
        usedCache.push_back(desc);
        tsoDescBytesUsed = 0;
    }

    if (useTso && tsoUsedLen == tsoTotalLen)
        useTso = false;


    DPRINTF(EthernetDesc,
            "TXD[%d] ------Packet of %d bytes ready for transmission-------\n", queueID,
            pktPtr->length);
    DPRINTF(EthernetDpdk,
            "TXD[%d] ------Packet of %d bytes ready for transmission-------\n", queueID,
            pktPtr->length);
    
    igbe->etherDeviceStats.txDMABytes += pktPtr->length;

    pktDone = true;
    pktWaiting = false;
    pktPtr = NULL;
    tsoPktHasHeader = false;

    if (igbe->regs.txdctl_array[queueID].wthresh() == 0) {
        DPRINTF(EthernetDesc, "TXD[%d] WTHRESH == 0, writing back descriptor\n", queueID);
        writeback(0);
    } else if (!igbe->regs.txdctl_array[queueID].gran() && igbe->regs.txdctl_array[queueID].wthresh() <=
               descInBlock(usedCache.size())) {
        DPRINTF(EthernetDesc, "TXD[%d] used > WTHRESH, writing back descriptor\n", queueID);
        writeback((igbe->cacheBlockSize()-1)>>4);
    } else if (igbe->regs.txdctl_array[queueID].wthresh() <= usedCache.size()) {
        DPRINTF(EthernetDesc, "TXD[%d] used > WTHRESH, writing back descriptor\n", queueID);
        writeback((igbe->cacheBlockSize()-1)>>4);
    }

    enableSm();
    igbe->checkDrain();
}

void
IGbE::TxDescCache::actionAfterWb()
{
    DPRINTF(EthernetDesc, "TXD[%d] actionAfterWb() completionEnabled: %d\n", queueID,
            completionEnabled);
    igbe->postInterrupt(igbreg::IT_TXDW);
    if (completionEnabled) {
        // descEnd = igbe->regs.tdh();
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
IGbE::TxDescCache::serialize(CheckpointOut &cp) const
{
    DescCache<TxDesc>::serialize(cp);

    SERIALIZE_SCALAR(pktDone);
    SERIALIZE_SCALAR(isTcp);
    SERIALIZE_SCALAR(pktWaiting);
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
IGbE::TxDescCache::unserialize(CheckpointIn &cp)
{
    DescCache<TxDesc>::unserialize(cp);

    UNSERIALIZE_SCALAR(pktDone);
    UNSERIALIZE_SCALAR(isTcp);
    UNSERIALIZE_SCALAR(pktWaiting);
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

bool
IGbE::TxDescCache::packetAvailable()
{
    if (pktDone) {
        // pktDone = false; -> it is set through unsetPacketDone()
        return true;
    }
    return false;
}

void
IGbE::TxDescCache::enableSm()
{
    if (igbe->drainState() != DrainState::Draining) {
        igbe->txTick = true;
        igbe->restartClock();
    }
}

bool
IGbE::TxDescCache::hasOutstandingEvents()
{
    return pktEvent.scheduled() || wbEvent.scheduled() ||
        fetchEvent.scheduled();
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
            if (!m2funcRxFifo.empty()) {
                DPRINTF(EthernetDpdk, "RXM2func[%d]: Processing CXL request. Because m2funcRxFifo is not empty\n", queueID);
                PacketPtr cxlReq = popCXLReqBuf();
                readM2funcPacket(cxlReq);
                DPRINTF(EthernetDpdk, "RXM2func[%d]: CXL request processed & Make a response using data in the m2funcRxFifo. Pop out from m2funcCXLReqBuf and return to DTA\n", queueID);
                DPRINTF(EthernetDpdk, "Current free CXLReqBuf: %ld, numCXLReq: %ld\n", numFreeCXLReq, numCXLReq);
                sendCXLResp(cxlReq);
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
}

void 
IGbE::enableSmTx() 
{
    if (drainState() != DrainState::Draining) {
        txTick = true;
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
    if (txPacketArray[queueID] && txDescCacheArray[queueID]->packetAvailable()
        && !txDescCacheArray[queueID]->packetMultiDesc() && txPacketArray[queueID]->length) {
        if (candidateTxQueue != queueID) {
            DPRINTF(EthernetSM, "TXD[%d]: Packet available to push FIFO, but not for this queue, candidate is %d\n", queueID, candidateTxQueue);
            DPRINTF(EthernetDpdk, "TXD[%d]: Packet available to push FIFO, but not for this queue, candidate is %d\n", queueID, candidateTxQueue);

            return txTickQueue;
        } else {
            DPRINTF(EthernetSM, "TXD[%d]: packet placed in TX FIFO\n", queueID);
            DPRINTF(EthernetDpdk, "TXD[%d]: packet placed in TX FIFO\n", queueID);
            bool success =
                // txFifo.push(txPacket);
                txFifo.push(txPacketArray[queueID]);
            txFifoTick = true && drainState() != DrainState::Draining;
            assert(success);
            // txPacket = NULL;
            txPacketArray[queueID] = nullptr;
            // txDescCache.writeback((cacheBlockSize()-1)>>4);
            txDescCacheArray[queueID]->writeback((cacheBlockSize()-1)>>4);
            // return;
            // set successTxQueueSend to true
            successTxQueueSend = true;
            // unset pktDone
            txDescCacheArray[queueID]->unsetPacketDone();
            return txTickQueue;
        }
    }

    // Only support descriptor granularity
    // if (regs.txdctl.lwthresh() &&
    //     txDescCache.descLeft() < (regs.txdctl.lwthresh() * 8)) {
    if (regs.txdctl_array[queueID].lwthresh() &&
        txDescCacheArray[queueID]->descLeft() < (regs.txdctl_array[queueID].lwthresh() * 8)) {
        DPRINTF(EthernetSM, "TXD[%d]: LWTHRESH caused posting of TXDLOW\n", queueID);
        postInterrupt(IT_TXDLOW);
    }

    // if (!txPacket) {
    //     txPacket = std::make_shared<EthPacketData>(16384);
    // }
    if (!txPacketArray[queueID]) {
        txPacketArray[queueID] = std::make_shared<EthPacketData>(16384);
    }

    // if (!txDescCache.packetWaiting()) {
    if (!txDescCacheArray[queueID]->packetWaiting()) {
        // if (txDescCache.descLeft() == 0) {
        if (txDescCacheArray[queueID]->descLeft() == 0) {
            etherDeviceStats.txRingBufferFull++; //TODO - jm: make stat as array
            postInterrupt(IT_TXQE);
            // txDescCache.writeback(0);
            // txDescCache.fetchDescriptors();
            txDescCacheArray[queueID]->writeback(0);
            txDescCacheArray[queueID]->fetchDescriptors();
            DPRINTF(EthernetSM, "TXD[%d]: No descriptors left in ring, forcing "
                    "writeback stopping ticking and posting TXQE\n", queueID);
            DPRINTF(EthernetDpdk, "TXD[%d]: No descriptors left in ring, forcing "
                    "writeback stopping ticking and posting TXQE\n", queueID);
            // txTick = false;
            txTickQueue = false;
            // return;
            return txTickQueue;
        }


        // if (!(txDescCache.descUnused())) {
        //     txDescCache.fetchDescriptors();
        if (!(txDescCacheArray[queueID]->descUnused())) {
            txDescCacheArray[queueID]->fetchDescriptors();
            etherDeviceStats.txDescCacheFullCount++; //TODO - jm: make stat as array
            DPRINTF(EthernetSM, "TXD[%d]: No descriptors available in cache, "
                    "fetching and stopping ticking\n", queueID);
            DPRINTF(EthernetDpdk, "TXD[%d]: No descriptors available in cache, "
                    "fetching and stopping ticking\n", queueID);
            // txTick = false;
            // return;
            txTickQueue = false;
            return txTickQueue;
        }


        // txDescCache.processContextDesc();
        txDescCacheArray[queueID]->processContextDesc();
        // if (txDescCache.packetWaiting()) {
        if (txDescCacheArray[queueID]->packetWaiting()) {
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
        unsigned size = txDescCacheArray[queueID]->getPacketSize(txPacketArray[queueID]);
        if (size > 0 && txFifo.avail() > size) {
            DPRINTF(EthernetSM, "TXD[%d]: Reserving %d bytes in FIFO and "
                    "beginning DMA of next packet\n", queueID, size);
            DPRINTF(EthernetDpdk, "TXD[%d]: Reserving %d bytes in FIFO and "
                    "beginning DMA of next packet\n", queueID, size);
            txFifo.reserve(size);
            // txDescCache.getPacketData(txPacket);
            txDescCacheArray[queueID]->getPacketData(txPacketArray[queueID]);
        } else if (size == 0) {
            DPRINTF(EthernetSM, "TXD[%d]: getPacketSize returned: %d\n", queueID, size);
            DPRINTF(EthernetSM,
                    "TXD[%d]: No packets to get, writing back used descriptors\n", queueID);
            DPRINTF(EthernetDpdk, "TXD[%d]: getPacketSize returned: %d\n", queueID, size);
            DPRINTF(EthernetDpdk,
                    "TXD[%d]: No packets to get, writing back used descriptors\n", queueID);
            // txDescCache.writeback(0);
            txDescCacheArray[queueID]->writeback(0);
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
            if (txPacketArray[queueID] && txDescCacheArray[queueID]->packetAvailable()
                && !txDescCacheArray[queueID]->packetMultiDesc() && txPacketArray[queueID]->length) {
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

bool
IGbE::ethRxPkt(EthPacketPtr pkt)
{
    
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
            if (txDescCacheArray[i]->packetWaiting() || txDescCacheArray[i]->descLeft() < 1024) {
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
        // rxTick = false;
        rxTickQueue = false;
        DPRINTF(EthernetSM, "RXS[%d]: RX disabled, stopping ticking\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: RX disabled, stopping ticking\n", queueID);
        return rxTickQueue;
    }
    // If the packet is done check for interrupts/descriptors/etc
    // if (rxDescCache.packetDone()) {
    if (rxDescCacheArray[queueID]->packetDone()) {
        // rxDmaPacket = false;
        rxDmaPacketArray[queueID] = false;
        rxDescCacheArray[queueID]->unsetPacketDone();

        DPRINTF(EthernetSM, "RXS[%d]: Packet completed DMA to memory\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: Packet completed DMA to memory\n", queueID);
        // int descLeft = rxDescCache.descLeft();
        int descLeft = rxDescCacheArray[queueID]->descLeft();
        DPRINTF(EthernetSM, "RXS[%d]: descLeft: %d rdmts: %d rdlen: %d\n", queueID,
                descLeft, regs.rctl.rdmts(), regs.rdlen_array[queueID]()); //regs.rdlen());
        DPRINTF(EthernetDpdk, "RXS[%d]: descLeft: %d rdmts: %d rdlen: %d\n", queueID,
                descLeft, regs.rctl.rdmts(), regs.rdlen_array[queueID]()); //regs.rdlen());

        // rdmts 2->1/8, 1->1/4, 0->1/2
        int ratio = (1ULL << (regs.rctl.rdmts() + 1));
        // if (descLeft * ratio <= regs.rdlen()) {
        if (descLeft * ratio <= regs.rdlen_array[queueID]()) {
            DPRINTF(Ethernet, "RXS[%d]: Interrupting (RXDMT) "
                    "because of descriptors left\n", queueID);
            DPRINTF(EthernetDpdk, "RXS[%d]: Interrupting (RXDMT) "
                    "because of descriptors left\n", queueID);
            // rxDescCache.writeback(0);
            rxDescCacheArray[queueID]->writeback(0);
         }

        if (descLeft < 32)
        {
            // rxDescCache.writeback(0);
            rxDescCacheArray[queueID]->writeback(0);
        }

        if (rxFifo.empty()) {
            // rxDescCache.writeback(0);
            rxDescCacheArray[queueID]->writeback(0);
        }

        if (descLeft == 0) { 
            etherDeviceStats.rxRingBufferFull++; //TODO - jm : make this as array 
            // rxDescCache.writeback(0);
            rxDescCacheArray[queueID]->writeback(0);
            DPRINTF(EthernetSM, "RXS[%d]: No descriptors left in ring, forcing"
                    " writeback and stopping ticking\n", queueID);
            DPRINTF(EthernetDpdk, "RXS[%d]: No descriptors left in ring, forcing"
                    " writeback and stopping ticking\n", queueID);
            // rxTick = false;
            rxTickQueue = false;
        }

        // only support descriptor granulaties
        // assert(regs.rxdctl.gran());
        assert(regs.rxdctl_array[queueID].gran());

        // if (regs.rxdctl.wthresh() >= rxDescCache.descUsed()) {
        if (regs.rxdctl_array[queueID].wthresh() >= rxDescCacheArray[queueID]->descUsed()) {
            DPRINTF(EthernetSM,
                    "RXS[%d]: Writing back because WTHRESH >= descUsed\n", queueID);
            DPRINTF(EthernetDpdk,
                    "RXS[%d]: Writing back because WTHRESH >= descUsed\n", queueID);
            // if (regs.rxdctl.wthresh() < (cacheBlockSize()>>4))
            //     rxDescCache.writeback(regs.rxdctl.wthresh()-1);
            // else
            //     rxDescCache.writeback((cacheBlockSize()-1)>>4);
            if (regs.rxdctl_array[queueID].wthresh() < (cacheBlockSize()>>4))
                rxDescCacheArray[queueID]->writeback(regs.rxdctl_array[queueID].wthresh()-1);
            else
                rxDescCacheArray[queueID]->writeback((cacheBlockSize()-1)>>4);
           
        }

        // if ((rxDescCache.descUnused() < regs.rxdctl.pthresh()) &&
        //     ((rxDescCache.descLeft() - rxDescCache.descUnused()) >
        //      regs.rxdctl.hthresh())) {
        //     DPRINTF(EthernetSM, "RXS: Fetching descriptors because "
        //             "descUnused < PTHRESH\n");
        //     DPRINTF(EthernetDpdk, "RXS: Fetching descriptors because "
        //             "descUnused < PTHRESH\n");
        //     rxDescCache.fetchDescriptors();
        // }
        if ((rxDescCacheArray[queueID]->descUnused() < regs.rxdctl_array[queueID].pthresh()) &&
            ((rxDescCacheArray[queueID]->descLeft() - rxDescCacheArray[queueID]->descUnused()) >
             regs.rxdctl_array[queueID].hthresh())) {
            DPRINTF(EthernetSM, "RXS[%d]: Fetching descriptors because "
                    "descUnused < PTHRESH\n", queueID);
            DPRINTF(EthernetDpdk, "RXS[%d]: Fetching descriptors because "
                    "descUnused < PTHRESH\n", queueID);
            rxDescCacheArray[queueID]->fetchDescriptors();
        }
        
        // if (rxDescCache.descUnused() == 0) {
            // rxDescCache.fetchDescriptors();
        if (rxDescCacheArray[queueID]->descUnused() == 0) {
            rxDescCacheArray[queueID]->fetchDescriptors();
            etherDeviceStats.rxDescCacheFullCount++;
            DPRINTF(EthernetSM, "RXS[%d]: No descriptors available in cache, "
                    "fetching descriptors and stopping ticking\n", queueID);
            DPRINTF(EthernetDpdk, "RXS[%d]: No descriptors available in cache, "
                    "fetching descriptors and stopping ticking\n", queueID);
            // rxTick = false;
            rxTickQueue = false;
        }
        // return;
        return rxTickQueue;
    }

    // if (rxDmaPacket) {
    if (rxDmaPacketArray[queueID]) {
        DPRINTF(EthernetSM,
                "RXS[%d]: stopping ticking until packet DMA completes\n", queueID);
        DPRINTF(EthernetDpdk,
                "RXS[%d]: stopping ticking until packet DMA completes\n", queueID);
        // rxTick = false;
        // return;
        rxTickQueue = false;
        if (rxPacketArray[queueID] != nullptr) {
            if (rxPacketArray[queueID]->rxFifoNotEmptyDmaBusyChecked == false) {
                etherDeviceStats.rxFifoNotEmptyDmaBusy++;
                rxPacketArray[queueID]->rxFifoNotEmptyDmaBusyChecked = true;
            }
        }
        return rxTickQueue;
    }

    // if (!rxDescCache.descUnused()) {
    //     rxDescCache.fetchDescriptors();
    if (!rxDescCacheArray[queueID]->descUnused()) {
        rxDescCacheArray[queueID]->fetchDescriptors();
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

    // if (rxFifo.empty()) {
    //     DPRINTF(EthernetSM, "RXS: RxFIFO empty, stopping ticking\n");
    //     DPRINTF(EthernetDpdk, "RXS: RxFIFO empty, stopping ticking\n");
    //     rxTick = false;
    //     return;
    // }

    // EthPacketPtr pkt;
    // pkt = rxFifo.front();
    
    // pktOffset = rxDescCache.writePacket(pkt, pktOffset);
    pktOffsetArray[queueID] = rxDescCacheArray[queueID]->writePacket(pkt, pktOffsetArray[queueID]);
    DPRINTF(EthernetSM, "RXS[%d]: Writing packet into memory\n", queueID);
    DPRINTF(EthernetDpdk, "RXS[%d]: Writing packet into memory\n", queueID);
    // if (pktOffset == pkt->length) {
    if (pktOffsetArray[queueID] == pkt->length) {
        DPRINTF(EthernetSM, "RXS[%d]: Removing packet from FIFO\n", queueID);
        DPRINTF(EthernetDpdk, "RXS[%d]: Removing packet from FIFO\n", queueID);
        // pktOffset = 0;
        pktOffsetArray[queueID] = 0;
        // rxFifo.pop();
        rxPacketArray[queueID] = nullptr;

        // Stat
        updateRxRingBufferDMAStartStat(curTick());
    }

    DPRINTF(EthernetSM, "RXS[%d]: stopping ticking until packet DMA completes\n", queueID);
    DPRINTF(EthernetDpdk, "RXS[%d]: stopping ticking until packet DMA completes\n", queueID);
    // rxTick = false;
    rxTickQueue = false;
    // rxDmaPacket = true;
    rxDmaPacketArray[queueID] = true;
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

        etherDeviceStats.txBytes += txFifo.front()->length;
        etherDeviceStats.txPackets++;

        txFifo.pop();
    }
}

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

} // namespace gem5
