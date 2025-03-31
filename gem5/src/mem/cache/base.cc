/*
 * Copyright (c) 2012-2013, 2018-2019 ARM Limited
 * All rights reserved.
 *
 * The license below extends only to copyright in the software and shall
 * not be construed as granting a license to any other intellectual
 * property including but not limited to intellectual property relating
 * to a hardware implementation of the functionality of the software
 * licensed hereunder.  You may use the software subject to the license
 * terms below provided that you ensure that this notice is replicated
 * unmodified and in its entirety in all distributions of the software,
 * modified or unmodified, in source code or in binary form.
 *
 * Copyright (c) 2003-2005 The Regents of The University of Michigan
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
 * Definition of BaseCache functions.
 */

#include "mem/cache/base.hh"

#include "base/compiler.hh"
#include "base/logging.hh"
#include "debug/Cache.hh"
#include "debug/CacheComp.hh"
#include "debug/CachePort.hh"
#include "debug/CacheRepl.hh"
#include "debug/CacheVerbose.hh"
#include "debug/HWPrefetch.hh"
#include "debug/Cache.hh"
#include "mem/cache/compressors/base.hh"
#include "mem/cache/mshr.hh"
#include "mem/cache/prefetch/base.hh"
#include "mem/cache/queue_entry.hh"
#include "mem/cache/tags/compressed_tags.hh"
#include "mem/cache/tags/super_blk.hh"
#include "params/BaseCache.hh"
#include "params/WriteAllocator.hh"
#include "params/DTA.hh"
#include "sim/cur_tick.hh"

// SHIN. debug IDIO
#include "debug/AdaptiveDdioOtf.hh"
#include "debug/AdaptiveDdioCache.hh"
#include "base.hh"

// LOG_LEVEL: 0 - no log, 1 - ring buffer log, 2 - dta log, 3 - dta double buffer log, 4 - dta log prepare
#define LOG_LEVEL 0


namespace gem5
{

BaseCache::CacheResponsePort::CacheResponsePort(const std::string &_name,
                                          BaseCache *_cache,
                                          const std::string &_label)
    : QueuedResponsePort(_name, _cache, queue),
      queue(*_cache, *this, true, _label),
      blocked(false), mustSendRetry(false),
      sendRetryEvent([this]{ processSendRetry(); }, _name)
{
}

BaseCache::BaseCache(const BaseCacheParams &p, unsigned blk_size)
    : ClockedObject(p),
      mlc_idx(p.mlc_idx), isMLC(p.is_mlc), isIOCache(p.is_iocache), send_header_only(p.send_header_only), 
      enableDTA(p.enable_dta), // TODO - JM
      DTARXJobAddr(p.dta_rx_job_addr), // TODO - JM
      DTATXJobAddr(p.dta_tx_job_addr), // TODO - JM
      dta(p.dta),
      cpuSidePort (p.name + ".cpu_side_port", this, "CpuSidePort"),
      memSidePort(p.name + ".mem_side_port", this, "MemSidePort"),
      mshrQueue("MSHRs", p.mshrs, 0, p.demand_mshr_reserve, p.name),
      writeBuffer("write buffer", p.write_buffers, p.mshrs, p.name),
      tags(p.tags),
      compressor(p.compressor),
      prefetcher(p.prefetcher),
      writeAllocator(p.write_allocator),
      writebackClean(p.writeback_clean),
      tempBlockWriteback(nullptr),
      writebackTempBlockAtomicEvent([this]{ writebackTempBlockAtomic(); },
                                    name(), false,
                                    EventBase::Delayed_Writeback_Pri),
      blkSize(blk_size),
      lookupLatency(p.tag_latency),
      dataLatency(p.data_latency),
      forwardLatency(p.tag_latency),
      fillLatency(p.data_latency),
      responseLatency(p.response_latency),
      sequentialAccess(p.sequential_access),
      numTarget(p.tgts_per_mshr),
      forwardSnoops(true),
      clusivity(p.clusivity),
      isReadOnly(p.is_read_only),
      replaceExpansions(p.replace_expansions),
      moveContractions(p.move_contractions),
      blocked(0),
      order(0),
      noTargetMSHR(nullptr),
      missCount(p.max_miss_count),
      addrRanges(p.addr_ranges.begin(), p.addr_ranges.end()),
      system(p.system),
      stats(*this),
      ddioEnabled(p.ddio_enabled), ddioDisabled(p.ddio_disabled),
      ddioWayPart(p.ddio_way_part),
      isLLC(p.is_llc), mlc_ddio(p.mlc_ddio), isMultiPort(p.is_multiport)
{
    // the MSHR queue has no reserve entries as we check the MSHR
    // queue on every single allocation, whereas the write queue has
    // as many reserve entries as we have MSHRs, since every MSHR may
    // eventually require a writeback, and we do not check the write
    // buffer before committing to an MSHR

    // forward snoops is overridden in init() once we can query
    // whether the connected requestor is actually snooping or not

    tempBlock = new TempCacheBlk(blkSize);

    tags->tagsInit();
    if (prefetcher)
        prefetcher->setCache(this);

    fatal_if(compressor && !dynamic_cast<CompressedTags*>(tags),
        "The tags of compressed cache %s must derive from CompressedTags",
        name());
    warn_if(!compressor && dynamic_cast<CompressedTags*>(tags),
        "Compressed cache %s does not have a compression algorithm", name());
    if (compressor)
        compressor->setCache(this);

    if (isLLC && enableDTA) {
        assert(isMultiPort);
    }
    
    if (isLLC && isMultiPort) {
        for (int i = 0; i < p.cpu_side_ports_connection_count; ++i) {
            std::string portName = csprintf("%s.cpu_side_port[%d]", name(), i);
            cpuSidePortList.emplace_back(new CpuSidePort(
                portName, this, "CpuSidePort", i));

        }
        // JM. split the address ranges for each port - split evenly
        int num_ports = cpuSidePortList.size();
        int num_ranges = addrRanges.size();
        if (num_ranges == 1) {
            // split the existing addrRange into num_ports
            AddrRange range = *addrRanges.begin();
            Addr start = range.start();
            Addr end = range.end();
            Addr size = range.size();
            Addr port_size = size / num_ports;
            Addr accum_size = 0;
            
            // JM - if enable DTA, we have to delete the range of DTARXJobAddr, DTATXJobAddr from the addrRanges
            if (enableDTA) {
                Addr dta_rx_end = DTARXJobAddr + 64;
                Addr dta_tx_end = DTATXJobAddr + 64;
                AddrRange dta_rx_range(DTARXJobAddr, dta_rx_end);
                AddrRange dta_tx_range(DTATXJobAddr, dta_tx_end);

                // Extract the range of DTARXJobAddr, DTATXJobAddr from the addrRanges
                AddrRangeList adjusted_ranges;
                assert(DTARXJobAddr < DTATXJobAddr);
                if (start < DTARXJobAddr) {
                    AddrRange adjusted_range(start, DTARXJobAddr);
                    adjusted_ranges.push_back(adjusted_range);
                }
                if (DTARXJobAddr < DTATXJobAddr) {
                    AddrRange adjusted_range(dta_rx_end, DTATXJobAddr);
                    adjusted_ranges.push_back(adjusted_range);
                }
                if (end > dta_tx_end) {
                    AddrRange adjusted_range(dta_tx_end, end);
                    adjusted_ranges.push_back(adjusted_range);
                }

                // Split the adjusted ranges
                Addr adjusted_size = 0;
                for (const auto& r : adjusted_ranges) {
                    adjusted_size += r.size();
                }

                port_size = adjusted_size / num_ports;

                for (int i = 0; i < num_ports - 1; i++) {
                    AddrRangeList new_ranges;
                    Addr remaining = port_size;
                    while (!adjusted_ranges.empty() && remaining > 0) {
                        AddrRange &current = adjusted_ranges.front();
                        Addr current_size = current.size();
                        if (current_size > remaining) {
                            // Split the current range
                            AddrRange new_range(current.start(), current.start() + remaining);
                            new_ranges.push_back(new_range);
                            current = AddrRange(current.start() + remaining, current.end());
                            remaining = 0;
                        } else {
                            new_ranges.push_back(current);
                            adjusted_ranges.pop_front();
                            remaining -= current_size;
                        }
                    }

                    printf("L3 cpu_side_port %d: ", i);
                    for (const auto& r : new_ranges) {
                        printf("%s ", r.to_string().c_str());
                    }
                    printf("\n");
                    addrRangesList.push_back(new_ranges);
                }
                // The last port - combine the remaining ranges
                AddrRangeList new_ranges;
                for (const auto& r : adjusted_ranges) {
                    new_ranges.push_back(r);
                }
                assert(new_ranges.size() > 0);
                printf("L3 cpu_side_port %d: ", num_ports - 1);
                for (const auto& r : new_ranges) {
                    printf("%s ", r.to_string().c_str());
                }
                printf("\n");
                addrRangesList.push_back(new_ranges);
                assert(addrRangesList.size() == num_ports);
            } else {
                for (int i = 0; i < num_ports - 1; i++) {
                    AddrRange new_range(start + accum_size, start + accum_size + port_size);
                    AddrRangeList new_ranges;
                    new_ranges.push_back(new_range);
                    accum_size += port_size;
                    printf("L3 cpu_side_port %d: %s\n", i, new_range.to_string().c_str());
                    addrRangesList.push_back(new_ranges);
                }
                AddrRange new_range(start + accum_size, end);
                AddrRangeList new_ranges;
                new_ranges.push_back(new_range);
                printf("L3 cpu_side_port %d: %s\n", num_ports - 1, new_range.to_string().c_str());
                addrRangesList.push_back(new_ranges);
                assert(addrRangesList.size() == num_ports);
            }
        } else {
            panic("L3 cache %s does not support multiple address ranges\n", name());
        }
    }

    if (isIOCache && enableDTA) {
        jobPort = new JobPort(p.name + ".job_port", this, "JobPort", p.dta_rx_job_addr, p.dta_tx_job_addr);
        ioSidePort = new IoSidePort(p.name + ".io_side_port", this, "IoSidePort");
        assert(dta != NULL);
        dta->setIOCache(this);
        printf("IOCache %s is enabled with DTA\n", name().c_str());
    } else {
        jobPort = nullptr;
        ioSidePort = nullptr;
        assert(dta == NULL);
        printf("IOCache %s is not enabled with DTA\n", name().c_str());
    }
}

BaseCache::~BaseCache()
{
    delete tempBlock;
}

void
BaseCache::CacheResponsePort::setBlocked()
{
    assert(!blocked);
    DPRINTF(CachePort, "Port is blocking new requests\n");
    blocked = true;
    // if we already scheduled a retry in this cycle, but it has not yet
    // happened, cancel it
    if (sendRetryEvent.scheduled()) {
        owner.deschedule(sendRetryEvent);
        DPRINTF(CachePort, "Port descheduled retry\n");
        mustSendRetry = true;
    }
}

void
BaseCache::CacheResponsePort::clearBlocked()
{
    assert(blocked);
    DPRINTF(CachePort, "Port is accepting new requests\n");
    blocked = false;
    if (mustSendRetry) {
        // @TODO: need to find a better time (next cycle?)
        owner.schedule(sendRetryEvent, curTick() + 1);
    }
}

void
BaseCache::CacheResponsePort::processSendRetry()
{
    DPRINTF(CachePort, "Port is sending retry\n");

    // reset the flag and call retry
    mustSendRetry = false;
    sendRetryReq();
}

Addr
BaseCache::regenerateBlkAddr(CacheBlk* blk)
{
    if (blk != tempBlock) {
        return tags->regenerateBlkAddr(blk);
    } else {
        return tempBlock->getAddr();
    }
}

void
BaseCache::init()
{
    if (isLLC && isMultiPort) {
        bool fullyConnected = true;
        for (const auto& cpu_port : cpuSidePortList) {
            if (!cpu_port->isConnected()) {
                fullyConnected = false;
                break;
            }
        }
        fullyConnected = fullyConnected && memSidePort.isConnected();
        if (!fullyConnected) {
            fatal("Not all CPU-side ports are connected to cache %s\n", name());
        }
        for (const auto& cpu_port : cpuSidePortList) {
            cpu_port->sendRangeChange();
            forwardSnoops = forwardSnoops || cpu_port->isSnooping();
        }
    } else if (isIOCache && enableDTA) {
        if (!ioSidePort->isConnected()) {
            fatal("DTA %s is not connected to IOBus (NIC)\n", name());
        }
        if (!jobPort->isConnected()) {
            fatal("DTA %s is not connected to Host\n", name());
        }
        jobPort->sendRangeChange(); // TODO - JM : have to check
        forwardSnoops = false; // IOCache does not forward snoops from memSidePort to the JobPort (Same L3XBar connected to both)
    } else {
        if (!cpuSidePort.isConnected() || !memSidePort.isConnected())
            fatal("Cache ports on %s are not connected\n", name());
        cpuSidePort.sendRangeChange();
        forwardSnoops = cpuSidePort.isSnooping();
    }
}

Port &
BaseCache::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_side") {
        return memSidePort;
    } else if (if_name == "cpu_side") {
        if (isLLC && isMultiPort) {
            if (idx < cpuSidePortList.size()) {
                return *cpuSidePortList[idx];
            } else {
                panic("L3 cache %s does not have a CPU-side port with index %d\n",
                      name(), idx);
            }
        } else {
            return cpuSidePort;
        }
    }  else if (if_name == "cpu_side_ports") { 
        if (isLLC && isMultiPort) {
            if (idx < cpuSidePortList.size()) {
                return *cpuSidePortList[idx];
            } else {
                panic("L3 cache %s does not have a CPU-side port with index %d\n",
                      name(), idx);
            }
        } else {
            if (isIOCache && enableDTA) {
                panic("IOCache %s does not have a CPU-side port\n", name());
            } else {
                return cpuSidePort;
            }
        }
    } else if (if_name == "io_side_port") {
        return *ioSidePort;
    } else if (if_name == "job_port") {
        return *jobPort;
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

bool
BaseCache::inRange(Addr addr) const
{
    for (const auto& r : addrRanges) {
        if (r.contains(addr)) {
            return true;
       }
    }
    return false;
}

void
BaseCache::handleTimingReqHit(PacketPtr pkt, CacheBlk *blk, Tick request_time, PortID cpu_side_port_id)
{
    if (pkt->needsResponse()) {
        // These delays should have been consumed by now
        assert(pkt->headerDelay == 0);
        assert(pkt->payloadDelay == 0);

        pkt->makeTimingResponse();

        // In this case we are considering request_time that takes
        // into account the delay of the xbar, if any, and just
        // lat, neglecting responseLatency, modelling hit latency
        // just as the value of lat overriden by access(), which calls
        // the calculateAccessLatency() function.
        if (isLLC && isMultiPort) {
            assert(cpu_side_port_id != InvalidPortID);
            assert(cpu_side_port_id < cpuSidePortList.size());
            cpuSidePortList[cpu_side_port_id]->schedTimingResp(pkt, request_time);
        } else if (isIOCache && pkt->isFromDTA()) {
            // This packet is from DTA, send it to DTA
            // Find the worker by using sender state
            assert(dta != nullptr);
            dta->recvTimingRespfromCache(pkt, request_time);
        } else {
            cpuSidePort.schedTimingResp(pkt, request_time);
        }
    } else {
        DPRINTF(Cache, "%s satisfied %s, no response needed\n", __func__,
                pkt->print());

        // queue the packet for deletion, as the sending cache is
        // still relying on it; if the block is found in access(),
        // CleanEvict and Writeback messages will be deleted
        // here as well
        pendingDelete.reset(pkt);
    }
}

void
BaseCache::handleTimingReqMiss(PacketPtr pkt, MSHR *mshr, CacheBlk *blk,
                               Tick forward_time, Tick request_time)
{
    if (writeAllocator && //writeAllocator is null in IOCache
        pkt && pkt->isWrite() && !pkt->req->isUncacheable()) {
        writeAllocator->updateMode(pkt->getAddr(), pkt->getSize(),
                                   pkt->getBlockAddr(blkSize));
    }

    if (mshr) {
        /// MSHR hit
        /// @note writebacks will be checked in getNextMSHR()
        /// for any conflicting requests to the same block

        //@todo remove hw_pf here

        // Coalesce unless it was a software prefetch (see above).
        if (pkt) {
            assert(!pkt->isWriteback());
            // CleanEvicts corresponding to blocks which have
            // outstanding requests in MSHRs are simply sunk here
            if (pkt->cmd == MemCmd::CleanEvict) {
                pendingDelete.reset(pkt);
            } else if (pkt->cmd == MemCmd::WriteClean) {
                // A WriteClean should never coalesce with any
                // outstanding cache maintenance requests.

                // We use forward_time here because there is an
                // uncached memory write, forwarded to WriteBuffer.
                allocateWriteBuffer(pkt, forward_time);
            } else {
                DPRINTF(Cache, "%s coalescing MSHR for %s\n", __func__,
                        pkt->print());

                assert(pkt->req->requestorId() < system->maxRequestors());
                stats.cmdStats(pkt).mshrHits[pkt->req->requestorId()]++;

                // We use forward_time here because it is the same
                // considering new targets. We have multiple
                // requests for the same address here. It
                // specifies the latency to allocate an internal
                // buffer and to schedule an event to the queued
                // port and also takes into account the additional
                // delay of the xbar.
                mshr->allocateTarget(pkt, forward_time, order++,
                                     allocOnFill(pkt->cmd));
                if (mshr->getNumTargets() == numTarget) {
                    noTargetMSHR = mshr;
                    setBlocked(Blocked_NoTargets);
                    // need to be careful with this... if this mshr isn't
                    // ready yet (i.e. time > curTick()), we don't want to
                    // move it ahead of mshrs that are ready
                    // mshrQueue.moveToFront(mshr);
                }
            }
        }
    } else {
        // no MSHR
        assert(pkt->req->requestorId() < system->maxRequestors());
        stats.cmdStats(pkt).mshrMisses[pkt->req->requestorId()]++;
        if (prefetcher && pkt->isDemand())
            prefetcher->incrDemandMhsrMisses();

        if (pkt->isEviction() || pkt->cmd == MemCmd::WriteClean) {
            // We use forward_time here because there is an
            // writeback or writeclean, forwarded to WriteBuffer.
            allocateWriteBuffer(pkt, forward_time);
        } else {
            if (blk && blk->isValid()) {
                // If we have a write miss to a valid block, we
                // need to mark the block non-readable.  Otherwise
                // if we allow reads while there's an outstanding
                // write miss, the read could return stale data
                // out of the cache block... a more aggressive
                // system could detect the overlap (if any) and
                // forward data out of the MSHRs, but we don't do
                // that yet.  Note that we do need to leave the
                // block valid so that it stays in the cache, in
                // case we get an upgrade response (and hence no
                // new data) when the write miss completes.
                // As long as CPUs do proper store/load forwarding
                // internally, and have a sufficiently weak memory
                // model, this is probably unnecessary, but at some
                // point it must have seemed like we needed it...
                assert((pkt->needsWritable() &&
                    !blk->isSet(CacheBlk::WritableBit)) ||
                    pkt->req->isCacheMaintenance());
                blk->clearCoherenceBits(CacheBlk::ReadableBit);
            }
            // Here we are using forward_time, modelling the latency of
            // a miss (outbound) just as forwardLatency, neglecting the
            // lookupLatency component.
            allocateMissBuffer(pkt, forward_time);
        }
    }
}

void
BaseCache::recvTimingReq(PacketPtr pkt, PortID cpu_side_port_id)
{
    // anything that is merely forwarded pays for the forward latency and
    // the delay provided by the crossbar
    Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;

    Cycles lat;
    CacheBlk *blk = nullptr;
    bool satisfied = false;
    {
        PacketList writebacks;
        // Note that lat is passed by reference here. The function
        // access() will set the lat value.
        // satisfied = access(pkt, blk, lat, writebacks);

        // SHIN. base DDIO
        satisfied = access(pkt, blk, lat, writebacks, pkt->isBlockIO() && ddioEnabled);

        // After the evicted blocks are selected, they must be forwarded
        // to the write buffer to ensure they logically precede anything
        // happening below
        doWritebacks(writebacks, clockEdge(lat + forwardLatency));
    }

    // For LOG
    // if LOG_LEVEL is 1, ring buffer log
    // if LOG_LEVEL is 2, dta log
    #if LOG_LEVEL == 1
        // For Ring Buffer LOG
        if (pkt->getAddr() == 1073752088) {
            printf("[LOG], %llu, %s, %s, %d, RX_TAIL_WR\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0);
        }
        if (pkt->getAddr() == 1073756184) {
            printf("[LOG], %llu, %s, %s, %d, TX_TAIL_WR\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0);
        }

        uint64_t rx_sw_ring_base = 0x20209bc80;
        uint64_t tx_sw_ring_base = 0x2020b4c80;
        uint64_t max_desc = 1024;
        uint64_t mbuf_ptr_size = 8;

        uint64_t rx_sw_ring_end = rx_sw_ring_base + max_desc * mbuf_ptr_size;
        uint64_t tx_sw_ring_end = tx_sw_ring_base + max_desc * mbuf_ptr_size;

        // RX Software Ring
        // if (pkt->getAddr() >= rx_sw_ring_base && pkt->getAddr() < rx_sw_ring_end) {
        //     int offset = (pkt->getAddr() - rx_sw_ring_base) / mbuf_ptr_size;
        //     printf("[LOG], %llu, %s, %s, %d, RX_SW_RING[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, offset);
        // }
        // // TX Software Ring
        // if (pkt->getAddr() >= tx_sw_ring_base && pkt->getAddr() < tx_sw_ring_end) {
        //     int offset = (pkt->getAddr() - tx_sw_ring_base) / mbuf_ptr_size;
        //     printf("[LOG], %llu, %s, %s, %d, TX_SW_RING[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, offset);
        // }

        // normal version
        // uint64_t rx_desc_base = 8624155648;
        // uint64_t tx_desc_base = 8624238080;

        // sve version
        uint64_t rx_desc_base = 8624135424;
        uint64_t tx_desc_base = 8624237824;

        uint64_t batch_size = 64;
        uint64_t desc_size = 16;

        // RX Descriptor (0-63)
        uint64_t rx_desc_0 = rx_desc_base + desc_size * 0;
        if (pkt->getAddr() >= rx_desc_0 && pkt->getAddr() < rx_desc_0 + batch_size * desc_size) {
            int offset = (pkt->getAddr() - rx_desc_0) / desc_size;
            int num_desc = (pkt->getSize() / desc_size);
            int last_offset = offset + num_desc - 1;
            if ((last_offset == 63 || last_offset == 62) && pkt->canGetDataPtr()) {
                // Get the last descriptor's DD bit
                uint8_t *data = pkt->getPtr<uint8_t>();
                uint8_t *last_desc = data + (num_desc - 1) * desc_size;
                E1000RXDescriptor *desc = (E1000RXDescriptor *)last_desc;
                bool dd = false;
                if ((pkt->isRead() && satisfied) || pkt->isWrite()) {
                    dd = desc->status_error & E1000_RXD_STAT_DD;
                }
                printf("[LOG], %llu, %s, %s, %d, RX_DESC_0_63[%d]_REQ_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
            }
            else {
                printf("[LOG], %llu, %s, %s, %d, RX_DESC_0_63[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, offset);
            }
        }

        // RX Descriptor (64-127)
        uint64_t rx_desc_64 = rx_desc_base + desc_size * 64;
        if (pkt->getAddr() >= rx_desc_64 && pkt->getAddr() < rx_desc_64 + batch_size * desc_size) {
            int offset = (pkt->getAddr() - rx_desc_64) / desc_size;
            int num_desc = (pkt->getSize() / desc_size);
            int last_offset = offset + num_desc - 1;
            if ((last_offset == 63 || last_offset == 62) && pkt->canGetDataPtr()) {
                // Get the last descriptor's DD bit
                uint8_t *data = pkt->getPtr<uint8_t>();
                uint8_t *last_desc = data + (num_desc - 1) * desc_size;
                E1000RXDescriptor *desc = (E1000RXDescriptor *)last_desc;
                bool dd = false;
                if ((pkt->isRead() && satisfied) || pkt->isWrite()) {
                    dd = desc->status_error & E1000_RXD_STAT_DD;
                }
                printf("[LOG], %llu, %s, %s, %d, RX_DESC_64_127[%d]_REQ_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
            }
            else {
                printf("[LOG], %llu, %s, %s, %d, RX_DESC_64_127[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, offset);
            }
        }

        // RX Descriptor (192-255)
        uint64_t rx_desc_192 = rx_desc_base + desc_size * 192;
        if (pkt->getAddr() >= rx_desc_192 && pkt->getAddr() < rx_desc_192 + batch_size * desc_size) {
            int offset = (pkt->getAddr() - rx_desc_192) / desc_size;
            int num_desc = (pkt->getSize() / desc_size);
            int last_offset = offset + num_desc - 1;
            if ((last_offset == 63 || last_offset == 62) && pkt->canGetDataPtr()) {
                // Get the last descriptor's DD bit
                uint8_t *data = pkt->getPtr<uint8_t>();
                uint8_t *last_desc = data + (num_desc - 1) * desc_size;
                E1000RXDescriptor *desc = (E1000RXDescriptor *)last_desc;
                bool dd = false;
                if ((pkt->isRead() && satisfied) || pkt->isWrite()) {
                    dd = desc->status_error & E1000_RXD_STAT_DD;
                }
                printf("[LOG], %llu, %s, %s, %d, RX_DESC_192_255[%d]_REQ_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
            }
            else {
                printf("[LOG], %llu, %s, %s, %d, RX_DESC_192_255[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, offset);
            }
        }

        // TX Descriptor (128-192)
        uint64_t tx_desc_128 = tx_desc_base + desc_size * 128;
        if (pkt->getAddr() >= tx_desc_128 && pkt->getAddr() < tx_desc_128 + batch_size * desc_size) {
            int offset = (pkt->getAddr() - tx_desc_128) / desc_size;
            int num_desc = (pkt->getSize() / desc_size);
            int last_offset = offset + num_desc - 1;
            if ((last_offset == 63) && pkt->canGetDataPtr()) {
                // Get the last descriptor's DD bit
                uint8_t *data = pkt->getPtr<uint8_t>();
                uint8_t *last_desc = data + (num_desc - 1) * desc_size;
                E1000TXDescriptor *desc = (E1000TXDescriptor *)last_desc;
                bool dd = false;
                if ((pkt->isRead() && satisfied) || pkt->isWrite()) {
                    dd = desc->wb.status & E1000_TXD_STAT_DD;
                }
                printf("[LOG], %llu, %s, %s, %d, TX_DESC_128_192[%d]_REQ_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
            }
            else {
                printf("[LOG], %llu, %s, %s, %d, TX_DESC_128_192[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, offset);
            }
        }

        // TX Descriptor (192-256)
        uint64_t tx_desc_192 = tx_desc_base + desc_size * 192;
        if (pkt->getAddr() >= tx_desc_192 && pkt->getAddr() < tx_desc_192 + batch_size * desc_size) {
            int offset = (pkt->getAddr() - tx_desc_192) / desc_size;
            int num_desc = (pkt->getSize() / desc_size);
            int last_offset = offset + num_desc - 1;
            if ((last_offset == 63) && pkt->canGetDataPtr()) {
                // Get the last descriptor's DD bit
                uint8_t *data = pkt->getPtr<uint8_t>();
                uint8_t *last_desc = data + (num_desc - 1) * desc_size;
                E1000TXDescriptor *desc = (E1000TXDescriptor *)last_desc;
                bool dd = false;
                if ((pkt->isRead() && satisfied) || pkt->isWrite()) {
                    dd = desc->wb.status & E1000_TXD_STAT_DD;
                }
                printf("[LOG], %llu, %s, %s, %d, TX_DESC_192_256[%d]_REQ_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
            }
            else {
                printf("[LOG], %llu, %s, %s, %d, TX_DESC_192_256[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, offset);
            }
        }

        // TX Descriptor (256-320)
        uint64_t tx_desc_256 = tx_desc_base + desc_size * 256;
        if (pkt->getAddr() >= tx_desc_256 && pkt->getAddr() < tx_desc_256 + batch_size * desc_size) {
            int offset = (pkt->getAddr() - tx_desc_256) / desc_size;
            int num_desc = (pkt->getSize() / desc_size);
            int last_offset = offset + num_desc - 1;
            if ((last_offset == 63) && pkt->canGetDataPtr()) {
                // Get the last descriptor's DD bit
                uint8_t *data = pkt->getPtr<uint8_t>();
                uint8_t *last_desc = data + (num_desc - 1) * desc_size;
                E1000TXDescriptor *desc = (E1000TXDescriptor *)last_desc;
                bool dd = false;
                if ((pkt->isRead() && satisfied) || pkt->isWrite()) {
                    dd = desc->wb.status & E1000_TXD_STAT_DD;
                }
                printf("[LOG], %llu, %s, %s, %d, TX_DESC_256_320[%d]_REQ_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
            }
            else {
                printf("[LOG], %llu, %s, %s, %d, TX_DESC_256_320[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, offset);
            }
        }

        // normal version
        // mbuf for RX Descriptor (63-95 pre-prev) - for tx mbuf free
        /*
        uint64_t mbuf_addr_set_95_preprev[] = {
            0x201650D80, 0x201651700, 0x201652080, 0x201652A00,
            0x201653380, 0x201653D00, 0x201654680, 0x201655000,
            0x201655980, 0x201656300, 0x201656C80, 0x201657600,
            0x201657F80, 0x201658900, 0x201659280, 0x201659C00,
            0x20165A580, 0x20165AF00, 0x20165B880, 0x20165C200,
            0x20165CB80, 0x20165D500, 0x20165DE80, 0x20165E800,
            0x20165F180, 0x20165FB00, 0x201660480, 0x201660E00,
            0x201661780, 0x201662100, 0x201662A80, 0x201663400
        };

        uint64_t mbuf_first_cacheline_set_95_preprev[] = {
            0x201650C80, 0x201651600, 0x201651F80, 0x201652900,
            0x201653280, 0x201653C00, 0x201654580, 0x201654F00,
            0x201655880, 0x201656200, 0x201656B80, 0x201657500,
            0x201657E80, 0x201658800, 0x201659180, 0x201659B00,
            0x20165A480, 0x20165AE00, 0x20165B780, 0x20165C100,
            0x20165CA80, 0x20165D400, 0x20165DD80, 0x20165E700,
            0x20165F080, 0x20165FA00, 0x201660380, 0x201660D00,
            0x201661680, 0x201662000, 0x201662980, 0x201663300
        };

        uint64_t mbuf_second_cacheline_set_95_preprev[] = {
            0x201650CC0, 0x201651640, 0x201651FC0, 0x201652940,
            0x2016532C0, 0x201653C40, 0x2016545C0, 0x201654F40,
            0x2016558C0, 0x201656240, 0x201656BC0, 0x201657540,
            0x201657EC0, 0x201658840, 0x2016591C0, 0x201659B40,
            0x20165A4C0, 0x20165AE40, 0x20165B7C0, 0x20165C140,
            0x20165CAC0, 0x20165D440, 0x20165DDC0, 0x20165E740,
            0x20165F0C0, 0x20165FA40, 0x2016603C0, 0x201660D40,
            0x2016616C0, 0x201662040, 0x2016629C0, 0x201663340
        };

        // mbuf for RX Descriptor (63-95 prev)
        uint64_t mbuf_addr_set_95_prev[] = {
            0x20154c000, 0x20154c980, 0x20154d300, 0x20154dc80, 
            0x20154e600, 0x20154ef80, 0x20154f900, 0x201550280, 
            0x201550c00, 0x201551580, 0x201551f00, 0x201552880, 
            0x201553200, 0x201553b80, 0x201554500, 0x201554e80, 
            0x201555800, 0x201556180, 0x201556b00, 0x201557480,
            0x201557e00, 0x201558780, 0x201559100, 0x201559a80, 
            0x20155a400, 0x20155ad80, 0x20155b700, 0x20155c080, 
            0x20155ca00, 0x20155d380, 0x20155dd00, 0x20155e680
        };

        // mbuf's first cacheline for RX Descriptor (63-95 prev)
        uint64_t mbuf_first_cacheline_set_95_prev[] = {
            0x20154bf00, 0x20154c880, 0x20154d200, 0x20154db80, 
            0x20154e500, 0x20154ee80, 0x20154f800, 0x201550180, 
            0x201550b00, 0x201551480, 0x201551e00, 0x201552780, 
            0x201553100, 0x201553a80, 0x201554400, 0x201554d80, 
            0x201555700, 0x201556080, 0x201556a00, 0x201557380, 
            0x201557d00, 0x201558680, 0x201559000, 0x201559980, 
            0x20155a300, 0x20155ac80, 0x20155b600, 0x20155bf80, 
            0x20155c900, 0x20155d280, 0x20155dc00, 0x20155e580
        };

        // mbuf's second cacheline for RX Descriptor (63-95 prev)
        uint64_t mbuf_second_cacheline_set_95_prev[] = {
            0x20154bf40, 0x20154c8c0, 0x20154d240, 0x20154dbc0,
            0x20154e540, 0x20154eec0, 0x20154f840, 0x2015501c0,
            0x201550b40, 0x2015514c0, 0x201551e40, 0x2015527c0,
            0x201553140, 0x201553ac0, 0x201554440, 0x201554dc0,
            0x201555740, 0x2015560c0, 0x201556a40, 0x2015573c0,
            0x201557d40, 0x2015586c0, 0x201559040, 0x2015599c0,
            0x20155a340, 0x20155acc0, 0x20155b640, 0x20155bfc0,
            0x20155c940, 0x20155d2c0, 0x20155dc40, 0x20155e5c0
        };

        // mbuf for RX Descriptor (63-95 new)
        uint64_t mbuf_addr_set_95_new[] = {
            0x201676d80, 0x201676400, 0x201675a80, 0x201675100,
            0x201674780, 0x201673e00, 0x201673480, 0x201672b00,
            0x201672180, 0x201671800, 0x201670e80, 0x201670500,
            0x20166fb80, 0x20166f200, 0x20166e880, 0x20166df00,
            0x20166d580, 0x20166cc00, 0x20166c280, 0x20166b900,
            0x20166af80, 0x20166a600, 0x201669c80, 0x201669300,
            0x201668980, 0x201668000, 0x201667680, 0x201666d00,
            0x201666380, 0x201665a00, 0x201665080, 0x201664700
        };

        // mbuf's first cacheline for RX Descriptor (63-95 new) - minus 256 from the mbuf_addr_set_95_new
        uint64_t mbuf_first_cacheline_set_95_new[] = {
            0x201676c80, 0x201676300, 0x201675980, 0x201675000,
            0x201674680, 0x201673d00, 0x201673380, 0x201672a00,
            0x201672080, 0x201671700, 0x201670d80, 0x201670400,
            0x20166fa80, 0x20166f100, 0x20166e780, 0x20166de00,
            0x20166d480, 0x20166cb00, 0x20166c180, 0x20166b800,
            0x20166ae80, 0x20166a500, 0x201669b80, 0x201669200,
            0x201668880, 0x201667f00, 0x201667580, 0x201666c00,
            0x201666280, 0x201665900, 0x201664f80, 0x201664600
        };

        // mbuf's second cacheline for RX Descriptor (63-95 new) + 64 from the mbuf_first_cacheline_set_95_new
        uint64_t mbuf_second_cacheline_set_95_new[] = {
            0x201676cc0, 0x201676340, 0x2016759c0, 0x201675040,
            0x2016746c0, 0x201673d40, 0x2016733c0, 0x201672a40,
            0x2016720c0, 0x201671740, 0x201670dc0, 0x201670440,
            0x20166fac0, 0x20166f140, 0x20166e7c0, 0x20166de40,
            0x20166d4c0, 0x20166cb40, 0x20166c1c0, 0x20166b840,
            0x20166aec0, 0x20166a540, 0x201669bc0, 0x201669240,
            0x2016688c0, 0x201667f40, 0x2016675c0, 0x201666c40,
            0x2016662c0, 0x201665940, 0x201664fc0, 0x201664640
        };

        // mbuf for RX Descriptor (95-127 prev) -> TX Descriptor (128-160)
        uint64_t mbuf_addr_set_127_prev[] = {
            0x201539000, 0x201539980, 0x20153A300, 0x20153AC80,
            0x20153B600, 0x20153BF80, 0x20153C900, 0x20153D280,
            0x20153DC00, 0x20153E580, 0x20153EF00, 0x20153F880,
            0x201540200, 0x201540B80, 0x201541500, 0x201541E80,
            0x201542800, 0x201543180, 0x201543B00, 0x201544480,
            0x201544E00, 0x201545780, 0x201546100, 0x201546A80,
            0x201547400, 0x201547D80, 0x201548700, 0x201549080,
            0x201549A00, 0x20154A380, 0x20154AD00, 0x20154B680
        };

        uint64_t mbuf_first_cacheline_set_127_prev[] = {
            0x201538F00, 0x201539880, 0x20153A200, 0x20153AB80,
            0x20153B500, 0x20153BE80, 0x20153C800, 0x20153D180,
            0x20153DB00, 0x20153E480, 0x20153EE00, 0x20153F780,
            0x201540100, 0x201540A80, 0x201541400, 0x201541D80,
            0x201542700, 0x201543080, 0x201543A00, 0x201544380,
            0x201544D00, 0x201545680, 0x201546000, 0x201546980,
            0x201547300, 0x201547C80, 0x201548600, 0x201548F80,
            0x201549900, 0x20154A280, 0x20154AC00, 0x20154B580
        };

        uint64_t mbuf_second_cacheline_set_127_prev[] = {
            0x201538F40, 0x2015398C0, 0x20153A240, 0x20153ABC0,
            0x20153B540, 0x20153BEC0, 0x20153C840, 0x20153D1C0,
            0x20153DB40, 0x20153E4C0, 0x20153EE40, 0x20153F7C0,
            0x201540140, 0x201540AC0, 0x201541440, 0x201541DC0,
            0x201542740, 0x2015430C0, 0x201543A40, 0x2015443C0,
            0x201544D40, 0x2015456C0, 0x201546040, 0x2015469C0,
            0x201547340, 0x201547CC0, 0x201548640, 0x201548FC0,
            0x201549940, 0x20154A2C0, 0x20154AC40, 0x20154B5C0
        };
        */

        // sve version
        // mbuf for TX Descriptor (128-191 current) - throught current phase's tdt 192 wr prefetch
        uint64_t mbuf_addr_set_192_cur[] = {
            0x2013B9D80, 0x2013BA700, 0x2013BB080, 0x2013BBA00,
            0x2013BC380, 0x2013BCD00, 0x2013BD680, 0x2013BE000,
            0x2013BE980, 0x2013BF300, 0x2013BFC80, 0x2013C0600,
            0x2013C0F80, 0x2013C1900, 0x2013C2280, 0x2013C2C00,
            0x2013C3580, 0x2013C3F00, 0x2013C4880, 0x2013C5200,
            0x2013C5B80, 0x2013C6500, 0x2013C6E80, 0x2013C7800,
            0x2013C8180, 0x2013C8B00, 0x2013C9480, 0x2013C9E00,
            0x2013CA780, 0x2013CB100, 0x2013CBA80, 0x2013CC400,
            0x2013CCD80, 0x2013CD700, 0x2013CE080, 0x2013CEA00,
            0x2013CF380, 0x2013CFD00, 0x2013D0680, 0x2013D1000,
            0x2013D1980, 0x2013D2300, 0x2013D2C80, 0x2013D3600,
            0x2013D3F80, 0x2013D4900, 0x2013D5280, 0x2013D5C00,
            0x2013D6580, 0x2013D6F00, 0x2013D7880, 0x2013D8200,
            0x2013D8B80, 0x2013D9500, 0x2013D9E80, 0x2013DA800,
            0x2013DB180, 0x2013DBB00, 0x2013DC480, 0x2013DCE00,
            0x2013DD780, 0x2013DE100, 0x2013DEA80, 0x2013DF400
        };

        uint64_t mbuf_first_cacheline_set_192_cur[] = {
            0x2013B9C80, 0x2013BA600, 0x2013BAF80, 0x2013BB900,
            0x2013BC280, 0x2013BCC00, 0x2013BD580, 0x2013BDF00,
            0x2013BE880, 0x2013BF200, 0x2013BFB80, 0x2013C0500,
            0x2013C0E80, 0x2013C1800, 0x2013C2180, 0x2013C2B00,
            0x2013C3480, 0x2013C3E00, 0x2013C4780, 0x2013C5100,
            0x2013C5A80, 0x2013C6400, 0x2013C6D80, 0x2013C7700,
            0x2013C8080, 0x2013C8A00, 0x2013C9380, 0x2013C9D00,
            0x2013CA680, 0x2013CB000, 0x2013CB980, 0x2013CC300,
            0x2013CCC80, 0x2013CD600, 0x2013CDF80, 0x2013CE900,
            0x2013CF280, 0x2013CFC00, 0x2013D0580, 0x2013D0F00,
            0x2013D1880, 0x2013D2200, 0x2013D2B80, 0x2013D3500,
            0x2013D3E80, 0x2013D4800, 0x2013D5180, 0x2013D5B00,
            0x2013D6480, 0x2013D6E00, 0x2013D7780, 0x2013D8100,
            0x2013D8A80, 0x2013D9400, 0x2013D9D80, 0x2013DA700,
            0x2013DB080, 0x2013DBA00, 0x2013DC380, 0x2013DCD00,
            0x2013DD680, 0x2013DE000, 0x2013DE980, 0x2013DF300
        };

        uint64_t mbuf_second_cacheline_set_192_cur[] = {
            0x2013B9CC0, 0x2013BA640, 0x2013BAFC0, 0x2013BB940,
            0x2013BC2C0, 0x2013BCC40, 0x2013BD5C0, 0x2013BDF40,
            0x2013BE8C0, 0x2013BF240, 0x2013BFBC0, 0x2013C0540,
            0x2013C0EC0, 0x2013C1840, 0x2013C21C0, 0x2013C2B40,
            0x2013C34C0, 0x2013C3E40, 0x2013C47C0, 0x2013C5140,
            0x2013C5AC0, 0x2013C6440, 0x2013C6DC0, 0x2013C7740,
            0x2013C80C0, 0x2013C8A40, 0x2013C93C0, 0x2013C9D40,
            0x2013CA6C0, 0x2013CB040, 0x2013CB9C0, 0x2013CC340,
            0x2013CCCC0, 0x2013CD640, 0x2013CDFC0, 0x2013CE940,
            0x2013CF2C0, 0x2013CFC40, 0x2013D05C0, 0x2013D0F40,
            0x2013D18C0, 0x2013D2240, 0x2013D2BC0, 0x2013D3540,
            0x2013D3EC0, 0x2013D4840, 0x2013D51C0, 0x2013D5B40,
            0x2013D64C0, 0x2013D6E40, 0x2013D77C0, 0x2013D8140,
            0x2013D8AC0, 0x2013D9440, 0x2013D9DC0, 0x2013DA740,
            0x2013DB0C0, 0x2013DBA40, 0x2013DC3C0, 0x2013DCD40,
            0x2013DD6C0, 0x2013DE040, 0x2013DE9C0, 0x2013DF340
        };
        
        // mbuf for RX Descriptor (64-127 new) - through next phase's tdt 128 wr prefetch
        uint64_t mbuf_addr_set_128_new[] = {
            0x201498B00, 0x201498180, 0x201497800, 0x201496E80,
            0x201496500, 0x201495B80, 0x201495200, 0x201494880,
            0x201493F00, 0x201493580, 0x201492C00, 0x201492280,
            0x201491900, 0x201490F80, 0x201490600, 0x20148FC80,
            0x20148F300, 0x20148E980, 0x20148E000, 0x20148D680,
            0x20148CD00, 0x20148C380, 0x20148BA00, 0x20148B080,
            0x20148A700, 0x201489D80, 0x201489400, 0x201488A80,
            0x201488100, 0x201487780, 0x201486E00, 0x201486480,
            0x201485B00, 0x201485180, 0x201484800, 0x201483E80,
            0x201483500, 0x201482B80, 0x201482200, 0x201481880,
            0x201480F00, 0x201480580, 0x20147FC00, 0x20147F280,
            0x20147E900, 0x20147DF80, 0x20147D600, 0x20147CC80,
            0x20147C300, 0x20147B980, 0x20147B000, 0x20147A680,
            0x201479D00, 0x201479380, 0x201478A00, 0x201478080,
            0x201477700, 0x201476D80, 0x201476400, 0x201475A80,
            0x201475100, 0x201474780, 0x201473E00, 0x201473480
        };

        uint64_t mbuf_first_cacheline_set_128_new[] = {
            0x201498A00, 0x201498080, 0x201497700, 0x201496D80,
            0x201496400, 0x201495A80, 0x201495100, 0x201494780,
            0x201493E00, 0x201493480, 0x201492B00, 0x201492180,
            0x201491800, 0x201490E80, 0x201490500, 0x20148FB80,
            0x20148F200, 0x20148E880, 0x20148DF00, 0x20148D580,
            0x20148CC00, 0x20148C280, 0x20148B900, 0x20148AF80,
            0x20148A600, 0x201489C80, 0x201489300, 0x201488980,
            0x201488000, 0x201487680, 0x201486D00, 0x201486380,
            0x201485A00, 0x201485080, 0x201484700, 0x201483D80,
            0x201483400, 0x201482A80, 0x201482100, 0x201481780,
            0x201480E00, 0x201480480, 0x20147FB00, 0x20147F180,
            0x20147E800, 0x20147DE80, 0x20147D500, 0x20147CB80,
            0x20147C200, 0x20147B880, 0x20147AF00, 0x20147A580,
            0x201479C00, 0x201479280, 0x201478900, 0x201477F80,
            0x201477600, 0x201476C80, 0x201476300, 0x201475980,
            0x201475000, 0x201474680, 0x201473D00, 0x201473380
        };

        uint64_t mbuf_second_cacheline_set_128_new[] = {
            0x201498A40, 0x2014980C0, 0x201497740, 0x201496DC0,
            0x201496440, 0x201495AC0, 0x201495140, 0x2014947C0,
            0x201493E40, 0x2014934C0, 0x201492B40, 0x2014921C0,
            0x201491840, 0x201490EC0, 0x201490540, 0x20148FBC0,
            0x20148F240, 0x20148E8C0, 0x20148DF40, 0x20148D5C0,
            0x20148CC40, 0x20148C2C0, 0x20148B940, 0x20148AFC0,
            0x20148A640, 0x201489CC0, 0x201489340, 0x2014889C0,
            0x201488040, 0x2014876C0, 0x201486D40, 0x2014863C0,
            0x201485A40, 0x2014850C0, 0x201484740, 0x201483DC0,
            0x201483440, 0x201482AC0, 0x201482140, 0x2014817C0,
            0x201480E40, 0x2014804C0, 0x20147FB40, 0x20147F1C0,
            0x20147E840, 0x20147DEC0, 0x20147D540, 0x20147CBC0,
            0x20147C240, 0x20147B8C0, 0x20147AF40, 0x20147A5C0,
            0x201479C40, 0x2014792C0, 0x201478940, 0x201477FC0,
            0x201477640, 0x201476CC0, 0x201476340, 0x2014759C0,
            0x201475040, 0x2014746C0, 0x201473D40, 0x2014733C0
        };

        // mbuf for RX Descriptor (192-255 current) - through current phase's tdt 256 wr prefetch
        uint64_t mbuf_addr_set_256_cur[] = {
            0x201393D80, 0x201394700, 0x201395080, 0x201395A00,
            0x201396380, 0x201396D00, 0x201397680, 0x201398000,
            0x201398980, 0x201399300, 0x201399C80, 0x20139A600,
            0x20139AF80, 0x20139B900, 0x20139C280, 0x20139CC00,
            0x20139D580, 0x20139DF00, 0x20139E880, 0x20139F200,
            0x20139FB80, 0x2013A0500, 0x2013A0E80, 0x2013A1800,
            0x2013A2180, 0x2013A2B00, 0x2013A3480, 0x2013A3E00,
            0x2013A4780, 0x2013A5100, 0x2013A5A80, 0x2013A6400,
            0x2013A6D80, 0x2013A7700, 0x2013A8080, 0x2013A8A00,
            0x2013A9380, 0x2013A9D00, 0x2013AA680, 0x2013AB000,
            0x2013AB980, 0x2013AC300, 0x2013ACC80, 0x2013AD600,
            0x2013ADF80, 0x2013AE900, 0x2013AF280, 0x2013AFC00,
            0x2013B0580, 0x2013B0F00, 0x2013B1880, 0x2013B2200,
            0x2013B2B80, 0x2013B3500, 0x2013B3E80, 0x2013B4800,
            0x2013B5180, 0x2013B5B00, 0x2013B6480, 0x2013B6E00,
            0x2013B7780, 0x2013B8100, 0x2013B8A80, 0x2013B9400
        };

        uint64_t mbuf_first_cacheline_set_256_cur[] = {
            0x201393C80, 0x201394600, 0x201394F80, 0x201395900,
            0x201396280, 0x201396C00, 0x201397580, 0x201397F00,
            0x201398880, 0x201399200, 0x201399B80, 0x20139A500,
            0x20139AE80, 0x20139B800, 0x20139C180, 0x20139CB00,
            0x20139D480, 0x20139DE00, 0x20139E780, 0x20139F100,
            0x20139FA80, 0x2013A0400, 0x2013A0D80, 0x2013A1700,
            0x2013A2080, 0x2013A2A00, 0x2013A3380, 0x2013A3D00,
            0x2013A4680, 0x2013A5000, 0x2013A5980, 0x2013A6300,
            0x2013A6C80, 0x2013A7600, 0x2013A7F80, 0x2013A8900,
            0x2013A9280, 0x2013A9C00, 0x2013AA580, 0x2013AAF00,
            0x2013AB880, 0x2013AC200, 0x2013ACB80, 0x2013AD500,
            0x2013ADE80, 0x2013AE800, 0x2013AF180, 0x2013AFB00,
            0x2013B0480, 0x2013B0E00, 0x2013B1780, 0x2013B2100,
            0x2013B2A80, 0x2013B3400, 0x2013B3D80, 0x2013B4700,
            0x2013B5080, 0x2013B5A00, 0x2013B6380, 0x2013B6D00,
            0x2013B7680, 0x2013B8000, 0x2013B8980, 0x2013B9300
        };

        uint64_t mbuf_second_cacheline_set_256_cur[] = {
            0x201393CC0, 0x201394640, 0x201394FC0, 0x201395940,
            0x2013962C0, 0x201396C40, 0x2013975C0, 0x201397F40,
            0x2013988C0, 0x201399240, 0x201399BC0, 0x20139A540,
            0x20139AEC0, 0x20139B840, 0x20139C1C0, 0x20139CB40,
            0x20139D4C0, 0x20139DE40, 0x20139E7C0, 0x20139F140,
            0x20139FAC0, 0x2013A0440, 0x2013A0DC0, 0x2013A1740,
            0x2013A20C0, 0x2013A2A40, 0x2013A33C0, 0x2013A3D40,
            0x2013A46C0, 0x2013A5040, 0x2013A59C0, 0x2013A6340,
            0x2013A6CC0, 0x2013A7640, 0x2013A7FC0, 0x2013A8940,
            0x2013A92C0, 0x2013A9C40, 0x2013AA5C0, 0x2013AAF40,
            0x2013AB8C0, 0x2013AC240, 0x2013ACBC0, 0x2013AD540,
            0x2013ADEC0, 0x2013AE840, 0x2013AF1C0, 0x2013AFB40,
            0x2013B04C0, 0x2013B0E40, 0x2013B17C0, 0x2013B2140,
            0x2013B2AC0, 0x2013B3440, 0x2013B3DC0, 0x2013B4740,
            0x2013B50C0, 0x2013B5A40, 0x2013B63C0, 0x2013B6D40,
            0x2013B76C0, 0x2013B8040, 0x2013B89C0, 0x2013B9340
        };

        // mbuf for TX Descriptor (256-319 prev) - through previous phase's tdt 320 wr prefetch
        uint64_t mbuf_addr_set_320_prev[] = {
            0x201451700, 0x201450D80, 0x201450400, 0x20144FA80,
            0x20144F100, 0x20144E780, 0x20144DE00, 0x20144D480,
            0x201456300, 0x201455980, 0x201455000, 0x201454680,
            0x201453D00, 0x201453380, 0x201452A00, 0x201452080,
            0x20145AF00, 0x20145A580, 0x201459C00, 0x201459280,
            0x201458900, 0x201457F80, 0x201457600, 0x201456C80,
            0x20145FB00, 0x20145F180, 0x20145E800, 0x20145DE80,
            0x20145D500, 0x20145CB80, 0x20145C200, 0x20145B880,
            0x201464700, 0x201463D80, 0x201463400, 0x201462A80,
            0x201462100, 0x201461780, 0x201460E00, 0x201460480,
            0x201469300, 0x201468980, 0x201468000, 0x201467680,
            0x201466D00, 0x201466380, 0x201465A00, 0x201465080,
            0x20146DF00, 0x20146D580, 0x20146CC00, 0x20146C280,
            0x20146B900, 0x20146AF80, 0x20146A600, 0x201469C80,
            0x201472B00, 0x201472180, 0x201471800, 0x201470E80,
            0x201470500, 0x20146FB80, 0x20146F200, 0x20146E880
        };

        uint64_t mbuf_first_cacheline_set_320_prev[] = {
            0x201451600, 0x201450C80, 0x201450300, 0x20144F980,
            0x20144F000, 0x20144E680, 0x20144DD00, 0x20144D380,
            0x201456200, 0x201455880, 0x201454F00, 0x201454580,
            0x201453C00, 0x201453280, 0x201452900, 0x201451F80,
            0x20145AE00, 0x20145A480, 0x201459B00, 0x201459180,
            0x201458800, 0x201457E80, 0x201457500, 0x201456B80,
            0x20145FA00, 0x20145F080, 0x20145E700, 0x20145DD80,
            0x20145D400, 0x20145CA80, 0x20145C100, 0x20145B780,
            0x201464600, 0x201463C80, 0x201463300, 0x201462980,
            0x201462000, 0x201461680, 0x201460D00, 0x201460380,
            0x201469200, 0x201468880, 0x201467F00, 0x201467580,
            0x201466C00, 0x201466280, 0x201465900, 0x201464F80,
            0x20146DE00, 0x20146D480, 0x20146CB00, 0x20146C180,
            0x20146B800, 0x20146AE80, 0x20146A500, 0x201469B80,
            0x201472A00, 0x201472080, 0x201471700, 0x201470D80,
            0x201470400, 0x20146FA80, 0x20146F100, 0x20146E780
        };

        uint64_t mbuf_second_cacheline_set_320_prev[] = {
            0x201451640, 0x201450CC0, 0x201450340, 0x20144F9C0,
            0x20144F040, 0x20144E6C0, 0x20144DD40, 0x20144D3C0,
            0x201456240, 0x2014558C0, 0x201454F40, 0x2014545C0,
            0x201453C40, 0x2014532C0, 0x201452940, 0x201451FC0,
            0x20145AE40, 0x20145A4C0, 0x201459B40, 0x2014591C0,
            0x201458840, 0x201457EC0, 0x201457540, 0x201456BC0,
            0x20145FA40, 0x20145F0C0, 0x20145E740, 0x20145DDC0,
            0x20145D440, 0x20145CAC0, 0x20145C140, 0x20145B7C0,
            0x201464640, 0x201463CC0, 0x201463340, 0x2014629C0,
            0x201462040, 0x2014616C0, 0x201460D40, 0x2014603C0,
            0x201469240, 0x2014688C0, 0x201467F40, 0x2014675C0,
            0x201466C40, 0x2014662C0, 0x201465940, 0x201464FC0,
            0x20146DE40, 0x20146D4C0, 0x20146CB40, 0x20146C1C0,
            0x20146B840, 0x20146AEC0, 0x20146A540, 0x201469BC0,
            0x201472A40, 0x2014720C0, 0x201471740, 0x201470DC0,
            0x201470440, 0x20146FAC0, 0x20146F140, 0x20146E7C0
        };        


        // mbuf's structure part
        for (int i = 0; i < 64; i ++) {
            uint64_t pkt_addr = pkt->getAddr();
            uint64_t pkt_addr_end = pkt_addr + pkt->getSize();
            // Check if the packet address is in the mbuf's structure part
            /*
            if (pkt_addr >= mbuf_first_cacheline_set_95_preprev[i] && pkt_addr_end <= mbuf_first_cacheline_set_95_preprev[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$0_95PREPREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_second_cacheline_set_95_preprev[i] && pkt_addr_end <= mbuf_second_cacheline_set_95_preprev[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$1_95PREPREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_first_cacheline_set_95_prev[i] && pkt_addr_end <= mbuf_first_cacheline_set_95_prev[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$0_95PREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_second_cacheline_set_95_prev[i] && pkt_addr_end <= mbuf_second_cacheline_set_95_prev[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$1_95PREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_first_cacheline_set_95_new[i] && pkt_addr_end <= mbuf_first_cacheline_set_95_new[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$0_95NEW[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_second_cacheline_set_95_new[i] && pkt_addr_end <= mbuf_second_cacheline_set_95_new[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$1_95NEW[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_first_cacheline_set_127_prev[i] && pkt_addr_end <= mbuf_first_cacheline_set_127_prev[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$0_127PREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_second_cacheline_set_127_prev[i] && pkt_addr_end <= mbuf_second_cacheline_set_127_prev[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$1_127PREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            */

            // SVE
            if (pkt_addr >= mbuf_first_cacheline_set_192_cur[i] && pkt_addr_end <= mbuf_first_cacheline_set_192_cur[i] + 64) {
                uint64_t pktdata = 111111111;
                if (pkt_addr - mbuf_first_cacheline_set_192_cur[i] == 0x38) {
                    // This is the access to the mbuf's pool addr
                    if (pkt->isRead() && satisfied) {
                        // Get value from pkt's data
                        int64_t* data = pkt->getPtr<int64_t>();
                        pktdata = data[0];
                    } else if (pkt->isWrite()) {
                        // Get value from pkt's data
                        int64_t* data = pkt->getPtr<int64_t>();
                        pktdata = data[0];
                    }
                    printf("[LOG], %llu, %s, %s, %d, MBUF_$0_192CUR[%d]_pool_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pktdata, i);
                } else {
                    printf("[LOG], %llu, %s, %s, %d, MBUF_$0_192CUR[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pktdata, i);
                }
            }
            if (pkt_addr >= mbuf_second_cacheline_set_192_cur[i] && pkt_addr_end <= mbuf_second_cacheline_set_192_cur[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$1_192CUR[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_first_cacheline_set_128_new[i] && pkt_addr_end <= mbuf_first_cacheline_set_128_new[i] + 64) {
                uint64_t pktdata = 111111111;
                if (pkt_addr - mbuf_first_cacheline_set_128_new[i] == 0x38) {
                    // This is the access to the mbuf's pool addr
                    if (pkt->isRead() && satisfied) {
                        // Get value from pkt's data
                        int64_t* data = pkt->getPtr<int64_t>();
                        pktdata = data[0];
                    } else if (pkt->isWrite()) {
                        // Get value from pkt's data
                        int64_t* data = pkt->getPtr<int64_t>();
                        pktdata = data[0];
                    }
                    printf("[LOG], %llu, %s, %s, %d, MBUF_$0_128NEW[%d]_pool_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pktdata, i);
                } else {
                    printf("[LOG], %llu, %s, %s, %d, MBUF_$0_128NEW[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pktdata, i);
                }
            }
            if (pkt_addr >= mbuf_second_cacheline_set_128_new[i] && pkt_addr_end <= mbuf_second_cacheline_set_128_new[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$1_128NEW[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_first_cacheline_set_256_cur[i] && pkt_addr_end <= mbuf_first_cacheline_set_256_cur[i] + 64) {
                uint64_t pktdata = 111111111;
                if (pkt_addr - mbuf_first_cacheline_set_256_cur[i] == 0x38) {
                    // This is the access to the mbuf's pool addr
                    if (pkt->isRead() && satisfied) {
                        // Get value from pkt's data
                        int64_t* data = pkt->getPtr<int64_t>();
                        pktdata = data[0];
                    } else if (pkt->isWrite()) {
                        // Get value from pkt's data
                        int64_t* data = pkt->getPtr<int64_t>();
                        pktdata = data[0];
                    }
                    printf("[LOG], %llu, %s, %s, %d, MBUF_$0_256CUR[%d]_pool_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pktdata, i);
                } else {
                    printf("[LOG], %llu, %s, %s, %d, MBUF_$0_256CUR[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pktdata, i);
                }
            }
            if (pkt_addr >= mbuf_second_cacheline_set_256_cur[i] && pkt_addr_end <= mbuf_second_cacheline_set_256_cur[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$1_256CUR[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt_addr >= mbuf_first_cacheline_set_320_prev[i] && pkt_addr_end <= mbuf_first_cacheline_set_320_prev[i] + 64) {
                uint64_t pktdata = 111111111;
                if (pkt_addr - mbuf_first_cacheline_set_320_prev[i] == 0x38) {
                    // This is the access to the mbuf's pool addr
                    if (pkt->isRead() && satisfied) {
                        // Get value from pkt's data
                        int64_t* data = pkt->getPtr<int64_t>();
                        pktdata = data[0];
                    } else if (pkt->isWrite()) {
                        // Get value from pkt's data
                        int64_t* data = pkt->getPtr<int64_t>();
                        pktdata = data[0];
                    }
                    printf("[LOG], %llu, %s, %s, %d, MBUF_$0_320PREV[%d]_pool_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pktdata, i);
                } else {
                    printf("[LOG], %llu, %s, %s, %d, MBUF_$0_320PREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pktdata, i);
                }
            }
            if (pkt_addr >= mbuf_second_cacheline_set_320_prev[i] && pkt_addr_end <= mbuf_second_cacheline_set_320_prev[i] + 64) {
                printf("[LOG], %llu, %s, %s, %d, MBUF_$1_320PREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            
        }

        // mbuf's data part
        for (int i = 0; i < 32; i++) {
            // if (pkt->getAddr() == mbuf_addr_set_95_preprev[i]) {
            //     printf("[LOG], %llu, %s, %s, %d, MBUF95PREPREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            // }
            // if (pkt->getAddr() == mbuf_addr_set_95_prev[i]) {
            //     printf("[LOG], %llu, %s, %s, %d, MBUF95PREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            // }
            // if (pkt->getAddr() == mbuf_addr_set_95_new[i]) {
            //     printf("[LOG], %llu, %s, %s, %d, MBUF95NEW[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            // }
            // if (pkt->getAddr() == mbuf_addr_set_127_prev[i]) {
            //     printf("[LOG], %llu, %s, %s, %d, MBUF127PREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            // }

            // SVE
            if (pkt->getAddr() == mbuf_addr_set_192_cur[i]) {
                printf("[LOG], %llu, %s, %s, %d, MBUF192CUR[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt->getAddr() == mbuf_addr_set_128_new[i]) {
                printf("[LOG], %llu, %s, %s, %d, MBUF128NEW[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt->getAddr() == mbuf_addr_set_256_cur[i]) {
                printf("[LOG], %llu, %s, %s, %d, MBUF256CUR[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            if (pkt->getAddr() == mbuf_addr_set_320_prev[i]) {
                printf("[LOG], %llu, %s, %s, %d, MBUF320PREV[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
            }
            
        }

    #endif

    #if LOG_LEVEL == 2
    // RX_JOB_SUBMIT
    if (pkt->getAddr() == 1073752256) {
        printf("[LOG], %llu, %s, %s, %d, RX_JOB_SUBMIT\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }
    // TX_JOB_SUBMIT
    if (pkt->getAddr() == 1073756352) {
        printf("[LOG], %llu, %s, %s, %d, TX_JOB_SUBMIT\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }

    uint64_t mbuf_addr1[] = { // RX_JOB_ID 7
        // for zero-copy
        // 0x201095200, 0x201094880, 0x201093f00, 0x201093580, 0x201092c00, 0x201092280, 
        // 0x201091900, 0x201090f80, 0x201090600, 0x20108fc80, 0x20108f300, 0x20108e980, 
        // 0x20108e000, 0x20108d680, 0x20108cd00, 0x20108c380, 0x20108ba00, 0x20108b080, 
        // 0x20108a700, 0x201089d80, 0x201089400, 0x201088a80, 0x201088100, 0x201087780, 
        // 0x201086e00, 0x201086480, 0x201085b00, 0x201085180, 0x201084800, 0x201083e80, 
        // 0x201083500, 0x201082b80

        // for non zero-copy
        0x2010a8200, 0x2010a7880, 0x2010a6f00, 0x2010a6580, 0x2010a5c00, 0x2010a5280,
        0x2010a4900, 0x2010a3f80, 0x2010a3600, 0x2010a2c80, 0x2010a2300, 0x2010a1980, 0x2010a1000, 0x2010a0680, 
        0x20109fd00, 0x20109f380, 0x20109ea00, 0x20109e080, 0x20109d700, 0x20109cd80, 0x20109c400, 0x20109ba80,
        0x20109b100, 0x20109a780, 0x201099e00, 0x201099480, 0x201098b00, 0x201098180, 0x201097800, 0x201096e80,
        0x201096500, 0x201095b80
    };

    uint64_t mbuf_addr2[] = { // RX_JOB_ID 9
        0x20106f200, 0x20106e880, 0x20106df00, 0x20106d580, 0x20106cc00, 0x20106c280,
        0x20106b900, 0x20106af80, 0x20106a600, 0x201069c80, 0x201069300, 0x201068980,
        0x201068000, 0x201067680, 0x201066d00, 0x201066380, 0x201065a00, 0x201065080,
        0x201064700, 0x201063d80, 0x201063400, 0x201062a80, 0x201062100, 0x201061780,
        0x201060e00, 0x201060480, 0x20105fb00, 0x20105f180, 0x20105e800, 0x20105de80,
        0x20105d500, 0x20105cb80
    };

    uint64_t desc_rx[] = {
        // for zero-copy
        // 8624136384, 8624136448, 8624136512, 8624136576, 8624136640, 8624136704, 8624136768, 8624136832

        // for non-zero-copy
        0x20209e0c0, 0x20209e100, 0x20209e140, 0x20209e180, 0x20209e1c0, 0x20209e200, 0x20209e240, 0x20209e280
    };

    uint64_t desc_tx[] = {
        // for zero-copy
        // 8624220032, 8624220096, 8624220160, 8624220224

        // for non-zero-copy
        0x2020b2780, 0x2020b27c0, 0x2020b2800, 0x2020b2840
    };

    // for zero-copy
    // uint64_t comp_rx = 0x20209e080;
    // uint64_t comp_tx = 0x2020b2c00;

    // for non-zero-copy
    uint64_t comp_rx = 0x20209e080;
    uint64_t comp_tx = 0x2020b2c00;


    // MBUF_1
    for (int i = 0; i < 32; i++) {
        if (pkt->getAddr() == mbuf_addr1[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_1[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }

    // MBUF_2
    for (int i = 0; i < 32; i++) {
        if (pkt->getAddr() == mbuf_addr2[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_2[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }

    // DESC_RX
    for (int i = 0; i < 8; i++) {
        if (pkt->getAddr() == desc_rx[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_RX[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }
    
    // DESC_TX
    for (int i = 0; i < 4; i++) {
        if (pkt->getAddr() == desc_tx[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_TX[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }

    // COMP_RX
    if (pkt->getAddr() == comp_rx) {
        printf("[LOG], %llu, %s, %s, %d, COMP_RX\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0);
    }
    // COMP_TX
    if (pkt->getAddr() == comp_tx) {
        printf("[LOG], %llu, %s, %s, %d, COMP_TX\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0);
    }

    #endif

    #if LOG_LEVEL == 3
    uint64_t comp_rx = 0x20209b800;
    uint64_t comp_rx2 = 0x20209b380;
    uint64_t comp_tx = 0x2020b4b80;
    uint64_t comp_tx2 = 0x2020b4600;
    
    uint64_t mbuf_arr_rx = 0x202099300;
    uint64_t mbuf_arr_rx2 = 0x202097280;

    // COMP_RX
    if (pkt->getAddr() == comp_rx) {
        int64_t comp_val = 1111111111;
        if (pkt->isRead() && satisfied) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        } else if (pkt->isWrite()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        }
        printf("[LOG], %llu, %s, %s, %ld, COMP_RX_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), comp_val);
    }
    if (pkt->getAddr() == comp_rx2) {
        int64_t comp_val = 1111111111;
        if (pkt->isRead() && satisfied) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        } else if (pkt->isWrite()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        }
        printf("[LOG], %llu, %s, %s, %ld, COMP_RX2_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), comp_val);
    }
    // COMP_TX
    if (pkt->getAddr() == comp_tx) {   
        int64_t comp_val = 1111111111;
        if (pkt->isRead() && satisfied) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        } else if (pkt->isWrite()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        }        
        printf("[LOG], %llu, %s, %s, %ld, COMP_TX_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), comp_val);
    }
    if (pkt->getAddr() == comp_tx2) {
        int64_t comp_val = 1111111111;
        if (pkt->isRead() && satisfied) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        } else if (pkt->isWrite()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        }
        printf("[LOG], %llu, %s, %s, %ld, COMP_TX2_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), comp_val);
    }

    // MBUF_ARR_RX
    if (pkt->getAddr() == mbuf_arr_rx) {
        int64_t pointer_addr = 1111111111;
        if (pkt->isRead() && satisfied) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            pointer_addr = data[0];
        } else if (pkt->isWrite()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            pointer_addr = data[0];
        }
        printf("[LOG], %llu, %s, %s, %d, MBUF_ARR_RX_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pointer_addr);
    }
    if (pkt->getAddr() == mbuf_arr_rx2) {
        int64_t pointer_addr = 1111111111;
        if (pkt->isRead() && satisfied) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            pointer_addr = data[0];
        } else if (pkt->isWrite()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            pointer_addr = data[0];
        }
        printf("[LOG], %llu, %s, %s, %d, MBUF_ARR_RX2_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), pointer_addr);
    }


    // RX_JOB_SUBMIT
    if (pkt->getAddr() == 1073752256) {
        printf("[LOG], %llu, %s, %s, %d, RX_JOB_SUBMIT\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }
    // TX_JOB_SUBMIT
    if (pkt->getAddr() == 1073756352) {
        printf("[LOG], %llu, %s, %s, %d, TX_JOB_SUBMIT\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }

    // RX_JOB_ID 12
    uint64_t mbuf_addr_set_RXJ_12[] = {
        0x201321D80, 0x201322700, 0x201323080, 0x201323A00,
        0x201324380, 0x201324D00, 0x201325680, 0x201326000,
        0x201326980, 0x201327300, 0x201327C80, 0x201328600,
        0x201328F80, 0x201329900, 0x20132A280, 0x20132AC00,
        0x20132B580, 0x20132BF00, 0x20132C880, 0x20132D200,
        0x20132DB80, 0x20132E500, 0x20132EE80, 0x20132F800,
        0x201330180, 0x201330B00, 0x201331480, 0x201331E00,
        0x201332780, 0x201333100, 0x201333A80, 0x201334400
    };

    uint64_t mbuf_first_cacheline_set_RXJ_12[] = {
        0x201321C80, 0x201322600, 0x201322F80, 0x201323900,
        0x201324280, 0x201324C00, 0x201325580, 0x201325F00,
        0x201326880, 0x201327200, 0x201327B80, 0x201328500,
        0x201328E80, 0x201329800, 0x20132A180, 0x20132AB00,
        0x20132B480, 0x20132BE00, 0x20132C780, 0x20132D100,
        0x20132DA80, 0x20132E400, 0x20132ED80, 0x20132F700,
        0x201330080, 0x201330A00, 0x201331380, 0x201331D00,
        0x201332680, 0x201333000, 0x201333980, 0x201334300
    };

    uint64_t mbuf_second_cacheline_set_RXJ_12[] = {
        0x201321CC0, 0x201322640, 0x201322FC0, 0x201323940,
        0x2013242C0, 0x201324C40, 0x2013255C0, 0x201325F40,
        0x2013268C0, 0x201327240, 0x201327BC0, 0x201328540,
        0x201328EC0, 0x201329840, 0x20132A1C0, 0x20132AB40,
        0x20132B4C0, 0x20132BE40, 0x20132C7C0, 0x20132D140,
        0x20132DAC0, 0x20132E440, 0x20132EDC0, 0x20132F740,
        0x2013300C0, 0x201330A40, 0x2013313C0, 0x201331D40,
        0x2013326C0, 0x201333040, 0x2013339C0, 0x201334340
    };

    uint64_t mbuf_addr_set_RXJ_10[] = {
        0x20131D180, 0x20131DB00, 0x20131E480, 0x20131EE00,
        0x20131F780, 0x201320100, 0x201320A80, 0x201321400,
        0x201318580, 0x201318F00, 0x201319880, 0x20131A200,
        0x20131AB80, 0x20131B500, 0x20131BE80, 0x20131C800,
        0x201313980, 0x201314300, 0x201314C80, 0x201315600,
        0x201315F80, 0x201316900, 0x201317280, 0x201317C00,
        0x20130ED80, 0x20130F700, 0x201310080, 0x201310A00,
        0x201311380, 0x201311D00, 0x201312680, 0x201313000
    };

    uint64_t mbuf_first_cacheline_set_RXJ_10[] = {
        0x20131D080, 0x20131DA00, 0x20131E380, 0x20131ED00,
        0x20131F680, 0x201320000, 0x201320980, 0x201321300,
        0x201318480, 0x201318E00, 0x201319780, 0x20131A100,
        0x20131AA80, 0x20131B400, 0x20131BD80, 0x20131C700,
        0x201313880, 0x201314200, 0x201314B80, 0x201315500,
        0x201315E80, 0x201316800, 0x201317180, 0x201317B00,
        0x20130EC80, 0x20130F600, 0x20130FF80, 0x201310900,
        0x201311280, 0x201311C00, 0x201312580, 0x201312F00
    };

    uint64_t mbuf_second_cacheline_set_RXJ_10[] = {
        0x20131D0C0, 0x20131DA40, 0x20131E3C0, 0x20131ED40,
        0x20131F6C0, 0x201320040, 0x2013209C0, 0x201321340,
        0x2013184C0, 0x201318E40, 0x2013197C0, 0x20131A140,
        0x20131AAC0, 0x20131B440, 0x20131BDC0, 0x20131C740,
        0x2013138C0, 0x201314240, 0x201314BC0, 0x201315540,
        0x201315EC0, 0x201316840, 0x2013171C0, 0x201317B40,
        0x20130ECC0, 0x20130F640, 0x20130FFC0, 0x201310940,
        0x2013112C0, 0x201311C40, 0x2013125C0, 0x201312F40
    };

    uint64_t mbuf_addr_set_RXJ_11[] = {
        0x20130A180, 0x20130AB00, 0x20130B480, 0x20130BE00,
        0x20130C780, 0x20130D100, 0x20130DA80, 0x20130E400,
        0x201305580, 0x201305F00, 0x201306880, 0x201307200,
        0x201307B80, 0x201308500, 0x201308E80, 0x201309800,
        0x201300980, 0x201301300, 0x201301C80, 0x201302600,
        0x201302F80, 0x201303900, 0x201304280, 0x201304C00,
        0x2012FBD80, 0x2012FC700, 0x2012FD080, 0x2012FDA00,
        0x2012FE380, 0x2012FED00, 0x2012FF680, 0x201300000
    };

    uint64_t mbuf_first_cacheline_set_RXJ_11[] = {
        0x20130A080, 0x20130AA00, 0x20130B380, 0x20130BD00,
        0x20130C680, 0x20130D000, 0x20130D980, 0x20130E300,
        0x201305480, 0x201305E00, 0x201306780, 0x201307100,
        0x201307A80, 0x201308400, 0x201308D80, 0x201309700,
        0x201300880, 0x201301200, 0x201301B80, 0x201302500,
        0x201302E80, 0x201303800, 0x201304180, 0x201304B00,
        0x2012FBC80, 0x2012FC600, 0x2012FCF80, 0x2012FD900,
        0x2012FE280, 0x2012FEC00, 0x2012FF580, 0x2012FFF00
    };

    uint64_t mbuf_second_cacheline_set_RXJ_11[] = {
        0x20130A0C0, 0x20130AA40, 0x20130B3C0, 0x20130BD40,
        0x20130C6C0, 0x20130D040, 0x20130D9C0, 0x20130E340,
        0x2013054C0, 0x201305E40, 0x2013067C0, 0x201307140,
        0x201307AC0, 0x201308440, 0x201308DC0, 0x201309740,
        0x2013008C0, 0x201301240, 0x201301BC0, 0x201302540,
        0x201302EC0, 0x201303840, 0x2013041C0, 0x201304B40,
        0x2012FBCC0, 0x2012FC640, 0x2012FCFC0, 0x2012FD940,
        0x2012FE2C0, 0x2012FEC40, 0x2012FF5C0, 0x2012FFF40
    };

    uint64_t mbuf_addr_set_RXJ_13[] = {
        0x20130ED80, 0x20130F700, 0x201310080, 0x201310A00,
        0x201311380, 0x201311D00, 0x201312680, 0x201313000,
        0x201313980, 0x201314300, 0x201314C80, 0x201315600,
        0x201315F80, 0x201316900, 0x201317280, 0x201317C00,
        0x201318580, 0x201318F00, 0x201319880, 0x20131A200,
        0x20131AB80, 0x20131B500, 0x20131BE80, 0x20131C800,
        0x20131D180, 0x20131DB00, 0x20131E480, 0x20131EE00,
        0x20131F780, 0x201320100, 0x201320A80, 0x201321400
    };

    uint64_t mbuf_first_cacheline_set_RXJ_13[] = {
        0x20130EC80, 0x20130F600, 0x20130FF80, 0x201310900,
        0x201311280, 0x201311C00, 0x201312580, 0x201312F00,
        0x201313880, 0x201314200, 0x201314B80, 0x201315500,
        0x201315E80, 0x201316800, 0x201317180, 0x201317B00,
        0x201318480, 0x201318E00, 0x201319780, 0x20131A100,
        0x20131AA80, 0x20131B400, 0x20131BD80, 0x20131C700,
        0x20131D080, 0x20131DA00, 0x20131E380, 0x20131ED00,
        0x20131F680, 0x201320000, 0x201320980, 0x201321300
    };

    uint64_t mbuf_second_cacheline_set_RXJ_13[] = {
        0x20130ECC0, 0x20130F640, 0x20130FFC0, 0x201310940,
        0x2013112C0, 0x201311C40, 0x2013125C0, 0x201312F40,
        0x2013138C0, 0x201314240, 0x201314BC0, 0x201315540,
        0x201315EC0, 0x201316840, 0x2013171C0, 0x201317B40,
        0x2013184C0, 0x201318E40, 0x2013197C0, 0x20131A140,
        0x20131AAC0, 0x20131B440, 0x20131BDC0, 0x20131C740,
        0x20131D0C0, 0x20131DA40, 0x20131E3C0, 0x20131ED40,
        0x20131F6C0, 0x201320040, 0x2013209C0, 0x201321340
    };



    uint64_t desc_rx1[] = {
        0x20209B840, 0x20209B880, 0x20209B8C0, 0x20209B900, 0x20209B940, 0x20209B980, 0x20209B9C0, 0x20209BA00
    };

    uint64_t desc_rx2[] = {
        0x20209B3C0, 0x20209B400, 0x20209B440, 0x20209B480, 0x20209B4C0, 0x20209B500, 0x20209B540, 0x20209B580
    };

    uint64_t desc_tx1[] = {
        0x2020b4700, 0x2020b4740, 0x2020b4780, 0x2020b47c0
    };

    uint64_t desc_tx2[] = {
        0x2020b4180, 0x2020b41c0, 0x2020b4200, 0x2020b4240
    };

    for (int i = 0; i < 32; i ++) {
        uint64_t pkt_addr = pkt->getAddr();
        uint64_t pkt_addr_end = pkt_addr + pkt->getSize();
        if (pkt_addr >= mbuf_first_cacheline_set_RXJ_12[i] && pkt_addr_end <= mbuf_first_cacheline_set_RXJ_12[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_RXJ12[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_RXJ_12[i] && pkt_addr_end <= mbuf_second_cacheline_set_RXJ_12[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_RXJ12[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_RXJ_10[i] && pkt_addr_end <= mbuf_first_cacheline_set_RXJ_10[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_RXJ10[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_RXJ_10[i] && pkt_addr_end <= mbuf_second_cacheline_set_RXJ_10[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_RXJ10[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_RXJ_11[i] && pkt_addr_end <= mbuf_first_cacheline_set_RXJ_11[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_RXJ11[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_RXJ_11[i] && pkt_addr_end <= mbuf_second_cacheline_set_RXJ_11[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_RXJ11[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_RXJ_13[i] && pkt_addr_end <= mbuf_first_cacheline_set_RXJ_13[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_RXJ13[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_RXJ_13[i] && pkt_addr_end <= mbuf_second_cacheline_set_RXJ_13[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_RXJ13[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }

    for (int i = 0; i < 32; i++) {
        if (pkt->getAddr() == mbuf_addr_set_RXJ_12[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_RXJ12[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_RXJ_10[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_RXJ10[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_RXJ_11[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_RXJ11[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_RXJ_13[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_RXJ13[%d]_REQ\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }

    // DESC_RX
    for (int i = 0; i < 8; i++) {
        if (pkt->getAddr() == desc_rx1[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_RX1_REQ[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }

    for (int i = 0; i <8; i++) {
        if (pkt->getAddr() == desc_rx2[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_RX2_REQ[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }
    
    // DESC_TX
    for (int i = 0; i < 4; i++) {
        if (pkt->getAddr() == desc_tx1[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_TX1_REQ[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }

    for (int i = 0; i < 4; i++) {
        if (pkt->getAddr() == desc_tx2[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_TX2_REQ[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), satisfied ? 1 : 0, i);
        }
    }

    #endif
    
    // Here we charge the headerDelay that takes into account the latencies
    // of the bus, if the packet comes from it.
    // The latency charged is just the value set by the access() function.
    // In case of a hit we are neglecting response latency.
    // In case of a miss we are neglecting forward latency.
    Tick request_time = clockEdge(lat);
    // Here we reset the timing of the packet.
    pkt->headerDelay = pkt->payloadDelay = 0;

    if(isMLC && pkt->isPrefetchHintPkt()){
        ppDdioHint->notify(pkt);
    }

    if (satisfied) {
        // notify before anything else as later handleTimingReqHit might turn
        // the packet in a response
        ppHit->notify(pkt);

        if (prefetcher && blk && blk->wasPrefetched()) {
            DPRINTF(Cache, "Hit on prefetch for addr %#x (%s)\n",
                    pkt->getAddr(), pkt->isSecure() ? "s" : "ns");
            blk->clearPrefetched();
        }

        handleTimingReqHit(pkt, blk, request_time, cpu_side_port_id);

        // SHIN.
        if(isIOCache){
            if(pkt->cmd==MemCmd::WriteReq || pkt->cmd==MemCmd::WriteLineReq){
                // Not Works
                blk->setDdioPrefetchDestination(pkt->getDdioPrefetchDestination());
                
                if(pkt->isDdioHeader()) blk->setDdioHeader();

                DPRINTF(AdaptiveDdioCache, "recvTimingReq qid %d, pkt %s\n", pkt->getDdioPrefetchDestination(), pkt->print());
            }
        }
    } else {
        handleTimingReqMiss(pkt, blk, forward_time, request_time); //make MSHR and send request

        ppMiss->notify(pkt);
    }

    if (prefetcher) {
        // track time of availability of next prefetch, if any
        Tick next_pf_time = prefetcher->nextPrefetchReadyTime();
        if (next_pf_time != MaxTick) {
            schedMemSideSendEvent(next_pf_time);
        }
    }
}

void
BaseCache::handleUncacheableWriteResp(PacketPtr pkt)
{
    Tick completion_time = clockEdge(responseLatency) +
        pkt->headerDelay + pkt->payloadDelay;

    // Reset the bus additional time as it is now accounted for
    pkt->headerDelay = pkt->payloadDelay = 0;

    if (isLLC && isMultiPort) {
        assert(pkt->cpu_side_port_id != InvalidPortID);
        assert(pkt->cpu_side_port_id < cpuSidePortList.size());
        cpuSidePortList[pkt->cpu_side_port_id]->schedTimingResp(pkt,
                                                                completion_time);
    } else if (isIOCache && pkt->isFromDTA()) {
        // This packet is from DTA, send it to DTA
        // Find the worker by using sender state
        assert(dta != nullptr);
        dta->recvTimingRespfromCache(pkt, completion_time);
    } else {
        assert(!(isIOCache && enableDTA));
        cpuSidePort.schedTimingResp(pkt, completion_time); 
    }
}

void
BaseCache::recvTimingResp(PacketPtr pkt)
{
    assert(pkt->isResponse());

    // all header delay should be paid for by the crossbar, unless
    // this is a prefetch response from above
    panic_if(pkt->headerDelay != 0 && pkt->cmd != MemCmd::HardPFResp,
             "%s saw a non-zero packet delay\n", name());

    const bool is_error = pkt->isError();

    if (is_error) {
        DPRINTF(Cache, "%s: Cache received %s with error\n", __func__,
                pkt->print());
    }

    DPRINTF(Cache, "%s: Handling response %s\n", __func__,
            pkt->print());

    #if LOG_LEVEL == 1
    // For Ring Buffer LOG
    if (pkt->getAddr() == 1073752088) {
        printf("[LOG], %llu, %s, %s, %d, RX_TAIL_WR_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }
    if (pkt->getAddr() == 1073756184) {
        printf("[LOG], %llu, %s, %s, %d, TX_TAIL_WR_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }

    uint64_t rx_sw_ring_base = 0x20209bc80;
    uint64_t tx_sw_ring_base = 0x2020b4c80;
    uint64_t max_desc = 1024;
    uint64_t mbuf_ptr_size = 8;

    uint64_t rx_sw_ring_end = rx_sw_ring_base + max_desc * mbuf_ptr_size;
    uint64_t tx_sw_ring_end = tx_sw_ring_base + max_desc * mbuf_ptr_size;

    // RX SW Ring
    // if (pkt->getAddr() >= rx_sw_ring_base && pkt->getAddr() < rx_sw_ring_end) {
    //     int offset = (pkt->getAddr() - rx_sw_ring_base) / mbuf_ptr_size;
    //     printf("[LOG], %llu, %s, %s, %d, RX_SW_RING[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, offset);
    // }
    // // TX SW Ring
    // if (pkt->getAddr() >= tx_sw_ring_base && pkt->getAddr() < tx_sw_ring_end) {
    //     int offset = (pkt->getAddr() - tx_sw_ring_base) / mbuf_ptr_size;
    //     printf("[LOG], %llu, %s, %s, %d, TX_SW_RING[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, offset);
    // }

    // normal version
    // uint64_t rx_desc_base = 8624155648;
    // uint64_t tx_desc_base = 8624238080;

    // sve version
    uint64_t rx_desc_base = 8624135424;
    uint64_t tx_desc_base = 8624237824;

    uint64_t batch_size = 64;
    uint64_t desc_size = 16;

    // RX Descriptor (0-63)
    uint64_t rx_desc_0 = rx_desc_base + desc_size * 0;
    if (pkt->getAddr() >= rx_desc_0 && pkt->getAddr() < rx_desc_0 + batch_size * desc_size) {
        int offset = (pkt->getAddr() - rx_desc_0) / desc_size;
        int num_desc = (pkt->getSize() / desc_size);  
        int last_offset = offset + num_desc - 1;
        if ((last_offset == 63 || last_offset == 62) && pkt->canGetDataPtr()) {
            // Get the last descriptor's DD bit
            uint8_t *data = pkt->getPtr<uint8_t>();
            uint8_t *last_desc = data + (num_desc - 1) * desc_size;
            E1000RXDescriptor *desc = (E1000RXDescriptor *)last_desc;
            bool dd = false;
            if (pkt->isRead()) {
                dd = desc->status_error & E1000_RXD_STAT_DD;
            }
            printf("[LOG], %llu, %s, %s, %d, RX_DESC_0_63[%d]_RES_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
        } else {
            printf("[LOG], %llu, %s, %s, %d, RX_DESC_0_63[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, offset);
        }
    }

    // RX Descriptor (64-127)
    uint64_t rx_desc_64 = rx_desc_base + desc_size * 64;
    if (pkt->getAddr() >= rx_desc_64 && pkt->getAddr() < rx_desc_64 + batch_size * desc_size) {
        int offset = (pkt->getAddr() - rx_desc_64) / desc_size;
        int num_desc = (pkt->getSize() / desc_size);
        int last_offset = offset + num_desc - 1;
        if ((last_offset == 63 || last_offset == 62) && pkt->canGetDataPtr()) {
            // Get the last descriptor's DD bit
            uint8_t *data = pkt->getPtr<uint8_t>();
            uint8_t *last_desc = data + (num_desc - 1) * desc_size;
            E1000RXDescriptor *desc = (E1000RXDescriptor *)last_desc;
            bool dd = false;
            if (pkt->isRead()) {
                dd = desc->status_error & E1000_RXD_STAT_DD;
            }
            printf("[LOG], %llu, %s, %s, %d, RX_DESC_64_127[%d]_RES_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
        } else {
            printf("[LOG], %llu, %s, %s, %d, RX_DESC_64_127[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, offset);
        }
    }

    // RX Descriptor (192-255)
    uint64_t rx_desc_192 = rx_desc_base + desc_size * 192;
    if (pkt->getAddr() >= rx_desc_192 && pkt->getAddr() < rx_desc_192 + batch_size * desc_size) {
        int offset = (pkt->getAddr() - rx_desc_192) / desc_size;
        int num_desc = (pkt->getSize() / desc_size);
        int last_offset = offset + num_desc - 1;
        if ((last_offset == 63 || last_offset == 62) && pkt->canGetDataPtr()) {
            // Get the last descriptor's DD bit
            uint8_t *data = pkt->getPtr<uint8_t>();
            uint8_t *last_desc = data + (num_desc - 1) * desc_size;
            E1000RXDescriptor *desc = (E1000RXDescriptor *)last_desc;
            bool dd = false;
            if (pkt->isRead()) {
                dd = desc->status_error & E1000_RXD_STAT_DD;
            }
            printf("[LOG], %llu, %s, %s, %d, RX_DESC_192_255[%d]_RES_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
        } else {
            printf("[LOG], %llu, %s, %s, %d, RX_DESC_192_255[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, offset);
        }
    }

    // TX Descriptor (128-192)
    uint64_t tx_desc_128 = tx_desc_base + desc_size * 128;
    if (pkt->getAddr() >= tx_desc_128 && pkt->getAddr() < tx_desc_128 + batch_size * desc_size) {
        int offset = (pkt->getAddr() - tx_desc_128) / desc_size;
        int num_desc = (pkt->getSize() / desc_size);
        int last_offset = offset + num_desc - 1;
        if ((last_offset == 63) && pkt->canGetDataPtr()) {
            // Get the last descriptor's DD bit
            uint8_t *data = pkt->getPtr<uint8_t>();
            uint8_t *last_desc = data + (num_desc - 1) * desc_size;
            E1000TXDescriptor *desc = (E1000TXDescriptor *)last_desc;
            bool dd = false;
            if (pkt->isRead()) {
                dd = desc->wb.status & E1000_TXD_STAT_DD;
            }
            printf("[LOG], %llu, %s, %s, %d, TX_DESC_128_192[%d]_RES_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
        } else {
            printf("[LOG], %llu, %s, %s, %d, TX_DESC_128_192[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, offset);
        }
    }

    // TX Descriptor (192-256)
    uint64_t tx_desc_192 = tx_desc_base + desc_size * 192;
    if (pkt->getAddr() >= tx_desc_192 && pkt->getAddr() < tx_desc_192 + batch_size * desc_size) {
        int offset = (pkt->getAddr() - tx_desc_192) / desc_size;
        int num_desc = (pkt->getSize() / desc_size);
        int last_offset = offset + num_desc - 1;
        if ((last_offset == 63) && pkt->canGetDataPtr()) {
            // Get the last descriptor's DD bit
            uint8_t *data = pkt->getPtr<uint8_t>();
            uint8_t *last_desc = data + (num_desc - 1) * desc_size;
            E1000TXDescriptor *desc = (E1000TXDescriptor *)last_desc;
            bool dd = false;
            if (pkt->isRead()) {
                dd = desc->wb.status & E1000_TXD_STAT_DD;
            }
            printf("[LOG], %llu, %s, %s, %d, TX_DESC_192_256[%d]_RES_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
        }
        else {
            printf("[LOG], %llu, %s, %s, %d, TX_DESC_192_256[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, offset);
        }
    }

    // TX Descriptor (256-320)
    uint64_t tx_desc_256 = tx_desc_base + desc_size * 256;
    if (pkt->getAddr() >= tx_desc_256 && pkt->getAddr() < tx_desc_256 + batch_size * desc_size) {
        int offset = (pkt->getAddr() - tx_desc_256) / desc_size;
        int num_desc = (pkt->getSize() / desc_size);
        int last_offset = offset + num_desc - 1;
        if ((last_offset == 63) && pkt->canGetDataPtr()) {
            // Get the last descriptor's DD bit
            uint8_t *data = pkt->getPtr<uint8_t>();
            uint8_t *last_desc = data + (num_desc - 1) * desc_size;
            E1000TXDescriptor *desc = (E1000TXDescriptor *)last_desc;
            bool dd = false;
            if (pkt->isRead()) {
                dd = desc->wb.status & E1000_TXD_STAT_DD;
            }
            printf("[LOG], %llu, %s, %s, %d, TX_DESC_256_320[%d]_RES_DD\n", curTick(), name().c_str(), pkt->print().c_str(), dd, last_offset);
        } else {
            printf("[LOG], %llu, %s, %s, %d, TX_DESC_256_320[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, offset);
        }
    }

    /*
    // mbuf for RX Descriptor (63-95 pre-prev) - for tx mbuf free
    uint64_t mbuf_addr_set_95_preprev[] = {
        0x201650D80, 0x201651700, 0x201652080, 0x201652A00,
        0x201653380, 0x201653D00, 0x201654680, 0x201655000,
        0x201655980, 0x201656300, 0x201656C80, 0x201657600,
        0x201657F80, 0x201658900, 0x201659280, 0x201659C00,
        0x20165A580, 0x20165AF00, 0x20165B880, 0x20165C200,
        0x20165CB80, 0x20165D500, 0x20165DE80, 0x20165E800,
        0x20165F180, 0x20165FB00, 0x201660480, 0x201660E00,
        0x201661780, 0x201662100, 0x201662A80, 0x201663400
    };

    uint64_t mbuf_first_cacheline_set_95_preprev[] = {
        0x201650C80, 0x201651600, 0x201651F80, 0x201652900,
        0x201653280, 0x201653C00, 0x201654580, 0x201654F00,
        0x201655880, 0x201656200, 0x201656B80, 0x201657500,
        0x201657E80, 0x201658800, 0x201659180, 0x201659B00,
        0x20165A480, 0x20165AE00, 0x20165B780, 0x20165C100,
        0x20165CA80, 0x20165D400, 0x20165DD80, 0x20165E700,
        0x20165F080, 0x20165FA00, 0x201660380, 0x201660D00,
        0x201661680, 0x201662000, 0x201662980, 0x201663300
    };

    uint64_t mbuf_second_cacheline_set_95_preprev[] = {
        0x201650CC0, 0x201651640, 0x201651FC0, 0x201652940,
        0x2016532C0, 0x201653C40, 0x2016545C0, 0x201654F40,
        0x2016558C0, 0x201656240, 0x201656BC0, 0x201657540,
        0x201657EC0, 0x201658840, 0x2016591C0, 0x201659B40,
        0x20165A4C0, 0x20165AE40, 0x20165B7C0, 0x20165C140,
        0x20165CAC0, 0x20165D440, 0x20165DDC0, 0x20165E740,
        0x20165F0C0, 0x20165FA40, 0x2016603C0, 0x201660D40,
        0x2016616C0, 0x201662040, 0x2016629C0, 0x201663340
    };

    // mbuf for RX Descriptor (63-95 prev)
    uint64_t mbuf_addr_set_95_prev[] = {
        0x20154c000, 0x20154c980, 0x20154d300, 0x20154dc80, 
        0x20154e600, 0x20154ef80, 0x20154f900, 0x201550280, 
        0x201550c00, 0x201551580, 0x201551f00, 0x201552880, 
        0x201553200, 0x201553b80, 0x201554500, 0x201554e80, 
        0x201555800, 0x201556180, 0x201556b00, 0x201557480,
        0x201557e00, 0x201558780, 0x201559100, 0x201559a80, 
        0x20155a400, 0x20155ad80, 0x20155b700, 0x20155c080, 
        0x20155ca00, 0x20155d380, 0x20155dd00, 0x20155e680
    };

    // mbuf's first cacheline for RX Descriptor (63-95 prev)
    uint64_t mbuf_first_cacheline_set_95_prev[] = {
        0x20154bf00, 0x20154c880, 0x20154d200, 0x20154db80, 
        0x20154e500, 0x20154ee80, 0x20154f800, 0x201550180, 
        0x201550b00, 0x201551480, 0x201551e00, 0x201552780, 
        0x201553100, 0x201553a80, 0x201554400, 0x201554d80, 
        0x201555700, 0x201556080, 0x201556a00, 0x201557380, 
        0x201557d00, 0x201558680, 0x201559000, 0x201559980, 
        0x20155a300, 0x20155ac80, 0x20155b600, 0x20155bf80, 
        0x20155c900, 0x20155d280, 0x20155dc00, 0x20155e580
    };

    // mbuf's second cacheline for RX Descriptor (63-95 prev)
    uint64_t mbuf_second_cacheline_set_95_prev[] = {
        0x20154bf40, 0x20154c8c0, 0x20154d240, 0x20154dbc0,
        0x20154e540, 0x20154eec0, 0x20154f840, 0x2015501c0,
        0x201550b40, 0x2015514c0, 0x201551e40, 0x2015527c0,
        0x201553140, 0x201553ac0, 0x201554440, 0x201554dc0,
        0x201555740, 0x2015560c0, 0x201556a40, 0x2015573c0,
        0x201557d40, 0x2015586c0, 0x201559040, 0x2015599c0,
        0x20155a340, 0x20155acc0, 0x20155b640, 0x20155bfc0,
        0x20155c940, 0x20155d2c0, 0x20155dc40, 0x20155e5c0
    };

    // mbuf for RX Descriptor (63-95 new)
    uint64_t mbuf_addr_set_95_new[] = {
        0x201676d80, 0x201676400, 0x201675a80, 0x201675100,
        0x201674780, 0x201673e00, 0x201673480, 0x201672b00,
        0x201672180, 0x201671800, 0x201670e80, 0x201670500,
        0x20166fb80, 0x20166f200, 0x20166e880, 0x20166df00,
        0x20166d580, 0x20166cc00, 0x20166c280, 0x20166b900,
        0x20166af80, 0x20166a600, 0x201669c80, 0x201669300,
        0x201668980, 0x201668000, 0x201667680, 0x201666d00,
        0x201666380, 0x201665a00, 0x201665080, 0x201664700
    };

    // mbuf's first cacheline for RX Descriptor (63-95 new) - minus 256 from the mbuf_addr_set_95_new
    uint64_t mbuf_first_cacheline_set_95_new[] = {
        0x201676c80, 0x201676300, 0x201675980, 0x201675000,
        0x201674680, 0x201673d00, 0x201673380, 0x201672a00,
        0x201672080, 0x201671700, 0x201670d80, 0x201670400,
        0x20166fa80, 0x20166f100, 0x20166e780, 0x20166de00,
        0x20166d480, 0x20166cb00, 0x20166c180, 0x20166b800,
        0x20166ae80, 0x20166a500, 0x201669b80, 0x201669200,
        0x201668880, 0x201667f00, 0x201667580, 0x201666c00,
        0x201666280, 0x201665900, 0x201664f80, 0x201664600
    };

    // mbuf's second cacheline for RX Descriptor (63-95 new) + 64 from the mbuf_first_cacheline_set_95_new
    uint64_t mbuf_second_cacheline_set_95_new[] = {
        0x201676cc0, 0x201676340, 0x2016759c0, 0x201675040,
        0x2016746c0, 0x201673d40, 0x2016733c0, 0x201672a40,
        0x2016720c0, 0x201671740, 0x201670dc0, 0x201670440,
        0x20166fac0, 0x20166f140, 0x20166e7c0, 0x20166de40,
        0x20166d4c0, 0x20166cb40, 0x20166c1c0, 0x20166b840,
        0x20166aec0, 0x20166a540, 0x201669bc0, 0x201669240,
        0x2016688c0, 0x201667f40, 0x2016675c0, 0x201666c40,
        0x2016662c0, 0x201665940, 0x201664fc0, 0x201664640
    };

    // mbuf for RX Descriptor (95-127 prev) -> TX Descriptor (128-160)
    uint64_t mbuf_addr_set_127_prev[] = {
        0x201539000, 0x201539980, 0x20153A300, 0x20153AC80,
        0x20153B600, 0x20153BF80, 0x20153C900, 0x20153D280,
        0x20153DC00, 0x20153E580, 0x20153EF00, 0x20153F880,
        0x201540200, 0x201540B80, 0x201541500, 0x201541E80,
        0x201542800, 0x201543180, 0x201543B00, 0x201544480,
        0x201544E00, 0x201545780, 0x201546100, 0x201546A80,
        0x201547400, 0x201547D80, 0x201548700, 0x201549080,
        0x201549A00, 0x20154A380, 0x20154AD00, 0x20154B680
    };

    uint64_t mbuf_first_cacheline_set_127_prev[] = {
        0x201538F00, 0x201539880, 0x20153A200, 0x20153AB80,
        0x20153B500, 0x20153BE80, 0x20153C800, 0x20153D180,
        0x20153DB00, 0x20153E480, 0x20153EE00, 0x20153F780,
        0x201540100, 0x201540A80, 0x201541400, 0x201541D80,
        0x201542700, 0x201543080, 0x201543A00, 0x201544380,
        0x201544D00, 0x201545680, 0x201546000, 0x201546980,
        0x201547300, 0x201547C80, 0x201548600, 0x201548F80,
        0x201549900, 0x20154A280, 0x20154AC00, 0x20154B580
    };

    uint64_t mbuf_second_cacheline_set_127_prev[] = {
        0x201538F40, 0x2015398C0, 0x20153A240, 0x20153ABC0,
        0x20153B540, 0x20153BEC0, 0x20153C840, 0x20153D1C0,
        0x20153DB40, 0x20153E4C0, 0x20153EE40, 0x20153F7C0,
        0x201540140, 0x201540AC0, 0x201541440, 0x201541DC0,
        0x201542740, 0x2015430C0, 0x201543A40, 0x2015443C0,
        0x201544D40, 0x2015456C0, 0x201546040, 0x2015469C0,
        0x201547340, 0x201547CC0, 0x201548640, 0x201548FC0,
        0x201549940, 0x20154A2C0, 0x20154AC40, 0x20154B5C0
    };
    */

    // sve version
    // mbuf for TX Descriptor (128-191 current) - throught current phase's tdt 192 wr prefetch
    uint64_t mbuf_addr_set_192_cur[] = {
        0x2013B9D80, 0x2013BA700, 0x2013BB080, 0x2013BBA00,
        0x2013BC380, 0x2013BCD00, 0x2013BD680, 0x2013BE000,
        0x2013BE980, 0x2013BF300, 0x2013BFC80, 0x2013C0600,
        0x2013C0F80, 0x2013C1900, 0x2013C2280, 0x2013C2C00,
        0x2013C3580, 0x2013C3F00, 0x2013C4880, 0x2013C5200,
        0x2013C5B80, 0x2013C6500, 0x2013C6E80, 0x2013C7800,
        0x2013C8180, 0x2013C8B00, 0x2013C9480, 0x2013C9E00,
        0x2013CA780, 0x2013CB100, 0x2013CBA80, 0x2013CC400,
        0x2013CCD80, 0x2013CD700, 0x2013CE080, 0x2013CEA00,
        0x2013CF380, 0x2013CFD00, 0x2013D0680, 0x2013D1000,
        0x2013D1980, 0x2013D2300, 0x2013D2C80, 0x2013D3600,
        0x2013D3F80, 0x2013D4900, 0x2013D5280, 0x2013D5C00,
        0x2013D6580, 0x2013D6F00, 0x2013D7880, 0x2013D8200,
        0x2013D8B80, 0x2013D9500, 0x2013D9E80, 0x2013DA800,
        0x2013DB180, 0x2013DBB00, 0x2013DC480, 0x2013DCE00,
        0x2013DD780, 0x2013DE100, 0x2013DEA80, 0x2013DF400
    };

    uint64_t mbuf_first_cacheline_set_192_cur[] = {
        0x2013B9C80, 0x2013BA600, 0x2013BAF80, 0x2013BB900,
        0x2013BC280, 0x2013BCC00, 0x2013BD580, 0x2013BDF00,
        0x2013BE880, 0x2013BF200, 0x2013BFB80, 0x2013C0500,
        0x2013C0E80, 0x2013C1800, 0x2013C2180, 0x2013C2B00,
        0x2013C3480, 0x2013C3E00, 0x2013C4780, 0x2013C5100,
        0x2013C5A80, 0x2013C6400, 0x2013C6D80, 0x2013C7700,
        0x2013C8080, 0x2013C8A00, 0x2013C9380, 0x2013C9D00,
        0x2013CA680, 0x2013CB000, 0x2013CB980, 0x2013CC300,
        0x2013CCC80, 0x2013CD600, 0x2013CDF80, 0x2013CE900,
        0x2013CF280, 0x2013CFC00, 0x2013D0580, 0x2013D0F00,
        0x2013D1880, 0x2013D2200, 0x2013D2B80, 0x2013D3500,
        0x2013D3E80, 0x2013D4800, 0x2013D5180, 0x2013D5B00,
        0x2013D6480, 0x2013D6E00, 0x2013D7780, 0x2013D8100,
        0x2013D8A80, 0x2013D9400, 0x2013D9D80, 0x2013DA700,
        0x2013DB080, 0x2013DBA00, 0x2013DC380, 0x2013DCD00,
        0x2013DD680, 0x2013DE000, 0x2013DE980, 0x2013DF300
    };

    uint64_t mbuf_second_cacheline_set_192_cur[] = {
        0x2013B9CC0, 0x2013BA640, 0x2013BAFC0, 0x2013BB940,
        0x2013BC2C0, 0x2013BCC40, 0x2013BD5C0, 0x2013BDF40,
        0x2013BE8C0, 0x2013BF240, 0x2013BFBC0, 0x2013C0540,
        0x2013C0EC0, 0x2013C1840, 0x2013C21C0, 0x2013C2B40,
        0x2013C34C0, 0x2013C3E40, 0x2013C47C0, 0x2013C5140,
        0x2013C5AC0, 0x2013C6440, 0x2013C6DC0, 0x2013C7740,
        0x2013C80C0, 0x2013C8A40, 0x2013C93C0, 0x2013C9D40,
        0x2013CA6C0, 0x2013CB040, 0x2013CB9C0, 0x2013CC340,
        0x2013CCCC0, 0x2013CD640, 0x2013CDFC0, 0x2013CE940,
        0x2013CF2C0, 0x2013CFC40, 0x2013D05C0, 0x2013D0F40,
        0x2013D18C0, 0x2013D2240, 0x2013D2BC0, 0x2013D3540,
        0x2013D3EC0, 0x2013D4840, 0x2013D51C0, 0x2013D5B40,
        0x2013D64C0, 0x2013D6E40, 0x2013D77C0, 0x2013D8140,
        0x2013D8AC0, 0x2013D9440, 0x2013D9DC0, 0x2013DA740,
        0x2013DB0C0, 0x2013DBA40, 0x2013DC3C0, 0x2013DCD40,
        0x2013DD6C0, 0x2013DE040, 0x2013DE9C0, 0x2013DF340
    };
    
    // mbuf for RX Descriptor (64-127 new) - through next phase's tdt 128 wr prefetch
    uint64_t mbuf_addr_set_128_new[] = {
        0x201498B00, 0x201498180, 0x201497800, 0x201496E80,
        0x201496500, 0x201495B80, 0x201495200, 0x201494880,
        0x201493F00, 0x201493580, 0x201492C00, 0x201492280,
        0x201491900, 0x201490F80, 0x201490600, 0x20148FC80,
        0x20148F300, 0x20148E980, 0x20148E000, 0x20148D680,
        0x20148CD00, 0x20148C380, 0x20148BA00, 0x20148B080,
        0x20148A700, 0x201489D80, 0x201489400, 0x201488A80,
        0x201488100, 0x201487780, 0x201486E00, 0x201486480,
        0x201485B00, 0x201485180, 0x201484800, 0x201483E80,
        0x201483500, 0x201482B80, 0x201482200, 0x201481880,
        0x201480F00, 0x201480580, 0x20147FC00, 0x20147F280,
        0x20147E900, 0x20147DF80, 0x20147D600, 0x20147CC80,
        0x20147C300, 0x20147B980, 0x20147B000, 0x20147A680,
        0x201479D00, 0x201479380, 0x201478A00, 0x201478080,
        0x201477700, 0x201476D80, 0x201476400, 0x201475A80,
        0x201475100, 0x201474780, 0x201473E00, 0x201473480
    };

    uint64_t mbuf_first_cacheline_set_128_new[] = {
        0x201498A00, 0x201498080, 0x201497700, 0x201496D80,
        0x201496400, 0x201495A80, 0x201495100, 0x201494780,
        0x201493E00, 0x201493480, 0x201492B00, 0x201492180,
        0x201491800, 0x201490E80, 0x201490500, 0x20148FB80,
        0x20148F200, 0x20148E880, 0x20148DF00, 0x20148D580,
        0x20148CC00, 0x20148C280, 0x20148B900, 0x20148AF80,
        0x20148A600, 0x201489C80, 0x201489300, 0x201488980,
        0x201488000, 0x201487680, 0x201486D00, 0x201486380,
        0x201485A00, 0x201485080, 0x201484700, 0x201483D80,
        0x201483400, 0x201482A80, 0x201482100, 0x201481780,
        0x201480E00, 0x201480480, 0x20147FB00, 0x20147F180,
        0x20147E800, 0x20147DE80, 0x20147D500, 0x20147CB80,
        0x20147C200, 0x20147B880, 0x20147AF00, 0x20147A580,
        0x201479C00, 0x201479280, 0x201478900, 0x201477F80,
        0x201477600, 0x201476C80, 0x201476300, 0x201475980,
        0x201475000, 0x201474680, 0x201473D00, 0x201473380
    };

    uint64_t mbuf_second_cacheline_set_128_new[] = {
        0x201498A40, 0x2014980C0, 0x201497740, 0x201496DC0,
        0x201496440, 0x201495AC0, 0x201495140, 0x2014947C0,
        0x201493E40, 0x2014934C0, 0x201492B40, 0x2014921C0,
        0x201491840, 0x201490EC0, 0x201490540, 0x20148FBC0,
        0x20148F240, 0x20148E8C0, 0x20148DF40, 0x20148D5C0,
        0x20148CC40, 0x20148C2C0, 0x20148B940, 0x20148AFC0,
        0x20148A640, 0x201489CC0, 0x201489340, 0x2014889C0,
        0x201488040, 0x2014876C0, 0x201486D40, 0x2014863C0,
        0x201485A40, 0x2014850C0, 0x201484740, 0x201483DC0,
        0x201483440, 0x201482AC0, 0x201482140, 0x2014817C0,
        0x201480E40, 0x2014804C0, 0x20147FB40, 0x20147F1C0,
        0x20147E840, 0x20147DEC0, 0x20147D540, 0x20147CBC0,
        0x20147C240, 0x20147B8C0, 0x20147AF40, 0x20147A5C0,
        0x201479C40, 0x2014792C0, 0x201478940, 0x201477FC0,
        0x201477640, 0x201476CC0, 0x201476340, 0x2014759C0,
        0x201475040, 0x2014746C0, 0x201473D40, 0x2014733C0
    };

    // mbuf for RX Descriptor (192-255 current) - through current phase's tdt 256 wr prefetch
    uint64_t mbuf_addr_set_256_cur[] = {
        0x201393D80, 0x201394700, 0x201395080, 0x201395A00,
        0x201396380, 0x201396D00, 0x201397680, 0x201398000,
        0x201398980, 0x201399300, 0x201399C80, 0x20139A600,
        0x20139AF80, 0x20139B900, 0x20139C280, 0x20139CC00,
        0x20139D580, 0x20139DF00, 0x20139E880, 0x20139F200,
        0x20139FB80, 0x2013A0500, 0x2013A0E80, 0x2013A1800,
        0x2013A2180, 0x2013A2B00, 0x2013A3480, 0x2013A3E00,
        0x2013A4780, 0x2013A5100, 0x2013A5A80, 0x2013A6400,
        0x2013A6D80, 0x2013A7700, 0x2013A8080, 0x2013A8A00,
        0x2013A9380, 0x2013A9D00, 0x2013AA680, 0x2013AB000,
        0x2013AB980, 0x2013AC300, 0x2013ACC80, 0x2013AD600,
        0x2013ADF80, 0x2013AE900, 0x2013AF280, 0x2013AFC00,
        0x2013B0580, 0x2013B0F00, 0x2013B1880, 0x2013B2200,
        0x2013B2B80, 0x2013B3500, 0x2013B3E80, 0x2013B4800,
        0x2013B5180, 0x2013B5B00, 0x2013B6480, 0x2013B6E00,
        0x2013B7780, 0x2013B8100, 0x2013B8A80, 0x2013B9400
    };

    uint64_t mbuf_first_cacheline_set_256_cur[] = {
        0x201393C80, 0x201394600, 0x201394F80, 0x201395900,
        0x201396280, 0x201396C00, 0x201397580, 0x201397F00,
        0x201398880, 0x201399200, 0x201399B80, 0x20139A500,
        0x20139AE80, 0x20139B800, 0x20139C180, 0x20139CB00,
        0x20139D480, 0x20139DE00, 0x20139E780, 0x20139F100,
        0x20139FA80, 0x2013A0400, 0x2013A0D80, 0x2013A1700,
        0x2013A2080, 0x2013A2A00, 0x2013A3380, 0x2013A3D00,
        0x2013A4680, 0x2013A5000, 0x2013A5980, 0x2013A6300,
        0x2013A6C80, 0x2013A7600, 0x2013A7F80, 0x2013A8900,
        0x2013A9280, 0x2013A9C00, 0x2013AA580, 0x2013AAF00,
        0x2013AB880, 0x2013AC200, 0x2013ACB80, 0x2013AD500,
        0x2013ADE80, 0x2013AE800, 0x2013AF180, 0x2013AFB00,
        0x2013B0480, 0x2013B0E00, 0x2013B1780, 0x2013B2100,
        0x2013B2A80, 0x2013B3400, 0x2013B3D80, 0x2013B4700,
        0x2013B5080, 0x2013B5A00, 0x2013B6380, 0x2013B6D00,
        0x2013B7680, 0x2013B8000, 0x2013B8980, 0x2013B9300
    };

    uint64_t mbuf_second_cacheline_set_256_cur[] = {
        0x201393CC0, 0x201394640, 0x201394FC0, 0x201395940,
        0x2013962C0, 0x201396C40, 0x2013975C0, 0x201397F40,
        0x2013988C0, 0x201399240, 0x201399BC0, 0x20139A540,
        0x20139AEC0, 0x20139B840, 0x20139C1C0, 0x20139CB40,
        0x20139D4C0, 0x20139DE40, 0x20139E7C0, 0x20139F140,
        0x20139FAC0, 0x2013A0440, 0x2013A0DC0, 0x2013A1740,
        0x2013A20C0, 0x2013A2A40, 0x2013A33C0, 0x2013A3D40,
        0x2013A46C0, 0x2013A5040, 0x2013A59C0, 0x2013A6340,
        0x2013A6CC0, 0x2013A7640, 0x2013A7FC0, 0x2013A8940,
        0x2013A92C0, 0x2013A9C40, 0x2013AA5C0, 0x2013AAF40,
        0x2013AB8C0, 0x2013AC240, 0x2013ACBC0, 0x2013AD540,
        0x2013ADEC0, 0x2013AE840, 0x2013AF1C0, 0x2013AFB40,
        0x2013B04C0, 0x2013B0E40, 0x2013B17C0, 0x2013B2140,
        0x2013B2AC0, 0x2013B3440, 0x2013B3DC0, 0x2013B4740,
        0x2013B50C0, 0x2013B5A40, 0x2013B63C0, 0x2013B6D40,
        0x2013B76C0, 0x2013B8040, 0x2013B89C0, 0x2013B9340
    };

    // mbuf for TX Descriptor (256-319 prev) - through previous phase's tdt 320 wr prefetch
    uint64_t mbuf_addr_set_320_prev[] = {
        0x201451700, 0x201450D80, 0x201450400, 0x20144FA80,
        0x20144F100, 0x20144E780, 0x20144DE00, 0x20144D480,
        0x201456300, 0x201455980, 0x201455000, 0x201454680,
        0x201453D00, 0x201453380, 0x201452A00, 0x201452080,
        0x20145AF00, 0x20145A580, 0x201459C00, 0x201459280,
        0x201458900, 0x201457F80, 0x201457600, 0x201456C80,
        0x20145FB00, 0x20145F180, 0x20145E800, 0x20145DE80,
        0x20145D500, 0x20145CB80, 0x20145C200, 0x20145B880,
        0x201464700, 0x201463D80, 0x201463400, 0x201462A80,
        0x201462100, 0x201461780, 0x201460E00, 0x201460480,
        0x201469300, 0x201468980, 0x201468000, 0x201467680,
        0x201466D00, 0x201466380, 0x201465A00, 0x201465080,
        0x20146DF00, 0x20146D580, 0x20146CC00, 0x20146C280,
        0x20146B900, 0x20146AF80, 0x20146A600, 0x201469C80,
        0x201472B00, 0x201472180, 0x201471800, 0x201470E80,
        0x201470500, 0x20146FB80, 0x20146F200, 0x20146E880
    };

    uint64_t mbuf_first_cacheline_set_320_prev[] = {
        0x201451600, 0x201450C80, 0x201450300, 0x20144F980,
        0x20144F000, 0x20144E680, 0x20144DD00, 0x20144D380,
        0x201456200, 0x201455880, 0x201454F00, 0x201454580,
        0x201453C00, 0x201453280, 0x201452900, 0x201451F80,
        0x20145AE00, 0x20145A480, 0x201459B00, 0x201459180,
        0x201458800, 0x201457E80, 0x201457500, 0x201456B80,
        0x20145FA00, 0x20145F080, 0x20145E700, 0x20145DD80,
        0x20145D400, 0x20145CA80, 0x20145C100, 0x20145B780,
        0x201464600, 0x201463C80, 0x201463300, 0x201462980,
        0x201462000, 0x201461680, 0x201460D00, 0x201460380,
        0x201469200, 0x201468880, 0x201467F00, 0x201467580,
        0x201466C00, 0x201466280, 0x201465900, 0x201464F80,
        0x20146DE00, 0x20146D480, 0x20146CB00, 0x20146C180,
        0x20146B800, 0x20146AE80, 0x20146A500, 0x201469B80,
        0x201472A00, 0x201472080, 0x201471700, 0x201470D80,
        0x201470400, 0x20146FA80, 0x20146F100, 0x20146E780
    };

    uint64_t mbuf_second_cacheline_set_320_prev[] = {
        0x201451640, 0x201450CC0, 0x201450340, 0x20144F9C0,
        0x20144F040, 0x20144E6C0, 0x20144DD40, 0x20144D3C0,
        0x201456240, 0x2014558C0, 0x201454F40, 0x2014545C0,
        0x201453C40, 0x2014532C0, 0x201452940, 0x201451FC0,
        0x20145AE40, 0x20145A4C0, 0x201459B40, 0x2014591C0,
        0x201458840, 0x201457EC0, 0x201457540, 0x201456BC0,
        0x20145FA40, 0x20145F0C0, 0x20145E740, 0x20145DDC0,
        0x20145D440, 0x20145CAC0, 0x20145C140, 0x20145B7C0,
        0x201464640, 0x201463CC0, 0x201463340, 0x2014629C0,
        0x201462040, 0x2014616C0, 0x201460D40, 0x2014603C0,
        0x201469240, 0x2014688C0, 0x201467F40, 0x2014675C0,
        0x201466C40, 0x2014662C0, 0x201465940, 0x201464FC0,
        0x20146DE40, 0x20146D4C0, 0x20146CB40, 0x20146C1C0,
        0x20146B840, 0x20146AEC0, 0x20146A540, 0x201469BC0,
        0x201472A40, 0x2014720C0, 0x201471740, 0x201470DC0,
        0x201470440, 0x20146FAC0, 0x20146F140, 0x20146E7C0
    };     

    // mbuf's structure part
    for (int i = 0; i < 64; i ++) {
        uint64_t pkt_addr = pkt->getAddr();
        uint64_t pkt_addr_end = pkt_addr + pkt->getSize();
        // Check if the packet address is in the mbuf's structure part
        /*
        if (pkt_addr >= mbuf_first_cacheline_set_95_preprev[i] && pkt_addr_end <= mbuf_first_cacheline_set_95_preprev[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_95PREPREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_95_preprev[i] && pkt_addr_end <= mbuf_second_cacheline_set_95_preprev[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_95PREPREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_95_prev[i] && pkt_addr_end <= mbuf_first_cacheline_set_95_prev[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_95PREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_95_prev[i] && pkt_addr_end <= mbuf_second_cacheline_set_95_prev[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_95PREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_95_new[i] && pkt_addr_end <= mbuf_first_cacheline_set_95_new[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_95NEW[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_95_new[i] && pkt_addr_end <= mbuf_second_cacheline_set_95_new[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_95NEW[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_127_prev[i] && pkt_addr_end <= mbuf_first_cacheline_set_127_prev[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_127PREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_127_prev[i] && pkt_addr_end <= mbuf_second_cacheline_set_127_prev[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_127PREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        */

        // SVE
        if (pkt_addr >= mbuf_first_cacheline_set_192_cur[i] && pkt_addr_end <= mbuf_first_cacheline_set_192_cur[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_192CUR[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_192_cur[i] && pkt_addr_end <= mbuf_second_cacheline_set_192_cur[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_192CUR[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_128_new[i] && pkt_addr_end <= mbuf_first_cacheline_set_128_new[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_128NEW[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_128_new[i] && pkt_addr_end <= mbuf_second_cacheline_set_128_new[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_128NEW[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_256_cur[i] && pkt_addr_end <= mbuf_first_cacheline_set_256_cur[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_256CUR[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_256_cur[i] && pkt_addr_end <= mbuf_second_cacheline_set_256_cur[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_256CUR[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_320_prev[i] && pkt_addr_end <= mbuf_first_cacheline_set_320_prev[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_320PREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_320_prev[i] && pkt_addr_end <= mbuf_second_cacheline_set_320_prev[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_320PREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }

    // mbuf's data part
    for (int i = 0; i < 32; i++) {
        /*
        if (pkt->getAddr() == mbuf_addr_set_95_preprev[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF95PREPREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_95_prev[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF95PREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_95_new[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF95NEW[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_127_prev[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF127PREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        */

       // SVE
        if (pkt->getAddr() == mbuf_addr_set_192_cur[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF192CUR[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_128_new[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF128NEW[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_256_cur[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF256CUR[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_320_prev[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF320PREV[%d]_RES\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }
    #endif

    #if LOG_LEVEL == 2
    // DTA log
    uint64_t mbuf_addr1[] = { // RX_JOB_ID 7
        // for zero-copy
        // 0x201095200, 0x201094880, 0x201093f00, 0x201093580, 0x201092c00, 0x201092280, 
        // 0x201091900, 0x201090f80, 0x201090600, 0x20108fc80, 0x20108f300, 0x20108e980, 
        // 0x20108e000, 0x20108d680, 0x20108cd00, 0x20108c380, 0x20108ba00, 0x20108b080, 
        // 0x20108a700, 0x201089d80, 0x201089400, 0x201088a80, 0x201088100, 0x201087780, 
        // 0x201086e00, 0x201086480, 0x201085b00, 0x201085180, 0x201084800, 0x201083e80, 
        // 0x201083500, 0x201082b80
        
        // for non zero-copy
        0x2010a8200, 0x2010a7880, 0x2010a6f00, 0x2010a6580, 0x2010a5c00, 0x2010a5280,
        0x2010a4900, 0x2010a3f80, 0x2010a3600, 0x2010a2c80, 0x2010a2300, 0x2010a1980, 0x2010a1000, 0x2010a0680, 
        0x20109fd00, 0x20109f380, 0x20109ea00, 0x20109e080, 0x20109d700, 0x20109cd80, 0x20109c400, 0x20109ba80,
        0x20109b100, 0x20109a780, 0x201099e00, 0x201099480, 0x201098b00, 0x201098180, 0x201097800, 0x201096e80,
        0x201096500, 0x201095b80
    };

    uint64_t mbuf_addr2[] = { // RX_JOB_ID 9
        0x20106f200, 0x20106e880, 0x20106df00, 0x20106d580, 0x20106cc00, 0x20106c280,
        0x20106b900, 0x20106af80, 0x20106a600, 0x201069c80, 0x201069300, 0x201068980,
        0x201068000, 0x201067680, 0x201066d00, 0x201066380, 0x201065a00, 0x201065080,
        0x201064700, 0x201063d80, 0x201063400, 0x201062a80, 0x201062100, 0x201061780,
        0x201060e00, 0x201060480, 0x20105fb00, 0x20105f180, 0x20105e800, 0x20105de80,
        0x20105d500, 0x20105cb80
    };

    uint64_t desc_rx[] = {
        // for zero-copy
        // 8624136384, 8624136448, 8624136512, 8624136576, 8624136640, 8624136704, 8624136768, 8624136832

        // for non-zero-copy
        0x20209e0c0, 0x20209e100, 0x20209e140, 0x20209e180, 0x20209e1c0, 0x20209e200, 0x20209e240, 0x20209e280
    };

    uint64_t desc_tx[] = {
        // for zero-copy
        // 8624220032, 8624220096, 8624220160, 8624220224

        // for non-zero-copy
        0x2020b2780, 0x2020b27c0, 0x2020b2800, 0x2020b2840
    };

    // for zero-copy
    // uint64_t comp_rx = 0x20209e080;
    // uint64_t comp_tx = 0x2020b2c00;

    // for non-zero-copy
    uint64_t comp_rx = 0x20209e080;
    uint64_t comp_tx = 0x2020b2c00;

    // MBUF_1
    for (int i = 0; i < 32; i++) {
        if (pkt->getAddr() == mbuf_addr1[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_1[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }

    // MBUF_2
    for (int i = 0; i < 32; i++) {
        if (pkt->getAddr() == mbuf_addr2[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_2[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }

    // DESC_RX
    for (int i = 0; i < 8; i++) {
        if (pkt->getAddr() == desc_rx[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_RX[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }

    // DESC_TX
    for (int i = 0; i < 4; i++) {
        if (pkt->getAddr() == desc_tx[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_TX[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }

    // COMP_RX
    if (pkt->getAddr() == comp_rx) {
        printf("[LOG], %llu, %s, %s, %d, COMP_RX\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }
    // COMP_TX
    if (pkt->getAddr() == comp_tx) {
        printf("[LOG], %llu, %s, %s, %d, COMP_TX\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }
    #endif

    #if LOG_LEVEL == 3
    uint64_t comp_rx = 0x20209b800;
    uint64_t comp_rx2 = 0x20209b380;
    uint64_t comp_tx = 0x2020b4b80;
    uint64_t comp_tx2 = 0x2020b4600;

    uint64_t mbuf_arr_rx = 0x202099300;
    uint64_t mbuf_arr_rx2 = 0x202097280;


    // COMP_RX
    if (pkt->getAddr() == comp_rx) {
        int64_t comp_val = 1111111111;
        if (pkt->isRead()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        }
        printf("[LOG], %llu, %s, %s, %ld, COMP_RX_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), comp_val);
    }
    if (pkt->getAddr() == comp_rx2) {
        int64_t comp_val = 1111111111;
        if (pkt->isRead()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        }
        printf("[LOG], %llu, %s, %s, %ld, COMP_RX2_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), comp_val);
    }
    // COMP_TX
    if (pkt->getAddr() == comp_tx) {
        int64_t comp_val = 1111111111;
        if (pkt->isRead()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        }
        printf("[LOG], %llu, %s, %s, %ld, COMP_TX_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), comp_val);
    }
    if (pkt->getAddr() == comp_tx2) {
        int64_t comp_val = 1111111111;
        if (pkt->isRead()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            comp_val = data[0];
        }
        printf("[LOG], %llu, %s, %s, %ld, COMP_TX2_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), comp_val);
    }

    // MBUF_ARR_RX
    if (pkt->getAddr() == mbuf_arr_rx) {
        int64_t pointer_addr = 1111111111;
        if (pkt->isRead()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            pointer_addr = data[0];
        } 
        printf("[LOG], %llu, %s, %s, %d, MBUF_ARR_RX_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), pointer_addr);
    }
    if (pkt->getAddr() == mbuf_arr_rx2) {
        int64_t pointer_addr = 1111111111;
        if (pkt->isRead()) {
            // Get value from pkt's data
            int64_t* data = pkt->getPtr<int64_t>();
            pointer_addr = data[0];
        }
        printf("[LOG], %llu, %s, %s, %d, MBUF_ARR_RX2_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), pointer_addr);
    }

    // RX_JOB_SUBMIT
    if (pkt->getAddr() == 1073752256) {
        printf("[LOG], %llu, %s, %s, %d, RX_JOB_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }
    // TX_JOB_SUBMIT
    if (pkt->getAddr() == 1073756352) {
        printf("[LOG], %llu, %s, %s, %d, TX_JOB_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0);
    }

    // RX_JOB_ID 12
    uint64_t mbuf_addr_set_RXJ_12[] = {
        0x201321D80, 0x201322700, 0x201323080, 0x201323A00,
        0x201324380, 0x201324D00, 0x201325680, 0x201326000,
        0x201326980, 0x201327300, 0x201327C80, 0x201328600,
        0x201328F80, 0x201329900, 0x20132A280, 0x20132AC00,
        0x20132B580, 0x20132BF00, 0x20132C880, 0x20132D200,
        0x20132DB80, 0x20132E500, 0x20132EE80, 0x20132F800,
        0x201330180, 0x201330B00, 0x201331480, 0x201331E00,
        0x201332780, 0x201333100, 0x201333A80, 0x201334400
    };

    uint64_t mbuf_first_cacheline_set_RXJ_12[] = {
        0x201321C80, 0x201322600, 0x201322F80, 0x201323900,
        0x201324280, 0x201324C00, 0x201325580, 0x201325F00,
        0x201326880, 0x201327200, 0x201327B80, 0x201328500,
        0x201328E80, 0x201329800, 0x20132A180, 0x20132AB00,
        0x20132B480, 0x20132BE00, 0x20132C780, 0x20132D100,
        0x20132DA80, 0x20132E400, 0x20132ED80, 0x20132F700,
        0x201330080, 0x201330A00, 0x201331380, 0x201331D00,
        0x201332680, 0x201333000, 0x201333980, 0x201334300
    };

    uint64_t mbuf_second_cacheline_set_RXJ_12[] = {
        0x201321CC0, 0x201322640, 0x201322FC0, 0x201323940,
        0x2013242C0, 0x201324C40, 0x2013255C0, 0x201325F40,
        0x2013268C0, 0x201327240, 0x201327BC0, 0x201328540,
        0x201328EC0, 0x201329840, 0x20132A1C0, 0x20132AB40,
        0x20132B4C0, 0x20132BE40, 0x20132C7C0, 0x20132D140,
        0x20132DAC0, 0x20132E440, 0x20132EDC0, 0x20132F740,
        0x2013300C0, 0x201330A40, 0x2013313C0, 0x201331D40,
        0x2013326C0, 0x201333040, 0x2013339C0, 0x201334340
    };

    uint64_t mbuf_addr_set_RXJ_10[] = {
        0x20131D180, 0x20131DB00, 0x20131E480, 0x20131EE00,
        0x20131F780, 0x201320100, 0x201320A80, 0x201321400,
        0x201318580, 0x201318F00, 0x201319880, 0x20131A200,
        0x20131AB80, 0x20131B500, 0x20131BE80, 0x20131C800,
        0x201313980, 0x201314300, 0x201314C80, 0x201315600,
        0x201315F80, 0x201316900, 0x201317280, 0x201317C00,
        0x20130ED80, 0x20130F700, 0x201310080, 0x201310A00,
        0x201311380, 0x201311D00, 0x201312680, 0x201313000
    };

    uint64_t mbuf_first_cacheline_set_RXJ_10[] = {
        0x20131D080, 0x20131DA00, 0x20131E380, 0x20131ED00,
        0x20131F680, 0x201320000, 0x201320980, 0x201321300,
        0x201318480, 0x201318E00, 0x201319780, 0x20131A100,
        0x20131AA80, 0x20131B400, 0x20131BD80, 0x20131C700,
        0x201313880, 0x201314200, 0x201314B80, 0x201315500,
        0x201315E80, 0x201316800, 0x201317180, 0x201317B00,
        0x20130EC80, 0x20130F600, 0x20130FF80, 0x201310900,
        0x201311280, 0x201311C00, 0x201312580, 0x201312F00
    };

    uint64_t mbuf_second_cacheline_set_RXJ_10[] = {
        0x20131D0C0, 0x20131DA40, 0x20131E3C0, 0x20131ED40,
        0x20131F6C0, 0x201320040, 0x2013209C0, 0x201321340,
        0x2013184C0, 0x201318E40, 0x2013197C0, 0x20131A140,
        0x20131AAC0, 0x20131B440, 0x20131BDC0, 0x20131C740,
        0x2013138C0, 0x201314240, 0x201314BC0, 0x201315540,
        0x201315EC0, 0x201316840, 0x2013171C0, 0x201317B40,
        0x20130ECC0, 0x20130F640, 0x20130FFC0, 0x201310940,
        0x2013112C0, 0x201311C40, 0x2013125C0, 0x201312F40
    };

    uint64_t mbuf_addr_set_RXJ_11[] = {
        0x20130A180, 0x20130AB00, 0x20130B480, 0x20130BE00,
        0x20130C780, 0x20130D100, 0x20130DA80, 0x20130E400,
        0x201305580, 0x201305F00, 0x201306880, 0x201307200,
        0x201307B80, 0x201308500, 0x201308E80, 0x201309800,
        0x201300980, 0x201301300, 0x201301C80, 0x201302600,
        0x201302F80, 0x201303900, 0x201304280, 0x201304C00,
        0x2012FBD80, 0x2012FC700, 0x2012FD080, 0x2012FDA00,
        0x2012FE380, 0x2012FED00, 0x2012FF680, 0x201300000
    };

    uint64_t mbuf_first_cacheline_set_RXJ_11[] = {
        0x20130A080, 0x20130AA00, 0x20130B380, 0x20130BD00,
        0x20130C680, 0x20130D000, 0x20130D980, 0x20130E300,
        0x201305480, 0x201305E00, 0x201306780, 0x201307100,
        0x201307A80, 0x201308400, 0x201308D80, 0x201309700,
        0x201300880, 0x201301200, 0x201301B80, 0x201302500,
        0x201302E80, 0x201303800, 0x201304180, 0x201304B00,
        0x2012FBC80, 0x2012FC600, 0x2012FCF80, 0x2012FD900,
        0x2012FE280, 0x2012FEC00, 0x2012FF580, 0x2012FFF00
    };

    uint64_t mbuf_second_cacheline_set_RXJ_11[] = {
        0x20130A0C0, 0x20130AA40, 0x20130B3C0, 0x20130BD40,
        0x20130C6C0, 0x20130D040, 0x20130D9C0, 0x20130E340,
        0x2013054C0, 0x201305E40, 0x2013067C0, 0x201307140,
        0x201307AC0, 0x201308440, 0x201308DC0, 0x201309740,
        0x2013008C0, 0x201301240, 0x201301BC0, 0x201302540,
        0x201302EC0, 0x201303840, 0x2013041C0, 0x201304B40,
        0x2012FBCC0, 0x2012FC640, 0x2012FCFC0, 0x2012FD940,
        0x2012FE2C0, 0x2012FEC40, 0x2012FF5C0, 0x2012FFF40
    };

    uint64_t mbuf_addr_set_RXJ_13[] = {
        0x20130ED80, 0x20130F700, 0x201310080, 0x201310A00,
        0x201311380, 0x201311D00, 0x201312680, 0x201313000,
        0x201313980, 0x201314300, 0x201314C80, 0x201315600,
        0x201315F80, 0x201316900, 0x201317280, 0x201317C00,
        0x201318580, 0x201318F00, 0x201319880, 0x20131A200,
        0x20131AB80, 0x20131B500, 0x20131BE80, 0x20131C800,
        0x20131D180, 0x20131DB00, 0x20131E480, 0x20131EE00,
        0x20131F780, 0x201320100, 0x201320A80, 0x201321400
    };

    uint64_t mbuf_first_cacheline_set_RXJ_13[] = {
        0x20130EC80, 0x20130F600, 0x20130FF80, 0x201310900,
        0x201311280, 0x201311C00, 0x201312580, 0x201312F00,
        0x201313880, 0x201314200, 0x201314B80, 0x201315500,
        0x201315E80, 0x201316800, 0x201317180, 0x201317B00,
        0x201318480, 0x201318E00, 0x201319780, 0x20131A100,
        0x20131AA80, 0x20131B400, 0x20131BD80, 0x20131C700,
        0x20131D080, 0x20131DA00, 0x20131E380, 0x20131ED00,
        0x20131F680, 0x201320000, 0x201320980, 0x201321300
    };

    uint64_t mbuf_second_cacheline_set_RXJ_13[] = {
        0x20130ECC0, 0x20130F640, 0x20130FFC0, 0x201310940,
        0x2013112C0, 0x201311C40, 0x2013125C0, 0x201312F40,
        0x2013138C0, 0x201314240, 0x201314BC0, 0x201315540,
        0x201315EC0, 0x201316840, 0x2013171C0, 0x201317B40,
        0x2013184C0, 0x201318E40, 0x2013197C0, 0x20131A140,
        0x20131AAC0, 0x20131B440, 0x20131BDC0, 0x20131C740,
        0x20131D0C0, 0x20131DA40, 0x20131E3C0, 0x20131ED40,
        0x20131F6C0, 0x201320040, 0x2013209C0, 0x201321340
    };

    uint64_t desc_rx1[] = {
        0x20209B840, 0x20209B880, 0x20209B8C0, 0x20209B900, 0x20209B940, 0x20209B980, 0x20209B9C0, 0x20209BA00
    };

    uint64_t desc_rx2[] = {
        0x20209B3C0, 0x20209B400, 0x20209B440, 0x20209B480, 0x20209B4C0, 0x20209B500, 0x20209B540, 0x20209B580
    };

    uint64_t desc_tx1[] = {
        0x2020b4700, 0x2020b4740, 0x2020b4780, 0x2020b47c0
    };

    uint64_t desc_tx2[] = {
        0x2020b4180, 0x2020b41c0, 0x2020b4200, 0x2020b4240
    };

    for (int i = 0; i < 32; i ++) {
        uint64_t pkt_addr = pkt->getAddr();
        uint64_t pkt_addr_end = pkt_addr + pkt->getSize();
        if (pkt_addr >= mbuf_first_cacheline_set_RXJ_12[i] && pkt_addr_end <= mbuf_first_cacheline_set_RXJ_12[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_RXJ12[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_RXJ_12[i] && pkt_addr_end <= mbuf_second_cacheline_set_RXJ_12[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_RXJ12[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_RXJ_10[i] && pkt_addr_end <= mbuf_first_cacheline_set_RXJ_10[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_RXJ10[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_RXJ_10[i] && pkt_addr_end <= mbuf_second_cacheline_set_RXJ_10[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_RXJ10[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_RXJ_11[i] && pkt_addr_end <= mbuf_first_cacheline_set_RXJ_11[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_RXJ11[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_RXJ_11[i] && pkt_addr_end <= mbuf_second_cacheline_set_RXJ_11[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_RXJ11[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_first_cacheline_set_RXJ_13[i] && pkt_addr_end <= mbuf_first_cacheline_set_RXJ_13[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$0_RXJ13[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt_addr >= mbuf_second_cacheline_set_RXJ_13[i] && pkt_addr_end <= mbuf_second_cacheline_set_RXJ_13[i] + 64) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_$1_RXJ13[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }

    for (int i = 0; i < 32; i++) {
        if (pkt->getAddr() == mbuf_addr_set_RXJ_12[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_RXJ12[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_RXJ_10[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_RXJ10[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_RXJ_11[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_RXJ11[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
        if (pkt->getAddr() == mbuf_addr_set_RXJ_13[i]) {
            printf("[LOG], %llu, %s, %s, %d, MBUF_RXJ13[%d]_RSP\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }

    // DESC_RX
    for (int i = 0; i < 8; i++) {
        if (pkt->getAddr() == desc_rx1[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_RX1_RSP[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }

    for (int i = 0; i <8; i++) {
        if (pkt->getAddr() == desc_rx2[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_RX2_RSP[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }
    
    // DESC_TX
    for (int i = 0; i < 4; i++) {
        if (pkt->getAddr() == desc_tx1[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_TX1_RSP[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }

    for (int i = 0; i < 4; i++) {
        if (pkt->getAddr() == desc_tx2[i]) {
            printf("[LOG], %llu, %s, %s, %d, DESC_TX2_RSP[%d]\n", curTick(), name().c_str(), pkt->print().c_str(), 0, i);
        }
    }
    #endif

    // if this is a write, we should be looking at an uncacheable
    // write
    if (pkt->isWrite()) {
        assert(pkt->req->isUncacheable());
        handleUncacheableWriteResp(pkt);
        return;
    }

    // we have dealt with any (uncacheable) writes above, from here on
    // we know we are dealing with an MSHR due to a miss or a prefetch
    MSHR *mshr = dynamic_cast<MSHR*>(pkt->popSenderState());
    assert(mshr);

    if (mshr == noTargetMSHR) {
        // we always clear at least one target
        clearBlocked(Blocked_NoTargets);
        noTargetMSHR = nullptr;
    }

    // Initial target is used just for stats
    const QueueEntry::Target *initial_tgt = mshr->getTarget();
    const Tick miss_latency = curTick() - initial_tgt->recvTime;
    if (pkt->req->isUncacheable()) {
        assert(pkt->req->requestorId() < system->maxRequestors());
        stats.cmdStats(initial_tgt->pkt)
            .mshrUncacheableLatency[pkt->req->requestorId()] += miss_latency;
    } else {
        assert(pkt->req->requestorId() < system->maxRequestors());
        stats.cmdStats(initial_tgt->pkt)
            .mshrMissLatency[pkt->req->requestorId()] += miss_latency;
    }

    PacketList writebacks;

    bool is_fill = !mshr->isForward &&
        (pkt->isRead() || pkt->cmd == MemCmd::UpgradeResp ||
         mshr->wasWholeLineWrite);

    // make sure that if the mshr was due to a whole line write then
    // the response is an invalidation
    assert(!mshr->wasWholeLineWrite || pkt->isInvalidate());

    CacheBlk *blk = tags->findBlock(pkt->getAddr(), pkt->isSecure());

    if(isIOCache){
        if(blk){
            blk->setDdioPrefetchDestination(pkt->getDdioPrefetchDestination());
            if(pkt->isDdioHeader()) blk->setDdioHeader();

            // Not works
            if(pkt->getDdioPrefetchDestination() != -1){
                blk->setDdioPrefetchDestination(pkt->getDdioPrefetchDestination());
            }
            DPRINTF(AdaptiveDdioOtf, "recvTimingResp qid %d, pkt %s\n", blk->getDdioPrefetchDestination(), pkt->print());
            

            if(blk->getDdioPrefetchDestination()!=-1)
            {
                DPRINTF(AdaptiveDdioCache, "Set MLC id %d, blk %s, pkt %s\n", blk->getDdioPrefetchDestination(), blk->print(), pkt->print());
            }
        }
        else{
            //DPRINTF(DDIO, "blk is nullptr\n");
        }
    }


    if (is_fill && !is_error) {
        DPRINTF(Cache, "Block for addr %#llx being updated in Cache\n",
                pkt->getAddr());

        const bool allocate = (writeAllocator && mshr->wasWholeLineWrite) ?
            writeAllocator->allocate() : mshr->allocOnFill();

        // SHIN.
        if(pkt->getDdioPrefetchDestination() == -1){
            pkt->setDdioPrefetchDestination(mshr->qid_from_dev);
            if(mshr->is_ddio_pkt) pkt->setDdioPkt();
            if(mshr->is_header) pkt->setDdioHeader();
        }

        // SHIN. ported from base
        //blk = handleFill(pkt, blk, writebacks, allocate);
        blk = handleFill(pkt, blk, writebacks, allocate,
                            mshr->wasBlockIO && ddioEnabled);


        assert(blk != nullptr);
        ppFill->notify(pkt);
    }

    if (blk && blk->isValid() && pkt->isClean() && !pkt->isInvalidate()) {
        // The block was marked not readable while there was a pending
        // cache maintenance operation, restore its flag.
        blk->setCoherenceBits(CacheBlk::ReadableBit);

        // This was a cache clean operation (without invalidate)
        // and we have a copy of the block already. Since there
        // is no invalidation, we can promote targets that don't
        // require a writable copy
        mshr->promoteReadable();
    }

    if (blk && blk->isSet(CacheBlk::WritableBit) &&
        !pkt->req->isCacheInvalidate()) {
        // If at this point the referenced block is writable and the
        // response is not a cache invalidate, we promote targets that
        // were deferred as we couldn't guarrantee a writable copy
        mshr->promoteWritable();
    }

    // For DTA's DMA request, the response will be sent to the DTA inside serviceMSHRTargets()
    serviceMSHRTargets(mshr, pkt, blk);

    if (mshr->promoteDeferredTargets()) {
        // avoid later read getting stale data while write miss is
        // outstanding.. see comment in timingAccess()
        if (blk) {
            blk->clearCoherenceBits(CacheBlk::ReadableBit);
        }
        mshrQueue.markPending(mshr);
        schedMemSideSendEvent(clockEdge() + pkt->payloadDelay);
    } else {
        // while we deallocate an mshr from the queue we still have to
        // check the isFull condition before and after as we might
        // have been using the reserved entries already
        const bool was_full = mshrQueue.isFull();
        mshrQueue.deallocate(mshr);
        if (was_full && !mshrQueue.isFull()) {
            clearBlocked(Blocked_NoMSHRs);
        }

        // Request the bus for a prefetch if this deallocation freed enough
        // MSHRs for a prefetch to take place
        if (prefetcher && mshrQueue.canPrefetch() && !isBlocked()) {
            Tick next_pf_time = std::max(prefetcher->nextPrefetchReadyTime(),
                                         clockEdge());
            if (next_pf_time != MaxTick)
                schedMemSideSendEvent(next_pf_time);
        }
    }

    // if we used temp block, check to see if its valid and then clear it out
    if (blk == tempBlock && tempBlock->isValid()) {
        evictBlock(blk, writebacks);
    }

    const Tick forward_time = clockEdge(forwardLatency) + pkt->headerDelay;
    // copy writebacks to write buffer
    doWritebacks(writebacks, forward_time);

    DPRINTF(CacheVerbose, "%s: Leaving with %s\n", __func__, pkt->print());
    delete pkt;
}


Tick
BaseCache::recvAtomic(PacketPtr pkt)
{
    // should assert here that there are no outstanding MSHRs or
    // writebacks... that would mean that someone used an atomic
    // access in timing mode

    // We use lookupLatency here because it is used to specify the latency
    // to access.
    Cycles lat = lookupLatency;

    CacheBlk *blk = nullptr;
    PacketList writebacks;
    bool satisfied = access(pkt, blk, lat, writebacks);

    if (pkt->isClean() && blk && blk->isSet(CacheBlk::DirtyBit)) {
        // A cache clean opearation is looking for a dirty
        // block. If a dirty block is encountered a WriteClean
        // will update any copies to the path to the memory
        // until the point of reference.
        DPRINTF(CacheVerbose, "%s: packet %s found block: %s\n",
                __func__, pkt->print(), blk->print());
        PacketPtr wb_pkt = writecleanBlk(blk, pkt->req->getDest(), pkt->id, pkt->cpu_side_port_id);
        writebacks.push_back(wb_pkt);
        pkt->setSatisfied();
    }

    // handle writebacks resulting from the access here to ensure they
    // logically precede anything happening below
    doWritebacksAtomic(writebacks);
    assert(writebacks.empty());

    if (!satisfied) {
        lat += handleAtomicReqMiss(pkt, blk, writebacks);
    }

    // Note that we don't invoke the prefetcher at all in atomic mode.
    // It's not clear how to do it properly, particularly for
    // prefetchers that aggressively generate prefetch candidates and
    // rely on bandwidth contention to throttle them; these will tend
    // to pollute the cache in atomic mode since there is no bandwidth
    // contention.  If we ever do want to enable prefetching in atomic
    // mode, though, this is the place to do it... see timingAccess()
    // for an example (though we'd want to issue the prefetch(es)
    // immediately rather than calling requestMemSideBus() as we do
    // there).

    // do any writebacks resulting from the response handling
    doWritebacksAtomic(writebacks);

    // if we used temp block, check to see if its valid and if so
    // clear it out, but only do so after the call to recvAtomic is
    // finished so that any downstream observers (such as a snoop
    // filter), first see the fill, and only then see the eviction
    if (blk == tempBlock && tempBlock->isValid()) {
        // the atomic CPU calls recvAtomic for fetch and load/store
        // sequentuially, and we may already have a tempBlock
        // writeback from the fetch that we have not yet sent
        if (tempBlockWriteback) {
            // if that is the case, write the prevoius one back, and
            // do not schedule any new event
            writebackTempBlockAtomic();
        } else {
            // the writeback/clean eviction happens after the call to
            // recvAtomic has finished (but before any successive
            // calls), so that the response handling from the fill is
            // allowed to happen first
            schedule(writebackTempBlockAtomicEvent, curTick());
        }

        tempBlockWriteback = evictBlock(blk);
    }

    if (pkt->needsResponse()) {
        pkt->makeAtomicResponse();
    }

    return lat * clockPeriod();
}

void
BaseCache::functionalAccess(PacketPtr pkt, bool from_cpu_side)
{
    Addr blk_addr = pkt->getBlockAddr(blkSize);
    bool is_secure = pkt->isSecure();
    CacheBlk *blk = tags->findBlock(pkt->getAddr(), is_secure);
    MSHR *mshr = mshrQueue.findMatch(blk_addr, is_secure);

    pkt->pushLabel(name());

    CacheBlkPrintWrapper cbpw(blk);

    // Note that just because an L2/L3 has valid data doesn't mean an
    // L1 doesn't have a more up-to-date modified copy that still
    // needs to be found.  As a result we always update the request if
    // we have it, but only declare it satisfied if we are the owner.

    // see if we have data at all (owned or otherwise)
    bool have_data = blk && blk->isValid()
        && pkt->trySatisfyFunctional(&cbpw, blk_addr, is_secure, blkSize,
                                     blk->data);

    // data we have is dirty if marked as such or if we have an
    // in-service MSHR that is pending a modified line
    bool have_dirty =
        have_data && (blk->isSet(CacheBlk::DirtyBit) ||
                      (mshr && mshr->inService && mshr->isPendingModified()));

    bool done = have_dirty ||
        // cpuSidePort.trySatisfyFunctional(pkt) ||
        mshrQueue.trySatisfyFunctional(pkt) ||
        writeBuffer.trySatisfyFunctional(pkt) ||
        memSidePort.trySatisfyFunctional(pkt);
    
    if (isLLC && isMultiPort) {
        for (const auto& cpu_port : cpuSidePortList) {
            done = done || cpu_port->trySatisfyFunctional(pkt);
        }
    } else if (isIOCache && enableDTA) {
      // TODO - JM: have to check   
        done = done || jobPort->trySatisfyFunctional(pkt);
    } else {
        done = done || cpuSidePort.trySatisfyFunctional(pkt);
    }

    DPRINTF(CacheVerbose, "%s: %s %s%s%s\n", __func__,  pkt->print(),
            (blk && blk->isValid()) ? "valid " : "",
            have_data ? "data " : "", done ? "done " : "");

    // We're leaving the cache, so pop cache->name() label
    pkt->popLabel();

    if (done) {
        pkt->makeResponse();
    } else {
        PortID cpu_port_id = isLLC && isMultiPort ? pkt->cpu_side_port_id : InvalidPortID;

        // if it came as a request from the CPU side then make sure it
        // continues towards the memory side
        if (from_cpu_side) {
            if (isIOCache && enableDTA) {
                assert(0 && "If this happen, we need to implement the DTA functional access");
            }
            memSidePort.sendFunctional(pkt);
        } else {
            if (isLLC && isMultiPort) {
                assert(cpu_port_id != InvalidPortID);
                assert(cpu_port_id < cpuSidePortList.size());
                if (cpuSidePortList[cpu_port_id]->isSnooping()) {
                    cpuSidePortList[cpu_port_id]->sendFunctionalSnoop(pkt);
                }
            } else if (isIOCache && enableDTA) {
                assert(0 && "If this happen, we need to implement the DTA functional access");
            } else if (cpuSidePort.isSnooping()) {
                // if it came from the memory side, it must be a snoop request
                // and we should only forward it if we are forwarding snoops
                cpuSidePort.sendFunctionalSnoop(pkt);
            }
        }
    }
}

void
BaseCache::updateBlockData(CacheBlk *blk, const PacketPtr cpkt,
    bool has_old_data)
{
    DataUpdate data_update(regenerateBlkAddr(blk), blk->isSecure());
    if (ppDataUpdate->hasListeners()) {
        if (has_old_data) {
            data_update.oldData = std::vector<uint64_t>(blk->data,
                blk->data + (blkSize / sizeof(uint64_t)));
        }
    }

    // Actually perform the data update
    if (cpkt) {
        cpkt->writeDataToBlock(blk->data, blkSize);
    }

    if (ppDataUpdate->hasListeners()) {
        if (cpkt) {
            data_update.newData = std::vector<uint64_t>(blk->data,
                blk->data + (blkSize / sizeof(uint64_t)));
        }
        ppDataUpdate->notify(data_update);
    }
}

void
BaseCache::cmpAndSwap(CacheBlk *blk, PacketPtr pkt)
{
    assert(pkt->isRequest());

    uint64_t overwrite_val;
    bool overwrite_mem;
    uint64_t condition_val64;
    uint32_t condition_val32;

    int offset = pkt->getOffset(blkSize);
    uint8_t *blk_data = blk->data + offset;

    assert(sizeof(uint64_t) >= pkt->getSize());

    // Get a copy of the old block's contents for the probe before the update
    DataUpdate data_update(regenerateBlkAddr(blk), blk->isSecure());
    if (ppDataUpdate->hasListeners()) {
        data_update.oldData = std::vector<uint64_t>(blk->data,
            blk->data + (blkSize / sizeof(uint64_t)));
    }

    overwrite_mem = true;
    // keep a copy of our possible write value, and copy what is at the
    // memory address into the packet
    pkt->writeData((uint8_t *)&overwrite_val);
    pkt->setData(blk_data);

    if (pkt->req->isCondSwap()) {
        if (pkt->getSize() == sizeof(uint64_t)) {
            condition_val64 = pkt->req->getExtraData();
            overwrite_mem = !std::memcmp(&condition_val64, blk_data,
                                         sizeof(uint64_t));
        } else if (pkt->getSize() == sizeof(uint32_t)) {
            condition_val32 = (uint32_t)pkt->req->getExtraData();
            overwrite_mem = !std::memcmp(&condition_val32, blk_data,
                                         sizeof(uint32_t));
        } else
            panic("Invalid size for conditional read/write\n");
    }

    if (overwrite_mem) {
        std::memcpy(blk_data, &overwrite_val, pkt->getSize());
        blk->setCoherenceBits(CacheBlk::DirtyBit);

        if (ppDataUpdate->hasListeners()) {
            data_update.newData = std::vector<uint64_t>(blk->data,
                blk->data + (blkSize / sizeof(uint64_t)));
            ppDataUpdate->notify(data_update);
        }
    }
}

QueueEntry*
BaseCache::getNextQueueEntry()
{
    // Check both MSHR queue and write buffer for potential requests,
    // note that null does not mean there is no request, it could
    // simply be that it is not ready
    MSHR *miss_mshr  = mshrQueue.getNext();
    WriteQueueEntry *wq_entry = writeBuffer.getNext();

    // If we got a write buffer request ready, first priority is a
    // full write buffer, otherwise we favour the miss requests
    if (wq_entry && (writeBuffer.isFull() || !miss_mshr)) {
        // need to search MSHR queue for conflicting earlier miss.
        MSHR *conflict_mshr = mshrQueue.findPending(wq_entry);

        if (conflict_mshr && conflict_mshr->order < wq_entry->order) {
            // Service misses in order until conflict is cleared.
            return conflict_mshr;

            // @todo Note that we ignore the ready time of the conflict here
        }

        // No conflicts; issue write
        return wq_entry;
    } else if (miss_mshr) {
        // need to check for conflicting earlier writeback
        WriteQueueEntry *conflict_mshr = writeBuffer.findPending(miss_mshr);
        if (conflict_mshr) {
            // not sure why we don't check order here... it was in the
            // original code but commented out.

            // The only way this happens is if we are
            // doing a write and we didn't have permissions
            // then subsequently saw a writeback (owned got evicted)
            // We need to make sure to perform the writeback first
            // To preserve the dirty data, then we can issue the write

            // should we return wq_entry here instead?  I.e. do we
            // have to flush writes in order?  I don't think so... not
            // for Alpha anyway.  Maybe for x86?
            return conflict_mshr;

            // @todo Note that we ignore the ready time of the conflict here
        }

        // No conflicts; issue read
        return miss_mshr;
    }

    // fall through... no pending requests.  Try a prefetch.
    assert(!miss_mshr && !wq_entry);
    if (prefetcher && mshrQueue.canPrefetch() && !isBlocked()) {
        // If we have a miss queue slot, we can try a prefetch
        PacketPtr pkt = prefetcher->getPacket();
        if (pkt) {
            Addr pf_addr = pkt->getBlockAddr(blkSize);
            if (tags->findBlock(pf_addr, pkt->isSecure())) {
                DPRINTF(HWPrefetch, "Prefetch %#x has hit in cache, "
                        "dropped.\n", pf_addr);
                prefetcher->pfHitInCache();
                // free the request and packet
                delete pkt;
            } else if (mshrQueue.findMatch(pf_addr, pkt->isSecure())) {
                DPRINTF(HWPrefetch, "Prefetch %#x has hit in a MSHR, "
                        "dropped.\n", pf_addr);
                prefetcher->pfHitInMSHR();
                // free the request and packet
                delete pkt;
            } else if (writeBuffer.findMatch(pf_addr, pkt->isSecure())) {
                DPRINTF(HWPrefetch, "Prefetch %#x has hit in the "
                        "Write Buffer, dropped.\n", pf_addr);
                prefetcher->pfHitInWB();
                // free the request and packet
                delete pkt;
            } else {
                // Update statistic on number of prefetches issued
                // (hwpf_mshr_misses)
                assert(pkt->req->requestorId() < system->maxRequestors());
                stats.cmdStats(pkt).mshrMisses[pkt->req->requestorId()]++;

                // allocate an MSHR and return it, note
                // that we send the packet straight away, so do not
                // schedule the send
                return allocateMissBuffer(pkt, curTick(), false);
            }
        }
    }

    return nullptr;
}

bool
BaseCache::handleEvictions(std::vector<CacheBlk*> &evict_blks,
    PacketList &writebacks)
{
    bool replacement = false;
    for (const auto& blk : evict_blks) {
        if (blk->isValid()) {
            replacement = true;

            const MSHR* mshr =
                mshrQueue.findMatch(regenerateBlkAddr(blk), blk->isSecure());
            if (mshr) {
                // Must be an outstanding upgrade or clean request on a block
                // we're about to replace
                assert((!blk->isSet(CacheBlk::WritableBit) &&
                    mshr->needsWritable()) || mshr->isCleaning());
                return false;
            }
        }
    }

    // The victim will be replaced by a new entry, so increase the replacement
    // counter if a valid block is being replaced
    if (replacement) {
        stats.replacements++;

        // Evict valid blocks associated to this victim block
        for (auto& blk : evict_blks) {
            if (blk->isValid()) {
                evictBlock(blk, writebacks);
            }
        }
    }

    return true;
}

bool
BaseCache::updateCompressionData(CacheBlk *&blk, const uint64_t* data,
                                 PacketList &writebacks)
{
    // tempBlock does not exist in the tags, so don't do anything for it.
    if (blk == tempBlock) {
        return true;
    }

    // The compressor is called to compress the updated data, so that its
    // metadata can be updated.
    Cycles compression_lat = Cycles(0);
    Cycles decompression_lat = Cycles(0);
    const auto comp_data =
        compressor->compress(data, compression_lat, decompression_lat);
    std::size_t compression_size = comp_data->getSizeBits();

    // Get previous compressed size
    CompressionBlk* compression_blk = static_cast<CompressionBlk*>(blk);
    GEM5_VAR_USED const std::size_t prev_size = compression_blk->getSizeBits();

    // If compressed size didn't change enough to modify its co-allocatability
    // there is nothing to do. Otherwise we may be facing a data expansion
    // (block passing from more compressed to less compressed state), or a
    // data contraction (less to more).
    bool is_data_expansion = false;
    bool is_data_contraction = false;
    const CompressionBlk::OverwriteType overwrite_type =
        compression_blk->checkExpansionContraction(compression_size);
    std::string op_name = "";
    if (overwrite_type == CompressionBlk::DATA_EXPANSION) {
        op_name = "expansion";
        is_data_expansion = true;
    } else if ((overwrite_type == CompressionBlk::DATA_CONTRACTION) &&
        moveContractions) {
        op_name = "contraction";
        is_data_contraction = true;
    }

    // If block changed compression state, it was possibly co-allocated with
    // other blocks and cannot be co-allocated anymore, so one or more blocks
    // must be evicted to make room for the expanded/contracted block
    std::vector<CacheBlk*> evict_blks;
    if (is_data_expansion || is_data_contraction) {
        std::vector<CacheBlk*> evict_blks;
        bool victim_itself = false;
        CacheBlk *victim = nullptr;
        if (replaceExpansions || is_data_contraction) {
            victim = tags->findVictim(regenerateBlkAddr(blk),
                blk->isSecure(), compression_size, evict_blks);

            // It is valid to return nullptr if there is no victim
            if (!victim) {
                return false;
            }

            // If the victim block is itself the block won't need to be moved,
            // and the victim should not be evicted
            if (blk == victim) {
                victim_itself = true;
                auto it = std::find_if(evict_blks.begin(), evict_blks.end(),
                    [&blk](CacheBlk* evict_blk){ return evict_blk == blk; });
                evict_blks.erase(it);
            }

            // Print victim block's information
            DPRINTF(CacheRepl, "Data %s replacement victim: %s\n",
                op_name, victim->print());
        } else {
            // If we do not move the expanded block, we must make room for
            // the expansion to happen, so evict every co-allocated block
            const SuperBlk* superblock = static_cast<const SuperBlk*>(
                compression_blk->getSectorBlock());
            for (auto& sub_blk : superblock->blks) {
                if (sub_blk->isValid() && (blk != sub_blk)) {
                    evict_blks.push_back(sub_blk);
                }
            }
        }

        // Try to evict blocks; if it fails, give up on update
        if (!handleEvictions(evict_blks, writebacks)) {
            return false;
        }

        DPRINTF(CacheComp, "Data %s: [%s] from %d to %d bits\n",
                op_name, blk->print(), prev_size, compression_size);

        if (!victim_itself && (replaceExpansions || is_data_contraction)) {
            // Move the block's contents to the invalid block so that it now
            // co-allocates with the other existing superblock entry
            tags->moveBlock(blk, victim);
            blk = victim;
            compression_blk = static_cast<CompressionBlk*>(blk);
        }
    }

    // Update the number of data expansions/contractions
    if (is_data_expansion) {
        stats.dataExpansions++;
    } else if (is_data_contraction) {
        stats.dataContractions++;
    }

    compression_blk->setSizeBits(compression_size);
    compression_blk->setDecompressionLatency(decompression_lat);

    return true;
}

void
BaseCache::satisfyRequest(PacketPtr pkt, CacheBlk *blk, bool, bool)
{
    assert(pkt->isRequest());

    assert(blk && blk->isValid());
    // Occasionally this is not true... if we are a lower-level cache
    // satisfying a string of Read and ReadEx requests from
    // upper-level caches, a Read will mark the block as shared but we
    // can satisfy a following ReadEx anyway since we can rely on the
    // Read requestor(s) to have buffered the ReadEx snoop and to
    // invalidate their blocks after receiving them.
    // assert(!pkt->needsWritable() || blk->isSet(CacheBlk::WritableBit));
    assert(pkt->getOffset(blkSize) + pkt->getSize() <= blkSize);

    // Check RMW operations first since both isRead() and
    // isWrite() will be true for them
    if (pkt->cmd == MemCmd::SwapReq) {
        if (pkt->isAtomicOp()) {
            // Get a copy of the old block's contents for the probe before
            // the update
            DataUpdate data_update(regenerateBlkAddr(blk), blk->isSecure());
            if (ppDataUpdate->hasListeners()) {
                data_update.oldData = std::vector<uint64_t>(blk->data,
                    blk->data + (blkSize / sizeof(uint64_t)));
            }

            // extract data from cache and save it into the data field in
            // the packet as a return value from this atomic op
            int offset = tags->extractBlkOffset(pkt->getAddr());
            uint8_t *blk_data = blk->data + offset;
            pkt->setData(blk_data);

            // execute AMO operation
            (*(pkt->getAtomicOp()))(blk_data);

            // Inform of this block's data contents update
            if (ppDataUpdate->hasListeners()) {
                data_update.newData = std::vector<uint64_t>(blk->data,
                    blk->data + (blkSize / sizeof(uint64_t)));
                ppDataUpdate->notify(data_update);
            }

            // set block status to dirty
            blk->setCoherenceBits(CacheBlk::DirtyBit);
        } else {
            cmpAndSwap(blk, pkt);
        }
    } else if (pkt->isWrite()) {
        // we have the block in a writable state and can go ahead,
        // note that the line may be also be considered writable in
        // downstream caches along the path to memory, but always
        // Exclusive, and never Modified
        assert(blk->isSet(CacheBlk::WritableBit));
        // Write or WriteLine at the first cache with block in writable state
        if (blk->checkWrite(pkt)) {
            updateBlockData(blk, pkt, true);
        }
        // Always mark the line as dirty (and thus transition to the
        // Modified state) even if we are a failed StoreCond so we
        // supply data to any snoops that have appended themselves to
        // this cache before knowing the store will fail.
        blk->setCoherenceBits(CacheBlk::DirtyBit);
        DPRINTF(CacheVerbose, "%s for %s (write)\n", __func__, pkt->print());
    } else if (pkt->isRead()) {
        if (pkt->isLLSC()) {
            blk->trackLoadLocked(pkt);
        }

        // all read responses have a data payload
        assert(pkt->hasRespData());
        pkt->setDataFromBlock(blk->data, blkSize);
    } else if (pkt->isUpgrade()) {
        // sanity check
        assert(!pkt->hasSharers());

        if (blk->isSet(CacheBlk::DirtyBit)) {
            // we were in the Owned state, and a cache above us that
            // has the line in Shared state needs to be made aware
            // that the data it already has is in fact dirty
            pkt->setCacheResponding();
            blk->clearCoherenceBits(CacheBlk::DirtyBit);
        }
    } else if (pkt->isClean()) {
        blk->clearCoherenceBits(CacheBlk::DirtyBit);
    } else {
        assert(pkt->isInvalidate());
        // SHIN
        // invalidateBlock(blk);
        invalidateBlock(blk, isLLCisMLCIOInvalid(pkt));
        DPRINTF(CacheVerbose, "%s for %s (invalidation)\n", __func__,
                pkt->print());
    }
}

/////////////////////////////////////////////////////
//
// Access path: requests coming in from the CPU side
//
/////////////////////////////////////////////////////
Cycles
BaseCache::calculateTagOnlyLatency(const uint32_t delay,
                                   const Cycles lookup_lat) const
{
    // A tag-only access has to wait for the packet to arrive in order to
    // perform the tag lookup.
    return ticksToCycles(delay) + lookup_lat;
}

Cycles
BaseCache::calculateAccessLatency(const CacheBlk* blk, const uint32_t delay,
                                  const Cycles lookup_lat) const
{
    Cycles lat(0);

    if (blk != nullptr) {
        // As soon as the access arrives, for sequential accesses first access
        // tags, then the data entry. In the case of parallel accesses the
        // latency is dictated by the slowest of tag and data latencies.
        if (sequentialAccess) {
            lat = ticksToCycles(delay) + lookup_lat + dataLatency;
        } else {
            lat = ticksToCycles(delay) + std::max(lookup_lat, dataLatency);
        }

        // Check if the block to be accessed is available. If not, apply the
        // access latency on top of when the block is ready to be accessed.
        const Tick tick = curTick() + delay;
        const Tick when_ready = blk->getWhenReady();
        if (when_ready > tick &&
            ticksToCycles(when_ready - tick) > lat) {
            lat += ticksToCycles(when_ready - tick);
        }
    } else {
        // In case of a miss, we neglect the data access in a parallel
        // configuration (i.e., the data access will be stopped as soon as
        // we find out it is a miss), and use the tag-only latency.
        lat = calculateTagOnlyLatency(delay, lookup_lat);
    }

    return lat;
}

bool
BaseCache::access(PacketPtr pkt, CacheBlk *&blk, Cycles &lat,
                  PacketList &writebacks, bool is_ddio)
{
    // sanity check
    assert(pkt->isRequest());

    chatty_assert(!(isReadOnly && pkt->isWrite()),
                  "Should never see a write in a read-only cache %s\n",
                  name());

    // Access block in the tags
    Cycles tag_latency(0);
    blk = tags->accessBlock(pkt, tag_latency);

    DPRINTF(Cache, "%s for %s %s\n", __func__, pkt->print(),
            blk ? "hit " + blk->print() : "miss");

    if (pkt->req->isCacheMaintenance()) {
        // A cache maintenance operation is always forwarded to the
        // memory below even if the block is found in dirty state.

        // We defer any changes to the state of the block until we
        // create and mark as in service the mshr for the downstream
        // packet.

        // Calculate access latency on top of when the packet arrives. This
        // takes into account the bus delay.
        lat = calculateTagOnlyLatency(pkt->headerDelay, tag_latency);

        return false;
    }

    if (pkt->isEviction()) {
        // We check for presence of block in above caches before issuing
        // Writeback or CleanEvict to write buffer. Therefore the only
        // possible cases can be of a CleanEvict packet coming from above
        // encountering a Writeback generated in this cache peer cache and
        // waiting in the write buffer. Cases of upper level peer caches
        // generating CleanEvict and Writeback or simply CleanEvict and
        // CleanEvict almost simultaneously will be caught by snoops sent out
        // by crossbar.
        WriteQueueEntry *wb_entry = writeBuffer.findMatch(pkt->getAddr(),
                                                          pkt->isSecure());
        if (wb_entry) {
            assert(wb_entry->getNumTargets() == 1);
            PacketPtr wbPkt = wb_entry->getTarget()->pkt;
            assert(wbPkt->isWriteback());

            // SHIN. ADQ
            if(isIOCache){
                if(pkt->cmd == MemCmd::WriteReq || pkt->cmd == MemCmd::WriteLineReq){
                    wbPkt->setDdioPrefetchDestination(pkt->getDdioPrefetchDestination());
                    if(pkt->isDdioPkt()) wbPkt->setDdioPkt();
                    if(pkt->isDdioHeader()) wbPkt->setDdioHeader();
                    //DPRINTF(AdaptiveDdioOtf, "wb_entry. pkt adq %d, pkt %s\n", pkt->getAdqQ(), pkt->print());
                }
            }

            if (pkt->isCleanEviction()) {
                // The CleanEvict and WritebackClean snoops into other
                // peer caches of the same level while traversing the
                // crossbar. If a copy of the block is found, the
                // packet is deleted in the crossbar. Hence, none of
                // the other upper level caches connected to this
                // cache have the block, so we can clear the
                // BLOCK_CACHED flag in the Writeback if set and
                // discard the CleanEvict by returning true.
                wbPkt->clearBlockCached();

                // A clean evict does not need to access the data array
                lat = calculateTagOnlyLatency(pkt->headerDelay, tag_latency);

                return true;
            } else {
                assert(pkt->cmd == MemCmd::WritebackDirty);
                // Dirty writeback from above trumps our clean
                // writeback... discard here
                // Note: markInService will remove entry from writeback buffer.
                markInService(wb_entry);
                delete wbPkt;
            }
        }
    }

    // The critical latency part of a write depends only on the tag access
    if (pkt->isWrite()) {
        lat = calculateTagOnlyLatency(pkt->headerDelay, tag_latency);
    }

    // Writeback handling is special case.  We can write the block into
    // the cache without having a writeable copy (or any copy at all).
    if (pkt->isWriteback()) {
        assert(blkSize == pkt->getSize());

        // we could get a clean writeback while we are having
        // outstanding accesses to a block, do the simple thing for
        // now and drop the clean writeback so that we do not upset
        // any ordering/decisions about ownership already taken
        if (pkt->cmd == MemCmd::WritebackClean &&
            mshrQueue.findMatch(pkt->getAddr(), pkt->isSecure())) {
            DPRINTF(Cache, "Clean writeback %#llx to block with MSHR, "
                    "dropping\n", pkt->getAddr());

            // A writeback searches for the block, then writes the data.
            // As the writeback is being dropped, the data is not touched,
            // and we just had to wait for the time to find a match in the
            // MSHR. As of now assume a mshr queue search takes as long as
            // a tag lookup for simplicity.
            return true;
        }

        const bool has_old_data = blk && blk->isValid();
        if (!blk) {
            // need to do a replacement
            blk = allocateBlock(pkt, writebacks);
            if (!blk) {
                // no replaceable block available: give up, fwd to next level.
                incMissCount(pkt);
                return false;
            }

            blk->setCoherenceBits(CacheBlk::ReadableBit);
        } else if (compressor) {
            // This is an overwrite to an existing block, therefore we need
            // to check for data expansion (i.e., block was compressed with
            // a smaller size, and now it doesn't fit the entry anymore).
            // If that is the case we might need to evict blocks.
            if (!updateCompressionData(blk, pkt->getConstPtr<uint64_t>(),
                writebacks)) {
                invalidateBlock(blk);
                return false;
            }
        }

        // only mark the block dirty if we got a writeback command,
        // and leave it as is for a clean writeback
        if (pkt->cmd == MemCmd::WritebackDirty) {
            // TODO: the coherent cache can assert that the dirty bit is set
            blk->setCoherenceBits(CacheBlk::DirtyBit);
        }
        // if the packet does not have sharers, it is passing
        // writable, and we got the writeback in Modified or Exclusive
        // state, if not we are in the Owned or Shared state
        if (!pkt->hasSharers()) {
            blk->setCoherenceBits(CacheBlk::WritableBit);
        }
        // nothing else to do; writeback doesn't expect response
        assert(!pkt->needsResponse());

        updateBlockData(blk, pkt, has_old_data);
        DPRINTF(Cache, "%s new state is %s\n", __func__, blk->print());
        incHitCount(pkt);

        // When the packet metadata arrives, the tag lookup will be done while
        // the payload is arriving. Then the block will be ready to access as
        // soon as the fill is done
        blk->setWhenReady(clockEdge(fillLatency) + pkt->headerDelay +
            std::max(cyclesToTicks(tag_latency), (uint64_t)pkt->payloadDelay));

        // SHIN
        if(isIOCache){
            if(pkt->getDdioPrefetchDestination() != -1){
                // Not Works
                blk->setDdioPrefetchDestination(pkt->getDdioPrefetchDestination());
                
            }
        }

        return true;
    } else if (pkt->cmd == MemCmd::CleanEvict) {
        // A CleanEvict does not need to access the data array
        lat = calculateTagOnlyLatency(pkt->headerDelay, tag_latency);

        if (blk) {
            // Found the block in the tags, need to stop CleanEvict from
            // propagating further down the hierarchy. Returning true will
            // treat the CleanEvict like a satisfied write request and delete
            // it.
            return true;
        }
        // We didn't find the block here, propagate the CleanEvict further
        // down the memory hierarchy. Returning false will treat the CleanEvict
        // like a Writeback which could not find a replaceable block so has to
        // go to next level.
        return false;
    } else if (pkt->cmd == MemCmd::WriteClean) {
        // WriteClean handling is a special case. We can allocate a
        // block directly if it doesn't exist and we can update the
        // block immediately. The WriteClean transfers the ownership
        // of the block as well.
        assert(blkSize == pkt->getSize());

        const bool has_old_data = blk && blk->isValid();
        if (!blk) {
            if (pkt->writeThrough()) {
                // if this is a write through packet, we don't try to
                // allocate if the block is not present
                return false;
            } else {
                // a writeback that misses needs to allocate a new block
                // SHIN Ported from base
                //blk = allocateBlock(pkt, writebacks);
                blk = allocateBlock(pkt, writebacks, is_ddio);

                if (!blk) {
                    // no replaceable block available: give up, fwd to
                    // next level.
                    incMissCount(pkt);
                    return false;
                }

                blk->setCoherenceBits(CacheBlk::ReadableBit);
            }
        } else if (compressor) {
            // This is an overwrite to an existing block, therefore we need
            // to check for data expansion (i.e., block was compressed with
            // a smaller size, and now it doesn't fit the entry anymore).
            // If that is the case we might need to evict blocks.
            if (!updateCompressionData(blk, pkt->getConstPtr<uint64_t>(),
                writebacks)) {
                invalidateBlock(blk);
                return false;
            }
        }

        // at this point either this is a writeback or a write-through
        // write clean operation and the block is already in this
        // cache, we need to update the data and the block flags
        assert(blk);
        // TODO: the coherent cache can assert that the dirty bit is set
        if (!pkt->writeThrough()) {
            blk->setCoherenceBits(CacheBlk::DirtyBit);
        }
        // nothing else to do; writeback doesn't expect response
        assert(!pkt->needsResponse());

        updateBlockData(blk, pkt, has_old_data);
        DPRINTF(Cache, "%s new state is %s\n", __func__, blk->print());

        incHitCount(pkt);

        // When the packet metadata arrives, the tag lookup will be done while
        // the payload is arriving. Then the block will be ready to access as
        // soon as the fill is done
        blk->setWhenReady(clockEdge(fillLatency) + pkt->headerDelay +
            std::max(cyclesToTicks(tag_latency), (uint64_t)pkt->payloadDelay));

        // SHIN
        if(isIOCache){
            if(pkt->getDdioPrefetchDestination() != -1){
                blk->setDdioPrefetchDestination(pkt->getDdioPrefetchDestination());
                if(pkt->isDdioPkt()) blk->setDdioPkt();
                if(pkt->isDdioHeader()) blk->setDdioHeader();
                //DPRINTF(AdaptiveDdioOtf, "Alloc blk WriteClean. pkt adq %d, pkt %s\n", pkt->getAdqQ(), pkt->print());
            }
        }

        // If this a write-through packet it will be sent to cache below
        return !pkt->writeThrough();
    } else if (blk && (pkt->needsWritable() ?
            blk->isSet(CacheBlk::WritableBit) :
            blk->isSet(CacheBlk::ReadableBit))) {
        // OK to satisfy access
        incHitCount(pkt);

        // Calculate access latency based on the need to access the data array
        if (pkt->isRead()) {
            lat = calculateAccessLatency(blk, pkt->headerDelay, tag_latency);

            // When a block is compressed, it must first be decompressed
            // before being read. This adds to the access latency.
            if (compressor) {
                lat += compressor->getDecompressionLatency(blk);
            }
        } else {
            lat = calculateTagOnlyLatency(pkt->headerDelay, tag_latency);
        }

        satisfyRequest(pkt, blk);
        maintainClusivity(pkt->fromCache(), blk);

        return true;
    }

    // Can't satisfy access normally... either no block (blk == nullptr)
    // or have block but need writable

    incMissCount(pkt);

    lat = calculateAccessLatency(blk, pkt->headerDelay, tag_latency);

    if (!blk && pkt->isLLSC() && pkt->isWrite()) {
        // complete miss on store conditional... just give up now
        pkt->req->setExtraData(0);
        return true;
    }

    return false;
}

void
BaseCache::maintainClusivity(bool from_cache, CacheBlk *blk)
{
    if (from_cache && blk && blk->isValid() &&
        !blk->isSet(CacheBlk::DirtyBit) && clusivity == enums::mostly_excl) {
        // if we have responded to a cache, and our block is still
        // valid, but not dirty, and this cache is mostly exclusive
        // with respect to the cache above, drop the block
        invalidateBlock(blk);
    }
}

CacheBlk*
BaseCache::handleFill(PacketPtr pkt, CacheBlk *blk, PacketList &writebacks,
                      bool allocate, bool is_ddio)      // SHIN
{
    assert(pkt->isResponse());
    Addr addr = pkt->getAddr();
    bool is_secure = pkt->isSecure();
    const bool has_old_data = blk && blk->isValid();
    const std::string old_state = blk ? blk->print() : "";

    // When handling a fill, we should have no writes to this line.
    assert(addr == pkt->getBlockAddr(blkSize));
    assert(!writeBuffer.findMatch(addr, is_secure));

    if (!blk) {
        // better have read new data...
        assert(pkt->hasData() || pkt->cmd == MemCmd::InvalidateResp);

        // need to do a replacement if allocating, otherwise we stick
        // with the temporary storage
        // SHIN
        // blk = allocate ? allocateBlock(pkt, writebacks) : nullptr;
        blk = allocate ? allocateBlock(pkt, writebacks, is_ddio) : nullptr;

        if (!blk) {
            // No replaceable block or a mostly exclusive
            // cache... just use temporary storage to complete the
            // current request and then get rid of it
            blk = tempBlock;
            tempBlock->insert(addr, is_secure);
            DPRINTF(Cache, "using temp block for %#llx (%s)\n", addr,
                    is_secure ? "s" : "ns");
        }
    } else {
        // existing block... probably an upgrade
        // don't clear block status... if block is already dirty we
        // don't want to lose that
    }

    // Block is guaranteed to be valid at this point
    assert(blk->isValid());
    assert(blk->isSecure() == is_secure);
    assert(regenerateBlkAddr(blk) == addr);

    blk->setCoherenceBits(CacheBlk::ReadableBit);

    // sanity check for whole-line writes, which should always be
    // marked as writable as part of the fill, and then later marked
    // dirty as part of satisfyRequest
    if (pkt->cmd == MemCmd::InvalidateResp) {
        assert(!pkt->hasSharers());
    }

    // here we deal with setting the appropriate state of the line,
    // and we start by looking at the hasSharers flag, and ignore the
    // cacheResponding flag (normally signalling dirty data) if the
    // packet has sharers, thus the line is never allocated as Owned
    // (dirty but not writable), and always ends up being either
    // Shared, Exclusive or Modified, see Packet::setCacheResponding
    // for more details
    if (!pkt->hasSharers()) {
        // we could get a writable line from memory (rather than a
        // cache) even in a read-only cache, note that we set this bit
        // even for a read-only cache, possibly revisit this decision
        blk->setCoherenceBits(CacheBlk::WritableBit);

        // check if we got this via cache-to-cache transfer (i.e., from a
        // cache that had the block in Modified or Owned state)
        if (pkt->cacheResponding()) {
            // we got the block in Modified state, and invalidated the
            // owners copy
            blk->setCoherenceBits(CacheBlk::DirtyBit);

            chatty_assert(!isReadOnly, "Should never see dirty snoop response "
                          "in read-only cache %s\n", name());

        }
    }

    DPRINTF(Cache, "Block addr %#llx (%s) moving from %s to %s\n",
            addr, is_secure ? "s" : "ns", old_state, blk->print());

    // if we got new data, copy it in (checking for a read response
    // and a response that has data is the same in the end)
    if (pkt->isRead()) {
        // sanity checks
        assert(pkt->hasData());
        assert(pkt->getSize() == blkSize);

        updateBlockData(blk, pkt, has_old_data);
    }
    // The block will be ready when the payload arrives and the fill is done
    blk->setWhenReady(clockEdge(fillLatency) + pkt->headerDelay +
                      pkt->payloadDelay);

    // SHIN. Set ADQ
    if(isIOCache)
    {
        if(pkt->getDdioPrefetchDestination() != -1)
        {
            blk->setDdioPrefetchDestination(pkt->getDdioPrefetchDestination());
            if(pkt->isDdioHeader()) blk->setDdioHeader();
            
            if(pkt->isDdioPkt())
            {
                blk->setDdioPkt();
            }
        }
    }

    return blk;
}

CacheBlk*
BaseCache::allocateBlock(const PacketPtr pkt, PacketList &writebacks, bool is_ddio) // SHIN
{
    // Get address
    const Addr addr = pkt->getAddr();

    // Get secure bit
    const bool is_secure = pkt->isSecure();

    // Block size and compression related access latency. Only relevant if
    // using a compressor, otherwise there is no extra delay, and the block
    // is fully sized
    std::size_t blk_size_bits = blkSize*8;
    Cycles compression_lat = Cycles(0);
    Cycles decompression_lat = Cycles(0);

    // If a compressor is being used, it is called to compress data before
    // insertion. Although in Gem5 the data is stored uncompressed, even if a
    // compressor is used, the compression/decompression methods are called to
    // calculate the amount of extra cycles needed to read or write compressed
    // blocks.
    if (compressor && pkt->hasData()) {
        const auto comp_data = compressor->compress(
            pkt->getConstPtr<uint64_t>(), compression_lat, decompression_lat);
        blk_size_bits = comp_data->getSizeBits();
    }

    // Find replacement victim
    std::vector<CacheBlk*> evict_blks;
    // SHIN. Change for DDIO/IDIO
    // CacheBlk *victim = tags->findVictim(addr, is_secure, blk_size_bits,
    //                                     evict_blks);
    CacheBlk *victim;

    if (is_ddio) {
        victim = tags->findVictimWayPart(addr, is_secure, evict_blks, ddioWayPart);
    } else {
        victim = tags->findVictim(addr, is_secure, blk_size_bits, evict_blks);
    }

    // It is valid to return nullptr if there is no victim
    if (!victim)
        return nullptr;

    // Print victim block's information
    DPRINTF(CacheRepl, "Replacement victim: %s\n", victim->print());

    // Try to evict blocks; if it fails, give up on allocation
    if (!handleEvictions(evict_blks, writebacks)) {
        return nullptr;
    }

    // Insert new block at victimized entry
    tags->insertBlock(pkt, victim);

    // If using a compressor, set compression data. This must be done after
    // insertion, as the compression bit may be set.
    if (compressor) {
        compressor->setSizeBits(victim, blk_size_bits);
        compressor->setDecompressionLatency(victim, decompression_lat);
    }

    // SHIN
    victim->setDdioPrefetchDestination(pkt->getDdioPrefetchDestination());

    return victim;
}

void
BaseCache::invalidateBlock(CacheBlk *blk, bool is_llc_inv) // SHIN.
{
    // If block is still marked as prefetched, then it hasn't been used
    if (blk->wasPrefetched()) {
        prefetcher->prefetchUnused();
    }

    // Notify that the data contents for this address are no longer present
    updateBlockData(blk, nullptr, blk->isValid());

    // If handling a block present in the Tags, let it do its invalidation
    // process, which will update stats and invalidate the block itself
    if (blk != tempBlock) {
        // SHIN
        // tags->invalidate(blk);
        if (is_llc_inv)
            tags->invalidateDDIO(blk);
        else
            tags->invalidate(blk);
    } else {
        tempBlock->invalidate();
    }
}

void
BaseCache::evictBlock(CacheBlk *blk, PacketList &writebacks)
{
    PacketPtr pkt = evictBlock(blk);
    if (pkt) {
        writebacks.push_back(pkt);
    }
}

PacketPtr
BaseCache::writebackBlk(CacheBlk *blk)
{
    chatty_assert(!isReadOnly || writebackClean,
                  "Writeback from read-only cache");
    assert(blk && blk->isValid() &&
        (blk->isSet(CacheBlk::DirtyBit) || writebackClean));

    stats.writebacks[Request::wbRequestorId]++;

    RequestPtr req = std::make_shared<Request>(
        regenerateBlkAddr(blk), blkSize, 0, Request::wbRequestorId);

    if (blk->isSecure())
        req->setFlags(Request::SECURE);

    req->taskId(blk->getTaskId());

    PacketPtr pkt =
        new Packet(req, blk->isSet(CacheBlk::DirtyBit) ?
                   MemCmd::WritebackDirty : MemCmd::WritebackClean);

    // SHIN
    if(isIOCache){
        // SHIN. Adpative-DDIO ADQ
        pkt->setDdioPrefetchDestination(blk->getDdioPrefetchDestination());
        pkt->setBlockIO();
        
        
        if(blk->isDdioPkt()) pkt->setDdioPkt();
        if(blk->isDdioHeader()) pkt->setDdioHeader();

        //DPRINTF(AdaptiveDdioOtf, "writebackBlk, qid %d, pkt %s\n", blk->getAdqQ(), pkt->print());
        
        
        blk->setDdioPrefetchDestination(-1);
        
    }

    DPRINTF(Cache, "Create Writeback %s writable: %d, dirty: %d\n",
        pkt->print(), blk->isSet(CacheBlk::WritableBit),
        blk->isSet(CacheBlk::DirtyBit));

    if (blk->isSet(CacheBlk::WritableBit)) {
        // not asserting shared means we pass the block in modified
        // state, mark our own block non-writeable
        blk->clearCoherenceBits(CacheBlk::WritableBit);
    } else {
        // we are in the Owned state, tell the receiver
        pkt->setHasSharers();
    }

    // make sure the block is not marked dirty
    blk->clearCoherenceBits(CacheBlk::DirtyBit);

    pkt->allocate();
    pkt->setDataFromBlock(blk->data, blkSize);

    // SHIN
    if (isIOCache)
        pkt->setBlockIO();

    // When a block is compressed, it must first be decompressed before being
    // sent for writeback.
    if (compressor) {
        pkt->payloadDelay = compressor->getDecompressionLatency(blk);
    }

    return pkt;
}

PacketPtr
BaseCache::writecleanBlk(CacheBlk *blk, Request::Flags dest, PacketId id, PortID cpu_side_port_id)
{
    RequestPtr req = std::make_shared<Request>(
        regenerateBlkAddr(blk), blkSize, 0, Request::wbRequestorId);

    if (blk->isSecure()) {
        req->setFlags(Request::SECURE);
    }
    req->taskId(blk->getTaskId());

    PacketPtr pkt = new Packet(req, MemCmd::WriteClean, blkSize, id, cpu_side_port_id);

    if (dest) {
        req->setFlags(dest);
        pkt->setWriteThrough();
    }

    DPRINTF(Cache, "Create %s writable: %d, dirty: %d\n", pkt->print(),
            blk->isSet(CacheBlk::WritableBit), blk->isSet(CacheBlk::DirtyBit));

    if (blk->isSet(CacheBlk::WritableBit)) {
        // not asserting shared means we pass the block in modified
        // state, mark our own block non-writeable
        blk->clearCoherenceBits(CacheBlk::WritableBit);
    } else {
        // we are in the Owned state, tell the receiver
        pkt->setHasSharers();
    }

    // make sure the block is not marked dirty
    blk->clearCoherenceBits(CacheBlk::DirtyBit);

    pkt->allocate();
    pkt->setDataFromBlock(blk->data, blkSize);

    // SHIN
    if (isIOCache)
        pkt->setBlockIO();

    // When a block is compressed, it must first be decompressed before being
    // sent for writeback.
    if (compressor) {
        pkt->payloadDelay = compressor->getDecompressionLatency(blk);
    }

    return pkt;
}


void
BaseCache::memWriteback()
{
    tags->forEachBlk([this](CacheBlk &blk) { writebackVisitor(blk); });
}

void
BaseCache::memInvalidate()
{
    tags->forEachBlk([this](CacheBlk &blk) { invalidateVisitor(blk); });
}

bool
BaseCache::isDirty() const
{
    return tags->anyBlk([](CacheBlk &blk) {
        return blk.isSet(CacheBlk::DirtyBit); });
}

bool
BaseCache::coalesce() const
{
    return writeAllocator && writeAllocator->coalesce();
}

void
BaseCache::writebackVisitor(CacheBlk &blk)
{
    if (blk.isSet(CacheBlk::DirtyBit)) {
        assert(blk.isValid());

        RequestPtr request = std::make_shared<Request>(
            regenerateBlkAddr(&blk), blkSize, 0, Request::funcRequestorId);

        request->taskId(blk.getTaskId());
        if (blk.isSecure()) {
            request->setFlags(Request::SECURE);
        }

        Packet packet(request, MemCmd::WriteReq);
        // SHIN
        if (isIOCache)
            packet.setBlockIO();

        packet.dataStatic(blk.data);

        memSidePort.sendFunctional(&packet);

        blk.clearCoherenceBits(CacheBlk::DirtyBit);
    }
}

void
BaseCache::invalidateVisitor(CacheBlk &blk)
{
    if (blk.isSet(CacheBlk::DirtyBit))
        warn_once("Invalidating dirty cache lines. " \
                  "Expect things to break.\n");

    if (blk.isValid()) {
        assert(!blk.isSet(CacheBlk::DirtyBit));
        invalidateBlock(&blk);
    }
}

Tick
BaseCache::nextQueueReadyTime() const
{
    Tick nextReady = std::min(mshrQueue.nextReadyTime(),
                              writeBuffer.nextReadyTime());

    // Don't signal prefetch ready time if no MSHRs available
    // Will signal once enoguh MSHRs are deallocated
    if (prefetcher && mshrQueue.canPrefetch() && !isBlocked()) {
        nextReady = std::min(nextReady,
                             prefetcher->nextPrefetchReadyTime());
    }

    return nextReady;
}


bool
BaseCache::sendMSHRQueuePacket(MSHR* mshr)
{
    assert(mshr);

    // use request from 1st target
    PacketPtr tgt_pkt = mshr->getTarget()->pkt;

    DPRINTF(Cache, "%s: MSHR %s\n", __func__, tgt_pkt->print());

    // if the cache is in write coalescing mode or (additionally) in
    // no allocation mode, and we have a write packet with an MSHR
    // that is not a whole-line write (due to incompatible flags etc),
    // then reset the write mode
    if (writeAllocator && writeAllocator->coalesce() && tgt_pkt->isWrite()) {
        if (!mshr->isWholeLineWrite()) {
            // if we are currently write coalescing, hold on the
            // MSHR as many cycles extra as we need to completely
            // write a cache line
            if (writeAllocator->delay(mshr->blkAddr)) {
                Tick delay = blkSize / tgt_pkt->getSize() * clockPeriod();
                DPRINTF(CacheVerbose, "Delaying pkt %s %llu ticks to allow "
                        "for write coalescing\n", tgt_pkt->print(), delay);
                mshrQueue.delay(mshr, delay);
                return false;
            } else {
                writeAllocator->reset();
            }
        } else {
            writeAllocator->resetDelay(mshr->blkAddr);
        }
    }

    CacheBlk *blk = tags->findBlock(mshr->blkAddr, mshr->isSecure);

    // either a prefetch that is not present upstream, or a normal
    // MSHR request, proceed to get the packet to send downstream
    PacketPtr pkt = createMissPacket(tgt_pkt, blk, mshr->needsWritable(),
                                     mshr->isWholeLineWrite());

    mshr->isForward = (pkt == nullptr);

    if (mshr->isForward) {
        // not a cache block request, but a response is expected
        // make copy of current packet to forward, keep current
        // copy for response handling
        pkt = new Packet(tgt_pkt, false, true);
        assert(!pkt->isWrite());
    }

    // play it safe and append (rather than set) the sender state,
    // as forwarded packets may already have existing state
    pkt->pushSenderState(mshr);

    if (pkt->isClean() && blk && blk->isSet(CacheBlk::DirtyBit)) {
        // A cache clean opearation is looking for a dirty block. Mark
        // the packet so that the destination xbar can determine that
        // there will be a follow-up write packet as well.
        pkt->setSatisfied();
    }

    if (!memSidePort.sendTimingReq(pkt)) {
        // we are awaiting a retry, but we
        // delete the packet and will be creating a new packet
        // when we get the opportunity
        delete pkt;

        // note that we have now masked any requestBus and
        // schedSendEvent (we will wait for a retry before
        // doing anything), and this is so even if we do not
        // care about this packet and might override it before
        // it gets retried
        return true;
    } else {
        // As part of the call to sendTimingReq the packet is
        // forwarded to all neighbouring caches (and any caches
        // above them) as a snoop. Thus at this point we know if
        // any of the neighbouring caches are responding, and if
        // so, we know it is dirty, and we can determine if it is
        // being passed as Modified, making our MSHR the ordering
        // point
        bool pending_modified_resp = !pkt->hasSharers() &&
            pkt->cacheResponding();
        markInService(mshr, pending_modified_resp);

        if (pkt->isClean() && blk && blk->isSet(CacheBlk::DirtyBit)) {
            // A cache clean opearation is looking for a dirty
            // block. If a dirty block is encountered a WriteClean
            // will update any copies to the path to the memory
            // until the point of reference.
            DPRINTF(CacheVerbose, "%s: packet %s found block: %s\n",
                    __func__, pkt->print(), blk->print());
            PacketPtr wb_pkt = writecleanBlk(blk, pkt->req->getDest(),
                                             pkt->id, pkt->cpu_side_port_id);
            PacketList writebacks;
            writebacks.push_back(wb_pkt);
            doWritebacks(writebacks, 0);
        }

        return false;
    }
}

bool
BaseCache::sendWriteQueuePacket(WriteQueueEntry* wq_entry)
{
    assert(wq_entry);

    // always a single target for write queue entries
    PacketPtr tgt_pkt = wq_entry->getTarget()->pkt;

    DPRINTF(Cache, "%s: write %s\n", __func__, tgt_pkt->print());

    // forward as is, both for evictions and uncacheable writes
    if (!memSidePort.sendTimingReq(tgt_pkt)) {
        // note that we have now masked any requestBus and
        // schedSendEvent (we will wait for a retry before
        // doing anything), and this is so even if we do not
        // care about this packet and might override it before
        // it gets retried
        return true;
    } else {
        markInService(wq_entry);
        return false;
    }
}

void
BaseCache::serialize(CheckpointOut &cp) const
{
    bool dirty(isDirty());

    if (dirty) {
        warn("*** The cache still contains dirty data. ***\n");
        warn("    Make sure to drain the system using the correct flags.\n");
        warn("    This checkpoint will not restore correctly " \
             "and dirty data in the cache will be lost!\n");
    }

    // Since we don't checkpoint the data in the cache, any dirty data
    // will be lost when restoring from a checkpoint of a system that
    // wasn't drained properly. Flag the checkpoint as invalid if the
    // cache contains dirty data.
    bool bad_checkpoint(dirty);
    SERIALIZE_SCALAR(bad_checkpoint);
}

void
BaseCache::unserialize(CheckpointIn &cp)
{
    bool bad_checkpoint;
    UNSERIALIZE_SCALAR(bad_checkpoint);
    if (bad_checkpoint) {
        fatal("Restoring from checkpoints with dirty caches is not "
              "supported in the classic memory system. Please remove any "
              "caches or drain them properly before taking checkpoints.\n");
    }
}


BaseCache::CacheCmdStats::CacheCmdStats(BaseCache &c,
                                        const std::string &name)
    : statistics::Group(&c, name.c_str()), cache(c),
      ADD_STAT(hits, statistics::units::Count::get(),
               ("number of " + name + " hits").c_str()),
      ADD_STAT(misses, statistics::units::Count::get(),
               ("number of " + name + " misses").c_str()),
      ADD_STAT(missLatency, statistics::units::Tick::get(),
               ("number of " + name + " miss ticks").c_str()),
      ADD_STAT(accesses, statistics::units::Count::get(),
               ("number of " + name + " accesses(hits+misses)").c_str()),
      ADD_STAT(missRate, statistics::units::Ratio::get(),
               ("miss rate for " + name + " accesses").c_str()),
      ADD_STAT(avgMissLatency, statistics::units::Rate<
                    statistics::units::Tick, statistics::units::Count>::get(),
               ("average " + name + " miss latency").c_str()),
      ADD_STAT(mshrHits, statistics::units::Count::get(),
               ("number of " + name + " MSHR hits").c_str()),
      ADD_STAT(mshrMisses, statistics::units::Count::get(),
               ("number of " + name + " MSHR misses").c_str()),
      ADD_STAT(mshrUncacheable, statistics::units::Count::get(),
               ("number of " + name + " MSHR uncacheable").c_str()),
      ADD_STAT(mshrMissLatency, statistics::units::Tick::get(),
               ("number of " + name + " MSHR miss ticks").c_str()),
      ADD_STAT(mshrUncacheableLatency, statistics::units::Tick::get(),
               ("number of " + name + " MSHR uncacheable ticks").c_str()),
      ADD_STAT(mshrMissRate, statistics::units::Ratio::get(),
               ("mshr miss rate for " + name + " accesses").c_str()),
      ADD_STAT(avgMshrMissLatency, statistics::units::Rate<
                    statistics::units::Tick, statistics::units::Count>::get(),
               ("average " + name + " mshr miss latency").c_str()),
      ADD_STAT(avgMshrUncacheableLatency, statistics::units::Rate<
                    statistics::units::Tick, statistics::units::Count>::get(),
               ("average " + name + " mshr uncacheable latency").c_str())
{
}

void
BaseCache::CacheCmdStats::regStatsFromParent()
{
    using namespace statistics;

    statistics::Group::regStats();
    System *system = cache.system;
    const auto max_requestors = system->maxRequestors();

    hits
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        hits.subname(i, system->getRequestorName(i));
    }

    // Miss statistics
    misses
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        misses.subname(i, system->getRequestorName(i));
    }

    // Miss latency statistics
    missLatency
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        missLatency.subname(i, system->getRequestorName(i));
    }

    // access formulas
    accesses.flags(total | nozero | nonan);
    accesses = hits + misses;
    for (int i = 0; i < max_requestors; i++) {
        accesses.subname(i, system->getRequestorName(i));
    }

    // miss rate formulas
    missRate.flags(total | nozero | nonan);
    missRate = misses / accesses;
    for (int i = 0; i < max_requestors; i++) {
        missRate.subname(i, system->getRequestorName(i));
    }

    // miss latency formulas
    avgMissLatency.flags(total | nozero | nonan);
    avgMissLatency = missLatency / misses;
    for (int i = 0; i < max_requestors; i++) {
        avgMissLatency.subname(i, system->getRequestorName(i));
    }

    // MSHR statistics
    // MSHR hit statistics
    mshrHits
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        mshrHits.subname(i, system->getRequestorName(i));
    }

    // MSHR miss statistics
    mshrMisses
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        mshrMisses.subname(i, system->getRequestorName(i));
    }

    // MSHR miss latency statistics
    mshrMissLatency
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        mshrMissLatency.subname(i, system->getRequestorName(i));
    }

    // MSHR uncacheable statistics
    mshrUncacheable
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        mshrUncacheable.subname(i, system->getRequestorName(i));
    }

    // MSHR miss latency statistics
    mshrUncacheableLatency
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        mshrUncacheableLatency.subname(i, system->getRequestorName(i));
    }

    // MSHR miss rate formulas
    mshrMissRate.flags(total | nozero | nonan);
    mshrMissRate = mshrMisses / accesses;

    for (int i = 0; i < max_requestors; i++) {
        mshrMissRate.subname(i, system->getRequestorName(i));
    }

    // mshrMiss latency formulas
    avgMshrMissLatency.flags(total | nozero | nonan);
    avgMshrMissLatency = mshrMissLatency / mshrMisses;
    for (int i = 0; i < max_requestors; i++) {
        avgMshrMissLatency.subname(i, system->getRequestorName(i));
    }

    // mshrUncacheable latency formulas
    avgMshrUncacheableLatency.flags(total | nozero | nonan);
    avgMshrUncacheableLatency = mshrUncacheableLatency / mshrUncacheable;
    for (int i = 0; i < max_requestors; i++) {
        avgMshrUncacheableLatency.subname(i, system->getRequestorName(i));
    }
}

BaseCache::CacheStats::CacheStats(BaseCache &c)
    : statistics::Group(&c), cache(c),

    ADD_STAT(demandHits, statistics::units::Count::get(),
             "number of demand (read+write) hits"),
    ADD_STAT(overallHits, statistics::units::Count::get(),
             "number of overall hits"),
    ADD_STAT(demandMisses, statistics::units::Count::get(),
             "number of demand (read+write) misses"),
    ADD_STAT(overallMisses, statistics::units::Count::get(),
             "number of overall misses"),
    ADD_STAT(demandMissLatency, statistics::units::Tick::get(),
             "number of demand (read+write) miss ticks"),
    ADD_STAT(overallMissLatency, statistics::units::Tick::get(),
             "number of overall miss ticks"),
    ADD_STAT(demandAccesses, statistics::units::Count::get(),
             "number of demand (read+write) accesses"),
    ADD_STAT(overallAccesses, statistics::units::Count::get(),
             "number of overall (read+write) accesses"),
    ADD_STAT(demandMissRate, statistics::units::Ratio::get(),
             "miss rate for demand accesses"),
    ADD_STAT(overallMissRate, statistics::units::Ratio::get(),
             "miss rate for overall accesses"),
    ADD_STAT(demandAvgMissLatency, statistics::units::Rate<
                statistics::units::Cycle, statistics::units::Count>::get(),
             "average overall miss latency"),
    ADD_STAT(overallAvgMissLatency, statistics::units::Rate<
                statistics::units::Cycle, statistics::units::Count>::get(),
             "average overall miss latency"),
    ADD_STAT(blockedCycles, statistics::units::Cycle::get(),
            "number of cycles access was blocked"),
    ADD_STAT(blockedCauses, statistics::units::Count::get(),
            "number of times access was blocked"),
    ADD_STAT(avgBlocked, statistics::units::Rate<
                statistics::units::Cycle, statistics::units::Count>::get(),
             "average number of cycles each access was blocked"),
    ADD_STAT(writebacks, statistics::units::Count::get(),
             "number of writebacks"),
    ADD_STAT(demandMshrHits, statistics::units::Count::get(),
             "number of demand (read+write) MSHR hits"),
    ADD_STAT(overallMshrHits, statistics::units::Count::get(),
             "number of overall MSHR hits"),
    ADD_STAT(demandMshrMisses, statistics::units::Count::get(),
             "number of demand (read+write) MSHR misses"),
    ADD_STAT(overallMshrMisses, statistics::units::Count::get(),
            "number of overall MSHR misses"),
    ADD_STAT(overallMshrUncacheable, statistics::units::Count::get(),
             "number of overall MSHR uncacheable misses"),
    ADD_STAT(demandMshrMissLatency, statistics::units::Tick::get(),
             "number of demand (read+write) MSHR miss ticks"),
    ADD_STAT(overallMshrMissLatency, statistics::units::Tick::get(),
             "number of overall MSHR miss ticks"),
    ADD_STAT(overallMshrUncacheableLatency, statistics::units::Tick::get(),
             "number of overall MSHR uncacheable ticks"),
    ADD_STAT(demandMshrMissRate, statistics::units::Ratio::get(),
             "mshr miss ratio for demand accesses"),
    ADD_STAT(overallMshrMissRate, statistics::units::Ratio::get(),
             "mshr miss ratio for overall accesses"),
    ADD_STAT(demandAvgMshrMissLatency, statistics::units::Rate<
                statistics::units::Cycle, statistics::units::Count>::get(),
             "average overall mshr miss latency"),
    ADD_STAT(overallAvgMshrMissLatency, statistics::units::Rate<
                statistics::units::Cycle, statistics::units::Count>::get(),
             "average overall mshr miss latency"),
    ADD_STAT(overallAvgMshrUncacheableLatency, statistics::units::Rate<
                statistics::units::Cycle, statistics::units::Count>::get(),
             "average overall mshr uncacheable latency"),
    ADD_STAT(replacements, statistics::units::Count::get(),
             "number of replacements"),
    ADD_STAT(dataExpansions, statistics::units::Count::get(),
             "number of data expansions"),
    ADD_STAT(dataContractions, statistics::units::Count::get(),
             "number of data contractions"),
    cmd(MemCmd::NUM_MEM_CMDS)
{
    for (int idx = 0; idx < MemCmd::NUM_MEM_CMDS; ++idx)
        cmd[idx].reset(new CacheCmdStats(c, MemCmd(idx).toString()));
}

void
BaseCache::CacheStats::regStats()
{
    using namespace statistics;

    statistics::Group::regStats();

    System *system = cache.system;
    const auto max_requestors = system->maxRequestors();

    for (auto &cs : cmd)
        cs->regStatsFromParent();

// These macros make it easier to sum the right subset of commands and
// to change the subset of commands that are considered "demand" vs
// "non-demand"
#define SUM_DEMAND(s)                                                   \
    (cmd[MemCmd::ReadReq]->s + cmd[MemCmd::WriteReq]->s +               \
     cmd[MemCmd::WriteLineReq]->s + cmd[MemCmd::ReadExReq]->s +         \
     cmd[MemCmd::ReadCleanReq]->s + cmd[MemCmd::ReadSharedReq]->s)

// should writebacks be included here?  prior code was inconsistent...
#define SUM_NON_DEMAND(s)                                       \
    (cmd[MemCmd::SoftPFReq]->s + cmd[MemCmd::HardPFReq]->s +    \
     cmd[MemCmd::SoftPFExReq]->s)

    demandHits.flags(total | nozero | nonan);
    demandHits = SUM_DEMAND(hits);
    for (int i = 0; i < max_requestors; i++) {
        demandHits.subname(i, system->getRequestorName(i));
    }

    overallHits.flags(total | nozero | nonan);
    overallHits = demandHits + SUM_NON_DEMAND(hits);
    for (int i = 0; i < max_requestors; i++) {
        overallHits.subname(i, system->getRequestorName(i));
    }

    demandMisses.flags(total | nozero | nonan);
    demandMisses = SUM_DEMAND(misses);
    for (int i = 0; i < max_requestors; i++) {
        demandMisses.subname(i, system->getRequestorName(i));
    }

    overallMisses.flags(total | nozero | nonan);
    overallMisses = demandMisses + SUM_NON_DEMAND(misses);
    for (int i = 0; i < max_requestors; i++) {
        overallMisses.subname(i, system->getRequestorName(i));
    }

    demandMissLatency.flags(total | nozero | nonan);
    demandMissLatency = SUM_DEMAND(missLatency);
    for (int i = 0; i < max_requestors; i++) {
        demandMissLatency.subname(i, system->getRequestorName(i));
    }

    overallMissLatency.flags(total | nozero | nonan);
    overallMissLatency = demandMissLatency + SUM_NON_DEMAND(missLatency);
    for (int i = 0; i < max_requestors; i++) {
        overallMissLatency.subname(i, system->getRequestorName(i));
    }

    demandAccesses.flags(total | nozero | nonan);
    demandAccesses = demandHits + demandMisses;
    for (int i = 0; i < max_requestors; i++) {
        demandAccesses.subname(i, system->getRequestorName(i));
    }

    overallAccesses.flags(total | nozero | nonan);
    overallAccesses = overallHits + overallMisses;
    for (int i = 0; i < max_requestors; i++) {
        overallAccesses.subname(i, system->getRequestorName(i));
    }

    demandMissRate.flags(total | nozero | nonan);
    demandMissRate = demandMisses / demandAccesses;
    for (int i = 0; i < max_requestors; i++) {
        demandMissRate.subname(i, system->getRequestorName(i));
    }

    overallMissRate.flags(total | nozero | nonan);
    overallMissRate = overallMisses / overallAccesses;
    for (int i = 0; i < max_requestors; i++) {
        overallMissRate.subname(i, system->getRequestorName(i));
    }

    demandAvgMissLatency.flags(total | nozero | nonan);
    demandAvgMissLatency = demandMissLatency / demandMisses;
    for (int i = 0; i < max_requestors; i++) {
        demandAvgMissLatency.subname(i, system->getRequestorName(i));
    }

    overallAvgMissLatency.flags(total | nozero | nonan);
    overallAvgMissLatency = overallMissLatency / overallMisses;
    for (int i = 0; i < max_requestors; i++) {
        overallAvgMissLatency.subname(i, system->getRequestorName(i));
    }

    blockedCycles.init(NUM_BLOCKED_CAUSES);
    blockedCycles
        .subname(Blocked_NoMSHRs, "no_mshrs")
        .subname(Blocked_NoTargets, "no_targets")
        ;


    blockedCauses.init(NUM_BLOCKED_CAUSES);
    blockedCauses
        .subname(Blocked_NoMSHRs, "no_mshrs")
        .subname(Blocked_NoTargets, "no_targets")
        ;

    avgBlocked
        .subname(Blocked_NoMSHRs, "no_mshrs")
        .subname(Blocked_NoTargets, "no_targets")
        ;
    avgBlocked = blockedCycles / blockedCauses;

    writebacks
        .init(max_requestors)
        .flags(total | nozero | nonan)
        ;
    for (int i = 0; i < max_requestors; i++) {
        writebacks.subname(i, system->getRequestorName(i));
    }

    demandMshrHits.flags(total | nozero | nonan);
    demandMshrHits = SUM_DEMAND(mshrHits);
    for (int i = 0; i < max_requestors; i++) {
        demandMshrHits.subname(i, system->getRequestorName(i));
    }

    overallMshrHits.flags(total | nozero | nonan);
    overallMshrHits = demandMshrHits + SUM_NON_DEMAND(mshrHits);
    for (int i = 0; i < max_requestors; i++) {
        overallMshrHits.subname(i, system->getRequestorName(i));
    }

    demandMshrMisses.flags(total | nozero | nonan);
    demandMshrMisses = SUM_DEMAND(mshrMisses);
    for (int i = 0; i < max_requestors; i++) {
        demandMshrMisses.subname(i, system->getRequestorName(i));
    }

    overallMshrMisses.flags(total | nozero | nonan);
    overallMshrMisses = demandMshrMisses + SUM_NON_DEMAND(mshrMisses);
    for (int i = 0; i < max_requestors; i++) {
        overallMshrMisses.subname(i, system->getRequestorName(i));
    }

    demandMshrMissLatency.flags(total | nozero | nonan);
    demandMshrMissLatency = SUM_DEMAND(mshrMissLatency);
    for (int i = 0; i < max_requestors; i++) {
        demandMshrMissLatency.subname(i, system->getRequestorName(i));
    }

    overallMshrMissLatency.flags(total | nozero | nonan);
    overallMshrMissLatency =
        demandMshrMissLatency + SUM_NON_DEMAND(mshrMissLatency);
    for (int i = 0; i < max_requestors; i++) {
        overallMshrMissLatency.subname(i, system->getRequestorName(i));
    }

    overallMshrUncacheable.flags(total | nozero | nonan);
    overallMshrUncacheable =
        SUM_DEMAND(mshrUncacheable) + SUM_NON_DEMAND(mshrUncacheable);
    for (int i = 0; i < max_requestors; i++) {
        overallMshrUncacheable.subname(i, system->getRequestorName(i));
    }


    overallMshrUncacheableLatency.flags(total | nozero | nonan);
    overallMshrUncacheableLatency =
        SUM_DEMAND(mshrUncacheableLatency) +
        SUM_NON_DEMAND(mshrUncacheableLatency);
    for (int i = 0; i < max_requestors; i++) {
        overallMshrUncacheableLatency.subname(i, system->getRequestorName(i));
    }

    demandMshrMissRate.flags(total | nozero | nonan);
    demandMshrMissRate = demandMshrMisses / demandAccesses;
    for (int i = 0; i < max_requestors; i++) {
        demandMshrMissRate.subname(i, system->getRequestorName(i));
    }

    overallMshrMissRate.flags(total | nozero | nonan);
    overallMshrMissRate = overallMshrMisses / overallAccesses;
    for (int i = 0; i < max_requestors; i++) {
        overallMshrMissRate.subname(i, system->getRequestorName(i));
    }

    demandAvgMshrMissLatency.flags(total | nozero | nonan);
    demandAvgMshrMissLatency = demandMshrMissLatency / demandMshrMisses;
    for (int i = 0; i < max_requestors; i++) {
        demandAvgMshrMissLatency.subname(i, system->getRequestorName(i));
    }

    overallAvgMshrMissLatency.flags(total | nozero | nonan);
    overallAvgMshrMissLatency = overallMshrMissLatency / overallMshrMisses;
    for (int i = 0; i < max_requestors; i++) {
        overallAvgMshrMissLatency.subname(i, system->getRequestorName(i));
    }

    overallAvgMshrUncacheableLatency.flags(total | nozero | nonan);
    overallAvgMshrUncacheableLatency =
        overallMshrUncacheableLatency / overallMshrUncacheable;
    for (int i = 0; i < max_requestors; i++) {
        overallAvgMshrUncacheableLatency.subname(i,
            system->getRequestorName(i));
    }

    dataExpansions.flags(nozero | nonan);
    dataContractions.flags(nozero | nonan);
}

void
BaseCache::regProbePoints()
{
    ppHit = new ProbePointArg<PacketPtr>(this->getProbeManager(), "Hit");
    ppMiss = new ProbePointArg<PacketPtr>(this->getProbeManager(), "Miss");
    ppFill = new ProbePointArg<PacketPtr>(this->getProbeManager(), "Fill");
    ppDdioHint = new ProbePointArg<PacketPtr>(this->getProbeManager(), "DdioHint"); // SHIN
    ppDataUpdate =
        new ProbePointArg<DataUpdate>(this->getProbeManager(), "Data Update");
}

///////////////
//
// CpuSidePort
//
///////////////
bool
BaseCache::CpuSidePort::recvTimingSnoopResp(PacketPtr pkt)
{
    // Snoops shouldn't happen when bypassing caches
    assert(!cache->system->bypassCaches());

    assert(pkt->isResponse());

    // Express snoop responses from requestor to responder, e.g., from L1 to L2
    cache->recvTimingSnoopResp(pkt);
    return true;
}


bool
BaseCache::CpuSidePort::tryTiming(PacketPtr pkt)
{
    if (cache->system->bypassCaches() || pkt->isExpressSnoop()) {
        // always let express snoop packets through even if blocked
        return true;
    } else if (blocked || mustSendRetry) {
        // either already committed to send a retry, or blocked
        mustSendRetry = true;
        return false;
    }
    mustSendRetry = false;
    return true;
}

bool
BaseCache::CpuSidePort::recvTimingReq(PacketPtr pkt)
{
    assert(pkt->isRequest());

    if (cache->system->bypassCaches()) {
        // Just forward the packet if caches are disabled.
        // @todo This should really enqueue the packet rather
        GEM5_VAR_USED bool success = cache->memSidePort.sendTimingReq(pkt);
        assert(success);
        return true;
    } else if (tryTiming(pkt)) {
        if (cache->isLLC && cache->isMultiPort) {
            assert(id != InvalidPortID);
            if (pkt->cpu_side_port_id == InvalidPortID) {
                pkt->cpu_side_port_id = id;
            } else {
                assert(pkt->cpu_side_port_id == id);
            }
            cache->recvTimingReq(pkt, id);
        } else {
            cache->recvTimingReq(pkt);
        }
        return true;
    }
    return false;
}

Tick
BaseCache::CpuSidePort::recvAtomic(PacketPtr pkt)
{
    if (cache->system->bypassCaches()) {
        // Forward the request if the system is in cache bypass mode.
        return cache->memSidePort.sendAtomic(pkt);
    } else {
        return cache->recvAtomic(pkt);
    }
}

void
BaseCache::CpuSidePort::recvFunctional(PacketPtr pkt)
{
    if (cache->system->bypassCaches()) {
        // The cache should be flushed if we are in cache bypass mode,
        // so we don't need to check if we need to update anything.
        cache->memSidePort.sendFunctional(pkt);
        return;
    }

    if (cache->isLLC && cache->isMultiPort) {
        // JM - set cpu_port_id in pkt
        assert(id != InvalidPortID);
        if (pkt->cpu_side_port_id == InvalidPortID) {
            pkt->cpu_side_port_id = id;
        } else {
            assert(pkt->cpu_side_port_id == id);
        }
    }

    // functional request
    cache->functionalAccess(pkt, true);
}

AddrRangeList
BaseCache::CpuSidePort::getAddrRanges() const
{   
    // JM - multiport cache
    if (cache->isLLC && cache->isMultiPort) {
        assert(id != InvalidPortID);
        return cache->getAddrRanges(id);
    } else {
        // JM - single port cache. InvalidPortID is meaningless
        return cache->getAddrRanges(InvalidPortID);
    }
}


BaseCache::
CpuSidePort::CpuSidePort(const std::string &_name, BaseCache *_cache,
                         const std::string &_label, PortID _id)
    : CacheResponsePort(_name, _cache, _label), cache(_cache), id(_id)
{
}

///////////////
//
// MemSidePort
//
///////////////
bool
BaseCache::MemSidePort::recvTimingResp(PacketPtr pkt)
{
    cache->recvTimingResp(pkt);
    return true;
}

// Express snooping requests to memside port
void
BaseCache::MemSidePort::recvTimingSnoopReq(PacketPtr pkt)
{
    // Snoops shouldn't happen when bypassing caches
    assert(!cache->system->bypassCaches());

    // handle snooping requests
    cache->recvTimingSnoopReq(pkt);
}

Tick
BaseCache::MemSidePort::recvAtomicSnoop(PacketPtr pkt)
{
    // Snoops shouldn't happen when bypassing caches
    assert(!cache->system->bypassCaches());

    return cache->recvAtomicSnoop(pkt);
}

void
BaseCache::MemSidePort::recvFunctionalSnoop(PacketPtr pkt)
{
    // Snoops shouldn't happen when bypassing caches
    assert(!cache->system->bypassCaches());

    // functional snoop (note that in contrast to atomic we don't have
    // a specific functionalSnoop method, as they have the same
    // behaviour regardless)
    cache->functionalAccess(pkt, false);
}

void
BaseCache::CacheReqPacketQueue::sendDeferredPacket()
{
    // sanity check
    assert(!waitingOnRetry);

    // there should never be any deferred request packets in the
    // queue, instead we resly on the cache to provide the packets
    // from the MSHR queue or write queue
    assert(deferredPacketReadyTime() == MaxTick);

    // check for request packets (requests & writebacks)
    QueueEntry* entry = cache.getNextQueueEntry();

    if (!entry) {
        // can happen if e.g. we attempt a writeback and fail, but
        // before the retry, the writeback is eliminated because
        // we snoop another cache's ReadEx.
    } else {
        // let our snoop responses go first if there are responses to
        // the same addresses
        if (checkConflictingSnoop(entry->getTarget()->pkt)) {
            return;
        }
        waitingOnRetry = entry->sendPacket(cache);
    }

    // if we succeeded and are not waiting for a retry, schedule the
    // next send considering when the next queue is ready, note that
    // snoop responses have their own packet queue and thus schedule
    // their own events
    if (!waitingOnRetry) {
        schedSendEvent(cache.nextQueueReadyTime());
    }
}

BaseCache::MemSidePort::MemSidePort(const std::string &_name,
                                    BaseCache *_cache,
                                    const std::string &_label)
    : CacheRequestPort(_name, _cache, _reqQueue, _snoopRespQueue),
      _reqQueue(*_cache, *this, _snoopRespQueue, _label),
      _snoopRespQueue(*_cache, *this, true, _label), cache(_cache)
{
}

void
WriteAllocator::updateMode(Addr write_addr, unsigned write_size,
                           Addr blk_addr)
{
    // check if we are continuing where the last write ended
    if (nextAddr == write_addr) {
        delayCtr[blk_addr] = delayThreshold;
        // stop if we have already saturated
        if (mode != WriteMode::NO_ALLOCATE) {
            byteCount += write_size;
            // switch to streaming mode if we have passed the lower
            // threshold
            if (mode == WriteMode::ALLOCATE &&
                byteCount > coalesceLimit) {
                mode = WriteMode::COALESCE;
                DPRINTF(Cache, "Switched to write coalescing\n");
            } else if (mode == WriteMode::COALESCE &&
                       byteCount > noAllocateLimit) {
                // and continue and switch to non-allocating mode if we
                // pass the upper threshold
                mode = WriteMode::NO_ALLOCATE;
                DPRINTF(Cache, "Switched to write-no-allocate\n");
            }
        }
    } else {
        // we did not see a write matching the previous one, start
        // over again
        byteCount = write_size;
        mode = WriteMode::ALLOCATE;
        resetDelay(blk_addr);
    }
    nextAddr = write_addr + write_size;
}

///////////////
//
// IOSidePort
//
///////////////
bool
BaseCache::IoSidePort::recvTimingResp(PacketPtr pkt)
{
    // Call the DTA function to handle the response from NIC
    assert((cache->dta) != nullptr);
    cache->dta->recvTimingRespfromNIC(pkt);
    return true;
}

void
BaseCache::IoSidePort::recvReqRetry()
{
    // call the dta's sendTimingReqToNIC function to retry sending the request to NIC
    assert((cache->dta) != nullptr);
    assert(waitingOnRetry);
    waitingOnRetry = false;
    cache->dta->sendRequestToNIC();
}


///////////////
//
// JobPort
//
///////////////
void 
BaseCache::JobPort::recvFunctional(PacketPtr pkt)
{
    recvAtomic(pkt); // Just throw away the latency returned from recvAtomic
}

bool 
BaseCache::JobPort::recvTimingReq(PacketPtr pkt)
{
    assert(pkt->isRequest());
    assert(pkt->isWrite());
    // Job submission from CPU will be write request
    if (pkt->isRXJobReq() || pkt->isTXJobReq()) {
        assert((cache->dta) != nullptr);
        return cache->dta->recvTimingReq(pkt);
    } else {
        panic("Invalid packet comes to DTA Job Port!\n");
    }
}

gem5::Tick 
BaseCache::JobPort::recvAtomic(PacketPtr pkt)
{
    // Because pkt will be write request, we will return immediately
    // We will ignore the job request from CPU in atomic mode
    assert(pkt->isRequest());
    assert(pkt->isWrite());
    pkt->makeAtomicResponse();

    gem5::Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
    pkt->headerDelay = pkt->payloadDelay = 0;
    const gem5::Tick delay = receive_delay + gem5::Tick(1);      // Just assume 1 cycle is needed to send the response

    return delay;
}


///////////////
//           //
//    DTA    //
//           //
///////////////

bool 
DTA::recvTimingReq(PacketPtr pkt)
{
    // Receive job request from CPU
    if (!isDTAEnabled()) {
        return false;
    }   
    if (pkt->isRXJobReq()) {
        if (rxJobSubmissionQueue.size() >= submissionQueueMaxSize) {
            return false;
        } else {
            // Have to check the second uint64_t value of the packet is less than the 1024 if the rxContext is not valid
            // Because, checkpoint can be made during sending the job packet from the DPDK.
            // Have to begin with the new job request if the rxContext is not valid
            bool isValidJob = true; // If there is no initialization stage RXContext, we don't need to check the second uint64_t value
            if (getInitializationStageRXContextID() == -1) {
                // If there are no intialization stage RXContext, we have to check the second uint64_t value to filter-out the mid-job packet
                // Check the packet's second uint64_t value
                uint8_t *data = pkt->getPtr<uint8_t>();
                // Parse the job request from CPU
                data = reinterpret_cast<uint8_t*>(data) + sizeof(Addr);
                uint64_t nb_pkts_64bit = *(uint64_t*)data;
                if (nb_pkts_64bit > 1024) {
                    isValidJob = false;
                    invalidRXJobRequestCount += 1;
                    // printf("The second uint64_t value of the RX job request is greater than 1024. It is not valid. So we will discard. The number of invalid RX job request is %d\n", invalidRXJobRequestCount);
                } else {
                    isValidJob = true;
                    validRXJobRequestCount += 1;
                    // printf("[LOG], %llu, DTA_RX, NEW_BATCH_JOB_REQ_RECV\n", curTick());
                    // printf("The second uint64_t value of the RX job request is less than 1024. It is valid. So we will start the new job. The number of valid RX job request is %d\n", validRXJobRequestCount);
                }
            }

            if (isValidJob) {
                // Only do this when the received packet is the valid job
                // printf("Receive the RX job request from CPU\n");
                #if LOG_LEVEL == 4 || LOG_LEVEL == 3
                    printf("[LOG], %llu, DTA_RX_JOB_REQ_RECV\n", curTick());
                #endif
                // Make copy of the pkt and push the copy to the submission queue
                // The original pkt will be responded to CPU
                PacketPtr pkt_copy = new Packet(pkt, false, true);
                assert(pkt_copy->isRXJobReq());
                rxJobSubmissionQueue.push_back(pkt_copy);
                // Restart the clock if it is not running
                rxTick = true;
                if (!tickEvent.scheduled()) {
                    restartClock();
                }
                DPRINTF(DDIO, "Received RX job request from CPU\n");
            } else {
                // printf("The RX job request is invalid. So we will discard the job request\n");
            }

            // Make response packet and send it to CPU
            pkt->makeResponse();
            Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
            pkt->headerDelay = pkt->payloadDelay = 0;
            const Tick delay = receive_delay + clockPeriod() * 1;      // Just assume 1 cycle is needed to send the response

            assert(ioCache != nullptr);
            ioCache->sendTimingResptoHost(pkt, curTick() + delay);
            DPRINTF(DDIO, "Responded RX job request to CPU\n");

            return true;
        }
    } else if (pkt->isTXJobReq()) {
        if (txJobSubmissionQueue.size() >= submissionQueueMaxSize) {
            return false;
        } else {
            // printf("Receive the TX job request from CPU\n");
            #if LOG_LEVEL == 4 || LOG_LEVEL == 3
                printf("[LOG], %llu, DTA_TX_JOB_REQ_RECV\n", curTick());
            #endif
            // Check if this is new batch job
            if (getInitializationStageTXContextID() == -1) {
                // This is new batch job
                // printf("[LOG], %llu, DTA_TX, NEW_BATCH_JOB_REQ_RECV\n", curTick());
            }
            // Make copy of the pkt and push the copy to the submission queue
            // The original pkt will be responded to CPU
            PacketPtr pkt_copy = new Packet(pkt, false, true);
            assert(pkt_copy->isTXJobReq());
            txJobSubmissionQueue.push_back(pkt_copy);
            // Restart the clock if it is not running
            txTick = true;
            if (!tickEvent.scheduled()) {
                restartClock();
            }
            DPRINTF(DDIO, "Received TX job request from CPU\n");

            // Make response packet and send it to CPU
            pkt->makeResponse();
            Tick receive_delay = pkt->headerDelay + pkt->payloadDelay;
            pkt->headerDelay = pkt->payloadDelay = 0;
            const Tick delay = receive_delay + clockPeriod() * 1;      // Just assume 1 cycle is needed to send the response

            assert(ioCache != nullptr);
            ioCache->sendTimingResptoHost(pkt, curTick() + delay);
            DPRINTF(DDIO, "Responded TX job request to CPU\n");

            return true;
        }
    } else {
        panic("Invalid packet comes to DTA!\n");
    }
}

bool 
DTA::checkSubmissionQueue() {
    // Check the submission queue and set the context if the context is not valid
    assert(isDTAEnabled());
    // If checkRXJobQueue is true, first check RXJobSubmissionQueue. If it is empty, check TXJobSubmissionQueue
    // If checkRXJobQueue is false, check TXJobSubmissionQueue. If it is empty, check RXJobSubmissionQueue
    bool scheduleRXJob = false;
    bool scheduleTXJob = false;

    bool canScheduleRXJob = (!rxJobSubmissionQueue.empty()) && 
                            (hasNonActiveRXContext() || (getInitializationStageRXContextID() != -1));
    bool canScheduleTXJob = (!txJobSubmissionQueue.empty()) && 
                            (hasNonActiveTXContext() || (getInitializationStageTXContextID() != -1));
    
    // Decide which job to schedule
    if (checkRXJobQueue) {
        // Priority to RX job
        if (canScheduleRXJob) {
            scheduleRXJob = true;
        } else if (canScheduleTXJob) {
            scheduleTXJob = true;
        }
    } else {
        // Priority to TX job
        if (canScheduleTXJob) {
            scheduleTXJob = true;
        } else if (canScheduleRXJob) {
            scheduleRXJob = true;
        }
    }

    // Set Next turn (RX or TX) - checkRXJobQueue
    if (scheduleRXJob) {
        // Current turn is RX. Next turn is TX
        checkRXJobQueue = false;
    } else if (scheduleTXJob) {
        // Current turn is TX. Next turn is RX
        checkRXJobQueue = true;
    } else {
        // Both queues cannot be scheduled (empty or context is already used)
        return false;
    }

    if (scheduleRXJob) {
        assert(!rxJobSubmissionQueue.empty());
        PacketPtr pkt = rxJobSubmissionQueue.front();
        assert(pkt->isRXJobReq());
        assert(hasNonActiveRXContext() || (getInitializationStageRXContextID() != -1));
        bool successParsing = parseJobRequestAndSetContext(pkt);
        if (successParsing) {
            rxJobSubmissionQueue.pop_front();
            delete pkt; // This is the copy of the original packet
            return true; // Need tick
        } else {
            return true; // Need tick
        }
    } else if (scheduleTXJob) {
        assert(!txJobSubmissionQueue.empty());
        PacketPtr pkt = txJobSubmissionQueue.front();
        assert(pkt->isTXJobReq());
        assert(hasNonActiveTXContext() || (getInitializationStageTXContextID() != -1));
        bool successParsing = parseJobRequestAndSetContext(pkt);
        if (successParsing) {
            txJobSubmissionQueue.pop_front();
            delete pkt; // This is the copy of the original packet
            return true; // Need tick
        } else {
            return true; // Need tick
        }
    } else {
        assert(0 && "Cannot reach here");
        return false; // No need tick
    }
}

bool 
DTA::parseJobRequestAndSetContext(PacketPtr pkt) 
{
    // Parse Job request from CPU and make a new context
    assert(isDTAEnabled());
    if (pkt->isRXJobReq()) {
        int rx_context_id = getParsingTargetRXContextID();
        if (rx_context_id == -1) {
            printf("DTA::parseJobRequestAndSetContext - RX Context ID is -1. Not enough RXContextList\n");
            return false;
        } else {
            if (!isActiveRXContext(rx_context_id)) {
                pushRXContextIDQueue((uint64_t)rx_context_id);
                DPRINTF(DDIO, "Parsing RX job request 0th packet from CPU\n");
                global_rx_job_id += 1;
                dtaRXContextList[rx_context_id].rx_job_id = global_rx_job_id;
                // if (global_rx_job_id <= 13) {
                //     printf("%llu, DTA Parsing RX Job ID: %llu\n", curTick(), global_rx_job_id);
                // }
                // printf("[LOG], %llu, DTA_RX, %lu, RX_JOB_REQ_PARSE\n", curTick(), global_rx_job_id);
                #if LOG_LEVEL == 4 || LOG_LEVEL == 3
                    printf("[LOG], %llu, DTA_RX, %lu, RX_JOB_REQ_PARSE\n", curTick(), global_rx_job_id);
                #endif
                dtaRXContextList[rx_context_id].valid = true;
                dtaRXContextList[rx_context_id].completion_stage = false;
                uint32_t n_job_packet_received = 0;
                uint8_t *data = pkt->getPtr<uint8_t>();
                // Parse the job request from CPU
                dtaRXContextList[rx_context_id].completion_addr =  *(Addr*)data;
                dtaRXContextList[rx_context_id].desc_addr = dtaRXContextList[rx_context_id].completion_addr + cacheLineSize;
                data = reinterpret_cast<uint8_t*>(data) + sizeof(Addr);
                uint64_t nb_pkts_64bit = *(uint64_t*)data;
                dtaRXContextList[rx_context_id].nb_pkts = (uint32_t)nb_pkts_64bit;
                data = reinterpret_cast<uint8_t*>(data) + sizeof(uint64_t);
                dtaRXContextList[rx_context_id].n_recv = 0;
                dtaRXContextList[rx_context_id].num_free_request_in_NIC = 0;
                dtaRXContextList[rx_context_id].lastDTARXWorker = 0;
                dtaRXContextList[rx_context_id].n_mbuf_addr_received = 0;
                dtaRXContextList[rx_context_id].n_extra_cxl_req_needed = 0;
                dtaRXContextList[rx_context_id].n_extra_cxl_req_received = 0;
                dtaRXContextList[rx_context_id].n_desc_write_completed = 0;
                dtaRXContextList[rx_context_id].n_cacheline_idx_write_completed = 0;
                dtaRXContextList[rx_context_id].RXDescMap.clear();
                dtaRXContextList[rx_context_id].RXCompleteMap.clear();
                
                n_job_packet_received = std::min(dtaRXContextList[rx_context_id].nb_pkts, rxFirstJobPacketNumMbufAddr);
                for (uint32_t i = 0; i < n_job_packet_received; i++) {
                    Addr temp_mbuf_addr = *(Addr*)data;
                    dtaRXContextList[rx_context_id].mbuf_addr[i] = temp_mbuf_addr;
                    dtaRXContextList[rx_context_id].RXCompleteMap[temp_mbuf_addr] = false;
                    data += sizeof(Addr);
                    dtaRXContextList[rx_context_id].n_mbuf_addr_received += 1;
                }            
                DPRINTF(DDIO, "Parsed RX job request from CPU\n");
                DPRINTF(DDIO, "dtaRXContextList[%d].completion_addr: %lx\n", rx_context_id, dtaRXContextList[rx_context_id].completion_addr);
                DPRINTF(DDIO, "dtaRXContextList[%d].desc_addr: %lx\n", rx_context_id, dtaRXContextList[rx_context_id].desc_addr);
                DPRINTF(DDIO, "dtaRXContextList[%d].nb_pkts: %d\n", rx_context_id, dtaRXContextList[rx_context_id].nb_pkts);
                DPRINTF(DDIO, "dtaRXContextList[%d].n_mbuf_addr_received: %d\n", rx_context_id, dtaRXContextList[rx_context_id].n_mbuf_addr_received);
                for (uint32_t i = 0; i < dtaRXContextList[rx_context_id].n_mbuf_addr_received; i++) {
                    DPRINTF(DDIO, "dtaRXContextList[%d].mbuf_addr[%d]: %lx\n", rx_context_id, i, dtaRXContextList[rx_context_id].mbuf_addr[i]);
                }

                #if LOG_LEVEL == 4
                if (global_rx_job_id <= 13) { //global_rx_job_id >= 11 && 
                    // Print the job info
                    printf("DTA RX Job ID: %llu\n", dtaRXContextList[rx_context_id].rx_job_id);
                    printf("DTA RX Completion Addr: %#lx\n", dtaRXContextList[rx_context_id].completion_addr);
                    printf("DTA RX Descriptor Addr: %#lx\n", dtaRXContextList[rx_context_id].desc_addr);
                    printf("DTA RX Number of Packets: %d\n", dtaRXContextList[rx_context_id].nb_pkts);
                    printf("DTA RX mbuf addresses:\n");
                    for (uint32_t i = 0; i < dtaRXContextList[rx_context_id].n_mbuf_addr_received; i++) {
                        printf("%#lx, ", dtaRXContextList[rx_context_id].mbuf_addr[i]);
                    }
                    printf("\n");
                }
                #endif

                // Intialize the completion address with 0
                // Make the Request
                RequestPtr req = std::make_shared<Request>(dtaRXContextList[rx_context_id].completion_addr, cacheLineSize, 0, requestorId);
                req->taskId(context_switch_task_id::DMA);
                PacketPtr wr_pkt = new Packet(req, MemCmd::WriteReq);
                // Make the data
                uint8_t *wr_data = new uint8_t[cacheLineSize];
                memset(wr_data, 0, cacheLineSize);
                wr_pkt->allocate();
                wr_pkt->setData(wr_data);
                wr_pkt->setFromDTA();

                delete[] wr_data;

                assert(ioCache != nullptr);
                pushIOCacheRequestQueue(wr_pkt);

            } else {
                // This job request packet is the next part of the previous job request packet
                // This packet only contains the mbuf address
                // Have to use the nb_pkts that is received from the first packet
                uint32_t n_job_packet_received = 0;
                uint32_t packet_index = 0;
                // Calculate the index of the job request packet
                // Can calculate the index of the job request packet using the number of packets that are received (n_mbuf_addr_received)
                packet_index = 1 + int((dtaRXContextList[rx_context_id].n_mbuf_addr_received - rxFirstJobPacketNumMbufAddr) / restJobPacketNumMbufAddr);
                DPRINTF(DDIO, "Parsing RX job request %dth packet from CPU\n", packet_index);
                
                uint8_t *data = pkt->getPtr<uint8_t>();
                // Parse the job request from CPU
                #if LOG_LEVEL == 4
                if (global_rx_job_id <= 13) { //global_rx_job_id >= 11 && 
                    // Print the job info
                    printf("DTA RX Job ID: %llu\n", dtaRXContextList[rx_context_id].rx_job_id);
                    printf("DTA RX mbuf addresses:\n");
                }
                #endif
                n_job_packet_received = std::min(restJobPacketNumMbufAddr, dtaRXContextList[rx_context_id].nb_pkts - dtaRXContextList[rx_context_id].n_mbuf_addr_received);
                for (uint32_t i = 0; i < n_job_packet_received; i++) {
                    Addr temp_mbuf_addr = *(Addr*)data;
                    dtaRXContextList[rx_context_id].mbuf_addr[dtaRXContextList[rx_context_id].n_mbuf_addr_received] = temp_mbuf_addr;
                    dtaRXContextList[rx_context_id].RXCompleteMap[temp_mbuf_addr] = false;
                    data += sizeof(Addr);
                    dtaRXContextList[rx_context_id].n_mbuf_addr_received += 1;
                    #if LOG_LEVEL == 4
                    if (global_rx_job_id <= 13) //global_rx_job_id >= 11 && 
                        printf("%#lx, ", temp_mbuf_addr);
                    #endif
                }
                #if LOG_LEVEL == 4
                if (global_rx_job_id <= 13) //global_rx_job_id >= 11 && 
                    printf("\n");
                #endif
                assert(dtaRXContextList[rx_context_id].n_mbuf_addr_received <= dtaRXContextList[rx_context_id].nb_pkts);
                if (dtaRXContextList[rx_context_id].n_mbuf_addr_received == dtaRXContextList[rx_context_id].nb_pkts) {
                    // Write the temporal completion id to the completion address. To stop the break of while loop in the DPDK
                    // Make the Request
                    RequestPtr req = std::make_shared<Request>(dtaRXContextList[rx_context_id].completion_addr, cacheLineSize, 0, requestorId);
                    req->taskId(context_switch_task_id::DMA);
                    PacketPtr wr_pkt = new Packet(req, MemCmd::WriteReq);
                    // Make the data
                    uint8_t *wr_data = new uint8_t[cacheLineSize];
                    memset(wr_data, 0, cacheLineSize);
                    int64_t completion_id = dtaRXContextList[rx_context_id].n_recv;
                    if (completion_id == 0) {
                        // Just set as 1, to notify DPDK to not exit the loop
                        completion_id = 1;
                    }
                    memcpy(wr_data, &completion_id, sizeof(int64_t));
                    wr_pkt->allocate();
                    wr_pkt->setData(wr_data);
                    wr_pkt->setFromDTA();

                    delete[] wr_data;

                    assert(ioCache != nullptr);
                    // printf("dtaRX Write the current n_recv: %ld to the completion address: %lx\n", completion_id, dtaRXContextList[rx_context_id].completion_addr);

                    pushIOCacheRequestQueue(wr_pkt);
                }
                DPRINTF(DDIO, "Parsed RX job request %dth packet from CPU, %ld/%ld mbuf addresses are received\n", packet_index, dtaRXContextList[rx_context_id].n_mbuf_addr_received, dtaRXContextList[rx_context_id].nb_pkts); 
            }
            return true;
        }
    } else if (pkt->isTXJobReq()) {
        int tx_context_id = getParsingTargetTXContextID();
        if (tx_context_id == -1) {
            printf("DTA::parseJobRequestAndSetContext - TX Context ID is -1. Not enough TXContextList\n");
            return false;
        } else {
            if (!isActiveTXContext(tx_context_id)) {
                pushTXContextIDQueue((uint64_t)tx_context_id);
                DPRINTF(DDIO, "Parsing TX job request 0th packet from CPU\n");
                global_tx_job_id += 1;
                dtaTXContextList[tx_context_id].tx_job_id = global_tx_job_id;
                // printf("[LOG], %llu, DTA_TX, %lu, TX_JOB_REQ_PARSE\n", curTick(), global_tx_job_id);
                #if LOG_LEVEL == 4 || LOG_LEVEL == 3
                    printf("[LOG], %llu, DTA_TX, %lu, TX_JOB_REQ_PARSE\n", curTick(), global_tx_job_id);
                #endif
                dtaTXContextList[tx_context_id].valid = true;
                uint32_t n_job_packet_received = 0;
                uint8_t *data = pkt->getPtr<uint8_t>();
                // Parse the job request from CPU
                // 1. Descriptor address: where the descriptors of batch are stored
                // 2. Completion address: where the completion info will be stored by DTA after the job is done
                // 3. Number of packets: number of packets in the batch
                // 4. mbuf address: the address of the mbufs in the batch (only for not zero-copy mode)
                dtaTXContextList[tx_context_id].desc_addr =  *(Addr*)data;
                data = reinterpret_cast<uint8_t*>(data) + sizeof(Addr);
                dtaTXContextList[tx_context_id].completion_addr =  *(Addr*)data;
                data = reinterpret_cast<uint8_t*>(data) + sizeof(Addr);
                uint64_t nb_pkts_64bit = *(uint64_t*)data;
                dtaTXContextList[tx_context_id].nb_pkts = (uint32_t)nb_pkts_64bit;
                data = reinterpret_cast<uint8_t*>(data) + sizeof(uint64_t);
                dtaTXContextList[tx_context_id].n_mbuf_addr_received = 0;
                dtaTXContextList[tx_context_id].n_sent = 0;
                dtaTXContextList[tx_context_id].descPayloadDMAAssigned.clear();
                dtaTXContextList[tx_context_id].descPayloadDMAWaiting.clear();
                dtaTXContextList[tx_context_id].descWaitingMbufAddr.clear();
                dtaTXContextList[tx_context_id].TXCompleteMap.clear();
                dtaTXContextList[tx_context_id].n_desc_ready = 0; 

                if (zeroCopy) {
                    assert(!tempTXMbufAddrList.empty());
                    mbufAddrList temp_mbuf_addr_list = tempTXMbufAddrList.front();
                    tempTXMbufAddrList.pop_front();
                    // Check the mbuf_addr is all valid until nb_pkts
                    for (uint32_t i = 0; i < dtaTXContextList[tx_context_id].nb_pkts; i++) {
                        if (temp_mbuf_addr_list.mbuf_addr[i] == 0) {
                            assert(0 && "The mbuf address is not valid");
                        }
                        dtaTXContextList[tx_context_id].mbuf_addr[i] = temp_mbuf_addr_list.mbuf_addr[i];
                        dtaTXContextList[tx_context_id].TXCompleteMap[temp_mbuf_addr_list.mbuf_addr[i]] = false;
                        dtaTXContextList[tx_context_id].n_mbuf_addr_received += 1;
                    }
                } else {
                    n_job_packet_received = std::min(dtaTXContextList[tx_context_id].nb_pkts, txFirstJobPacketNumMbufAddr);
                    for (uint32_t i = 0; i < n_job_packet_received; i++) {
                        Addr temp_mbuf_addr = *(Addr*)data;
                        dtaTXContextList[tx_context_id].mbuf_addr[i] = temp_mbuf_addr;
                        dtaTXContextList[tx_context_id].TXCompleteMap[temp_mbuf_addr] = false;
                        data += sizeof(Addr);
                        dtaTXContextList[tx_context_id].n_mbuf_addr_received += 1;
                    }
                    #if LOG_LEVEL == 4
                    if (global_tx_job_id <= 18) { // global_tx_job_id >= 7 && 
                        // Print the job info
                        printf("DTA TX Job ID: %llu\n", dtaTXContextList[tx_context_id].tx_job_id);
                        printf("DTA TX Descriptor Addr: %#lx\n", dtaTXContextList[tx_context_id].desc_addr);
                        printf("DTA TX Completion Addr: %#lx\n", dtaTXContextList[tx_context_id].completion_addr);
                        printf("DTA TX Number of Packets: %d\n", dtaTXContextList[tx_context_id].nb_pkts);
                        printf("DTA TX mbuf addresses:\n");
                        for (uint32_t i = 0; i < dtaTXContextList[tx_context_id].n_mbuf_addr_received; i++) {
                            printf("%#lx, ", dtaTXContextList[tx_context_id].mbuf_addr[i]);
                        }
                        printf("\n");
                    }
                    #endif
                }
                DPRINTF(DDIO, "Parsed TX job request from CPU\n");
            } else {
                // printf("DTA Subsequent Parsing of TX Job ID: %llu\n", dtaTXContextList[tx_context_id].tx_job_id);
                // If zeroCopy is true, the TX job request will only send once with descriptor address, completion address, and number of packets.
                // Zerocopy will reuse the mbuf address that is used in RX job request
                assert(!zeroCopy);
                // This job request packet is the next part of the previous job request packet
                // This packet only contains the mbuf address
                // Have to use the nb_pkts that is received from the first packet
                uint32_t n_job_packet_received = 0;
                uint32_t packet_index = 0;
                // Calculate the index of the job request packet
                // Can calculate the index of the job request packet using the number of packets that are received (n_mbuf_addr_received)
                packet_index = 1 + int((dtaTXContextList[tx_context_id].n_mbuf_addr_received - txFirstJobPacketNumMbufAddr) / restJobPacketNumMbufAddr);
                DPRINTF(DDIO, "Parsing TX job request %dth packet from CPU\n", packet_index);

                #if LOG_LEVEL == 4
                if (global_tx_job_id <= 18) { //global_tx_job_id >= 7 && global_tx_job_id <= 9
                    // Print the job info
                    printf("DTA TX Job ID: %llu\n", dtaTXContextList[tx_context_id].tx_job_id);
                    printf("DTA TX mbuf addresses:\n");
                }
                #endif
                uint8_t *data = pkt->getPtr<uint8_t>();
                // Parse the job request from CPU
                n_job_packet_received = std::min(restJobPacketNumMbufAddr, dtaTXContextList[tx_context_id].nb_pkts - dtaTXContextList[tx_context_id].n_mbuf_addr_received);
                for (uint32_t i = 0; i < n_job_packet_received; i++) {
                    Addr mbuf_addr = *(Addr*)data;
                    dtaTXContextList[tx_context_id].mbuf_addr[dtaTXContextList[tx_context_id].n_mbuf_addr_received] = mbuf_addr;
                    dtaTXContextList[tx_context_id].TXCompleteMap[mbuf_addr] = false;
                    data += sizeof(Addr);
                    // Check dtaTXContextList[tx_context_id].descWaitingMbufAddr with n_mbuf_addr_received to move the TXDesc to the dtaTXContextList[tx_context_id].descPayloadDMAWaiting
                    if (dtaTXContextList[tx_context_id].descWaitingMbufAddr.find(dtaTXContextList[tx_context_id].n_mbuf_addr_received) != dtaTXContextList[tx_context_id].descWaitingMbufAddr.end()) {
                        assert(dtaTXContextList[tx_context_id].descPayloadDMAWaiting.find(mbuf_addr) == dtaTXContextList[tx_context_id].descPayloadDMAWaiting.end());
                        TXDescriptor _desc = dtaTXContextList[tx_context_id].descWaitingMbufAddr[dtaTXContextList[tx_context_id].n_mbuf_addr_received];
                        dtaTXContextList[tx_context_id].descPayloadDMAWaiting[mbuf_addr] = _desc;
                        dtaTXContextList[tx_context_id].descWaitingMbufAddr.erase(dtaTXContextList[tx_context_id].n_mbuf_addr_received);
                    }
                    dtaTXContextList[tx_context_id].n_mbuf_addr_received += 1;
                    #if LOG_LEVEL == 4
                    if (global_tx_job_id <= 18) // global_tx_job_id >= 7 && global_tx_job_id <= 9
                        printf("%#lx, ", dtaTXContextList[tx_context_id].mbuf_addr[dtaTXContextList[tx_context_id].n_mbuf_addr_received - 1]);
                    #endif
                }
                #if LOG_LEVEL == 4
                if (global_tx_job_id <= 18) // global_tx_job_id >= 7 && global_tx_job_id <= 9
                    printf("\n");
                #endif
                assert(dtaTXContextList[tx_context_id].n_mbuf_addr_received <= dtaTXContextList[tx_context_id].nb_pkts);
                DPRINTF(DDIO, "Parsed TX job request %dth packet from CPU, %ld/%ld mbuf addresses are received\n", packet_index, dtaTXContextList[tx_context_id].n_mbuf_addr_received, dtaTXContextList[tx_context_id].nb_pkts);
            }
            return true;
        }
    }
}

// Request Sender part
bool 
DTA::sendTimingReqToNIC(PacketPtr pkt) 
{   
    // TODO
    // Send M2func RD/WR request to DTA
    assert(isDTAEnabled());
    assert(ioCache != nullptr);
    return ioCache->sendTimingReqtoNIC(pkt);
}

void
DTA::sendAtomicReqToNIC(PacketPtr pkt)
{
    // Send M2func RD/WR request to DTA
    assert(isDTAEnabled());
    assert(ioCache != nullptr);
    ioCache->sendAtomicReqtoNIC(pkt);

    recvTimingRespfromNIC(pkt);
}

bool 
DTA::checkThreshold(uint64_t rx_context_ptr)
{
    // Check the threshold for the number of requests that is sent to NIC, but the response is not received yet - If true, have to send new request
    if (dtaRXContextList[rx_context_ptr].valid) {
        // num_free_request_in_NIC should be smaller and equal to the (dtaRXContextList[rx_context_ptr].n_mbuf_addr_received - dtaRXContextList[rx_context_ptr].n_recv)
        // Also, have to consider the extra CXL requests that are needed to receive the remaining part of the ethernet packet (dtaRXContextList[rx_context_ptr].n_extra_cxl_req_needed - dtaRXContextList[rx_context_ptr].n_extra_cxl_req_received)
        // Because, if the number of requests is greater, then even though the response is coming, the mbuf address is not yet received from the host!
        assert(dtaRXContextList[rx_context_ptr].n_extra_cxl_req_needed >= dtaRXContextList[rx_context_ptr].n_extra_cxl_req_received);
        uint32_t num_extra_cxl_req_needed = dtaRXContextList[rx_context_ptr].n_extra_cxl_req_needed - dtaRXContextList[rx_context_ptr].n_extra_cxl_req_received;
        assert(dtaRXContextList[rx_context_ptr].n_mbuf_addr_received >= dtaRXContextList[rx_context_ptr].n_recv);
        uint32_t num_cxl_req_needed = dtaRXContextList[rx_context_ptr].n_mbuf_addr_received - dtaRXContextList[rx_context_ptr].n_recv;
        if (dtaRXContextList[rx_context_ptr].num_free_request_in_NIC < (num_cxl_req_needed + num_extra_cxl_req_needed)) {
            return true; // Send new request
        } else {
            return false; // Do not send new request
        }
    } else {
        return false; // Do not send new request
    }
}

void
DTA::sendRequestToNIC()
{
    // Send the request to NIC - internally call sendTimingReqToNIC
    assert(isDTAEnabled());
    assert(ioCache != nullptr);
    bool hasRXContext = hasRXContextIDQueue();
    bool hasTXContext = hasTXContextIDQueue();
    
    // Check the CXL.mem RD/WR request queue
    if (ioCache->isIOPortWaitingOnRetry()) {
        // Do nothing for now. Just wait for the retry request
    } else {
        if (cxlReqQueue.size() > 0) {
            assert(ioCache != nullptr);
            if (ioCache->system->isTimingMode()) {
                PacketPtr pkt = cxlReqQueue.front();
                bool success = sendTimingReqToNIC(pkt);
                if (success) {
                    if (pkt->isRead()) {
                        // CXL RD request to read the RX data from the NIC
                        numSentM2funcRXReq++;
                        allocDTARequest(pkt);
                    } else if (pkt->isWrite()) {
                        // CXL WR request to write the TX data to the NIC
                        numSentM2funcTXReq++;
                        if (pkt->isDdioHeader()) {
                            if (hasTXContext) {
                                uint64_t tx_context_ptr = topTXContextIDQueue();
                                dtaTXContextList[tx_context_ptr].n_sent++;
                            } else {
                                assert(0 && "TX Context is not available");
                            }
                        }
                    }
                    cxlReqQueue.pop_front();
                } else {
                    // Have to skip the request
                    ioCache->setIOPortWaitingOnRetry();
                }
            } else if (ioCache->system->isAtomicMode()) {
                while (!cxlReqQueue.empty()) {
                    // Send every request in the queue to send in zero time
                    PacketPtr pkt = cxlReqQueue.front();
                    if (pkt->isRead()) {
                        // CXL RD request to read the RX data from the NIC
                        numSentM2funcRXReq++;
                        allocDTARequest(pkt);
                    } else if (pkt->isWrite()) {
                        // CXL WR request to write the TX data to the NIC
                        numSentM2funcTXReq++;
                        if (pkt->isDdioHeader()) {
                            if (hasTXContext) {
                                uint64_t tx_context_ptr = topTXContextIDQueue();
                                dtaTXContextList[tx_context_ptr].n_sent++;
                            } else {
                                assert(0 && "TX Context is not available");
                            }
                        }
                    }
                    cxlReqQueue.pop_front();
                    sendAtomicReqToNIC(pkt);
                }
            }
        }
    }

    //Check threshold to determine whether to send the RD request to NIC
    if (hasRXContext) {
        uint64_t rx_context_ptr = topRXContextIDQueue();
        if (checkThreshold(rx_context_ptr)) {
            PacketPtr pkt = createDTARequest(rx_context_ptr);
            if (cxlReqQueue.size() < cxlReqQueueMaxSize) {
                cxlReqQueue.push_back(pkt);
                assert(dtaRXContextList[rx_context_ptr].valid);
                dtaRXContextList[rx_context_ptr].num_free_request_in_NIC++;
            } else {
                // Drop the packet
                delete pkt;
            }
        }
    }
}

bool
DTA::checkRXJobCompletion(uint64_t rx_context_ptr)
{   
    // Check dtaRXContextList[rx_context_ptr]'s RXCompleteMap
    // Check the dtaRXContextList[rx_context_ptr]'s n_recv and nb_pkts
    if (dtaRXContextList[rx_context_ptr].valid) {
        if (!dtaRXContextList[rx_context_ptr].completion_stage) {
            // First check n_recv == nb_pkts -> All the packets are received from the NIC
            // If above condition met, then Check the RXCompleteMap -> All the packets are DMAed to the memory
            assert(dtaRXContextList[rx_context_ptr].nb_pkts != 0);
            bool allPacketsReceived = (dtaRXContextList[rx_context_ptr].n_recv == dtaRXContextList[rx_context_ptr].nb_pkts);
            bool allPacketsDMAed = true;
            uint32_t numPacketsNotDMAed = 0;
            if (allPacketsReceived) {
                for (uint32_t i = 0; i < dtaRXContextList[rx_context_ptr].nb_pkts; i++) {
                    if (dtaRXContextList[rx_context_ptr].RXCompleteMap[dtaRXContextList[rx_context_ptr].mbuf_addr[i]] == false) {
                        allPacketsDMAed = false;
                        numPacketsNotDMAed++;
                    }
                }
                DPRINTF(DDIO, "DTA RX completion checker: All packets received from the NIC, %d/%d packets are DMAed to the memory\n", (dtaRXContextList[rx_context_ptr].nb_pkts - numPacketsNotDMAed), dtaRXContextList[rx_context_ptr].nb_pkts);
                
            }

            if (allPacketsReceived && allPacketsDMAed) {
                dtaRXContextList[rx_context_ptr].completion_stage = true; // Now, we can start the write completion info to the completion address
                DPRINTF(DDIO, "DTA RX completion checker: All packets received from the NIC and DMAed to the memory. Now, start the completion stage\n");
            }

        }
        
        if (dtaRXContextList[rx_context_ptr].completion_stage) {
            // All the packets are received from the NIC and DMAed to the memory
            // Send the completion id to the completion address
            DPRINTF(DDIO, "DTA RX completion checker: All packets received from the NIC and DMAed to the memory. So write completion status to the completion address\n");
            
            // Firstly, write the rx descriptors to the completion address. Making the cache line and write the descriptors to the cache line
            // Then, write the completion id to the completion address
            // The write process should be done 1 cache line at a time
            // Therefore, this have to check the all descriptors are written to the memory
            // If all descriptors are written to the memory, then write the completion id to the completion address

            if (dtaRXContextList[rx_context_ptr].n_desc_write_completed < dtaRXContextList[rx_context_ptr].nb_pkts) {
                // 1. Write the descriptors to the memory
                // Packing the descriptors
                uint32_t num_desc_to_write_in_one_cache_line = cacheLineSize / sizeof(RXDescriptor);
                uint32_t num_desc_to_write = std::min(num_desc_to_write_in_one_cache_line, dtaRXContextList[rx_context_ptr].nb_pkts - dtaRXContextList[rx_context_ptr].n_desc_write_completed);

                // Make the data
                uint8_t *data = new uint8_t[cacheLineSize];
                memset(data, 0, cacheLineSize);
                for (uint32_t i = dtaRXContextList[rx_context_ptr].n_desc_write_completed; i < dtaRXContextList[rx_context_ptr].n_desc_write_completed + num_desc_to_write; i++) {
                    assert(dtaRXContextList[rx_context_ptr].RXDescMap.find(dtaRXContextList[rx_context_ptr].mbuf_addr[i]) != dtaRXContextList[rx_context_ptr].RXDescMap.end());
                    assert(dtaRXContextList[rx_context_ptr].RXDescMap[dtaRXContextList[rx_context_ptr].mbuf_addr[i]] != nullptr);
                    RXDescriptor *desc = dtaRXContextList[rx_context_ptr].RXDescMap[dtaRXContextList[rx_context_ptr].mbuf_addr[i]];
                    uint32_t data_offset = (i - dtaRXContextList[rx_context_ptr].n_desc_write_completed) * sizeof(RXDescriptor);
                    memcpy(data + data_offset, desc, sizeof(RXDescriptor));
                }
                
                // Make the Request
                Addr writeAddress = dtaRXContextList[rx_context_ptr].desc_addr + (dtaRXContextList[rx_context_ptr].n_cacheline_idx_write_completed * cacheLineSize);
                RequestPtr req = std::make_shared<Request>(writeAddress, cacheLineSize, 0, requestorId);
                req->taskId(context_switch_task_id::DMA);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                pkt->allocate();
                pkt->setData(data);
                pkt->setFromDTA();

                delete[] data;

                // Send the packet through iocache (The name may be confused, because of the term "recv", but in the point of view of iocache, it receives the packet from DTA)
                assert(ioCache != nullptr);
                pushIOCacheRequestQueue(pkt);

                dtaRXContextList[rx_context_ptr].n_desc_write_completed += num_desc_to_write;
                assert(dtaRXContextList[rx_context_ptr].n_desc_write_completed <= dtaRXContextList[rx_context_ptr].nb_pkts);
                dtaRXContextList[rx_context_ptr].n_cacheline_idx_write_completed += 1;
                DPRINTF(DDIO, "DTA RX completion stage: Write the %d/%d descriptors to the memory at the %dth cache line (addr: %x)\n", 
                        dtaRXContextList[rx_context_ptr].n_desc_write_completed, dtaRXContextList[rx_context_ptr].nb_pkts, dtaRXContextList[rx_context_ptr].n_cacheline_idx_write_completed, writeAddress);
                if (dtaRXContextList[rx_context_ptr].n_desc_write_completed == dtaRXContextList[rx_context_ptr].nb_pkts) {
                    // All the descriptors are written to the memory
                    DPRINTF(DDIO, "DTA RX completion checker: All the descriptors are written to the memory\n");
                }
                // printf("DTA RX Completion Checker: Write the %d/%d descriptors to the memory with the RX Job ID: %llu\n", dtaRXContextList[rx_context_ptr].n_desc_write_completed, dtaRXContextList[rx_context_ptr].nb_pkts, dtaRXContextList[rx_context_ptr].rx_job_id);

                return true; // Need tick - Maybe need to write the next cache line

            } else {
                // 2. Write the completion id to the completion address
                // All the descriptors are written to the memory
                // Write the completion id to the completion address
                DPRINTF(DDIO, "DTA RX completion checker: All the descriptors are written to the memory. Now, write the completion id to the completion address\n");
                assert(dtaRXContextList[rx_context_ptr].n_desc_write_completed == dtaRXContextList[rx_context_ptr].nb_pkts);
                assert(dtaRXContextList[rx_context_ptr].n_cacheline_idx_write_completed == uint32_t((dtaRXContextList[rx_context_ptr].nb_pkts + (cacheLineSize/sizeof(RXDescriptor) - 1)) / (cacheLineSize/sizeof(RXDescriptor))));
                // printf("DTA RX Completion Checker: Write the completion id to the completion address with the RX Job ID: %llu completion addr: %lx\n", dtaRXContextList[rx_context_ptr].rx_job_id, dtaRXContextList[rx_context_ptr].completion_addr);

                // Make the Request
                RequestPtr req = std::make_shared<Request>(dtaRXContextList[rx_context_ptr].completion_addr, cacheLineSize, 0, requestorId);
                req->taskId(context_switch_task_id::DMA);
                PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
                // Make the data
                uint8_t *data = new uint8_t[cacheLineSize];
                memset(data, 0, cacheLineSize);
                int64_t completion_id = dtaRXContextList[rx_context_ptr].n_recv;
                memcpy(data, &completion_id, sizeof(int64_t));
                pkt->allocate();
                pkt->setData(data);
                pkt->setFromDTA();

                delete[] data;

                assert(ioCache != nullptr);
                pushIOCacheRequestQueue(pkt);

                if (zeroCopy) {
                    // Before reset, move the mbuf_addr to the tempTXMbufAddrList
                    // printf("DTA RX completion checker: Write Completion packets to completion addr: %lx with the RX Job ID: %llu. Move the mbuf addresses to the TX context\n", dtaRXContextList[rx_context_ptr].completion_addr, dtaRXContextList[rx_context_ptr].rx_job_id);
                    copyMbufAddrFromRXContextToTXContext(rx_context_ptr);
                }
                // Reset the RX context
                clearDTARXContext(rx_context_ptr);

                // Pop the RX context ID queue
                popRXContextIDQueue();
                return false; // No need tick

            }
        } else {
            return true; // Need tick
        }

    } else {
        return false;
    }
}

bool
DTA::checkTXJobCompletion(uint64_t tx_context_ptr)
{   
    // Check the dtaTXContextList[tx_context_ptr]'s n_sent and nb_pkts
    // write the completion id to the completion address
    if (dtaTXContextList[tx_context_ptr].valid) {
        assert(dtaTXContextList[tx_context_ptr].nb_pkts != 0);
        bool allPacketsSenttoNIC = (dtaTXContextList[tx_context_ptr].n_sent == dtaTXContextList[tx_context_ptr].nb_pkts);
        
        if (allPacketsSenttoNIC) {
            // For the TX, if the n_sent == nb_pkts, then all packets should be DMAed from the memory
            bool allPacketsDMAed = true;
            uint32_t numPacketsNotDMAed = 0;
            for (uint32_t i = 0; i < dtaTXContextList[tx_context_ptr].nb_pkts; i++) {
                assert(dtaTXContextList[tx_context_ptr].mbuf_addr[i] != 0);
                assert(dtaTXContextList[tx_context_ptr].TXCompleteMap.find(dtaTXContextList[tx_context_ptr].mbuf_addr[i]) != dtaTXContextList[tx_context_ptr].TXCompleteMap.end());
                if (dtaTXContextList[tx_context_ptr].TXCompleteMap[dtaTXContextList[tx_context_ptr].mbuf_addr[i]] == false) {
                    allPacketsDMAed = false;
                    numPacketsNotDMAed++;
                }
            }
            DPRINTF(DDIO, "DTA TX completion checker: All packets sent to the NIC? %d, %d/%d packets are DMAed from the memory\n", allPacketsSenttoNIC, (dtaTXContextList[tx_context_ptr].nb_pkts - numPacketsNotDMAed), dtaTXContextList[tx_context_ptr].nb_pkts);

            assert(allPacketsDMAed);

            // Write the last completion id to the completion address
            // Make the Request
            RequestPtr req = std::make_shared<Request>(dtaTXContextList[tx_context_ptr].completion_addr, cacheLineSize, 0, requestorId);
            req->taskId(context_switch_task_id::DMA);
            PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
            // Make the data
            uint8_t *data = new uint8_t[cacheLineSize];
            memset(data, 0, cacheLineSize);
            int64_t completion_id = dtaTXContextList[tx_context_ptr].n_sent;
            memcpy(data, &completion_id, sizeof(int64_t));
            pkt->allocate();
            pkt->setData(data);
            pkt->setFromDTA();

            delete[] data;           

            // Send the packet through iocache
            assert(ioCache != nullptr);
            pushIOCacheRequestQueue(pkt);               
            // Reset the TX context
            clearDTATXContext(tx_context_ptr);

            // Pop the TX context ID queue
            popTXContextIDQueue();
            return false; // No need tick
        } else {
            return true; // Need tick
        }
    } else {
        return false; // No need tick
    }
}

PacketPtr
DTA::createDTARequest(uint64_t rx_context_ptr)
{
    // Create M2func RD/WR request to NIC
    assert(isDTAEnabled());
    // Make Request
    RequestPtr req = std::make_shared<Request>(M2funcRXAddr, flitSize, 0, requestorId);
    PacketPtr pkt = new Packet(req, MemCmd::ReadReq);
    pkt->allocate(); // Allocate the data buffer
    pkt->setJobID(dtaRXContextList[rx_context_ptr].rx_job_id);
    
    return pkt;
}

// Request Tracker: track the sent request to NIC and track the response is coming or not
void 
DTA::allocDTARequest(PacketPtr pkt)
{
    // Allocate new DTA request tracker entry
    assert(isDTAEnabled());
    assert(dtaRequestTracker.find(pkt->req) == dtaRequestTracker.end());
    dtaRequestTracker[pkt->req] = false;
}

bool 
DTA::findDTARequest(PacketPtr pkt)
{
    // Find the DTA request tracker entry
    assert(isDTAEnabled());
    if (dtaRequestTracker.find(pkt->req) != dtaRequestTracker.end()) {
        return true;
    } else {
        return false;
    }
}

void 
DTA::freeDTARequest(PacketPtr pkt) 
{
    // Free the DTA request tracker entry 
    assert(isDTAEnabled());
    RequestPtr req = pkt->req;
    assert(dtaRequestTracker.find(req) != dtaRequestTracker.end());
    dtaRequestTracker.erase(req);

    if (hasRXContextIDQueue()) {
        uint64_t rx_context_ptr = topRXContextIDQueue();

        if (dtaRXContextList[rx_context_ptr].valid) {
            if (dtaRXContextList[rx_context_ptr].rx_job_id == pkt->getJobID()) {
                // The request is the response for the RX job
                // Increase the number of free request in NIC
                dtaRXContextList[rx_context_ptr].num_free_request_in_NIC--;
            }
        }
    }
}

void 
DTA::sendIOCacheRequest()
{
    // check the iocache request queue
    // try send request to iocache
    // Have to check iocache is blocked or not
    // If full, have to wait for the retry
    if (!ioCache->isBlocked()) {
        if (hasIOCacheRequestQueue()) {
            PacketPtr pkt = ioCacheRequestQueue.front();
            bool success = ioCache->recvTimingReqfromDTA(pkt);
            if (success) {
                ioCacheRequestQueue.pop_front();
            }
        }
    }
}


DTA::DTARXWorker *
DTA::findDTARXWorker(PacketPtr pkt, uint64_t rx_context_ptr)
{
    if (pkt->isDdioHeader()) {
        // New Ethernet packet -> find an idle worker
        for (uint32_t i = 0; i < numDTARXWorker; i++) {
            if (dtaRXWorkerList[i]->isFree()) {
                // Assign this free worker for the new packet
                dtaRXContextList[rx_context_ptr].lastDTARXWorker = i;
                return dtaRXWorkerList[i];
            }
        }

        return nullptr;
    } else {
        // Continuation of the previous packet -> find the worker that is processing the packet
        uint32_t lastWorker = dtaRXContextList[rx_context_ptr].lastDTARXWorker;
        if (lastWorker >= 0 && lastWorker < numDTARXWorker) {
            DTA::DTARXWorker* worker = dtaRXWorkerList[lastWorker];
            if (!worker->isFree()) {
                return worker;
            } else {
                panic("Error: Cannot find the last worker that is processing the packet\n");
            }
        } else {
            panic("Error: Wrong last worker index that is processing the packet\n");
        }
    }
}

void
DTA::recvTimingRespfromNIC(PacketPtr pkt)
{   
    // Receive M2func RD/WR response from NIC - it can be RD/WR response
    assert(isDTAEnabled());
    assert(pkt->isResponse());
    if (hasRXContextIDQueue() && dtaRXContextList[topRXContextIDQueue()].valid && dtaRXContextList[topRXContextIDQueue()].rx_job_id == pkt->getJobID()) {
        uint64_t rx_context_ptr = topRXContextIDQueue();
        if (findDTARequest(pkt)) {
            freeDTARequest(pkt);
            assert(pkt->isRead());
            // Push to the worker waiting queue
            // Check the packet data - contain only 0s
            uint8_t* data = pkt->getPtr<uint8_t>();
            bool allZero = true;
            for (int i = 0; i < pkt->getSize(); i++) {
                if (data[i] != 0) {
                    allZero = false;
                    break;
                }
            }
            if (allZero) {
                DPRINTF(DDIO, "DTA receive response from NIC. But the packet data is all 0s\n");
                // printf("[NO_ETH_RX_PACKET]: DTA receive response from NIC. But the packet data is all 0s pkt's job id: %llu\n", pkt->getJobID());
                delete pkt;
                // Have to write -1 to the completion address to notify the host that the packet is not received
                // Make the Request
                RequestPtr req = std::make_shared<Request>(dtaRXContextList[rx_context_ptr].completion_addr, cacheLineSize, 0, requestorId);
                req->taskId(context_switch_task_id::DMA);
                PacketPtr wr_pkt = new Packet(req, MemCmd::WriteReq);
                // Make the data
                uint8_t *data = new uint8_t[cacheLineSize];
                memset(data, 0, cacheLineSize);
                int64_t completion_id = -1;
                memcpy(data, &completion_id, sizeof(int64_t));
                wr_pkt->allocate();
                wr_pkt->setData(data);
                wr_pkt->setFromDTA();

                delete[] data;

                assert(ioCache != nullptr);
                pushIOCacheRequestQueue(wr_pkt);
                
                // Reset the RX context
                clearDTARXContext(rx_context_ptr);

                // Pop the RX context ID queue
                popRXContextIDQueue();

                return;
                
            } else {
                // TODO - maybe have to check the queue size
                workerWaitingQueue.push_back(std::pair<PacketPtr, uint32_t>(pkt, dtaRXContextList[rx_context_ptr].n_recv)); 

                if (pkt->isDdioHeader()) {
                    // Only increase the n_recv when the packet is the header of the ethernet packet
                    // If not, it means one ethernet packet is not fully received from NIC (partially received in CXL flit size unit)
                    dtaRXContextList[rx_context_ptr].n_recv++; 
                } else {
                    // This is the continuation of the previous packet
                    dtaRXContextList[rx_context_ptr].n_extra_cxl_req_received++;
                }     

                // If tickEvent is not running, start the clock
                rxTick = true;
                if (rxTick && !tickEvent.scheduled()) {
                    restartClock();
                }  
            }
        } else {
            // printf("Error: DTA receive response. But cannot find the DTA request, pkt addr: %lx\n", pkt->req->getPaddr());
            panic("Error: DTA receive response. But cannot find the DTA request\n");
        }
         
    } else if (hasTXContextIDQueue() && pkt->isWrite()) {
        uint64_t tx_context_ptr = topTXContextIDQueue();
        assert(dtaTXContextList[tx_context_ptr].valid);
        // Nothing to do now. But we can add some stats here.
        // This is the response for the TX packet write request, which is sent to the NIC
        delete pkt;
    } else {
        // This can happen when the DTA TX response is received, but the DTA TX context is cleared (not checking the response of NIC)
        // Also, for the RX, this can happen when there are no ethernet packets to receive from the NIC. DTARXContext is already cleared from the previous CXL resp with all 0s
        // Check the findDTARequest function
        if (hasRXContextIDQueue() && dtaRXContextList[topRXContextIDQueue()].valid && dtaRXContextList[topRXContextIDQueue()].rx_job_id != pkt->getJobID()) {
            // printf("DTA receive response from NIC. But the DTA context's job id (%llu) is different from the packet's job id (%llu)\n", dtaRXContextList[topRXContextIDQueue()].rx_job_id, pkt->getJobID());
        } else {
            // printf("DTA receive response from NIC. But the DTA context is not valid. pkt's job id: %llu\n", pkt->getJobID());
        }
        if (findDTARequest(pkt)) {
            freeDTARequest(pkt);
        }
        delete pkt;
    }
    
    
}

void
DTA::recvTimingRespfromCache(PacketPtr pkt, gem5::Tick when)
{
    // Receive DMA response from the cache
    assert(isDTAEnabled());
    assert(when >= curTick());
    auto it = DMARespWaitingQueue.end();
    while (it != DMARespWaitingQueue.begin()) {
        --it;
        if (it->tick <= when) {
            DMARespWaitingQueue.emplace(++it, when, pkt);
            return;
        }
    }
    
    // If the queue is empty or, this new packet should be the first one, explicitly call the schedDMACompletionEvent
    DMARespWaitingQueue.emplace_front(when, pkt);
    schedDMACompletionEvent(when);

}

void 
DTA::schedDMACompletionEvent(gem5::Tick when)
{
    if (when != MaxTick) {
        when = (when > (curTick() + 1)) ? when : (curTick() + 1);
        if (!dmaCompletionEvent.scheduled()) {
            schedule(dmaCompletionEvent, when);
        } else {
            reschedule(dmaCompletionEvent, when);
        }
    }
}

void
DTA::handleDMACompletion()
{
    // Pop the packet from the DMARespWaitingQueue
    // This function is scheduled by the Event
    assert(isDTAEnabled());
    assert(DMARespWaitingQueue.size() > 0);
    WaitingPacket wp = DMARespWaitingQueue.front();
    DMARespWaitingQueue.pop_front();
    PacketPtr pkt = wp.pkt;
    if (pkt->isRXDMA()) {
        // RX DMA response
        int workerID = pkt->getWorkerID();
        assert(workerID >= 0 && workerID < numDTARXWorker);
        DTARXWorker* worker = dtaRXWorkerList[workerID];
        assert(worker->getState() != DTARXWorker::workerState::IDLE);
        worker->handleDMACompletion(pkt);

        delete pkt;
    } else if (pkt->isTXDMA()) {
        // TX DMA response
        int workerID = pkt->getWorkerID();
        assert(workerID >= 0 && workerID < numDTATXWorker);
        DTATXWorker* worker = dtaTXWorkerList[workerID];
        assert(worker->getState() != DTATXWorker::workerState::IDLE);
        worker->handleDMACompletion(pkt);

        delete pkt;
    } else {
        // This can be the response to the request of the completion status
        delete pkt;
    }

    // Schedule the next event if there is any packet in the queue
    Tick nextTick = DMARespWaitingQueue.empty()? MaxTick : DMARespWaitingQueue.front().tick;
    schedDMACompletionEvent(nextTick);
}

bool 
DTA::allocateWorkerFromQueue(uint64_t rx_context_ptr)
{
    // Allocate worker from the queue
    if (workerWaitingQueue.size() > 0) {
        PacketPtr respPkt = workerWaitingQueue.front().first;
        uint32_t n_recv = workerWaitingQueue.front().second;
        // Find free DTARXWorker or DTARXWorker waiting new CXL.mem response
        DTARXWorker* worker = findDTARXWorker(respPkt, rx_context_ptr);
        if (worker != nullptr) {
            if (worker->isFree()) {
                // If free, set new RX packet setting 
                assert(dtaRXContextList[rx_context_ptr].mbuf_addr[n_recv] != 0);
                worker->allocateWorker(dtaRXContextList[rx_context_ptr].mbuf_addr[n_recv], rx_context_ptr);
                //Process the response ASAP, to set the payload size !!! Must do that at this point
            } 
            worker->processDTAResponse(respPkt);

            workerWaitingQueue.pop_front();
            delete respPkt;
        }
        return true; // Need tick
    } else {
        return false; // No need tick
    }

}

void
DTA::DTARXWorker::processDTAResponse(PacketPtr currPkt)
{
    // Process the response from NIC
    // return true: done processing one ethernet packet, false: need to receive more CXL.mem responses
    assert(dta->isDTAEnabled());
    assert(payloadWCBuffer != nullptr);
    assert(currProcessingPkt == nullptr);
    currProcessingPkt = currPkt;
    // Parsing descriptor and payload from the response
    uint8_t* data = currProcessingPkt->getPtr<uint8_t>();
    uint64_t payloadInCurrentPkt = currProcessingPkt->getSize();

    if (descriptor == nullptr) {
        assert(currProcessingPkt->isDdioHeader());
        // The first CXL.mem response contains the descriptor part
        // currProcessingPkt contains the descriptor part
        descriptor = new RXDescriptor();
        memcpy(descriptor, data, sizeof(RXDescriptor));
        
        // Ethernet packet size
        assert(payloadSize == 0);
        payloadSize = descriptor->pkt_len;

        // currPayloadSize has to be modified to exclude the descriptor part
        payloadInCurrentPkt -= sizeof(RXDescriptor);
        receivedPayloadSize += payloadInCurrentPkt;

        // Calculate the extra needed CXL.req to receive the full ethernet packet
        uint32_t extraCxlReq = ((payloadSize - receivedPayloadSize) + dta->flitSize - 1) / dta->flitSize;
        assert(rxContextID != -1);
        assert(rxContextID < dta->dtaRXContextList.size());
        dta->dtaRXContextList[rxContextID].n_extra_cxl_req_needed += extraCxlReq;

        // Copy payload part to payloadWCBuffer
        assert(currentPayloadWCBufferSize == 0);
        memcpy(payloadWCBuffer + currentPayloadWCBufferSize, data + sizeof(RXDescriptor), payloadInCurrentPkt);
        currentPayloadWCBufferSize += payloadInCurrentPkt;
        
        // Set descriptor to the RXDescMap of the dtaRXContextList[rxContextID]
        assert(mbuf_addr != 0);
        setDescriptor(descriptor, mbuf_addr);
        
        DPRINTF(DDIO, "DTARXWorker[%d]: received the descriptor part of the ethernet packet\n", workerID);
        DPRINTF(DDIO, "DTARXWorker[%d]: total payload size (excluding descriptor): %ld, received payload size: %ld\n", workerID, payloadSize, receivedPayloadSize);

    } else {
        // The following CXL.mem responses contain the payload part
        // currProcessingPkt contains the payload part
        assert(payloadSize != 0);
        assert(receivedPayloadSize != 0);

        // currPayloadSize has to be modified to exclude the descriptor part
        receivedPayloadSize += payloadInCurrentPkt;
        assert(receivedPayloadSize <= payloadSize);

        // Copy payload part to payloadWCBuffer
        memcpy(payloadWCBuffer + currentPayloadWCBufferSize, data, payloadInCurrentPkt);
        currentPayloadWCBufferSize += payloadInCurrentPkt;

        DPRINTF(DDIO, "DTARXWorker[%d]: received the payload part of the ethernet packet\n", workerID);
        DPRINTF(DDIO, "DTARXWorker[%d]: total payload size (excluding descriptor): %ld, received payload size: %ld\n", workerID, payloadSize, receivedPayloadSize);

    }

    if (!dmaReady && (currentPayloadWCBufferSize >= cacheLineSize)) {
        // The first moment that can start DMA transfer
        dmaReady = true;
        // MAke dmaReqState
        dmaReqState = new DTADMAReqState(mbuf_addr, payloadSize, nullptr);
        DPRINTF(DDIO, "DTA RX Worker[%d]: DMA_READY! Received enough payload exceeding cacheline size. currentPayloadWCBufferSize: %ld\n", workerID, currentPayloadWCBufferSize);
    }

    if (receivedPayloadSize == payloadSize) {
        // Done receiving the ethernet packet from NIC
        allPayloadReceived = true;
        DPRINTF(DDIO, "DTA RX Worker[%d]: All payload received!\n", workerID);

        // Have to check dmaReady is not set.
        // This can happen when the payload size is smaller than the cache line size
        if (!dmaReady) {
            // The first moment that can start DMA transfer
            dmaReady = true;
            // MAke dmaReqState
            dmaReqState = new DTADMAReqState(mbuf_addr, payloadSize, nullptr);
            DPRINTF(DDIO, "DTA RX Worker[%d]: DMA_READY! Not received enough payload exceeding cacheline size, but this payload size (%ld) is smaller than cacheline size (%ld)\n", workerID, payloadSize, cacheLineSize);
        }
    }

    
    currProcessingPkt = nullptr;
}

PacketPtr 
DTA::DTARXWorker::makePacket()
{
    // Initiate DMA transfer - exploit existing iocache's DMA functions
    // Make Request and Packet for DMA transfer
    
    // Calculate dma address. The base address is the mbuf_addr. Have to add the offset of the payloadWCBuffer
    Addr dmaAddr = mbuf_addr + sentDMASize;
    // set dma size
    uint64_t dmaSize = (currentPayloadWCBufferSize - sentDMASize) >= cacheLineSize ? cacheLineSize : (currentPayloadWCBufferSize - sentDMASize);
    // set dma data
    uint8_t dmaData[cacheLineSize];
    memset(dmaData, 0, cacheLineSize);
    memcpy(dmaData, payloadWCBuffer + sentDMASize, dmaSize);
    
    // Make Request
    // TODO - have to check the requestorId
    RequestPtr req = std::make_shared<Request>(dmaAddr, cacheLineSize, 0, dta->requestorId);
    req->taskId(context_switch_task_id::DMA);

    PacketPtr pkt = new Packet(req, MemCmd::WriteReq);

    if (dta->enableDdio)
    {
        pkt->setDdioPrefetchId(-1); //Not Idio
        pkt->setDdioPrefetchDestination(-1); //Not Idio
        pkt->setDdioPkt();

        if (sentDMASize == 0) {
            pkt->setDdioHeader();
        }
    }

    pkt->allocate(); // Allocate the data buffer
    pkt->setRXDMA();
    pkt->setWorkerID(workerID);
    pkt->setFromDTA();

    pkt->setData(dmaData);

    assert(dmaReqState != nullptr);
    pkt->senderState = dmaReqState;

    // Adjust the metadata
    sentDMASize += dmaSize;
    
    return pkt;
}

bool
DTA::DTARXWorker::sendDMA(PacketPtr pkt)
{
    // Send DMA request - exploit existing iocache's DMA functions
    assert(dta->ioCache != nullptr);
    dta->pushIOCacheRequestQueue(pkt);
    return true;
}

void
DTA::DTARXWorker::handleDMACompletion(PacketPtr pkt)
{   
    // Handle DMA completion - exploit existing iocache's DMA functions
    dmaTransferredSize += pkt->getSize();

    if (dmaTransferredSize >= payloadSize) {
        // Done DMA transfer
        dmaComplete = true;
        // notify DTA that the DMA is complete by updating the dtaRXContextList[rxContextID]'s data
        assert(mbuf_addr != 0);
        assert(rxContextID != -1);
        assert(rxContextID < dta->dtaRXContextList.size());
        assert(dta->dtaRXContextList[rxContextID].RXCompleteMap.find(mbuf_addr) != dta->dtaRXContextList[rxContextID].RXCompleteMap.end());
        dta->dtaRXContextList[rxContextID].RXCompleteMap[mbuf_addr] = true;

        DPRINTF(DDIO, "DTA RX Worker: DMA Complete!\n");
    }

     // Check tickEvent is not running, start the clock
    dta->rxTick = true;
    if (dta->rxTick && !dta->tickEvent.scheduled()) {
        dta->restartClock();
    }
}

bool
DTA::DTARXWorker::DTAWork()
{
    // DTA work function
    // return true: need ticking, return false: no need ticking
    if (state == workerState::IDLE) {
        return false; // No need ticking
    } else if (state == workerState::WORKING) {
        // The worker is working
        // Have to check if there is a packet to send

        // Prioritize the retry packet
        PacketPtr sendPacket = inRetry ? inRetry : nullptr;

        // Check if the worker is ready for making new DMA request
        bool WCBufferReadyforDMA = false;
        if ( currentPayloadWCBufferSize - sentDMASize >= cacheLineSize) {
            WCBufferReadyforDMA = true;
        } else if ( allPayloadReceived && currentPayloadWCBufferSize - sentDMASize > 0) {
            WCBufferReadyforDMA = true;
        }

        // If no retry packet, check if the worker is ready for making new DMA request & make new DMA request
        if (!sendPacket) {
            if (WCBufferReadyforDMA) {
                assert(dmaReady);
                sendPacket = makePacket();
            }
        }
        inRetry = nullptr;

        if (sendPacket) {
            // Send the packet
            if (!sendDMA(sendPacket)) {
                // If failed to send the packet, set the packet to inRetry
                inRetry = sendPacket;
            }
        }

        // Check if the worker is done
        if (sentDMASize < payloadSize) {
            return true; // Need ticking
        } else if (allPayloadReceived && dmaComplete) {
            // Done DMAed all the payload
            freeWorker();
            return false; // No need ticking
        }
        else {
            // Done sending all DMA requests. Wait for the completion
            // When DMA response is received, handleDMACompletion() will trigger DTAWork() again
            return false; // No need ticking
        }

    }
    
}

void 
DTA::allocateDescriptorWorker(uint64_t tx_context_ptr)
{   
    // For TX, have to allocate workers for descriptor DMA
    // Allocate workers for descriptor DMA
    // Have to allocate until the nb_pkts * 8B is DMAed 
    // Total DMA size = nb_pkts * 8B (descriptor size)
    // Worker will be allocated with cache line size (maybe 64B)
    // If no worker is available, have to wait until the worker is free

    if (dtaTXContextList[tx_context_ptr].valid) {
        if (dtaTXContextList[tx_context_ptr].n_desc_ready >= dtaTXContextList[tx_context_ptr].nb_pkts) {
            // All the descriptor is ready
            return;
        }

        for (auto& worker: dtaTXWorkerList) {
            if (worker->isFree() && dtaTXContextList[tx_context_ptr].n_desc_ready < dtaTXContextList[tx_context_ptr].nb_pkts) {
                uint64_t dmaSize = cacheLineSize;
                if ((dtaTXContextList[tx_context_ptr].nb_pkts - dtaTXContextList[tx_context_ptr].n_desc_ready) * sizeof(TXDescriptor) < cacheLineSize) {
                    dmaSize = (dtaTXContextList[tx_context_ptr].nb_pkts - dtaTXContextList[tx_context_ptr].n_desc_ready) * sizeof(TXDescriptor);
                }
                Addr dmaAddr = dtaTXContextList[tx_context_ptr].desc_addr + dtaTXContextList[tx_context_ptr].n_desc_ready * sizeof(TXDescriptor);
                worker->allocateWorker(dmaAddr, dmaSize, tx_context_ptr, DTATXWorker::workerState::DESC_PROCESSING);
                worker->setOffset(dtaTXContextList[tx_context_ptr].n_desc_ready);

                // Increase the number of descriptor ready
                uint32_t numDescSent = dmaSize / sizeof(TXDescriptor);
                assert(numDescSent * sizeof(TXDescriptor) == dmaSize);
                dtaTXContextList[tx_context_ptr].n_desc_ready += numDescSent;
            }
        }
    }

}

void 
DTA::allocatePayloadWorker(uint64_t tx_context_ptr)
{   
    // For TX, have to allocate workers for payload DMA
    // Allocate workers for payload DMA
    // If descPayloadDMAWaiting has the descriptor, have to allocate the worker for the payload DMA
    // And delete that pair from descPayloadDMAWaiting and add to descPayloadDMAAssigned
    // If no worker is available, have to wait until the worker is free
    for (auto& waiter: dtaTXContextList[tx_context_ptr].descPayloadDMAWaiting) {
        // Check if the worker is free
        for (auto& worker: dtaTXWorkerList) {
            if (worker->isFree()) {
                Addr dmaAddr = waiter.first;
                TXDescriptor dtaTXDesc = waiter.second;
                uint64_t dmaSize = dtaTXDesc.dtalen;

                // Add to descPayloadDMAAssigned
                assert(dtaTXContextList[tx_context_ptr].descPayloadDMAAssigned.find(dmaAddr) == dtaTXContextList[tx_context_ptr].descPayloadDMAAssigned.end());
                dtaTXContextList[tx_context_ptr].descPayloadDMAAssigned[dmaAddr] = dtaTXDesc;

                // Allocate the worker
                worker->allocateWorker(dmaAddr, dmaSize, tx_context_ptr,
                            DTATXWorker::workerState::PAYLOAD_PROCESSING, &(dtaTXContextList[tx_context_ptr].descPayloadDMAAssigned[dmaAddr]));
                
                // Delete from descPayloadDMAWaiting
                dtaTXContextList[tx_context_ptr].descPayloadDMAWaiting.erase(dmaAddr);

                break;
            }
        }
    }

}


PacketPtr
DTA::DTATXWorker::makePacket()
{
    if (remainingSize > 0) {
        Addr dmaAddr = baseAddr + transferredSize;       
        uint64_t dmaSize = (cacheLineSize < remainingSize) ? cacheLineSize : remainingSize;

        //Create Request
        RequestPtr req = std::make_shared<Request>(dmaAddr, dmaSize, 0, dta->requestorId);
        req->taskId(context_switch_task_id::DMA);
        PacketPtr pkt = new Packet(req, MemCmd::ReadReq);

        pkt->allocate(); // Allocate the data buffer
        pkt->setTXDMA();
        pkt->setWorkerID(workerID);
        pkt->setFromDTA();
        assert(txContextID != -1);
        assert(txContextID < dta->dtaTXContextList.size());
        pkt->setJobID(dta->dtaTXContextList[txContextID].tx_job_id);

        assert(dmaReqState != nullptr);
        pkt->senderState = dmaReqState;

        // Adjust the metadata
        transferredSize += dmaSize;
        remainingSize -= dmaSize;

        return pkt;
    } else {
        return nullptr;
    }
}

bool
DTA::DTATXWorker::sendDMA(PacketPtr pkt)
{   
    // Send DMA request - exploit existing iocache's DMA functions
    assert(dta->ioCache != nullptr);
    dta->pushIOCacheRequestQueue(pkt);
    return true;
}

void  
DTA::DTATXWorker::handleDMACompletion(PacketPtr pkt)
{  
    // Parse the received data
    if (state == workerState::DESC_PROCESSING) {
        // Parsing the packet data to the multiple TXDescriptors
        // Calculate the offset of each TXDescriptor through offsetFromDescAddr
        // Get the corresponding mbuf_addr using the offset and set the descPayloadDMAWaiting

        uint8_t* data = pkt->getPtr<uint8_t>();
        uint64_t pktSize = pkt->getSize();
        uint64_t numDesc = pktSize / sizeof(TXDescriptor);
        assert(numDesc * sizeof(TXDescriptor) == pktSize);
        uint64_t startOffset = offsetFromDescAddr;
        for (uint64_t i = 0; i < numDesc; i++) {
            TXDescriptor dtaTXDesc;
            memcpy(&dtaTXDesc, data + i * sizeof(TXDescriptor), sizeof(TXDescriptor));
            uint64_t mbufListOffset = startOffset + i;
            assert(mbufListOffset < DTA_MAX_MBUF_NUM);
            if (dta->dtaTXContextList[txContextID].mbuf_addr[mbufListOffset] == 0) {
                assert(!dta->zeroCopy);
                // This can happen, when the mbuf_addr is not received yet. (Not enough job request to NIC).
                // So, temporarily store the descriptor to the descWaitingMbufAddr with it's mbufListOffset. 
                // When the job request is received, descWaitingMbufAddr will be checked and the descriptor will be moved to the descPayloadDMAWaiting
                assert(dta->dtaTXContextList[txContextID].descWaitingMbufAddr.find(mbufListOffset) == dta->dtaTXContextList[txContextID].descWaitingMbufAddr.end());
                dta->dtaTXContextList[txContextID].descWaitingMbufAddr[mbufListOffset] = dtaTXDesc;
            } else {
                Addr mbufAddr = dta->dtaTXContextList[txContextID].mbuf_addr[mbufListOffset];
                assert(dta->dtaTXContextList[txContextID].descPayloadDMAWaiting.find(mbufAddr) == dta->dtaTXContextList[txContextID].descPayloadDMAWaiting.end());
                dta->dtaTXContextList[txContextID].descPayloadDMAWaiting[mbufAddr] = dtaTXDesc;
            }
        }

    } else if (state == workerState::PAYLOAD_PROCESSING) {
        // Parsing the packet data to the payloadBuffer
        // Use dmaReceivedSize to know the point to copy the data
        uint8_t* data = pkt->getPtr<uint8_t>();
        uint64_t pktSize = pkt->getSize();
        Addr dmaAddr = pkt->getAddr();
        Addr offsetFromBaseAddr = dmaAddr - baseAddr;
        assert(dmaReceivedSize + pktSize <= dmaSize);
        memcpy(payloadBuffer + offsetFromBaseAddr, data, pktSize);
    }

    // Handle DMA completion - exploit existing iocache's DMA functions
    dmaReceivedSize += pkt->getSize();

    if (dmaReceivedSize >= dmaSize) {
        // Done DMA transfer
        dmaComplete = true;
        DPRINTF(DDIO, "DTA TX Worker: DMA Complete!\n");
    }

     // Check tickEvent is not running, start the clock
    dta->txTick = true;
    if (dta->txTick && !dta->tickEvent.scheduled()) {
        dta->restartClock();
    }
}

bool 
DTA::DTATXWorker::notifyWorkerCompletion()
{
    // Notify the worker completion to the DTA
    // Return true: success, false: fail

    if (state == workerState::DESC_PROCESSING) {
        // Nothing to do
        return true;
    } else if (state == workerState::PAYLOAD_PROCESSING) {
        // Calculate the needed CXL.mem request number to check the cxlWRReqQueue size
        uint64_t numCxlReq = ((dmaSize + sizeof(TXDescriptor)) + dta->flitSize - 1) / dta->flitSize;
        if (dta->cxlReqQueue.size() + numCxlReq > dta->cxlReqQueueMaxSize) {
            return false; // Cannot send the request
        }
        
        // Set dtaTXContextList[txContextID]'s TXCompleteMap to notify the completion
        assert(dta->dtaTXContextList[txContextID].TXCompleteMap.find(baseAddr) != dta->dtaTXContextList[txContextID].TXCompleteMap.end());
        if (dta->dtaTXContextList[txContextID].TXCompleteMap[baseAddr] == true) {
            printf("Error: TXCompleteMap[%lx] is already true\n", baseAddr);
            fflush(stdout);
        }
        assert(dta->dtaTXContextList[txContextID].TXCompleteMap[baseAddr] == false);
        dta->dtaTXContextList[txContextID].TXCompleteMap[baseAddr] = true;

        // Push to the cxlWRReqQueue
        // Have to make the multipe CXL flit size packets using the descriptor and payload
        // Descriptor will be at the first part of the first packet
        // Payload will be at the second part of the first packet and the following packets
        // Have to make the multiple packets until the payload is fully sent
        // Use descriptor & payloadBuffer

        uint8_t * ethernetPkt = new uint8_t[dmaSize + sizeof(TXDescriptor)];
        memcpy(ethernetPkt, descriptor, sizeof(TXDescriptor));
        memcpy(ethernetPkt + sizeof(TXDescriptor), payloadBuffer, dmaSize);

        // Erase finished descPayloadDMAAssigned 
        assert(dta->dtaTXContextList[txContextID].descPayloadDMAAssigned.find(baseAddr) != dta->dtaTXContextList[txContextID].descPayloadDMAAssigned.end());
        dta->dtaTXContextList[txContextID].descPayloadDMAAssigned.erase(baseAddr);

        // Make multiple packets
        for (uint64_t i = 0; i < dmaSize + sizeof(TXDescriptor); i += dta->flitSize) {
            uint64_t size = ((dta->flitSize) < (dmaSize + sizeof(TXDescriptor) - i)) ? dta->flitSize : (dmaSize + sizeof(TXDescriptor) - i);
            
            RequestPtr req = std::make_shared<Request>(dta->M2funcTXAddr, size, 0, dta->requestorId);
            PacketPtr pkt = new Packet(req, MemCmd::WriteReq);
            pkt->allocate(); // Allocate the data buffer
            pkt->setData(ethernetPkt + i);

            if (i == 0) {
                pkt->setDdioHeader();
            }

            dta->cxlReqQueue.push_back(pkt);
        }

        // Delete the ethernetPkt
        delete[] ethernetPkt;

        return true;

    }
}

bool  
DTA::DTATXWorker::DTAWork()
{
    if (state == workerState::IDLE) {
        return false;
    }

    // Have to check if there is a packet to send

    // Prioritize the retry packet
    PacketPtr sendPacket = inRetry ? inRetry : nullptr;

    // Check if the worker is ready for making new DMA request
    bool workerReadyforDMA = false;
    if (remainingSize > 0) {
        workerReadyforDMA = true;
    }

    // If no retry packet, check if the worker is ready for making new DMA request & make new DMA request
    if (!sendPacket) {
        if (workerReadyforDMA) {
            sendPacket = makePacket();
        }
    }
    inRetry = nullptr;

    if (sendPacket) {
        // Send the packet
        if (!sendDMA(sendPacket)) {
            // If failed to send the packet, set the packet to inRetry
            inRetry = sendPacket;
        }
    }

    // Check if the worker is done
    if (remainingSize > 0) {
        return true; // Need ticking
    } else if (dmaComplete) {
        // Done DMAed all the descriptor
        if (notifyWorkerCompletion()) {
            // Success to notify the worker completion - push the CXL.mem WR requests to the queue
            freeWorker();
            return false; // No need ticking
        } else {
            // Fail to notify the worker completion - have to wait until the queue is free
            return true; // Need ticking
        }
    }
    else {
        // Done sending all DMA requests. Wait for the completion
        // When DMA response is received, handleDMACompletion() will trigger DTAWork() again
        return false; // No need ticking
    }
    
}


bool
DTA::DTARXStateMachine()
{
    bool needTicking = false;

    if (hasRXContextIDQueue()) {
        // Check the RX job completion
        uint64_t rx_context_ptr = topRXContextIDQueue();
        needTicking = checkRXJobCompletion(rx_context_ptr) | needTicking;

        // Iterate through all the workers
        for (auto& worker: dtaRXWorkerList) {
            if (worker->DTAWork()) {
                needTicking = true | needTicking;
            }
        }

        // Allocate Worker
        needTicking = allocateWorkerFromQueue(rx_context_ptr) | needTicking;
    }

    return needTicking;

}
    
bool
DTA::DTATXStateMachine()
{
    bool needTicking = false;

    if (hasTXContextIDQueue()) {
        uint64_t tx_context_ptr = topTXContextIDQueue();
        // Check the TX job completion
        needTicking = checkTXJobCompletion(tx_context_ptr) | needTicking;

        // Iterate through all the workers
        for (auto& worker: dtaTXWorkerList) {
            if (worker->DTAWork()) {
                needTicking = true | needTicking;
            }
        }

        // Allocate Worker for Payload DMA
        allocatePayloadWorker(tx_context_ptr);

        // Allocate Worker for Descriptor DMA
        allocateDescriptorWorker(tx_context_ptr);
    }

    return needTicking;

}

void 
DTA::tick()
{   
    if (isDTAEnabled()) {
        DPRINTF(DDIO, "DTA tick\n");

        sendRequestToNIC();
        sendIOCacheRequest();

        if (rxTick) {
            rxTick |= DTARXStateMachine();
        }
        if (txTick) {
            txTick |= DTATXStateMachine();
        }
        
        checkSubmissionQueue();

        if (rxTick || txTick || (hasIOCacheRequestQueue()) || (cxlReqQueue.size() > 0)) {
            schedule(tickEvent, curTick() + clockPeriod());
        }   
    }
}

void 
DTA::restartClock()
{
    assert(isDTAEnabled());
    if (!tickEvent.scheduled()) {
        schedule(tickEvent, clockEdge(Cycles(1)));
    }
}

} // namespace gem5
