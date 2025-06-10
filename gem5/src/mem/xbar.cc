/*
 * Copyright (c) 2011-2015, 2018-2020 ARM Limited
 * All rights reserved
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

/**
 * @file
 * Definition of a crossbar object.
 */

#include "mem/xbar.hh"

#include "base/logging.hh"
#include "base/trace.hh"
#include "debug/AddrRanges.hh"
#include "debug/Drain.hh"
#include "debug/XBar.hh"

namespace gem5
{

BaseXBar::BaseXBar(const BaseXBarParams &p)
    : ClockedObject(p),
      frontendLatency(p.frontend_latency),
      forwardLatency(p.forward_latency),
      responseLatency(p.response_latency),
      headerLatency(p.header_latency),
      width(p.width),
      gotAddrRanges(p.port_default_connection_count +
                          p.port_mem_side_ports_connection_count, false),
      gotAllAddrRanges(false), defaultPortID(InvalidPortID), DTAJobPortID(InvalidPortID), M2funcPortID(InvalidPortID),
      useDefaultRange(p.use_default_range),
      isIOXBar(p.is_ioxbar),
      isL3XBar(false),
      enableDTA(p.enable_dta),
      modelPCIe1us(p.model_pcie_1us),

      ADD_STAT(transDist, statistics::units::Count::get(),
               "Transaction distribution"),
      ADD_STAT(pktCount, statistics::units::Count::get(),
               "Packet count per connected requestor and responder"),
      ADD_STAT(pktSize, statistics::units::Byte::get(),
               "Cumulative packet size per connected requestor and responder")
{
}

BaseXBar::~BaseXBar()
{
    for (auto port: memSidePorts)
        delete port;

    for (auto port: cpuSidePorts)
        delete port;
}

Port &
BaseXBar::getPort(const std::string &if_name, PortID idx)
{
    if (if_name == "mem_side_ports" && idx < memSidePorts.size()) {
        // the memory-side ports index translates directly to the vector
        // position
        return *memSidePorts[idx];
    } else  if (if_name == "default") {
        return *memSidePorts[defaultPortID];
    } else if (if_name == "cpu_side_ports" && idx < cpuSidePorts.size()) {
        // the CPU-side ports index translates directly to the vector position
        return *cpuSidePorts[idx];
    } else {
        return ClockedObject::getPort(if_name, idx);
    }
}

void
BaseXBar::calcPacketTiming(PacketPtr pkt, Tick header_delay)
{
    // the crossbar will be called at a time that is not necessarily
    // coinciding with its own clock, so start by determining how long
    // until the next clock edge (could be zero)
    Tick offset = clockEdge() - curTick();

    // the header delay depends on the path through the crossbar, and
    // we therefore rely on the caller to provide the actual
    // value
    pkt->headerDelay += offset + header_delay;

    // note that we add the header delay to the existing value, and
    // align it to the crossbar clock

    // do a quick sanity check to ensure the timings are not being
    // ignored, note that this specific value may cause problems for
    // slower interconnects
    panic_if(pkt->headerDelay > sim_clock::as_int::us,
             "Encountered header delay exceeding 1 us\n");

    if (pkt->hasData()) {
        // the payloadDelay takes into account the relative time to
        // deliver the payload of the packet, after the header delay,
        // we take the maximum since the payload delay could already
        // be longer than what this parcitular crossbar enforces.
        pkt->payloadDelay = std::max<Tick>(pkt->payloadDelay,
                                           divCeil(pkt->getSize(), width) *
                                           clockPeriod());
    }

    // the payload delay is not paying for the clock offset as that is
    // already done using the header delay, and the payload delay is
    // also used to determine how long the crossbar layer is busy and
    // thus regulates throughput
}

template <typename SrcType, typename DstType>
BaseXBar::Layer<SrcType, DstType>::Layer(DstType& _port, BaseXBar& _xbar,
                                       const std::string& _name) :
    statistics::Group(&_xbar, _name.c_str()),
    port(_port), xbar(_xbar), _name(xbar.name() + "." + _name), state(IDLE),
    waitingForPeer(NULL), releaseEvent([this]{ releaseLayer(); }, name()),
    ADD_STAT(occupancy, statistics::units::Tick::get(), "Layer occupancy (ticks)"),
    ADD_STAT(utilization, statistics::units::Ratio::get(), "Layer utilization"),
    ADD_STAT(dataOccupancy, statistics::units::Tick::get(), "Layer data occupancy (ticks)"),
    ADD_STAT(headerOccupancy, statistics::units::Tick::get(), "Layer header-only occupancy (ticks) (without data packet)"),
    ADD_STAT(failOccupancy, statistics::units::Tick::get(), "Layer failed occupancy (ticks)"),
    ADD_STAT(writeReqHeaderOccupancy, statistics::units::Tick::get(), "Layer write request header-only occupancy (ticks)"),
    ADD_STAT(readReqHeaderOccupancy, statistics::units::Tick::get(), "Layer read request header-only occupancy (ticks)"),
    ADD_STAT(elseHeaderOccupancy, statistics::units::Tick::get(), "Layer other request header-only occupancy (ticks)"),
    ADD_STAT(writeRespHeaderOccupancy, statistics::units::Tick::get(), "Layer write response header-only occupancy (ticks)"),
    ADD_STAT(readRespHeaderOccupancy, statistics::units::Tick::get(), "Layer read response header-only occupancy (ticks)"),
    ADD_STAT(descOccupancy, statistics::units::Tick::get(), "Layer descriptor occupancy (ticks)"),
    ADD_STAT(mbufOccupancy, statistics::units::Tick::get(), "Layer mbuf occupancy (ticks)"),
    ADD_STAT(mmioOccupancy, statistics::units::Tick::get(), "Layer mmio occupancy (ticks)")
{
    occupancy
        .flags(statistics::nozero);

    utilization
        .precision(1)
        .flags(statistics::nozero);

    utilization = occupancy / simTicks;

    dataOccupancy
        .flags(statistics::nozero);
    
    headerOccupancy
        .flags(statistics::nozero);
    
    failOccupancy
        .flags(statistics::nozero);

    writeReqHeaderOccupancy
        .flags(statistics::nozero);
    
    readReqHeaderOccupancy
        .flags(statistics::nozero);
    
    elseHeaderOccupancy
        .flags(statistics::nozero);
    
    writeRespHeaderOccupancy
        .flags(statistics::nozero);

    readRespHeaderOccupancy
        .flags(statistics::nozero);
    
    descOccupancy
        .flags(statistics::nozero);
    
    mbufOccupancy
        .flags(statistics::nozero);
    
    mmioOccupancy
        .flags(statistics::nozero);
}

template <typename SrcType, typename DstType>
void BaseXBar::Layer<SrcType, DstType>::occupyLayer(Tick until)
{
    // ensure the state is busy at this point, as the layer should
    // transition from idle as soon as it has decided to forward the
    // packet to prevent any follow-on calls to sendTiming seeing an
    // unoccupied layer
    assert(state == BUSY);

    // until should never be 0 as express snoops never occupy the layer
    assert(until != 0);
    xbar.schedule(releaseEvent, until);

    // account for the occupied ticks
    occupancy += until - curTick();

    DPRINTF(BaseXBar, "The crossbar layer is now busy from tick %d to %d\n",
            curTick(), until);
}

template <typename SrcType, typename DstType>
bool
BaseXBar::Layer<SrcType, DstType>::tryTiming(SrcType* src_port)
{
    // if we are in the retry state, we will not see anything but the
    // retrying port (or in the case of the snoop ports the snoop
    // response port that mirrors the actual CPU-side port) as we leave
    // this state again in zero time if the peer does not immediately
    // call the layer when receiving the retry

    // first we see if the layer is busy, next we check if the
    // destination port is already engaged in a transaction waiting
    // for a retry from the peer
    if (state == BUSY || waitingForPeer != NULL) {
        // the port should not be waiting already
        assert(std::find(waitingForLayer.begin(), waitingForLayer.end(),
                         src_port) == waitingForLayer.end());

        // put the port at the end of the retry list waiting for the
        // layer to be freed up (and in the case of a busy peer, for
        // that transaction to go through, and then the layer to free
        // up)
        waitingForLayer.push_back(src_port);
        return false;
    }

    state = BUSY;

    return true;
}

template <typename SrcType, typename DstType>
void
BaseXBar::Layer<SrcType, DstType>::succeededTiming(Tick busy_time)
{
    // we should have gone from idle or retry to busy in the tryTiming
    // test
    assert(state == BUSY);

    // occupy the layer accordingly
    occupyLayer(busy_time);
}

template <typename SrcType, typename DstType>
void
BaseXBar::Layer<SrcType, DstType>::failedTiming(SrcType* src_port,
                                              Tick busy_time)
{
    // ensure no one got in between and tried to send something to
    // this port
    assert(waitingForPeer == NULL);

    // if the source port is the current retrying one or not, we have
    // failed in forwarding and should track that we are now waiting
    // for the peer to send a retry
    waitingForPeer = src_port;

    // we should have gone from idle or retry to busy in the tryTiming
    // test
    assert(state == BUSY);

    // occupy the bus accordingly
    occupyLayer(busy_time);
}

template <typename SrcType, typename DstType>
void
BaseXBar::Layer<SrcType, DstType>::releaseLayer()
{
    // releasing the bus means we should now be idle
    assert(state == BUSY);
    assert(!releaseEvent.scheduled());

    // update the state
    state = IDLE;

    // bus layer is now idle, so if someone is waiting we can retry
    if (!waitingForLayer.empty()) {
        // there is no point in sending a retry if someone is still
        // waiting for the peer
        if (waitingForPeer == NULL)
            retryWaiting();
    } else if (waitingForPeer == NULL && drainState() == DrainState::Draining) {
        DPRINTF(Drain, "Crossbar done draining, signaling drain manager\n");
        //If we weren't able to drain before, do it now.
        signalDrainDone();
    }
}

template <typename SrcType, typename DstType>
void
BaseXBar::Layer<SrcType, DstType>::retryWaiting()
{
    // this should never be called with no one waiting
    assert(!waitingForLayer.empty());

    // we always go to retrying from idle
    assert(state == IDLE);

    // update the state
    state = RETRY;

    // set the retrying port to the front of the retry list and pop it
    // off the list
    SrcType* retryingPort = waitingForLayer.front();
    waitingForLayer.pop_front();

    // tell the port to retry, which in some cases ends up calling the
    // layer again
    sendRetry(retryingPort);

    // If the layer is still in the retry state, sendTiming wasn't
    // called in zero time (e.g. the cache does this when a writeback
    // is squashed)
    if (state == RETRY) {
        // update the state to busy and reset the retrying port, we
        // have done our bit and sent the retry
        state = BUSY;

        // occupy the crossbar layer until the next clock edge
        occupyLayer(xbar.clockEdge());

        // JM - count retry as failed occupancy
        failOccupancy += xbar.clockEdge() - curTick();
    }
}

template <typename SrcType, typename DstType>
void
BaseXBar::Layer<SrcType, DstType>::recvRetry()
{
    // we should never get a retry without having failed to forward
    // something to this port
    assert(waitingForPeer != NULL);

    // add the port where the failed packet originated to the front of
    // the waiting ports for the layer, this allows us to call retry
    // on the port immediately if the crossbar layer is idle
    waitingForLayer.push_front(waitingForPeer);

    // we are no longer waiting for the peer
    waitingForPeer = NULL;

    // if the layer is idle, retry this port straight away, if we
    // are busy, then simply let the port wait for its turn
    if (state == IDLE) {
        retryWaiting();
    } else {
        assert(state == BUSY);
    }
}

PortID
BaseXBar::findPort(AddrRange addr_range)
{
    
    DPRINTF(AddrRanges, "In findPort: gotAllAddrRanges = %s\n",
            gotAllAddrRanges ? "true" : "false");

    
    DPRINTF(AddrRanges, "Looking for port matching addr_range: start = %#x, end = %#x, range = %s\n",
            addr_range.start(), addr_range.end(), addr_range.to_string());
    // we should never see any address lookups before we've got the
    // ranges of all connected CPU-side-port modules
    assert(gotAllAddrRanges);

    // Check the address map interval tree
    auto i = portMap.contains(addr_range);
    if (i != portMap.end()) {
        return i->second;
    }

    // Check if this matches the default range
    if (useDefaultRange) {
        if (addr_range.isSubset(defaultRange)) {
            DPRINTF(AddrRanges, "  found addr %s on default\n",
                    addr_range.to_string());
            return defaultPortID;
        }
    } else if (defaultPortID != InvalidPortID) {
        DPRINTF(AddrRanges, "Unable to find destination for %s, "
                "will use default port\n", addr_range.to_string());
        return defaultPortID;
    }

    // we should use the range for the default port and it did not
    // match, or the default port is not set
    fatal("Unable to find destination for %s on %s\n", addr_range.to_string(),
          name());
}

bool 
BaseXBar::isWithinAddrRanges(AddrRange addrRange) const
{
    assert(gotAllAddrRanges);

    // Check the address map interval tree
    if (portMap.contains(addrRange) != portMap.end()) {
        return true;
    } else {
        return false;
    }
}

/** Function called by the port when the crossbar is receiving a range change.*/
void
BaseXBar::recvRangeChange(PortID mem_side_port_id)
{
    DPRINTF(AddrRanges, "Received range change from cpu_side_ports %s\n",
            memSidePorts[mem_side_port_id]->getPeer());

    // remember that we got a range from this memory-side port and thus the
    // connected CPU-side-port module
    gotAddrRanges[mem_side_port_id] = true;

    if (memSidePorts[mem_side_port_id]->getPeer().name().find("job") != std::string::npos) {
        if (isL3XBar) {
            DTAJobPortID = mem_side_port_id;
            printf("DTAJobPortID: %d\n", DTAJobPortID);
        }
    }

    if (memSidePorts[mem_side_port_id]->getPeer().name().find("m2func") != std::string::npos) {
        if (isIOXBar) {
            M2funcPortID = mem_side_port_id;
            printf("M2funcPortID: %d\n", M2funcPortID);
        }
    }

    // update the global flag
    if (!gotAllAddrRanges) {
    gotAllAddrRanges = true;
    std::vector<bool>::const_iterator r = gotAddrRanges.begin();
    int idx = 0;
    while (gotAllAddrRanges && r != gotAddrRanges.end()) {
        if (!(*r)) {
            DPRINTF(AddrRanges, "[DBG] Port %d has not yet provided address range\n", idx);

            if (idx < memSidePorts.size() && memSidePorts[idx] && memSidePorts[idx]->isConnected()) {
                DPRINTF(AddrRanges, "[DBG] Port %d name: %s\n", idx,
                        memSidePorts[idx]->getPeer().name());


                const AddrRangeList& ranges = memSidePorts[idx]->getAddrRanges();
                if (ranges.empty()) {
                    DPRINTF(AddrRanges, "[DBG] --> No address range registered yet.\n");
                } else {
                    for (const auto& range : ranges) {
                        DPRINTF(AddrRanges, "[DBG] --> Existing Range: %s\n",
                                range.to_string());
                    }
                }
            } else {
                DPRINTF(AddrRanges, "[DBG] Port %d is not connected or invalid\n", idx);
            }
        }
        gotAllAddrRanges &= *r++;
        ++idx;
    }
    if (gotAllAddrRanges)
        DPRINTF(AddrRanges, "Got address ranges from all responders\n");
    else
        DPRINTF(AddrRanges, "Still waiting for some ports to provide address ranges\n");
}

    // note that we could get the range from the default port at any
    // point in time, and we cannot assume that the default range is
    // set before the other ones are, so we do additional checks once
    // all ranges are provided
    if (mem_side_port_id == defaultPortID) {
        // only update if we are indeed checking ranges for the
        // default port since the port might not have a valid range
        // otherwise
        if (useDefaultRange) {
            AddrRangeList ranges = memSidePorts[mem_side_port_id]->
                                   getAddrRanges();

            if (ranges.size() != 1)
                fatal("Crossbar %s may only have a single default range",
                      name());

            defaultRange = ranges.front();
        }
    } else {
        // the ports are allowed to update their address ranges
        // dynamically, so remove any existing entries
        if (gotAddrRanges[mem_side_port_id]) {
            for (auto p = portMap.begin(); p != portMap.end(); ) {
                if (p->second == mem_side_port_id)
                    // erasing invalidates the iterator, so advance it
                    // before the deletion takes place
                    portMap.erase(p++);
                else
                    p++;
            }
        }

        AddrRangeList ranges = memSidePorts[mem_side_port_id]->
                               getAddrRanges();
        
        std::vector<AddrRangeMap<PortID, 3>::iterator> conflicts;
        AddrRangeList adjustedRanges;

        // Sanity check for IOXBar with DTA enabled
        if (isIOXBar && enableDTA) {
            assert(M2funcPortID != InvalidPortID);
            // Check for all conflicting ranges
            for (const auto& r : ranges) {
                printf("Checking for conflicts for range: %s mem_side_port_id: %d M2funcPortID: %d\n", r.to_string().c_str(), mem_side_port_id, M2funcPortID);
                for (auto it = portMap.begin(); it != portMap.end(); ++it) {
                    if (it->first.intersects(r)) {
                        // First, check if the it's address range is same as the address range of the conflicts' already inserted
                        bool isSameRange = false;
                        for (auto conflict: conflicts) {
                            if (conflict->first == it->first) {
                                isSameRange = true;
                                break;
                            }
                        }
                        if (isSameRange) {
                            continue;
                        } else {
                            conflicts.push_back(it);
                            printf("Conflict range: %s conflict_id: %d\n", it->first.to_string().c_str(), it->second);
                        }
                    }
                }
            }
            printf("Conflicts size: %d\n", conflicts.size());
            if (conflicts.size() > 0) {
                if (mem_side_port_id == M2funcPortID) {
                    // In this case, conflicts should contain only one range
                    if (conflicts.size() > 1) {
                        for (auto conflict: conflicts) {
                            printf("Conflict range: %s conflict_id: %d\n", conflict->first.to_string(), conflict->second);
                        }
                        assert(conflicts.size() == 1);
                    }
                } else {
                    // In this case, conflicts should contain two ranges & both it's portID should be M2funcPortID
                    assert(conflicts.size() == 2);
                    for (auto conflict: conflicts) {
                        assert(conflict->second == M2funcPortID);
                        printf("Conflict range: %s conflict_id: %d\n", conflict->first.to_string().c_str(), conflict->second);
                    }
                }
            }
        }
        
        // If there are conflicts, adjust the range of port that is not M2funcPort
        //Caution! Below code assume that another conflict port (PIO) range fully includes M2funcPort range 
        if (conflicts.size() == 2) {
            // This case, new port is not M2funcPort
            // sort the conflicts by start address
            printf("%s has conflicts, mem_side_port_id: %d, conflict_id: %d, conflicts: ", name().c_str(), mem_side_port_id, conflicts[0]->second);
            for (auto conflict: conflicts) {
                printf("%s ID: %d, ", conflict->first.to_string().c_str(), conflict->second);
            }
            printf("\n");
            if (conflicts.front()->first.start() > conflicts.back()->first.start()) {
                std::swap(conflicts.front(), conflicts.back());
            }
            printf("After sorting, conflicts: ");
            for (auto conflict: conflicts) {
                printf("%s ID: %d, ", conflict->first.to_string().c_str(), conflict->second);
            }
            printf("\n");

            assert(ranges.size() == 1); // Another case is not handled yet
            AddrRange adjustedRange = ranges.front();
            Addr start = adjustedRange.start();
            for (const auto& conflict: conflicts) {
                if (conflict->first.start() > start) {
                    adjustedRanges.push_back(AddrRange(start, conflict->first.start()));
                }
                start = std::max(start, conflict->first.end());
            }
            if (start < adjustedRange.end()) {
                adjustedRanges.push_back(AddrRange(start, adjustedRange.end()));
            }
            // Add adjusted ranges
            for (const auto& r: adjustedRanges) {
                DPRINTF(AddrRanges, "Adding range %s for id %d\n",
                        r.to_string(), mem_side_port_id);
                printf("Adding range %s for id %d\n",
                        r.to_string().c_str(), mem_side_port_id);
                if (portMap.insert(r, mem_side_port_id) == portMap.end()) {
                    PortID conflict_id = portMap.intersects(r)->second;
                    fatal("%s has two ports responding within range "
                        "%s:\n\t%s\n\t%s\n",
                        name(),
                        r.to_string(),
                        memSidePorts[mem_side_port_id]->getPeer(),
                        memSidePorts[conflict_id]->getPeer());
                }
            }
        } else if (conflicts.size() == 1) {
            // This case, new port is M2funcPort
            AddrRange adjustedRange = conflicts.front()->first;
            Addr start = adjustedRange.start();
            // sort the ranges by start address
            std::vector<AddrRange> sorted_ranges;
            assert(ranges.size() == 2);
            if (ranges.front().start() > ranges.back().start()) {
                sorted_ranges.push_back(ranges.back());
                sorted_ranges.push_back(ranges.front());
            } else {
                sorted_ranges.push_back(ranges.front());
                sorted_ranges.push_back(ranges.back());
            }

            printf("%s has conflicts, mem_side_port_id: %d, conflict_id: %d, conflicts: ", name().c_str(), mem_side_port_id, conflicts[0]->second);
            for (auto conflict: conflicts) {
                printf("%s ID: %d, ", conflict->first.to_string().c_str(), conflict->second);
            }
            printf("\n");
            printf("Sorted Ranges: ");
            for (auto r: sorted_ranges) {
                printf("%s, ", r.to_string().c_str());
            }
            printf("\n");

            for (const auto& r: ranges) {
                if (r.start() > start) {
                    adjustedRanges.push_back(AddrRange(start, r.start()));
                }
                start = std::max(start, r.end());
            }
            if (start < adjustedRange.end()) {
                adjustedRanges.push_back(AddrRange(start, adjustedRange.end()));
            }
            // Remove original conflicting range & Add adjusted ranges
            PortID conflict_id = conflicts.front()->second;
            portMap.erase(conflicts.front());
            
            for (const auto& r: adjustedRanges) {
                DPRINTF(AddrRanges, "Adding range %s for conflict_id %d\n",
                        r.to_string(), conflict_id);
                printf("Adding range %s for conflict_id %d\n",
                        r.to_string().c_str(), conflict_id);
                if (portMap.insert(r, conflict_id) == portMap.end()) {
                    PortID another_conflict_id = portMap.intersects(r)->second;
                    fatal("%s has two ports responding within range "
                        "%s:\n\t%s\n\t%s\n",
                        name(),
                        r.to_string(),
                        memSidePorts[conflict_id]->getPeer(),
                        memSidePorts[another_conflict_id]->getPeer());
                }
            }
            // Add M2funcPort range
            for (const auto& r: ranges) {
                DPRINTF(AddrRanges, "Adding range %s for id %d\n",
                        r.to_string(), mem_side_port_id);
                printf("Adding range %s for id %d\n",
                        r.to_string().c_str(), mem_side_port_id);
                if (portMap.insert(r, mem_side_port_id) == portMap.end()) {
                    PortID conflict_id = portMap.intersects(r)->second;
                    fatal("%s has two ports responding within range "
                        "%s:\n\t%s\n\t%s\n",
                        name(),
                        r.to_string(),
                        memSidePorts[mem_side_port_id]->getPeer(),
                        memSidePorts[conflict_id]->getPeer());
                }
            }
        } else {
            for (const auto& r: ranges) {
                DPRINTF(AddrRanges, "Adding range %s for id %d\n",
                        r.to_string(), mem_side_port_id);
                printf("Adding range %s for id %d\n",
                        r.to_string().c_str(), mem_side_port_id);
                if (portMap.insert(r, mem_side_port_id) == portMap.end()) {
                    PortID conflict_id = portMap.intersects(r)->second;
                    fatal("%s has two ports responding within range "
                        "%s:\n\t%s\n\t%s\n",
                        name(),
                        r.to_string(),
                        memSidePorts[mem_side_port_id]->getPeer(),
                        memSidePorts[conflict_id]->getPeer());
                }
            }
        }
    }

    // if we have received ranges from all our neighbouring CPU-side-port
    // modules, go ahead and tell our connected memory-side-port modules in
    // turn, this effectively assumes a tree structure of the system
    if (gotAllAddrRanges) {
        DPRINTF(AddrRanges, "Aggregating address ranges\n");
        xbarRanges.clear();

        // start out with the default range
        if (useDefaultRange) {
            if (!gotAddrRanges[defaultPortID])
                fatal("Crossbar %s uses default range, but none provided",
                      name());

            xbarRanges.push_back(defaultRange);
            DPRINTF(AddrRanges, "-- Adding default %s\n",
                    defaultRange.to_string());
        }

        // merge all interleaved ranges and add any range that is not
        // a subset of the default range
        std::vector<AddrRange> intlv_ranges;
        for (const auto& r: portMap) {
            // if the range is interleaved then save it for now
            if (r.first.interleaved()) {
                // if we already got interleaved ranges that are not
                // part of the same range, then first do a merge
                // before we add the new one
                if (!intlv_ranges.empty() &&
                    !intlv_ranges.back().mergesWith(r.first)) {
                    DPRINTF(AddrRanges, "-- Merging range from %d ranges\n",
                            intlv_ranges.size());
                    AddrRange merged_range(intlv_ranges);
                    // next decide if we keep the merged range or not
                    if (!(useDefaultRange &&
                          merged_range.isSubset(defaultRange))) {
                        xbarRanges.push_back(merged_range);
                        DPRINTF(AddrRanges, "-- Adding merged range %s\n",
                                merged_range.to_string());
                    }
                    intlv_ranges.clear();
                }
                intlv_ranges.push_back(r.first);
            } else {
                // keep the current range if not a subset of the default
                if (!(useDefaultRange &&
                      r.first.isSubset(defaultRange))) {
                    xbarRanges.push_back(r.first);
                    DPRINTF(AddrRanges, "-- Adding range %s\n",
                            r.first.to_string());
                }
            }
        }

        // if there is still interleaved ranges waiting to be merged,
        // go ahead and do it
        if (!intlv_ranges.empty()) {
            DPRINTF(AddrRanges, "-- Merging range from %d ranges\n",
                    intlv_ranges.size());
            AddrRange merged_range(intlv_ranges);
            if (!(useDefaultRange && merged_range.isSubset(defaultRange))) {
                xbarRanges.push_back(merged_range);
                DPRINTF(AddrRanges, "-- Adding merged range %s\n",
                        merged_range.to_string());
            }
        }

        // also check that no range partially intersects with the
        // default range, this has to be done after all ranges are set
        // as there are no guarantees for when the default range is
        // update with respect to the other ones
        if (useDefaultRange) {
            for (const auto& r: xbarRanges) {
                // see if the new range is partially
                // overlapping the default range
                if (r.intersects(defaultRange) &&
                    !r.isSubset(defaultRange))
                    fatal("Range %s intersects the "                    \
                          "default range of %s but is not a "           \
                          "subset\n", r.to_string(), name());
            }
        }

        // tell all our neighbouring memory-side ports that our address
        // ranges have changed
        for (const auto& port: cpuSidePorts)
            port->sendRangeChange();
        
        if (isIOXBar && enableDTA) {
            // Print the portMap info
            for(auto p = portMap.begin(); p != portMap.end(); ++p) {
                printf("PortMap: %s %d\n", p->first.to_string().c_str(), p->second);
            }
        }
    }
}

AddrRangeList
BaseXBar::getAddrRanges() const
{
    // we should never be asked without first having sent a range
    // change, and the latter is only done once we have all the ranges
    // of the connected devices
    assert(gotAllAddrRanges);

    // at the moment, this never happens, as there are no cycles in
    // the range queries and no devices on the memory side of a crossbar
    // (CPU, cache, bridge etc) actually care about the ranges of the
    // ports they are connected to

    DPRINTF(AddrRanges, "Received address range request\n");

    return xbarRanges;
}

void
BaseXBar::regStats()
{
    ClockedObject::regStats();

    using namespace statistics;

    transDist
        .init(MemCmd::NUM_MEM_CMDS)
        .flags(nozero);

    // get the string representation of the commands
    for (int i = 0; i < MemCmd::NUM_MEM_CMDS; i++) {
        MemCmd cmd(i);
        const std::string &cstr = cmd.toString();
        transDist.subname(i, cstr);
    }

    pktCount
        .init(cpuSidePorts.size(), memSidePorts.size())
        .flags(total | nozero | nonan);

    pktSize
        .init(cpuSidePorts.size(), memSidePorts.size())
        .flags(total | nozero | nonan);

    // both the packet count and total size are two-dimensional
    // vectors, indexed by CPU-side port id and memory-side port id, thus the
    // neighbouring memory-side ports and CPU-side ports, they do not
    // differentiate what came from the memory-side ports and was forwarded to
    // the CPU-side ports (requests and snoop responses) and what came from
    // the CPU-side ports and was forwarded to the memory-side ports (responses
    // and snoop requests)
    for (int i = 0; i < cpuSidePorts.size(); i++) {
        pktCount.subname(i, cpuSidePorts[i]->getPeer().name());
        pktSize.subname(i, cpuSidePorts[i]->getPeer().name());
        for (int j = 0; j < memSidePorts.size(); j++) {
            pktCount.ysubname(j, memSidePorts[j]->getPeer().name());
            pktSize.ysubname(j, memSidePorts[j]->getPeer().name());
        }
    }
}

template <typename SrcType, typename DstType>
DrainState
BaseXBar::Layer<SrcType, DstType>::drain()
{
    //We should check that we're not "doing" anything, and that noone is
    //waiting. We might be idle but have someone waiting if the device we
    //contacted for a retry didn't actually retry.
    if (state != IDLE) {
        DPRINTF(Drain, "Crossbar not drained\n");
        return DrainState::Draining;
    } else {
        return DrainState::Drained;
    }
}

/**
 * Crossbar layer template instantiations. Could be removed with _impl.hh
 * file, but since there are only two given options (RequestPort and
 * ResponsePort) it seems a bit excessive at this point.
 */
template class BaseXBar::Layer<ResponsePort, RequestPort>;
template class BaseXBar::Layer<RequestPort, ResponsePort>;

} // namespace gem5
