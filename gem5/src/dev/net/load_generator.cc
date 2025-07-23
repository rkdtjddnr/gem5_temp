#include "dev/net/load_generator.hh"
#include <inttypes.h>
#include "sim/sim_exit.hh"
#include <netinet/in.h>
#include "base/trace.hh"
#include "debug/LoadgenDebug.hh"
#include "debug/LoadgenLatency.hh"
#ifdef USE_ENSO
#include <netinet/ip.h>
#include <net/ethernet.h>
#endif

namespace gem5
{
    LoadGenerator::LoadGeneratorStats::LoadGeneratorStats(statistics::Group *parent)
        : statistics::Group(parent, "LoadGenerator"),
        ADD_STAT(sentPackets, statistics::units::Count::get(), "Number of Generated Packets"),
        ADD_STAT(recvPackets, statistics::units::Count::get(), "Number of Recieved Packets"),
        ADD_STAT(latency, statistics::units::Second::get(), "Distribution of Latency in ms")
        {
            sentPackets.precision(0);
            recvPackets.precision(0);
            latency.init(100);
        }


    LoadGenerator::LoadGenerator(const LoadGeneratorParams &p) : SimObject(p), loadgenId(p.loadgen_id), packetSize(p.packet_size), packetRate(p.packet_rate), 
    startTick(p.start_tick), stopTick(p.stop_tick), checkLossInterval(5000), incrementInterval(5e+8/(packetSize*8)),// Whats a good value for this?
    burstWidth(p.burst_width), burstGap(p.burst_gap), burstStartTick(0),
    lastRxCount(0), lastTxCount(0),
    sendPacketEvent([this]{sendPacket();}, name()), checkLossEvent([this]{checkLoss();}, name()), loadGeneratorStats(this)
    {
        if (p.mode == "Static")
            loadgenMode = Mode::Static;
        else if (p.mode == "Increment")
            loadgenMode = Mode::Increment;
        else if (p.mode == "Burst")
            loadgenMode = Mode::Burst;

        LoadGenerator::interface = new LoadGenInt("interface", this);
    }

    Tick LoadGenerator::frequency()
    {
        return (1e12/packetRate);
    }

    void LoadGenerator::startup()
    {
        if (curTick() > stopTick) return;
        
        if (curTick() > startTick)
            schedule(sendPacketEvent, curTick() + 1);
        else
            schedule(sendPacketEvent, startTick + 1);
    }

    Port & LoadGenerator::getPort(const std::string &if_name, PortID idx)
    {
        return *interface;
    }

    void LoadGenerator::buildPacket(EthPacketPtr ethpacket)
    {
        // Build Packet header
        // DSTMAC 6 | SRCMAC 6 | LENGTH 2 | DATA
        uint8_t dst_mac[6] = {0x00, 0x90, 0x00, 0x00, 0x00, 0x01 + loadgenId}; // Use paired NIC's MAC
        uint8_t src_mac[6] = {0x00, 0x80, 0x00, 0x00, 0x00, 0x01 + loadgenId};

        uint16_t size = ethpacket->length;

        #ifndef USE_ENSO
        if(1 != htons(1)) size = htons(size);

        uint8_t head[MACHeaderSize];
        memcpy(head, dst_mac, 6);
        memcpy(head + 6, src_mac, 6);
        memcpy(head + 12, &size, 2);
        memcpy(ethpacket->data, head, MACHeaderSize);
        uint64_t timeStamp = gem5::curTick();
        memcpy(&(ethpacket->data[MACHeaderSize]), &timeStamp, sizeof(uint64_t));
        #else
        const int mac_len = 14;
        const int ip_len = 20;
        uint8_t* pkt = ethpacket->data;

        // Ethernet Header
        memcpy(pkt, dst_mac, 6);
        memcpy(pkt + 6, src_mac, 6);
        uint16_t ether_type = htons(ETHERTYPE_IP);
        memcpy(pkt + 12, &ether_type, 2);

        // IP Header (minimal)
        uint8_t* ip_hdr = pkt + mac_len;
        memset(ip_hdr, 0, ip_len);  // zero out IP header
        ip_hdr[0] = 0x45;           // Version (4) + IHL (5)

        // Total Length
        uint16_t total_len = htons(ethpacket->length - mac_len);
        memcpy(ip_hdr + 2, &total_len, 2);  // offset 2: Total Length field

        // Timestamp payload
        uint64_t timestamp = gem5::curTick();
        memcpy(pkt + mac_len + ip_len, &timestamp, sizeof(uint64_t));
        #endif
        ethpacket->rxMadeTick = gem5::curTick();
    }

    void LoadGenerator::sendPacket()
    {
        if (loadGeneratorStats.sentPackets.value() == 0) {
            printf("Load Generator %d Started at %lu \n", loadgenId, curTick());
        }
        loadGeneratorStats.sentPackets++;
        lastTxCount++;

        EthPacketPtr txPacket = std::make_shared<EthPacketData>(packetSize);
        txPacket->length = packetSize;
        buildPacket(txPacket);
        // need to make ip header for Enso
        interface->sendPacket(txPacket);
        
        if (curTick() < stopTick)
        {
            if (loadgenMode == Mode::Increment)
            {
                if (lastTxCount == checkLossInterval)
                    // allow enough time for any in flight packets to be recieved
                    schedule(checkLossEvent, curTick() + 100000000);
                else
                    schedule(sendPacketEvent, curTick() + frequency());
            } else if (loadgenMode == Mode::Static)
            {
                schedule(sendPacketEvent, curTick() + frequency());
            } else if (loadgenMode == Mode::Burst)
            {
                if (curTick() - burstStartTick > burstWidth)
                    {
                        burstStartTick = curTick() + burstGap;
                        DPRINTF(LoadgenDebug, "Burst Ended, next Burst Starts at %lu \n", burstStartTick);
                        schedule(sendPacketEvent, burstStartTick);
                    }
                else 
                    schedule(sendPacketEvent, curTick() + frequency());
            }
        }
    }

    void LoadGenerator::checkLoss()
    {
        if (lastTxCount - lastRxCount < 10)
        {
            packetRate = packetRate + incrementInterval;
            schedule(sendPacketEvent, curTick() + frequency());
            DPRINTF(LoadgenDebug, "Rate Incremented, now sending packets at %u \n", packetRate);
            DPRINTF(LoadgenDebug, "Rx %lu, Tx %lu \n", lastRxCount, lastTxCount);
        }
        else
        {
            if ((packetRate - incrementInterval) < packetRate)
                packetRate = packetRate - incrementInterval;
            
            // add extra delay to prevent previouse loss from affecting results
            schedule(sendPacketEvent, curTick() + frequency() + 100000000);
            DPRINTF(LoadgenDebug, "Loss Detected, now sending packets at %u \n", packetRate);
            DPRINTF(LoadgenDebug, "Rx %lu, Tx %lu \n", lastRxCount, lastTxCount);
        }
            lastTxCount = 0;
            lastRxCount = 0;
    }

    void LoadGenerator::endTest()
    {
        exitSimLoop("m5_exit by loadgen End Simulator.", 0, curTick(), 0, true);
    }

    bool LoadGenerator::processRxPkt(EthPacketPtr pkt)
    {
        loadGeneratorStats.recvPackets++;
        lastRxCount++;

        uint64_t sendTick;
        #ifndef USE_ENSO
        memcpy(&sendTick, &(pkt->data[MACHeaderSize]), sizeof(uint64_t));
        #else
        memcpy(&sendTick, (pkt->data + 14 + 20), sizeof(uint64_t));
        #endif
        float delta = float((gem5::curTick() - sendTick))/10.0e8;
        loadGeneratorStats.latency.sample(delta);
        DPRINTF(LoadgenLatency, "Latency %f \n", delta);
        return true;
    }
}