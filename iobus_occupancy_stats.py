import os
import re
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

num_queues = 1
script = "dpdk-testpmd"
# num_queues = 2
# script = "dpdk-set" # have to set
base_dir="/home/jmhhh/Documents/CXL_network/gem5_dpdk_multiqueue/gem5-dpdk-setup/rundir"
base_folder = "20250415-dpdk-set-1core-l3-2port-4ns-sve-FF-B64-pDMA128-descDMA32-cxlmem-long-log"  # have to set
#"20250414-dpdk-set-1core-l3-2port-4ns-sve-16RXD-FF-B64-tbl-lcore-pDMA128-descDMA32-dmbfix-mmio-dma-1us-normal-spec-vecreg-256-2xunit-4LSU-predreg-sqfix-iocache-wait-long-log"
base_folder = os.path.join(base_dir, base_folder)

import re
import pandas as pd

def parse_layer_occupancy_from_stats(folder_path):
    stats_file = os.path.join(folder_path, "stats.txt")
    if os.path.exists(stats_file):
        # 필요한 패턴 정의
        pattern = re.compile(r'^system\.iobus\.(reqLayer18|respLayer2)\.(\w+)\s+([\d\.]+)')

        # 필요한 데이터 구조 초기화
        data = {
            'reqLayer18': {
                'occupancy': 0,
                'failOccupancy': 0,
                'descOccupancy': 0,
                'mbufOccupancy': 0,
                'mmioOccupancy': 0,
                'utilization': 0
            },
            'respLayer2': {
                'occupancy': 0,
                'failOccupancy': 0,
                'descOccupancy': 0,
                'mbufOccupancy': 0,
                'mmioOccupancy': 0,
                'utilization': 0
            }
        }

        # 로그 파싱
        with open(stats_file, 'r') as f:
            for line in f:
                match = pattern.match(line)
                if match:
                    layer, key, value = match.groups()
                    if key in data[layer]:
                        data[layer][key] = float(value)

        # 비율 계산 및 데이터 정리
        records = []
        for layer, values in data.items():
            occ = values['occupancy']
            fail = values['failOccupancy']
            desc = values['descOccupancy']
            mbuf = values['mbufOccupancy']
            mmio = values['mmioOccupancy']
            util = values['utilization']

            if occ > 0:
                records.append({
                    'layer': layer,
                    'utilization(%)': util * 100,
                    'fail(%)': fail / occ * 100,
                    'desc(%)': desc / occ * 100,
                    'mbuf(%)': mbuf / occ * 100,
                    'mmio(%)': mmio / occ * 100,
                })

        return pd.DataFrame(records)
    else:
        print(f"File {stats_file} does not exist.")
        return None

def print_occupancy_df(df):
    for _, row in df.iterrows():
        print(f"[{row['layer']}]")
        print(f"  - Utilization: {row['utilization(%)']:.2f}%")
        print(f"  - Occupancy breakdown:")
        print(f"      fail:  {row['fail(%)']:.2f}%")
        print(f"      desc:  {row['desc(%)']:.2f}%")
        print(f"      mbuf:  {row['mbuf(%)']:.2f}%")
        print(f"      mmio:  {row['mmio(%)']:.2f}%")


def parse_stats(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    drop_metrics = ["dmaDrops", "coreDrops", "txDrops", "unknownDrops", "rxdisabledDrops"]
    drops_per_metric = {}
    total_drops = 0
    total_rx_packets = 0

    experiment_done = False
    time = 0
    # Skip warmup stats, only use the last occurrence of each metric
    for line in lines:
        if "rxPackets" in line:
            total_rx_packets = int(line.split()[1]) 
            time += 1
        else:
            for metric in drop_metrics:
                if metric in line:
                    drops_per_metric[metric] = int(line.split()[1])
                    # print(f"{metric}: {drops_per_metric[metric]}")
    
    for metric, drops in drops_per_metric.items():
        total_drops += drops      
    
    if time != 1:
        total_drops = -1
        total_rx_packets = -1

    return total_drops, total_rx_packets

def get_drop_rate(folder_path):
    stats_file = os.path.join(folder_path, "stats.txt")
    if os.path.exists(stats_file):
        total_drops, total_rx_packets = parse_stats(stats_file)
        if total_rx_packets > 0:
            drop_rate = (total_drops / total_rx_packets) * 100
            return drop_rate
        else:
            print(f"for {folder_path}, total_rx_packets is 0")
            return None
    return None

def main(base_folder):
    data = {}
    # for packet_size in ["64", "128", "256", "512", "1024", "1518"]:
    # for packet_size in ["48", "64"]:
    for packet_size in ["64"]:
        data[packet_size] = {}
        print(f"base_folder: {base_folder}")

        for folder in os.listdir(base_folder):
            match = re.match(rf"1NIC-{num_queues}Qs-{packet_size}SIZE-(\d+)RATE-(\d+)Gbps-ddio-enabled-{script}.sh", folder)
            if match:
                # throughput = int(match.group(2))
                pps = int(match.group(1))
                throughput = pps * int(packet_size) * 8 / 1e9
                folder_path = os.path.join(base_folder, folder)
                drop_rates = get_drop_rate(folder_path)
                if drop_rates is not None:
                    occupancy_stat = parse_layer_occupancy_from_stats(folder_path)
                    print("===============================================================")
                    print(f"Packet Size: {packet_size} | PPS: {pps} | Throughput: {throughput} Gbps | Drop Rate: {drop_rates:.2f}%")
                    print_occupancy_df(occupancy_stat)
    
    
    

if __name__ == "__main__":
    main(base_folder)
