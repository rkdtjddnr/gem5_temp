import os
import re
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

num_queues = 1
script = "dpdk-testpmd"
# num_queues = 2
# script = "dpdk-set" # have to set
base_folder = "/home/jmhhh/Documents/CXL_network/gem5_dpdk_multiqueue/gem5-dpdk-setup/rundir/dpdk-set-1core-l3-2port-4ns-breakdown"  # have to set

def parse_stats(file_path):
    with open(file_path, 'r') as file:
        stat_content = file.read()
    
    regex_patterns = {
        'txDMABytes': r"system\.nics\.EtherDevice\.txDMABytes\s+(\d+)",
        'rxDMABytes': r"system\.nics\.EtherDevice\.rxDMABytes\s+(\d+)",
        'rxDescFetchBytes': r"system\.nics\.EtherDevice\.rxDescFetchBytes\s+(\d+)",
        'txDescFetchBytes': r"system\.nics\.EtherDevice\.txDescFetchBytes\s+(\d+)",
        'rxDescWBBytes': r"system\.nics\.EtherDevice\.rxDescWBBytes\s+(\d+)",
        'txDescWBBytes': r"system\.nics\.EtherDevice\.txDescWBBytes\s+(\d+)",
        'rxTailWriteBytes': r"system\.nics\.EtherDevice\.rxTailWriteBytes\s+(\d+)",
        'txTailWriteBytes': r"system\.nics\.EtherDevice\.txTailWriteBytes\s+(\d+)"
    }
    
    stats = {}
    for key, pattern in regex_patterns.items():
        match = re.search(pattern, stat_content)
        if match:
            stats[key] = int(match.group(1))
    
    return stats

def calculate_breakdown(stats):
    total_rx = stats['rxDMABytes'] + stats['rxDescFetchBytes'] + stats['rxDescWBBytes'] + stats['rxTailWriteBytes']
    total_tx = stats['txDMABytes'] + stats['txDescFetchBytes'] + stats['txDescWBBytes'] + stats['txTailWriteBytes']

    rx_breakdown = {
        'Descriptor Fetch': stats['rxDescFetchBytes'],
        'Descriptor Writeback': stats['rxDescWBBytes'],
        'Tail Write': stats['rxTailWriteBytes'],
        'Payload': stats['rxDMABytes']
    }

    tx_breakdown = {
        'Descriptor Fetch': stats['txDescFetchBytes'],
        'Descriptor Writeback': stats['txDescWBBytes'],
        'Tail Write': stats['txTailWriteBytes'],
        'Payload': stats['txDMABytes']
    }

    rx_breakdown_percent = {key: (value / total_rx) * 100 for key, value in rx_breakdown.items()}
    tx_breakdown_percent = {key: (value / total_tx) * 100 for key, value in tx_breakdown.items()}

    return rx_breakdown_percent, tx_breakdown_percent

def plot_breakdown(rx_breakdown_percent, tx_breakdown_percent, packet_size):
    fig, ax = plt.subplots(2, 1, figsize=(8, 6))

    # RX Breakdown
    ax[0].bar(rx_breakdown_percent.keys(), rx_breakdown_percent.values(), color='skyblue')
    ax[0].set_title(f"RX Traffic Breakdown ({packet_size}B)")
    ax[0].set_ylabel("Percentage (%)")
    
    # Adding percentage labels to RX bars
    for i, (key, value) in enumerate(rx_breakdown_percent.items()):
        ax[0].text(i, value + 1, f'{value:.2f}%', ha='center', va='bottom')

    # TX Breakdown
    ax[1].bar(tx_breakdown_percent.keys(), tx_breakdown_percent.values(), color='lightcoral')
    ax[1].set_title(f"TX Traffic Breakdown ({packet_size}B)")
    ax[1].set_ylabel("Percentage (%)")
    
    # Adding percentage labels to TX bars
    for i, (key, value) in enumerate(tx_breakdown_percent.items()):
        ax[1].text(i, value + 1, f'{value:.2f}%', ha='center', va='bottom')

    plt.tight_layout()
    plt.savefig(f"traffic_breakdown_{packet_size}B_{num_queues}Qs_{script}.png")


def make_plot(folder_path, packet_size):
    stats_file = os.path.join(folder_path, "stats.txt")
    if os.path.exists(stats_file):
        stats = parse_stats(stats_file)
        rx_breakdown_percent, tx_breakdown_percent = calculate_breakdown(stats)
        plot_breakdown(rx_breakdown_percent, tx_breakdown_percent, packet_size)
    else:
        print(f"Stats file not found: {stats_file}")

def main(base_folder):
    for packet_size in ["64", "128", "256", "512", "1024", "1518"]:
        for folder in os.listdir(base_folder):
            match = re.match(rf"1NIC-{num_queues}Qs-{packet_size}SIZE-(\d+)RATE-(\d+)Gbps-ddio-enabled-{script}.sh", folder)
            if match:
                folder_path = os.path.join(base_folder, folder)
                make_plot(folder_path, packet_size)
                print(f"Make plot for {folder}")

if __name__ == "__main__":
    main(base_folder)
