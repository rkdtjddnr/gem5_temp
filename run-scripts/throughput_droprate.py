import os
import re
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

# num_queues = 1
# script = "dpdk-testpmd"
num_queues = 2
script = "dpdk-set" # have to set
base_folder = "/home/jmhhh/Documents/CXL_network/gem5_dpdk_multiqueue/gem5-dpdk-setup/rundir/dpdk-set-2core-l3-4port-4ns"  # have to set

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
            return None
    return None

def main(base_folder):
    data = {}
    for packet_size in ["64", "128", "256", "512", "1024", "1518"]:
        data[packet_size] = {}

        for folder in os.listdir(base_folder):
            match = re.match(rf"1NIC-{num_queues}Qs-{packet_size}SIZE-(\d+)RATE-(\d+)Gbps-ddio-enabled-{script}.sh", folder)
            if match:
                throughput = int(match.group(2))
                folder_path = os.path.join(base_folder, folder)
                drop_rate = get_drop_rate(folder_path)
                if drop_rate is not None:
                    data[packet_size][throughput] = drop_rate
                    print(f"Packet Size: {packet_size} | Throughput: {throughput} Gbps | Drop Rate: {drop_rate:.2f}%")
    
    # data["256"][32] = 0.0
    
    # Plotting
    plt.figure(figsize=(10, 6))
    for packet_size, values in data.items():
        throughputs = sorted(values.keys())
        drop_rates = [values[tp] for tp in throughputs]
        plt.plot(throughputs, drop_rates, marker='o', label=f'{packet_size}')
        plt.annotate(f'{drop_rates[0]:.2f}%', (throughputs[0], drop_rates[0]), textcoords="offset points", xytext=(0, 10), ha='center')
        plt.annotate(f'{drop_rates[-1]:.2f}%', (throughputs[-1], drop_rates[-1]), textcoords="offset points", xytext=(0, 10), ha='center')
    
    plt.xlabel('Throughput (Gbps)')
    plt.ylabel('Drop Rate (%)')
    plt.title('Drop Rate vs Throughput')
    plt.legend(title='Packet Size')
    plt.grid(True)
    # Set x-axis ticks to be 1 unit apart
    # Set x-axis value to be 10 unit apart
    plt.gca().xaxis.set_major_locator(ticker.MultipleLocator(10))
    plt.gca().xaxis.set_minor_locator(ticker.MultipleLocator(1))
    plt.gca().yaxis.set_minor_locator(ticker.MultipleLocator(1))
    plt.grid(which='minor', color='gray', linestyle=':', linewidth=0.5)
    
    plt.savefig('drop_rate_vs_throughput.png')

if __name__ == "__main__":
    main(base_folder)
