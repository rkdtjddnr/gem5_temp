#!/bin/bash

# Function to calculate packet rate
calculate_packet_rate() {
    local rate=$1
    local size=$2
    echo $((rate / size / 8))
}

# Array of packet sizes and their corresponding rates (in bytes)
declare -A packet_configs
packet_configs=(
    # 16Gbps, 48Gbps, 64Gbps, 80Gbps, 96Gbps
    [64]="$((2 * 1024 * 1024 * 1024)) $((6 * 1024 * 1024 * 1024)) $((8 * 1024 * 1024 * 1024)) $((10 * 1024 * 1024 * 1024)) $((12 * 1024 * 1024 * 1024))"
)

FREQ=3GHz

# Base command
base_command="./l2fwd-ckp-enso.sh --num-nics 1 --num-queues 8 --script dpdk-testpmd.sh --freq 3GHz"

# Number of processors available
nprocs=$(nproc)

# Generate all commands and store them in a list
commands=()

for size in "${!packet_configs[@]}"; do
    rates=(${packet_configs[$size]})
    for rate in "${rates[@]}"; do
        packet_rate=$(calculate_packet_rate $rate $size)
        command="$base_command --packet-rate $packet_rate --packet-size $size"
        commands+=("$command")
    done
done

# Execute the commands in parallel with respect to the number of processors
printf "%s\n" "${commands[@]}" | xargs -I {} -n 1 -P "$nprocs" bash -c 'echo "Executing: {}"; eval "{}"'
# printf "%s\n" "${commands[@]}" | xargs -I {} -n 1 -P 1 bash -c 'echo "Executing: {}";'