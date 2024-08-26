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
    [64]="$((8 * 1024 * 1024 * 1024)) $((10 * 1024 * 1024 * 1024))"
    [128]="$((16 * 1024 * 1024 * 1024)) $((20 * 1024 * 1024 * 1024))"
    [256]="$((32 * 1024 * 1024 * 1024)) $((37 * 1024 * 1024 * 1024))"
    [512]="$((53 * 1024 * 1024 * 1024)) $((60 * 1024 * 1024 * 1024))"
    [1024]="$((54 * 1024 * 1024 * 1024)) $((58 * 1024 * 1024 * 1024))"
    [1518]="$((55 * 1024 * 1024 * 1024)) $((59 * 1024 * 1024 * 1024))"
)

FREQ=3GHz

# Base command
base_command="./l2fwd-ckp-dpdk-set-2core-2q.sh --num-nics 1 --num-queues 2 --script dpdk-set.sh --freq $FREQ"

# Iterate over packet sizes and their rates
for size in "${!packet_configs[@]}"; do
    rates=(${packet_configs[$size]})
    for rate in "${rates[@]}"; do
        packet_rate=$(calculate_packet_rate $rate $size)
        command="$base_command --packet-rate $packet_rate --packet-size $size"
        echo "Executing: $command"
        eval "$command &"
    done
done

wait