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
    # [64]="$((6 * 1024 * 1024 * 1024)) $((7 * 1024 * 1024 * 1024)) $((8 * 1024 * 1024 * 1024)) $((10 * 1024 * 1024 * 1024))"
    # [128]="$((14 * 1024 * 1024 * 1024)) $((16 * 1024 * 1024 * 1024)) $((20 * 1024 * 1024 * 1024))"
    # [256]="$((28 * 1024 * 1024 * 1024)) $((30 * 1024 * 1024 * 1024)) $((32 * 1024 * 1024 * 1024)) $((37 * 1024 * 1024 * 1024))"
    # [512]="$((53 * 1024 * 1024 * 1024)) $((55 * 1024 * 1024 * 1024)) $((60 * 1024 * 1024 * 1024)) $((62 * 1024 * 1024 * 1024)) $((70 * 1024 * 1024 * 1024)) $((75 * 1024 * 1024 * 1024)) $((80 * 1024 * 1024 * 1024)) $((90 * 1024 * 1024 * 1024))"
    # [1024]="$((70 * 1024 * 1024 * 1024)) $((85 * 1024 * 1024 * 1024)) $((90 * 1024 * 1024 * 1024)) $((92 * 1024 * 1024 * 1024))"
    # [1518]="$((70 * 1024 * 1024 * 1024)) $((80 * 1024 * 1024 * 1024)) $((90 * 1024 * 1024 * 1024)) $((92 * 1024 * 1024 * 1024)) $((94 * 1024 * 1024 * 1024))"
    
    [64]="$((14 * 1024 * 1024 * 1024)) $((16 * 1024 * 1024 * 1024))"
    [128]="$((30 * 1024 * 1024 * 1024)) $((32 * 1024 * 1024 * 1024))"
    [256]="$((52 * 1024 * 1024 * 1024)) $((60 * 1024 * 1024 * 1024))"
    [512]="$((90 * 1024 * 1024 * 1024)) $((91 * 1024 * 1024 * 1024))"
    [1024]="$((90 * 1024 * 1024 * 1024)) $((92 * 1024 * 1024 * 1024))"
    [1518]="$((84 * 1024 * 1024 * 1024)) $((89 * 1024 * 1024 * 1024))"

)

FREQ=3GHz

# Base command
base_command="./l2fwd-ckp-dpdk-set-2core-2q-multi-port-4ns.sh --num-nics 1 --num-queues 2 --script dpdk-set.sh --freq $FREQ"

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