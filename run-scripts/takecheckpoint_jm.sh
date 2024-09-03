#!/bin/bash


# Take checkpoint
#2 core 2 queue
# ./l2fwd-ckp-dpdk-set-2core-2q.sh --take-checkpoint --num-nics 1 --num-queues 2 --script dpdk-set.sh --freq 3GHz

# 1 core 1 queue
./l2fwd-ckp-dpdk-set-1core-1q.sh --take-checkpoint --num-nics 1 --num-queues 1 --script dpdk-testpmd.sh --freq 3GHz



