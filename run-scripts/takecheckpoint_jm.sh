#!/bin/bash


# Take checkpoint
#2 core 2 queue
# ./l2fwd-ckp-dpdk-set-2core-2q.sh --take-checkpoint --num-nics 1 --num-queues 2 --script dpdk-set.sh --freq 3GHz

# 1 core 1 queue
# ./l2fwd-ckp-dpdk-set-1core-1q.sh --take-checkpoint --num-nics 1 --num-queues 1 --script dpdk-testpmd.sh --freq 3GHz


# M2func
# 1 core 1 queue
# ./l2fwd-ckp-dpdk-set-1core-1q-multi-port-4ns-m2func.sh --take-checkpoint --num-nics 1 --num-queues 1 --script dpdk-testpmd.sh --freq 3GHz



# build disk image for gem5
cd $GIT_ROOT/buildroot_new

# # echo current directory
# echo "Current directory: $(pwd)"

make clean

echo "After cleaning the buildroot directory"
echo "Start building the disk image for gem5"

make BR2_EXTERNAL=$GIT_ROOT/buildroot gem5_defconfig && make -j$(nproc)

echo "After building the disk image for gem5"

# # After building the disk image, copy the disk image to the gem5 resources-dpdk-m2func
cp $GIT_ROOT/buildroot_new/output/images/vmlinux $GIT_ROOT/resources-dpdk-m2func-dta-wo-printf/ && cp $GIT_ROOT/buildroot_new/output/images/rootfs.ext2 $GIT_ROOT/resources-dpdk-m2func-dta-wo-printf/

# Move to run directory
cd $GIT_ROOT/run-scripts

# echo current directory
echo "Current directory: $(pwd)"
echo "Start running the gem5 simulation"

# 1 core 1 queue
cd $GIT_ROOT/run-scripts
echo "Current directory: $(pwd)"
./l2fwd-ckp-dpdk-set-1core-1q-multi-port-4ns-m2func-dta-wo-printf.sh --take-checkpoint --num-nics 1 --num-queues 1 --script dpdk-testpmd.sh --freq 3GHz