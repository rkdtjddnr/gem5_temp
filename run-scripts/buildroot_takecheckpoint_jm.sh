#!/bin/bash

export BR2_EXTERNAL_DPDK_GEM5_PATH=$GIT_ROOT/buildroot

# build disk image for gem5
cd $GIT_ROOT/buildroot_new

# # echo current directory
echo "Current directory: $(pwd)"

make clean

echo "After cleaning the buildroot directory"
echo "Start building the disk image for gem5"

make BR2_EXTERNAL=$GIT_ROOT/buildroot gem5_defconfig && make -j$(nproc)

echo "After building the disk image for gem5"

# After building the disk image, copy the disk image to the gem5 resources-dpdk-m2func
cp $GIT_ROOT/buildroot_new/output/images/vmlinux $GIT_ROOT/resources-dpdk-m2func-wo-printf/ && cp $GIT_ROOT/buildroot_new/output/images/rootfs.ext2 $GIT_ROOT/resources-dpdk-m2func-wo-printf/

# Move to run directory
cd $GIT_ROOT/run-scripts

# echo current directory
echo "Current directory: $(pwd)"
echo "Start running the gem5 simulation"

# Take checkpoint
# 1 core 1 queue
./l2fwd-ckp-dpdk-set-1core-1q-multi-port-4ns-m2func.sh --take-checkpoint --num-nics 1 --num-queues 1 --script dpdk-testpmd.sh --freq 3GHz
