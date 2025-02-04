#!/bin/bash


RESOURCE_DIR='resources-dpdk-polling-test'

# build disk image for gem5
cd $GIT_ROOT/buildroot_new

# # # echo current directory
echo "Current directory: $(pwd)"

make clean

echo "After cleaning the buildroot directory"
echo "Start building the disk image for gem5"

make BR2_EXTERNAL=$GIT_ROOT/buildroot gem5_defconfig && make -j$(nproc)

echo "After building the disk image for gem5"

# After building the disk image, copy the disk image to the gem5 resources-dpdk-m2func
cp $GIT_ROOT/buildroot_new/output/images/vmlinux $GIT_ROOT/$RESOURCE_DIR/ && cp $GIT_ROOT/buildroot_new/output/images/rootfs.ext2 $GIT_ROOT/$RESOURCE_DIR/

