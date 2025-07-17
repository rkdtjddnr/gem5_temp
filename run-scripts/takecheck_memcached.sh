
#run
./memcached-ckp-dpdk-set-1core-1q-sve-16RXD-fastfree-B64-tbl-lcore-pDMA128-descDMA32-dmbfix-mmio-dma-1us-normal-spec-vecreg-256-2xunit-4LSUnit-predreg.sh --take-checkpoint --num-nics 1 --num-queues 12 --script memcached_dpdk.sh --l2-size 1MB --freq 3GHz
#./memcached-gdb.sh --take-checkpoint --num-nics 1 --num-queues 12 --script memcached_dpdk.sh --l2-size 1MB --freq 3GHz