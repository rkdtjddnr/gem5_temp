#!/bin/bash

# 원하는 packet rate 리스트
RATES=(5000000 6000000)
RATES_DROP=(7100000 7200000 7300000 7400000 7500000 7600000 7700000 7800000 7900000)
RATES_DEBUG=(300000 500000)


# if you want taking checkpoint
# ./run_mica_all.sh checkpoint
if [ "$1" == "checkpoint" ]; then
    echo "✅ Creating checkpoint..."
    ./mica-ckp-dpdk-set-1core-1q-sve-16RXD-fastfree-B64-tbl-lcore-pDMA128-descDMA32-dmbfix-mmio-dma-1us-normal-spec-vecreg-256-2xunit-4LSUnit-predreg.sh \
        --take-checkpoint \
        --num-nics 1 \
        --num-queues 1 \
        --script mica_dpdk.sh \
        --l2-size 1MB \
        --freq 3GHz

    # (선택사항) 잠시 대기
    sleep 1
else
    echo "Skip creating checkpoint process..."
fi  

for rate in "${RATES[@]}"; do
    echo "Running MICA simulation with packet rate: $rate"

    # 실행
    ./mica-ckp-dpdk-set-1core-1q-sve-16RXD-fastfree-B64-tbl-lcore-pDMA128-descDMA32-dmbfix-mmio-dma-1us-normal-spec-vecreg-256-2xunit-4LSUnit-predreg.sh \
        --num-nics 1 \
        --num-queues 1 \
        --script mica_dpdk.sh \
        --packet-rate "$rate" \
        --l2-size 1MB \
        --freq 3GHz

    # 시뮬레이션이 실패했는지 체크
    if [ $? -ne 0 ]; then
        echo "Simulation failed at rate $rate"
        break
    else
        echo "✅ Done with rate $rate"
    fi

    # (선택사항) 잠시 대기
    sleep 1
done

echo "All simulations completed."