#!/bin/bash

# 원하는 packet rate 리스트
RATE=100000
NUMS=(0 1 2 3 4)
OUT_DIR="../rundir/mica-dpdk-findbw-cpu-type-exp/parse_stat"
TARGET_DIR="../rundir/mica-dpdk-findbw-cpu-type-exp/1NIC-1Queues-ckp-mica_dpdk.sh-3GHz-ddio-enabled-$RATE"
TARGET="system.terminal"
PCAP_DIR="../resources-dpdk/replay_trace/mica_16k"

# if you want taking checkpoint
# ./run_mica_auto.sh checkpoint
if [ "$1" == "checkpoint" ]; then
    echo "✅ Creating checkpoint..."
    ./mica_dpdk.sh \
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

for num in "${NUMS[@]}"; do
    FILENAME="mica_16k_$num.pcap"
    PCAP_FILENAME="$PCAP_DIR/$FILENAME"
    echo "Running MICA simulation with $FILENAME"

    ./mica_dpdk.sh \
        --num-nics 1 \
        --num-queues 1 \
        --script mica_dpdk.sh \
        --packet-rate "$RATE" \
        --l2-size 1MB \
        --freq 3GHz \
        --pcap-file $PCAP_FILENAME

     # 시뮬레이션이 실패했는지 체크
    if [ $? -ne 0 ]; then
        echo "Simulation failed"
        break
    else
         # 여기서 system.terminal 이름 바꾸고 폴더 옮기는 작업 수행
        # filename = mica_4k_0_100000.txt, mica_4k_0_300000.txt ...
        mv "$TARGET_DIR/$TARGET" "$OUT_DIR/mica_16k_${num}_${RATE}.txt"
    fi

    # (선택사항) 잠시 대기
    sleep 1

done

echo "All simulations completed."