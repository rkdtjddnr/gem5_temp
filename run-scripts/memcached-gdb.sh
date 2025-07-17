#!/bin/bash
#wbWidth=4 causes error when you run
CACHE_CONFIG="--caches --l2cache --l3cache --l3_multiport --l3_size 16MB --l3_assoc 16 --ddio-enabled --l1i_size=64kB --l1i_assoc=8 \
--l1d_size=64kB --l1d_assoc=8 --l2_size=1MB --l2_assoc=8 --cacheline_size=64 --l3_cpu_side_ports_connection_count 2" 
CPU_CONFIG="--param=system.l3.mshrs=256 --param=system.cpu[0:4].l2cache.mshrs=46 --param=system.cpu[0:4].dcache.mshrs=20 --param=system.cpu[0:4].icache.mshrs=20 \
  --param=system.l3.tgts_per_mshr=12 --param=system.cpu[0:4].l2cache.tgts_per_mshr=12 --param=system.cpu[0:4].dcache.tgts_per_mshr=20 --param=system.cpu[0:4].icache.tgts_per_mshr=20 \
  --param=system.l3.data_latency=30 --param=system.l3.response_latency=30 --param=system.l3.tag_latency=30 --param=system.l3.ddio_way_part=4 \
  --param=system.iocache.mshrs=96 --param=system.iocache.tgts_per_mshr=20 \
  --param=system.switch_cpus[0:4].decodeWidth=8 --param=system.l3.is_llc=True \
  --param=system.switch_cpus[0:4].numROBEntries=512 --param=system.switch_cpus[0:4].numIQEntries=512 \
  --param=system.switch_cpus[0:4].LQEntries=248 --param=system.switch_cpus[0:4].SQEntries=248 \
  --param=system.switch_cpus[0:4].numPhysIntRegs=256 --param=system.switch_cpus[0:4].numPhysFloatRegs=256 --param=system.switch_cpus[0:4].numPhysVecRegs=256 --param=system.switch_cpus[0:4].numPhysVecPredRegs=64 \
  --param=system.switch_cpus[0:4].branchPred.BTBEntries=16384 --param=system.switch_cpus[0:4].issueWidth=8 \
  --param=system.switch_cpus[0:4].commitWidth=8 --param=system.switch_cpus[0:4].dispatchWidth=8 \
  --param=system.switch_cpus[0:4].fetchWidth=8 --param=system.switch_cpus[0:4].wbWidth=8 \
  --param=system.switch_cpus[0:4].squashWidth=8 --param=system.switch_cpus[0:4].renameWidth=8 \
  --param=system.iobus.is_ioxbar=True --param=system.iobus.model_pcie_1us=True --param=system.bridge.model_pcie_1us=True"

function usage {
  echo "Usage: $0 --num-nics <num_nics> [--script <script>] [--packet-rate <packet_rate>] [--packet-size <packet_size>] [--loadgen-find-bw] [--take-checkpoint] [-h|--help]"
  echo "  --num-nics <num_nics> : number of NICs to use"
  echo "  --num-queues <num_queues> : number of queues per NIC"
  echo "  --script <script> : guest script to run"
  echo "  --packet-rate <packet_rate> : packet rate in PPS"
  echo "  --packet-size <packet_size> : packet size in bytes"
  echo "  --loadgen-find-bw : run loadgen in find bandwidth mode"
  echo "  --take-checkpoint : take checkpoint after running"
  echo "  -h --help : print this message"
  exit 1
}

function setup_dirs {
  mkdir -p "$CKPT_DIR"
  mkdir -p "$RUNDIR"
}

function run_simulation {
  # echo running command
  echo "Running command is !!!!!!!"
  echo "$GEM5_DIR/build/ARM/gem5.$GEM5TYPE" $DEBUG_FLAGS --outdir="$RUNDIR" \
  "$GEM5_DIR"/configs/example/fs.py --cpu-type=$CPUTYPE \
  --kernel="$RESOURCES/vmlinux" --disk="$RESOURCES/rootfs.ext2" --bootloader="$RESOURCES/boot.arm64" --root=/dev/sda \
  --num-cpus=$(($num_nics)) --mem-type=DDR4_2400_16x4 --mem-channels=4 --mem-size=65536MB --script="$GUEST_SCRIPT_DIR/$GUEST_SCRIPT" \
  --num-nics="$num_nics" --num-loadgens="$num_nics" --num-queues="$num_queues" --num-dma-engines=128 --num-desc-dma-engines=32 \
  --checkpoint-dir="$CKPT_DIR" $CONFIGARGS

  "$GEM5_DIR/build/ARM/gem5.$GEM5TYPE" $DEBUG_FLAGS --outdir="$RUNDIR" \
  "$GEM5_DIR"/configs/example/fs.py --cpu-type=$CPUTYPE \
  --kernel="$RESOURCES/vmlinux" --disk="$RESOURCES/rootfs.ext2" --bootloader="$RESOURCES/boot.arm64" --root=/dev/sda \
  --num-cpus=$(($num_nics)) --mem-type=DDR4_2400_16x4 --mem-channels=4 --mem-size=65536MB --script="$GUEST_SCRIPT_DIR/$GUEST_SCRIPT" \
  --num-nics="$num_nics" --num-loadgens="$num_nics" --num-queues="$num_queues" --num-dma-engines=128 --num-desc-dma-engines=32 \
  --checkpoint-dir="$CKPT_DIR" $CONFIGARGS
}

function run_gdb_simulation {
  gdb --args "$GEM5_DIR/build/ARM/gem5.$GEM5TYPE" $DEBUG_FLAGS --outdir="$RUNDIR" \
  "$GEM5_DIR"/configs/example/fs.py --cpu-type=$CPUTYPE \
  --kernel="$RESOURCES/vmlinux" --disk="$RESOURCES/rootfs.ext2" --bootloader="$RESOURCES/boot.arm64" --root=/dev/sda \
  --num-cpus=$(($num_nics)) --mem-type=DDR4_2400_16x4 --mem-channels=4 --mem-size=65536MB --script="$GUEST_SCRIPT_DIR/$GUEST_SCRIPT" \
  --num-nics="$num_nics" --num-loadgens="$num_nics" --num-queues="$num_queues" --num-dma-engines=128 --num-desc-dma-engines=32 \
  --checkpoint-dir="$CKPT_DIR" $CONFIGARGS
}

if [[ -z "${GIT_ROOT}" ]]; then
  echo "Please export env var GIT_ROOT to point to the root of the CAL-DPDK-GEM5 repo"
  exit 1
fi

GEM5_DIR=${GIT_ROOT}/gem5
# RESOURCES=${GIT_ROOT}/resources
RESOURCES=${GIT_ROOT}/resources-dpdk
GUEST_SCRIPT_DIR=${GIT_ROOT}/guest-scripts

# parse command line arguments
TEMP=$(getopt -o 'h' --long freq:,take-checkpoint,num-nics:,cpu-types:,l2-size:,script:,packet-rate:,num-queues:,loadgen-find-bw,help -n 'dpdk-loadgen' -- "$@")


# check for parsing errors
if [ $? != 0 ]; then
  echo "Error: unable to parse command line arguments" >&2
  exit 1
fi

eval set -- "$TEMP"

while true; do
  case "$1" in
  --num-nics)
    num_nics="$2"
    shift 2
    ;;
  --num-queues)
    num_queues="$2"
    shift 2
    ;;
  --l2-size)
    L2_SIZE="$2"
    shift 2
    ;;
  --cpu-types)
    CPUTYPE="$2"
    shift 2
    ;;
  --freq)
    FREQ="$2"
    shift 2
    ;;
  --take-checkpoint)
    checkpoint=1
    shift 1
    ;;
  --script)
    GUEST_SCRIPT="$2"
    shift 2
    ;;
  --packet-rate)
    PACKET_RATE="$2"
    shift 2
    ;;
  --loadgen-find-bw)
    LOADGENREPLAYMODE="ReplayAndAdjustThroughput"
    shift 1
    ;;
  -h | --help)
    usage
    ;;
  --)
    shift
    break
    ;;
  *) break ;;
  esac
done

# CKPT_DIR=${GIT_ROOT}/ckpts/$num_nics"NIC"-$GUEST_SCRIPT
CKPT_DIR=${GIT_ROOT}/ckpts/ckpts-with-new-vmlinux/$num_nics"NIC"-$num_queues"Queues"-$GUEST_SCRIPT
if [[ -z "$num_nics" ]]; then
  echo "Error: missing argument --num-nics" >&2
  usage
fi
if [[ -z "$num_queues" ]]; then
  echo "Error: missing argument --num-queues" >&2
  usage
fi

if [[ -n "$checkpoint" ]]; then
  # RUNDIR=${GIT_ROOT}/rundir/$num_nics"NIC-ckp-"$GUEST_SCRIPT
  RUNDIR=${GIT_ROOT}/rundir/ISPASS-2024-memcached-exps/$num_nics"NIC"-$num_queues"Queues"-"ckp"-$GUEST_SCRIPT
  setup_dirs
  echo "Taking Checkpoint for NICs=$num_nics Queues=$num_queues">&2
  GEM5TYPE="fast"
  #GEM5TYPE="opt"
  # DEBUG_FLAGS="--debug-flags=LoadgenDebug"
  PORT=11211
  CPUTYPE="AtomicSimpleCPU"
  #CPUTYPE="O3_ARM_v7a_3"
  PACKET_RATE=5000
  LOADGENREPLAYMODE=ConstThroughput
  PCAP_FILENAME="../resources-dpdk/warmup-dpdk-5k.pcap"
  #CONFIGARGS="-r 2 --max-checkpoints 1 --checkpoint-at-end --cpu-clock=$FREQ --l2_size=$L2_SIZE $CACHE_CONFIG $CPU_CONFIG --loadgen-start=6654416674681 --loadgen-type=Pcap --loadgen-stack=DPDKStack --loadgen_pcap_filename=$PCAP_FILENAME --packet-rate=$PACKET_RATE --loadgen-replymode=$LOADGENREPLAYMODE --loadgen-port-filter=$PORT"
  CONFIGARGS="--max-checkpoints 2 --cpu-clock=$FREQ --l2_size=$L2_SIZE $CACHE_CONFIG --loadgen-start=600011771117451658 --loadgen-type=Pcap --loadgen-stack=DPDKStack --loadgen_pcap_filename=$PCAP_FILENAME --packet-rate=$PACKET_RATE --loadgen-replymode=$LOADGENREPLAYMODE --loadgen-port-filter=$PORT"
  #run_simulation > ${RUNDIR}/simout
  run_gdb_simulation
  exit 0
else
  if [[ -z "$PACKET_RATE" ]]; then
    echo "Error: missing argument --packet_rate" >&2
    usage
  fi
  PORT=11211
  #PCAP_FILENAME="../resources-dpdk/request-dpdk-10k.pcap"
  PCAP_FILENAME="../resources-dpdk/replay_trace/memcached_8_8_50000_1_get_50.pcap"
  # PCAP_FILENAME="../resources/request-dpdk-trace.pcap"
  ((INCR_INTERVAL = PACKET_RATE / 10)) 
  LOADGENREPLAYMODE=${LOADGENREPLAYMODE:-"ConstThroughput"}
  #RUNDIR=${GIT_ROOT}/rundir/memcached-dpdk-findbw-cpu-type-exp/$num_nics"NIC"-$GUEST_SCRIPT-$FREQ"-ddio-enabled"-$PACKET_RATE
  RUNDIR=${GIT_ROOT}/rundir/memcached-dpdk-findbw-cpu-type-exp/$num_nics"NIC"-$num_queues"Queues"-"ckp"-$GUEST_SCRIPT-$FREQ"-ddio-enabled"-$PACKET_RATE
  setup_dirs
  CPUTYPE="O3_ARM_v7a_3" # just because DerivO3CPU is too slow sometimes
  GEM5TYPE="opt"
  # LOADGENREPLAYMODE=${LOADGENREPLAYMODE:-"ConstThroughput"}
  DEBUG_FLAGS="" #"--debug-flags=LoadgenDebug"
  CONFIGARGS="--l2_size=$L2_SIZE $CACHE_CONFIG $CPU_CONFIG -r 2 --cpu-clock=$FREQ --loadgen-type=Pcap --loadgen-stack=DPDKStack \
  --loadgen_pcap_filename=$PCAP_FILENAME --loadgen-start=7653557427454 --packet-rate=$PACKET_RATE \
  --loadgen-replymode=$LOADGENREPLAYMODE --loadgen-port-filter=$PORT --loadgen-increment-interva=$INCR_INTERVAL"
  #run_simulation > ${RUNDIR}/simout
  run_gdb_simulation
  exit
fi


# safety
# 32263966421416