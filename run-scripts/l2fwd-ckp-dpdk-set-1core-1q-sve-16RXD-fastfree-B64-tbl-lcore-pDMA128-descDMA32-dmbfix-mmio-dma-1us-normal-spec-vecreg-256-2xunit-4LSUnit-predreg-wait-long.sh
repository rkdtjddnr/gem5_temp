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
  --num-cpus=$(($num_nics+1)) --mem-type=DDR4_2400_16x4 --mem-channels=4 --mem-size=8192MB --script="$GUEST_SCRIPT_DIR/$GUEST_SCRIPT" \
  --num-nics="$num_nics" --num-loadgens="$num_nics" --num-queues="$num_queues" --num-dma-engines=128 --num-desc-dma-engines=32 \
  --checkpoint-dir="$CKPT_DIR" $CONFIGARGS

  "$GEM5_DIR/build/ARM/gem5.$GEM5TYPE" $DEBUG_FLAGS --outdir="$RUNDIR" \
  "$GEM5_DIR"/configs/example/fs.py --cpu-type=$CPUTYPE \
  --kernel="$RESOURCES/vmlinux" --disk="$RESOURCES/rootfs.ext2" --bootloader="$RESOURCES/boot.arm64" --root=/dev/sda \
  --num-cpus=$(($num_nics+1)) --mem-type=DDR4_2400_16x4 --mem-channels=4 --mem-size=8192MB --script="$GUEST_SCRIPT_DIR/$GUEST_SCRIPT" \
  --num-nics="$num_nics" --num-loadgens="$num_nics" --num-queues="$num_queues" --num-dma-engines=128 --num-desc-dma-engines=32 \
  --checkpoint-dir="$CKPT_DIR" $CONFIGARGS
}

if [[ -z "${GIT_ROOT}" ]]; then
  echo "Please export env var GIT_ROOT to point to the root of the CAL-DPDK-GEM5 repo"
  exit 1
fi

GEM5_DIR=${GIT_ROOT}/gem5
# RESOURCES=${GIT_ROOT}/resources
RESOURCES=${GIT_ROOT}/resources-dpdk-sve-16RXD-fastfree-B64-tbl-lcore
GUEST_SCRIPT_DIR=${GIT_ROOT}/guest-scripts

# parse command line arguments
TEMP=$(getopt -o 'h' --long take-checkpoint,num-nics:,script:,packet-rate:,packet-size:,loadgen-find-bw,freq:,num-queues:,help -n 'dpdk-loadgen' -- "$@")

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
  --take-checkpoint)
    checkpoint=1
    shift 1
    ;;
  --script)
    GUEST_SCRIPT="$2"
    shift 2
    ;;
  --packet-size)
    PACKET_SIZE="$2"
    shift 2
    ;;
  --packet-rate)
    PACKET_RATE="$2"
    shift 2
    ;;
  --loadgen-find-bw)
    LOADGENMODE="Increment"
    shift 1
    ;;
  --freq)
    Freq=$2
    shift 2
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

CKPT_DIR=${GIT_ROOT}/ckpts/"250408-"$num_nics"NIC"-$num_queues"Qs-SVE-16RXD-FF-B64-tbl-lcore-parallel-dmbfix"-$GUEST_SCRIPT
if [[ -z "$num_nics" ]]; then
  echo "Error: missing argument --num-nics" >&2
  usage
fi
if [[ -z "$num_queues" ]]; then
  echo "Error: missing argument --num-queues" >&2
  usage
fi

if [[ -n "$checkpoint" ]]; then
  # RUNDIR=${GIT_ROOT}/rundir/$num_nics"NIC-ckp"-$GUEST_SCRIPT
  RUNDIR=${GIT_ROOT}/rundir/250403-mac-SVE-16RXD-FF-B64-tbl-lcore-parallel-dmbfix-SingleQueue/$num_nics"NIC-"$num_queues"Qs-1core-ckp-"$GUEST_SCRIPT
  setup_dirs
  echo "Taking Checkpoint for NICs=$num_nics Queues=$num_queues" >&2
  GEM5TYPE="fast"
  # packet-size = 0 leads to segfault
  PACKET_SIZE=48
  CPUTYPE="AtomicSimpleCPU"
  CONFIGARGS="--max-checkpoints 3 --cpu-clock=$Freq --loadgen-start=2628842328231400"
  # CONFIGARGS="--max-checkpoints 1 -r 1 --cpu-clock=$Freq --loadgen-start=2628842328231400"
  run_simulation > $RUNDIR/simout
  exit 0
else
  if [[ -z "$PACKET_SIZE" ]]; then
    echo "Error: missing argument --packet_size" >&2
    usage
  fi

  if [[ -z "$PACKET_RATE" ]]; then
    echo "Error: missing argument --packet_rate" >&2
    usage
  fi
  ((RATE = PACKET_RATE * PACKET_SIZE * 8 / 1024 / 1024 / 1024))
  RUNDIR=${GIT_ROOT}/rundir/$(date +%Y%m%d)-dpdk-set-1core-l3-2port-4ns-sve-FF-B64-pDMA128-descDMA32-cxlio-long-log/$num_nics"NIC-"$num_queues"Qs-"$PACKET_SIZE"SIZE-"$PACKET_RATE"RATE-"$RATE"Gbps-ddio-enabled"-$GUEST_SCRIPT
  setup_dirs
# /dpdk-testpmd-freq-scaling-test
  echo "Running NICs=$num_nics at $RATE GBPS" >&2
  CPUTYPE="O3_ARM_v7a_3"
  GEM5TYPE="opt"
  # GEM5TYPE="debug"
  LOADGENMODE=${LOADGENMODE:-"Static"}
  # DEBUG_FLAGS="--debug-flags=EthernetDesc"
  # DEBUG_FLAGS="--debug-flags=O3CPUAll,Exec,CacheAll --debug-start=11339418155440 --debug-end=11340022599000"
  # DEBUG_FLAGS="--debug-flags=LoadgenDebug,EthernetDesc,EthernetDpdk" #--debug-start=33952834348" #EthernetAll,EthernetDesc,LoadgenDebug

  CONFIGARGS="$CACHE_CONFIG $CPU_CONFIG  --cpu-clock=$Freq -r 3 --loadgen-start=11539398155439 --rel-max-tick=400010000000 --packet-rate=$PACKET_RATE --packet-size=$PACKET_SIZE --loadgen-mode=$LOADGENMODE \
  --warmup-dpdk 200000000000"

  # CONFIGARGS="$CACHE_CONFIG $CPU_CONFIG  --cpu-clock=$Freq -r 3 --loadgen-start=11339418155439 --rel-max-tick=400010000000 --packet-rate=$PACKET_RATE --packet-size=$PACKET_SIZE --loadgen-mode=$LOADGENMODE \
  # --warmup-dpdk 20000000"
  run_simulation > ${RUNDIR}/simout
  exit
fi
#loadgen-start=26488422623982