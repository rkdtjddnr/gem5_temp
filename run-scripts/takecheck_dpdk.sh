#!/bin/bash

set -e 

echo "=== Step 1: Running l2fwd-ckp-enso-macswap.sh ==="
./l2fwd-ckp-enso-macswap.sh \
  --take-checkpoint \
  --num-nics 1 \
  --num-queues 8 \
  --script dpdk-testpmd.sh \
  --freq 3GHz

echo "=== Step 1 complete ==="
echo

echo "=== Step 2: Running l2fwd-ckp-enso-touchfwd.sh ==="
./l2fwd-ckp-enso-touchfwd.sh \
  --take-checkpoint \
  --num-nics 1 \
  --num-queues 8 \
  --script dpdk-testpmd-touchfwd.sh \
  --freq 3GHz

echo "=== Step 2 complete ==="