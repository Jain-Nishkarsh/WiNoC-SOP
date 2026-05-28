#!/bin/bash
# run_ocean_sweep.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PROTOCOLS=("token_packet" "fuzzy_token" "fuzzy_swj")
SEEDS=(1 2 3 4 5 6 7 8 9 10)

mkdir -p "${REPO_ROOT}/results/ocean_cp/"

for PROTO in "${PROTOCOLS[@]}"; do
    for SEED in "${SEEDS[@]}"; do
        CONFIG="${REPO_ROOT}/configs/custom_config/ocean_cp_trace_${PROTO}_8x8_64hubs.yaml"
        OUTPUT="${REPO_ROOT}/results/ocean_cp/${PROTO}_seed${SEED}.log"

        echo "Running: $PROTO seed=$SEED"

        cd "${REPO_ROOT}/bin" && ./noxim \
            -config "${CONFIG}" \
            -seed ${SEED} \
            > "${OUTPUT}" 2>&1

        if [ ! -f "$OUTPUT" ]; then
            echo "ERROR: No output for $PROTO seed $SEED"
        fi
    done
done

echo "Sweep complete. Results in results/ocean_cp/"
