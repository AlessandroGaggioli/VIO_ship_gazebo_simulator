#!/usr/bin/env bash
# Run all 4 (comp × 3dof) combinations × 5 repeats on the motion_trajectory bag.

BAG="motion_trajectory"
FILE_PREFIX="mot_trajectory"
BASE_PATH="${BASE_PATH:-/home/alienware/Gaggioli/SimulationResults/motion_trajectory}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
# shellcheck source=lib_batch_runs.sh
source "${SCRIPT_DIR}/lib_batch_runs.sh"

run_all_combinations
