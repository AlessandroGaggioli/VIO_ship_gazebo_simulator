#!/usr/bin/env bash
# Run all 4 (comp × 3dof) combinations × repeats on the motion_circle_speed bag.
# Record the bag first: cmd_vel_pub.py with config/circle_speed.yaml.

BAG="motion_circle_speed"
FILE_PREFIX="mot_circle_speed"
BASE_PATH="${BASE_PATH:-/home/alienware/Gaggioli/SimulationResults/motion_circle_speed}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"
# shellcheck source=lib_batch_runs.sh
source "${SCRIPT_DIR}/lib_batch_runs.sh"

run_all_combinations
