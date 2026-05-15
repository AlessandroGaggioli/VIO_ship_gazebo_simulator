#!/usr/bin/env bash
# Shared library: run all (comp × 3dof × runs A..E) combinations for one bag.
#
# Wrapper scripts must set:
#   BAG          — rosbag2 directory name under ~/ship_ws/bags
#   FILE_PREFIX  — short tag for output filenames (e.g. mot_line)
#   BASE_PATH    — output root (config subdirs created under it)
# Then call:    run_all_combinations
#
# Optional overrides via env vars:
#   RUNS                  default: (A B C D E)
#   COMBOS                default: 4 combinations (see below)
#   POLL_INTERVAL         default: 2
#   PDF_TIMEOUT           default: 600
#   KILL_GRACE            default: 10
#   COOLDOWN              default: 12
#   POST_PDF_WAIT         default: 10   (extra wait after odom PDF so imu/map finish)
#   LOG_DIR               default: /tmp/ship_batch_logs

set -u

: "${RUNS:=A B C D E F G H I J}"
: "${COMBOS:=true:false true:true false:false false:true}"
: "${POLL_INTERVAL:=2}"
: "${PDF_TIMEOUT:=600}"
: "${KILL_GRACE:=10}"
: "${COOLDOWN:=12}"
: "${POST_PDF_WAIT:=10}"
: "${LOG_DIR:=/tmp/ship_batch_logs}"

ts() { date +"%H:%M:%S"; }

start_in_group() {
    # Run command in its own session/process group, redirect output, echo leader PID.
    local cmd="$1"
    local logfile="$2"
    setsid bash -c "exec ${cmd}" >"$logfile" 2>&1 &
    echo $!
}

kill_group() {
    # Kill the whole process group identified by leader PID (which equals PGID
    # because we launched it via setsid).
    local pid="$1"
    [ -z "$pid" ] && return 0
    if kill -0 "$pid" 2>/dev/null; then
        kill -INT -"$pid" 2>/dev/null || true
        local i
        for i in $(seq 1 "$KILL_GRACE"); do
            kill -0 "$pid" 2>/dev/null || return 0
            sleep 1
        done
        kill -KILL -"$pid" 2>/dev/null || true
    fi
}

reap_orphans() {
    # Kill anything that might have escaped the process groups.
    pkill -f "ros2 bag play"     2>/dev/null || true
    pkill -f "stereo_odometry"   2>/dev/null || true
    pkill -f "rtabmap"           2>/dev/null || true
    pkill -f "rqt_image_view"    2>/dev/null || true
    pkill -f "imu_compensator"   2>/dev/null || true
    pkill -f "odom_comparator"   2>/dev/null || true
    pkill -f "imu_comparator"    2>/dev/null || true
    pkill -f "map_analyzer"      2>/dev/null || true
    pkill -f "rgbd_image_relay"  2>/dev/null || true
}

PID_REPLAY=""
PID_ANALYZE=""
cleanup_on_exit() {
    echo
    echo "[$(ts)] Interrupted — terminating background launches..."
    kill_group "$PID_REPLAY"
    kill_group "$PID_ANALYZE"
    reap_orphans
    exit 130
}

run_all_combinations() {
    : "${BAG:?BAG must be set by wrapper}"
    : "${FILE_PREFIX:?FILE_PREFIX must be set by wrapper}"
    : "${BASE_PATH:?BASE_PATH must be set by wrapper}"

    local BAG_DIR="${HOME}/ship_ws/bags/${BAG}"
    if [ ! -d "$BAG_DIR" ]; then
        echo "[$(ts)] ERROR: bag directory not found: $BAG_DIR" >&2
        exit 1
    fi

    mkdir -p "$LOG_DIR" "$BASE_PATH"
    trap cleanup_on_exit INT TERM

    echo "================================================================"
    echo "  Batch run                bag = $BAG"
    echo "  Output root              $BASE_PATH"
    echo "  File prefix              $FILE_PREFIX"
    echo "  Combinations             ${COMBOS}"
    echo "  Runs per combo           ${RUNS}"
    echo "  Logs                     $LOG_DIR"
    echo "================================================================"

    local total_runs=0
    local skipped=0
    local started=0
    local failed=0

    # shellcheck disable=SC2206
    local -a combos_arr=(${COMBOS})
    # shellcheck disable=SC2206
    local -a runs_arr=(${RUNS})

    for combo in "${combos_arr[@]}"; do
        IFS=':' read -r COMP DOF <<<"$combo"
        [ "$COMP" = "true" ] && comp_tag="comp_on" || comp_tag="comp_off"
        [ "$DOF"  = "true" ] && dof_tag="3dof_on"  || dof_tag="3dof_off"
        local CONFIG="${comp_tag}_${dof_tag}"
        local SAVE_PATH="${BASE_PATH}/${CONFIG}"
        mkdir -p "$SAVE_PATH"

        for run in "${runs_arr[@]}"; do
            total_runs=$((total_runs + 1))
            local FILENAME="${FILE_PREFIX}_${CONFIG}_${run}"
            local PDF_ODOM="${SAVE_PATH}/${FILENAME}_odom.pdf"

            if [ -f "$PDF_ODOM" ]; then
                echo "[$(ts)] SKIP   ${CONFIG}/${run} — PDF already exists"
                skipped=$((skipped + 1))
                continue
            fi

            echo "[$(ts)] START  ${CONFIG}/${run}  comp=${COMP} 3dof=${DOF}"

            PID_ANALYZE=$(start_in_group \
                "ros2 launch ship_gazebo analyze_sim.launch.py \
                    save_path:=${SAVE_PATH} \
                    filename:=${FILENAME} \
                    save_odom:=true save_imu:=true save_map:=true" \
                "${LOG_DIR}/analyze_${CONFIG}_${run}.log")

            # Let analyze subscribers come up before bag play starts publishing.
            sleep 3

            PID_REPLAY=$(start_in_group \
                "ros2 launch ship_gazebo replay.launch.py \
                    bag:=${BAG} 3dof:=${DOF} comp:=${COMP}" \
                "${LOG_DIR}/replay_${CONFIG}_${run}.log")

            # Poll for odom_comparator's final PDF — written in its `finally`
            # block after motion has stopped for >5s.
            local elapsed=0
            while [ $elapsed -lt "$PDF_TIMEOUT" ]; do
                if [ -f "$PDF_ODOM" ]; then
                    echo "[$(ts)]   PDF ready after ${elapsed}s — waiting ${POST_PDF_WAIT}s for imu/map"
                    sleep "$POST_PDF_WAIT"
                    break
                fi
                # Bail early if the analyze launch died unexpectedly.
                if ! kill -0 "$PID_ANALYZE" 2>/dev/null; then
                    echo "[$(ts)]   WARN: analyze process exited before PDF (check log)"
                    break
                fi
                sleep "$POLL_INTERVAL"
                elapsed=$((elapsed + POLL_INTERVAL))
            done

            if [ -f "$PDF_ODOM" ]; then
                started=$((started + 1))
            else
                failed=$((failed + 1))
                echo "[$(ts)]   FAIL ${CONFIG}/${run} — no PDF after ${PDF_TIMEOUT}s"
                echo "[$(ts)]        log: ${LOG_DIR}/analyze_${CONFIG}_${run}.log"
            fi

            echo "[$(ts)]   Cleaning up..."
            kill_group "$PID_REPLAY";  PID_REPLAY=""
            kill_group "$PID_ANALYZE"; PID_ANALYZE=""
            reap_orphans

            echo "[$(ts)] DONE   ${CONFIG}/${run} — cooldown ${COOLDOWN}s"
            sleep "$COOLDOWN"
        done
    done

    echo "================================================================"
    echo "[$(ts)] Batch complete on bag=$BAG"
    echo "       total=${total_runs}  ok=${started}  skipped=${skipped}  failed=${failed}"
    echo "================================================================"
}
