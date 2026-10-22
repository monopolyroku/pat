#!/usr/bin/env bash
# Run plant + estimator for one scenario, score it, and shut everything down.
#   run_scenario.sh <fusion_mode> <plant_params.yaml> [duration_s] [settle_s] [label]
set -eo pipefail   # no -u: ROS setup scripts reference unset variables
mode="$1"; plant="$2"; duration="${3:-10}"; settle="${4:-2}"; label="${5:-$mode}"
# Walk up from this script until a colcon workspace root (install/setup.bash) is found.
ws="$(cd "$(dirname "$0")" && pwd)"
while [ "$ws" != "/" ] && [ ! -f "$ws/install/setup.bash" ]; do ws="$(dirname "$ws")"; done
[ -f "$ws/install/setup.bash" ] || { echo "cannot find install/setup.bash above $0" >&2; exit 2; }
source "$ws/install/setup.bash"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(ros2 pkg prefix pat_sim)/share/pat_sim/config/cyclonedds_localhost.xml"

if pgrep -f "lib/pat_sim/synthetic_plant|lib/pat_estimator/state_estimator" >/dev/null; then
  echo "refusing to start: plant/estimator processes already running" >&2
  pgrep -af "lib/pat_sim/synthetic_plant|lib/pat_estimator/state_estimator" >&2
  exit 2
fi

ros2 launch pat_estimator estimator_sim.launch.py \
  fusion_mode:="$mode" plant_params:="$plant" >/tmp/run_scenario_launch.log 2>&1 &
launch_pid=$!
cleanup() {
  kill -INT "$launch_pid" 2>/dev/null || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do
    pgrep -f "lib/pat_sim/synthetic_plant|lib/pat_estimator/state_estimator" >/dev/null || break
    sleep 0.5
  done
  pkill -9 -f "lib/pat_sim/synthetic_plant|lib/pat_estimator/state_estimator" 2>/dev/null || true
}
trap cleanup EXIT

sleep 3
ros2 run pat_sim score_estimate.py --duration "$duration" --settle "$settle" --label "$label"
