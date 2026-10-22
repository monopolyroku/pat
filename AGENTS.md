# PAT terminal on-board software

ROS 2 Jazzy / C++17 / Ubuntu 24.04 / `rmw_cyclonedds_cpp`. Design documents live in
`docs/` (git-ignored, read-only reference): `design_part_a.md` is the architecture,
`design_rationale.md` records why each decision was made. Section references in code
comments (e.g. `A.3.3`, `TD-4`) point into those files.

## Layout

- `ros2_ws/src/pat_msgs` — fixed-size message definitions (AD-7)
- `ros2_ws/src/pat_common` — header-only: shared QoS profiles (`qos.hpp`, the only
  place QoS may be defined), topic names, units, `SeqlockBuffer`, `SpscQueue`
- `ros2_ws/src/pat_estimator` — `pat_estimator_core` (ROS-free ResidualLosEstimator,
  delayed camera fusion, TD-4) + `state_estimator` lifecycle node with the E1/E2
  two-executor layout
- `ros2_ws/src/pat_sim` — `PlantModel` physics + `synthetic_plant` node standing in
  for all four drivers, scorer and scenario runner; `README.md` there documents the
  plant and all test evidence

## Running the simulation

```bash
# plant + estimator, scored against truth, cleaned up on exit (preferred):
ros2_ws/install/pat_sim/lib/pat_sim/run_scenario.sh replay|apply_at_arrival \
    ros2_ws/src/pat_sim/config/plant_motion_1degs.yaml [duration_s] [settle_s] [label]

# or manually:
ros2 launch pat_estimator estimator_sim.launch.py fusion_mode:=replay plant_params:=...
ros2 launch pat_sim plant.launch.py [params:=file.yaml]
# in another shell, before any ros2 topic/node command:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/ros2_ws/src/pat_sim/config/cyclonedds_localhost.xml
```

Gotchas:
- ROS 2 YAML cannot express an empty list parameter; omit schedule params to disable.
- Killing a `ros2 launch` shell from a tool harness can orphan the node processes;
  check `pgrep -af "lib/pat_sim/synthetic_plant|lib/pat_estimator/state_estimator"`
  before trusting any live measurement. `run_scenario.sh` enforces this.
- Test scripts publishing on control topics must match the deadline QoS in
  `pat_common/qos.hpp` or the middleware silently drops them.

## Build and test

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
colcon test --event-handlers console_direct+   # add --packages-select <pkg> to narrow
colcon test-result --verbose
source install/setup.bash
```

## Conventions

- Angles in urad, rates in rad/s, stamps are measurement time (exposure mid-time
  for camera), never arrival time.
- Every publisher/subscription takes its QoS from `pat_common/qos.hpp`.
- Fast-path code (anything on executor E1): no heap allocation after configure,
  no mutex, no logging, no service calls (A.4.3 audit list).
- All numeric thresholds from the design are ROS parameters with units in the name.

## Environment notes

- This dev host runs a generic (non-PREEMPT_RT) kernel; timing figures gathered
  here are development evidence only (TD-10). RT scheduling / affinity calls must
  degrade gracefully when they fail.
