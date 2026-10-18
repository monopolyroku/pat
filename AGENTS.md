# PAT terminal on-board software

ROS 2 Jazzy / C++17 / Ubuntu 24.04 / `rmw_cyclonedds_cpp`. Design documents live in
`docs/` (git-ignored, read-only reference): `design_part_a.md` is the architecture,
`design_rationale.md` records why each decision was made. Section references in code
comments (e.g. `A.3.3`, `TD-4`) point into those files.

## Layout

- `ros2_ws/src/pat_msgs` — fixed-size message definitions (AD-7)
- `ros2_ws/src/pat_common` — header-only: shared QoS profiles (`qos.hpp`, the only
  place QoS may be defined), topic names, units, `SeqlockBuffer`, `SpscQueue`

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
