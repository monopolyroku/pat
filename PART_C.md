# Part C — Implementation: delayed camera fusion in `state_estimator`

Part B ranked the delayed, asynchronous camera correction in the state
estimator as the highest-risk element of the design. Part C implements it,
verifies it in isolation, and runs it as a ROS 2 node against a synthetic plant
over the real middleware. Section references (`A.0.3`, `TD-4`, …) point into
`docs/design_part_a.md` and `docs/design_rationale.md`.

---

## 1. Target environment

| Item | Value |
|---|---|
| ROS 2 distribution | **Jazzy Jalisco** (`rclcpp` 28.x, `rclcpp_lifecycle`) |
| OS | Ubuntu 24.04 LTS (Noble) |
| RMW | `rmw_cyclonedds_cpp` (`ros-jazzy-rmw-cyclonedds-cpp`), loopback-only configuration |
| Language / standard | C++17 (`-Wall -Wextra -Wpedantic`, warning-free); Python 3.12 for scoring |
| Build | `colcon`, `ament_cmake`, `ament_cmake_gtest` |
| Extra system deps | none beyond `ros-jazzy-desktop` + `ros-jazzy-rmw-cyclonedds-cpp` |
| Kernel used for the figures below | generic 6.17 / 7.0 (**not** PREEMPT_RT); all timing is development evidence only (TD-10) |

Workspace layout:

```
ros2_ws/src/
  pat_msgs/       fixed-size messages: SpotError, LosEstimate, FsmState, FsmCmd,
                  GimbalCmd, GimbalState, Mode, SimTruth (AD-7)
  pat_common/     header-only: qos.hpp (the only place QoS is defined), topics.hpp,
                  units.hpp, SeqlockBuffer, SpscQueue (TD-7)
  pat_estimator/  pat_estimator_core (ROS-free ResidualLosEstimator) +
                  state_estimator lifecycle node + two-executor executable
  pat_sim/        PlantModel physics + synthetic_plant node, scenario configs,
                  score_estimate.py, run_scenario.sh
```

---

## 2. Build and run

### 2.1 Build

```bash
sudo apt install ros-jazzy-rmw-cyclonedds-cpp        # once
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

### 2.2 Unit tests (no ROS graph needed)

```bash
colcon test --event-handlers console_direct+
colcon test-result --verbose
# or a single binary:
./build/pat_estimator/test_residual_los_estimator
./build/pat_sim/test_plant_model
./build/pat_common/test_lockfree
```

Expected: `27 gtest cases, 0 failures` (`colcon test-result` prints 30 because
it also counts the three ctest wrappers).

### 2.3 Run the node against the synthetic plant (preferred: scenario runner)

```bash
# <fusion_mode> <plant.yaml> [scored_duration_s] [settle_s] [label]
install/pat_sim/lib/pat_sim/run_scenario.sh replay \
    src/pat_sim/config/plant_motion_1degs.yaml 10 2 "replay, 1 deg/s"

install/pat_sim/lib/pat_sim/run_scenario.sh apply_at_arrival \
    src/pat_sim/config/plant_motion_1degs.yaml 10 2 "baseline, 1 deg/s"

install/pat_sim/lib/pat_sim/run_scenario.sh replay \
    src/pat_sim/config/plant_camera_gaps.yaml 14 1 "camera gaps + gimbal ramp"
```

The runner launches `synthetic_plant` + `state_estimator`, waits 3 s, scores
`/pat/estimate/los_error` against `/pat/sim/truth` for the requested duration,
prints a report, and shuts both nodes down. It **refuses to start if plant or
estimator processes already exist** (see §3.5 for why).

Sample report:

```
--- score (replay, 1 deg/s) ---
matched samples      : 9937  (estimates received 11936)
error  RMS / p99 / max: 11.38 / 25.98 / 36.52 urad
error  RMS while valid: 11.38 urad  (valid 100.0 % of samples)
|err|/sigma median    : 0.63  (~0.8-1.2 means the covariance is honest)
camera_age_ms  p50/p99: 36.1 / 49.1
modes                 : IMU_AND_CAMERA 100.0 %
camera frames accepted/rejected: 762 / 2; imu samples rejected: 0
```

### 2.4 Run manually

```bash
# terminal 1
ros2 launch pat_estimator estimator_sim.launch.py \
    fusion_mode:=replay \
    plant_params:=$PWD/src/pat_sim/config/plant_motion_1degs.yaml \
    estimator_params:=$PWD/src/pat_estimator/config/estimator_default.yaml \
    fast_thread_priority:=0

# terminal 2 — must select the same RMW and Cyclone config as the launch file
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/src/pat_sim/config/cyclonedds_localhost.xml
ros2 lifecycle get /state_estimator                 # -> active [3]
ros2 topic hz /pat/estimate/los_error               # ~1000 Hz
ros2 topic echo --once /pat/estimate/los_error
ros2 run pat_sim score_estimate.py --duration 10 --settle 2 --label manual
```

Plant alone: `ros2 launch pat_sim plant.launch.py [params:=file.yaml]`.

Launch arguments: `fusion_mode` = `replay` (TD-4 option c, default) or
`apply_at_arrival` (the measured baseline); `fast_thread_priority` = SCHED_FIFO
priority for the E1 thread, `0` = off (needs `CAP_SYS_NICE`; without it the
node logs a warning and continues).

### 2.5 Scenario files

| File (`pat_sim/config/`) | What it exercises |
|---|---|
| `plant_default.yaml` | All `[A]` assumptions of A.0.2; static platform with 100 urad vibration |
| `plant_motion_1degs.yaml` | 1°/s base motion, camera 25 ± 5 ms — the TD-4 case |
| `plant_camera_gaps.yaml` | 150 ms and 400 ms camera gaps (CD-6 thresholds) plus a 0.1°/s gimbal RECENTRE ramp (feedforward) |

Knobs: `platform_rate_deg_s`, `vib_rms_urad`, `camera_latency_ms`,
`camera_jitter_ms` (> 10 ms produces out-of-order frames), `camera_dropout_prob`,
`camera_gaps_s`, `imu_gaps_s`, `imu_bursts_s`, `imu_glitch_rate_hz`,
`gimbal_profile: hold|ramp|sine`, `fsm_profile: hold|sine`.

Gotchas:

- ROS 2 YAML cannot express an empty list parameter; **omit** a schedule
  parameter to disable it rather than writing `[]`.
- Anything publishing on a control topic from a script must use the same
  deadline QoS as `pat_common/qos.hpp`, or the middleware silently drops it.
- Before trusting a live number, check
  `pgrep -af "lib/pat_sim/synthetic_plant|lib/pat_estimator/state_estimator"`
  shows exactly one of each.

---

## 3. Tests run and results

### 3.1 Live, end-to-end over CycloneDDS (`run_scenario.sh`)

Host: laptop, generic kernel, normal scheduling priority. Each row is a
separate clean run, 10 s scored after a 2 s settle (14 s / 1 s for the gap
scenario). Error is `|ê − e_true|` matched by header stamp at ~1 kHz.

| Scenario | `fusion_mode` | RMS / p99 / max (urad) | valid | Notes |
|---|---|---|---|---|
| 1°/s base motion, 100 urad vibration, camera 25 ± 5 ms + 5 urad noise, gyro bias 200 urad/s + 50 urad/s/√Hz | **replay** | **11.4 / 26.0 / 36.5** | 100 % | 762 frames accepted, 2 rejected at start-up; output at 998 Hz; `|err|/σ` 0.63 |
| same | **apply_at_arrival** | **443.8 / 565 / 583** | 100 % | = rate × delay (436 urad) as TD-4 predicts; `|err|/σ` = 45, i.e. the covariance does not know it is wrong |
| 0.2°/s, camera gaps 150 ms / 400 ms, gimbal ramp 0.1°/s from t = 3 s | replay | 12.5 / 32.0 / 39.6 | 96.5 % | `IMU_ONLY` mode 5.7 % of samples (age > 100 ms); `valid=false` exactly in the > 200 ms portions of the long gaps; max error 39.6 urad while coasting 400 ms with 698 urad of commanded gimbal motion fed forward |

Interpretation: with the assumed sensors, replay fusion keeps the residual
inside the 20 urad fine-lock budget under 1°/s motion; apply-at-arrival
exceeds it 20×. The 11 urad floor is gyro noise (≈ 7 urad random walk between
20 ms frames) plus camera noise, not fusion error — the same estimator reaches
0.4 urad on noise-free inputs (§3.3).

Also checked live: `ros2 lifecycle get /state_estimator` → `active`;
`fast_thread_priority:=50` without privilege → `SCHED_FIFO 50 not applied
(Operation not permitted); running at normal priority`, node continues.

### 3.2 Plant node alone, live

| Check | Result |
|---|---|
| Topic rates (300-sample window) | IMU 999 Hz, camera 50.2 Hz, FSM state 1000 Hz, gimbal state 200 Hz, truth 1000 Hz |
| Camera latency = receive − `header.stamp` (352 frames) | min 21.2 / mean 25.9 / max 30.4 ms |
| Camera `ex` vs truth at the same stamp (351 matched) | residual σ 5.07 urad, mean −0.07 |
| `ex == target − platform − gimbal − fsm` on every truth message | holds to 1e-9 |
| Platform vibration RMS from truth stream | 100 urad |
| External `/pat/fsm/cmd` = 500 urad at 1 kHz | FSM settles at 500.0 urad; profile overridden |

### 3.3 `pat_estimator` unit tests (14, ROS-free scripted truth model)

| Test | Verified |
|---|---|
| `Fusion.StaticPlatformReplayEqualsApplyAtArrival` | with no motion the two modes agree within 1e-3 urad |
| `Fusion.OneDegPerSecondMotionExposesApplyAtArrival` | baseline error ≈ 436 urad (±15 %); replay < 5 urad RMS, < 10 peak |
| `Fusion.JitteredDelayWithVibrationStaysAccurate` | 0.5°/s + 100 urad @ 37 Hz, 25 ± 5 ms jitter, 5 urad noise → 4.15 urad RMS; jitter adds 0.03 urad |
| `Fusion.OutOfOrderFramesAreRejectedNotFused` | 15 ms jitter (> frame period) → `out_of_order` counter > 0, accuracy unchanged |
| `Fusion.GimbalCommandIsFedForwardWithoutCamera` | 1 s of 0.1°/s commanded motion with camera off → error < 5 urad, `valid=false` |
| `Fusion.MeasuredFsmMotionIsSubtracted` | 800 urad @ 3 Hz FSM motion → < 5 urad RMS |
| `Gates.StaleFutureInvalidAndOutlierFrames` | out-of-order, too-old (> 128 ms ring), future (> 5 ms), invalid, and 3σ-with-low-confidence each rejected into their own counter; same 3σ jump with confidence 0.95 accepted |
| `Gates.CameraGapGrowsCovarianceAndExpiresValidity` | 70 ms: valid; 170 ms: valid, `IMU_ONLY`, P grown; 270 ms: invalid; recovers |
| `Gates.CorrectionOlderThanRingIsCountedAsOverrun` | correction applied after ring wrap is refused, never half-applied |
| `Imu.SingleSpikeIsDroppedWithoutDisturbingEstimate` | 20 rad/s sample counted, peak error < 5 urad |
| `Imu.BurstSwitchesToCameraOnlyThenRecovers` | 10 bad samples in 10 ms → `CAMERA_ONLY`; nominal again after 1 s clean |
| `Imu.AbsentImuIsCameraOnlyMode` | watchdog propagation reports `CAMERA_ONLY` |
| `FastPath.NoHeapAllocationInSteadyState` | global `operator new` counter: 100 propagations + prepare/apply + snapshot = **0 allocations** |
| `FastPath.ConcurrentCameraCorrectionsNeverCorruptState` | separate E1/E2 threads for 500 ms: > 50 corrections applied, estimate within 10 urad of truth, zero overruns; 5 reruns, no flakes |

A real defect found by these tests: rectangular integration of the gyro rate
lagged the true angle by half a sample (≈ 10 urad at 37 Hz / 100 urad —
half the fine-lock budget). Switching to trapezoidal integration cut the
perfect-input residual from 9.7 to 0.38 urad.

### 3.4 `pat_sim` (8) and `pat_common` (5) unit tests

| Test | Verified |
|---|---|
| `PlantModel.MeasurementEquationHolds` | truth terms sum correctly to 1e-9; analytic platform angle/rate match |
| `PlantModel.FsmStepResponseMatchesSecondOrderModel` | ζ = 0.7 → 4.6 % overshoot (1046 ± 10 on a 1000 step) |
| `PlantModel.FsmExactStepIsStableAtOneKilohertz` | exact ZOH step converges at ωn·dt = 5 (Euler would diverge) |
| `PlantModel.FsmSaturatesAtRange`, `GimbalRateAndRangeLimits` | ±2000 urad / 20°/s / ±60° with flags |
| `PlantModel.VibrationRmsAndRateAreConsistent` | 20 s RMS = 100 ± 5 urad; analytic rate = numerical derivative |
| `PlantModel.GyroSampleIsTruthPlusBiasPlusNoise`, `CameraSampleIsTruthPlusNoise` | statistics match the configured noise |
| `SeqlockBuffer.NeverObservesTornState` | 300 ms writer/reader stress, zero torn reads |
| `SpscQueue.FifoAndCapacity`, `CrossThreadOrdering` | 200 000 items in order across threads; full queue refuses, never blocks |
| `Units.Conversions` | deg↔urad, ns↔`builtin_interfaces/Time` |

### 3.5 Things that went wrong during testing (and what changed)

| Observation | Cause | Fix |
|---|---|---|
| `camera frames accepted` rising at ~1 kHz instead of 50 Hz in the first live runs | Killing `ros2 launch` from the tool harness orphaned the node processes; two plants and two estimators were publishing on the same topics. The estimator was correct; the measurement was not. | `run_scenario.sh` refuses to start with stale processes and kills its children on exit. All numbers in §3.1 are from clean runs. |
| First `/pat/fsm/cmd` test publisher silently ignored | Script did not use the shared deadline QoS; middleware reported *incompatible QoS: DEADLINE* | Script fixed; this is the A.4.1 mismatch guard working as designed |
| `synthetic_plant` crashed at start with `camera_gaps_s: []` | ROS 2 cannot type an empty YAML list | Schedules omitted from the default yaml; documented |
| Jitter unit test failed at 9.7 urad with perfect inputs | Half-sample lag of rectangular rate integration | Trapezoidal integration (see §3.3) |

### 3.6 Not tested / known limits

- No timing percentiles (TD-10). Wall timers, generic kernel, normal priority:
  the ~1 kHz rates are averages with unbounded jitter.
- Authority enforcement (`authority_seq`), `/pat/mode` handling, and a lifecycle
  manager are not implemented; the estimator autostarts itself.
- The estimate is scored open-loop; no controller closes on it yet.
- Reported covariance is ~1.6× conservative (`|err|/σ` ≈ 0.63); untuned.

---

## 4. How it works

### 4.1 ResidualLosEstimator (ROS-free core)

State `x = [ex, ey, vx, vy]`: residual LOS error (urad) and an unmodelled
drift rate (urad/s) that absorbs gyro bias and target LOS drift (AD-3, AD-4).

**Propagation (E1, on every IMU sample, TD-1).** Discrete form of
`ė = −ω_plat − θ̇_g_cmd − θ̇_f` (A.0.3): gyro rate (trapezoidal), commanded
gimbal rate from `/pat/gimbal/cmd`, and *measured* FSM position difference from
`/pat/fsm/state`. `P` propagates through `F = [[I, dt·I],[0, I]]` with gyro
white noise on `e` and random walk on `v`. Every step writes an entry
`{stamp, dt, u·dt, x, P}` into a 128-slot ring (≥ 100 ms, TD-4) guarded by a
per-slot seqlock. IMU samples beyond 10 rad/s or with a per-sample step
> 1 rad/s are replaced by the previous sample and counted; ≥ 5 rejections in
100 ms switch to camera-only propagation (zero rate, inflated Q) until 1 s
clean (A.5.1).

**Delayed correction (E2, on each camera frame, A.3.3).** `prepare_correction`
gates the frame (invalid, older than the last accepted, older than the ring,
> 5 ms in the future), locates the two ring entries bracketing the exposure
stamp `t_k` through the seqlock, interpolates `x(t_k)` and propagates `P` to
`t_k`, forms the innovation with `R = (σ_cam / confidence)²`, rejects a > 3σ
innovation only if confidence < 0.6, and performs a Joseph-form Kalman update
at `t_k`. It returns a POD `Correction {t_k, entry_index, alpha, x, P}` — it
never writes the estimate.

**Replay (E1, at the start of the next tick).** `apply_correction` replays the
ring from `entry_index` to the newest entry using the *stored* input
increments, rewriting each entry so later frames interpolate against corrected
history, and adopts the result as the live state. If the ring has wrapped past
`entry_index` the correction is refused and counted (`overrun`). Because the
model is linear with known inputs, replay is exact; cost is ≤ 128 small vector
operations, no allocation, no mutex.

**`fusion_mode: apply_at_arrival`** forces `t_k := newest stamp`, i.e. the
measurement is treated as current — the TD-4 baseline, kept selectable so the
comparison is a measured number.

**Validity** (CD-6): `valid = camera_age < 200 ms ∧ at least one frame accepted
∧ trace(P_pos) < threshold`. `mode` reports `IMU_ONLY` when camera age exceeds
100 ms and `CAMERA_ONLY` when the IMU is in burst or absent.

### 4.2 `state_estimator` node and process layout

`rclcpp_lifecycle::LifecycleNode`. Subscriptions and the publisher are created
in `on_configure`; outputs flow only when `active`. `autostart: true`
configures and activates on the first spin (a lifecycle manager would do this
in the full system, AD-8).

| Callback group | Executor / thread | Callbacks |
|---|---|---|
| `imu_grp` (mutually exclusive) | **E1**, own `SingleThreadedExecutor` and thread, optional `SCHED_FIFO` | `/pat/imu/data` → drain correction queue, `propagate`, publish `LosEstimate`; `/pat/fsm/state`; `/pat/gimbal/cmd`; 2 ms IMU-loss watchdog (propagates with `imu_present=false` after 5 ms silence) |
| `cam_grp` (mutually exclusive) | **E2**, second `SingleThreadedExecutor` with the node's default group | `/pat/camera/spot_error` → `prepare_correction` → `SpscQueue<Correction, 8>` |

All inputs the fast tick reads (`fsm_pos_`, `gimbal_rate_`) are written only by
callbacks in `imu_grp`, so E1 owns all mutable estimator state; E2 reads the
ring through the seqlock and communicates only through the SPSC queue (TD-7).
QoS for every endpoint comes from `pat_common/qos.hpp` (best-effort, depth 1,
deadline for sensor/estimate topics; A.4.1).

### 4.3 Synthetic plant

`synthetic_plant` stands in for `camera_driver`, `imu_driver`, `fsm_driver` and
`gimbal_driver` (A.1.2) and publishes on the real topics with the shared QoS;
it also publishes `/pat/sim/truth` for scoring only.

Truth from the measurement equation, computed analytically in
`PlantModel::truth()`:

| Term | Model |
|---|---|
| `θ_tgt` | constant offset + linear LOS drift |
| `θ_plat` | constant rate + optional slow sway + vibration: `vib_lines` log-spaced sinusoids, amplitude ∝ 1/f, random phase per axis, scaled to `vib_rms_urad`; angle and rate both analytic so IMU and camera are exactly consistent |
| `θ_g` | first-order servo at 5 Hz, rate limit 20°/s, range ±60°/±30° with `at_limit` |
| `θ_f` | second-order low-pass (800 Hz, ζ 0.7) via the exact ZOH solution; clamped at ±2 mrad with `saturated` |

Sensors: IMU at every 1 ms tick (white noise √(fs/2)-scaled, constant bias +
random walk, scheduled gaps, scheduled garbage bursts, Poisson spikes); camera
at 50 Hz **stamped at exposure, delivered 25 ± 5 ms later** from an internal
queue in arrival order, with random dropouts and scheduled gaps; FSM state at
1 kHz; gimbal state at 200 Hz.

Actuators follow external `/pat/fsm/cmd` / `/pat/gimbal/cmd` if fresh within
200 ms, else an open-loop profile (`hold | ramp | sine`). In gimbal profile mode
the plant republishes the command it is following on `/pat/gimbal/cmd` so the
estimator receives feedforward (its subscription ignores local publications).
Physics advance by measured wall-clock `dt` per tick (clamped 0.25–5 periods)
and stamp with the node clock, so timer jitter appears as irregular spacing
rather than stamp drift.

### 4.4 Scoring

`score_estimate.py` subscribes to `/pat/estimate/los_error` and `/pat/sim/truth`
with the matching QoS, pairs messages by exact header stamp (the plant stamps
IMU and truth identically per tick; the estimator stamps output with the IMU
stamp), and reports RMS / p99 / max error, error while `valid`, valid fraction,
`|err|/σ` from the published covariance, camera-age percentiles, mode
occupancy, and the estimator's own counters.
