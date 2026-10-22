# pat_sim — synthetic plant, and what has been tested so far

This file covers two things: how the `synthetic_plant` node works, and the
verification evidence gathered for the code written to date (`pat_common`,
`pat_estimator`, `pat_sim`). Section references (`A.0.3`, `TD-4`, …) point into
`docs/design_part_a.md` and `docs/design_rationale.md`.

---

## 1. How the synthetic plant works

### 1.1 Purpose

`synthetic_plant` stands in for the four hardware driver nodes of the design
(`camera_driver`, `imu_driver`, `fsm_driver`, `gimbal_driver`, A.1.2). It
publishes on the **real topic names with the shared QoS profiles** from
`pat_common/qos.hpp`, so a flight node cannot tell whether it is talking to the
plant or to hardware. It also publishes ground truth on `/pat/sim/truth` for
scoring, which no flight node is allowed to consume.

The plant is split in two:

| Layer | File | Role |
|---|---|---|
| `PlantModel` | `include/pat_sim/plant_model.hpp`, `src/plant_model.cpp` | ROS-free physics and sensor noise. Deterministic given a seed. Unit-tested. |
| `SyntheticPlantNode` | `src/synthetic_plant_node.cpp` | Timers, topics, QoS, camera latency queue, dropout/glitch schedules, open-loop actuator profiles, external command override. |

### 1.2 The truth model

Everything is built from the measurement equation in A.0.3, written once in
`PlantModel::truth()`:

```
e_cam(t) = θ_tgt(t) − θ_plat(t) − θ_g(t) − θ_f(t)        (urad, per axis)
```

| Term | Model | Parameters |
|---|---|---|
| `θ_tgt` target direction | constant offset + linear LOS drift | `target_offset_urad`, `target_drift_urad_s` |
| `θ_plat` platform attitude | constant rate + optional slow sway + vibration | `platform_rate_deg_s`, `platform_sway_amp_deg/freq_hz`, `vib_*` |
| vibration | sum of `vib_lines` log-spaced sinusoids between `vib_f_lo_hz` and `vib_f_hi_hz`, amplitude ∝ 1/f (falling with frequency, as CD-3 assumes), random phase per axis, scaled so total RMS = `vib_rms_urad`. Angle and rate are both analytic, so the IMU is exactly consistent with the camera. | |
| `θ_g` gimbal | first-order servo at `gimbal_bw_hz` toward the command, rate-limited to `gimbal_max_rate_deg_s`, range-limited to `gimbal_range_deg` with `at_limit` flag | |
| `θ_f` FSM | second-order low-pass (`fsm_wn_hz`, `fsm_zeta`) discretised with the **exact zero-order-hold solution** of the damped oscillator, so it is stable at any step size (naive Euler diverges at ωn·dt = 5). Clamped to ±`fsm_range_urad` with `saturated` flag. | |

Default values are the `[A]` assumptions of A.0.2 and live in
`config/plant_default.yaml`.

### 1.3 Sensors

| Topic | Rate | Content | Imperfections modelled |
|---|---|---|---|
| `/pat/imu/data` (`sensor_msgs/Imu`) | every tick (1 kHz) | `angular_velocity.x/.y` = true platform rate on the ex/ey axes (the static platform→optical rotation is identity in sim) | white noise `gyro_noise_urad_s_sqrt_hz` scaled by √(fs/2); constant bias `gyro_bias_urad_s` plus random walk `gyro_bias_rw_urad_s_sqrt_s`; scheduled gaps `imu_gaps_s`; scheduled garbage bursts `imu_bursts_s`; Poisson single-sample spikes `imu_glitch_rate_hz` of `imu_glitch_rad_s` |
| `/pat/camera/spot_error` (`pat_msgs/SpotError`) | `camera_rate_hz` (50) | true `e_cam` at exposure time + `camera_noise_urad`; confidence `camera_confidence` ± noise | **stamp = exposure mid-time; delivery delayed by `camera_latency_ms` ± uniform `camera_jitter_ms`** via an internal queue drained each tick; random dropouts `camera_dropout_prob`; scheduled gaps `camera_gaps_s`. Frames are released in *arrival* order, so when jitter exceeds the frame period they arrive out of stamp order — the estimator must reject those (A.3.3 item 3). |
| `/pat/fsm/state` (`pat_msgs/FsmState`) | every tick | mirror position, `saturated` | none (strain gauge assumed clean) |
| `/pat/gimbal/state` (`pat_msgs/GimbalState`) | `gimbal_state_rate_hz` (200) | position, rate, `at_limit` | none |
| `/pat/sim/truth` (`pat_msgs/SimTruth`) | `truth_rate_hz` (1000) | all terms of the measurement equation, reliable QoS | — |

### 1.4 Actuator commands

The plant subscribes to `/pat/fsm/cmd` and `/pat/gimbal/cmd`. If a command has
arrived within `external_cmd_timeout_ms` it is used; otherwise an **open-loop
profile** drives the actuator so the estimator can be exercised without any
controller running:

- `gimbal_profile`: `hold` | `ramp` (`gimbal_profile_rate_deg_s` after `gimbal_profile_start_s`, default 0.1°/s = the RECENTRE offload rate limit of A.2.6) | `sine`.
  In profile mode the plant also **publishes the command it is following on `/pat/gimbal/cmd`** with its rate, because the estimator feeds commanded gimbal motion forward (A.0.3, A.2.3). The subscription uses `ignore_local_publications` so the plant does not mistake its own profile for an external command.
- `fsm_profile`: `hold` | `sine` (`fsm_profile_amp_urad`, `fsm_profile_freq_hz`). The estimator uses *measured* FSM position, so no FSM command is republished.

Authority tokens (`authority_seq`, A.2.2) are **not** enforced by the plant yet;
that belongs with the driver behaviour when controllers exist.

### 1.5 Timing

A single 1 kHz wall timer drives everything. The physics step uses the
*measured* elapsed time between ticks (clamped to 0.25–5 periods) and stamps
with the node clock, so timer jitter on a non-RT kernel shows up as slightly
irregular sample spacing rather than as a drift between stamps and wall time.
Lower-rate outputs (camera, gimbal state, truth) are derived with
phase accumulators so their average rates are exact.

### 1.6 Running it

```bash
source ros2_ws/install/setup.bash
ros2 launch pat_sim plant.launch.py                       # defaults
ros2 launch pat_sim plant.launch.py params:=/path/my.yaml # scenario file
```

The launch file sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and
`CYCLONEDDS_URI` to `config/cyclonedds_localhost.xml` (loopback only, no
multicast — A.4.4 / TD-9). To inspect from a shell, export the same two
variables first, otherwise `ros2 topic` will be on a different RMW.

Scenario knobs that matter most for the estimator: `platform_rate_deg_s`
(exposes apply-at-arrival fusion, TD-4), `camera_jitter_ms` > 10 (forces
out-of-order frames), `camera_gaps_s` (RECOVER thresholds, CD-6),
`imu_bursts_s` / `imu_glitch_rate_hz` (A.5.1 IMU handling),
`gimbal_profile: ramp` (feedforward during RECENTRE).

**YAML caveat:** ROS 2 cannot parse an empty list (`[]`) as a typed parameter;
omit a schedule parameter to disable it rather than setting it empty.

---

## 2. What has been tested

All tests run with `colcon test` (see `AGENTS.md`). Counts below are from the
current tree: **27 gtest cases, 0 failures** (5 `pat_common`, 14 `pat_estimator`,
8 `pat_sim`; `colcon test-result` reports 30 because it also counts the three
ctest wrappers).

### 2.1 `pat_common` — lock-free primitives (TD-7)

| Test | What it establishes |
|---|---|
| `SeqlockBuffer.NeverObservesTornState` | 300 ms writer/reader stress with a 4-field payload whose fields must agree: zero torn reads. |
| `SpscQueue.FifoAndCapacity`, `CrossThreadOrdering` | 200 000 items across threads arrive in order; full queue refuses rather than blocks. |
| `Units.Conversions` | deg↔urad and ns↔`builtin_interfaces/Time` round-trip. |

### 2.2 `pat_estimator` — ResidualLosEstimator core

Tests use a scripted truth model (in the test file) built from the same
measurement equation, run at 1 kHz with camera frames stamped at exposure and
delivered after a configurable delay.

**Fusion correctness (the risk #1 claim, TD-4)**

| Scenario | apply-at-arrival (option a) | buffer-and-replay (option c) |
|---|---|---|
| Static platform, 25 ms delay | exact | exact; identical to (a) within 1e-3 urad |
| 1°/s platform rate, 25 ms delay | **≈ 436 urad steady error** (asserted within 15 % of rate × delay) | **< 5 urad RMS, < 10 urad peak** |
| 0.5°/s + 100 urad vibration at 37 Hz, 25 ± 5 ms jitter, 5 urad camera noise | — | 4.15 urad RMS (noise-limited); jitter contributes 0.03 urad |
| Jitter 15 ms (> frame period) | — | out-of-order frames rejected and counted; accuracy unaffected |
| Gimbal ramp 0.1°/s, camera switched off after 1.5 s | — | after 1 s of camera-less coasting error < 5 urad (feedforward works); `valid` correctly false |
| FSM sine 800 urad @ 3 Hz | — | < 5 urad RMS (measured FSM motion subtracted) |

A real defect was found and fixed by these tests: rectangular integration of the
gyro rate lagged the true angle by half a sample (~10 urad at 37 Hz), half the
fine-lock budget. Trapezoidal integration reduced the perfect-input residual
from 9.7 to 0.38 urad.

**Gates (A.3.3 item 3, A.5.1)**

| Test | Behaviour verified |
|---|---|
| `StaleFutureInvalidAndOutlierFrames` | frames older than the last accepted → `out_of_order`; older than the 128 ms ring → `too_old`; > 5 ms in the future → `future`; `valid=false` → `invalid`; 3σ innovation with confidence 0.3 → `outlier`, same jump with confidence 0.95 → accepted. Each increments its own counter. |
| `CameraGapGrowsCovarianceAndExpiresValidity` | 70 ms age: valid, nominal; 170 ms: valid, mode `IMU_ONLY`, covariance grown; 270 ms: `valid=false` (CD-6); recovers on next frame. |
| `CorrectionOlderThanRingIsCountedAsOverrun` | a Correction applied after the ring has wrapped is refused and counted, never half-applied. |

**IMU handling**

| Test | Behaviour verified |
|---|---|
| `SingleSpikeIsDroppedWithoutDisturbingEstimate` | one 20 rad/s sample: counted, previous rate held, peak error < 5 urad. |
| `BurstSwitchesToCameraOnlyThenRecovers` | 10 bad samples in 10 ms → `CAMERA_ONLY`; after 1 s clean → nominal, valid. |
| `AbsentImuIsCameraOnlyMode` | watchdog-driven propagation with `imu_present=false` reports `CAMERA_ONLY`. |

**Fast-path audit (A.4.3)**

| Test | Behaviour verified |
|---|---|
| `NoHeapAllocationInSteadyState` | global `operator new` counter: 100 propagations + one prepare/apply + snapshot allocate **zero** bytes. |
| `ConcurrentCameraCorrectionsNeverCorruptState` | separate E1 (propagate/apply) and E2 (prepare) threads for 500 ms: > 50 corrections applied, estimate within 10 urad of truth, finite covariance, zero overruns. Re-run 5× with no flakes. |

### 2.3 `pat_sim` — PlantModel physics

| Test | Behaviour verified |
|---|---|
| `MeasurementEquationHolds` | published `e_cam` equals `target − platform − gimbal − fsm` to 1e-9; platform angle/rate analytic values match; FSM and gimbal settle on their commands. |
| `FsmStepResponseMatchesSecondOrderModel` | ζ = 0.7 step overshoot 4.6 % (peak 1046 ± 10 on a 1000 step), settles to 1000. |
| `FsmExactStepIsStableAtOneKilohertz` | 1 ms steps with ωn·dt = 5 converge exactly (would diverge under Euler). |
| `FsmSaturatesAtRange` | clamps at ±2000 urad with `saturated=true`. |
| `GimbalRateAndRangeLimits` | rate clamps at 20°/s; stops at ±60° with `at_limit=true`, rate 0. |
| `VibrationRmsAndRateAreConsistent` | 20 s RMS = 100 ± 5 urad; analytic rate equals numerical derivative of angle to 1e-6 rad/s. |
| `GyroSampleIsTruthPlusBiasPlusNoise` | mean = rate + bias; σ = σ_n·√(fs/2). |
| `CameraSampleIsTruthPlusNoise` | residual σ = 5 ± 0.3 urad; confidence in [0, 1]. |

### 2.4 `synthetic_plant` node — live check over CycloneDDS

Run with the default parameters and inspected from a separate `rclpy` process
on the loopback-only Cyclone configuration:

| Check | Result |
|---|---|
| Topic rates (`ros2 topic hz`, 300-sample window) | IMU 999 Hz, camera 50.2 Hz, FSM state 1000 Hz, gimbal state 200 Hz, truth 1000 Hz |
| Camera latency = receive time − `header.stamp` (352 frames) | min 21.2 / mean 25.9 / max 30.4 ms — matches 25 ± 5 ms |
| Camera `ex` vs truth at the same stamp (351 matched) | residual σ 5.07 urad, mean −0.07 urad |
| Truth consistency | `ex == target − platform − gimbal − fsm` on every truth message |
| Platform vibration RMS from truth stream | 100 urad |
| IMU mean | rate + 200 urad/s bias (+ vibration average over the window) |
| External `/pat/fsm/cmd` = 500 urad published at 1 kHz | FSM state settles at 500.0 urad; profile overridden |

One useful negative result: the first attempt to publish `/pat/fsm/cmd` from the
test script was rejected by the middleware with *incompatible QoS: DEADLINE*,
because the script had not used the shared profile. That is the AD-7 / A.4.1
mismatch guard behaving as designed; the fix was on the script side.

### 2.5 `state_estimator` node — end-to-end over CycloneDDS

The node (`pat_estimator/src/state_estimator_node.cpp`) wraps the core as a
`LifecycleNode` with two callback groups — `imu_grp` (IMU, FSM state, gimbal
cmd, IMU-loss watchdog) and `cam_grp` (camera) — which the executable
(`state_estimator_main.cpp`) places on two `SingleThreadedExecutor`s on
separate threads (E1/E2 of A.4.2). Camera corrections cross from E2 to E1
through `pat_common::SpscQueue` and are applied at the start of the next IMU
tick. `autostart: true` configures and activates on launch in lieu of a
lifecycle manager.

Scenarios are run with `run_scenario.sh <fusion_mode> <plant.yaml>`, which
launches plant + estimator, scores `/pat/estimate/los_error` against
`/pat/sim/truth` by matching header stamps, and shuts everything down. Results
on this host (generic kernel, normal priority, 10 s scored after 2 s settle):

| Scenario (`config/`) | fusion_mode | error RMS / p99 / max (urad) | valid | notes |
|---|---|---|---|---|
| `plant_motion_1degs.yaml`: 1°/s base motion, 100 urad vibration, camera 25 ± 5 ms, 5 urad noise, gyro bias 200 urad/s + 50 urad/s/√Hz noise | **replay** | **11.4 / 26.0 / 36.5** | 100 % | 762 frames accepted, 2 rejected (start-up); estimate at 998 Hz |
| same | apply_at_arrival | **443.8 / 565 / 583** | 100 % | ≈ rate × delay = 436 urad, as TD-4 predicts; `|err|/σ` = 45 — the covariance has no idea it is wrong |
| `plant_camera_gaps.yaml`: 0.2°/s, camera gaps of 150 ms and 400 ms, gimbal RECENTRE ramp 0.1°/s from t = 3 s | replay | 12.5 / 32.0 / 39.6 | 96.5 % | mode `IMU_ONLY` 5.7 % of samples (age > 100 ms); `valid=false` exactly during the > 200 ms portions of the long gaps; max error 39.6 urad while coasting 400 ms *with the gimbal moving* (698 urad of commanded motion fed forward) |

Why 11 urad live versus 4 urad in the unit tests: the plant adds the assumed
gyro noise (50 urad/s/√Hz → ~7 urad random walk between 20 ms frames) and a
200 urad/s bias the drift state must track; the unit-test truth model had
neither. 11 urad is within the 20 urad fine-lock budget of A.0.2 with the
assumed sensors. `|err|/σ` ≈ 0.63 means the reported covariance is
conservative by ~1.6×, which is acceptable for gating but worth tuning once
real noise figures exist.

Also checked: `fast_thread_priority:=50` without privilege logs
`SCHED_FIFO 50 not applied (Operation not permitted); running at normal
priority` and continues (TD-10 graceful degradation).

**A measurement mistake worth recording.** The first live runs showed
`camera frames accepted` growing at ~1 kHz instead of 50 Hz. The cause was not
in the estimator: killing a `ros2 launch` shell from the tool harness orphaned
the node processes, so two plants and two estimators were publishing on the
same topics. `run_scenario.sh` now refuses to start if plant/estimator
processes exist and kills its children on exit. All numbers above are from
clean runs.

### 2.6 Not yet tested / known limits

- No timing-percentile evidence (TD-10): the generic kernel, wall timers and
  normal priority mean the ~1 kHz rates are averages with unbounded jitter.
- Authority enforcement, `/pat/mode` handling and lifecycle management are not
  implemented; the estimator autostarts itself.
- The estimator's output is scored open-loop (no controllers close on it).
- Covariance calibration (`|err|/σ` ≈ 0.63) has not been tuned.
