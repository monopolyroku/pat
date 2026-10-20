// Residual line-of-sight error estimator with delayed camera fusion.
//
// Implements design_part_a.md A.0.3 (measurement model), A.3.3 items 2-4
// (buffer-and-replay correction, stale/out-of-order rejection, lock-free
// hand-off) and A.5.1 (IMU outlier / burst handling). ROS-free so the logic
// is unit-testable; the node wrapper only marshals messages.
//
// Threading model (TD-7):
//   E1 (fast tick, 1 kHz): propagate(), apply_correction(), snapshot().
//   E2 (camera callback, 50 Hz): prepare_correction().
// E1 is the sole writer of the live state and the ring buffer. E2 reads ring
// entries through a per-entry seqlock, computes the Kalman update at the
// camera's exposure time and returns a Correction; E1 applies it by replaying
// its own ring forward. No mutex anywhere; no heap allocation after construction.
#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace pat_estimator
{

struct Vec2
{
  double x{0.0};
  double y{0.0};
};

// State x = [ex, ey, vx, vy]: residual LOS error (urad) and its unmodelled
// drift rate (urad/s). Covariance is row-major 4x4.
using Vec4 = std::array<double, 4>;
using Mat4 = std::array<double, 16>;

enum class FusionMode : uint8_t
{
  kReplay,           // TD-4 option (c): correct at exposure time, replay forward
  kApplyAtArrival,   // TD-4 option (a): measured baseline, demonstrably wrong under motion
};

enum class SensorMode : uint8_t
{
  kImuAndCamera = 0,
  kCameraOnly = 1,   // IMU rejected (burst) or absent
  kImuOnly = 2,      // coasting: camera older than camera_coast_age_ms
};

struct Params
{
  FusionMode fusion_mode{FusionMode::kReplay};

  // Process noise. [A] values from A.0.2.
  double gyro_noise_urad_s_sqrt_hz{50.0};
  double drift_rate_random_walk_urad_s_sqrt_s{10.0};
  double camera_only_q_scale{16.0};

  // Camera measurement. [A] A.0.2; gates from A.3.3 item 3.
  double camera_noise_urad{5.0};
  double camera_confidence_floor{0.05};
  double camera_outlier_gate_sigma{3.0};
  double camera_outlier_min_confidence{0.6};
  double camera_future_tolerance_ms{5.0};

  // Validity / mode thresholds. A.2.5, A.5.1, CD-6.
  double camera_coast_age_ms{100.0};
  double camera_valid_age_ms{200.0};
  double imu_valid_age_ms{5.0};
  double pos_var_valid_urad2{1.0e5};

  // IMU gating. A.5.1: absolute rate limit 10 rad/s [A]. The per-sample step
  // limit replaces the "6 sigma vs prediction" wording because real vibration
  // (200 Hz, 100 urad) legitimately changes rate by ~0.16 rad/s per ms. [A]
  double imu_max_rate_rad_s{10.0};
  double imu_max_step_rad_s{1.0};
  int imu_burst_count{5};
  double imu_burst_window_ms{100.0};
  double imu_recover_ms{1000.0};

  double max_dt_s{0.010};
  double initial_pos_sigma_urad{2000.0};
  double initial_vel_sigma_urad_s{500.0};
};

struct CameraSample
{
  int64_t stamp_ns{0};   // exposure mid-time
  Vec2 z;                // urad, optical frame
  double confidence{0.0};
  bool valid{false};
};

// Output of prepare_correction(): the posterior state at the camera exposure
// time t_k, which lies inside ring step `entry_index` at fraction `alpha`.
struct Correction
{
  int64_t camera_stamp_ns{0};
  uint64_t entry_index{0};
  double alpha{1.0};
  Vec4 x{};
  Mat4 P{};
};

struct Counters
{
  uint32_t camera_accepted{0};
  uint32_t camera_rejected_invalid{0};
  uint32_t camera_rejected_too_old{0};
  uint32_t camera_rejected_out_of_order{0};
  uint32_t camera_rejected_future{0};
  uint32_t camera_rejected_outlier{0};
  uint32_t camera_rejected_overrun{0};
  uint32_t imu_rejected{0};
  uint32_t imu_dt_clamped{0};
};

struct Snapshot
{
  int64_t stamp_ns{0};
  Vec4 x{};
  Mat4 P{};
  float camera_age_ms{0.0f};
  float imu_age_ms{0.0f};
  SensorMode mode{SensorMode::kImuAndCamera};
  bool valid{false};
  Counters counters;
};

class ResidualLosEstimator
{
public:
  static constexpr std::size_t kRingSize = 128;   // >= 100 ms at 1 kHz (TD-4)

  explicit ResidualLosEstimator(const Params & params);

  // --- E1 ---------------------------------------------------------------
  // One propagation step to `stamp_ns`. `omega_rad_s` is the platform
  // angular rate in the optical frame (axes matching ex, ey); pass
  // imu_present=false from the IMU-loss watchdog. `fsm_pos_urad` is the
  // measured mirror position (AD-3 "with real hardware" variant).
  void propagate(
    int64_t stamp_ns, const Vec2 & omega_rad_s, bool imu_present,
    const Vec2 & gimbal_rate_rad_s, const Vec2 & fsm_pos_urad, bool fsm_fresh);

  // Replay the ring forward from the correction and adopt the result as the
  // live state. Returns false (and counts) if the ring has already wrapped
  // past the correction's entry.
  bool apply_correction(const Correction & c);

  Snapshot snapshot(int64_t now_ns) const;

  // --- E2 ---------------------------------------------------------------
  // Gate the sample, locate the state at its stamp, run the Kalman update
  // there. Never blocks E1; bounded by kRingSize seqlock reads.
  std::optional<Correction> prepare_correction(const CameraSample & s);

  Counters counters() const;
  const Params & params() const {return p_;}

private:
  struct Entry
  {
    int64_t stamp_ns{0};
    double dt{0.0};
    double q_scale{1.0};
    Vec2 u_dt;    // input increment applied over this step, urad
    Vec4 x{};     // state after this step
    Mat4 P{};
  };
  struct Slot
  {
    std::atomic<uint32_t> seq{0};
    Entry e;
  };

  void step(Vec4 & x, Mat4 & P, double dt, const Vec2 & u_dt, double q_scale) const;
  void write_entry(uint64_t idx, const Entry & e);
  bool read_entry(uint64_t idx, Entry & out) const;
  bool imu_sample_ok(int64_t stamp_ns, const Vec2 & omega) const;
  void note_imu_rejection(int64_t stamp_ns);

  Params p_;
  double q_e_{0.0};   // (urad/s)^2 per Hz -> urad^2 per second
  double q_v_{0.0};

  // Ring: written by E1 only, read by E2 through the per-slot seqlock.
  std::array<Slot, kRingSize> ring_;
  std::atomic<uint64_t> write_index_{0};

  // E1-owned live state.
  bool initialised_{false};
  int64_t last_stamp_ns_{0};
  int64_t last_imu_stamp_ns_{0};
  int64_t last_camera_stamp_ns_{0};
  bool have_camera_{false};
  Vec4 x_{};
  Mat4 P_{};
  Vec2 last_omega_;
  Vec2 last_fsm_pos_;
  bool have_fsm_{false};
  bool imu_burst_{false};
  int64_t last_imu_reject_ns_{0};
  std::array<int64_t, 16> imu_reject_ring_{};
  std::size_t imu_reject_head_{0};

  // E2-owned.
  int64_t last_accepted_camera_ns_{INT64_MIN};

  // Shared counters (relaxed atomics; each is written by one thread).
  struct AtomicCounters
  {
    std::atomic<uint32_t> camera_accepted{0};
    std::atomic<uint32_t> camera_rejected_invalid{0};
    std::atomic<uint32_t> camera_rejected_too_old{0};
    std::atomic<uint32_t> camera_rejected_out_of_order{0};
    std::atomic<uint32_t> camera_rejected_future{0};
    std::atomic<uint32_t> camera_rejected_outlier{0};
    std::atomic<uint32_t> camera_rejected_overrun{0};
    std::atomic<uint32_t> imu_rejected{0};
    std::atomic<uint32_t> imu_dt_clamped{0};
  } cnt_;
};

}  // namespace pat_estimator
