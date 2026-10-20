#include "pat_estimator/residual_los_estimator.hpp"

#include <algorithm>
#include <cmath>

namespace pat_estimator
{
namespace
{

constexpr double kUradPerRad = 1.0e6;
constexpr double kNsPerMs = 1.0e6;
constexpr double kNsPerS = 1.0e9;

inline double & at(Mat4 & m, int r, int c) {return m[r * 4 + c];}
inline double at(const Mat4 & m, int r, int c) {return m[r * 4 + c];}

Mat4 mul(const Mat4 & a, const Mat4 & b)
{
  Mat4 r{};
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      double s = 0.0;
      for (int k = 0; k < 4; ++k) {s += at(a, i, k) * at(b, k, j);}
      at(r, i, j) = s;
    }
  }
  return r;
}

Mat4 transpose(const Mat4 & a)
{
  Mat4 r{};
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {at(r, i, j) = at(a, j, i);}
  }
  return r;
}

Mat4 identity()
{
  Mat4 r{};
  for (int i = 0; i < 4; ++i) {at(r, i, i) = 1.0;}
  return r;
}

inline double sq(double v) {return v * v;}

}  // namespace

ResidualLosEstimator::ResidualLosEstimator(const Params & params)
: p_(params),
  q_e_(sq(params.gyro_noise_urad_s_sqrt_hz)),
  q_v_(sq(params.drift_rate_random_walk_urad_s_sqrt_s))
{
}

// x' = F x + u ; P' = F P F^T + Q with F = [[I, dt I], [0, I]].
void ResidualLosEstimator::step(
  Vec4 & x, Mat4 & P, double dt, const Vec2 & u_dt, double q_scale) const
{
  x[0] += x[2] * dt + u_dt.x;
  x[1] += x[3] * dt + u_dt.y;

  Mat4 F = identity();
  at(F, 0, 2) = dt;
  at(F, 1, 3) = dt;
  P = mul(mul(F, P), transpose(F));
  const double qe = q_e_ * q_scale * dt;
  const double qv = q_v_ * dt;
  at(P, 0, 0) += qe;
  at(P, 1, 1) += qe;
  at(P, 2, 2) += qv;
  at(P, 3, 3) += qv;
}

void ResidualLosEstimator::write_entry(uint64_t idx, const Entry & e)
{
  Slot & s = ring_[idx % kRingSize];
  const uint32_t seq = s.seq.load(std::memory_order_relaxed);
  s.seq.store(seq + 1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);
  s.e = e;
  std::atomic_thread_fence(std::memory_order_release);
  s.seq.store(seq + 2, std::memory_order_release);
}

bool ResidualLosEstimator::read_entry(uint64_t idx, Entry & out) const
{
  const Slot & s = ring_[idx % kRingSize];
  for (int attempt = 0; attempt < 8; ++attempt) {
    const uint32_t s1 = s.seq.load(std::memory_order_acquire);
    if (s1 & 1u) {continue;}
    out = s.e;
    std::atomic_thread_fence(std::memory_order_acquire);
    const uint32_t s2 = s.seq.load(std::memory_order_relaxed);
    const uint64_t w = write_index_.load(std::memory_order_acquire);
    // Slot still holds entry idx iff the writer has not advanced kRingSize past it.
    if (s1 == s2 && idx < w && w - idx <= kRingSize) {return true;}
    if (idx >= w || w - idx > kRingSize) {return false;}
  }
  return false;
}

bool ResidualLosEstimator::imu_sample_ok(int64_t, const Vec2 & w) const
{
  const double mag = std::hypot(w.x, w.y);
  if (!(mag <= p_.imu_max_rate_rad_s)) {return false;}   // also rejects NaN
  const double step_mag = std::hypot(w.x - last_omega_.x, w.y - last_omega_.y);
  return step_mag <= p_.imu_max_step_rad_s;
}

void ResidualLosEstimator::note_imu_rejection(int64_t stamp_ns)
{
  cnt_.imu_rejected.fetch_add(1, std::memory_order_relaxed);
  last_imu_reject_ns_ = stamp_ns;
  imu_reject_ring_[imu_reject_head_ % imu_reject_ring_.size()] = stamp_ns;
  ++imu_reject_head_;
  const std::size_t n = static_cast<std::size_t>(std::max(1, p_.imu_burst_count));
  if (imu_reject_head_ >= n && n <= imu_reject_ring_.size()) {
    const int64_t oldest = imu_reject_ring_[(imu_reject_head_ - n) % imu_reject_ring_.size()];
    if (static_cast<double>(stamp_ns - oldest) <= p_.imu_burst_window_ms * kNsPerMs) {
      imu_burst_ = true;
    }
  }
}

void ResidualLosEstimator::propagate(
  int64_t stamp_ns, const Vec2 & omega, bool imu_present,
  const Vec2 & gimbal_rate, const Vec2 & fsm_pos, bool fsm_fresh)
{
  if (!initialised_) {
    x_ = {};
    P_ = {};
    at(P_, 0, 0) = at(P_, 1, 1) = sq(p_.initial_pos_sigma_urad);
    at(P_, 2, 2) = at(P_, 3, 3) = sq(p_.initial_vel_sigma_urad_s);
    last_stamp_ns_ = stamp_ns;
    last_imu_stamp_ns_ = imu_present ? stamp_ns : INT64_MIN / 2;
    last_omega_ = imu_present ? omega : Vec2{};
    if (fsm_fresh) {last_fsm_pos_ = fsm_pos; have_fsm_ = true;}
    Entry e;
    e.stamp_ns = stamp_ns;
    e.x = x_;
    e.P = P_;
    write_entry(write_index_.load(std::memory_order_relaxed), e);
    write_index_.fetch_add(1, std::memory_order_release);
    initialised_ = true;
    return;
  }

  double dt = static_cast<double>(stamp_ns - last_stamp_ns_) / kNsPerS;
  if (dt <= 0.0) {return;}   // duplicate or backwards stamp: nothing to do
  if (dt > p_.max_dt_s) {
    dt = p_.max_dt_s;
    cnt_.imu_dt_clamped.fetch_add(1, std::memory_order_relaxed);
  }
  last_stamp_ns_ = stamp_ns;

  // IMU gating (A.5.1): single outliers replaced by the previous sample,
  // bursts switch to camera-only propagation until clean for imu_recover_ms.
  // Trapezoidal rate integration: rectangular integration lags the true
  // angle by half a sample (~11 urad at 37 Hz / 100 urad vibration).
  Vec2 w = last_omega_;
  if (imu_present) {
    last_imu_stamp_ns_ = stamp_ns;
    if (imu_sample_ok(stamp_ns, omega)) {
      w.x = 0.5 * (last_omega_.x + omega.x);
      w.y = 0.5 * (last_omega_.y + omega.y);
      last_omega_ = omega;
    } else {
      note_imu_rejection(stamp_ns);
    }
    if (imu_burst_ &&
      static_cast<double>(stamp_ns - last_imu_reject_ns_) > p_.imu_recover_ms * kNsPerMs)
    {
      imu_burst_ = false;
    }
  }
  const bool camera_only = imu_burst_ || !imu_present;
  if (camera_only) {w = Vec2{};}

  // A.0.3: de = -omega_plat - gimbal_rate_cmd - d(theta_fsm).
  Vec2 u_dt;
  u_dt.x = -(w.x + gimbal_rate.x) * kUradPerRad * dt;
  u_dt.y = -(w.y + gimbal_rate.y) * kUradPerRad * dt;
  if (fsm_fresh) {
    if (have_fsm_) {
      u_dt.x -= fsm_pos.x - last_fsm_pos_.x;
      u_dt.y -= fsm_pos.y - last_fsm_pos_.y;
    }
    last_fsm_pos_ = fsm_pos;
    have_fsm_ = true;
  }

  const double q_scale = camera_only ? p_.camera_only_q_scale : 1.0;
  step(x_, P_, dt, u_dt, q_scale);

  Entry e;
  e.stamp_ns = stamp_ns;
  e.dt = dt;
  e.q_scale = q_scale;
  e.u_dt = u_dt;
  e.x = x_;
  e.P = P_;
  write_entry(write_index_.load(std::memory_order_relaxed), e);
  write_index_.fetch_add(1, std::memory_order_release);
}

std::optional<Correction> ResidualLosEstimator::prepare_correction(const CameraSample & s)
{
  if (!s.valid || !std::isfinite(s.z.x) || !std::isfinite(s.z.y)) {
    cnt_.camera_rejected_invalid.fetch_add(1, std::memory_order_relaxed);
    return std::nullopt;
  }
  const uint64_t w = write_index_.load(std::memory_order_acquire);
  Entry newest;
  if (w == 0 || !read_entry(w - 1, newest)) {
    cnt_.camera_rejected_too_old.fetch_add(1, std::memory_order_relaxed);
    return std::nullopt;
  }

  int64_t t_k = s.stamp_ns;
  if (p_.fusion_mode == FusionMode::kApplyAtArrival) {
    t_k = newest.stamp_ns;
  } else if (static_cast<double>(t_k - newest.stamp_ns) > p_.camera_future_tolerance_ms * kNsPerMs) {
    cnt_.camera_rejected_future.fetch_add(1, std::memory_order_relaxed);
    return std::nullopt;
  } else if (t_k > newest.stamp_ns) {
    t_k = newest.stamp_ns;
  }
  if (s.stamp_ns <= last_accepted_camera_ns_) {
    cnt_.camera_rejected_out_of_order.fetch_add(1, std::memory_order_relaxed);
    return std::nullopt;
  }

  // Locate the ring step containing t_k: entry a with stamp <= t_k, scanning
  // back from the newest. Bounded by kRingSize validated reads.
  uint64_t a_idx = w - 1;
  Entry a = newest;
  while (a.stamp_ns > t_k) {
    if (a_idx == 0 || !read_entry(a_idx - 1, a)) {
      cnt_.camera_rejected_too_old.fetch_add(1, std::memory_order_relaxed);
      return std::nullopt;
    }
    --a_idx;
  }

  Correction c;
  c.camera_stamp_ns = s.stamp_ns;
  if (a_idx == w - 1) {
    c.entry_index = a_idx;
    c.alpha = 1.0;
    c.x = a.x;
    c.P = a.P;
  } else {
    Entry b;
    if (!read_entry(a_idx + 1, b)) {
      cnt_.camera_rejected_too_old.fetch_add(1, std::memory_order_relaxed);
      return std::nullopt;
    }
    const double span = static_cast<double>(b.stamp_ns - a.stamp_ns);
    const double alpha = span > 0.0 ? static_cast<double>(t_k - a.stamp_ns) / span : 1.0;
    c.entry_index = a_idx + 1;
    c.alpha = alpha;
    for (int i = 0; i < 4; ++i) {c.x[i] = a.x[i] + alpha * (b.x[i] - a.x[i]);}
    c.P = a.P;
    Vec4 dummy = a.x;
    step(dummy, c.P, alpha * b.dt, Vec2{}, b.q_scale);
  }

  // Kalman update with H = [I 0]; R scaled by confidence.
  const double conf = std::clamp(s.confidence, p_.camera_confidence_floor, 1.0);
  const double r = sq(p_.camera_noise_urad / conf);
  const double nu0 = s.z.x - c.x[0];
  const double nu1 = s.z.y - c.x[1];
  const double s00 = at(c.P, 0, 0) + r, s01 = at(c.P, 0, 1), s11 = at(c.P, 1, 1) + r;
  const double det = s00 * s11 - s01 * s01;
  if (!(det > 0.0)) {
    cnt_.camera_rejected_invalid.fetch_add(1, std::memory_order_relaxed);
    return std::nullopt;
  }
  const double i00 = s11 / det, i01 = -s01 / det, i11 = s00 / det;
  const double d2 = nu0 * (i00 * nu0 + i01 * nu1) + nu1 * (i01 * nu0 + i11 * nu1);
  if (d2 > sq(p_.camera_outlier_gate_sigma) && s.confidence < p_.camera_outlier_min_confidence) {
    cnt_.camera_rejected_outlier.fetch_add(1, std::memory_order_relaxed);
    return std::nullopt;
  }

  // K = P H^T S^-1 (4x2).
  double K[4][2];
  for (int i = 0; i < 4; ++i) {
    const double p0 = at(c.P, i, 0), p1 = at(c.P, i, 1);
    K[i][0] = p0 * i00 + p1 * i01;
    K[i][1] = p0 * i01 + p1 * i11;
  }
  for (int i = 0; i < 4; ++i) {c.x[i] += K[i][0] * nu0 + K[i][1] * nu1;}

  // Joseph form: P = (I-KH) P (I-KH)^T + K R K^T.
  Mat4 ikh = identity();
  for (int i = 0; i < 4; ++i) {
    at(ikh, i, 0) -= K[i][0];
    at(ikh, i, 1) -= K[i][1];
  }
  c.P = mul(mul(ikh, c.P), transpose(ikh));
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      at(c.P, i, j) += r * (K[i][0] * K[j][0] + K[i][1] * K[j][1]);
    }
  }

  last_accepted_camera_ns_ = s.stamp_ns;
  return c;
}

bool ResidualLosEstimator::apply_correction(const Correction & c)
{
  const uint64_t w = write_index_.load(std::memory_order_relaxed);
  if (c.entry_index >= w || w - c.entry_index > kRingSize) {
    cnt_.camera_rejected_overrun.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  Vec4 x = c.x;
  Mat4 P = c.P;

  // Finish the partial step the camera stamp fell inside, then replay every
  // later step with its stored input increment, rewriting the ring so a
  // subsequent correction interpolates against corrected history.
  for (uint64_t i = c.entry_index; i < w; ++i) {
    Entry e = ring_[i % kRingSize].e;   // E1 is the only writer: direct read is safe
    const double frac = (i == c.entry_index) ? (1.0 - c.alpha) : 1.0;
    step(x, P, e.dt * frac, Vec2{e.u_dt.x * frac, e.u_dt.y * frac}, e.q_scale);
    e.x = x;
    e.P = P;
    write_entry(i, e);
  }
  x_ = x;
  P_ = P;
  last_camera_stamp_ns_ = c.camera_stamp_ns;
  have_camera_ = true;
  cnt_.camera_accepted.fetch_add(1, std::memory_order_relaxed);
  return true;
}

Snapshot ResidualLosEstimator::snapshot(int64_t now_ns) const
{
  Snapshot s;
  s.stamp_ns = last_stamp_ns_;
  s.x = x_;
  s.P = P_;
  const double kNoData = 1.0e6;
  s.camera_age_ms = have_camera_ ?
    static_cast<float>(static_cast<double>(now_ns - last_camera_stamp_ns_) / kNsPerMs) :
    static_cast<float>(kNoData);
  const bool imu_fresh = last_imu_stamp_ns_ > INT64_MIN / 4 &&
    static_cast<double>(now_ns - last_imu_stamp_ns_) <= p_.imu_valid_age_ms * kNsPerMs;
  s.imu_age_ms = last_imu_stamp_ns_ > INT64_MIN / 4 ?
    static_cast<float>(static_cast<double>(now_ns - last_imu_stamp_ns_) / kNsPerMs) :
    static_cast<float>(kNoData);
  const bool camera_only = imu_burst_ || !imu_fresh;
  s.mode = camera_only ? SensorMode::kCameraOnly :
    (s.camera_age_ms > p_.camera_coast_age_ms ? SensorMode::kImuOnly :
    SensorMode::kImuAndCamera);
  const double pos_var = at(P_, 0, 0) + at(P_, 1, 1);
  s.valid = initialised_ && have_camera_ &&
    s.camera_age_ms < p_.camera_valid_age_ms &&
    pos_var < p_.pos_var_valid_urad2;
  s.counters = counters();
  return s;
}

Counters ResidualLosEstimator::counters() const
{
  Counters c;
  c.camera_accepted = cnt_.camera_accepted.load(std::memory_order_relaxed);
  c.camera_rejected_invalid = cnt_.camera_rejected_invalid.load(std::memory_order_relaxed);
  c.camera_rejected_too_old = cnt_.camera_rejected_too_old.load(std::memory_order_relaxed);
  c.camera_rejected_out_of_order =
    cnt_.camera_rejected_out_of_order.load(std::memory_order_relaxed);
  c.camera_rejected_future = cnt_.camera_rejected_future.load(std::memory_order_relaxed);
  c.camera_rejected_outlier = cnt_.camera_rejected_outlier.load(std::memory_order_relaxed);
  c.camera_rejected_overrun = cnt_.camera_rejected_overrun.load(std::memory_order_relaxed);
  c.imu_rejected = cnt_.imu_rejected.load(std::memory_order_relaxed);
  c.imu_dt_clamped = cnt_.imu_dt_clamped.load(std::memory_order_relaxed);
  return c;
}

}  // namespace pat_estimator
