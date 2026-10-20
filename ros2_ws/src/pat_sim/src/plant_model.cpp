#include "pat_sim/plant_model.hpp"

#include <algorithm>
#include <cmath>

namespace pat_sim
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kUradPerRad = 1.0e6;
constexpr double kUradPerDeg = kPi / 180.0 * kUradPerRad;
constexpr double kRadPerDeg = kPi / 180.0;

}  // namespace

PlantModel::PlantModel(const PlantParams & p)
: p_(p), rng_(p.seed), gyro_bias_(p.gyro_bias_urad_s)
{
  // Log-spaced lines with amplitude ~ 1/f, normalised so sum(amp^2 / 2) = rms^2.
  const int n = std::max(1, p_.vib_lines);
  std::uniform_real_distribution<double> ph(0.0, 2.0 * kPi);
  double sum_sq = 0.0;
  for (int i = 0; i < n; ++i) {
    const double frac = n == 1 ? 0.0 : static_cast<double>(i) / (n - 1);
    const double f = p_.vib_f_lo_hz * std::pow(p_.vib_f_hi_hz / p_.vib_f_lo_hz, frac);
    const double a = 1.0 / f;
    vib_.push_back({f, a, {ph(rng_), ph(rng_)}});
    sum_sq += 0.5 * a * a;
  }
  const double scale = p_.vib_rms_urad > 0.0 && sum_sq > 0.0 ? p_.vib_rms_urad / std::sqrt(sum_sq) : 0.0;
  for (auto & l : vib_) {l.amp_urad *= scale;}
}

Vec2 PlantModel::target_at(double t) const
{
  return {p_.target_offset_urad[0] + p_.target_drift_urad_s[0] * t,
    p_.target_offset_urad[1] + p_.target_drift_urad_s[1] * t};
}

Vec2 PlantModel::platform_at(double t) const
{
  Vec2 th;
  const double sway = p_.platform_sway_amp_deg * kUradPerDeg *
    std::sin(2.0 * kPi * p_.platform_sway_freq_hz * t);
  for (int k = 0; k < 2; ++k) {
    th[k] = p_.platform_rate_deg_s[k] * kUradPerDeg * t + sway;
    for (const auto & l : vib_) {
      th[k] += l.amp_urad * std::sin(2.0 * kPi * l.f_hz * t + l.phase[k]);
    }
  }
  return th;
}

Vec2 PlantModel::platform_rate_at(double t) const
{
  Vec2 w;
  const double sway_rate = p_.platform_sway_amp_deg * kRadPerDeg *
    2.0 * kPi * p_.platform_sway_freq_hz * std::cos(2.0 * kPi * p_.platform_sway_freq_hz * t);
  for (int k = 0; k < 2; ++k) {
    w[k] = p_.platform_rate_deg_s[k] * kRadPerDeg + sway_rate;
    for (const auto & l : vib_) {
      w[k] += l.amp_urad / kUradPerRad * 2.0 * kPi * l.f_hz *
        std::cos(2.0 * kPi * l.f_hz * t + l.phase[k]);
    }
  }
  return w;
}

void PlantModel::step(double dt, const PlantInputs & in)
{
  // Gimbal: first-order approach to the command at bandwidth gimbal_bw_hz,
  // rate-limited, range-limited.
  const double k = 2.0 * kPi * p_.gimbal_bw_hz;
  const double vmax = p_.gimbal_max_rate_deg_s * kRadPerDeg;
  gimbal_at_limit_ = false;
  for (int i = 0; i < 2; ++i) {
    double v = k * (in.gimbal_cmd_urad[i] - gimbal_pos_[i]) / kUradPerRad;
    v = std::clamp(v, -vmax, vmax);
    double pos = gimbal_pos_[i] + v * kUradPerRad * dt;
    const double lim = p_.gimbal_range_deg[i] * kUradPerDeg;
    if (pos >= lim || pos <= -lim) {
      pos = std::clamp(pos, -lim, lim);
      v = 0.0;
      gimbal_at_limit_ = true;
    }
    gimbal_pos_[i] = pos;
    gimbal_vel_[i] = v;
  }

  // FSM: exact ZOH step of y'' + 2 zeta wn y' + wn^2 y = 0 with y = x - u.
  const double wn = 2.0 * kPi * p_.fsm_wn_hz;
  const double zeta = std::clamp(p_.fsm_zeta, 0.01, 0.999);
  const double sigma = zeta * wn;
  const double wd = wn * std::sqrt(1.0 - zeta * zeta);
  const double e = std::exp(-sigma * dt);
  const double c = std::cos(wd * dt), s = std::sin(wd * dt);
  fsm_saturated_ = false;
  for (int i = 0; i < 2; ++i) {
    const double y0 = fsm_pos_[i] - in.fsm_cmd_urad[i];
    const double v0 = fsm_vel_[i];
    const double y = e * (y0 * c + (v0 + sigma * y0) / wd * s);
    const double v = e * (v0 * c - (sigma * v0 + wn * wn * y0) / wd * s);
    double pos = y + in.fsm_cmd_urad[i];
    double vel = v;
    if (pos >= p_.fsm_range_urad || pos <= -p_.fsm_range_urad) {
      pos = std::clamp(pos, -p_.fsm_range_urad, p_.fsm_range_urad);
      vel = 0.0;
      fsm_saturated_ = true;
    }
    fsm_pos_[i] = pos;
    fsm_vel_[i] = vel;
  }

  // Gyro bias random walk.
  const double rw = p_.gyro_bias_rw_urad_s_sqrt_s * std::sqrt(std::max(dt, 0.0));
  for (int i = 0; i < 2; ++i) {gyro_bias_[i] += rw * gauss_(rng_);}

  t_ += dt;
}

PlantTruth PlantModel::truth() const
{
  PlantTruth tr;
  tr.t = t_;
  tr.target = target_at(t_);
  tr.platform = platform_at(t_);
  tr.platform_rate = platform_rate_at(t_);
  tr.gimbal = gimbal_pos_;
  tr.gimbal_rate = gimbal_vel_;
  tr.gimbal_at_limit = gimbal_at_limit_;
  tr.fsm = fsm_pos_;
  tr.fsm_saturated = fsm_saturated_;
  for (int i = 0; i < 2; ++i) {
    tr.e_cam[i] = tr.target[i] - tr.platform[i] - tr.gimbal[i] - tr.fsm[i];
  }
  return tr;
}

Vec2 PlantModel::sample_gyro(double dt)
{
  const Vec2 w = platform_rate_at(t_);
  // White noise density sigma_n [urad/s/sqrt(Hz)] sampled at 1/dt => sigma_n * sqrt(1/(2 dt)).
  const double sigma = dt > 0.0 ? p_.gyro_noise_urad_s_sqrt_hz * std::sqrt(0.5 / dt) : 0.0;
  Vec2 out;
  for (int i = 0; i < 2; ++i) {
    out[i] = w[i] + (gyro_bias_[i] + sigma * gauss_(rng_)) / kUradPerRad;
  }
  return out;
}

Vec2 PlantModel::sample_camera(double & confidence)
{
  const PlantTruth tr = truth();
  confidence = std::clamp(p_.camera_confidence + p_.camera_confidence_noise * gauss_(rng_), 0.0, 1.0);
  return {tr.e_cam[0] + p_.camera_noise_urad * gauss_(rng_),
    tr.e_cam[1] + p_.camera_noise_urad * gauss_(rng_)};
}

}  // namespace pat_sim
