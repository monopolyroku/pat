// Synthetic PAT plant: truth model behind the camera, IMU, FSM and gimbal
// drivers, built directly from design_part_a.md A.0.3:
//
//   e_cam(t) = theta_tgt(t) - theta_plat(t) - theta_g(t) - theta_f(t)
//
// ROS-free so the physics can be unit-tested; synthetic_plant_node wraps it.
// Angles in urad, rates in rad/s, time in seconds. Axis 0 = ex (az), 1 = ey (el).
#pragma once

#include <array>
#include <cstdint>
#include <random>
#include <vector>

namespace pat_sim
{

using Vec2 = std::array<double, 2>;

struct PlantParams
{
  // Target direction in the platform frame: offset plus slow LOS drift.
  Vec2 target_offset_urad{300.0, -200.0};
  Vec2 target_drift_urad_s{0.0, 0.0};

  // Platform base motion: constant rate plus an optional slow sway. [A] A.0.2
  Vec2 platform_rate_deg_s{0.0, 0.0};
  double platform_sway_amp_deg{0.0};
  double platform_sway_freq_hz{0.1};

  // Vibration: sum of log-spaced sinusoids, amplitude falling as 1/f,
  // scaled to the requested total RMS. [A] 1-200 Hz, 100 urad RMS
  double vib_rms_urad{100.0};
  double vib_f_lo_hz{1.0};
  double vib_f_hi_hz{200.0};
  int vib_lines{12};

  // Gimbal: first-order servo with rate and range limits. [A]
  double gimbal_bw_hz{5.0};
  double gimbal_max_rate_deg_s{20.0};
  Vec2 gimbal_range_deg{60.0, 30.0};

  // FSM: second-order low-pass, exact ZOH discretisation. [A]
  double fsm_wn_hz{800.0};
  double fsm_zeta{0.7};
  double fsm_range_urad{2000.0};

  // Sensors. [A]
  double camera_noise_urad{5.0};
  double camera_confidence{0.9};
  double camera_confidence_noise{0.05};
  double gyro_noise_urad_s_sqrt_hz{50.0};
  Vec2 gyro_bias_urad_s{200.0, -150.0};
  double gyro_bias_rw_urad_s_sqrt_s{5.0};

  uint32_t seed{42};
};

struct PlantInputs
{
  Vec2 fsm_cmd_urad{0.0, 0.0};
  Vec2 gimbal_cmd_urad{0.0, 0.0};
};

struct PlantTruth
{
  double t{0.0};
  Vec2 e_cam;           // true residual error, urad
  Vec2 target;
  Vec2 platform;
  Vec2 platform_rate;   // rad/s
  Vec2 gimbal;
  Vec2 gimbal_rate;     // rad/s
  bool gimbal_at_limit{false};
  Vec2 fsm;
  bool fsm_saturated{false};
};

class PlantModel
{
public:
  explicit PlantModel(const PlantParams & p);

  // Advance actuators by dt under the given commands and move the clock.
  void step(double dt, const PlantInputs & in);

  double time() const {return t_;}
  PlantTruth truth() const;

  // Analytic truth at an arbitrary time (actuators excluded).
  Vec2 target_at(double t) const;
  Vec2 platform_at(double t) const;
  Vec2 platform_rate_at(double t) const;

  // Noisy sensor samples at the current time.
  Vec2 sample_gyro(double dt);                     // rad/s, bias + white noise
  Vec2 sample_camera(double & confidence);         // urad

  const PlantParams & params() const {return p_;}

private:
  struct VibLine
  {
    double f_hz;
    double amp_urad;
    Vec2 phase;
  };

  PlantParams p_;
  std::vector<VibLine> vib_;
  std::mt19937 rng_;
  std::normal_distribution<double> gauss_{0.0, 1.0};

  double t_{0.0};
  Vec2 gimbal_pos_{0.0, 0.0};
  Vec2 gimbal_vel_{0.0, 0.0};   // rad/s
  bool gimbal_at_limit_{false};
  Vec2 fsm_pos_{0.0, 0.0};
  Vec2 fsm_vel_{0.0, 0.0};      // urad/s
  bool fsm_saturated_{false};
  Vec2 gyro_bias_;              // urad/s, random-walks
};

}  // namespace pat_sim
