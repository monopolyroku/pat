#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "pat_sim/plant_model.hpp"

using pat_sim::PlantInputs;
using pat_sim::PlantModel;
using pat_sim::PlantParams;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kUradPerDeg = kPi / 180.0 * 1e6;

PlantParams quiet()
{
  PlantParams p;
  p.vib_rms_urad = 0.0;
  p.camera_noise_urad = 0.0;
  p.gyro_noise_urad_s_sqrt_hz = 0.0;
  p.gyro_bias_urad_s = {0.0, 0.0};
  p.gyro_bias_rw_urad_s_sqrt_s = 0.0;
  return p;
}
}  // namespace

TEST(PlantModel, MeasurementEquationHolds)
{
  PlantParams p = quiet();
  p.platform_rate_deg_s = {0.3, -0.1};
  p.target_drift_urad_s = {50.0, 20.0};
  PlantModel m(p);
  PlantInputs in;
  in.fsm_cmd_urad = {400.0, -300.0};
  in.gimbal_cmd_urad = {2000.0, 1000.0};
  for (int i = 0; i < 2000; ++i) {m.step(0.001, in);}
  const auto tr = m.truth();
  for (int k = 0; k < 2; ++k) {
    EXPECT_NEAR(tr.e_cam[k], tr.target[k] - tr.platform[k] - tr.gimbal[k] - tr.fsm[k], 1e-9);
  }
  EXPECT_NEAR(tr.platform[0], 0.3 * kUradPerDeg * 2.0, 1e-6);
  EXPECT_NEAR(tr.platform_rate[0], 0.3 * kPi / 180.0, 1e-12);
  EXPECT_NEAR(tr.fsm[0], 400.0, 1e-6);          // FSM settled on its command
  EXPECT_NEAR(tr.gimbal[0], 2000.0, 1.0);        // gimbal settled (5 Hz, 2 s)
}

TEST(PlantModel, FsmStepResponseMatchesSecondOrderModel)
{
  // zeta = 0.7 => ~4.6 % overshoot; wn = 800 Hz => settle well inside 2 ms.
  PlantModel m(quiet());
  PlantInputs in;
  in.fsm_cmd_urad = {1000.0, 0.0};
  double peak = 0.0;
  const double dt = 1e-5;   // fine step to resolve the peak; exact ZOH so any dt is valid
  for (int i = 0; i < 300; ++i) {
    m.step(dt, in);
    peak = std::max(peak, m.truth().fsm[0]);
  }
  EXPECT_NEAR(peak, 1046.0, 10.0);
  EXPECT_NEAR(m.truth().fsm[0], 1000.0, 1.0);
}

TEST(PlantModel, FsmExactStepIsStableAtOneKilohertz)
{
  // Naive Euler at wn*dt = 5 would diverge; the closed-form step must not.
  PlantModel m(quiet());
  PlantInputs in;
  in.fsm_cmd_urad = {500.0, -500.0};
  for (int i = 0; i < 100; ++i) {m.step(0.001, in);}
  EXPECT_NEAR(m.truth().fsm[0], 500.0, 1e-6);
  EXPECT_NEAR(m.truth().fsm[1], -500.0, 1e-6);
  EXPECT_FALSE(m.truth().fsm_saturated);
}

TEST(PlantModel, FsmSaturatesAtRange)
{
  PlantModel m(quiet());
  PlantInputs in;
  in.fsm_cmd_urad = {5000.0, 0.0};
  for (int i = 0; i < 50; ++i) {m.step(0.001, in);}
  EXPECT_DOUBLE_EQ(m.truth().fsm[0], 2000.0);
  EXPECT_TRUE(m.truth().fsm_saturated);
}

TEST(PlantModel, GimbalRateAndRangeLimits)
{
  PlantModel m(quiet());
  PlantInputs in;
  in.gimbal_cmd_urad = {80.0 * kUradPerDeg, 0.0};   // beyond the +/-60 deg range
  m.step(0.001, in);
  EXPECT_NEAR(m.truth().gimbal_rate[0], 20.0 * kPi / 180.0, 1e-9) << "rate limited to 20 deg/s";
  for (int i = 0; i < 5000; ++i) {m.step(0.001, in);}
  EXPECT_NEAR(m.truth().gimbal[0], 60.0 * kUradPerDeg, 1e-6);
  EXPECT_TRUE(m.truth().gimbal_at_limit);
  EXPECT_DOUBLE_EQ(m.truth().gimbal_rate[0], 0.0);
}

TEST(PlantModel, VibrationRmsAndRateAreConsistent)
{
  PlantParams p = quiet();
  p.vib_rms_urad = 100.0;
  PlantModel m(p);
  double sum2 = 0.0;
  int n = 0;
  const double dt = 1e-4;
  for (double t = 0.0; t < 20.0; t += dt, ++n) {
    sum2 += std::pow(m.platform_at(t)[0], 2);
  }
  EXPECT_NEAR(std::sqrt(sum2 / n), 100.0, 5.0);

  // Analytic rate equals the numerical derivative of the angle.
  const double t0 = 1.2345, h = 1e-7;
  const double num = (m.platform_at(t0 + h)[1] - m.platform_at(t0 - h)[1]) / (2 * h) * 1e-6;
  EXPECT_NEAR(m.platform_rate_at(t0)[1], num, 1e-6);
}

TEST(PlantModel, GyroSampleIsTruthPlusBiasPlusNoise)
{
  PlantParams p = quiet();
  p.platform_rate_deg_s = {1.0, 0.0};
  p.gyro_bias_urad_s = {200.0, 0.0};
  p.gyro_noise_urad_s_sqrt_hz = 50.0;
  PlantModel m(p);
  double sum = 0.0, sum2 = 0.0;
  const int n = 20000;
  for (int i = 0; i < n; ++i) {
    const double w = m.sample_gyro(0.001)[0];
    sum += w;
    sum2 += w * w;
  }
  const double mean = sum / n;
  const double sd = std::sqrt(sum2 / n - mean * mean);
  EXPECT_NEAR(mean, kPi / 180.0 + 200e-6, 50e-6);
  EXPECT_NEAR(sd, 50e-6 * std::sqrt(500.0), 0.1e-3);   // sigma_n * sqrt(fs/2)
}

TEST(PlantModel, CameraSampleIsTruthPlusNoise)
{
  PlantParams p = quiet();
  p.camera_noise_urad = 5.0;
  PlantModel m(p);
  double sum2 = 0.0;
  const int n = 5000;
  for (int i = 0; i < n; ++i) {
    double conf;
    const double z = m.sample_camera(conf)[0];
    sum2 += std::pow(z - m.truth().e_cam[0], 2);
    EXPECT_GE(conf, 0.0);
    EXPECT_LE(conf, 1.0);
  }
  EXPECT_NEAR(std::sqrt(sum2 / n), 5.0, 0.3);
}
