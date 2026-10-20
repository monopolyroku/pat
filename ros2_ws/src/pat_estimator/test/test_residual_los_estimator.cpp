// Unit tests for ResidualLosEstimator against a scripted truth model built
// directly from the measurement equation in design_part_a.md A.0.3:
//   e_cam(t) = theta_tgt - theta_plat(t) - theta_g(t) - theta_f(t)
// The IMU reports d(theta_plat)/dt; the camera reports e_cam at exposure
// time t_k, delivered `delay` later. Angles urad, rates rad/s.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <functional>
#include <new>
#include <random>
#include <thread>
#include <utility>

#include <gtest/gtest.h>

#include "pat_estimator/residual_los_estimator.hpp"

using pat_estimator::CameraSample;
using pat_estimator::Correction;
using pat_estimator::FusionMode;
using pat_estimator::Params;
using pat_estimator::ResidualLosEstimator;
using pat_estimator::SensorMode;
using pat_estimator::Vec2;

// ---- allocation audit hooks -------------------------------------------------
static std::atomic<uint64_t> g_allocs{0};
void * operator new(std::size_t n)
{
  g_allocs.fetch_add(1, std::memory_order_relaxed);
  if (void * p = std::malloc(n)) {return p;}
  throw std::bad_alloc();
}
void operator delete(void * p) noexcept {std::free(p);}
void operator delete(void * p, std::size_t) noexcept {std::free(p);}

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr int64_t kMs = 1'000'000;
constexpr int64_t kTick = 1 * kMs;
constexpr double kDegPerS = kPi / 180.0;

struct Scenario
{
  double e0_urad{300.0};                 // target offset at t=0
  double platform_rate_rad_s{0.0};       // constant base motion
  double vib_amp_urad{0.0};              // sinusoidal vibration
  double vib_freq_hz{0.0};
  double gimbal_rate_rad_s{0.0};         // commanded AND executed (perfect servo)
  std::function<double(double)> fsm_pos_urad = [](double) {return 0.0;};
  double camera_delay_ms{25.0};
  double camera_jitter_ms{0.0};          // uniform +/- jitter
  double camera_noise_urad{0.0};
  double camera_confidence{0.9};
  std::function<bool(double)> camera_on = [](double) {return true;};
  std::function<Vec2(double, Vec2)> imu_tamper = [](double, Vec2 w) {return w;};
  unsigned seed{1};
};

struct RunResult
{
  double rms_err_after_1s{0.0};
  double max_err_after_1s{0.0};
  double final_err{0.0};
  pat_estimator::Snapshot last;
};

class TruthSim
{
public:
  TruthSim(const Scenario & sc, const Params & p)
  : sc_(sc), est_(p), rng_(sc.seed) {}

  double theta_plat(double t) const
  {
    return sc_.platform_rate_rad_s * 1e6 * t + sc_.vib_amp_urad * std::sin(2 * kPi * sc_.vib_freq_hz * t);
  }
  double omega_plat(double t) const
  {
    return sc_.platform_rate_rad_s +
           sc_.vib_amp_urad * 1e-6 * 2 * kPi * sc_.vib_freq_hz * std::cos(2 * kPi * sc_.vib_freq_hz * t);
  }
  double e_true(double t) const
  {
    return sc_.e0_urad - theta_plat(t) - sc_.gimbal_rate_rad_s * 1e6 * t - sc_.fsm_pos_urad(t);
  }

  // Advance one 1 ms tick: propagate, then deliver any camera frames due.
  void tick()
  {
    const double t = static_cast<double>(t_ns_) * 1e-9;
    Vec2 w{omega_plat(t), 0.0};
    w = sc_.imu_tamper(t, w);
    est_.propagate(t_ns_, w, true, Vec2{sc_.gimbal_rate_rad_s, 0.0}, Vec2{sc_.fsm_pos_urad(t), 0.0}, true);

    if (t_ns_ % (20 * kMs) == 0 && sc_.camera_on(t)) {
      std::uniform_real_distribution<double> jit(-sc_.camera_jitter_ms, sc_.camera_jitter_ms);
      std::normal_distribution<double> noise(0.0, sc_.camera_noise_urad);
      CameraSample s;
      s.stamp_ns = t_ns_;
      s.z = Vec2{e_true(t) + (sc_.camera_noise_urad > 0 ? noise(rng_) : 0.0), 0.0};
      s.confidence = sc_.camera_confidence;
      s.valid = true;
      const int64_t arrival = t_ns_ + static_cast<int64_t>((sc_.camera_delay_ms + jit(rng_)) * kMs);
      pending_.push_back({arrival, s});
    }
    for (auto it = pending_.begin(); it != pending_.end(); ) {
      if (it->first <= t_ns_) {
        deliver(it->second);
        it = pending_.erase(it);
      } else {
        ++it;
      }
    }
    t_ns_ += kTick;
  }

  void deliver(const CameraSample & s)
  {
    if (auto c = est_.prepare_correction(s)) {est_.apply_correction(*c);}
  }

  RunResult run(double seconds)
  {
    double sum2 = 0.0, mx = 0.0;
    int n = 0;
    RunResult r;
    while (static_cast<double>(t_ns_) * 1e-9 < seconds) {
      tick();
      const double t = static_cast<double>(t_ns_ - kTick) * 1e-9;
      const double err = est_.snapshot(t_ns_ - kTick).x[0] - e_true(t);
      r.final_err = err;
      if (t >= 1.0) {
        sum2 += err * err;
        mx = std::max(mx, std::fabs(err));
        ++n;
      }
    }
    r.rms_err_after_1s = n ? std::sqrt(sum2 / n) : 0.0;
    r.max_err_after_1s = mx;
    r.last = est_.snapshot(t_ns_ - kTick);
    return r;
  }

  ResidualLosEstimator & est() {return est_;}
  int64_t now_ns() const {return t_ns_;}

private:
  Scenario sc_;
  ResidualLosEstimator est_;
  std::mt19937 rng_;
  int64_t t_ns_{0};
  std::deque<std::pair<int64_t, CameraSample>> pending_;
};

Params params(FusionMode m = FusionMode::kReplay)
{
  Params p;
  p.fusion_mode = m;
  return p;
}

}  // namespace

// ---- fusion correctness -----------------------------------------------------

TEST(Fusion, StaticPlatformReplayEqualsApplyAtArrival)
{
  Scenario sc;
  const auto a = TruthSim(sc, params(FusionMode::kReplay)).run(3.0);
  const auto b = TruthSim(sc, params(FusionMode::kApplyAtArrival)).run(3.0);
  // Without motion the two fusion modes differ only by the covariance being
  // evaluated at t_k vs now (slightly different gain); both are numerically exact.
  EXPECT_LT(a.rms_err_after_1s, 1e-3);
  EXPECT_LT(b.rms_err_after_1s, 1e-3);
  EXPECT_NEAR(a.final_err, b.final_err, 1e-3);
}

TEST(Fusion, OneDegPerSecondMotionExposesApplyAtArrival)
{
  Scenario sc;
  sc.platform_rate_rad_s = 1.0 * kDegPerS;
  sc.camera_delay_ms = 25.0;
  const auto replay = TruthSim(sc, params(FusionMode::kReplay)).run(3.0);
  const auto naive = TruthSim(sc, params(FusionMode::kApplyAtArrival)).run(3.0);

  // TD-4: rate x delay = 17453 urad/s x 25 ms = 436 urad.
  const double expected_bias = 1.0 * kDegPerS * 1e6 * 0.025;
  EXPECT_NEAR(naive.rms_err_after_1s, expected_bias, 0.15 * expected_bias);
  EXPECT_LT(replay.rms_err_after_1s, 5.0);
  EXPECT_LT(replay.max_err_after_1s, 10.0);
}

TEST(Fusion, JitteredDelayWithVibrationStaysAccurate)
{
  Scenario sc;
  sc.platform_rate_rad_s = 0.5 * kDegPerS;
  sc.vib_amp_urad = 100.0;
  sc.vib_freq_hz = 37.0;
  sc.camera_delay_ms = 25.0;
  sc.camera_jitter_ms = 5.0;
  sc.camera_noise_urad = 5.0;
  const auto r = TruthSim(sc, params()).run(4.0);
  EXPECT_LT(r.rms_err_after_1s, 6.0) << "replay must be insensitive to +/-5 ms delay jitter";
  EXPECT_EQ(r.last.counters.camera_rejected_too_old, 0u);
  EXPECT_EQ(r.last.counters.camera_rejected_overrun, 0u);
  EXPECT_TRUE(r.last.valid);
}

TEST(Fusion, OutOfOrderFramesAreRejectedNotFused)
{
  Scenario sc;
  sc.platform_rate_rad_s = 0.5 * kDegPerS;
  sc.camera_jitter_ms = 15.0;   // larger than the 20 ms frame period => reordering
  const auto r = TruthSim(sc, params()).run(4.0);
  EXPECT_GT(r.last.counters.camera_rejected_out_of_order, 0u);
  EXPECT_LT(r.rms_err_after_1s, 5.0);
}

TEST(Fusion, GimbalCommandIsFedForwardWithoutCamera)
{
  Scenario sc;
  sc.gimbal_rate_rad_s = 0.1 * kDegPerS;                 // RECENTRE rate limit (A.2.6)
  sc.camera_on = [](double t) {return t < 1.5;};        // then coast on feedforward
  const auto r = TruthSim(sc, params()).run(2.5);
  EXPECT_LT(std::fabs(r.final_err), 5.0) << "1 s of commanded gimbal motion (1745 urad) must be anticipated";
  EXPECT_FALSE(r.last.valid) << "but validity must have expired (camera age > 200 ms)";
}

TEST(Fusion, MeasuredFsmMotionIsSubtracted)
{
  Scenario sc;
  sc.fsm_pos_urad = [](double t) {return 800.0 * std::sin(2 * kPi * 3.0 * t);};
  const auto r = TruthSim(sc, params()).run(3.0);
  EXPECT_LT(r.rms_err_after_1s, 5.0);
}

// ---- gates ------------------------------------------------------------------

TEST(Gates, StaleFutureInvalidAndOutlierFrames)
{
  Scenario sc;
  TruthSim sim(sc, params());
  sim.run(1.0);
  auto & est = sim.est();
  const auto before = est.snapshot(sim.now_ns());

  CameraSample s;
  s.valid = true;
  s.confidence = 0.9;
  s.z = Vec2{before.x[0], 0.0};

  s.stamp_ns = sim.now_ns() - 200 * kMs;        // older than the last accepted frame
  EXPECT_FALSE(est.prepare_correction(s).has_value());
  EXPECT_EQ(est.counters().camera_rejected_out_of_order, 1u);

  {
    // Older than the 128 ms ring, on an estimator that has never accepted a frame.
    ResidualLosEstimator fresh(params());
    for (int64_t t = 0; t <= 300 * kMs; t += kTick) {
      fresh.propagate(t, Vec2{}, true, Vec2{}, Vec2{}, true);
    }
    CameraSample old = s;
    old.stamp_ns = 100 * kMs;
    EXPECT_FALSE(fresh.prepare_correction(old).has_value());
    EXPECT_EQ(fresh.counters().camera_rejected_too_old, 1u);
  }

  s.stamp_ns = sim.now_ns() + 50 * kMs;         // beyond future tolerance
  EXPECT_FALSE(est.prepare_correction(s).has_value());
  EXPECT_EQ(est.counters().camera_rejected_future, 1u);

  s.stamp_ns = sim.now_ns() - 10 * kMs;
  s.valid = false;
  EXPECT_FALSE(est.prepare_correction(s).has_value());
  EXPECT_EQ(est.counters().camera_rejected_invalid, 1u);

  // 3-sigma outlier with low confidence: dropped. Same jump with high
  // confidence: believed (A.3.3 item 3).
  s.valid = true;
  s.z.x = before.x[0] + 1000.0;
  s.confidence = 0.3;
  EXPECT_FALSE(est.prepare_correction(s).has_value());
  EXPECT_EQ(est.counters().camera_rejected_outlier, 1u);

  s.stamp_ns += 1 * kMs;
  s.confidence = 0.95;
  auto c = est.prepare_correction(s);
  ASSERT_TRUE(c.has_value());
  EXPECT_GT(c->x[0], before.x[0] + 300.0);
  EXPECT_TRUE(est.apply_correction(*c));
  EXPECT_GT(est.snapshot(sim.now_ns()).x[0], before.x[0] + 300.0);
}

TEST(Gates, CameraGapGrowsCovarianceAndExpiresValidity)
{
  Scenario sc;
  sc.camera_on = [](double t) {return t < 1.0 || t > 1.5;};
  TruthSim sim(sc, params());
  sim.run(1.05);
  const auto s0 = sim.est().snapshot(sim.now_ns());
  EXPECT_TRUE(s0.valid);
  EXPECT_EQ(s0.mode, SensorMode::kImuAndCamera);

  sim.run(1.15);   // camera age ~150 ms: degraded coasting, still valid
  const auto s1 = sim.est().snapshot(sim.now_ns());
  EXPECT_TRUE(s1.valid);
  EXPECT_EQ(s1.mode, SensorMode::kImuOnly);
  EXPECT_GT(s1.P[0], s0.P[0]);

  sim.run(1.25);   // camera age ~250 ms: invalid (CD-6)
  const auto s2 = sim.est().snapshot(sim.now_ns());
  EXPECT_FALSE(s2.valid);
  EXPECT_GT(s2.camera_age_ms, 200.0f);
  EXPECT_GT(s2.P[0], s1.P[0]);

  sim.run(1.6);    // camera back
  const auto s3 = sim.est().snapshot(sim.now_ns());
  EXPECT_TRUE(s3.valid);
  EXPECT_LT(s3.P[0], s2.P[0]);
}

TEST(Gates, CorrectionOlderThanRingIsCountedAsOverrun)
{
  Scenario sc;
  TruthSim sim(sc, params());
  sim.run(1.0);
  CameraSample s;
  s.valid = true;
  s.confidence = 0.9;
  s.stamp_ns = sim.now_ns() - 10 * kMs;
  s.z = Vec2{sim.est().snapshot(sim.now_ns()).x[0], 0.0};
  auto c = sim.est().prepare_correction(s);
  ASSERT_TRUE(c.has_value());
  sim.run(1.3);   // 300 ms of new entries: the ring has wrapped past c
  EXPECT_FALSE(sim.est().apply_correction(*c));
  EXPECT_EQ(sim.est().counters().camera_rejected_overrun, 1u);
}

// ---- IMU handling -------------------------------------------------------------

TEST(Imu, SingleSpikeIsDroppedWithoutDisturbingEstimate)
{
  Scenario sc;
  sc.platform_rate_rad_s = 0.2 * kDegPerS;
  sc.imu_tamper = [](double t, Vec2 w) {
      if (std::fabs(t - 2.0) < 0.0005) {w.x = 20.0;}   // one 20 rad/s sample
      return w;
    };
  const auto r = TruthSim(sc, params()).run(3.0);
  EXPECT_EQ(r.last.counters.imu_rejected, 1u);
  EXPECT_LT(r.max_err_after_1s, 5.0);
  EXPECT_EQ(r.last.mode, SensorMode::kImuAndCamera);
}

TEST(Imu, BurstSwitchesToCameraOnlyThenRecovers)
{
  Scenario sc;
  sc.imu_tamper = [](double t, Vec2 w) {
      if (t >= 2.0 && t < 2.010) {w.x = 20.0;}   // 10 bad samples in 10 ms
      return w;
    };
  TruthSim sim(sc, params());
  sim.run(2.05);
  auto s = sim.est().snapshot(sim.now_ns());
  EXPECT_GE(s.counters.imu_rejected, 5u);
  EXPECT_EQ(s.mode, SensorMode::kCameraOnly);
  sim.run(3.2);   // > 1 s clean
  s = sim.est().snapshot(sim.now_ns());
  EXPECT_EQ(s.mode, SensorMode::kImuAndCamera);
  EXPECT_TRUE(s.valid);
}

TEST(Imu, AbsentImuIsCameraOnlyMode)
{
  ResidualLosEstimator est(params());
  int64_t t = 0;
  for (int i = 0; i < 50; ++i, t += kTick) {
    est.propagate(t, Vec2{}, /*imu_present=*/ false, Vec2{}, Vec2{}, false);
  }
  const auto s = est.snapshot(t);
  EXPECT_EQ(s.mode, SensorMode::kCameraOnly);
  EXPECT_GT(s.imu_age_ms, 1000.0f);
}

// ---- fast-path audit ----------------------------------------------------------

TEST(FastPath, NoHeapAllocationInSteadyState)
{
  Scenario sc;
  sc.platform_rate_rad_s = 0.5 * kDegPerS;
  TruthSim sim(sc, params());
  sim.run(0.5);
  auto & est = sim.est();

  CameraSample s;
  s.valid = true;
  s.confidence = 0.9;
  s.stamp_ns = sim.now_ns() - 25 * kMs;
  s.z = Vec2{sim.e_true(static_cast<double>(s.stamp_ns) * 1e-9), 0.0};

  const uint64_t before = g_allocs.load();
  for (int i = 0; i < 100; ++i) {
    est.propagate(sim.now_ns() + i * kTick, Vec2{0.01, 0.0}, true, Vec2{}, Vec2{}, true);
  }
  auto c = est.prepare_correction(s);
  ASSERT_TRUE(c.has_value());
  est.apply_correction(*c);
  (void)est.snapshot(sim.now_ns());
  EXPECT_EQ(g_allocs.load(), before) << "propagate/prepare/apply/snapshot must not allocate";
}

TEST(FastPath, ConcurrentCameraCorrectionsNeverCorruptState)
{
  // E1 thread propagates at full speed; E2 thread prepares corrections from
  // true values. The estimate must stay close to truth with no torn reads.
  Params p = params();
  ResidualLosEstimator est(p);
  std::atomic<bool> stop{false};
  std::atomic<int64_t> now{0};
  const double rate = 0.5 * kDegPerS;
  auto truth = [&](int64_t t) {return 300.0 - rate * 1e6 * static_cast<double>(t) * 1e-9;};

  std::atomic<uint32_t> applied{0};
  std::atomic<uint32_t> prepared{0};
  Correction pending{};
  std::atomic<bool> pending_ready{false};

  std::thread e1([&] {
      int64_t t = 0;
      while (!stop.load(std::memory_order_relaxed)) {
        est.propagate(t, Vec2{rate, 0.0}, true, Vec2{}, Vec2{}, true);
        now.store(t, std::memory_order_release);
        if (pending_ready.load(std::memory_order_acquire)) {
          if (est.apply_correction(pending)) {applied.fetch_add(1);}
          pending_ready.store(false, std::memory_order_release);
        }
        t += kTick;
        std::this_thread::sleep_for(std::chrono::microseconds(20));
      }
    });
  std::thread e2([&] {
      while (!stop.load(std::memory_order_relaxed)) {
        const int64_t t = now.load(std::memory_order_acquire);
        if (t > 50 * kMs && !pending_ready.load(std::memory_order_acquire)) {
          CameraSample s;
          s.valid = true;
          s.confidence = 0.9;
          s.stamp_ns = t - 25 * kMs;
          s.z = Vec2{truth(s.stamp_ns), 0.0};
          if (auto c = est.prepare_correction(s)) {
            pending = *c;
            prepared.fetch_add(1);
            pending_ready.store(true, std::memory_order_release);
          }
        }
        std::this_thread::sleep_for(std::chrono::microseconds(400));
      }
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  stop = true;
  e1.join();
  e2.join();

  const int64_t t = now.load();
  const auto s = est.snapshot(t);
  EXPECT_GT(applied.load(), 50u);
  EXPECT_NEAR(s.x[0], truth(t), 10.0);
  EXPECT_TRUE(std::isfinite(s.P[0]));
  EXPECT_EQ(s.counters.camera_rejected_overrun, 0u);
}
