#include "pat_estimator/state_estimator_node.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"

#include "pat_common/qos.hpp"
#include "pat_common/topics.hpp"
#include "pat_common/units.hpp"

namespace pat_estimator
{

using namespace std::chrono_literals;
namespace qos = pat_common::qos;
namespace topics = pat_common::topics;
using pat_common::to_ns;

StateEstimatorNode::StateEstimatorNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("state_estimator", options)
{
  // Groups are created with automatically_add_to_executor_with_node=false so
  // the executable decides which executor runs them (A.4.2).
  imu_grp_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  cam_grp_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  declare_parameter<std::string>("fusion_mode", "replay");
  declare_parameter("gyro_noise_urad_s_sqrt_hz", 50.0);
  declare_parameter("drift_rate_random_walk_urad_s_sqrt_s", 10.0);
  declare_parameter("camera_only_q_scale", 16.0);
  declare_parameter("camera_noise_urad", 5.0);
  declare_parameter("camera_confidence_floor", 0.05);
  declare_parameter("camera_outlier_gate_sigma", 3.0);
  declare_parameter("camera_outlier_min_confidence", 0.6);
  declare_parameter("camera_future_tolerance_ms", 5.0);
  declare_parameter("camera_coast_age_ms", 100.0);
  declare_parameter("camera_valid_age_ms", 200.0);
  declare_parameter("imu_valid_age_ms", 5.0);
  declare_parameter("pos_var_valid_urad2", 1.0e5);
  declare_parameter("imu_max_rate_rad_s", 10.0);
  declare_parameter("imu_max_step_rad_s", 1.0);
  declare_parameter<int64_t>("imu_burst_count", 5);
  declare_parameter("imu_burst_window_ms", 100.0);
  declare_parameter("imu_recover_ms", 1000.0);
  declare_parameter("max_dt_s", 0.010);
  declare_parameter("initial_pos_sigma_urad", 2000.0);
  declare_parameter("initial_vel_sigma_urad_s", 500.0);
  declare_parameter("imu_to_optical", std::vector<double>(imu_to_optical_.begin(), imu_to_optical_.end()));
  declare_parameter("fsm_fresh_ms", fsm_fresh_ms_);
  declare_parameter("gimbal_cmd_fresh_ms", gimbal_cmd_fresh_ms_);
  declare_parameter("imu_timeout_ms", imu_timeout_ms_);
  declare_parameter("autostart", autostart_);

  autostart_ = get_parameter("autostart").as_bool();
  if (autostart_) {
    // Configure and activate on the first spin so a plain launch works; a
    // lifecycle manager would do this externally in the real system (AD-8).
    autostart_timer_ = create_wall_timer(
      100ms, [this] {
        autostart_timer_->cancel();
        if (configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {activate();}
      });
  }
}

Params StateEstimatorNode::read_params()
{
  Params p;
  const auto mode = get_parameter("fusion_mode").as_string();
  p.fusion_mode = mode == "apply_at_arrival" ? FusionMode::kApplyAtArrival : FusionMode::kReplay;
  if (mode != "replay" && mode != "apply_at_arrival") {
    RCLCPP_WARN(get_logger(), "unknown fusion_mode '%s', using replay", mode.c_str());
  }
  p.gyro_noise_urad_s_sqrt_hz = get_parameter("gyro_noise_urad_s_sqrt_hz").as_double();
  p.drift_rate_random_walk_urad_s_sqrt_s = get_parameter("drift_rate_random_walk_urad_s_sqrt_s").as_double();
  p.camera_only_q_scale = get_parameter("camera_only_q_scale").as_double();
  p.camera_noise_urad = get_parameter("camera_noise_urad").as_double();
  p.camera_confidence_floor = get_parameter("camera_confidence_floor").as_double();
  p.camera_outlier_gate_sigma = get_parameter("camera_outlier_gate_sigma").as_double();
  p.camera_outlier_min_confidence = get_parameter("camera_outlier_min_confidence").as_double();
  p.camera_future_tolerance_ms = get_parameter("camera_future_tolerance_ms").as_double();
  p.camera_coast_age_ms = get_parameter("camera_coast_age_ms").as_double();
  p.camera_valid_age_ms = get_parameter("camera_valid_age_ms").as_double();
  p.imu_valid_age_ms = get_parameter("imu_valid_age_ms").as_double();
  p.pos_var_valid_urad2 = get_parameter("pos_var_valid_urad2").as_double();
  p.imu_max_rate_rad_s = get_parameter("imu_max_rate_rad_s").as_double();
  p.imu_max_step_rad_s = get_parameter("imu_max_step_rad_s").as_double();
  p.imu_burst_count = static_cast<int>(get_parameter("imu_burst_count").as_int());
  p.imu_burst_window_ms = get_parameter("imu_burst_window_ms").as_double();
  p.imu_recover_ms = get_parameter("imu_recover_ms").as_double();
  p.max_dt_s = get_parameter("max_dt_s").as_double();
  p.initial_pos_sigma_urad = get_parameter("initial_pos_sigma_urad").as_double();
  p.initial_vel_sigma_urad_s = get_parameter("initial_vel_sigma_urad_s").as_double();

  const auto m = get_parameter("imu_to_optical").as_double_array();
  if (m.size() == 6) {
    std::copy(m.begin(), m.end(), imu_to_optical_.begin());
  } else {
    RCLCPP_WARN(get_logger(), "imu_to_optical needs 6 elements, using identity mapping");
  }
  fsm_fresh_ms_ = get_parameter("fsm_fresh_ms").as_double();
  gimbal_cmd_fresh_ms_ = get_parameter("gimbal_cmd_fresh_ms").as_double();
  imu_timeout_ms_ = get_parameter("imu_timeout_ms").as_double();
  return p;
}

StateEstimatorNode::CallbackReturn StateEstimatorNode::on_configure(const rclcpp_lifecycle::State &)
{
  const Params p = read_params();
  est_ = std::make_unique<ResidualLosEstimator>(p);
  out_ = pat_msgs::msg::LosEstimate();
  out_.header.frame_id = "optical";

  rclcpp::SubscriptionOptions fast, slow;
  fast.callback_group = imu_grp_;
  slow.callback_group = cam_grp_;

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    topics::kImuData, qos::imu_data(), [this](const sensor_msgs::msg::Imu & m) {on_imu(m);}, fast);
  fsm_sub_ = create_subscription<pat_msgs::msg::FsmState>(
    topics::kFsmState, qos::fsm_state(), [this](const pat_msgs::msg::FsmState & m) {on_fsm_state(m);}, fast);
  gimbal_cmd_sub_ = create_subscription<pat_msgs::msg::GimbalCmd>(
    topics::kGimbalCmd, qos::gimbal_cmd(), [this](const pat_msgs::msg::GimbalCmd & m) {on_gimbal_cmd(m);}, fast);
  cam_sub_ = create_subscription<pat_msgs::msg::SpotError>(
    topics::kCameraSpotError, qos::camera_spot_error(),
    [this](const pat_msgs::msg::SpotError & m) {on_camera(m);}, slow);
  pub_ = create_publisher<pat_msgs::msg::LosEstimate>(topics::kLosEstimate, qos::los_estimate());

  // IMU-loss fallback (TD-1): keeps the estimate and its publication alive
  // when the IMU stops; a no-op while IMU samples are arriving.
  watchdog_ = create_wall_timer(2ms, [this] {on_watchdog();}, imu_grp_);

  RCLCPP_INFO(get_logger(), "configured: fusion_mode=%s, ring=%zu entries",
    p.fusion_mode == FusionMode::kReplay ? "replay" : "apply_at_arrival",
    ResidualLosEstimator::kRingSize);
  return CallbackReturn::SUCCESS;
}

StateEstimatorNode::CallbackReturn StateEstimatorNode::on_activate(const rclcpp_lifecycle::State & s)
{
  LifecycleNode::on_activate(s);
  RCLCPP_INFO(get_logger(), "active");
  return CallbackReturn::SUCCESS;
}

StateEstimatorNode::CallbackReturn StateEstimatorNode::on_deactivate(const rclcpp_lifecycle::State & s)
{
  LifecycleNode::on_deactivate(s);
  return CallbackReturn::SUCCESS;
}

StateEstimatorNode::CallbackReturn StateEstimatorNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  watchdog_.reset();
  imu_sub_.reset();
  fsm_sub_.reset();
  gimbal_cmd_sub_.reset();
  cam_sub_.reset();
  pub_.reset();
  est_.reset();
  return CallbackReturn::SUCCESS;
}

StateEstimatorNode::CallbackReturn StateEstimatorNode::on_shutdown(const rclcpp_lifecycle::State & s)
{
  return on_cleanup(s);
}

// ---- E1 -------------------------------------------------------------------

void StateEstimatorNode::on_imu(const sensor_msgs::msg::Imu & m)
{
  const auto & r = imu_to_optical_;
  const auto & w = m.angular_velocity;
  const Vec2 omega{r[0] * w.x + r[1] * w.y + r[2] * w.z, r[3] * w.x + r[4] * w.y + r[5] * w.z};
  last_imu_arrival_ns_.store(now().nanoseconds(), std::memory_order_relaxed);
  tick(to_ns(m.header.stamp), omega, true);
}

void StateEstimatorNode::on_fsm_state(const pat_msgs::msg::FsmState & m)
{
  fsm_pos_ = Vec2{m.x, m.y};
  fsm_stamp_ns_ = to_ns(m.header.stamp);
}

void StateEstimatorNode::on_gimbal_cmd(const pat_msgs::msg::GimbalCmd & m)
{
  gimbal_rate_ = Vec2{m.az_rate, m.el_rate};
  gimbal_cmd_stamp_ns_ = to_ns(m.header.stamp);
}

void StateEstimatorNode::on_watchdog()
{
  const int64_t t = now().nanoseconds();
  if (static_cast<double>(t - last_imu_arrival_ns_.load(std::memory_order_relaxed)) * 1e-6 < imu_timeout_ms_) {
    return;
  }
  tick(t, Vec2{}, false);
}

void StateEstimatorNode::tick(int64_t stamp_ns, const Vec2 & omega, bool imu_present)
{
  if (!est_) {return;}
  Correction c;
  while (corrections_.pop(c)) {est_->apply_correction(c);}

  const bool fsm_fresh = static_cast<double>(stamp_ns - fsm_stamp_ns_) * 1e-6 < fsm_fresh_ms_;
  const bool gimbal_fresh = static_cast<double>(stamp_ns - gimbal_cmd_stamp_ns_) * 1e-6 < gimbal_cmd_fresh_ms_;
  est_->propagate(stamp_ns, omega, imu_present, gimbal_fresh ? gimbal_rate_ : Vec2{}, fsm_pos_, fsm_fresh);

  if (pub_ && pub_->is_activated()) {publish(est_->snapshot(now().nanoseconds()), stamp_ns);}
}

void StateEstimatorNode::publish(const Snapshot & s, int64_t stamp_ns)
{
  out_.header.stamp = pat_common::from_ns(stamp_ns);
  out_.seq = ++seq_;
  out_.ex = s.x[0];
  out_.ey = s.x[1];
  out_.vx = s.x[2];
  out_.vy = s.x[3];
  std::copy(s.P.begin(), s.P.end(), out_.covariance.begin());
  out_.camera_age_ms = s.camera_age_ms;
  out_.imu_age_ms = s.imu_age_ms;
  out_.mode = static_cast<uint8_t>(s.mode);
  out_.valid = s.valid;
  out_.camera_frames_accepted = s.counters.camera_accepted;
  out_.camera_frames_rejected = s.counters.camera_rejected_invalid + s.counters.camera_rejected_too_old +
    s.counters.camera_rejected_out_of_order + s.counters.camera_rejected_future +
    s.counters.camera_rejected_outlier + s.counters.camera_rejected_overrun;
  out_.imu_samples_rejected = s.counters.imu_rejected;
  pub_->publish(out_);
}

// ---- E2 -------------------------------------------------------------------

void StateEstimatorNode::on_camera(const pat_msgs::msg::SpotError & m)
{
  if (!est_) {return;}
  CameraSample s;
  s.stamp_ns = to_ns(m.header.stamp);
  s.z = Vec2{m.ex, m.ey};
  s.confidence = m.confidence;
  s.valid = m.valid;
  if (auto c = est_->prepare_correction(s)) {
    if (!corrections_.push(*c)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "correction queue full; frame dropped");
    }
  }
}

}  // namespace pat_estimator
