// synthetic_plant: stands in for camera_driver, imu_driver, fsm_driver and
// gimbal_driver (design_part_a.md A.1.2) on top of pat_sim::PlantModel.
//
// Publishes on the real topics with the shared QoS so flight nodes cannot
// tell the difference. Adds the sensor-path effects the estimator must
// survive: camera latency and jitter (A.0.2), dropouts and scheduled gaps
// (A.5.1), IMU glitches, bursts and gaps. Optional open-loop actuator
// profiles let the estimator be exercised without controllers; external
// commands on /pat/fsm/cmd and /pat/gimbal/cmd override them.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "pat_common/qos.hpp"
#include "pat_common/topics.hpp"
#include "pat_common/units.hpp"
#include "pat_msgs/msg/fsm_cmd.hpp"
#include "pat_msgs/msg/fsm_state.hpp"
#include "pat_msgs/msg/gimbal_cmd.hpp"
#include "pat_msgs/msg/gimbal_state.hpp"
#include "pat_msgs/msg/sim_truth.hpp"
#include "pat_msgs/msg/spot_error.hpp"
#include "pat_sim/plant_model.hpp"

namespace pat_sim
{

using namespace std::chrono_literals;
namespace qos = pat_common::qos;
namespace topics = pat_common::topics;

class SyntheticPlantNode : public rclcpp::Node
{
public:
  explicit SyntheticPlantNode(const rclcpp::NodeOptions & options)
  : Node("synthetic_plant", options)
  {
    declare_params();
    model_ = std::make_unique<PlantModel>(plant_params_);
    rng_.seed(plant_params_.seed + 1);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(topics::kImuData, qos::imu_data());
    cam_pub_ = create_publisher<pat_msgs::msg::SpotError>(topics::kCameraSpotError, qos::camera_spot_error());
    fsm_state_pub_ = create_publisher<pat_msgs::msg::FsmState>(topics::kFsmState, qos::fsm_state());
    gimbal_state_pub_ = create_publisher<pat_msgs::msg::GimbalState>(topics::kGimbalState, qos::gimbal_state());
    gimbal_cmd_pub_ = create_publisher<pat_msgs::msg::GimbalCmd>(topics::kGimbalCmd, qos::gimbal_cmd());
    truth_pub_ = create_publisher<pat_msgs::msg::SimTruth>(topics::kSimTruth, qos::sim_truth());

    // The plant republishes its own profile on /pat/gimbal/cmd for estimator
    // feedforward; it must not then mistake that for an external command.
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.ignore_local_publications = true;
    fsm_cmd_sub_ = create_subscription<pat_msgs::msg::FsmCmd>(
      topics::kFsmCmd, qos::fsm_cmd(),
      [this](const pat_msgs::msg::FsmCmd & m) {
        ext_fsm_cmd_ = {m.x, m.y};
        ext_fsm_cmd_time_ = now();
      }, sub_opts);
    gimbal_cmd_sub_ = create_subscription<pat_msgs::msg::GimbalCmd>(
      topics::kGimbalCmd, qos::gimbal_cmd(),
      [this](const pat_msgs::msg::GimbalCmd & m) {
        ext_gimbal_cmd_ = {m.az, m.el};
        ext_gimbal_cmd_time_ = now();
      }, sub_opts);

    const auto period = std::chrono::duration<double>(1.0 / tick_rate_hz_);
    start_time_ = now();
    last_tick_ = std::chrono::steady_clock::now();
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period), [this] {tick();});

    RCLCPP_INFO(get_logger(),
      "synthetic_plant: tick %.0f Hz, camera %.0f Hz latency %.1f+/-%.1f ms, vib %.0f urad RMS, "
      "platform rate [%.2f, %.2f] deg/s, gimbal profile '%s', fsm profile '%s'",
      tick_rate_hz_, camera_rate_hz_, camera_latency_ms_, camera_jitter_ms_,
      plant_params_.vib_rms_urad, plant_params_.platform_rate_deg_s[0],
      plant_params_.platform_rate_deg_s[1], gimbal_profile_.c_str(), fsm_profile_.c_str());
  }

private:
  // ---- parameters ---------------------------------------------------------
  Vec2 vec2_param(const std::string & name, Vec2 def)
  {
    const auto v = declare_parameter<std::vector<double>>(name, {def[0], def[1]});
    if (v.size() != 2) {
      RCLCPP_WARN(get_logger(), "parameter %s needs 2 elements, using default", name.c_str());
      return def;
    }
    return {v[0], v[1]};
  }

  void declare_params()
  {
    auto & p = plant_params_;
    p.target_offset_urad = vec2_param("target_offset_urad", p.target_offset_urad);
    p.target_drift_urad_s = vec2_param("target_drift_urad_s", p.target_drift_urad_s);
    p.platform_rate_deg_s = vec2_param("platform_rate_deg_s", p.platform_rate_deg_s);
    p.platform_sway_amp_deg = declare_parameter("platform_sway_amp_deg", p.platform_sway_amp_deg);
    p.platform_sway_freq_hz = declare_parameter("platform_sway_freq_hz", p.platform_sway_freq_hz);
    p.vib_rms_urad = declare_parameter("vib_rms_urad", p.vib_rms_urad);
    p.vib_f_lo_hz = declare_parameter("vib_f_lo_hz", p.vib_f_lo_hz);
    p.vib_f_hi_hz = declare_parameter("vib_f_hi_hz", p.vib_f_hi_hz);
    p.vib_lines = static_cast<int>(declare_parameter<int64_t>("vib_lines", p.vib_lines));
    p.gimbal_bw_hz = declare_parameter("gimbal_bw_hz", p.gimbal_bw_hz);
    p.gimbal_max_rate_deg_s = declare_parameter("gimbal_max_rate_deg_s", p.gimbal_max_rate_deg_s);
    p.gimbal_range_deg = vec2_param("gimbal_range_deg", p.gimbal_range_deg);
    p.fsm_wn_hz = declare_parameter("fsm_wn_hz", p.fsm_wn_hz);
    p.fsm_zeta = declare_parameter("fsm_zeta", p.fsm_zeta);
    p.fsm_range_urad = declare_parameter("fsm_range_urad", p.fsm_range_urad);
    p.camera_noise_urad = declare_parameter("camera_noise_urad", p.camera_noise_urad);
    p.camera_confidence = declare_parameter("camera_confidence", p.camera_confidence);
    p.camera_confidence_noise = declare_parameter("camera_confidence_noise", p.camera_confidence_noise);
    p.gyro_noise_urad_s_sqrt_hz = declare_parameter("gyro_noise_urad_s_sqrt_hz", p.gyro_noise_urad_s_sqrt_hz);
    p.gyro_bias_urad_s = vec2_param("gyro_bias_urad_s", p.gyro_bias_urad_s);
    p.gyro_bias_rw_urad_s_sqrt_s = declare_parameter("gyro_bias_rw_urad_s_sqrt_s", p.gyro_bias_rw_urad_s_sqrt_s);
    p.seed = static_cast<uint32_t>(declare_parameter<int64_t>("seed", p.seed));

    tick_rate_hz_ = declare_parameter("tick_rate_hz", 1000.0);
    gimbal_state_rate_hz_ = declare_parameter("gimbal_state_rate_hz", 200.0);
    truth_rate_hz_ = declare_parameter("truth_rate_hz", 1000.0);
    camera_rate_hz_ = declare_parameter("camera_rate_hz", 50.0);
    camera_latency_ms_ = declare_parameter("camera_latency_ms", 25.0);
    camera_jitter_ms_ = declare_parameter("camera_jitter_ms", 5.0);
    camera_dropout_prob_ = declare_parameter("camera_dropout_prob", 0.0);
    const std::vector<double> no_schedule;
    camera_gaps_s_ = declare_parameter("camera_gaps_s", no_schedule);
    imu_gaps_s_ = declare_parameter("imu_gaps_s", no_schedule);
    imu_bursts_s_ = declare_parameter("imu_bursts_s", no_schedule);
    imu_glitch_rate_hz_ = declare_parameter("imu_glitch_rate_hz", 0.0);
    imu_glitch_rad_s_ = declare_parameter("imu_glitch_rad_s", 20.0);
    external_cmd_timeout_ms_ = declare_parameter("external_cmd_timeout_ms", 200.0);

    gimbal_profile_ = declare_parameter<std::string>("gimbal_profile", "hold");
    gimbal_profile_start_s_ = declare_parameter("gimbal_profile_start_s", 2.0);
    gimbal_profile_rate_deg_s_ = declare_parameter("gimbal_profile_rate_deg_s", 0.1);
    gimbal_profile_amp_urad_ = declare_parameter("gimbal_profile_amp_urad", 1000.0);
    gimbal_profile_freq_hz_ = declare_parameter("gimbal_profile_freq_hz", 0.2);
    fsm_profile_ = declare_parameter<std::string>("fsm_profile", "hold");
    fsm_profile_amp_urad_ = declare_parameter("fsm_profile_amp_urad", 800.0);
    fsm_profile_freq_hz_ = declare_parameter("fsm_profile_freq_hz", 3.0);
  }

  static bool in_schedule(const std::vector<double> & pairs, double t)
  {
    for (std::size_t i = 0; i + 1 < pairs.size(); i += 2) {
      if (t >= pairs[i] && t < pairs[i + 1]) {return true;}
    }
    return false;
  }

  // ---- open-loop profiles ---------------------------------------------------
  // Returns the gimbal command and its rate (rad/s) for the feedforward topic.
  void gimbal_profile(double t, Vec2 & cmd, Vec2 & rate) const
  {
    cmd = {0.0, 0.0};
    rate = {0.0, 0.0};
    const double dt = t - gimbal_profile_start_s_;
    if (gimbal_profile_ == "ramp" && dt > 0.0) {
      const double r = pat_common::deg_to_rad(gimbal_profile_rate_deg_s_);
      cmd[0] = r * 1e6 * dt;
      rate[0] = r;
    } else if (gimbal_profile_ == "sine" && dt > 0.0) {
      const double w = 2.0 * pat_common::kPi * gimbal_profile_freq_hz_;
      cmd[0] = gimbal_profile_amp_urad_ * std::sin(w * dt);
      rate[0] = gimbal_profile_amp_urad_ * 1e-6 * w * std::cos(w * dt);
    }
  }

  Vec2 fsm_profile(double t) const
  {
    if (fsm_profile_ == "sine") {
      return {fsm_profile_amp_urad_ * std::sin(2.0 * pat_common::kPi * fsm_profile_freq_hz_ * t), 0.0};
    }
    return {0.0, 0.0};
  }

  // ---- main tick --------------------------------------------------------------
  void tick()
  {
    const auto wall = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(wall - last_tick_).count();
    last_tick_ = wall;
    dt = std::clamp(dt, 0.25 / tick_rate_hz_, 5.0 / tick_rate_hz_);

    const rclcpp::Time stamp = now();
    const double t = (stamp - start_time_).seconds();

    // Commands: external if fresh, else profile.
    PlantInputs in;
    Vec2 gimbal_rate{0.0, 0.0};
    const auto ext_timeout = rclcpp::Duration::from_seconds(external_cmd_timeout_ms_ * 1e-3);
    const bool ext_gimbal = ext_gimbal_cmd_time_.nanoseconds() > 0 && (stamp - ext_gimbal_cmd_time_) < ext_timeout;
    const bool ext_fsm = ext_fsm_cmd_time_.nanoseconds() > 0 && (stamp - ext_fsm_cmd_time_) < ext_timeout;
    if (ext_gimbal) {
      in.gimbal_cmd_urad = ext_gimbal_cmd_;
    } else {
      gimbal_profile(t, in.gimbal_cmd_urad, gimbal_rate);
    }
    in.fsm_cmd_urad = ext_fsm ? ext_fsm_cmd_ : fsm_profile(t);

    model_->step(dt, in);
    const PlantTruth tr = model_->truth();

    publish_imu(stamp, t, dt);
    publish_fsm_state(stamp, tr);
    if (accumulate(gimbal_acc_, gimbal_state_rate_hz_, dt)) {
      publish_gimbal_state(stamp, tr);
      if (!ext_gimbal && gimbal_profile_ != "hold") {publish_gimbal_cmd(stamp, in.gimbal_cmd_urad, gimbal_rate);}
    }
    if (accumulate(camera_acc_, camera_rate_hz_, dt)) {expose_camera(stamp, t);}
    deliver_camera(stamp);
    if (accumulate(truth_acc_, truth_rate_hz_, dt)) {publish_truth(stamp, tr);}
  }

  bool accumulate(double & acc, double rate_hz, double dt)
  {
    if (rate_hz <= 0.0) {return false;}
    acc += dt * rate_hz;
    if (acc >= 1.0) {
      acc -= 1.0;
      return true;
    }
    return false;
  }

  // ---- sensors --------------------------------------------------------------------
  void publish_imu(const rclcpp::Time & stamp, double t, double dt)
  {
    if (in_schedule(imu_gaps_s_, t)) {return;}
    Vec2 w = model_->sample_gyro(dt);
    if (in_schedule(imu_bursts_s_, t)) {
      w = {imu_glitch_rad_s_, 0.0};
    } else if (imu_glitch_rate_hz_ > 0.0) {
      std::uniform_real_distribution<double> u(0.0, 1.0);
      if (u(rng_) < imu_glitch_rate_hz_ * dt) {w[0] += imu_glitch_rad_s_;}
    }
    sensor_msgs::msg::Imu m;
    m.header.stamp = stamp;
    m.header.frame_id = "optical";
    // Axis convention: angular_velocity.x drives ex, .y drives ey (static
    // platform->optical rotation is identity in the simulation).
    m.angular_velocity.x = w[0];
    m.angular_velocity.y = w[1];
    m.angular_velocity.z = 0.0;
    m.orientation_covariance[0] = -1.0;            // orientation not provided
    m.linear_acceleration_covariance[0] = -1.0;    // not simulated (AD-4)
    const double var = std::pow(plant_params_.gyro_noise_urad_s_sqrt_hz * 1e-6, 2) * 0.5 / std::max(dt, 1e-6);
    m.angular_velocity_covariance[0] = m.angular_velocity_covariance[4] = m.angular_velocity_covariance[8] = var;
    imu_pub_->publish(m);
  }

  void expose_camera(const rclcpp::Time & stamp, double t)
  {
    if (in_schedule(camera_gaps_s_, t)) {return;}
    std::uniform_real_distribution<double> u(0.0, 1.0);
    if (camera_dropout_prob_ > 0.0 && u(rng_) < camera_dropout_prob_) {return;}

    double conf = 0.0;
    const Vec2 z = model_->sample_camera(conf);
    pat_msgs::msg::SpotError m;
    m.header.stamp = stamp;           // exposure mid-time
    m.header.frame_id = "camera";
    m.seq = ++camera_seq_;
    m.ex = z[0];
    m.ey = z[1];
    m.confidence = static_cast<float>(conf);
    m.valid = true;

    std::uniform_real_distribution<double> jit(-camera_jitter_ms_, camera_jitter_ms_);
    const double latency_ms = std::max(0.0, camera_latency_ms_ + jit(rng_));
    pending_frames_.push_back({stamp + rclcpp::Duration::from_seconds(latency_ms * 1e-3), m});
  }

  // Frames are released in arrival order, which may differ from stamp order
  // when jitter exceeds the frame period; the estimator must cope (A.3.3).
  void deliver_camera(const rclcpp::Time & now_t)
  {
    for (auto it = pending_frames_.begin(); it != pending_frames_.end(); ) {
      if (it->first <= now_t) {
        cam_pub_->publish(it->second);
        it = pending_frames_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void publish_fsm_state(const rclcpp::Time & stamp, const PlantTruth & tr)
  {
    pat_msgs::msg::FsmState m;
    m.header.stamp = stamp;
    m.header.frame_id = "optical";
    m.seq = ++fsm_seq_;
    m.x = tr.fsm[0];
    m.y = tr.fsm[1];
    m.saturated = tr.fsm_saturated;
    fsm_state_pub_->publish(m);
  }

  void publish_gimbal_state(const rclcpp::Time & stamp, const PlantTruth & tr)
  {
    pat_msgs::msg::GimbalState m;
    m.header.stamp = stamp;
    m.header.frame_id = "optical";
    m.seq = ++gimbal_seq_;
    m.az = tr.gimbal[0];
    m.el = tr.gimbal[1];
    m.az_rate = tr.gimbal_rate[0];
    m.el_rate = tr.gimbal_rate[1];
    m.at_limit = tr.gimbal_at_limit;
    gimbal_state_pub_->publish(m);
  }

  void publish_gimbal_cmd(const rclcpp::Time & stamp, const Vec2 & cmd, const Vec2 & rate)
  {
    pat_msgs::msg::GimbalCmd m;
    m.header.stamp = stamp;
    m.seq = ++gimbal_cmd_seq_;
    m.authority_seq = 0;
    m.az = cmd[0];
    m.el = cmd[1];
    m.az_rate = rate[0];
    m.el_rate = rate[1];
    gimbal_cmd_pub_->publish(m);
  }

  void publish_truth(const rclcpp::Time & stamp, const PlantTruth & tr)
  {
    pat_msgs::msg::SimTruth m;
    m.header.stamp = stamp;
    m.header.frame_id = "optical";
    m.ex = tr.e_cam[0];
    m.ey = tr.e_cam[1];
    m.target_x = tr.target[0];
    m.target_y = tr.target[1];
    m.platform_x = tr.platform[0];
    m.platform_y = tr.platform[1];
    m.platform_rate_x = tr.platform_rate[0];
    m.platform_rate_y = tr.platform_rate[1];
    m.gimbal_x = tr.gimbal[0];
    m.gimbal_y = tr.gimbal[1];
    m.fsm_x = tr.fsm[0];
    m.fsm_y = tr.fsm[1];
    m.gimbal_at_limit = tr.gimbal_at_limit;
    m.fsm_saturated = tr.fsm_saturated;
    truth_pub_->publish(m);
  }

  // ---- members -----------------------------------------------------------------------
  PlantParams plant_params_;
  std::unique_ptr<PlantModel> model_;
  std::mt19937 rng_;

  double tick_rate_hz_{1000.0}, gimbal_state_rate_hz_{200.0}, truth_rate_hz_{1000.0};
  double camera_rate_hz_{50.0}, camera_latency_ms_{25.0}, camera_jitter_ms_{5.0}, camera_dropout_prob_{0.0};
  std::vector<double> camera_gaps_s_, imu_gaps_s_, imu_bursts_s_;
  double imu_glitch_rate_hz_{0.0}, imu_glitch_rad_s_{20.0};
  double external_cmd_timeout_ms_{200.0};
  std::string gimbal_profile_, fsm_profile_;
  double gimbal_profile_start_s_{2.0}, gimbal_profile_rate_deg_s_{0.1}, gimbal_profile_amp_urad_{1000.0},
    gimbal_profile_freq_hz_{0.2};
  double fsm_profile_amp_urad_{800.0}, fsm_profile_freq_hz_{3.0};

  rclcpp::Time start_time_;
  std::chrono::steady_clock::time_point last_tick_;
  double gimbal_acc_{0.0}, camera_acc_{0.0}, truth_acc_{0.0};
  uint32_t camera_seq_{0}, fsm_seq_{0}, gimbal_seq_{0}, gimbal_cmd_seq_{0};
  std::deque<std::pair<rclcpp::Time, pat_msgs::msg::SpotError>> pending_frames_;

  Vec2 ext_fsm_cmd_{0.0, 0.0}, ext_gimbal_cmd_{0.0, 0.0};
  rclcpp::Time ext_fsm_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time ext_gimbal_cmd_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<pat_msgs::msg::SpotError>::SharedPtr cam_pub_;
  rclcpp::Publisher<pat_msgs::msg::FsmState>::SharedPtr fsm_state_pub_;
  rclcpp::Publisher<pat_msgs::msg::GimbalState>::SharedPtr gimbal_state_pub_;
  rclcpp::Publisher<pat_msgs::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<pat_msgs::msg::SimTruth>::SharedPtr truth_pub_;
  rclcpp::Subscription<pat_msgs::msg::FsmCmd>::SharedPtr fsm_cmd_sub_;
  rclcpp::Subscription<pat_msgs::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace pat_sim

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(pat_sim::SyntheticPlantNode)
