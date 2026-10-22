// state_estimator lifecycle node (design_part_a.md A.1.2, A.4.3).
//
// Wraps ResidualLosEstimator. Two callback groups:
//   imu_grp (E1): /pat/imu/data, /pat/fsm/state, /pat/gimbal/cmd, IMU-loss
//                 watchdog. Propagates on IMU arrival (TD-1), drains pending
//                 corrections, publishes /pat/estimate/los_error.
//   cam_grp (E2): /pat/camera/spot_error. Prepares the delayed correction
//                 and hands it to E1 through an SPSC queue (TD-7).
// The executable assigns the groups to separate executors; see
// state_estimator_main.cpp.
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "pat_common/spsc_queue.hpp"
#include "pat_estimator/residual_los_estimator.hpp"
#include "pat_msgs/msg/fsm_state.hpp"
#include "pat_msgs/msg/gimbal_cmd.hpp"
#include "pat_msgs/msg/los_estimate.hpp"
#include "pat_msgs/msg/spot_error.hpp"

namespace pat_estimator
{

class StateEstimatorNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit StateEstimatorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  // Callback group that must run on the fast executor (E1), and the one
  // that must not (E2).
  rclcpp::CallbackGroup::SharedPtr fast_group() const {return imu_grp_;}
  rclcpp::CallbackGroup::SharedPtr slow_group() const {return cam_grp_;}

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;

private:
  Params read_params();

  // E1
  void on_imu(const sensor_msgs::msg::Imu & m);
  void on_fsm_state(const pat_msgs::msg::FsmState & m);
  void on_gimbal_cmd(const pat_msgs::msg::GimbalCmd & m);
  void on_watchdog();
  void tick(int64_t stamp_ns, const Vec2 & omega, bool imu_present);
  void publish(const Snapshot & s, int64_t stamp_ns);

  // E2
  void on_camera(const pat_msgs::msg::SpotError & m);

  rclcpp::CallbackGroup::SharedPtr imu_grp_;
  rclcpp::CallbackGroup::SharedPtr cam_grp_;

  std::unique_ptr<ResidualLosEstimator> est_;
  pat_common::SpscQueue<Correction, 8> corrections_;

  // Static platform->optical mapping of gyro axes onto (ex, ey): 2x3 row-major.
  std::array<double, 6> imu_to_optical_{1.0, 0.0, 0.0, 0.0, 1.0, 0.0};
  double fsm_fresh_ms_{5.0};
  double gimbal_cmd_fresh_ms_{100.0};
  double imu_timeout_ms_{5.0};
  bool autostart_{true};

  // E1-owned latest inputs (all callbacks in imu_grp are mutually exclusive).
  Vec2 fsm_pos_;
  int64_t fsm_stamp_ns_{INT64_MIN / 2};
  Vec2 gimbal_rate_;
  int64_t gimbal_cmd_stamp_ns_{INT64_MIN / 2};
  std::atomic<int64_t> last_imu_arrival_ns_{INT64_MIN / 2};
  uint32_t seq_{0};
  pat_msgs::msg::LosEstimate out_;   // pre-allocated

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<pat_msgs::msg::FsmState>::SharedPtr fsm_sub_;
  rclcpp::Subscription<pat_msgs::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<pat_msgs::msg::SpotError>::SharedPtr cam_sub_;
  rclcpp_lifecycle::LifecyclePublisher<pat_msgs::msg::LosEstimate>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  rclcpp::TimerBase::SharedPtr autostart_timer_;
};

}  // namespace pat_estimator
