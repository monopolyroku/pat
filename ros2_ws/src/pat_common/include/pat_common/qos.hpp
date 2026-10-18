// Shared QoS profiles for every PAT topic (design_part_a.md A.4.1).
//
// Every publisher and subscription in the system must take its profile from
// this header so that a reliability/durability mismatch is impossible.
#pragma once

#include <chrono>

#include "rclcpp/qos.hpp"

namespace pat_common::qos
{

using namespace std::chrono_literals;

// Sensor and actuator streams: freshest sample wins, never block, never replay.
inline rclcpp::QoS imu_data()
{
  return rclcpp::QoS(rclcpp::KeepLast(1))
         .best_effort()
         .durability_volatile()
         .deadline(2ms)
         .liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC)
         .liveliness_lease_duration(10ms);
}

inline rclcpp::QoS camera_spot_error()
{
  return rclcpp::QoS(rclcpp::KeepLast(3)).best_effort().durability_volatile().deadline(40ms);
}

inline rclcpp::QoS los_estimate()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile().deadline(2ms);
}

inline rclcpp::QoS fsm_cmd()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile().deadline(2ms);
}

inline rclcpp::QoS gimbal_cmd()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile().deadline(20ms);
}

inline rclcpp::QoS fsm_state()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile().deadline(2ms);
}

inline rclcpp::QoS gimbal_state()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile().deadline(10ms);
}

// Low-rate events: a dropped sample changes behaviour, so make them reliable.
inline rclcpp::QoS fsm_offload()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
}

inline rclcpp::QoS status()
{
  return rclcpp::QoS(rclcpp::KeepLast(5)).reliable().durability_volatile();
}

// System mode: late joiners get the current mode; liveliness lets every node
// detect mode_manager death and fall to safe output.
inline rclcpp::QoS mode()
{
  return rclcpp::QoS(rclcpp::KeepLast(1))
         .reliable()
         .transient_local()
         .liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC)
         .liveliness_lease_duration(100ms);
}

inline rclcpp::QoS pointing_seed()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

inline rclcpp::QoS heartbeat()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile().deadline(200ms);
}

inline rclcpp::QoS telemetry()
{
  return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
}

}  // namespace pat_common::qos
