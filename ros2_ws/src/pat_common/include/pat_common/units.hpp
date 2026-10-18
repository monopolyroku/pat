// Unit helpers. Angles are carried in urad, rates in rad/s (A.0.2).
#pragma once

#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"

namespace pat_common
{

inline constexpr double kUradPerRad = 1.0e6;
inline constexpr double kRadPerUrad = 1.0e-6;
inline constexpr double kPi = 3.14159265358979323846;

inline constexpr double deg_to_urad(double deg) {return deg * kPi / 180.0 * kUradPerRad;}
inline constexpr double urad_to_deg(double urad) {return urad * kRadPerUrad * 180.0 / kPi;}
inline constexpr double deg_to_rad(double deg) {return deg * kPi / 180.0;}

inline constexpr int64_t to_ns(const builtin_interfaces::msg::Time & t)
{
  return static_cast<int64_t>(t.sec) * 1000000000LL + static_cast<int64_t>(t.nanosec);
}

inline builtin_interfaces::msg::Time from_ns(int64_t ns)
{
  builtin_interfaces::msg::Time t;
  t.sec = static_cast<int32_t>(ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return t;
}

}  // namespace pat_common
