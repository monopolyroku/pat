// Canonical topic and service names (design_part_a.md A.1.1).
#pragma once

namespace pat_common::topics
{

inline constexpr const char * kImuData = "/pat/imu/data";
inline constexpr const char * kCameraSpotError = "/pat/camera/spot_error";
inline constexpr const char * kLosEstimate = "/pat/estimate/los_error";
inline constexpr const char * kFsmCmd = "/pat/fsm/cmd";
inline constexpr const char * kFsmState = "/pat/fsm/state";
inline constexpr const char * kFsmOffload = "/pat/fsm/offload";
inline constexpr const char * kGimbalCmd = "/pat/gimbal/cmd";
inline constexpr const char * kGimbalState = "/pat/gimbal/state";
inline constexpr const char * kMode = "/pat/mode";
inline constexpr const char * kAbort = "/pat/abort";
inline constexpr const char * kLockStatus = "/pat/lock_status";
inline constexpr const char * kFineStatus = "/pat/fine/status";
inline constexpr const char * kCoarseStatus = "/pat/coarse/status";
inline constexpr const char * kHealth = "/pat/health";
inline constexpr const char * kTelemetry = "/pat/telemetry";
inline constexpr const char * kPointingSeed = "/pat/host/pointing_seed";
inline constexpr const char * kModeRequest = "/pat/host/mode_request";

}  // namespace pat_common::topics
