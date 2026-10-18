# Fused residual line-of-sight error estimate, published at the IMU rate
# (design_part_a.md A.1.2, A.3.3).
std_msgs/Header header
uint32 seq
# Residual LOS error, urad, optical frame.
float64 ex
float64 ey
# Residual LOS error rate, urad/s.
float64 vx
float64 vy
# Row-major 4x4 covariance over (ex, ey, vx, vy).
float64[16] covariance
# Age of the newest measurement actually fused, ms.
float32 camera_age_ms
float32 imu_age_ms
# Estimator operating mode.
uint8 MODE_IMU_AND_CAMERA=0
uint8 MODE_CAMERA_ONLY=1
uint8 MODE_IMU_ONLY=2
uint8 mode
# True when the estimate is usable for fine control.
bool valid
# Diagnostic counters since activation.
uint32 camera_frames_accepted
uint32 camera_frames_rejected
uint32 imu_samples_rejected
