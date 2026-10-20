# Ground truth from the synthetic plant, for scoring estimators and
# controllers. Never consumed by flight nodes. Angles urad, rates rad/s.
std_msgs/Header header
# True residual LOS error e = theta_tgt - theta_plat - theta_g - theta_f.
float64 ex
float64 ey
# Components of the measurement equation (A.0.3).
float64 target_x
float64 target_y
float64 platform_x
float64 platform_y
float64 platform_rate_x
float64 platform_rate_y
float64 gimbal_x
float64 gimbal_y
float64 fsm_x
float64 fsm_y
bool gimbal_at_limit
bool fsm_saturated
