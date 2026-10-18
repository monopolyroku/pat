# Gimbal command, 100 Hz. Also consumed by the estimator as feedforward
# of commanded LOS motion (design_part_a.md A.0.3).
std_msgs/Header header
uint32 seq
uint32 authority_seq
# Commanded position, urad.
float64 az
float64 el
# Commanded rate, rad/s (feedforward term for the estimator).
float64 az_rate
float64 el_rate
