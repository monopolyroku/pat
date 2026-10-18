# Gimbal encoder feedback, 200 Hz.
std_msgs/Header header
uint32 seq
# Position, urad.
float64 az
float64 el
# Rate, rad/s.
float64 az_rate
float64 el_rate
# True when either axis is at a mechanical limit.
bool at_limit
