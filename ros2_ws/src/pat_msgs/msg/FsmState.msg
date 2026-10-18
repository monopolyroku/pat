# Fast steering mirror position feedback (strain gauge), 1 kHz.
std_msgs/Header header
uint32 seq
# Mirror angle mapped into the optical frame, urad.
float64 x
float64 y
# True when the mirror is at its mechanical/driver hard limit.
bool saturated
