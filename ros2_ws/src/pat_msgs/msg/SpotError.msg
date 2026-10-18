# Camera residual optical error (design_part_a.md A.0.3, A.1.2).
# header.stamp = exposure mid-time on the IMU-synchronised clock.
std_msgs/Header header
uint32 seq
# Residual error, urad, optical frame. +ex = spot right of boresight.
float64 ex
float64 ey
# Spot SNR-derived confidence in [0, 1].
float32 confidence
# False when no spot was detected; ex/ey are then undefined.
bool valid
