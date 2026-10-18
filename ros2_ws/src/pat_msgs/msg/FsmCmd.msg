# Fast steering mirror command, 1 kHz. Volatile QoS; drivers enforce
# a command timeout and the authority token (design_part_a.md A.2.2).
std_msgs/Header header
uint32 seq
# Must equal Mode.authority_seq of the latest mode or the driver ignores it.
uint32 authority_seq
# Commanded mirror angle in the optical frame, urad.
float64 x
float64 y
