# PAT system mode and command authority (design_part_a.md A.2.1, A.2.2).
# Reliable + transient-local, depth 1, manual-by-topic liveliness.
std_msgs/Header header
uint32 seq

uint8 SAFE=0
uint8 STANDBY=1
uint8 ACQUIRE=2
uint8 SEARCH=3
uint8 COARSE_TRACK=4
uint8 HANDOFF=5
uint8 FINE_TRACK=6
uint8 RECENTRE=7
uint8 RECOVER=8
uint8 state

# Incremented on every authority change; actuator drivers reject
# commands whose authority_seq does not match.
uint32 authority_seq

uint8 AUTHORITY_NONE=0
uint8 AUTHORITY_DRIVER=1
uint8 AUTHORITY_COARSE_CONTROLLER=2
uint8 AUTHORITY_FINE_CONTROLLER=3
uint8 gimbal_authority
uint8 fsm_authority

# Transition provenance for post-flight analysis (A.5.3).
uint16 cause_code
float64 cause_metric
