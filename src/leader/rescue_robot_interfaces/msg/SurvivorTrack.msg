# A mission-runtime survivor record. Tentative candidates are not published.

uint8 STATUS_CONFIRMED=1
uint8 STATUS_LOST=2

uint32 id
geometry_msgs/Point raw_position
geometry_msgs/Point filtered_position
uint8 status
bool visible
uint32 observation_count
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
