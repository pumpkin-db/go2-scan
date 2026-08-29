uint8 DETECTED=0
uint8 CONFIRMED=1
uint8 TEMP_INVALID=2

std_msgs/Header header
uint32 id
uint8 state
geometry_msgs/Pose entry_pose
geometry_msgs/Pose exit_pose
geometry_msgs/Vector3 heading
float32 slope
float32 width
float32 rise
float32 confidence
uint32 observation_count
time last_seen
