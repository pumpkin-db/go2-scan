# Stair Navigation

Bounded standalone stair episode for navigation-level simulation tests. It uses confirmed
`StairTrack` geometry only; no scene registry or hard-coded stair coordinates.

Pipeline:

`stair_traverser -> /cmd_vel_stair -> motion_arbiter -> /cmd_vel`

`active StairTrack + body pose -> terrain_follow_sim_adapter -> /sim/body_z_target`

The Gazebo ModelPlugin remains the sole pose writer. Every state has a timeout; failure holds
zero velocity. Current follower uses perceived stair centerline; wall-plane corridor support is
pending. Launch P2 with `scan_enabled:=false stair_episode:=true remote_drive:=true`.
