# Physical Go2 ROS1 workspace

This is the isolated Gazebo Classic 11 backend for the physical HIMLoco Go2.
It is intentionally separate from the legacy `scan_planner` workspace because
both provide a package called `go2_description`, but only this workspace uses
Gazebo contacts plus `gazebo_ros_control` and `rl_sim` joint-torque control.

Source provenance:

- `rl_sar`, `robot_joint_controller`, `robot_msgs`, and Go2 description were
  copied from `fan-ziqi/rl_sar` revision `376d42c9b128f963ab08579762d5a216a976ce39`.
- Go2 HIMLoco policy files are under `policy/go2/himloco/`.

The LibTorch CPU runtime is deliberately not versioned (about 764 MB). Before
building, provision it at `library/inference_runtime/libtorch` using
`bash simulation/setup_physical_go2_deps.sh`; the script reuses a local upstream
copy when present or downloads LibTorch 2.3.0 CPU into this ignored workspace.
Then build only the Gazebo Go2 packages from this directory:

```bash
catkin_make --pkg go2_scan_physical_sim robot_msgs robot_joint_controller rl_sar go2_description
```

Do not invoke unrestricted `catkin_make`: upstream also exposes unrelated
real-robot executables whose vendor SDK dependencies are intentionally not part
of this simulation backend.

The only supported launcher is `simulation/launch_gazebo_sim_3D.sh`. It sources
this workspace alone and rejects a resolved `go2_description` outside it.
Do not source the legacy SCAN workspace in the same launch shell.

The hotel launch starts paused, waits for the heavy world to become stable,
spawns and verifies all 13 Go2 links, then unpauses. `rl_sim` starts the joint
controllers and triggers GetUp only after all 12 joint feedback streams and a
stable prone body pose are observed. This ordering is required: concurrent
world/model spawning previously produced a server-side entity that was listed
by Gazebo but invisible and unusable in gzclient.
