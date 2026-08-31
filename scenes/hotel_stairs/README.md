# Hotel stairs benchmark

Default go2-scan multi-floor scene: three complete floors, two static stair
groups, rooms/corridors/furniture, and sealed elevator shafts. Normal interior
doors were removed after visual review; the L1 lobby outer door remains closed.
Elevators are forbidden, not controlled simulation objects.

Use `scene:=hotel_stairs` explicitly, or request `multi_floor:=true` without a
scene to select it. Depot remains `scene:=depot` for regression only.

The default spawn is the collision-cleared indoor exploration start near the
L1 lobby door:

```bash
bash simulation/launch_gazebo_sim.sh scene:=hotel_stairs spawn_mode:=exploration
```

Use the centered flat-floor stair smoke-test start explicitly:

```bash
bash simulation/launch_gazebo_sim.sh scene:=hotel_stairs spawn_mode:=stair_test global_planner:=none
```

Both authoritative poses are stored in `scene.yaml`. The stair entry recorded
by RCI is a navigation goal, not a safe Gazebo spawn pose.
