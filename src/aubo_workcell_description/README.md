# JAKA workcell scene integration

This package contains the non-arm workcell geometry imported from an earlier scene:
- base_link/workcell structure
- box
- store
- camera_holder

The JAKA Zu12 arm model is maintained separately in `aubo_description`; the suction gripper, ros2_control and MoveIt configuration are attached to that arm.

## Why the scene is spawned at z=-0.95
The workcell mesh was authored with the arm mounting point 0.95 m above the workcell origin. The JAKA `base_link` is at world z=0, so this separate workcell model is spawned at z=-0.95 m to align the mounting point with the robot base. This offset belongs to the workcell scene, not to the current arm-only `robot.urdf`.

The default Gazebo ground plane at z=0 is removed and replaced by a ground plane at z=-0.95 so the full workcell remains visible.

## Launch
Start the existing JAKA Gazebo simulation first, then in another terminal:

```bash
ros2 launch aubo_workcell_description spawn_workcell.launch.py
```
