# AUBO workcell scene integration

This package imports only the non-arm geometry from the supplied `robot.urdf`:
- base_link/workcell structure
- box
- store
- camera_holder

The existing AUBO i10 model, suction gripper, ros2_control and MoveIt configuration remain unchanged.

## Why the scene is spawned at z=-0.95
The original URDF has `Joint_1` at z=+0.95 m relative to the workcell base. The current AUBO simulation already uses its own `base_link` at world z=0. Therefore the imported scene is shifted down by 0.95 m so the original arm mounting location exactly coincides with the current AUBO base.

The default Gazebo ground plane at z=0 is removed and replaced by a ground plane at z=-0.95 so the full workcell remains visible.

## Launch
Start the existing AUBO Gazebo simulation first, then in another terminal:

```bash
ros2 launch aubo_workcell_description spawn_workcell.launch.py
```
