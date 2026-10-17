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

## 物块接触参数

- 物块模型：`models/grasp_cube/model.sdf`（工作空间根目录）。
- 物块、料箱底部、接收箱和地面的 ODE 双方向摩擦系数均为 1.5，
  `slip1/slip2=0`。这两个摩擦系数表示两个切向方向，不是静/动摩擦。
- 料箱侧壁保持 0.8，避免提高底面稳定性时同时增加靠墙抓取阻力。
- 物块增加 `velocity_decay`：linear=0.001、angular=0.005；
  这是额外耗散，也会影响自由运动，调整物理步长后需要重新检查。
- 质量仍为 0.05 kg，惯量按 0.15×0.10×0.05 m 均匀长方体计算。
  恢复系数保持 0；不冻结物块、不禁用重力、不改变吸盘控制。

这些是初始调参值，不能保证持续外力或穿模情况下物块完全不动。
构建后需重启工作站并重新生成物块才会加载新参数：

```bash
./aubo.sh build --packages-select aubo_workcell_description
./aubo.sh start gui
./aubo.sh pick 6
```

重启前请先结束当前抓取。离线检查使用
`python3 -m pytest -q tests/test_object_physics.py`；
实际滑动距离、堆叠和吸取稳定性仍需在 Gazebo 中验证。
