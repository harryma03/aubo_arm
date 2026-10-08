# Robot model interface

The workcell selects its robot through one public environment variable:

```bash
ROBOT_MODEL=aubo_i10 ./aubo.sh start gui
```

`aubo_i10` is the default. The legacy `AUBO_MOVEIT_MODEL` variable remains
an internal compatibility alias and should not be used for new configuration.

Every selectable robot description must expose this canonical interface:

| Role | Required name |
| --- | --- |
| Base link | `base_link` |
| Flange link | `wrist3_Link` |
| Tool centre point | `suction_tcp` |
| MoveIt group | `manipulator` |
| Joint 1 | `shoulder_joint` |
| Joint 2 | `upperArm_joint` |
| Joint 3 | `foreArm_joint` |
| Joint 4 | `wrist1_joint` |
| Joint 5 | `wrist2_joint` |
| Joint 6 | `wrist3_joint` |

Brand-native names should be adapted in the model URDF/Xacro. Keeping this
boundary stable allows the controller YAML, MoveIt SRDF and grasp nodes to be
reused without per-brand branches.

To add another arm:

1. Add `urdf/<ROBOT_MODEL>.urdf` and its meshes to `aubo_description`.
2. Adapt its links and joints to the canonical names above.
3. Ensure every simulated moving link has valid inertial data.
4. Add the model to the accepted choices in `scripts/start.sh` and
   `aubo_gazebo.launch.py`.
5. Add a model-specific MoveIt joint-limit YAML when its speeds differ.
6. Build the description, Gazebo and MoveIt packages, then validate the
   generated URDF with `check_urdf`.

The defaults and exported interface names live in
`src/aubo_workcell_description/config/workcell_layout.env`.
