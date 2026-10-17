#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive


class WorkcellScene(Node):

    def __init__(self):
        super().__init__('setup_workcell_scene')

        # Gazebo spawns workcell_environment with a global pose, while this
        # node publishes collision objects directly in base_link/world.
        # Apply the same pose here so both representations stay aligned.
        self.declare_parameter('workcell_x', 0.0)
        self.declare_parameter('workcell_y', 0.0)
        self.declare_parameter('workcell_z', -0.95)
        self.declare_parameter('hopper_local_x', 0.00000335)
        self.declare_parameter('hopper_local_y', -0.75005309)
        self.declare_parameter('hopper_local_z', 0.93443794)
        self.declare_parameter('ground_x', 0.0)
        self.declare_parameter('ground_y', 0.0)
        self.declare_parameter('ground_z', -0.95)

        workcell_x = float(self.get_parameter('workcell_x').value)
        workcell_y = float(self.get_parameter('workcell_y').value)
        workcell_z = float(self.get_parameter('workcell_z').value)

        # Existing coordinates below describe the default workcell pose
        # (0, 0, -0.95). This offset converts them to the requested pose.
        self.workcell_offset = (
            workcell_x,
            workcell_y,
            workcell_z - (-0.95)
        )

        # These values are the pose of the hopper link inside the SDF model.
        # Convert its local Z to the old default world frame so add_box() can
        # apply the same global workcell offset as every other collision box.
        self.hopper_local_x = float(
            self.get_parameter('hopper_local_x').value
        )
        self.hopper_local_y = float(
            self.get_parameter('hopper_local_y').value
        )
        self.hopper_local_z = float(
            self.get_parameter('hopper_local_z').value
        )

        self.ground_pose = (
            float(self.get_parameter('ground_x').value),
            float(self.get_parameter('ground_y').value),
            float(self.get_parameter('ground_z').value) - 0.015
        )

        self.client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

    # =========================================================
    # Quaternion
    # =========================================================

    @staticmethod
    def quat_from_rpy(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        q = (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        )

        return q

    # =========================================================
    # XY transform for original workcell STL-local coordinates
    # =========================================================

    @staticmethod
    def transform_xy(origin, yaw, local):

        ox, oy, oz = origin
        lx, ly, lz = local

        c = math.cos(yaw)
        s = math.sin(yaw)

        x = ox + c * lx - s * ly
        y = oy + s * lx + c * ly
        z = oz + lz

        return x, y, z

    # =========================================================
    # CollisionObject helpers
    # =========================================================

    def make_object(self, object_id):

        obj = CollisionObject()

        # Current MoveIt / AUBO reference frame
        obj.header.frame_id = 'base_link'

        obj.id = object_id
        obj.operation = CollisionObject.ADD

        return obj

    def add_box(
        self,
        obj,
        center,
        size,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        apply_workcell_offset=True
    ):

        primitive = SolidPrimitive()

        primitive.type = SolidPrimitive.BOX

        primitive.dimensions = [
            float(size[0]),
            float(size[1]),
            float(size[2])
        ]

        pose = Pose()

        if apply_workcell_offset:
            dx, dy, dz = self.workcell_offset
        else:
            dx, dy, dz = (0.0, 0.0, 0.0)

        pose.position.x = float(center[0]) + dx
        pose.position.y = float(center[1]) + dy
        pose.position.z = float(center[2]) + dz

        qx, qy, qz, qw = self.quat_from_rpy(
            roll,
            pitch,
            yaw
        )

        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)

    def add_local_box(
        self,
        obj,
        link_origin,
        link_yaw,
        local_center,
        size,
        apply_workcell_offset=True
    ):

        center = self.transform_xy(
            link_origin,
            link_yaw,
            local_center
        )

        self.add_box(
            obj,
            center,
            size,
            yaw=link_yaw,
            apply_workcell_offset=apply_workcell_offset
        )

    # =========================================================
    # Build complete Planning Scene
    # =========================================================

    def build_scene(self):

        objects = []

        # =====================================================
        # 1. Ground
        # =====================================================

        ground = self.make_object(
            'workcell_ground'
        )

        self.add_box(
            ground,
            center=self.ground_pose,
            size=(4.0, 4.0, 0.03),
            apply_workcell_offset=False
        )

        objects.append(ground)

        # =====================================================
        # 2. AUBO mounting pedestal
        # =====================================================

        pedestal = self.make_object(
            'workcell_pedestal'
        )

        self.add_box(
            pedestal,
            center=(0.0, 0.0, -0.515),
            size=(0.50, 0.50, 0.87)
        )

        objects.append(pedestal)

        # =====================================================
        # 3. SOURCE HOPPER
        #
        # Matches current Gazebo hopper.
        #
        # Top inner opening:
        #   0.740 x 1.095 m
        #
        # Bottom inner opening:
        #   0.615 x 0.930 m
        #
        # Inner depth:
        #   0.810 m
        #
        # Structure:
        #   +X sloped wall
        #   -X sloped wall
        #   -Y sloped wall
        #   +Y sloped wall
        #   bottom
        #
        # IMPORTANT:
        # The interior is EMPTY.
        # Do NOT replace this with a solid box.
        # =====================================================

        hopper = self.make_object(
            'workcell_box'
        )

        # -----------------------------------------------------
        # Gazebo hopper top-center in base_link/world
        #
        # model local; must match model.sdf link "box" pose:
        #   x =  0.00000335
        #   y = -0.75005309
        #   z =  0.93443794
        #
        # entire workcell spawn offset:
        #   z = -0.95
        #
        # The workcell_offset below converts this default pose to the
        # requested world/base_link pose.
        # -----------------------------------------------------

        hopper_x = self.hopper_local_x
        hopper_y = self.hopper_local_y
        hopper_top_z = self.hopper_local_z - 0.95

        wall_center_z = hopper_top_z - 0.405
        bottom_center_z = hopper_top_z - 0.825

        # -----------------------------------------------------
        # +X wall
        # -----------------------------------------------------

        self.add_box(
            hopper,
            center=(
                hopper_x + 0.33875,
                hopper_y,
                wall_center_z
            ),
            size=(
                0.020,
                1.095,
                0.812408
            ),
            pitch=0.0770079
        )

        # -----------------------------------------------------
        # -X wall
        # -----------------------------------------------------

        self.add_box(
            hopper,
            center=(
                hopper_x - 0.33875,
                hopper_y,
                wall_center_z
            ),
            size=(
                0.020,
                1.095,
                0.812408
            ),
            pitch=-0.0770079
        )

        # -----------------------------------------------------
        # -Y wall
        #
        # Keep RPY consistent with current Gazebo SDF.
        # -----------------------------------------------------

        self.add_box(
            hopper,
            center=(
                hopper_x,
                hopper_y - 0.50625,
                wall_center_z
            ),
            size=(
                0.740,
                0.020,
                0.814191
            ),
            # Must match model.sdf hopper_front_collision:
            # <pose>0 -0.50625 -0.405 0.1015018 0 0</pose>
            roll=0.1015018
        )

        # -----------------------------------------------------
        # +Y wall
        # -----------------------------------------------------

        self.add_box(
            hopper,
            center=(
                hopper_x,
                hopper_y + 0.50625,
                wall_center_z
            ),
            size=(
                0.740,
                0.020,
                0.814191
            ),
            # Must match model.sdf hopper_back_collision:
            # <pose>0 0.50625 -0.405 -0.1015018 0 0</pose>
            roll=-0.1015018
        )

        # -----------------------------------------------------
        # Bottom
        # -----------------------------------------------------

        self.add_box(
            hopper,
            center=(
                hopper_x,
                hopper_y,
                bottom_center_z
            ),
            size=(
                0.615,
                0.930,
                0.030
            )
        )

        objects.append(hopper)

        # =====================================================
        # 4. RECEIVING STORE
        #
        # Keep original hollow receiving bin.
        # =====================================================

        store_origin = (
            -0.42499759,
            -0.64593152,
            -0.93995323
        )

        store_yaw = -1.57079265

        store = self.make_object(
            'workcell_store'
        )

        store_parts = [

            # =================================================
            # Hollow upper receiving bin
            # =================================================

            # bottom
            (
                (0.2500, -0.1500, 0.6010),
                (0.5500, 0.4000, 0.0200)
            ),

            # x-min wall
            (
                (-0.0125, -0.1500, 0.7610),
                (0.0250, 0.4000, 0.3000)
            ),

            # x-max wall
            (
                (0.5125, -0.1500, 0.7610),
                (0.0250, 0.4000, 0.3000)
            ),

            # y-min wall
            (
                (0.2500, -0.3375, 0.7610),
                (0.5000, 0.0250, 0.3000)
            ),

            # y-max wall
            (
                (0.2500, 0.0375, 0.7610),
                (0.5000, 0.0250, 0.3000)
            ),

            # =================================================
            # Support structure
            # =================================================

            # lower plate
            (
                (0.2500, -0.1750, 0.0610),
                (0.5500, 0.4000, 0.0200)
            ),

            # upper support plate
            (
                (0.2500, -0.1750, 0.5810),
                (0.5500, 0.4000, 0.0200)
            ),

            # support posts
            (
                (0.5000, -0.3500, 0.3210),
                (0.0500, 0.0500, 0.5000)
            ),

            (
                (0.0000, -0.3500, 0.3210),
                (0.0500, 0.0500, 0.5000)
            ),

            (
                (0.5000, 0.0000, 0.3210),
                (0.0500, 0.0500, 0.5000)
            ),

            (
                (0.0000, 0.0000, 0.3210),
                (0.0500, 0.0500, 0.5000)
            ),
        ]

        for center, size in store_parts:

            self.add_local_box(
                store,
                store_origin,
                store_yaw,
                center,
                size
            )

        objects.append(store)

        # =====================================================
        # 5. CAMERA HOLDER
        # =====================================================

        camera_origin = (
            0.69012976,
            -0.92535016,
            -0.93995425
        )

        camera_yaw = 1.5708
        # Keep MoveIt's simplified holder geometry slightly larger than the
        # Gazebo visual/collision mesh.  A 20 mm per-side margin prevents the
        # arm from accepting paths that merely graze the camera assembly.
        camera_collision_padding = 0.020

        camera_parts = [

            # main vertical post
            (
                'post',
                (0.0140, -0.00437, 0.9500),
                (0.080, 0.040, 1.900)
            ),

            # top horizontal beam
            (
                'top_beam',
                (0.0140, 0.26563, 1.8800),
                (0.080, 0.500, 0.040)
            ),

            # camera/head assembly
            (
                'camera_head',
                (0.0116, 0.56413, 1.8051),
                (0.323, 0.097, 0.100)
            ),

            # base plate
            (
                'base',
                (0.0140, -0.00487, 0.010),
                (0.300, 0.200, 0.020)
            ),

            # top support
            (
                'top_support',
                (0.0116, 0.48563, 1.810),
                (0.280, 0.060, 0.100)
            ),

            # curved/support approximation
            (
                'curved_support',
                (0.0140, 0.14813, 1.7275),
                (0.080, 0.265, 0.265)
            ),
        ]

        for part_name, center, size in camera_parts:
            holder_part = self.make_object(
                f'camera_holder_{part_name}'
            )
            padded_size = tuple(
                dimension + 2.0 * camera_collision_padding
                for dimension in size
            )

            self.add_local_box(
                holder_part,
                camera_origin,
                camera_yaw,
                center,
                padded_size
            )
            objects.append(holder_part)

        return objects

    # =========================================================
    # Apply Planning Scene
    # =========================================================

    def apply(self):

        self.get_logger().info(
            'Waiting for /apply_planning_scene...'
        )

        if not self.client.wait_for_service(
            timeout_sec=10.0
        ):
            raise RuntimeError(
                '/apply_planning_scene is unavailable. '
                'Start move_group first.'
            )

        scene = PlanningScene()

        scene.is_diff = True
        scene.robot_state.is_diff = True

        scene.world.collision_objects = self.build_scene()

        request = ApplyPlanningScene.Request()
        request.scene = scene

        self.get_logger().info(
            f'Adding {len(scene.world.collision_objects)} '
            'workcell collision objects...'
        )

        future = self.client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        result = future.result()

        if result is None:
            raise RuntimeError(
                'No response from /apply_planning_scene'
            )

        if not result.success:
            raise RuntimeError(
                'MoveIt rejected Planning Scene'
            )

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'WORKCELL PLANNING SCENE ADDED'
        )

        self.get_logger().info(
            '======================================'
        )

        self.get_logger().info(
            'Added: workcell_ground'
        )

        self.get_logger().info(
            'Added: workcell_pedestal'
        )

        self.get_logger().info(
            'Added: workcell_box '
            '(hollow 0.81 m deep hopper)'
        )

        self.get_logger().info(
            'Added: workcell_store'
        )

        self.get_logger().info(
            'Added: camera_holder (6 independently named collision parts)'
        )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = WorkcellScene()

    failed = False

    try:

        node.apply()

    except Exception as e:

        failed = True

        node.get_logger().error(
            f'FAILED: {e}'
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
