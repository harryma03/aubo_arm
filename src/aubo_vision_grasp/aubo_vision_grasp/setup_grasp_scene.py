#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

from moveit_msgs.msg import PlanningScene, CollisionObject
from moveit_msgs.srv import ApplyPlanningScene


class SceneSetup(Node):

    def __init__(self):
        super().__init__('setup_grasp_scene')

        self.client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

    def make_box(self, object_id, xyz, size):
        obj = CollisionObject()

        obj.header.frame_id = 'world'
        obj.id = object_id

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            float(size[0]),
            float(size[1]),
            float(size[2])
        ]

        pose = Pose()
        pose.position.x = float(xyz[0])
        pose.position.y = float(xyz[1])
        pose.position.z = float(xyz[2])
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        return obj

    def run(self):

        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                '/apply_planning_scene unavailable'
            )

        objects = []

        # ---------------------------------
        # Gazebo中的桌子
        #
        # size = 0.8 x 0.8 x 0.70
        # center = (0.65, 0, 0.352)
        # table top ~= 0.702 m
        # ---------------------------------
        objects.append(
            self.make_box(
                'grasp_table',
                (0.65, 0.0, 0.352),
                (0.80, 0.80, 0.70)
            )
        )

        # ---------------------------------
        # 地面
        #
        # 不直接放一个完整4x4地板，
        # 因为base_link本身就在地面。
        #
        # 留出机器人底座附近0.5 x 0.5 m的孔。
        # ---------------------------------

        # 前方
        objects.append(
            self.make_box(
                'ground_front',
                (1.125, 0.0, -0.025),
                (1.75, 4.0, 0.05)
            )
        )

        # 后方
        objects.append(
            self.make_box(
                'ground_back',
                (-1.125, 0.0, -0.025),
                (1.75, 4.0, 0.05)
            )
        )

        # 左侧
        objects.append(
            self.make_box(
                'ground_left',
                (0.0, 1.125, -0.025),
                (0.50, 1.75, 0.05)
            )
        )

        # 右侧
        objects.append(
            self.make_box(
                'ground_right',
                (0.0, -1.125, -0.025),
                (0.50, 1.75, 0.05)
            )
        )

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True

        scene.world.collision_objects = objects

        req = ApplyPlanningScene.Request()
        req.scene = scene

        future = self.client.call_async(req)

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
                'MoveIt rejected planning scene'
            )

        self.get_logger().info(
            'Planning scene added successfully: '
            'table + collision ground'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SceneSetup()

    try:
        node.run()

    except Exception as e:
        node.get_logger().error(str(e))

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
