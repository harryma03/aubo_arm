"""
Typed access to the AUBO workcell environment configuration.

Shell launchers load ``workcell_layout.env`` once.  This module is the only
place where Python nodes translate those exported strings into typed values
and derive shared geometry.
"""

from dataclasses import dataclass
import os


def env_float(name, default):
    return float(os.environ.get(name, str(default)))


def env_int(name, default):
    return int(os.environ.get(name, str(default)))


def env_bool(name, default=False):
    fallback = '1' if default else '0'
    return os.environ.get(name, fallback).strip().lower() not in (
        '0', 'false', 'no', 'off', ''
    )


def env_float_tuple(name, expected_length):
    raw = os.environ.get(name, '').strip()
    if not raw:
        return None
    values = tuple(float(item.strip()) for item in raw.split(','))
    if len(values) != expected_length:
        raise ValueError(
            f'{name} must contain {expected_length} comma-separated values'
        )
    return values


@dataclass(frozen=True)
class WorkcellConfig:
    x: float
    y: float
    z: float
    hopper_local_x: float
    hopper_local_y: float
    hopper_local_z: float
    hopper_bottom_x: float
    hopper_bottom_y: float
    cube_x: float
    cube_y: float
    cube_z: float
    wall_margin: float
    cube_drop_height: float

    @property
    def dz(self):
        return self.z - (-0.95)

    @property
    def base_x(self):
        """Workcell X expressed in the shifted robot base_link frame."""
        return self.x - env_float('AUBO_ROBOT_X', 0.0)

    @property
    def base_y(self):
        """Workcell Y expressed in the shifted robot base_link frame."""
        return self.y - env_float('AUBO_ROBOT_Y', 0.0)

    @property
    def hopper_x(self):
        return self.x + self.hopper_local_x

    @property
    def hopper_y(self):
        return self.y + self.hopper_local_y

    @property
    def hopper_top_z(self):
        return self.z + self.hopper_local_z

    @property
    def hopper_dz(self):
        return self.hopper_top_z - (-0.01556206)

    @property
    def safe_half_x(self):
        return (
            self.hopper_bottom_x * 0.5
            - self.cube_x * 0.5
            - self.wall_margin
        )

    @property
    def safe_half_y(self):
        return (
            self.hopper_bottom_y * 0.5
            - self.cube_y * 0.5
            - self.wall_margin
        )


@dataclass(frozen=True)
class VisionConfig:
    rgb_topic: str
    depth_topic: str
    camera_info_topic: str
    processing_rate: float
    sync_slop: float
    split_enable: bool
    split_peak_ratio: float
    split_min_area_ratio: float
    split_count_tolerance: float
    split_candidate_min_ratio: float
    split_candidate_max_ratio: float
    min_target_z: float
    max_target_z: float
    max_depth_spread: float
    min_area_ratio: float
    max_area_ratio: float
    display_enabled: bool


@dataclass(frozen=True)
class QueueConfig:
    initial_scene_settle_time: float
    stable_time: float
    empty_stable_time: float
    message_timeout: float
    stability_distance: float
    match_distance: float
    missing_confirmations: int
    max_retries: int
    edge_margin: float
    edge_max_retries: int
    edge_grasp_compensation: bool
    edge_grasp_trigger: float
    edge_grasp_offset: float
    snapshot_timeout: float
    settle_time: float
    collision_padding: float
    stable_frames: int
    ambiguous_obstacle_scale: float


@dataclass(frozen=True)
class MotionConfig:
    recovery_timeout: float
    velocity_scaling: float
    acceleration_scaling: float
    ompl_planning_time: float
    ompl_planning_attempts: int
    grasp_candidate_count: int
    grasp_search_budget: float
    recovery_joint_tolerance: float
    ready_joint_tolerance: float
    ready_joint_values: tuple | None


@dataclass(frozen=True)
class SystemConfig:
    workcell: WorkcellConfig
    vision: VisionConfig
    queue: QueueConfig
    motion: MotionConfig


def load_system_config():
    """Read one immutable configuration snapshot from the environment."""
    workcell = WorkcellConfig(
        x=env_float('AUBO_WORKCELL_X', 0.0),
        y=env_float('AUBO_WORKCELL_Y', 0.0),
        z=env_float('AUBO_WORKCELL_Z', -0.95),
        hopper_local_x=env_float('AUBO_HOPPER_LOCAL_X', 0.00000335),
        hopper_local_y=env_float('AUBO_HOPPER_LOCAL_Y', -0.75005309),
        hopper_local_z=env_float('AUBO_HOPPER_LOCAL_Z', 0.93443794),
        hopper_bottom_x=env_float('AUBO_HOPPER_BOTTOM_SIZE_X', 0.615),
        hopper_bottom_y=env_float('AUBO_HOPPER_BOTTOM_SIZE_Y', 0.930),
        cube_x=env_float('AUBO_CUBE_SIZE_X', 0.150),
        cube_y=env_float('AUBO_CUBE_SIZE_Y', 0.100),
        cube_z=env_float('AUBO_CUBE_SIZE_Z', 0.050),
        wall_margin=env_float('AUBO_HOPPER_WALL_MARGIN', 0.005),
        cube_drop_height=env_float('AUBO_CUBE_DROP_HEIGHT', 0.040),
    )
    vision = VisionConfig(
        rgb_topic=os.environ.get(
            'AUBO_RGB_TOPIC', '/camera/workcell_camera/image_raw'
        ),
        depth_topic=os.environ.get(
            'AUBO_DEPTH_TOPIC',
            '/camera/workcell_camera/depth/image_raw',
        ),
        camera_info_topic=os.environ.get(
            'AUBO_CAMERA_INFO_TOPIC',
            '/camera/workcell_camera/depth/camera_info',
        ),
        processing_rate=env_float('AUBO_VISION_PROCESSING_RATE', 5.0),
        sync_slop=env_float('AUBO_RGBD_SYNC_SLOP', 0.100),
        split_enable=env_bool('AUBO_TARGET_SPLIT_ENABLE', False),
        split_peak_ratio=env_float('AUBO_TARGET_SPLIT_PEAK_RATIO', 0.45),
        split_min_area_ratio=env_float(
            'AUBO_TARGET_SPLIT_MIN_AREA_RATIO', 1.55
        ),
        split_count_tolerance=env_float(
            'AUBO_TARGET_SPLIT_COUNT_TOLERANCE', 0.20
        ),
        split_candidate_min_ratio=env_float(
            'AUBO_TARGET_SPLIT_CANDIDATE_MIN_RATIO', 0.45
        ),
        split_candidate_max_ratio=env_float(
            'AUBO_TARGET_SPLIT_CANDIDATE_MAX_RATIO', 1.35
        ),
        min_target_z=env_float('AUBO_TARGET_MIN_Z', -0.82),
        max_target_z=env_float('AUBO_TARGET_MAX_Z', -0.55),
        max_depth_spread=env_float('AUBO_TARGET_MAX_DEPTH_SPREAD', 0.060),
        min_area_ratio=env_float('AUBO_TARGET_MIN_AREA_RATIO', 0.25),
        max_area_ratio=env_float('AUBO_TARGET_MAX_AREA_RATIO', 1.85),
        display_enabled=env_bool('AUBO_VISION_DISPLAY', True),
    )
    queue = QueueConfig(
        initial_scene_settle_time=env_float(
            'AUBO_INITIAL_SCENE_SETTLE_TIME', 2.0
        ),
        stable_time=env_float('AUBO_TARGET_STABLE_TIME', 0.80),
        empty_stable_time=env_float('AUBO_TARGET_EMPTY_STABLE_TIME', 2.00),
        message_timeout=env_float('AUBO_TARGET_MESSAGE_TIMEOUT', 4.00),
        stability_distance=env_float(
            'AUBO_TARGET_STABILITY_DISTANCE', 0.020
        ),
        match_distance=env_float('AUBO_TARGET_MATCH_DISTANCE', 0.120),
        missing_confirmations=env_int(
            'AUBO_TARGET_MISSING_CONFIRMATIONS', 3
        ),
        max_retries=env_int('AUBO_TARGET_MAX_RETRIES', 1),
        edge_margin=env_float('AUBO_TARGET_EDGE_MARGIN', 0.080),
        edge_max_retries=env_int('AUBO_TARGET_EDGE_MAX_RETRIES', 1),
        edge_grasp_compensation=env_bool(
            'AUBO_EDGE_GRASP_COMPENSATION', True
        ),
        edge_grasp_trigger=env_float('AUBO_EDGE_GRASP_TRIGGER', 0.080),
        edge_grasp_offset=env_float('AUBO_EDGE_GRASP_OFFSET', 0.010),
        snapshot_timeout=env_float('AUBO_TARGET_SNAPSHOT_TIMEOUT', 8.0),
        settle_time=env_float('AUBO_TARGET_SETTLE_TIME', 0.8),
        collision_padding=env_float(
            'AUBO_TARGET_COLLISION_PADDING', 0.003
        ),
        stable_frames=env_int('AUBO_TARGET_STABLE_FRAMES', 8),
        ambiguous_obstacle_scale=env_float(
            'AUBO_AMBIGUOUS_OBSTACLE_SCALE', 2.50
        ),
    )
    motion = MotionConfig(
        recovery_timeout=env_float('AUBO_RECOVERY_TIMEOUT', 12.0),
        velocity_scaling=env_float(
            'AUBO_MOVE_VELOCITY_SCALING', 0.35
        ),
        acceleration_scaling=env_float(
            'AUBO_MOVE_ACCELERATION_SCALING', 0.25
        ),
        ompl_planning_time=env_float('AUBO_OMPL_PLANNING_TIME', 4.0),
        ompl_planning_attempts=env_int('AUBO_OMPL_PLANNING_ATTEMPTS', 3),
        grasp_candidate_count=max(1, env_int('AUBO_GRASP_CANDIDATE_COUNT', 3)),
        grasp_search_budget=max(0.0, env_float('AUBO_GRASP_SEARCH_BUDGET', 8.0)),
        recovery_joint_tolerance=env_float(
            'AUBO_RECOVERY_JOINT_TOLERANCE', 0.12
        ),
        ready_joint_tolerance=env_float(
            'AUBO_READY_JOINT_TOLERANCE', 0.025
        ),
        ready_joint_values=env_float_tuple('AUBO_READY_JOINTS', 6),
    )
    return SystemConfig(workcell, vision, queue, motion)
