#!/usr/bin/env python3

import random
import os
import subprocess
import time
from pathlib import Path

from aubo_vision_grasp.system_config import load_system_config


# One count interface shared with run_random_pick.sh.  Six is the default
# pressure-test size; the detector/queue remain independent of an exact count.
N = int(os.environ.get('AUBO_RANDOM_CUBE_COUNT', '6'))
MAX_SUPPORTED_CUBES = 6
MAX_CLEANUP_CUBES = 8
if not 1 <= N <= MAX_SUPPORTED_CUBES:
    raise ValueError(
        'AUBO_RANDOM_CUBE_COUNT must be between 1 and '
        f'{MAX_SUPPORTED_CUBES}, got {N}'
    )
CONFIG = load_system_config()
WORKCELL = CONFIG.workcell
INITIAL_SCENE_SETTLE_TIME = CONFIG.queue.initial_scene_settle_time
HOPPER_X = WORKCELL.hopper_x
HOPPER_Y = WORKCELL.hopper_y
# Keep the spawn centre clear of the hopper rim, but low enough that the cubes
# settle quickly instead of bouncing into a different visual scene.
CUBE_Z = WORKCELL.cube_drop_height + WORKCELL.dz
CUBE_MODEL_FILE = Path(
    os.environ.get(
        'CUBE_MODEL_FILE',
        str(
            Path(__file__).resolve().parent
            / 'models' / 'grasp_cube' / 'model.sdf'
        )
    )
)

if not CUBE_MODEL_FILE.is_file():
    raise FileNotFoundError(
        f'Cube model not found: {CUBE_MODEL_FILE}. '
        'Set CUBE_MODEL_FILE to a Gazebo SDF/URDF file.'
    )

# Keep random placement continuous, but never create two collision boxes
# already interpenetrating. Natural touching and stacking can still happen
# after impact; initial overlap is not a physically meaningful drop case.
HALF_X = WORKCELL.safe_half_x
HALF_Y = WORKCELL.safe_half_y
SPAWN_X_MIN = float(os.environ.get('AUBO_RANDOM_SPAWN_X_MIN', -HALF_X))
SPAWN_X_MAX = float(os.environ.get('AUBO_RANDOM_SPAWN_X_MAX', HALF_X))
SPAWN_Y_MIN = float(os.environ.get('AUBO_RANDOM_SPAWN_Y_MIN', -HALF_Y))
SPAWN_Y_MAX = float(os.environ.get('AUBO_RANDOM_SPAWN_Y_MAX', HALF_Y))
SPAWN_CLEARANCE = float(
    os.environ.get("AUBO_RANDOM_SPAWN_CLEARANCE", "0.010")
)
FRONT_CORNER_Y = float(
    os.environ.get("AUBO_RANDOM_FRONT_CORNER_Y", "0.30")
)
FRONT_CORNER_X_LIMIT = float(
    os.environ.get("AUBO_RANDOM_FRONT_CORNER_X_LIMIT", "0.17")
)

if not (
    -HALF_X <= SPAWN_X_MIN < SPAWN_X_MAX <= HALF_X
    and -HALF_Y <= SPAWN_Y_MIN < SPAWN_Y_MAX <= HALF_Y
):
    raise ValueError(
        'Random spawn offsets must be ordered and remain inside the '
        f'hopper-safe region x=[{-HALF_X:.3f}, {HALF_X:.3f}], '
        f'y=[{-HALF_Y:.3f}, {HALF_Y:.3f}]'
    )

if SPAWN_CLEARANCE < 0.0:
    raise ValueError("AUBO_RANDOM_SPAWN_CLEARANCE must be non-negative")


def select_non_overlapping_positions(
    count,
    max_layout_attempts=200,
    max_candidate_attempts=500,
):
    # Restart a layout that becomes greedily jammed instead of spending the
    # entire attempt budget trying to append to an impossible partial layout.
    required_x = WORKCELL.cube_x + SPAWN_CLEARANCE
    required_y = WORKCELL.cube_y + SPAWN_CLEARANCE
    for _ in range(max_layout_attempts):
        selected_positions = []
        for _ in range(max_candidate_attempts):
            candidate = (
                random.uniform(SPAWN_X_MIN, SPAWN_X_MAX),
                random.uniform(SPAWN_Y_MIN, SPAWN_Y_MAX),
            )
            # JAKA cannot lower its forearm into the two front corners without
            # colliding with the hopper/pedestal. Keep the enlarged centre
            # strip while clipping only those experimentally unreachable
            # corners.
            if (
                candidate[1] > FRONT_CORNER_Y
                and abs(candidate[0]) > FRONT_CORNER_X_LIMIT
            ):
                continue
            if all(
                abs(candidate[0] - existing[0]) >= required_x
                or abs(candidate[1] - existing[1]) >= required_y
                for existing in selected_positions
            ):
                selected_positions.append(candidate)
                if len(selected_positions) == count:
                    return selected_positions
    raise RuntimeError(
        f"Could not place {count} non-overlapping cubes in the configured "
        "spawn region; enlarge the region or reduce "
        "AUBO_RANDOM_SPAWN_CLEARANCE"
    )


selected = select_non_overlapping_positions(N)


def run_quiet(cmd, timeout=4):
    try:
        subprocess.run(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout,
            check=False
        )
    except subprocess.TimeoutExpired:
        pass


print()
print("=" * 55)
print("RESET OLD CUBES")
print("=" * 55)

# ------------------------------------------------------------
# 1. 确保吸盘关闭
# ------------------------------------------------------------

print("[1] suction_off")

run_quiet([
    "ros2",
    "service",
    "call",
    "/suction_off",
    "std_srvs/srv/Trigger",
    "{}"
])


# ------------------------------------------------------------
# 2. Delete every supported random-cube name.  Cleaning through eight also
# removes stale grasp_cube_5..8 entities left by an earlier larger run.
# ------------------------------------------------------------

for i in range(1, MAX_CLEANUP_CUBES + 1):

    name = f"grasp_cube_{i}"

    print(f"[2] delete {name}")

    run_quiet([
        "ros2",
        "service",
        "call",
        "/delete_entity",
        "gazebo_msgs/srv/DeleteEntity",
        f"{{name: '{name}'}}"
    ])


# 删除旧的单物体测试对象
print("[2] delete grasp_cube")

run_quiet([
    "ros2",
    "service",
    "call",
    "/delete_entity",
    "gazebo_msgs/srv/DeleteEntity",
    "{name: 'grasp_cube'}"
])

time.sleep(0.5)


# ------------------------------------------------------------
# 3. Spawn the requested number of random cubes.
# ------------------------------------------------------------

print()
print("=" * 55)
print(f"SPAWN {N} RANDOM CUBES")
print("=" * 55)

for i, (base_x, base_y) in enumerate(selected, start=1):

    x = HOPPER_X + base_x
    y = HOPPER_Y + base_y

    name = f"grasp_cube_{i}"

    print(
        f"{name}: "
        f"x={x:+.4f}, "
        f"y={y:+.4f}"
    )

    result = subprocess.run([
        "ros2",
        "run",
        "gazebo_ros",
        "spawn_entity.py",
        "-entity",
        name,
        "-file",
        str(CUBE_MODEL_FILE),
        "-x",
        f"{x:.4f}",
        "-y",
        f"{y:.4f}",
        "-z",
        f"{CUBE_Z:.4f}"
    ])

    if result.returncode != 0:
        raise RuntimeError(
            f"Failed to spawn {name}, "
            f"returncode={result.returncode}"
        )

    time.sleep(0.25)


print(
    f"Waiting {INITIAL_SCENE_SETTLE_TIME:.1f}s for cubes to settle "
    "before visual detection..."
)
time.sleep(INITIAL_SCENE_SETTLE_TIME)


print()
print("=" * 55)
print(f"SPAWN COMPLETE: {N} cubes")
print("=" * 55)
