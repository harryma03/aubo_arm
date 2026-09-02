#!/usr/bin/env python3

import random
import subprocess
import time


# ============================================================
# Random number of objects
# ============================================================

# Fixed number of objects.
# Positions are still randomized every run.
N = 4


# ============================================================
# 12 safe candidate cells on source table
#
# Current validated source-table region:
#
# x roughly [-0.30, +0.30]
# y roughly [-1.10, -0.47]
#
# Keep a margin from table edges.
# ============================================================

XS = [
    -0.18,
    -0.06,
     0.06,
     0.18,
]

YS = [
    -0.95,
    -0.78,
    -0.61,
]

cells = [
    (x, y)
    for y in YS
    for x in XS
]

random.shuffle(cells)

selected = cells[:N]


# ============================================================
# Disconnect anything that might still be attached
# ============================================================

subprocess.run(
    [
        'ros2',
        'service',
        'call',
        '/suction_off',
        'std_srvs/srv/Trigger',
        '{}'
    ],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL
)


# ============================================================
# Delete previous objects
# ============================================================

for i in range(1, 13):

    subprocess.run(
        [
            'ros2',
            'service',
            'call',
            '/delete_entity',
            'gazebo_msgs/srv/DeleteEntity',
            f"{{name: 'grasp_cube_{i}'}}"
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL
    )

# Also delete old single-object test cube.
subprocess.run(
    [
        'ros2',
        'service',
        'call',
        '/delete_entity',
        'gazebo_msgs/srv/DeleteEntity',
        "{name: 'grasp_cube'}"
    ],
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL
)

time.sleep(0.5)


# ============================================================
# Spawn
# ============================================================

print()
print('=' * 60)
print(f'RANDOM OBJECT COUNT = {N}')
print('=' * 60)

for i, (base_x, base_y) in enumerate(
    selected,
    start=1
):

    # Small random offset so layout is not a fixed grid.
    x = base_x + random.uniform(
        -0.012,
        +0.012
    )

    y = base_y + random.uniform(
        -0.012,
        +0.012
    )

    name = f'grasp_cube_{i}'

    print(
        f'{name:15s} '
        f'x={x:+.4f} '
        f'y={y:+.4f}'
    )

    subprocess.run(
        [
            'ros2',
            'run',
            'gazebo_ros',
            'spawn_entity.py',

            '-entity',
            name,

            '-file',
            '/tmp/grasp_cube.sdf',

            '-x',
            f'{x:.4f}',

            '-y',
            f'{y:.4f}',

            '-z',
            '0.08'
        ],
        check=True
    )

    time.sleep(0.20)


print('=' * 60)
print(f'SPAWN COMPLETE: {N} cubes')
print('=' * 60)
