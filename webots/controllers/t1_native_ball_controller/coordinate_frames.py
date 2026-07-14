from __future__ import annotations

import math


def node_world_pose(node):
    return _pose(node, None)


def node_pose_relative_to(node, reference):
    return _pose(node, reference)


def transform_point(matrix, local_point):
    if matrix is None or local_point is None or len(matrix) < 16 or len(local_point) < 3:
        return None
    x, y, z = local_point[:3]
    return [
        matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3],
        matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7],
        matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11],
    ]


def inverse_transform(matrix):
    if matrix is None or len(matrix) < 16:
        return None
    r = [
        [matrix[0], matrix[1], matrix[2]],
        [matrix[4], matrix[5], matrix[6]],
        [matrix[8], matrix[9], matrix[10]],
    ]
    t = [matrix[3], matrix[7], matrix[11]]
    rt = [[r[j][i] for j in range(3)] for i in range(3)]
    inv_t = [
        -(rt[0][0] * t[0] + rt[0][1] * t[1] + rt[0][2] * t[2]),
        -(rt[1][0] * t[0] + rt[1][1] * t[1] + rt[1][2] * t[2]),
        -(rt[2][0] * t[0] + rt[2][1] * t[1] + rt[2][2] * t[2]),
    ]
    return [
        rt[0][0], rt[0][1], rt[0][2], inv_t[0],
        rt[1][0], rt[1][1], rt[1][2], inv_t[1],
        rt[2][0], rt[2][1], rt[2][2], inv_t[2],
        0.0, 0.0, 0.0, 1.0,
    ]


def world_to_robot(point, robot_world_pose):
    return transform_point(inverse_transform(robot_world_pose), point)


def robot_to_world(point, robot_world_pose):
    return transform_point(robot_world_pose, point)


def normalize_xy(vector):
    if vector is None or len(vector) < 2:
        return None
    norm = math.hypot(vector[0], vector[1])
    if norm <= 1e-9:
        return None
    return [vector[0] / norm, vector[1] / norm]


def horizontal_distance(a, b):
    if a is None or b is None or len(a) < 2 or len(b) < 2:
        return None
    return math.hypot(a[0] - b[0], a[1] - b[1])


def pose_translation(matrix):
    if matrix is None or len(matrix) < 16:
        return None
    return [matrix[3], matrix[7], matrix[11]]


def pose_translation_column_major(matrix):
    if matrix is None or len(matrix) < 16:
        return None
    return [matrix[12], matrix[13], matrix[14]]


def _pose(node, reference):
    if node is None or not hasattr(node, "getPose"):
        return None
    try:
        return [float(v) for v in node.getPose(reference)]
    except Exception:
        return None
