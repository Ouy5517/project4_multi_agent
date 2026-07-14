from __future__ import annotations

import math


def dot(a, b) -> float:
    return float(a[0] * b[0] + a[1] * b[1] + a[2] * b[2])


def sub(a, b) -> list[float]:
    return [float(a[0] - b[0]), float(a[1] - b[1]), float(a[2] - b[2])]


def add(a, b) -> list[float]:
    return [float(a[0] + b[0]), float(a[1] + b[1]), float(a[2] + b[2])]


def scale(a, value: float) -> list[float]:
    return [float(a[0] * value), float(a[1] * value), float(a[2] * value)]


def norm(a) -> float:
    return math.sqrt(dot(a, a))


def unit(a) -> list[float] | None:
    length = norm(a)
    if length <= 1e-12:
        return None
    return [a[0] / length, a[1] / length, a[2] / length]


def clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def axes_from_orientation(orientation: list[float]) -> list[list[float]]:
    return [
        [orientation[0], orientation[3], orientation[6]],
        [orientation[1], orientation[4], orientation[7]],
        [orientation[2], orientation[5], orientation[8]],
    ]


def project_point_to_local_frame(point, center, axes) -> list[float]:
    delta = sub(point, center)
    return [dot(delta, axis) for axis in axes]


def point_to_oriented_box_distance(point, center, half_extents, axes) -> dict:
    local = project_point_to_local_frame(point, center, axes)
    clamped = [
        clamp(local[0], -half_extents[0], half_extents[0]),
        clamp(local[1], -half_extents[1], half_extents[1]),
        clamp(local[2], -half_extents[2], half_extents[2]),
    ]
    closest = list(center)
    for axis, value in zip(axes, clamped):
        closest = add(closest, scale(axis, value))
    delta = sub(point, closest)
    distance = norm(delta)
    inside = all(abs(local[i]) <= half_extents[i] for i in range(3))
    if inside:
        margins = [half_extents[i] - abs(local[i]) for i in range(3)]
        axis_index = min(range(3), key=lambda i: margins[i])
        sign = 1.0 if local[axis_index] >= 0 else -1.0
        normal = scale(axes[axis_index], sign)
        closest = list(center)
        for i, axis in enumerate(axes):
            value = local[i]
            if i == axis_index:
                value = sign * half_extents[i]
            closest = add(closest, scale(axis, value))
        distance = -margins[axis_index]
    else:
        normal = unit(delta) or [0.0, 0.0, 1.0]
    return {
        "distance": distance,
        "closest_point": closest,
        "closest_normal": normal,
        "local_point": local,
        "inside": inside,
    }


def sphere_overlap_depth(signed_surface_distance: float) -> float:
    return max(0.0, -float(signed_surface_distance))


def sphere_to_oriented_box_signed_distance(sphere_center, radius: float, box: dict) -> dict:
    point = point_to_oriented_box_distance(sphere_center, box["center"], box["half_extents"], box["axes"])
    signed = point["distance"] - radius
    return {
        **point,
        "signed_surface_distance": signed,
        "overlap_depth": sphere_overlap_depth(signed),
        "overlapping": signed < 0.0,
    }


def sphere_to_axis_aligned_box_signed_distance(sphere_center, radius: float, box_min, box_max) -> dict:
    center = [
        (box_min[0] + box_max[0]) / 2.0,
        (box_min[1] + box_max[1]) / 2.0,
        (box_min[2] + box_max[2]) / 2.0,
    ]
    half = [
        (box_max[0] - box_min[0]) / 2.0,
        (box_max[1] - box_min[1]) / 2.0,
        (box_max[2] - box_min[2]) / 2.0,
    ]
    return sphere_to_oriented_box_signed_distance(
        sphere_center,
        radius,
        {"center": center, "half_extents": half, "axes": [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]},
    )


def oriented_box_corners_world(center, half_extents, axes) -> list[list[float]]:
    corners = []
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            for sz in (-1.0, 1.0):
                point = list(center)
                for axis, extent, sign in zip(axes, half_extents, (sx, sy, sz)):
                    point = add(point, scale(axis, extent * sign))
                corners.append(point)
    return corners
