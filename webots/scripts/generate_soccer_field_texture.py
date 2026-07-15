#!/usr/bin/env python3
"""Generate the Webots formal soccer field texture.

The texture is intentionally self-contained and uses only the Python standard
library so it can be regenerated in minimal simulation environments.
"""

from __future__ import annotations

import argparse
import math
import os
import struct
import zlib
from dataclasses import dataclass
from pathlib import Path


# Full green ground and existing playable marked area in the current Webots worlds.
GROUND_LENGTH = 7.0
GROUND_WIDTH = 5.0
FIELD_LENGTH = 6.0
FIELD_WIDTH = 4.0
STRIPE_COUNT = 12
LINE_WIDTH = 0.05

GRASS_DARK = (39, 105, 46)
GRASS_LIGHT = (49, 125, 54)
OUTER_GRASS = (35, 83, 42)
LINE_COLOR = (240, 245, 238, 255)
CORNER_ARC_RADIUS = 0.075


@dataclass(frozen=True)
class Rect:
    x_min: float
    x_max: float
    y_min: float
    y_max: float


@dataclass(frozen=True)
class FieldSpec:
    ground_length: float = GROUND_LENGTH
    ground_width: float = GROUND_WIDTH
    field_length: float = FIELD_LENGTH
    field_width: float = FIELD_WIDTH
    stripe_count: int = STRIPE_COUNT
    line_width: float = LINE_WIDTH

    @property
    def play(self) -> Rect:
        return Rect(
            -self.field_length / 2.0,
            self.field_length / 2.0,
            -self.field_width / 2.0,
            self.field_width / 2.0,
        )


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def mix(a: tuple[int, int, int], b: tuple[int, int, int], t: float) -> tuple[int, int, int]:
    return tuple(int(round(a[i] * (1.0 - t) + b[i] * t)) for i in range(3))


def stable_noise(ix: int, iy: int) -> float:
    # A small deterministic integer hash. The result is in [-1, 1].
    n = (ix * 374761393 + iy * 668265263) & 0xFFFFFFFF
    n = (n ^ (n >> 13)) * 1274126177 & 0xFFFFFFFF
    return ((n ^ (n >> 16)) & 0xFFFF) / 32767.5 - 1.0


def segment_distance(px: float, py: float, ax: float, ay: float, bx: float, by: float) -> float:
    vx = bx - ax
    vy = by - ay
    length_sq = vx * vx + vy * vy
    if length_sq <= 1e-12:
        return math.hypot(px - ax, py - ay)
    t = clamp(((px - ax) * vx + (py - ay) * vy) / length_sq, 0.0, 1.0)
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def line_alpha(distance: float, half_width: float, feather: float) -> float:
    if distance <= half_width - feather:
        return 1.0
    if distance >= half_width + feather:
        return 0.0
    return (half_width + feather - distance) / (2.0 * feather)


def add_segment_alpha(
    current: float,
    x: float,
    y: float,
    a: tuple[float, float],
    b: tuple[float, float],
    half_width: float,
    feather: float,
) -> float:
    return max(current, line_alpha(segment_distance(x, y, a[0], a[1], b[0], b[1]), half_width, feather))


def add_ring_alpha(
    current: float,
    x: float,
    y: float,
    center: tuple[float, float],
    radius: float,
    half_width: float,
    feather: float,
    arc_test: callable | None = None,
) -> float:
    dx = x - center[0]
    dy = y - center[1]
    if arc_test is not None and not arc_test(x, y, dx, dy):
        return current
    return max(current, line_alpha(abs(math.hypot(dx, dy) - radius), half_width, feather))


def add_disc_alpha(
    current: float,
    x: float,
    y: float,
    center: tuple[float, float],
    radius: float,
    feather: float,
) -> float:
    distance = math.hypot(x - center[0], y - center[1])
    if distance <= radius - feather:
        return max(current, 1.0)
    if distance >= radius + feather:
        return current
    return max(current, (radius + feather - distance) / (2.0 * feather))


def line_coverage(x: float, y: float, spec: FieldSpec, feather: float) -> float:
    play = spec.play
    half = spec.line_width / 2.0

    alpha = 0.0
    # Boundary and center lines.
    alpha = add_segment_alpha(alpha, x, y, (play.x_min, play.y_min), (play.x_max, play.y_min), half, feather)
    alpha = add_segment_alpha(alpha, x, y, (play.x_min, play.y_max), (play.x_max, play.y_max), half, feather)
    alpha = add_segment_alpha(alpha, x, y, (play.x_min, play.y_min), (play.x_min, play.y_max), half, feather)
    alpha = add_segment_alpha(alpha, x, y, (play.x_max, play.y_min), (play.x_max, play.y_max), half, feather)
    alpha = add_segment_alpha(alpha, x, y, (0.0, play.y_min), (0.0, play.y_max), half, feather)

    center_radius = spec.field_width * 0.134
    alpha = add_ring_alpha(alpha, x, y, (0.0, 0.0), center_radius, half, feather)
    alpha = add_disc_alpha(alpha, x, y, (0.0, 0.0), spec.line_width * 0.75, feather)

    penalty_depth = spec.field_length * 0.157
    penalty_width = spec.field_width * 0.596
    goal_depth = spec.field_length * 0.052
    goal_width = spec.field_width * 0.269
    penalty_mark = spec.field_length * 0.105
    corner_radius = CORNER_ARC_RADIUS

    for sign in (-1.0, 1.0):
        goal_x = play.x_max * sign
        inner_penalty_x = goal_x - sign * penalty_depth
        inner_goal_x = goal_x - sign * goal_depth
        penalty_y = penalty_width / 2.0
        goal_y = goal_width / 2.0
        spot_x = goal_x - sign * penalty_mark

        alpha = add_segment_alpha(alpha, x, y, (goal_x, -penalty_y), (inner_penalty_x, -penalty_y), half, feather)
        alpha = add_segment_alpha(alpha, x, y, (inner_penalty_x, -penalty_y), (inner_penalty_x, penalty_y), half, feather)
        alpha = add_segment_alpha(alpha, x, y, (inner_penalty_x, penalty_y), (goal_x, penalty_y), half, feather)

        alpha = add_segment_alpha(alpha, x, y, (goal_x, -goal_y), (inner_goal_x, -goal_y), half, feather)
        alpha = add_segment_alpha(alpha, x, y, (inner_goal_x, -goal_y), (inner_goal_x, goal_y), half, feather)
        alpha = add_segment_alpha(alpha, x, y, (inner_goal_x, goal_y), (goal_x, goal_y), half, feather)

        alpha = add_disc_alpha(alpha, x, y, (spot_x, 0.0), spec.line_width * 0.9, feather)

        if sign < 0.0:
            alpha = add_ring_alpha(
                alpha,
                x,
                y,
                (spot_x, 0.0),
                center_radius,
                half,
                feather,
                lambda px, _py, _dx, _dy, limit=inner_penalty_x: px >= limit,
            )
        else:
            alpha = add_ring_alpha(
                alpha,
                x,
                y,
                (spot_x, 0.0),
                center_radius,
                half,
                feather,
                lambda px, _py, _dx, _dy, limit=inner_penalty_x: px <= limit,
            )

    for corner in (
        (play.x_min, play.y_min),
        (play.x_min, play.y_max),
        (play.x_max, play.y_min),
        (play.x_max, play.y_max),
    ):
        cx, cy = corner
        alpha = add_ring_alpha(
            alpha,
            x,
            y,
            corner,
            corner_radius,
            half,
            feather,
            lambda px, py, _dx, _dy, cx=cx, cy=cy: (px - cx) * (-cx) >= -1e-9
            and (py - cy) * (-cy) >= -1e-9,
        )

    return clamp(alpha, 0.0, 1.0)


def grass_color(x: float, y: float, ix: int, iy: int, spec: FieldSpec) -> tuple[int, int, int]:
    play = spec.play
    in_play = play.x_min <= x <= play.x_max and play.y_min <= y <= play.y_max
    if in_play:
        stripe_width = spec.field_length / spec.stripe_count
        stripe_index = int((x - play.x_min) / stripe_width)
        base = GRASS_LIGHT if stripe_index % 2 == 0 else GRASS_DARK
    else:
        base = OUTER_GRASS

    broad = 0.5 + 0.5 * math.sin(13.0 * y + 1.7 * math.sin(2.0 * x))
    fine = stable_noise(ix // 2, iy // 2)
    fiber = 0.5 + 0.5 * math.sin(74.0 * x + 19.0 * stable_noise(ix // 16, iy // 16))
    shade = (broad - 0.5) * 2.0 + fine * 1.2 + (fiber - 0.5) * 0.8
    return tuple(int(clamp(channel + shade, 0, 255)) for channel in base)


def write_png(path: Path, width: int, height: int, rgb: bytearray) -> None:
    def chunk(kind: bytes, data: bytes) -> bytes:
        body = kind + data
        return struct.pack(">I", len(data)) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)

    stride = width * 3
    raw = bytearray()
    for row in range(height):
        raw.append(0)
        start = row * stride
        raw.extend(rgb[start : start + stride])

    png = bytearray(b"\x89PNG\r\n\x1a\n")
    png.extend(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
    png.extend(chunk(b"IDAT", zlib.compress(bytes(raw), 6)))
    png.extend(chunk(b"IEND", b""))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(png)


def generate(path: Path, width: int, height: int, spec: FieldSpec) -> None:
    rgb = bytearray(width * height * 3)
    px_per_meter = width / spec.ground_length
    feather = 1.3 / px_per_meter

    for iy in range(height):
        y = spec.ground_width / 2.0 - (iy + 0.5) / height * spec.ground_width
        for ix in range(width):
            x = -spec.ground_length / 2.0 + (ix + 0.5) / width * spec.ground_length
            color = grass_color(x, y, ix, iy, spec)
            alpha = line_coverage(x, y, spec, feather)
            if alpha > 0.0:
                color = mix(color, LINE_COLOR, alpha)
            offset = (iy * width + ix) * 3
            rgb[offset : offset + 3] = bytes(color)

    write_png(path, width, height, rgb)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="textures/formal_soccer_field.png")
    parser.add_argument("--width", type=int, default=2048)
    parser.add_argument(
        "--height",
        type=int,
        default=2048,
        help="Power-of-two height avoids Webots texture rescaling; UV mapping keeps field proportions correct.",
    )
    parser.add_argument("--ground-length", type=float, default=GROUND_LENGTH)
    parser.add_argument("--ground-width", type=float, default=GROUND_WIDTH)
    parser.add_argument("--field-length", type=float, default=FIELD_LENGTH)
    parser.add_argument("--field-width", type=float, default=FIELD_WIDTH)
    parser.add_argument("--stripe-count", type=int, default=STRIPE_COUNT)
    parser.add_argument("--line-width", type=float, default=LINE_WIDTH)
    args = parser.parse_args()

    spec = FieldSpec(
        ground_length=args.ground_length,
        ground_width=args.ground_width,
        field_length=args.field_length,
        field_width=args.field_width,
        stripe_count=args.stripe_count,
        line_width=args.line_width,
    )
    output = Path(args.output)
    if not output.is_absolute():
        output = Path(os.getcwd()) / output
    generate(output, args.width, args.height, spec)
    print(f"wrote {output} ({args.width}x{args.height})")


if __name__ == "__main__":
    main()
