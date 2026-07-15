from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import pygame

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from common.robot_action import ActionType, RobotAction
from common.world_state import Point, RobotState, WorldState, load_world_state
from strategy.team_strategy import TeamStrategy


WIDTH = 1180
HEIGHT = 760
FPS = 30
MARGIN = 44
SIDE_PANEL_WIDTH = 360
FIELD_RECT = pygame.Rect(MARGIN, MARGIN, WIDTH - SIDE_PANEL_WIDTH - MARGIN * 2, 520)

FIELD_GREEN = (39, 126, 70)
FIELD_LINE = (232, 244, 232)
BLUE = (48, 112, 218)
BLUE_DARK = (25, 63, 136)
RED = (218, 72, 72)
RED_DARK = (137, 35, 35)
BALL_ORANGE = (240, 152, 48)
GOAL_YELLOW = (246, 207, 83)
WHITE = (246, 248, 246)
TEXT = (32, 39, 45)
MUTED = (92, 103, 112)
PANEL_BG = (244, 246, 248)
ARROW_COLORS = {
    ActionType.PASS: (252, 206, 82),
    ActionType.DRIBBLE: (55, 185, 122),
    ActionType.SHOOT: (247, 107, 83),
    ActionType.MARK_OPPONENT: (162, 100, 230),
    ActionType.CHASE_BALL: (44, 171, 218),
}


class FieldViewer:
    def __init__(self, world_state: WorldState, actions: list[RobotAction]) -> None:
        self.world_state = world_state
        self.actions = actions
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption(
            f"Booster Soccer Field Viewer - {world_state.scenario_name}"
        )
        self.clock = pygame.time.Clock()
        self.font = self._font(18)
        self.small_font = self._font(15)
        self.bold_font = self._font(20, bold=True)
        self.title_font = self._font(24, bold=True)

    def run(self) -> None:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN and event.key in {
                    pygame.K_ESCAPE,
                    pygame.K_q,
                }:
                    running = False

            self._draw()
            pygame.display.flip()
            self.clock.tick(FPS)

    def _draw(self) -> None:
        self.screen.fill((226, 231, 235))
        self._draw_field()
        self._draw_goals()
        self._draw_action_arrows()
        self._draw_robots()
        self._draw_opponents()
        self._draw_ball()
        self._draw_panel()

    def _draw_field(self) -> None:
        pygame.draw.rect(self.screen, FIELD_GREEN, FIELD_RECT, border_radius=8)
        pygame.draw.rect(self.screen, FIELD_LINE, FIELD_RECT, width=3, border_radius=8)
        center_x = FIELD_RECT.centerx
        pygame.draw.line(
            self.screen,
            FIELD_LINE,
            (center_x, FIELD_RECT.top),
            (center_x, FIELD_RECT.bottom),
            2,
        )
        pygame.draw.circle(self.screen, FIELD_LINE, FIELD_RECT.center, 74, 2)
        pygame.draw.circle(self.screen, FIELD_LINE, FIELD_RECT.center, 4)

        goal_box_w = 92
        goal_box_h = 210
        left_box = pygame.Rect(
            FIELD_RECT.left,
            FIELD_RECT.centery - goal_box_h // 2,
            goal_box_w,
            goal_box_h,
        )
        right_box = pygame.Rect(
            FIELD_RECT.right - goal_box_w,
            FIELD_RECT.centery - goal_box_h // 2,
            goal_box_w,
            goal_box_h,
        )
        pygame.draw.rect(self.screen, FIELD_LINE, left_box, width=2)
        pygame.draw.rect(self.screen, FIELD_LINE, right_box, width=2)

    def _draw_goals(self) -> None:
        our_goal = self._to_screen(self.world_state.our_goal)
        enemy_goal = self._to_screen(self.world_state.enemy_goal)
        pygame.draw.rect(
            self.screen,
            GOAL_YELLOW,
            pygame.Rect(our_goal[0] - 8, our_goal[1] - 42, 16, 84),
            border_radius=3,
        )
        pygame.draw.rect(
            self.screen,
            GOAL_YELLOW,
            pygame.Rect(enemy_goal[0] - 8, enemy_goal[1] - 42, 16, 84),
            border_radius=3,
        )
        self._draw_text("Our Goal", (our_goal[0] + 12, our_goal[1] - 50), self.small_font)
        self._draw_text(
            "Enemy Goal", (enemy_goal[0] - 96, enemy_goal[1] - 50), self.small_font
        )

    def _draw_robots(self) -> None:
        for robot in self.world_state.robots:
            pos = self._to_screen(robot.point)
            pygame.draw.circle(self.screen, BLUE_DARK, pos, 19)
            pygame.draw.circle(self.screen, BLUE, pos, 16)
            heading = (
                pos[0] + int(math.cos(robot.theta) * 24),
                pos[1] - int(math.sin(robot.theta) * 24),
            )
            pygame.draw.line(self.screen, WHITE, pos, heading, 3)
            self._draw_centered_text(robot.robot_id, (pos[0], pos[1] - 34), self.small_font)

    def _draw_opponents(self) -> None:
        for opponent in self.world_state.opponents:
            pos = self._to_screen(opponent.point)
            points = [(pos[0], pos[1] - 19), (pos[0] - 18, pos[1] + 15), (pos[0] + 18, pos[1] + 15)]
            pygame.draw.polygon(self.screen, RED_DARK, points)
            inner = [(pos[0], pos[1] - 14), (pos[0] - 13, pos[1] + 11), (pos[0] + 13, pos[1] + 11)]
            pygame.draw.polygon(self.screen, RED, inner)
            self._draw_centered_text(
                self._opponent_label(opponent.opponent_id),
                (pos[0], pos[1] + 30),
                self.small_font,
            )

    def _draw_ball(self) -> None:
        pos = self._to_screen(Point(self.world_state.ball.x, self.world_state.ball.y))
        pygame.draw.circle(self.screen, (136, 84, 26), pos, 11)
        pygame.draw.circle(self.screen, BALL_ORANGE, pos, 8)
        self._draw_text("Ball", (pos[0] + 12, pos[1] - 10), self.small_font)

    def _draw_action_arrows(self) -> None:
        for action in self.actions:
            start = self._action_start(action)
            end = self._action_end(action)
            if start is None or end is None:
                continue
            color = ARROW_COLORS.get(action.action_type)
            if color is None:
                continue
            self._draw_arrow(start, end, color, width=4)
            midpoint = ((start[0] + end[0]) // 2, (start[1] + end[1]) // 2)
            self._draw_centered_text(action.action_type.value, (midpoint[0], midpoint[1] - 18), self.small_font)

    def _draw_panel(self) -> None:
        panel = pygame.Rect(WIDTH - SIDE_PANEL_WIDTH, 0, SIDE_PANEL_WIDTH, HEIGHT)
        pygame.draw.rect(self.screen, PANEL_BG, panel)
        pygame.draw.line(self.screen, (204, 212, 218), panel.topleft, panel.bottomleft, 2)

        x = panel.left + 22
        y = 26
        self._draw_text("Strategy Actions", (x, y), self.title_font)
        y += 36
        self._draw_text(f"Scenario: {self.world_state.scenario_name}", (x, y), self.small_font, MUTED)
        y += 30

        if not self.actions:
            self._draw_text("No action", (x, y), self.font, MUTED)
            return

        for action in self.actions:
            color = ARROW_COLORS.get(action.action_type, BLUE)
            pygame.draw.circle(self.screen, color, (x + 7, y + 11), 7)
            self._draw_text(
                f"{action.robot_id}  {action.action_type.value}",
                (x + 22, y),
                self.bold_font,
                TEXT,
            )
            y += 27
            target_text = f"target={self._format_target(action.target)}"
            for line in self._wrap_text(target_text, self.small_font, SIDE_PANEL_WIDTH - 44):
                self._draw_text(line, (x, y), self.small_font, MUTED)
                y += 20
            for line in self._wrap_text(action.reason, self.small_font, SIDE_PANEL_WIDTH - 44):
                self._draw_text(line, (x, y), self.small_font, TEXT)
                y += 20
            y += 16

        hint = "Q / Esc: exit"
        self._draw_text(hint, (x, HEIGHT - 36), self.small_font, MUTED)

    def _action_start(self, action: RobotAction) -> tuple[int, int] | None:
        robot = self._robot_by_id(action.robot_id)
        if robot is None:
            return None
        return self._to_screen(robot.point)

    def _action_end(self, action: RobotAction) -> tuple[int, int] | None:
        if action.action_type == ActionType.PASS:
            target_robot = action.target.get("robot_id")
            if isinstance(target_robot, str):
                robot = self._robot_by_id(target_robot)
                if robot is not None:
                    return self._to_screen(robot.point)
            return self._target_to_screen(action)
        if action.action_type in {ActionType.DRIBBLE, ActionType.SHOOT}:
            return self._to_screen(self.world_state.enemy_goal)
        if action.action_type == ActionType.MARK_OPPONENT:
            return self._target_to_screen(action)
        if action.action_type == ActionType.CHASE_BALL:
            return self._to_screen(Point(self.world_state.ball.x, self.world_state.ball.y))
        return None

    def _target_to_screen(self, action: RobotAction) -> tuple[int, int] | None:
        x = action.target.get("x")
        y = action.target.get("y")
        if not isinstance(x, (int, float)) or not isinstance(y, (int, float)):
            return None
        return self._to_screen(Point(float(x), float(y)))

    def _to_screen(self, point: Point) -> tuple[int, int]:
        x_ratio = (point.x + self.world_state.field_width / 2) / self.world_state.field_width
        y_ratio = (self.world_state.field_height / 2 - point.y) / self.world_state.field_height
        x = FIELD_RECT.left + x_ratio * FIELD_RECT.width
        y = FIELD_RECT.top + y_ratio * FIELD_RECT.height
        return int(x), int(y)

    def _robot_by_id(self, robot_id: str) -> RobotState | None:
        return next(
            (robot for robot in self.world_state.robots if robot.robot_id == robot_id),
            None,
        )

    def _draw_arrow(
        self,
        start: tuple[int, int],
        end: tuple[int, int],
        color: tuple[int, int, int],
        width: int = 3,
    ) -> None:
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length = math.hypot(dx, dy)
        if length < 1:
            return
        start_trim = 24
        end_trim = 22
        sx = start[0] + dx / length * start_trim
        sy = start[1] + dy / length * start_trim
        ex = end[0] - dx / length * end_trim
        ey = end[1] - dy / length * end_trim
        pygame.draw.line(self.screen, color, (sx, sy), (ex, ey), width)

        angle = math.atan2(ey - sy, ex - sx)
        head_len = 15
        head_angle = math.pi / 7
        left = (
            ex - head_len * math.cos(angle - head_angle),
            ey - head_len * math.sin(angle - head_angle),
        )
        right = (
            ex - head_len * math.cos(angle + head_angle),
            ey - head_len * math.sin(angle + head_angle),
        )
        pygame.draw.polygon(self.screen, color, [(ex, ey), left, right])

    def _draw_text(
        self,
        text: str,
        pos: tuple[int, int],
        font: pygame.font.Font,
        color: tuple[int, int, int] = TEXT,
    ) -> None:
        surface = font.render(text, True, color)
        self.screen.blit(surface, pos)

    def _draw_centered_text(
        self,
        text: str,
        pos: tuple[int, int],
        font: pygame.font.Font,
        color: tuple[int, int, int] = TEXT,
    ) -> None:
        surface = font.render(text, True, color)
        rect = surface.get_rect(center=pos)
        self.screen.blit(surface, rect)

    def _wrap_text(self, text: str, font: pygame.font.Font, max_width: int) -> list[str]:
        if not text:
            return [""]

        lines: list[str] = []
        current = ""
        for char in text:
            candidate = current + char
            if current and font.size(candidate)[0] > max_width:
                lines.append(current)
                current = char
            else:
                current = candidate
        if current:
            lines.append(current)
        return lines

    def _font(self, size: int, bold: bool = False) -> pygame.font.Font:
        candidates = [
            "Noto Sans CJK SC",
            "WenQuanYi Micro Hei",
            "Microsoft YaHei",
            "SimHei",
            "Arial Unicode MS",
            "DejaVu Sans",
        ]
        return pygame.font.SysFont(candidates, size, bold=bold)

    def _format_target(self, target: dict) -> str:
        parts = []
        for key, value in target.items():
            if isinstance(value, float):
                parts.append(f"{key}={value:.2f}")
            else:
                parts.append(f"{key}={value}")
        return ", ".join(parts)

    def _opponent_label(self, opponent_id: str) -> str:
        if opponent_id.startswith("OP_"):
            return f"O{opponent_id.removeprefix('OP_')}"
        return opponent_id


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="2D 足球场策略可视化演示")
    parser.add_argument(
        "--scenario",
        type=Path,
        default=PROJECT_ROOT / "scenarios" / "pass_success.json",
        help="场景 JSON 文件路径，例如 scenarios/pass_success.json。",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    scenario_path = args.scenario
    if not scenario_path.is_absolute():
        scenario_path = PROJECT_ROOT / scenario_path

    world_state = load_world_state(scenario_path)
    actions = TeamStrategy().decide(world_state)

    pygame.init()
    try:
        FieldViewer(world_state, actions).run()
    finally:
        pygame.quit()


if __name__ == "__main__":
    main()
