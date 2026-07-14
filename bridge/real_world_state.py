"""
真实 WorldState 提供者（桩代码）
================================
从外部仿真环境（MuJoCo/Webots）或实机（Booster SDK/ROS2）读取世界状态。

TODO (等待项目二/六接口确定后实现):
    1. 确定数据来源: 仿真器真值 API / ROS2 Topic / 共享内存
    2. 确定坐标系统和单位
    3. 确定数据频率 (要求 >= 30Hz)
    4. 确定机器人 ID 映射方案

当前实现: 返回默认 WorldState, 保证 --mode real 不会崩溃,
         实际使用时需要替换为真实数据源。
"""

from typing import Optional
from common.world_state import (
    WorldState, Ball, Robot, Goal, create_default_world_state
)
from common.config import (
    FIELD_WIDTH, FIELD_HEIGHT, GOAL_WIDTH,
    GOAL_X, OUR_GOAL_X, TEAM_BLUE, TEAM_YELLOW
)


class RealWorldStateProvider:
    """
    真实世界状态提供者。

    设计意图:
    - 从外部仿真器/实机读取原始数据
    - 转换为项目四内部统一的 WorldState 格式
    - 对上层决策模块完全透明

    使用方式:
        provider = RealWorldStateProvider(source=...)
        ws = provider.get()  # 返回 WorldState
    """

    def __init__(self, source=None):
        """
        Args:
            source: 数据源对象。
                    仿真模式: MuJoCo/Webots 的 Python API 句柄
                    实机模式: ROS2 Node 或 Booster SDK Client
                    当前为 None (桩代码)

        TODO: 和项目二确定 source 的具体类型和接口
        """
        self._source = source
        self._frame_count: int = 0

    def get(self) -> WorldState:
        """
        获取当前世界状态快照。

        TODO: 从 source 读取真实数据。
        当前返回默认 WorldState, 保证程序不崩溃。

        Returns:
            WorldState: 世界状态快照
        """
        self._frame_count += 1

        if self._source is not None:
            return self._from_source()

        # 桩代码: 返回默认世界状态 + 时间戳递增
        ws = create_default_world_state()
        ws.timestamp = self._frame_count / 30.0  # 模拟 30Hz
        return ws

    def _from_source(self) -> WorldState:
        """
        从外部数据源构建 WorldState。

        TODO: 实现真实数据读取逻辑。

        预期实现框架:
            raw = self._source.read()  # 或 ROS2 subscriber 回调

            ball = Ball(
                x=raw.ball_x, y=raw.ball_y, z=raw.ball_z,
                vx=raw.ball_vx, vy=raw.ball_vy, vz=raw.ball_vz
            )

            teammates = [Robot(id=..., team=Team.BLUE, x=..., y=..., z=..., theta=...)
                         for ... in raw.blue_robots]
            opponents = [Robot(id=..., team=Team.YELLOW, x=..., y=..., z=..., theta=...)
                         for ... in raw.yellow_robots]

            return WorldState(
                ball=ball,
                teammates=teammates,
                opponents=opponents,
                our_goal=Goal(...),
                opponent_goal=Goal(...),
                timestamp=raw.timestamp
            )
        """
        # 桩代码实现: 返回默认状态
        # 实际对接时删除此段, 取消上面注释并填入真实逻辑
        ws = create_default_world_state()
        ws.timestamp = self._frame_count / 30.0
        return ws

    def set_source(self, source):
        """
        运行时设置/替换数据源。

        用于:
        - 从 Mock 切换到真实接口
        - 联调时动态切换数据源
        """
        self._source = source

    @property
    def is_real(self) -> bool:
        """是否已接入真实数据源"""
        return self._source is not None
