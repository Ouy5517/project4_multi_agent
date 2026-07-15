"""Four-robot RPC client with robot name mapping.
Fallback mode: single active mck for BLUE_1.
"""
import json, time
import rclpy
from rclpy.node import Node
from booster_interface.srv import RpcService

ROBOT_MAP = {
    'blue1': 'T1_BLUE_1',
    'blue2': 'T1_BLUE_2',
    'red1': 'T1_RED_1', 
    'red2': 'T1_RED_2',
}

class MultiRobotClient:
    def __init__(self, active_robots=None):
        """active_robots: list of robot keys that have real mck.
        None = all 4 (requires 4 mck instances — NOT SUPPORTED).
        """
        self.active = active_robots or ['blue1']  # Default: only BLUE_1
        self.node = Node('multi_robot_client')
        self.cli = self.node.create_client(RpcService, 'booster_rpc_service')
        if not self.cli.wait_for_service(timeout_sec=5):
            raise RuntimeError("RPC service unavailable")
        self.log = []

    def call(self, api_id, body, label):
        req = RpcService.Request()
        req.msg.api_id = api_id
        req.msg.body = body
        f = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self.node, f, timeout_sec=5)
        if f.done():
            r = f.result()
            self.log.append({'label': label, 'api_id': api_id, 'code': r.msg.status})
            return r.msg.status
        return -3

    def prepare(self, robot):
        if robot not in self.active: return -1
        return self.call(2000, json.dumps({'mode': 1}), f'{robot}_prepare')

    def walking(self, robot):
        if robot not in self.active: return -1
        return self.call(2000, json.dumps({'mode': 2}), f'{robot}_walking')

    def move(self, robot, vx, vy, vyaw):
        if robot not in self.active: return -1
        return self.call(2001, json.dumps({'vx': vx, 'vy': vy, 'vyaw': vyaw}), f'{robot}_move')

    def stop(self, robot):
        if robot not in self.active: return -1
        return self.move(robot, 0, 0, 0)

    def all_stop(self):
        for r in self.active:
            self.stop(r)

# NOTE: Multi-robot RPC isolation requires:
# - Independent ROS_DOMAIN_ID per robot
# - Independent FastDDS profiles  
# - Per-robot rpc_service_node instances
# NOT IMPLEMENTED — single rpc_service_node serves all.
