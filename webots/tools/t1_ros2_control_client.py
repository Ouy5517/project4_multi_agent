#!/usr/bin/python3.10
"""
Booster T1 ROS2 High-Level Control Client

Communicates with rpc_service_node via ROS2 RpcService interface.
All commands use verified API IDs and JSON parameter formats from the official SDK.

Usage:
    python3 tools/t1_ros2_control_client.py status
    python3 tools/t1_ros2_control_client.py mode
    python3 tools/t1_ros2_control_client.py prepare
    python3 tools/t1_ros2_control_client.py stand
    python3 tools/t1_ros2_control_client.py move
    python3 tools/t1_ros2_control_client.py stop
    python3 tools/t1_ros2_control_client.py turn
    python3 tools/t1_ros2_control_client.py safe-demo
"""

import argparse
import json
import signal
import sys
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from booster_interface.srv import RpcService

# ── Verified API IDs from booster::robot::b1::LocoApiId ──
API_ID = {
    "change_mode": 2000,   # LocoApiId::kChangeMode
    "move":        2001,   # LocoApiId::kMove
    "get_mode":    2017,   # LocoApiId::kGetMode
    "get_status":  2018,   # LocoApiId::kGetStatus
}

# ── RobotMode from booster::robot::RobotMode ──
ROBOT_MODE = {
    "damping":  0,
    "prepare":  1,
    "walking":  2,
    "custom":   3,
    "soccer":   4,
}

# ── Status codes from booster::robot::rpc ──
STATUS_NAMES = {
    -1:  "INVALID",
    0:   "SUCCESS",
    100: "TIMEOUT",
    400: "BAD_REQUEST",
    409: "CONFLICT",
    429: "TOO_FREQUENT",
    500: "INTERNAL_ERROR",
    501: "SERVER_REFUSED",
    502: "STATE_TRANSITION_FAILED",
}

SERVICE_NAME = "booster_rpc_service"
SERVICE_TIMEOUT = 4.0  # seconds per call

# ── Safe motion limits ──
SAFE_MAX_SPEED = 0.05      # m/s  (minimal forward speed)
SAFE_MAX_YAW_RATE = 0.1    # rad/s (minimal turn rate)
SAFE_DURATION = 0.5         # seconds max per continuous motion
SAFE_STOP_WAIT = 3.0        # seconds between motions


class T1ControlClient(Node):
    """ROS2 node that calls booster_rpc_service."""

    def __init__(self):
        super().__init__("t1_control_client")
        self.cli = self.create_client(RpcService, SERVICE_NAME)
        self.results = []

    def call(self, api_id: int, body: str, label: str = "") -> dict:
        """Call the RPC service and return parsed result."""
        if not self.cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT):
            err = {
                "label": label,
                "api_id": api_id,
                "success": False,
                "status_code": -2,
                "status_name": "SERVICE_UNAVAILABLE",
                "response_body": "",
                "error": f"Service '{SERVICE_NAME}' not available within {SERVICE_TIMEOUT}s",
                "timestamp": datetime.now().isoformat(),
            }
            self.results.append(err)
            return err

        req = RpcService.Request()
        req.msg.api_id = api_id
        req.msg.body = body

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT)

        result = {
            "label": label,
            "api_id": api_id,
            "request_body": body,
            "timestamp": datetime.now().isoformat(),
        }

        if future.done():
            resp = future.result()
            result["success"] = (resp.msg.status == 0)
            result["status_code"] = resp.msg.status
            result["status_name"] = STATUS_NAMES.get(resp.msg.status, f"UNKNOWN({resp.msg.status})")
            result["response_body"] = resp.msg.body
            try:
                result["response_parsed"] = json.loads(resp.msg.body) if resp.msg.body else {}
            except json.JSONDecodeError:
                result["response_parsed"] = {"raw": resp.msg.body}
        else:
            future.cancel()
            result["success"] = False
            result["status_code"] = -3
            result["status_name"] = "CALL_TIMEOUT"
            result["response_body"] = ""
            result["error"] = f"Call timed out after {SERVICE_TIMEOUT}s"

        self.results.append(result)
        return result

    def change_mode(self, mode: int) -> dict:
        """Change robot mode. mode=1 (prepare), mode=2 (walking)."""
        body = json.dumps({"mode": mode})
        mode_name = {v: k for k, v in ROBOT_MODE.items()}.get(mode, str(mode))
        return self.call(API_ID["change_mode"], body, f"change_mode->{mode_name}")

    def move(self, vx: float, vy: float, vyaw: float) -> dict:
        """Send velocity command. vx=forward, vy=left, vyaw=yaw rate."""
        body = json.dumps({"vx": vx, "vy": vy, "vyaw": vyaw})
        return self.call(API_ID["move"], body, f"move(vx={vx},vy={vy},vyaw={vyaw})")

    def stop(self) -> dict:
        """Stop all motion (zero velocity)."""
        return self.move(0.0, 0.0, 0.0)

    def get_mode(self) -> dict:
        """Query current robot mode."""
        return self.call(API_ID["get_mode"], "", "get_mode")

    def get_status(self) -> dict:
        """Query full robot status."""
        return self.call(API_ID["get_status"], "", "get_status")

    # ── Kick commands ──

    def shoot(self) -> dict:
        """Powerful kick (kShoot, api_id=2024). No parameters."""
        return self.call(2024, "", "kShoot(powerful)")

    def visual_kick(self, version: int = 0) -> dict:
        """Side-foot kick (kVisualKick, api_id=2038). version: 0=V1, 1=V2."""
        body = json.dumps({"start": True, "version": version})
        return self.call(2038, body, f"kVisualKick(v{version})")

    def kick_demo(self) -> list:
        """Safe kick demo: Prepare → Walking → Kick → Stop."""
        results = []
        self._log("KICK-DEMO: Prepare...")
        r = self.change_mode(ROBOT_MODE["prepare"])
        self._print_result(r); results.append(r)
        if not r["success"]: return results
        time.sleep(3)

        self._log("KICK-DEMO: Walking...")
        r = self.change_mode(ROBOT_MODE["walking"])
        self._print_result(r); results.append(r)
        if not r["success"]: return results
        time.sleep(5)

        self._log("KICK-DEMO: VisualKick V0...")
        r = self.visual_kick(0)
        self._print_result(r); results.append(r)
        time.sleep(10)

        self._log("KICK-DEMO: Stop...")
        r = self.stop()
        self._print_result(r); results.append(r)
        return results

    def safe_move(self, vx: float, vy: float, vyaw: float, duration: float) -> list:
        """Move for a limited duration, then stop. Returns all results."""
        results = []
        results.append(self.move(vx, vy, vyaw))
        time.sleep(duration)
        results.append(self.stop())
        return results

    def safe_demo(self) -> list:
        """Run the safe demo sequence."""
        all_results = []

        # Step 1: Prepare (stand)
        self._log("SAFE-DEMO: Changing to Prepare mode...")
        r = self.change_mode(ROBOT_MODE["prepare"])
        self._print_result(r)
        all_results.append(r)
        if not r["success"]:
            self._log("ABORT: Cannot enter Prepare mode.")
            return all_results
        time.sleep(3)

        # Step 2: Walking mode
        self._log("SAFE-DEMO: Changing to Walking mode...")
        r = self.change_mode(ROBOT_MODE["walking"])
        self._print_result(r)
        all_results.append(r)
        if not r["success"]:
            self._log("ABORT: Cannot enter Walking mode. Stopping...")
            self.change_mode(ROBOT_MODE["prepare"])
            return all_results
        time.sleep(1)

        # Step 3: Small forward (0.5s)
        self._log(f"SAFE-DEMO: Moving forward at {SAFE_MAX_SPEED} m/s for {SAFE_DURATION}s...")
        r = self.move(SAFE_MAX_SPEED, 0.0, 0.0)
        self._print_result(r)
        all_results.append(r)
        time.sleep(SAFE_DURATION)

        r = self.stop()
        self._print_result(r)
        all_results.append(r)
        time.sleep(SAFE_STOP_WAIT)

        # Step 4: Small turn (0.5s)
        self._log(f"SAFE-DEMO: Turning at {SAFE_MAX_YAW_RATE} rad/s for {SAFE_DURATION}s...")
        r = self.move(0.0, 0.0, SAFE_MAX_YAW_RATE)
        self._print_result(r)
        all_results.append(r)
        time.sleep(SAFE_DURATION)

        r = self.stop()
        self._print_result(r)
        all_results.append(r)
        time.sleep(SAFE_STOP_WAIT)

        self._log("SAFE-DEMO: Complete.")
        return all_results

    def _print_result(self, r: dict):
        """Print a single result line."""
        status = "OK" if r.get("success") else "FAIL"
        code = r.get("status_code", "?")
        sname = r.get("status_name", "?")
        resp = r.get("response_parsed", r.get("response_body", ""))
        self._log(f"  [{status}] code={code} {sname}  resp={resp}")

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:12]
        print(f"[{ts}] {msg}")


def main():
    parser = argparse.ArgumentParser(description="Booster T1 ROS2 Control Client")
    parser.add_argument("command", nargs="?", default="status",
                        choices=["status", "mode", "prepare", "stand",
                                 "move", "stop", "turn", "safe-demo",
                                 "shoot", "visual-kick", "visual-kick-v2", "kick-demo"],
                        help="Control command")
    parser.add_argument("--timeout", type=float, default=SERVICE_TIMEOUT,
                        help=f"Service call timeout (default: {SERVICE_TIMEOUT}s)")
    args = parser.parse_args()

    rclpy.init(args=sys.argv)

    client = T1ControlClient()
    results = []

    # Ensure Stop is attempted on exit
    def cleanup(signum=None, frame=None):
        print("\n[cleanup] Sending Stop...")
        try:
            client.stop()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    try:
        if args.command == "status":
            r = client.get_status()
            client._print_result(r)
            results.append(r)

        elif args.command == "mode":
            r = client.get_mode()
            client._print_result(r)
            if r.get("response_parsed"):
                mode_num = r["response_parsed"].get("mode", "?")
                mode_name = {v: k for k, v in ROBOT_MODE.items()}.get(mode_num, f"unknown({mode_num})")
                print(f"  Current mode: {mode_name} (id={mode_num})")
            results.append(r)

        elif args.command == "prepare":
            r = client.change_mode(ROBOT_MODE["prepare"])
            client._print_result(r)
            results.append(r)

        elif args.command == "stand":
            # Stand = Prepare mode (robot stands on both feet)
            r = client.change_mode(ROBOT_MODE["prepare"])
            client._print_result(r)
            results.append(r)

        elif args.command == "move":
            r = client.move(SAFE_MAX_SPEED, 0.0, 0.0)
            client._print_result(r)
            results.append(r)
            # Auto-stop after move command (no duration = immediate stop)
            time.sleep(0.5)
            r = client.stop()
            client._print_result(r)
            results.append(r)

        elif args.command == "stop":
            r = client.stop()
            client._print_result(r)
            results.append(r)

        elif args.command == "turn":
            r = client.move(0.0, 0.0, SAFE_MAX_YAW_RATE)
            client._print_result(r)
            results.append(r)
            time.sleep(0.5)
            r = client.stop()
            client._print_result(r)
            results.append(r)

        elif args.command == "shoot":
            r = client.shoot()
            client._print_result(r); results.append(r)
        elif args.command == "visual-kick":
            r = client.visual_kick(0)
            client._print_result(r); results.append(r)
        elif args.command == "visual-kick-v2":
            r = client.visual_kick(1)
            client._print_result(r); results.append(r)
        elif args.command == "kick-demo":
            results = client.kick_demo()
        elif args.command == "safe-demo":
            results = client.safe_demo()

    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Guaranteed stop attempt
        print("[finally] Ensuring Stop is sent...")
        try:
            client.stop()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # Print summary
    print(f"\n{'='*60}")
    print(f"Results: {len(results)} call(s)")
    for i, r in enumerate(results):
        status = "OK" if r.get("success") else "FAIL"
        print(f"  [{i}] {r.get('label','?')}: {status} code={r.get('status_code','?')}")
    print(f"{'='*60}")

    # Save to JSONL
    try:
        out_path = "/home/plon/Workspace/booster_soccer_project/results/ros2_control_test.jsonl"
        with open(out_path, "a") as f:
            for r in results:
                f.write(json.dumps(r, default=str) + "\n")
        print(f"Results appended to: {out_path}")
    except Exception as e:
        print(f"[warn] Could not save results: {e}")


if __name__ == "__main__":
    main()
