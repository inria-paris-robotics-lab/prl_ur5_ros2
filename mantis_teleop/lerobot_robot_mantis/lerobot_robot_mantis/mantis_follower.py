"""MantisFollower: a lerobot `Robot` that drives the mantis LEFT arm over ROS2.

Topics, joint names, gripper widths and shaper limits are read from the teleop's own
YAML by importing teleop_mantis, and commands go through the same CommandShaper the
teleop uses, so a replayed episode runs through the stage that recorded it. Beyond
pass-through: a 100 Hz feeder thread interpolates between the fps-rate waypoints of
lerobot-replay, a first action far from the measured pose is reached by a slow ramp,
and that approach path is checked against the teleop's collision floors first.
"""
import importlib
import os
import sys
import threading
import time

import numpy as np

from lerobot.robots.robot import Robot

from .config_mantis_follower import MantisFollowerConfig

FEED_PERIOD_S = 0.01
DEFAULT_GAP_S = 1.0 / 15.0


class MantisFollower(Robot):
    config_class = MantisFollowerConfig
    name = "mantis_follower"

    def __init__(self, config: MantisFollowerConfig):
        super().__init__(config)
        self.config = config
        self._T = None
        self._node = None
        self._exe = None
        self._spin_thread = None
        self._feed_thread = None
        self._shaper = None
        self._grip = None
        self._grip_sent = None
        self._pos = {}
        self._pos_lock = threading.Lock()
        self._parked = {}
        self._first_action = True
        self._connected = False
        self._we_inited_rclpy = False
        self._feed_lock = threading.Lock()
        self._seg = None
        self._gap_ema = DEFAULT_GAP_S
        self._last_send_t = None
        self._ik = None

    def _teleop(self):
        """Import teleop_mantis (loads the YAML next to it, or $teleop_config)."""
        if self._T is None:
            path = os.path.abspath(self.config.teleop_config)
            os.environ.setdefault("teleop_config", path)
            sys.path.insert(0, os.path.dirname(path))
            self._T = importlib.import_module("teleop_mantis")
        return self._T

    @property
    def observation_features(self) -> dict:
        T = self._teleop()
        ft = {f"{j}.pos": float for j in T.ARM}
        ft["gripper.pos"] = float
        return ft

    @property
    def action_features(self) -> dict:
        return dict(self.observation_features)

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        pass

    def connect(self, calibrate: bool = True) -> None:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from rclpy.action import ActionClient
        from control_msgs.action import GripperCommand

        T = self._teleop()
        if not rclpy.ok():
            rclpy.init()
            self._we_inited_rclpy = True
        self._node = rclpy.create_node("mantis_follower")
        qos_js = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self._node.create_subscription(JointState, T.JOINT_STATES_TOPIC, self._on_js, qos_js)
        pub = self._node.create_publisher(Float64MultiArray, T.ARM_CMD_TOPIC, 10)
        self._shaper = T.CommandShaper(pub, T.OUT_RATE or 250.0, T.OUT_VEL, T.OUT_ACCEL, T.OUT_KP)
        self._grip = ActionClient(self._node, GripperCommand, T.GRIP_ACTION)

        self._exe = rclpy.executors.SingleThreadedExecutor()
        self._exe.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._exe.spin, daemon=True)
        self._spin_thread.start()

        t0 = time.monotonic()
        while time.monotonic() - t0 < self.config.connect_timeout:
            with self._pos_lock:
                if all(j in self._pos for j in T.ARM_CMD_JOINTS):
                    break
            time.sleep(0.05)
        else:
            self.disconnect()
            raise TimeoutError(
                f"no complete {T.JOINT_STATES_TOPIC} within {self.config.connect_timeout}s "
                f"— is the robot stack up?")
        with self._pos_lock:
            self._parked = {j: float(self._pos[j]) for j in T.ARM_CMD_JOINTS if j not in T.ARM}
        self._shaper.start()
        self._feed_thread = threading.Thread(target=self._feeder, daemon=True)
        self._connected = True
        self._feed_thread.start()
        self._node.get_logger().info(
            f"mantis_follower connected: cmd {T.ARM_CMD_TOPIC} @ {T.OUT_RATE:.0f} Hz, "
            f"{len(self._parked)} joints latched parked")

    def disconnect(self) -> None:
        if self._connected and self._shaper is not None:
            with self._feed_lock:
                seg = self._seg
            if seg is not None and seg[3] <= 0.3:
                t0 = time.monotonic()
                while time.monotonic() - t0 < 0.5:
                    with self._feed_lock:
                        if self._segment_value(time.monotonic())[1] >= 1.0:
                            break
                    time.sleep(0.02)
                time.sleep(0.15)
        self._connected = False
        if self._shaper is not None:
            self._shaper.stop()
            self._shaper = None
        if self._exe is not None:
            self._exe.shutdown()
            self._exe = None
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        if self._we_inited_rclpy:
            import rclpy
            if rclpy.ok():
                rclpy.shutdown()
            self._we_inited_rclpy = False

    def _on_js(self, msg):
        with self._pos_lock:
            self._pos.update(zip(msg.name, msg.position))

    def _segment_value(self, now):
        """(current point, progress s) of the active segment. Caller holds _feed_lock."""
        q_from, q_to, t0, T = self._seg
        s = 1.0 if T <= 0.0 else min(1.0, (now - t0) / T)
        return q_from + s * (q_to - q_from), s

    def _feeder(self):
        """100 Hz continuous-target feed (see module doc: judder fix + keepalive)."""
        while self._connected:
            with self._feed_lock:
                cur = self._segment_value(time.monotonic())[0] if self._seg is not None else None
            if cur is not None:
                self._set_target(cur)
            time.sleep(FEED_PERIOD_S)

    def _set_segment(self, q_from, q_to, T):
        with self._feed_lock:
            self._seg = (np.asarray(q_from, float), np.asarray(q_to, float),
                         time.monotonic(), float(T))

    def _wait_segment(self):
        """Block until the active segment completes (used for the long approach)."""
        while self._connected:
            with self._feed_lock:
                s = self._segment_value(time.monotonic())[1] if self._seg is not None else 1.0
            if s >= 1.0:
                return
            time.sleep(0.02)

    def _set_target(self, q_arm):
        T = self._T
        full = []
        m = dict(zip(T.ARM, q_arm))
        for j in T.ARM_CMD_JOINTS:
            if j in m:
                full.append(float(m[j]))
            elif j in self._parked:
                full.append(self._parked[j])
            else:
                raise RuntimeError(f"joint {j} neither driven nor latched")
        self._shaper.set_target(np.asarray(full, float))

    def get_observation(self) -> dict:
        T = self._teleop()
        with self._pos_lock:
            pos = dict(self._pos)
        obs = {f"{j}.pos": float(pos[j]) for j in T.ARM}
        obs["gripper.pos"] = float(pos.get(T.GRIP_JOINT, self._grip_sent
                                           if self._grip_sent is not None else T.GRIP_OPEN))
        return obs

    def send_action(self, action: dict) -> dict:
        T = self._teleop()
        q = np.array([float(action[f"{j}.pos"]) for j in T.ARM], float)
        now = time.monotonic()
        if self._first_action:
            self._first_action = False
            with self._pos_lock:
                meas = np.array([self._pos[j] for j in T.ARM], float)
            gap = float(np.max(np.abs(q - meas)))
            if gap > self.config.approach_tol:
                self._path_check(meas, q)
                v = min(self.config.approach_vel, 0.9 * T.OUT_VEL,
                        0.5 * T.MAX_JOINT_LEAD * T.OUT_KP)
                self._node.get_logger().info(
                    f"pre-positioning to the first waypoint ({gap:.2f} rad away, "
                    f"{gap / v:.1f}s at {v:.2f} rad/s, path collision-checked)")
                self._set_segment(meas, q, gap / max(v, 1e-6))
                self._wait_segment()
            else:
                self._set_segment(meas, q, max(gap / 0.5, 0.02))
        else:
            if self._last_send_t is not None:
                g = min(max(now - self._last_send_t, 0.02), 0.3)
                self._gap_ema += 0.3 * (g - self._gap_ema)
            with self._feed_lock:
                cur = self._segment_value(now)[0] if self._seg is not None else q
            self._set_segment(cur, q, self._gap_ema)
        self._last_send_t = now
        if "gripper.pos" in action:
            g = float(action["gripper.pos"])
            if self._grip_sent is None or abs(g - self._grip_sent) > self.config.gripper_tol:
                self._send_grip(g)
        return dict(action)

    def _path_check(self, q_from, q_to):
        """Sample the straight joint segment through the teleop's collision floors and refuse a colliding approach."""
        T = self._T
        if self._ik is None:
            self._node.get_logger().info(
                "building the collision model for the approach check (~seconds)...")
            self._ik = T.PinkIK(T.URDF, T.EE_FRAME, T.ARM, srdf_path=T.srdf_path(),
                                package_dirs=T.mesh_pkg_dirs(), locked_q=dict(self._parked))
        ik = self._ik
        if ik.geom is None:
            return
        q_from = np.asarray(q_from, float)
        delta = np.asarray(q_to, float) - q_from
        n = int(np.ceil(float(np.max(np.abs(delta))) / np.radians(2.0))) + 1
        for s in np.linspace(0.0, 1.0, min(max(n, 2), 400)):
            qf = ik.neutral()
            for j, v in zip(T.ARM, q_from + s * delta):
                qf[ik.qindex(j)] = float(v)
            m, pair = ik.margin_at(qf)
            if m < 0.0:
                raise RuntimeError(
                    f"approach path to the episode start COLLIDES ({pair}, "
                    f"{1000.0 * m:.1f} mm below its floor) — move the arm clear "
                    f"first (teleop HOME button), then rerun the replay")

    def _send_grip(self, width):
        T = self._T
        from control_msgs.action import GripperCommand
        if not self._grip.server_is_ready():
            self._node.get_logger().warn("gripper action server not ready", throttle_duration_sec=2.0)
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(width)
        goal.command.max_effort = T.GRIP_EFFORT
        self._grip.send_goal_async(goal)
        self._grip_sent = float(width)
