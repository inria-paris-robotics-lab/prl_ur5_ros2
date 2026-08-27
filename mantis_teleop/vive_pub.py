"""HTC Vive controller publisher: pose and buttons from SteamVR onto the /vive topics."""

import os
_profile = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fastdds_udp_only.xml")
if "FASTRTPS_DEFAULT_PROFILES_FILE" not in os.environ and os.path.exists(_profile):
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = _profile

import numpy as np
import openvr
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray

POSE_PREDICTION = float(os.environ.get("VIVE_POSE_PREDICTION", "0.05"))

TRACK_OK = getattr(openvr, "TrackingResult_Running_OK", 200)
_TRACK_NAMES = {getattr(openvr, _n): _n.replace("TrackingResult_", "")
                for _n in dir(openvr) if _n.startswith("TrackingResult_")
                and isinstance(getattr(openvr, _n), int)}


def track_name(r):
    return _TRACK_NAMES.get(int(r), f"tracking_result={int(r)}")


class VivePub(Node):
    def __init__(self):
        super().__init__("vive_pub")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.pose_pub = self.create_publisher(Float64MultiArray, "/vive/pose", qos)
        self.btn_pub = self.create_publisher(Float64MultiArray, "/vive/buttons", qos)
        self.vr = None
        self.dev = None
        self.create_timer(1.0/250.0, self.tick)
        self.get_logger().info(f"vive_pub up (pose prediction {POSE_PREDICTION*1000:.0f} ms)")

    def tick(self):
        if self.vr is None:
            try:
                self.vr = openvr.init(openvr.VRApplication_Other)
                self.get_logger().info("SteamVR connected")
            except Exception as exc:
                self.get_logger().info(f"Waiting for SteamVR: {exc}", throttle_duration_sec=2.0)
                return
        vr = self.vr
        UNIVERSE = openvr.TrackingUniverseRawAndUncalibrated
        PAD = 1 << openvr.k_EButton_SteamVR_Touchpad
        MENU = 1 << openvr.k_EButton_ApplicationMenu
        GRIPB = 1 << openvr.k_EButton_Grip
        TRIGB = 1 << openvr.k_EButton_SteamVR_Trigger
        poses = vr.getDeviceToAbsoluteTrackingPose(UNIVERSE, POSE_PREDICTION, openvr.k_unMaxTrackedDeviceCount)
        if self.dev is None or vr.getTrackedDeviceClass(self.dev) != openvr.TrackedDeviceClass_Controller:
            self.dev = next((i for i in range(openvr.k_unMaxTrackedDeviceCount) if vr.getTrackedDeviceClass(i) == openvr.TrackedDeviceClass_Controller), None)
        if self.dev is None:
            return
        p = poses[self.dev]
        if not p.bPoseIsValid:
            self.get_logger().warn(
                f"controller NOT TRACKED -> no pose published "
                f"(connected={bool(p.bDeviceIsConnected)}, {track_name(p.eTrackingResult)}); "
                f"teleop will refuse to engage (PAD ignored ... fresh=False)",
                throttle_duration_sec=2.0)
        elif p.eTrackingResult != TRACK_OK:
            self.get_logger().warn(
                f"controller tracking DEGRADED ({track_name(p.eTrackingResult)}) -> poses "
                f"published but unreliable; expect glitch-gate rejects and PAD ignored",
                throttle_duration_sec=5.0)
        if p.bPoseIsValid:
            m = p.mDeviceToAbsoluteTracking
            msg = Float64MultiArray()
            msg.data = [float(m[0][3]), float(m[1][3]), float(m[2][3]),
                        float(m[0][0]), float(m[0][1]), float(m[0][2]),
                        float(m[1][0]), float(m[1][1]), float(m[1][2]),
                        float(m[2][0]), float(m[2][1]), float(m[2][2])]
            self.pose_pub.publish(msg)
        res, state = vr.getControllerState(self.dev)
        if res:
            pad = 1.0 if (state.ulButtonPressed & PAD) else 0.0
            menu = 1.0 if (state.ulButtonPressed & MENU) else 0.0
            trig = float(state.rAxis[1].x)
            trig_click = 1.0 if (state.ulButtonPressed & TRIGB) else 0.0
            home = 1.0 if (state.ulButtonPressed & GRIPB) else 0.0
            bmsg = Float64MultiArray()
            bmsg.data = [trig, pad, menu, trig_click, home]
            self.btn_pub.publish(bmsg)


def main():
    import signal

    def _sigterm(*_):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _sigterm)
    rclpy.init()
    node = VivePub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.vr is not None:
            openvr.shutdown()
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()

if __name__ == "__main__":
    main()
