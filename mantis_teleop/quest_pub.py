"""Meta Quest 2 controller publisher, the Quest counterpart of vive_pub.py."""

import itertools
import os
import signal
import sys
import time
import math

_profile = os.path.join(os.path.dirname(os.path.abspath(__file__)), "fastdds_udp_only.xml")
if "FASTRTPS_DEFAULT_PROFILES_FILE" not in os.environ and os.path.exists(_profile):
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = _profile

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Float64MultiArray


def _env(name, default):
    return os.environ.get(name, default)


def _envf(name, default):
    try:
        return float(os.environ.get(name, default))
    except ValueError:
        return float(default)


BACKEND = _env("QUEST_BACKEND", "openvr").strip().lower()
HAND = _env("QUEST_HAND", "right").strip().lower()
POSE_TOPIC = _env("QUEST_POSE_TOPIC", "/vive/pose")
BUTTONS_TOPIC = _env("QUEST_BUTTONS_TOPIC", "/vive/buttons")
RATE = _envf("QUEST_RATE", "250")

POSE_PREDICTION = _envf("QUEST_POSE_PREDICTION", "0.05")

ENGAGE_BUTTON = _env("QUEST_ENGAGE_BUTTON", "thumbstick").strip().lower()
MENU_BUTTON = _env("QUEST_MENU_BUTTON", "b").strip().lower()
HOME_BUTTON = _env("QUEST_HOME_BUTTON", "a").strip().lower()
AXISLOCK_BUTTON = _env("QUEST_AXISLOCK_BUTTON", "grip").strip().lower()

TRIGGER_CLICK_SRC = _env("QUEST_TRIGGER_CLICK", "soft").strip().lower()
CLICK_ON = _envf("QUEST_CLICK_ON", "0.90")
CLICK_OFF = _envf("QUEST_CLICK_OFF", "0.60")

SCAN = ("--scan" in sys.argv) or _env("QUEST_SCAN", "0").strip().lower() not in ("0", "", "false", "no", "off")

_rot_off = _env("QUEST_ROT_OFFSET", "").strip()


def _orthonormalize(R):
    U, _, Vt = np.linalg.svd(R)
    R = U @ Vt
    if np.linalg.det(R) < 0.0:
        R[:, -1] *= -1.0
    return R


def _log3(R):
    """Rotation matrix -> rotation vector (axis * angle)."""
    c = (np.trace(R) - 1.0) * 0.5
    c = min(1.0, max(-1.0, c))
    ang = math.acos(c)
    if ang < 1e-8:
        return np.zeros(3)
    if ang > math.pi - 1e-6:
        A = (R + np.eye(3)) * 0.5
        k = int(np.argmax(np.diag(A)))
        axis = A[:, k] / max(math.sqrt(max(A[k, k], 1e-12)), 1e-12)
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        return axis * ang
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (ang / (2.0 * math.sin(ang)))


def _exp3(w):
    """Rotation vector -> rotation matrix (Rodrigues)."""
    th = float(np.linalg.norm(w))
    if th < 1e-9:
        return np.eye(3)
    k = w / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)


def _parse_rot_offset(spec):
    """"rx,ry,rz" in degrees -> body-fixed rotation matrix (identity if unset)."""
    if not spec:
        return None
    parts = [p for p in spec.replace(";", ",").split(",") if p.strip()]
    if len(parts) != 3:
        raise SystemExit("QUEST_ROT_OFFSET must be 'rx,ry,rz' in degrees, got %r" % spec)
    v = np.array([float(p) for p in parts]) * math.pi / 180.0
    return None if np.linalg.norm(v) < 1e-12 else _exp3(v)


ROT_OFFSET = _parse_rot_offset(_rot_off)


class Sample(object):
    """One controller sample: position, rotation, and the time it was VALID."""
    __slots__ = ("p", "R", "t")

    def __init__(self, p, R, t):
        self.p = p
        self.R = R
        self.t = t


_DEFAULT_BITS = {
    "thumbstick": 32,
    "trigger": 33,
    "grip": 2,
    "a": 7,
    "b": 1,
}

_UNIVERSES = {"raw": "TrackingUniverseRawAndUncalibrated",
              "standing": "TrackingUniverseStanding",
              "seated": "TrackingUniverseSeated"}


class OpenVRBackend(object):
    """Quest 2 through ALVR's SteamVR driver, the same API vive_pub.py uses."""

    name = "openvr"

    def __init__(self, log):
        import openvr
        self.openvr = openvr
        self.log = log
        self.vr = None
        self.dev = None
        self._dev_warned = False

        uni = _env("QUEST_UNIVERSE", "standing").strip().lower()
        self.universe = getattr(openvr, _UNIVERSES.get(uni, _UNIVERSES["standing"]))
        self.role = (openvr.TrackedControllerRole_LeftHand if HAND.startswith("l")
                     else openvr.TrackedControllerRole_RightHand)
        self.invalid = getattr(openvr, "k_unTrackedDeviceIndexInvalid", 0xFFFFFFFF)

        self.track_ok = getattr(openvr, "TrackingResult_Running_OK", 200)
        self._track_names = {getattr(openvr, n): n.replace("TrackingResult_", "")
                             for n in dir(openvr) if n.startswith("TrackingResult_")
                             and isinstance(getattr(openvr, n), int)}

        self.bits = {}
        for k, default in _DEFAULT_BITS.items():
            self.bits[k] = int(_envf("QUEST_BIT_" + k.upper(), default))

    def track_name(self, r):
        return self._track_names.get(int(r), "tracking_result=%d" % int(r))

    def _connect(self):
        if self.vr is not None:
            return True
        try:
            self.vr = self.openvr.init(self.openvr.VRApplication_Other)
            self.log.info("SteamVR connected (ALVR driver should list a Quest HMD + 2 controllers)")
            return True
        except Exception as exc:
            self.log.info("Waiting for SteamVR/ALVR: %s" % exc, throttle_duration_sec=2.0)
            return False

    def _resolve_device(self):
        vr = self.vr
        ok = (self.dev is not None
              and vr.getTrackedDeviceClass(self.dev) == self.openvr.TrackedDeviceClass_Controller)
        if ok:
            return
        idx = vr.getTrackedDeviceIndexForControllerRole(self.role)
        if idx != self.invalid and vr.getTrackedDeviceClass(idx) == self.openvr.TrackedDeviceClass_Controller:
            if idx != self.dev:
                self.log.info("using the %s controller (device %d)" % (HAND.upper(), idx))
            self.dev = idx
            self._dev_warned = False
            return
        self.dev = next((i for i in range(self.openvr.k_unMaxTrackedDeviceCount)
                         if vr.getTrackedDeviceClass(i) == self.openvr.TrackedDeviceClass_Controller), None)
        if self.dev is not None and not self._dev_warned:
            self._dev_warned = True
            self.log.warn("SteamVR has not assigned hand roles yet -> falling back to controller %d, "
                          "which may be the WRONG HAND. Wake both controllers (press a button on "
                          "each) and this resolves itself." % self.dev)

    def poll(self, now):
        """-> (Sample|None, logical-button dict|None)."""
        if not self._connect():
            return None, None
        vr = self.vr
        self._resolve_device()
        if self.dev is None:
            self.log.warn("no controller visible to SteamVR (is the headset awake and streaming?)",
                          throttle_duration_sec=2.0)
            return None, None

        poses = vr.getDeviceToAbsoluteTrackingPose(self.universe, POSE_PREDICTION,
                                                   self.openvr.k_unMaxTrackedDeviceCount)
        p = poses[self.dev]
        sample = None
        if not p.bPoseIsValid:
            self.log.warn(
                "controller NOT TRACKED -> no pose published "
                "(connected=%s, %s); teleop will refuse to engage (PAD ignored ... fresh=False). "
                "On a Quest this is usually the hand leaving the headset's view, or the headset "
                "having gone to sleep." % (bool(p.bDeviceIsConnected), self.track_name(p.eTrackingResult)),
                throttle_duration_sec=2.0)
        else:
            if p.eTrackingResult != self.track_ok:
                self.log.warn(
                    "controller tracking DEGRADED (%s) -> poses published but unreliable; "
                    "expect glitch-gate rejects and PAD ignored" % self.track_name(p.eTrackingResult),
                    throttle_duration_sec=5.0)
            m = p.mDeviceToAbsoluteTracking
            pos = np.array([m[0][3], m[1][3], m[2][3]], float)
            R = np.array([[m[0][0], m[0][1], m[0][2]],
                          [m[1][0], m[1][1], m[1][2]],
                          [m[2][0], m[2][1], m[2][2]]], float)
            sample = Sample(pos, R, now)

        btn = None
        res, state = vr.getControllerState(self.dev)
        if res:
            pressed = int(state.ulButtonPressed)
            btn = {k: bool(pressed & (1 << b)) for k, b in self.bits.items()}
            btn["trigger_analog"] = float(state.rAxis[1].x)
        return sample, btn

    def scan(self):
        """Human-readable dump for --scan."""
        if not self._connect():
            return "waiting for SteamVR/ALVR ..."
        vr = self.vr
        classes = {getattr(self.openvr, n): n.replace("TrackedDeviceClass_", "")
                   for n in dir(self.openvr) if n.startswith("TrackedDeviceClass_")
                   and isinstance(getattr(self.openvr, n), int)}
        lines = []
        for i in range(self.openvr.k_unMaxTrackedDeviceCount):
            cls = vr.getTrackedDeviceClass(i)
            if cls == self.openvr.TrackedDeviceClass_Invalid:
                continue
            role = vr.getControllerRoleForTrackedDeviceIndex(i)
            rname = {self.openvr.TrackedControllerRole_LeftHand: "LEFT",
                     self.openvr.TrackedControllerRole_RightHand: "RIGHT"}.get(role, "-")
            try:
                model = vr.getStringTrackedDeviceProperty(i, self.openvr.Prop_RenderModelName_String)
            except Exception:
                model = "?"
            lines.append("  device %2d  %-14s role=%-5s  %s"
                         % (i, classes.get(cls, "class=%d" % cls), rname, model))
        self._resolve_device()
        if self.dev is not None:
            res, state = vr.getControllerState(self.dev)
            if res:
                pressed = int(state.ulButtonPressed)
                touched = int(state.ulButtonTouched)
                bits = [b for b in range(64) if pressed & (1 << b)]
                lines.append("  selected device %d  pressed_bits=%s  touched_bits=%s"
                             % (self.dev, bits or "-",
                                [b for b in range(64) if touched & (1 << b)] or "-"))
                lines.append("  axes: " + "  ".join(
                    "a%d=(%+.2f,%+.2f)" % (i, state.rAxis[i].x, state.rAxis[i].y) for i in range(5)))
        return "\n".join(lines) if lines else "  (no devices)"

    def shutdown(self):
        if self.vr is not None:
            self.openvr.shutdown()


_ADB_KEYS = {
    "right": {"a": "A", "b": "B", "thumbstick": "RJ", "grip": "RG",
              "trigger": "RTr", "trigger_analog": "rightTrig", "pose": "r"},
    "left": {"a": "X", "b": "Y", "thumbstick": "LJ", "grip": "LG",
             "trigger": "LTr", "trigger_analog": "leftTrig", "pose": "l"},
}


class AdbBackend(object):
    """Quest 2 over adb via the oculus_reader APK: no SteamVR, no video stream."""

    name = "adb"

    def __init__(self, log):
        self.log = log
        self.reader = None
        self.keys = _ADB_KEYS["left" if HAND.startswith("l") else "right"]
        self.mode = _env("QUEST_ADB_MODE", "native").strip().lower()
        if self.mode not in ("native", "hold", "extrapolate"):
            raise SystemExit("QUEST_ADB_MODE must be native|hold|extrapolate, got %r" % self.mode)
        self._emitted_t = None
        self.max_extrap = _envf("QUEST_ADB_MAX_EXTRAP", "0.08")
        self.max_v = _envf("QUEST_ADB_MAX_V", "3.0")
        self.max_w = _envf("QUEST_ADB_MAX_W", "12.0")
        self.stale = _envf("QUEST_ADB_STALE", "0.15")
        self.keepalive = _envf("QUEST_ADB_KEEPALIVE", "0.05")
        self.prev = None
        self.last = None
        self._last_raw_t = 0.0
        self._last_read_t = 0.0
        self._emitted_at = 0.0
        self._missing_warned = set()
        self._keys_logged = False

    def _connect(self):
        if self.reader is not None:
            return True
        try:
            from oculus_reader.reader import OculusReader
        except ImportError:
            try:
                from oculus_reader import OculusReader
            except ImportError as exc:
                self.log.error("oculus_reader not importable (%s). Install it with:\n"
                               "  git clone https://github.com/rail-berkeley/oculus_reader\n"
                               "  pip install -e oculus_reader\n"
                               "and make sure `adb devices` lists the headset." % exc,
                               throttle_duration_sec=10.0)
                return False
        try:
            ip = _env("QUEST_ADB_IP", "").strip()
            self.reader = OculusReader(ip_address=ip) if ip else OculusReader()
            self.log.info("oculus_reader started (%s)" % ("wifi adb %s" % ip if ip else "USB adb"))
            return True
        except Exception as exc:
            self.log.warn("oculus_reader failed to start: %s (is the headset unlocked, in "
                          "Developer Mode, and authorised for adb?)" % exc,
                          throttle_duration_sec=5.0)
            self.reader = None
            return False

    def _get(self, buttons, logical, default=False):
        key = self.keys.get(logical)
        if key in buttons:
            return buttons[key]
        if key not in self._missing_warned:
            self._missing_warned.add(key)
            self.log.warn("oculus_reader reports no button '%s' (for '%s') -> treated as released. "
                          "Run with --scan to see the real key names." % (key, logical))
        return default

    def _read_raw(self, now):
        """Pull one sample from the headset; returns the logical button dict."""
        try:
            transforms, buttons = self.reader.get_transformations_and_buttons()
        except Exception as exc:
            self.log.warn("oculus_reader read failed: %s" % exc, throttle_duration_sec=5.0)
            return None
        if not buttons:
            return None
        if not self._keys_logged:
            self._keys_logged = True
            self.log.info("oculus_reader button keys: %s" % sorted(buttons.keys()))
        T = (transforms or {}).get(self.keys["pose"])
        if T is not None:
            T = np.asarray(T, float)
            if T.shape == (4, 4) and np.isfinite(T).all():
                self._last_read_t = now
                p, R = T[:3, 3].copy(), _orthonormalize(T[:3, :3])
                if self.last is None or not np.array_equal(p, self.last.p) \
                        or not np.array_equal(R, self.last.R):
                    self.prev, self.last = self.last, Sample(p, R, now)
                    self._last_raw_t = now
        trig = self._get(buttons, "trigger_analog", 0.0)
        if isinstance(trig, (list, tuple, np.ndarray)):
            trig = float(trig[0]) if len(trig) else 0.0
        out = {k: bool(self._get(buttons, k)) for k in ("a", "b", "thumbstick", "grip", "trigger")}
        out["trigger_analog"] = float(trig)
        return out

    def poll(self, now):
        if not self._connect():
            return None, None
        btn = self._read_raw(now)
        if self.last is None:
            return None, btn
        if (now - self._last_read_t) > self.stale:
            self.log.warn("no data from the headset for %.2f s -> pose stream held "
                          "(headset asleep? adb dropped? app not running?)"
                          % (now - self._last_read_t), throttle_duration_sec=2.0)
            return None, btn
        if self.mode == "native":
            if self._emitted_t == self._last_raw_t and (now - self._emitted_at) < self.keepalive:
                return None, btn
            self._emitted_t = self._last_raw_t
            self._emitted_at = now
            return Sample(self.last.p, self.last.R, now), btn
        if self.mode == "hold":
            return Sample(self.last.p, self.last.R, now), btn

        dt_ahead = (now - self.last.t) + POSE_PREDICTION
        dt_ahead = max(0.0, min(dt_ahead, self.max_extrap))
        if self.prev is None or dt_ahead <= 0.0:
            return Sample(self.last.p, self.last.R, now), btn
        dt = self.last.t - self.prev.t
        if dt <= 1e-4:
            return Sample(self.last.p, self.last.R, now), btn
        v = (self.last.p - self.prev.p) / dt
        w = _log3(self.last.R @ self.prev.R.T) / dt
        nv, nw = float(np.linalg.norm(v)), float(np.linalg.norm(w))
        if nv > self.max_v:
            v *= self.max_v / nv
        if nw > self.max_w:
            w *= self.max_w / nw
        return Sample(self.last.p + v * dt_ahead, _exp3(w * dt_ahead) @ self.last.R, now), btn

    def scan(self):
        if not self._connect():
            return "waiting for the headset over adb ..."
        try:
            transforms, buttons = self.reader.get_transformations_and_buttons()
        except Exception as exc:
            return "  read failed: %s" % exc
        if not buttons:
            return "  (no data yet — is the oculus_reader APK running on the headset?)"
        on = sorted(k for k, v in buttons.items() if v is True)
        an = {k: v for k, v in buttons.items() if not isinstance(v, bool)}
        return ("  poses: %s\n  pressed: %s\n  analog: %s"
                % (sorted((transforms or {}).keys()), on or "-", an))

    def shutdown(self):
        if self.reader is not None:
            try:
                self.reader.stop()
            except Exception:
                pass


class QuestPub(Node):
    def __init__(self):
        super().__init__("quest_pub")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.pose_pub = self.create_publisher(Float64MultiArray, POSE_TOPIC, qos)
        self.btn_pub = self.create_publisher(Float64MultiArray, BUTTONS_TOPIC, qos)

        log = self.get_logger()
        if BACKEND in ("openvr", "alvr", "steamvr"):
            self.backend = OpenVRBackend(log)
        elif BACKEND in ("adb", "oculus_reader", "usb"):
            self.backend = AdbBackend(log)
        else:
            raise SystemExit("QUEST_BACKEND must be 'openvr' or 'adb', got %r" % BACKEND)

        _assigned = {"QUEST_ENGAGE_BUTTON": ENGAGE_BUTTON, "QUEST_MENU_BUTTON": MENU_BUTTON,
                     "QUEST_HOME_BUTTON": HOME_BUTTON, "QUEST_AXISLOCK_BUTTON": AXISLOCK_BUTTON}
        for (va, na), (vb, nb) in itertools.combinations(_assigned.items(), 2):
            if na == nb:
                log.warn("%s and %s are both %r -> one press fires BOTH controls at once"
                         % (va, vb, na))
        _valid = ("thumbstick", "grip", "a", "b", "trigger")
        for var, name in _assigned.items():
            if name not in _valid:
                log.error("%s=%r is not one of %s -> that button will NEVER read as pressed"
                          % (var, name, "/".join(_valid)))

        self._click_latched = False

        if SCAN:
            self.create_timer(0.2, self.scan_tick)
            log.info("SCAN MODE — no topics are published. Press the buttons one at a time and "
                     "watch which bit / key lights up.")
            return

        self.create_timer(1.0 / RATE, self.tick)
        if self.backend.name == "adb" and getattr(self.backend, "mode", "") == "native":
            rate_txt = "poll %.0f Hz -> publish ~72 Hz (native, no prediction)" % RATE
        else:
            rate_txt = "%.0f Hz, prediction %.0f ms" % (RATE, POSE_PREDICTION * 1000.0)
        log.info("quest_pub up | backend=%s hand=%s | pose->%s buttons->%s | %s"
                 % (self.backend.name, HAND.upper(), POSE_TOPIC, BUTTONS_TOPIC, rate_txt))
        if self.backend.name == "adb" and getattr(self.backend, "mode", "") == "extrapolate":
            log.warn("QUEST_ADB_MODE=extrapolate: measured on hardware 2026-08-10 to produce "
                     "up to 158 mm jumps between ticks (39 m/s implied) and to trip the teleop's "
                     "glitch gate into TRACKING DEGRADED. Re-run .quest_adb_diag.py before trusting it.")
        log.info("buttons: engage/freeze=%s  EPISODE=%s  HOME=%s  AXIS-LOCK=%s  gripper=trigger click (%s%s)"
                 % (ENGAGE_BUTTON, MENU_BUTTON, HOME_BUTTON, AXISLOCK_BUTTON, TRIGGER_CLICK_SRC,
                    ", on>%.2f off<%.2f" % (CLICK_ON, CLICK_OFF) if TRIGGER_CLICK_SRC == "soft" else ""))
        if ROT_OFFSET is not None:
            log.warn("QUEST_ROT_OFFSET=%s active -> the published controller frame is ROTATED. "
                     "Re-check the orientation regression before using this on hardware." % _rot_off)

    def scan_tick(self):
        print("\n--- %s scan ---\n%s" % (self.backend.name, self.backend.scan()), flush=True)

    def _click(self, btn):
        """Gripper-toggle edge, with hysteresis when derived from the analog axis."""
        if TRIGGER_CLICK_SRC == "hw":
            return bool(btn.get("trigger", False))
        a = float(btn.get("trigger_analog", 0.0))
        if self._click_latched:
            if a < CLICK_OFF:
                self._click_latched = False
        elif a >= CLICK_ON:
            self._click_latched = True
        return self._click_latched

    def tick(self):
        now = time.monotonic()
        sample, btn = self.backend.poll(now)
        if sample is not None:
            R = _orthonormalize(sample.R)
            if ROT_OFFSET is not None:
                R = R @ ROT_OFFSET
            p = sample.p
            if np.isfinite(p).all() and np.isfinite(R).all():
                msg = Float64MultiArray()
                msg.data = [float(p[0]), float(p[1]), float(p[2]),
                            float(R[0][0]), float(R[0][1]), float(R[0][2]),
                            float(R[1][0]), float(R[1][1]), float(R[1][2]),
                            float(R[2][0]), float(R[2][1]), float(R[2][2])]
                self.pose_pub.publish(msg)
            else:
                log = getattr(self, "get_logger", None)
                if log is not None:
                    log().warn("non-finite pose from the backend -> dropped",
                               throttle_duration_sec=2.0)
        if btn is not None:
            bmsg = Float64MultiArray()
            bmsg.data = [float(btn.get("trigger_analog", 0.0)),
                         1.0 if btn.get(ENGAGE_BUTTON, False) else 0.0,
                         1.0 if btn.get(MENU_BUTTON, False) else 0.0,
                         1.0 if self._click(btn) else 0.0,
                         1.0 if btn.get(HOME_BUTTON, False) else 0.0,
                         1.0 if btn.get(AXISLOCK_BUTTON, False) else 0.0]
            self.btn_pub.publish(bmsg)


def _sigterm(*_):
    raise KeyboardInterrupt


def main():
    signal.signal(signal.SIGTERM, _sigterm)
    rclpy.init()
    node = QuestPub()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.backend.shutdown()
        node.destroy_node()
        rclpy.ok() and rclpy.shutdown()


if __name__ == "__main__":
    main()
