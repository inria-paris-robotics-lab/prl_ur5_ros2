"""Mantis (dual UR5) VR teleop bridge: VR controller pose -> Pink diff-IK -> left arm."""

import os
import yaml
import time
import threading
import warnings
import numpy as np
import pinocchio as pin
import qpsolvers
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from std_msgs.msg import Float64MultiArray
from pink import Configuration, solve_ik
from pink.exceptions import PinkError
from pink.tasks import FrameTask, PostureTask
from pink.limits import ConfigurationLimit, VelocityLimit
from scipy.spatial import ConvexHull
from ament_index_python.packages import get_package_share_directory

path = os.environ.get("teleop_config", os.path.join(os.path.dirname(os.path.abspath(__file__)), "config_teleop_mantis.yaml"))
with open(path) as f:
    CFG = yaml.safe_load(f)

for _k in ("collision_barrier", "d_min", "d_min_self", "d_influence", "barrier_gain", "barrier_safe_gain",
           "self_collision_min_hops", "drop_dist_thresh", "n_collision_pairs"):
    if _k in (CFG.get("teleop") or {}):
        print(f"WARNING: teleop.{_k} has moved to ik.{_k}; the value under teleop is IGNORED "
              f"(effective: {CFG.get('ik', {}).get(_k, 'built-in default')})")

_CFG_DIR = os.path.dirname(os.path.abspath(path))


def _cfg_path(p):
    return p if os.path.isabs(p) else os.path.join(_CFG_DIR, p)

URDF = _cfg_path(CFG["urdf"])
EE_FRAME = CFG["ee_frame"]
ARM = list(CFG["arm"])
ARM_CMD_JOINTS = list(CFG["arm_cmd_joints"])
SHOULDER_JOINT = CFG.get("shoulder_joint", "left_shoulder_lift_joint")
LOCKED_Q = dict(CFG.get("locked_q") or {})
LOCK_OPEN = list(CFG.get("ik", {}).get("lock_open_joints", ["left_gripper_joint"]))

# Robot-shape generalization. Every default below reproduces the mantis 6-DOF UR5 exactly, so
# an existing config is unchanged; a differently shaped arm (e.g. a 7-DOF Franka) overrides them
# in its own config.
#   reach_shell_joints : joints varied to sample the reach envelope; default = all but the last
#                        two orientation joints (ARM[:4] on a 6-DOF arm).
#   roll_joint         : the tool-roll joint for the pour gesture; default = the last arm joint
#                        (wrist_3 on a UR, joint7 on a Franka).
#   wrist_vel_joints   : joints capped at ik.wrist_vel_scale; default = any joint named "*wrist*".
#   sing_report_joint  : joint value shown in the near-singularity log (diagnostic only).
REACH_SHELL_JOINTS = list(CFG["teleop"].get("reach_shell_joints") or ARM[:max(1, len(ARM) - 2)])
ROLL_JOINT = CFG["teleop"].get("roll_joint")            # None -> the last arm joint
WRIST_VEL_JOINTS = CFG["ik"].get("wrist_vel_joints")    # None -> substring match on "wrist"
SING_REPORT_JOINT = CFG["teleop"].get("sing_report_joint") or (ARM[4] if len(ARM) > 4 else ARM[-1])

RATE = float(CFG["rate"])
DT = 1.0 / RATE

POSITION_COST = float(CFG["ik"]["position_cost"])
ORIENTATION_COST = float(CFG["ik"]["orientation_cost"])
LM_DAMPING = float(CFG["ik"]["lm_damping"])
TASK_GAIN = float(CFG["ik"]["task_gain"])
POSTURE_COST = float(CFG["ik"]["posture_cost"])
VEL_SCALE = float(CFG["ik"]["vel_scale"])
WRIST_VEL_SCALE = float(CFG["ik"].get("wrist_vel_scale", 0.0))
COLLISION_MARGIN = float(CFG["ik"]["collision_margin"])
TCP_OFFSET = CFG["ik"].get("tcp_offset", None)
TCP_AUTO_KEYWORDS = tuple(CFG["ik"].get("tcp_auto_keywords", ["finger"]))
SING_SIGMA0 = float(CFG["ik"].get("sing_sigma0", 0.0))
SING_LAMBDA = float(CFG["ik"].get("sing_lambda", 0.0))
QP_DAMPING = 1e-12

COLLISION_BARRIER = bool(CFG["ik"].get("collision_barrier", True))
D_MIN = float(CFG["ik"].get("d_min", 0.015))
D_MIN_TABLE = float(CFG["ik"].get("d_min_table", CFG["ik"].get("d_min", 0.015)))
D_MIN_TABLE_GRIP = float(CFG["ik"].get("d_min_table_grip", D_MIN_TABLE))
GRIP_TABLE_KEYWORDS = tuple(CFG["ik"].get("grip_table_keywords", ["finger"]))
D_MIN_SELF = float(CFG["ik"].get("d_min_self", 0.006))
BARRIER_GAIN = float(CFG["ik"].get("barrier_gain", 15.0))
BARRIER_SAFE_GAIN = float(CFG["ik"].get("barrier_safe_gain", 0.0))
SELF_MIN_HOPS = int(CFG["ik"].get("self_collision_min_hops", 2))
N_COLLISION_PAIRS = int(CFG["ik"].get("n_collision_pairs", 32))
WIRE_BOX_KEYWORDS = tuple(CFG["ik"].get("wire_box_keywords", ["connector"]))
WIRE_BOX_DMIN = float(CFG["ik"].get("wire_box_dmin", CFG["ik"].get("d_min_self", 0.006)))
DROP_DIST_THRESH = float(CFG["ik"].get("drop_dist_thresh", 0.03))
HULL_SPLIT = dict(CFG["ik"].get("hull_split", {"left_forearm_link_0": 4,
                                               "left_upper_arm_link_0": 4}) or {})
RETREAT_RATE = float(CFG["ik"].get("retreat_rate", 0.05))
RETREAT_VEL_FRAC = float(CFG["ik"].get("retreat_vel_frac", 0.5))
ENV_KEYWORDS = tuple(CFG["ik"].get("env_keywords", ["table", "wall", "floor", "ground"]))
PRUNE_SAMPLES = int(CFG["ik"].get("prune_samples", 20000))
PRUNE_MARGIN = float(CFG["ik"].get("prune_margin", 0.05))

SCALE = float(CFG["teleop"]["scale"])
AZ_GAIN = float(CFG["teleop"]["az_gain"])
REACH_LO_FRAC = float(CFG["teleop"]["reach_lo_frac"])
REACH_HI_FRAC = float(CFG["teleop"]["reach_hi_frac"])

AXIS_SIGN = np.array(CFG["teleop"]["axis_sign"])
AXIS_MAP = list(CFG["teleop"]["axis_map"])
M = np.zeros((3, 3))
for i in range(3):
    M[i, AXIS_MAP[i]] = AXIS_SIGN[i]

ORI_SIGN = np.array(CFG["teleop"].get("ori_sign", [1.0, 1.0, 1.0]), float)

YAW_FALLBACK_SIN = np.sin(np.radians(float(CFG["teleop"].get("yaw_fallback_deg", 15.0))))

MAX_TARGET_SPEED = float(CFG["teleop"].get("max_target_speed", 1.2))
MAX_LEAD = float(CFG["teleop"].get("max_lead", 0.08))
MAX_ANG_SPEED = float(CFG["teleop"].get("max_ang_speed", 4.0))
MAX_ANG_LEAD = float(CFG["teleop"].get("max_ang_lead", 0.5))
ORI_ALPHA = float(CFG["teleop"].get("ori_alpha", 0.8))
ORI_BETA = float(CFG["teleop"].get("ori_beta", 0.0))
ORI_D_CUTOFF = float(CFG["teleop"].get("ori_d_cutoff", 1.0))
ORI_CUTOFF_MAX = float(CFG["teleop"].get("ori_cutoff_max", 0.0))
ORI_NOMINAL_DT = 0.01
ORI_MIN_CUTOFF = float(CFG["teleop"].get("ori_min_cutoff", 0.0)) or (
    1.0 / (2.0 * np.pi * (ORI_NOMINAL_DT * (1.0 - ORI_ALPHA) / ORI_ALPHA))
    if 0.0 < ORI_ALPHA < 1.0 else 0.0)
FILTER_MIN_CUTOFF = float(CFG["teleop"].get("filter_min_cutoff", 1.5))
FILTER_BETA = float(CFG["teleop"].get("filter_beta", 15.0))
MEDIAN_N = int(CFG["teleop"].get("pose_median", 5))
POSE_TIMEOUT = float(CFG["teleop"].get("pose_timeout", 0.2))
POSE_HZ_WARN = float(CFG["teleop"].get("pose_hz_warn", 65.0))
PIVOT_MIN_ROT = float(CFG["teleop"].get("pivot_min_rot", 0.05))
POSE_JUMP_LIN = float(CFG["teleop"].get("pose_jump_lin", 0.0))
POSE_JUMP_ANG = float(CFG["teleop"].get("pose_jump_ang", 0.0))
JOINT_TIMEOUT = float(CFG["teleop"].get("joint_timeout", 0.3))
BUTTONS_TIMEOUT = float(CFG["teleop"].get("buttons_timeout", 0.5))
PAD_DEBOUNCE = float(CFG["teleop"].get("pad_debounce", 0.25))
PAD_BOUNCE = float(CFG["teleop"].get("pad_bounce", 0.08))
HOME_Q = np.radians(np.asarray(CFG["teleop"].get(
    "home_q_deg", [0.0, -110.0, 66.0, -90.0, -90.0, 0.0]), float))
HOME_VEL = float(CFG["teleop"].get("home_vel", 0.3))
HOME_CHECK_STEP = np.radians(float(CFG["teleop"].get("home_check_step_deg", 2.0)))
HOME_DEBOUNCE = float(CFG["teleop"].get("home_debounce", 0.5))
MENU_DEBOUNCE = float(CFG["teleop"].get("menu_debounce", 0.5))
HOME_DONE_TOL = float(CFG["teleop"].get("home_done_tol", 0.05))
HOME_SETTLE_GRACE = float(CFG["teleop"].get("home_settle_grace", 3.0))
BLEND_TICKS = int(CFG["teleop"].get("disengage_blend_ticks", 5))
MAX_JOINT_LEAD = float(CFG["teleop"].get("max_joint_lead", 0.15))
CLAMP_ASYM = bool(CFG["teleop"].get("clamp_asymmetric", True))
CLAMP_HIST = max(1, int(CFG["teleop"].get("clamp_hist_ticks", 5)))
CLAMP_DCMD_EPS = float(CFG["teleop"].get("clamp_dcmd_eps", 0.02))
CLAMP_REANCHOR = bool(CFG["teleop"].get("clamp_reanchor_ori", True))
W_ERR_CAP = float(CFG["teleop"].get("w_err_cap", 4.712))
ROLL_GATE = bool(CFG["teleop"].get("roll_gate", True))
ROLL_GATE_ALLOW = float(CFG["teleop"].get("roll_gate_allow", 0.0))
ROLL_GATE_MARGIN = float(CFG["teleop"].get("roll_gate_margin", 0.006))
ROLL_GATE_PROBE = float(CFG["teleop"].get("roll_gate_probe", 0.03))
ROLL_GATE_LIMIT = float(CFG["teleop"].get("roll_gate_limit", 0.09))
ROLL_GATE_SLOPE = float(CFG["teleop"].get("roll_gate_slope", 0.003))
ROLL_GATE_RIDE = float(CFG["teleop"].get("roll_gate_ride", 0.0015))
ROLL_GATE_HOLD = max(1, int(CFG["teleop"].get("roll_gate_hold", 3)))
MEAS_GATE_FRAC = float(CFG["teleop"].get("meas_gate_frac", 0.6))
NOT_FOLLOW_LEAD = float(CFG["teleop"].get("not_follow_lead", 0.14))
NOT_FOLLOW_T = float(CFG["teleop"].get("not_follow_time", 0.6))
VR_RATE = float(CFG["teleop"].get("vr_rate", 250.0))
VR_DT = 1.0 / VR_RATE
MAX_ANG_STEP = MAX_ANG_SPEED * DT
ANTIWIND_RATE = float(CFG["teleop"].get("antiwind_rate", 0.25))
ANTIWIND_STUCK_SPEED = float(CFG["teleop"].get("antiwind_stuck_speed", 0.02))
AXIS_LOCK = bool(CFG["teleop"].get("axis_lock", True))
AXIS_LOCK_MODE = str(CFG["teleop"].get("axis_lock_mode", "hold")).strip().lower()
AXIS_LOCK_AXIS = CFG["teleop"].get("axis_lock_axis", None)
AXIS_LOCK_DEBOUNCE = float(CFG["teleop"].get("axis_lock_debounce", 0.15))
ORI_ANTIWIND_RATE = float(CFG["teleop"].get("ori_antiwind_rate", 0.8))
ORI_ANTIWIND_STUCK = float(CFG["teleop"].get("ori_antiwind_stuck_speed", 0.05))
ORI_ANTIWIND_DEADBAND = float(CFG["teleop"].get("ori_antiwind_deadband", 0.1))

PARKED_SNAPSHOT_TIMEOUT = float(CFG["teleop"].get("parked_snapshot_timeout", 5.0))
HOLD_PARKED = bool(CFG["teleop"].get("hold_parked_joints", True))
DEBUG_PERIOD = float(CFG["teleop"].get("debug_period", 0.5))
SHAKE_WATCH = float(CFG["teleop"].get("shake_watch", 1.0))
SHAKE_MIN_PP = float(CFG["teleop"].get("shake_min_pp", 0.02))
SHAKE_MIN_HZ = float(CFG["teleop"].get("shake_min_hz", 0.5))
SHAKE_SPLIT_HZ = float(CFG["teleop"].get("shake_split_hz", 3.0))
SHAKE_MAX_HZ = float(CFG["teleop"].get("shake_max_hz", 25.0))
TCP_CUTOFF = float(CFG["teleop"].get("tcp_cutoff", 3.0))
DEGRADE_MAX_RATE = float(CFG["teleop"].get("degrade_max_rate", 5.0))
DEGRADE_WINDOW = float(CFG["teleop"].get("degrade_window", 1.0))
DEGRADE_HOLD = float(CFG["teleop"].get("degrade_hold", 0.3))
ORI_ANTIALIAS = bool(CFG["teleop"].get("ori_antialias", True))

OUT_RATE = float(CFG["teleop"].get("out_rate", 250.0))
OUT_VEL = float(CFG["teleop"].get("out_vel", 1.5))
OUT_ACCEL = float(CFG["teleop"].get("out_accel", 10.0))
OUT_KP = float(CFG["teleop"].get("out_kp", 20.0))
OUT_FF = bool(CFG["teleop"].get("out_ff", False))
OUT_FF_TAU = float(CFG["teleop"].get("out_ff_tau", 0.1))
OUT_STALE = max(3.0 * DT, 0.05)

GRIP_OPEN = float(CFG["gripper"]["grip_open"])
GRIP_CLOSE = float(CFG["gripper"]["grip_close"])
GRIP_ACTION = CFG["gripper"]["action"]
GRIP_EFFORT = float(CFG["gripper"].get("effort", 40.0))
GRIP_DEBOUNCE = float(CFG["gripper"].get("toggle_debounce", 0.25))
CLICK_THRESH = float(CFG["gripper"].get("click_thresh", 0.99))
GRIP_JOINT = str(CFG["gripper"].get("grip_joint", "left_gripper_joint") or "")
GRIP_RETRY = float(CFG["gripper"].get("retry_period", 1.0))
GRIP_RETRY_TOL = float(CFG["gripper"].get("retry_tol", 0.01))
GRIP_DT = 1.0 / max(1.0, float(CFG["gripper"].get("rate", 50.0)))

ARM_CMD_TOPIC = CFG["topics"]["arm_cmd"]
JOINT_STATES_TOPIC = CFG["topics"]["joint_states"]
VIVE_POSE_TOPIC = CFG["topics"].get("vive_pose", "/vive/pose")
VIVE_BUTTONS_TOPIC = CFG["topics"].get("vive_buttons", "/vive/buttons")

RLIMITS = {k: tuple(v) for k, v in (CFG.get("rlimits") or {}).items()}

RECORD_CFG = dict(CFG.get("record") or {})


def mesh_pkg_dirs():
    """Package dirs used to resolve mesh paths referenced by the URDF."""
    prefix = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    return [os.path.join(p, "share") for p in prefix if p]


def srdf_path():
    """Path to the MoveIt SRDF, used to disable allowed collision pairs."""
    return _cfg_path(CFG["srdf"])


class OneEuroFilter:
    """3D one-euro filter: low lag in motion, smooths jitter at rest; cutoff_max caps the adaptive cutoff (0 = uncapped)."""

    def __init__(self, min_cutoff=1.0, beta=0.0, d_cutoff=1.0, cutoff_max=0.0):
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        self.cutoff_max = float(cutoff_max)
        self.x_prev = None
        self.t_prev = None
        self.dx_prev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, x, t):
        x = np.asarray(x, float)
        if self.t_prev is None:
            self.t_prev = t
            self.x_prev = x
            self.dx_prev = np.zeros_like(x)
            return x
        dt = t - self.t_prev
        if dt <= 0.0:
            return self.x_prev
        dx = (x - self.x_prev) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        dx_hat = a_d * dx + (1.0 - a_d) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(dx_hat))
        if self.cutoff_max > 0.0:
            cutoff = min(cutoff, self.cutoff_max)
        a = self._alpha(cutoff, dt)
        x_hat = a * x + (1.0 - a) * self.x_prev
        self.t_prev = t
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        return x_hat


class CommandShaper:
    """Publishes the latest IK joint vector at out_rate under per-joint velocity and acceleration limits."""

    def __init__(self, publisher, rate, v_max, a_max, kp, ff_tau=0.0):
        self.pub = publisher
        self.rate = float(rate)
        self.v_max = float(v_max)
        self.a_max = float(a_max)
        self.kp = float(kp)
        self.ff_tau = float(ff_tau)
        self._lock = threading.Lock()
        self._target = None
        self._v_ff = None
        self._ff_f = None
        self._t_target = 0.0
        self._q = None
        self._v = None
        self._dts = []
        self._stop = False
        self._pub_errors = 0
        self._snaps = 0
        self.resync_snap = MAX_JOINT_LEAD
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop = True
        self._thread.join(timeout=1.0)

    def set_target(self, vec, v_ff=None):
        """Set the tracked joint vector; v_ff is its own velocity, None marks a discontinuity (engage, re-anchor, freeze)."""
        vec = np.asarray(vec, float)
        with self._lock:
            self._target = vec
            self._v_ff = None if v_ff is None else np.asarray(v_ff, float)
            self._t_target = time.monotonic()

    def stats(self):
        """(n, p50, p95, max, snaps) of publish intervals [ms] since last call."""
        with self._lock:
            arr, self._dts = self._dts, []
            snaps, self._snaps = self._snaps, 0
        if not arr:
            return None
        a = np.array(arr)
        return len(a), float(np.percentile(a, 50)), float(np.percentile(a, 95)), float(a.max()), snaps

    def _step(self, tgt, dt, v_ff=None):
        """One output sample: velocity- and acceleration-limited step toward the target with smoothed feed-forward."""
        if self._q is None or self._q.shape != tgt.shape:
            self._q = tgt.copy()
            self._v = np.zeros_like(tgt)
            self._ff_f = np.zeros_like(tgt)
            return self._q
        if self._ff_f is None or self._ff_f.shape != tgt.shape:
            self._ff_f = np.zeros_like(tgt)
        if v_ff is None:
            self._ff_f = np.zeros_like(tgt)
        elif self.ff_tau > 0.0:
            self._ff_f = self._ff_f + min(1.0, dt / self.ff_tau) * (v_ff - self._ff_f)
        else:
            self._ff_f = np.asarray(v_ff, float)
        err = tgt - self._q
        v_brake = np.sqrt(2.0 * self.a_max * np.abs(err))
        v_des = np.clip(self._ff_f + np.clip(self.kp * err, -v_brake, v_brake),
                        -self.v_max, self.v_max)
        self._v = self._v + np.clip(v_des - self._v, -self.a_max * dt, self.a_max * dt)
        self._q = self._q + self._v * dt
        return self._q

    def _run(self):
        period = 1.0 / self.rate
        next_t = time.monotonic()
        t_last = next_t
        while not self._stop:
            next_t += period
            delay = next_t - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()
            now = time.monotonic()
            dt = min(max(now - t_last, 0.25 * period), 4.0 * period)
            t_last = now
            with self._lock:
                tgt = self._target
                v_ff = self._v_ff
                fresh = tgt is not None and (now - self._t_target) < OUT_STALE
            if tgt is None:
                continue
            if not fresh:
                self._v = None if self._q is None else np.zeros_like(self._q)
                self._ff_f = None if self._q is None else np.zeros_like(self._q)
                continue
            if self._q is not None and self._q.shape == tgt.shape \
                    and float(np.max(np.abs(tgt - self._q))) > self.resync_snap:
                with self._lock:
                    self._snaps += 1
                self._q = None
            q = self._step(tgt, dt, v_ff)
            with self._lock:
                self._dts.append(dt * 1000.0)
            msg = Float64MultiArray()
            msg.data = [float(x) for x in q]
            try:
                self.pub.publish(msg)
            except Exception:
                if self._stop:
                    return
                self._pub_errors += 1
                if self._pub_errors > 50:
                    return


try:
    from pink.barriers import SelfCollisionBarrier as _SCBarrier
except Exception:
    _SCBarrier = None

if _SCBarrier is not None:
    class MarginSelfCollisionBarrier(_SCBarrier):
        """SelfCollisionBarrier with a per-pair floor: h_i = d_i - d_min_i (self pairs get D_MIN_SELF, everything else D_MIN)."""
        def __init__(self, n, gain, safe_displacement_gain, d_min_vec, pair_joints):
            d_min_vec = np.asarray(d_min_vec, float)
            super().__init__(n, gain=gain, safe_displacement_gain=safe_displacement_gain, d_min=float(d_min_vec.min()))
            self.d_min_vec = d_min_vec
            self.pair_joints = np.asarray(pair_joints, int)
            self._cq = None
            self._cm = None
            self._cidx = None
            self._cjac = None

        def _margins(self, configuration):
            if self._cm is not None and configuration.q is self._cq:
                return self._cm
            n_pairs = len(configuration.collision_model.collisionPairs)
            cd = configuration.collision_data
            d = np.fromiter((cd.distanceResults[k].min_distance for k in range(n_pairs)), float, n_pairs)
            m = d - self.d_min_vec[:n_pairs]
            self._cq, self._cm, self._cidx, self._cjac = configuration.q, m, None, None
            return m

        def _select(self, margins):
            if self._cidx is not None and margins is self._cm:
                return self._cidx
            if self.dim >= len(margins):
                idx = np.arange(len(margins))
            else:
                idx = np.argpartition(-margins, -self.dim)[-self.dim:]
            if margins is self._cm:
                self._cidx = idx
            return idx

        def compute_barrier(self, configuration):
            m = self._margins(configuration)
            return m[self._select(m)]

        def compute_jacobian(self, configuration):
            m = self._margins(configuration)
            if self._cjac is not None and m is self._cm:
                return self._cjac
            model, data = configuration.model, configuration.data
            cd = configuration.collision_data
            idxs = self._select(m)
            nv = model.nv
            n_sel = len(idxs)
            JJ = np.empty((model.njoints, 6, nv))
            for j in range(model.njoints):
                JJ[j] = pin.getJointJacobian(model, data, j, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
            W1 = np.empty((n_sel, 3))
            W2 = np.empty((n_sel, 3))
            for i, k in enumerate(idxs):
                dr = cd.distanceResults[int(k)]
                W1[i] = dr.getNearestPoint1()
                W2[i] = dr.getNearestPoint2()
            sep = W1 - W2
            ln = np.linalg.norm(sep, axis=1)
            good = ln > 0.0
            N = sep / np.where(good, ln, 1.0)[:, None]
            j1 = self.pair_joints[idxs, 0]
            j2 = self.pair_joints[idxs, 1]
            oMi = data.oMi
            T1 = np.array([oMi[int(j)].translation for j in j1])
            T2 = np.array([oMi[int(j)].translation for j in j2])
            A1 = np.concatenate([N, np.cross(W1 - T1, N)], axis=1)
            A2 = np.concatenate([N, np.cross(W2 - T2, N)], axis=1)
            rows = np.einsum('ij,ijk->ik', A1, JJ[j1]) - np.einsum('ij,ijk->ik', A2, JJ[j2])
            rows[~good] = 0.0
            J = np.zeros((self.dim, nv))
            J[:n_sel] = rows
            J = np.nan_to_num(J)
            if m is self._cm:
                self._cjac = J
            return J

        def active_rows(self, configuration, v, tol=1e-4):
            """Indices (into collisionPairs) of the barrier rows whose inequality J v >= -gain*h is tight for the solved velocity v — i.e."""
            if float(np.max(np.abs(v))) < 1e-3:
                return []
            m = self._margins(configuration)
            idxs = self._select(m)
            J = self.compute_jacobian(configuration)
            h = m[idxs]
            slack = J[:len(idxs)] @ v + self.gain[:len(idxs)] * h
            return [int(idxs[i]) for i in np.nonzero(slack <= tol)[0]]


class DistanceConfiguration(Configuration):
    """pink Configuration that computes collision distances only, skipping the unused boolean collision pass."""

    def update(self, q=None):
        if q is not None:
            q_readonly = q.copy()
            q_readonly.setflags(write=False)
            self.q = q_readonly
        if self.collision_model is not None:
            pin.computeDistances(self.model, self.data, self.collision_model,
                                 self.collision_data, self.q)
        pin.computeJointJacobians(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)


class PinkIK:
    """Pinocchio model + Pink diff-IK for the mantis left arm, with self- and environment-collision avoidance."""

    def __init__(self, urdf_path, ee_frame, arm_joints, position_cost=POSITION_COST, orientation_cost=ORIENTATION_COST, lm_damping=LM_DAMPING, gain=TASK_GAIN, posture_cost=POSTURE_COST, vel_scale=VEL_SCALE, solver=None, srdf_path=None, package_dirs=None, collision_margin=COLLISION_MARGIN, locked_q=None, logger=None):
        """Build the reduced model, collision geometry, tasks, joint limits, collision barrier and QP solver."""
        self.log = logger
        full = pin.buildModelFromUrdf(urdf_path)
        keep = set(arm_joints)
        locked = [full.getJointId(n) for n in full.names[1:] if n not in keep]
        q_ref = pin.neutral(full)
        if locked_q is None:
            locked_q = LOCKED_Q
        for jname, val in locked_q.items():
            if full.existJointName(jname):
                q_ref[full.joints[full.getJointId(jname)].idx_q] = float(val)
            else:
                self._warn(f"locked_q joint '{jname}' not in URDF, ignored")
        for jname in LOCK_OPEN:
            if not full.existJointName(jname):
                continue
            j = full.joints[full.getJointId(jname)]
            if j.nq == 1:
                q_ref[j.idx_q] = float(full.upperPositionLimit[j.idx_q])
                self._info(f"collision model: {jname} pinned OPEN at {q_ref[j.idx_q]:.4f}")
        self.locked_ref = {}
        for jid in locked:
            j = full.joints[jid]
            if j.nq == 1 and full.names[jid] not in LOCK_OPEN:
                self.locked_ref[full.names[jid]] = float(q_ref[j.idx_q])
        self.geom = None
        self.blocked = False
        self.active_pairs = []
        if srdf_path and package_dirs:
            geom_full = pin.buildGeomFromUrdf(full, urdf_path, pin.GeometryType.COLLISION, package_dirs=list(package_dirs))
            if locked:
                self.model, self.geom = pin.buildReducedModel(full, geom_full, locked, q_ref)
            else:
                self.model, self.geom = full, geom_full
            self._convexify()
            self._table_box_override()
            self.geom.addAllCollisionPairs()
            pin.removeCollisionPairs(self.model, self.geom, srdf_path, False)
            self._readd_wire_box_pairs(arm_joints)
            self._filter_pairs(arm_joints)
            self._apply_hull_split()
            self.geom_data = self.geom.createData()
            for req in self.geom_data.collisionRequests:
                req.security_margin = float(collision_margin)
            self.col_data = self.model.createData()
            self._gate_data = None
            self._gate_gdata = None
        else:
            self.model = pin.buildReducedModel(full, locked, q_ref) if locked else full
        self.data = self.model.createData()
        if not self.model.existFrame(ee_frame):
            raise ValueError(f"EE frame '{ee_frame}' not found in URDF")
        self.ee = ee_frame
        self.arm_joints = list(arm_joints)
        self._qidx = {j: self.model.joints[self.model.getJointId(j)].idx_q for j in self.arm_joints}
        self._roll_joint = ROLL_JOINT or self.arm_joints[-1]
        self._a3_local = self._joint_axis_local(self._roll_joint)
        self._roll_pairs_idx = None
        self.fix_limits(vel_scale)
        self.solver = solver or ("daqp" if "daqp" in qpsolvers.available_solvers else qpsolvers.available_solvers[0])
        self.ee_task = FrameTask(ee_frame, position_cost=position_cost, orientation_cost=orientation_cost, lm_damping=lm_damping, gain=gain)
        self.posture = PostureTask(cost=posture_cost)
        self.barrier = None
        self.pair_dmin = None
        if COLLISION_BARRIER and self.geom is not None:
            n_avail = len(self.geom.collisionPairs)
            if n_avail > 0:
                try:
                    if _SCBarrier is None:
                        raise ImportError("pink.barriers not importable")
                    arm_jids = {self.model.getJointId(j) for j in self.arm_joints if self.model.existJointName(j)}
                    on_arm = lambda gi: self.geom.geometryObjects[gi].parentJoint in arm_jids
                    grip_table_names, plain_table_names = [], []
                    def _floor(cp):
                        both = on_arm(cp.first) and on_arm(cp.second)
                        if both and (self._is_wire_box(cp.first) or self._is_wire_box(cp.second)):
                            return WIRE_BOX_DMIN
                        if both:
                            return D_MIN_SELF
                        na = self.geom.geometryObjects[cp.first].name.lower()
                        nb = self.geom.geometryObjects[cp.second].name.lower()
                        if "table" in na or "table" in nb:
                            other = nb if "table" in na else na
                            if any(k in other for k in GRIP_TABLE_KEYWORDS):
                                grip_table_names.append(other)
                                return D_MIN_TABLE_GRIP
                            plain_table_names.append(other)
                            return D_MIN_TABLE
                        return D_MIN
                    self.pair_dmin = np.array([_floor(cp) for cp in self.geom.collisionPairs])
                    pair_joints = np.array([[self.geom.geometryObjects[cp.first].parentJoint,
                                             self.geom.geometryObjects[cp.second].parentJoint]
                                            for cp in self.geom.collisionPairs], int)
                    n_bar = n_avail if N_COLLISION_PAIRS <= 0 else min(N_COLLISION_PAIRS, n_avail)
                    self.barrier = MarginSelfCollisionBarrier(n_bar, gain=BARRIER_GAIN, safe_displacement_gain=BARRIER_SAFE_GAIN, d_min_vec=self.pair_dmin, pair_joints=pair_joints)
                    n_self = int(np.sum(self.pair_dmin == D_MIN_SELF))
                    self._info(f"MarginSelfCollisionBarrier dim={n_bar} of {n_avail} pairs (d_min={D_MIN}, d_min_self={D_MIN_SELF} on {n_self} self pairs, gain={BARRIER_GAIN})")
                    self._info(f"  table floors: d_min_table_grip={D_MIN_TABLE_GRIP} on {len(grip_table_names)} pairs {sorted(grip_table_names)}; "
                               f"d_min_table={D_MIN_TABLE} on {len(plain_table_names)} pairs {sorted(plain_table_names)}")
                except Exception as exc:
                    self._warn(f"pink.barriers unavailable ({exc}) -> falling back to collision reject")
            else:
                self._warn("collision_barrier on, but 0 collision pairs after filtering")
        if self.barrier is not None:
            self.configuration = DistanceConfiguration(self.model, self.data, pin.neutral(self.model), collision_model=self.geom, collision_data=self.geom_data)
        else:
            self.configuration = Configuration(self.model, self.data, pin.neutral(self.model))
        self.posture.set_target(self.configuration.q)
        self.limits = [ConfigurationLimit(self.model), VelocityLimit(self.model)]
        self.resolve_tcp(TCP_OFFSET, TCP_AUTO_KEYWORDS)
        self.retreats = 0
        self.sigma_min = float("inf")
        self.sing_kind = "wrist"

    def _info(self, msg):
        if self.log is not None:
            self.log.info(msg)

    def _warn(self, msg, throttle=None):
        if self.log is not None:
            if throttle is None:
                self.log.warn(msg)
            else:
                self.log.warn(msg, throttle_duration_sec=throttle)

    def _convexify(self):
        """Replace BVH collision meshes with their convex hulls, which the barrier's distance queries require."""
        try:
            import coal as fcl
        except Exception:
            import hppfcl as fcl
        def _to_convex(V):
            hull = ConvexHull(V)
            used = np.unique(np.concatenate([hull.vertices, hull.simplices.ravel()]))
            remap = {int(old): new for new, old in enumerate(used)}
            pv = fcl.StdVec_Vec3s()
            for p in V[used]:
                pv.append(np.asarray(p, float))
            tris = fcl.StdVec_Triangle()
            for s in hull.simplices:
                tris.append(fcl.Triangle(remap[int(s[0])], remap[int(s[1])], remap[int(s[2])]))
            return fcl.Convex(pv, tris)
        n_hull, failed = 0, []
        self._split_hulls = {}
        for go in self.geom.geometryObjects:
            g = go.geometry
            if not isinstance(g, fcl.BVHModelBase):
                continue
            if go.name == "table_link_0":
                continue
            V = np.asarray(g.vertices())
            if go.name in HULL_SPLIT:
                try:
                    T = np.array([[t[0], t[1], t[2]] for t in
                                  (g.tri_indices(i) for i in range(g.num_tris))], int)
                    pieces = self._tri_slabs(_to_convex, V, T, int(HULL_SPLIT[go.name]))
                    if len(pieces) > 1:
                        self._split_hulls[go.name] = pieces
                except Exception as exc:
                    self._warn(f"hull split failed for {go.name} ({exc}) -> single hull")
            done = False
            for grid in (0.0, 0.002, 0.005, 0.01):
                try:
                    Vg = V if grid == 0.0 else np.unique(np.round(V / grid) * grid, axis=0)
                    go.geometry = _to_convex(Vg)
                    n_hull += 1
                    done = True
                    if grid > 0.0:
                        self._info(f"hull {go.name}: decimated at {grid*1000:.0f}mm grid")
                    break
                except Exception:
                    continue
            if not done:
                failed.append(go.name)
                self._warn(f"hull FAILED for {go.name} -> left as raw BVH, distances unreliable")
        self._info(f"built {n_hull} convex hulls" + (f", failed: {failed}" if failed else "")
                   + (f"; split {{{', '.join(f'{k}:{len(v)}' for k, v in self._split_hulls.items())}}}"
                      if self._split_hulls else ""))

    @staticmethod
    def _tri_slabs(to_convex, V, T, k):
        """Split a mesh into k convex slabs by triangle centroid along its longest axis; never over-reports a gap."""
        V = np.asarray(V, float)
        T = np.asarray(T, int)
        ax = int(np.argmax(V.max(0) - V.min(0)))
        cen = V[T].mean(axis=1)[:, ax]
        edges = np.linspace(cen.min(), cen.max(), int(k) + 1)
        edges[-1] += 1e-9
        pieces = []
        for i in range(int(k)):
            sel = T[(cen >= edges[i]) & (cen < edges[i + 1])]
            if len(sel) == 0:
                continue
            pts = V[np.unique(sel.ravel())]
            if len(pts) < 4:
                continue
            try:
                pieces.append(to_convex(pts))
            except Exception:
                continue
        return pieces

    def _apply_hull_split(self):
        """Expand every collision pair that references a split link into one pair per convex piece."""
        if not getattr(self, "_split_hulls", None):
            return
        name_to_gid = {go.name: i for i, go in enumerate(self.geom.geometryObjects)}
        sub = {}
        for name, pieces in self._split_hulls.items():
            gid = name_to_gid.get(name)
            if gid is None:
                continue
            base = self.geom.geometryObjects[gid]
            ids = []
            for i, cv in enumerate(pieces):
                go = pin.GeometryObject(base)
                go.name = f"{name}#{i}"
                go.geometry = cv
                ids.append(self.geom.addGeometryObject(go))
            sub[gid] = ids
        if not sub:
            return
        n_before = len(self.geom.collisionPairs)
        expanded = []
        for cp in self.geom.collisionPairs:
            a = list(sub.get(cp.first, [cp.first]))
            b = list(sub.get(cp.second, [cp.second]))
            for ia in a:
                for ib in b:
                    if ia != ib:
                        expanded.append(pin.CollisionPair(ia, ib))
        self.geom.removeAllCollisionPairs()
        for cp in expanded:
            self.geom.addCollisionPair(cp)
        self._info(f"hull split: {len(sub)} link(s) -> {sum(len(v) for v in sub.values())} conservative "
                   f"pieces; collision pairs {n_before} -> {len(expanded)}")

    def _table_box_override(self):
        """Replace the non-convex table mesh with a flat box snapped to its top surface."""
        try:
            import coal as fcl
        except Exception:
            import hppfcl as fcl
        for gid, go in enumerate(self.geom.geometryObjects):
            if go.name != "table_link_0":
                continue
            try:
                go.geometry.computeLocalAABB()
                al = go.geometry.aabb_local
                mn = np.array(al.min_); mx = np.array(al.max_)
                d = self.model.createData(); gd = self.geom.createData()
                pin.updateGeometryPlacements(self.model, d, self.geom, gd, pin.neutral(self.model))
                oMg = gd.oMg[gid]
                corners = np.array([[x, y, z] for x in (mn[0], mx[0]) for y in (mn[1], mx[1]) for z in (mn[2], mx[2])])
                wc = (oMg.rotation @ corners.T).T + oMg.translation
                wmin = wc.min(0); wmax = wc.max(0)
                sx = float(wmax[0] - wmin[0]) + 0.02
                sy = float(wmax[1] - wmin[1]) + 0.02
                sz = 0.10
                top = float(wmax[2])
                center = np.array([0.5 * (wmin[0] + wmax[0]), 0.5 * (wmin[1] + wmax[1]), top - 0.5 * sz])
                oMj = d.oMi[go.parentJoint]
                go.geometry = fcl.Box(sx, sy, sz)
                go.placement = oMj.inverse() * pin.SE3(np.eye(3), center)
                self._info(f"table override: mesh -> Box size=[{sx:.3f},{sy:.3f},{sz:.3f}] top={top:.3f}")
            except Exception as exc:
                self._warn(f"table override skipped: {exc}")
            break

    def _is_wire_box(self, gi):
        """True if geometry gi is a protruding wire-box link (by name substring)."""
        if not WIRE_BOX_KEYWORDS:
            return False
        n = self.geom.geometryObjects[gi].name.lower()
        return any(k in n for k in WIRE_BOX_KEYWORDS)

    def _readd_wire_box_pairs(self, arm_joints):
        """Re-add wire-box vs arm-link pairs disabled by the SRDF, skipping the box's own wrist_3 cluster."""
        if not WIRE_BOX_KEYWORDS:
            return
        arm_jids = {self.model.getJointId(j) for j in arm_joints if self.model.existJointName(j)}
        on_arm = lambda gi: self.geom.geometryObjects[gi].parentJoint in arm_jids
        existing = {tuple(sorted((cp.first, cp.second))) for cp in self.geom.collisionPairs}
        boxes = [gi for gi in range(len(self.geom.geometryObjects)) if self._is_wire_box(gi)]
        added = 0
        for bi in boxes:
            jb = self.geom.geometryObjects[bi].parentJoint
            for gi in range(len(self.geom.geometryObjects)):
                if gi == bi or self._is_wire_box(gi) or not on_arm(gi):
                    continue
                if self.geom.geometryObjects[gi].parentJoint == jb:
                    continue
                key = tuple(sorted((bi, gi)))
                if key in existing:
                    continue
                self.geom.addCollisionPair(pin.CollisionPair(bi, gi))
                existing.add(key)
                added += 1
        if added:
            self._info(f"wire-box: re-added {added} box<->arm pairs (checked vs every arm link except own wrist_3 cluster)")

    def _filter_pairs(self, arm_joints):
        """Keep only collision pairs involving the teleoperated arm, dropping adjacent and permanently-close ones."""
        arm_jids = {self.model.getJointId(j) for j in arm_joints if self.model.existJointName(j)}
        on_arm = lambda gi: self.geom.geometryObjects[gi].parentJoint in arm_jids
        def _hops(a, b):
            chain = []
            x = a
            while True:
                chain.append(x)
                if x == 0:
                    break
                x = self.model.parents[x]
            depth = {j: i for i, j in enumerate(chain)}
            db, x = 0, b
            while x not in depth:
                x = self.model.parents[x]
                db += 1
            return depth[x] + db
        n0 = len(self.geom.collisionPairs)
        pairs = [pin.CollisionPair(cp.first, cp.second) for cp in self.geom.collisionPairs if on_arm(cp.first) or on_arm(cp.second)]
        n_moving = len(pairs)
        def _struct_neighbor(cp):
            if not (on_arm(cp.first) and on_arm(cp.second)):
                return False
            ja = self.geom.geometryObjects[cp.first].parentJoint
            jb = self.geom.geometryObjects[cp.second].parentJoint
            if self._is_wire_box(cp.first) or self._is_wire_box(cp.second):
                return ja == jb
            return _hops(ja, jb) < SELF_MIN_HOPS
        pairs = [cp for cp in pairs if not _struct_neighbor(cp)]
        n_topo = len(pairs)
        self.geom.removeAllCollisionPairs()
        for cp in pairs:
            self.geom.addCollisionPair(cp)
        gd = self.geom.createData()
        dcol = self.model.createData()
        pin.computeDistances(self.model, dcol, self.geom, gd, pin.neutral(self.model))
        dmins = [gd.distanceResults[k].min_distance for k in range(len(self.geom.collisionPairs))]
        thresh = max(2.0 * D_MIN, DROP_DIST_THRESH)
        def _is_env(cp):
            na = self.geom.geometryObjects[cp.first].name.lower()
            nb = self.geom.geometryObjects[cp.second].name.lower()
            return any(k in na or k in nb for k in ENV_KEYWORDS)
        both_arm = lambda cp: on_arm(cp.first) and on_arm(cp.second)
        final_pairs = []
        for k, cp in enumerate(self.geom.collisionPairs):
            if (not both_arm(cp)) and (not _is_env(cp)) and dmins[k] < thresh:
                a = self.geom.geometryObjects[cp.first].name
                b = self.geom.geometryObjects[cp.second].name
                self._info(f"drop permanently-near pair {a}<->{b} (d={dmins[k]*1000:.0f}mm at reference)")
                continue
            final_pairs.append(pin.CollisionPair(cp.first, cp.second))
        self.geom.removeAllCollisionPairs()
        for cp in final_pairs:
            self.geom.addCollisionPair(cp)
        n_dist = len(final_pairs)
        n_reach = self._reachability_prune(arm_joints, on_arm, both_arm)
        self._info(f"collision pairs funnel: srdf={n0} -> moving={n_moving} -> topo={n_topo} -> dist={n_dist} -> reach={n_reach}")

    def _reachability_prune(self, arm_joints, on_arm, both_arm):
        """Drop arm-vs-static pairs whose bounding spheres can never meet over random arm configurations."""
        if PRUNE_MARGIN <= 0.0 or PRUNE_SAMPLES <= 0 or len(self.geom.collisionPairs) == 0:
            return len(self.geom.collisionPairs)
        centers, radii = [], []
        for go in self.geom.geometryObjects:
            try:
                go.geometry.computeLocalAABB()
                al = go.geometry.aabb_local
                mn = np.array(al.min_); mx = np.array(al.max_)
                centers.append(0.5 * (mn + mx))
                radii.append(0.5 * float(np.linalg.norm(mx - mn)))
            except Exception:
                centers.append(np.zeros(3))
                radii.append(1e3)
        rng = np.random.default_rng(0)
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit
        qidx = [self.model.joints[self.model.getJointId(j)].idx_q for j in arm_joints]
        d = self.model.createData()
        gd = self.geom.createData()
        q = pin.neutral(self.model)
        moving = [gi for gi in range(len(self.geom.geometryObjects)) if on_arm(gi)]
        static = [gi for gi in range(len(self.geom.geometryObjects)) if not on_arm(gi)]
        pin.updateGeometryPlacements(self.model, d, self.geom, gd, q)
        c_static = {gi: gd.oMg[gi].rotation @ centers[gi] + gd.oMg[gi].translation for gi in static}
        samples = rng.uniform([lo[i] for i in qidx], [hi[i] for i in qidx], size=(PRUNE_SAMPLES, len(qidx)))
        traces = {gi: np.empty((PRUNE_SAMPLES, 3)) for gi in moving}
        for si, s in enumerate(samples):
            for kk, i in enumerate(qidx):
                q[i] = s[kk]
            pin.forwardKinematics(self.model, d, q)
            pin.updateGeometryPlacements(self.model, d, self.geom, gd)
            for gi in moving:
                traces[gi][si] = gd.oMg[gi].rotation @ centers[gi] + gd.oMg[gi].translation
        keep = []
        for cp in self.geom.collisionPairs:
            a, b = cp.first, cp.second
            if both_arm(cp):
                keep.append(pin.CollisionPair(a, b))
                continue
            ga, gb = (a, b) if on_arm(a) else (b, a)
            cb = c_static.get(gb)
            if cb is None:
                keep.append(pin.CollisionPair(a, b))
                continue
            dmin = float(np.min(np.linalg.norm(traces[ga] - cb, axis=1))) - radii[ga] - radii[gb]
            if dmin > PRUNE_MARGIN:
                continue
            keep.append(pin.CollisionPair(a, b))
        self.geom.removeAllCollisionPairs()
        for cp in keep:
            self.geom.addCollisionPair(cp)
        return len(keep)

    def fix_limits(self, vel_scale):
        """Replace non-finite joint limits, apply the extra clamps from RLIMITS and scale the velocity limits."""
        lo = np.array(self.model.lowerPositionLimit, float)
        hi = np.array(self.model.upperPositionLimit, float)
        lo[~np.isfinite(lo)] = -np.pi
        hi[~np.isfinite(hi)] = np.pi
        for j, (jlo, jhi) in RLIMITS.items():
            if j in self._qidx:
                qi = self._qidx[j]
                lo[qi] = max(lo[qi], float(jlo))
                hi[qi] = min(hi[qi], float(jhi))
        self.model.lowerPositionLimit, self.model.upperPositionLimit = lo, hi
        if getattr(self, "_vl_raw", None) is None:
            vl = np.array(self.model.velocityLimit, float)
            vl[~np.isfinite(vl) | (vl <= 0)] = np.pi
            self._vl_raw = vl
        vl = self._vl_raw.copy()
        scale = np.full(vl.shape, float(vel_scale))
        if WRIST_VEL_SCALE > 0.0:
            for j in self.arm_joints:
                is_wrist = (j in WRIST_VEL_JOINTS) if WRIST_VEL_JOINTS is not None else ("wrist" in j)
                if is_wrist and self.model.existJointName(j):
                    scale[self.model.joints[self.model.getJointId(j)].idx_v] = WRIST_VEL_SCALE
            self._info(f"wrist_vel_scale={WRIST_VEL_SCALE} on wrist joints (others x{vel_scale})")
        self.model.velocityLimit = vl * scale

    def in_collision(self, q):
        """Return True if configuration q is in collision (False when no geometry is loaded)."""
        if self.geom is None:
            return False
        return bool(pin.computeCollisions(self.model, self.col_data, self.geom, self.geom_data, np.asarray(q, float), True))

    def qindex(self, joint_name):
        """Index of a named joint inside the configuration vector q."""
        return self._qidx[joint_name]

    def neutral(self):
        """Model neutral configuration."""
        return pin.neutral(self.model)

    @property
    def q(self):
        """Current configuration q (copy)."""
        return self.configuration.q.copy()

    def arm_positions(self):
        """Current positions of the arm joints, in ARM order."""
        q = self.configuration.q
        return np.array([q[self._qidx[j]] for j in self.arm_joints], float)

    def fk_rotation(self):
        """EE frame rotation in world for the current configuration."""
        return self.configuration.get_transform_frame_to_world(self.ee).rotation.copy()

    def fk_translation(self):
        """EE frame position in world for the current configuration."""
        return self.configuration.get_transform_frame_to_world(self.ee).translation.copy()

    def fk_grasp(self, R=None):
        """World position of the control point (EE frame offset by the TCP); R overrides the rotation used for the offset."""
        T = self.configuration.get_transform_frame_to_world(self.ee)
        return (T.translation + (T.rotation if R is None else R) @ self.tcp).copy()

    def resolve_tcp(self, cfg, keywords):
        """Control point on the tool in EE-frame coordinates: 'auto' derives it from the finger geometry, or pass [x, y, z]."""
        self.tcp = np.zeros(3)
        if cfg is None:
            return self.tcp
        if not isinstance(cfg, str):
            self.tcp = np.asarray(cfg, float).reshape(3)
            self._info(f"TCP control point: {self.tcp} (explicit)")
            return self.tcp
        if self.geom is None:
            self._warn("tcp_offset: auto needs the collision model -> TCP left at the flange")
            return self.tcp
        fid = self.model.getFrameId(self.ee)
        fk = self.model.createData()
        gd = self.geom.createData()
        q0 = pin.neutral(self.model)
        pin.forwardKinematics(self.model, fk, q0)
        pin.updateFramePlacements(self.model, fk)
        pin.updateGeometryPlacements(self.model, fk, self.geom, gd, q0)
        T0 = fk.oMf[fid]
        lo, hi = 1e9, -1e9
        for i, go in enumerate(self.geom.geometryObjects):
            if not any(k in go.name.lower() for k in keywords):
                continue
            try:
                V = np.asarray(go.geometry.points())
            except Exception:
                continue
            if V.size == 0:
                continue
            Mrel = T0.actInv(gd.oMg[i])
            z = ((Mrel.rotation @ V.T).T + Mrel.translation)[:, 2]
            lo, hi = min(lo, float(z.min())), max(hi, float(z.max()))
        if hi < lo:
            self._warn(f"tcp_offset: auto found no geometry matching {list(keywords)} -> TCP left at the flange")
            return self.tcp
        self.tcp = np.array([0.0, 0.0, 0.5 * (lo + hi)])
        self._info(f"TCP control point: {self.tcp} (auto: {keywords} span z={lo:.3f}..{hi:.3f} m from {self.ee})")
        return self.tcp

    def reset_to(self, qf):
        """Reset the configuration to qf and re-anchor the posture target there."""
        qf = np.clip(np.asarray(qf, float), self.model.lowerPositionLimit, self.model.upperPositionLimit)
        if self.barrier is not None:
            self.configuration = DistanceConfiguration(self.model, self.data, qf, collision_model=self.geom, collision_data=self.geom_data)
        else:
            self.configuration = Configuration(self.model, self.data, qf)
        self.posture.set_target(self.configuration.q)

    def set_arm(self, values):
        """Overwrite the arm joint positions in the internal configuration, to re-sync IK to the measured robot."""
        q = self.configuration.q.copy()
        for j, v in zip(self.arm_joints, values):
            q[self._qidx[j]] = float(v)
        q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)
        self.configuration.update(q)

    def _floors(self):
        """Per-pair floor vector (d_min_self for self pairs, d_min for the rest)."""
        n = len(self.geom.collisionPairs)
        if self.pair_dmin is not None and len(self.pair_dmin) >= n:
            return self.pair_dmin[:n]
        return np.full(n, D_MIN)

    def min_gap(self):
        """(distance, margin, pair name) of the lowest-margin collision pair at the current configuration."""
        if self.geom is None or len(self.geom.collisionPairs) == 0:
            return float("inf"), float("inf"), ""
        cd = getattr(self.configuration, "collision_data", None)
        if cd is None:
            pin.computeDistances(self.model, self.col_data, self.geom, self.geom_data, self.configuration.q)
            cd = self.geom_data
        dists = np.array([cd.distanceResults[k].min_distance for k in range(len(self.geom.collisionPairs))])
        margins = dists - self._floors()
        k = int(np.argmin(margins))
        cp = self.geom.collisionPairs[k]
        name = self.geom.geometryObjects[cp.first].name + "<->" + self.geom.geometryObjects[cp.second].name
        return float(dists[k]), float(margins[k]), name

    def margin_at(self, q, floor_scale=1.0):
        """(min margin, pair name) at an arbitrary configuration q, computed on buffers separate from the barrier's."""
        if self.geom is None or len(self.geom.collisionPairs) == 0:
            return float("inf"), ""
        if self._gate_gdata is None:
            self._gate_data = self.model.createData()
            self._gate_gdata = self.geom.createData()
        pin.computeDistances(self.model, self._gate_data, self.geom, self._gate_gdata, np.asarray(q, float))
        n = len(self.geom.collisionPairs)
        dists = np.array([self._gate_gdata.distanceResults[k].min_distance for k in range(n)])
        m = dists - floor_scale * self._floors()
        k = int(np.argmin(m))
        cp = self.geom.collisionPairs[k]
        name = self.geom.geometryObjects[cp.first].name + "<->" + self.geom.geometryObjects[cp.second].name
        return float(m[k]), name

    def _joint_axis_local(self, jname):
        """Rotation axis of a revolute joint in its own frame, measured numerically."""
        jid = self.model.getJointId(jname)
        d = self.model.createData()
        q = pin.neutral(self.model)
        pin.forwardKinematics(self.model, d, q)
        R0 = d.oMi[jid].rotation.copy()
        q[self._qidx[jname]] += 0.5
        pin.forwardKinematics(self.model, d, q)
        w = pin.log3(R0.T @ d.oMi[jid].rotation)
        n = float(np.linalg.norm(w))
        return w / n if n > 1e-9 else np.array([0.0, 0.0, 1.0])

    def roll_axis(self):
        """World-frame rotation axis of the tool-roll joint at the current commanded configuration."""
        jid = self.model.getJointId(self._roll_joint)
        return self.configuration.data.oMi[jid].rotation @ self._a3_local

    def _roll_pairs(self):
        """Indices of the self-collision pairs that a pure tool roll can drive toward contact."""
        if self._roll_pairs_idx is None:
            jid = self.model.getJointId(self._roll_joint)
            arm_jids = {self.model.getJointId(j) for j in self.arm_joints if self.model.existJointName(j)}
            gos = self.geom.geometryObjects
            self._roll_pairs_idx = [
                k for k, cp in enumerate(self.geom.collisionPairs)
                if ((gos[cp.first].parentJoint == jid) != (gos[cp.second].parentJoint == jid))
                and gos[cp.first].parentJoint in arm_jids
                and gos[cp.second].parentJoint in arm_jids]
        return self._roll_pairs_idx

    def _pair_name(self, k):
        cp = self.geom.collisionPairs[k]
        return self.geom.geometryObjects[cp.first].name + "<->" + self.geom.geometryObjects[cp.second].name

    def roll_conflict(self, direction):
        """True if the tool-roll joint cannot turn in `direction`: a tool-cluster pair closes, or the joint is at its limit."""
        self.roll_conflict_info = ""
        qi = self._qidx[self._roll_joint]
        q = self.configuration.q
        if direction > 0.0:
            if float(self.model.upperPositionLimit[qi] - q[qi]) < ROLL_GATE_LIMIT:
                self.roll_conflict_info = "w3 at +limit"
                return True
        elif float(q[qi] - self.model.lowerPositionLimit[qi]) < ROLL_GATE_LIMIT:
            self.roll_conflict_info = "w3 at -limit"
            return True
        if self.geom is None or len(self.geom.collisionPairs) == 0:
            return False
        idx = self._roll_pairs()
        if not idx:
            return False
        floors = self._floors()
        cd = getattr(self.configuration, "collision_data", None)
        near = idx
        if cd is not None:
            near = [k for k in idx if cd.distanceResults[k].min_distance - floors[k] < ROLL_GATE_MARGIN]
            if not near:
                return False
        if self._gate_gdata is None:
            self._gate_data = self.model.createData()
            self._gate_gdata = self.geom.createData()
        pin.computeDistances(self.model, self._gate_data, self.geom, self._gate_gdata, q)
        d0 = {k: self._gate_gdata.distanceResults[k].min_distance for k in near}
        near = [k for k in near if d0[k] - floors[k] < ROLL_GATE_MARGIN]
        if not near:
            return False
        q2 = q.copy()
        q2[qi] += float(direction) * ROLL_GATE_PROBE
        pin.computeDistances(self.model, self._gate_data, self.geom, self._gate_gdata, q2)
        thresh = ROLL_GATE_PROBE * ROLL_GATE_SLOPE
        for k in near:
            dd = self._gate_gdata.distanceResults[k].min_distance - d0[k]
            if -dd > thresh:
                self.roll_conflict_info = (f"{'riding' if d0[k] - floors[k] < ROLL_GATE_RIDE else 'closing'} "
                                           f"{self._pair_name(k)}")
                return True
        return False

    def _min_margin_at(self, q):
        pin.computeDistances(self.model, self.col_data, self.geom, self.geom_data, np.asarray(q, float))
        n = len(self.geom.collisionPairs)
        if n == 0:
            return 1e9
        dists = np.array([self.geom_data.distanceResults[k].min_distance for k in range(n)])
        return float(np.min(dists - self._floors()))

    def _sing_damping(self):
        """Tikhonov weight for this tick's QP, ramped in near a wrist singularity; sing_sigma0 = 0 disables it."""
        J = self.configuration.get_frame_jacobian(self.ee)
        s = float(np.linalg.svd(J, compute_uv=False)[-1])
        self.sigma_min = s
        if SING_SIGMA0 <= 0.0 or SING_LAMBDA <= 0.0 or s >= SING_SIGMA0:
            return QP_DAMPING
        u = np.linalg.svd(J)[0][:, -1]
        self.sing_kind = "shoulder/elbow" if float(u[:3] @ u[:3]) > 0.5 else "wrist"
        r = s / SING_SIGMA0
        return QP_DAMPING + (SING_LAMBDA * SING_LAMBDA) * (1.0 - r * r)

    def step(self, target_pos, target_R, dt=DT):
        """One diff-IK step toward (target_pos, target_R); retreats along the barrier gradient when the QP is infeasible."""
        T = pin.SE3(np.asarray(target_R, float), np.asarray(target_pos, float))
        self.ee_task.set_target(T)
        q_prec = self.configuration.q.copy()
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit
        mu = self._sing_damping()

        if self.barrier is None:
            q_new = q_prec
            try:
                v = solve_ik(self.configuration, [self.ee_task, self.posture], dt, solver=self.solver, limits=self.limits, damping=mu, safety_break=False)
                q_new = pin.integrate(self.model, q_prec, v * dt)
            except Exception as exc:
                self._warn(f"IK solve skipped: {exc}", throttle=2.0)
            q_new = np.clip(q_new, lo, hi)
            if not np.isfinite(q_new).all():
                q_new = q_prec
            self.blocked = self.in_collision(q_new)
            if self.blocked:
                q_new = q_prec
            if not np.array_equal(q_new, self.configuration.q):
                self.configuration.update(q_new)
            return self.arm_positions()

        v, reason = None, None
        try:
            v = solve_ik(self.configuration, [self.ee_task, self.posture], dt, solver=self.solver, limits=self.limits, barriers=[self.barrier], damping=mu, safety_break=False)
        except PinkError as exc:
            reason = f"PinkError: {exc}"
        except Exception as exc:
            reason = f"{type(exc).__name__}: {exc}"
        if v is None and reason is None:
            reason = "solve_ik returned None"
        q_new = None
        if v is not None:
            q_new = np.clip(pin.integrate(self.model, q_prec, v * dt), lo, hi)
            if not np.isfinite(q_new).all():
                reason = "non-finite q from IK"
                q_new = None
        self.active_pairs = []
        if v is not None and q_new is not None:
            try:
                self.active_pairs = self.barrier.active_rows(self.configuration, v)
            except Exception:
                pass
        if q_new is None:
            self.retreats += 1
            self._warn(f"IK(barrier) infeasible -> retreat ({reason})", throttle=1.0)
            q_new = self._retreat(q_prec, dt)
        if not np.array_equal(q_new, self.configuration.q):
            self.configuration.update(q_new)
        self.blocked = bool(self.active_pairs) or v is None
        return self.arm_positions()

    def _retreat(self, q_prec, dt):
        """Move away from the closest collision pair along the outward barrier gradient, holding at q_prec if that fails."""
        lo, hi = self.model.lowerPositionLimit, self.model.upperPositionLimit
        try:
            h = self.barrier.compute_barrier(self.configuration)
            J = self.barrier.compute_jacobian(self.configuration)
            g = J[int(np.argmin(h))]
            gn = float(np.linalg.norm(g))
            if gn < 1e-9:
                self._warn("retreat gradient ~0 -> holding", throttle=1.0)
                return q_prec
            g = g / gn
            g0 = self._min_margin_at(q_prec)
            q_probe = np.clip(pin.integrate(self.model, q_prec, g * 1e-3), lo, hi)
            if self._min_margin_at(q_probe) < g0:
                g = -g
            k_ret = min(RETREAT_RATE / max(gn, 1e-9), RETREAT_VEL_FRAC * float(np.max(self.model.velocityLimit)))
            v_ret = np.clip(k_ret * g, -self.model.velocityLimit, self.model.velocityLimit)
            q_ret = np.clip(pin.integrate(self.model, q_prec, v_ret * dt), lo, hi)
            if not np.isfinite(q_ret).all():
                self._min_margin_at(q_prec)
                self._warn("retreat produced non-finite q -> holding", throttle=1.0)
                return q_prec
            if self._min_margin_at(q_ret) < g0:
                self._min_margin_at(q_prec)
                self._warn("retreat worsened gap -> holding", throttle=1.0)
                return q_prec
            return q_ret
        except Exception as exc:
            try:
                self._min_margin_at(q_prec)
            except Exception:
                pass
            self._warn(f"retreat failed ({exc}) -> holding", throttle=1.0)
            return q_prec


class Bridge(Node):
    """ROS2 node: VR controller pose (/vive topics) -> Pink diff-IK -> mantis left arm."""

    def __init__(self, parked=None):
        """Build IK at the measured parked pose, compute the reach shell, set up publishers, subscribers and timers."""
        super().__init__("vive_mantis_pink_bridge")
        locked_q = dict(LOCKED_Q)
        self._parked_measured = bool(parked) and all(j in parked for j in ARM_CMD_JOINTS)
        if parked:
            locked_q.update({n: v for n, v in parked.items() if n not in ARM})
            if self._parked_measured:
                self.get_logger().info(f"collision model built at the MEASURED parked pose ({len(parked)} joints)")
            else:
                self.get_logger().warn(
                    f"/joint_states snapshot INCOMPLETE ({len(parked)} joints, missing "
                    f"{sorted(j for j in ARM_CMD_JOINTS if j not in parked)}) -> collision model built at "
                    f"the measured pose where known; it will be rebuilt once the real pose is known")
        else:
            self.get_logger().warn("no /joint_states snapshot at startup -> collision model built at "
                                   "config locked_q; it will be rebuilt once the real pose is known")
        self.ik = PinkIK(URDF, EE_FRAME, ARM, srdf_path=srdf_path(), package_dirs=mesh_pkg_dirs(), locked_q=locked_q, logger=self.get_logger())
        self.model = self.ik.model
        self.shoulder = self.shoulder_origin()
        mn, mx = self.reach_shell()
        self.r_min = mn + REACH_LO_FRAC * (mx - mn)
        self.r_max = mn + REACH_HI_FRAC * (mx - mn)
        self.get_logger().info(f"reach shell: r_min={self.r_min:.3f} r_max={self.r_max:.3f} (raw {mn:.3f}..{mx:.3f})")
        self.phase = "wait"
        self.pub = self.create_publisher(Float64MultiArray, ARM_CMD_TOPIC, 10)
        self.shaper = None
        if OUT_RATE > 0:
            self.shaper = CommandShaper(self.pub, OUT_RATE, OUT_VEL, OUT_ACCEL, OUT_KP,
                                        OUT_FF_TAU if OUT_FF else 0.0)
            self.shaper.start()
            self.get_logger().info(f"CommandShaper: {OUT_RATE:.0f} Hz, v_max={OUT_VEL}, a_max={OUT_ACCEL}, kp={OUT_KP}")
        qos_latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        qos_js = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, JOINT_STATES_TOPIC, self.on_joint_states, qos_js)
        self.create_subscription(Float64MultiArray, VIVE_POSE_TOPIC, self.on_vive_pose, qos_latest)
        self.create_subscription(Float64MultiArray, VIVE_BUTTONS_TOPIC, self.on_vive_buttons, qos_latest)
        self.create_timer(DT, self.tick)
        self.create_timer(VR_DT, self.vr_tick)
        self.create_timer(GRIP_DT, self.grip_tick)
        self.grip = ActionClient(self, GripperCommand, GRIP_ACTION)
        self._grip_want_closed = False
        self._click_now = False
        self._click_was = False
        self._grip_toggle_t = -1e9
        self._grip_sent = None
        self._grip_send_t = -1e9
        self._grip_last_w = None
        self.Rc_ref = None
        self.R_anchor = None
        self._blk = 0
        self.pos = 0
        self.eff = 0
        self.dbg = 0
        self.shared = {"target": None, "home": None, "anchor": None, "ref": None, "engaged": False, "ready": False, "Rc": np.eye(3)}
        self._cur = None
        self._pose_t = 0.0
        self._pose_msg_t = 0.0
        self._pose_lost = False
        self._pose_dts = []
        self._pose_drops = 0
        self._pose_worst = 0.0
        self._med_buf = []
        self._raw_p = None
        self._raw_R = None
        self._pose_vlin = []
        self._pose_vang = []
        self._pose_vlin_ok = []
        self._pose_vang_ok = []
        self._pose_glitches = 0
        self._pose_glitch_worst = 0.0
        self._glitch_t = []
        self._track_bad_until = 0.0
        self._degrade_holds = 0
        self._piv_p = None; self._piv_R = None
        self._piv_M = np.zeros((3, 3)); self._piv_b = np.zeros(3)
        self._piv_n = 0; self._piv_dp2 = 0.0
        self._js_t = 0.0
        self._btn_t = 0.0
        self._parked_cmd = {}
        self._ee_prev = None
        self._ee_dt = 0.0
        self._antiwind_prog = 0.0
        self._R_ref = np.eye(3)
        self._R_des_prev = None
        self._R_tgt_prev = None
        self._w_err = np.zeros(3)
        self._roll_disc = 0.0
        self._roll_conf_run = 0
        self._roll_dropped = 0.0
        self._ori_acc = np.zeros(3)
        self._ori_acc_n = 0
        self._ori_acc_R = None
        self._gate_t = 0.0
        self._debug_t = 0.0
        self._R_tcp = None
        self._shk = []
        self._shk_t = []
        self._shk_i = 0
        self._ori_what = np.zeros(3)
        self._vr_t_last = None
        self._tick_t_last = None
        self._pad_was = False
        self._pad_t = 0.0
        self._pad_now = False
        self._menu_was = False
        self._menu_now = False
        self._menu_t = 0.0
        self._trig_now = 0.0
        self._home_now = False
        self._home_was = False
        self._home_t = -1e9
        self._home_active = False
        self._home_v = HOME_VEL
        self._home_cur = None
        self._home_done_t = None
        self._home_stop_rec = False
        self._axlock_now = False
        self._axlock_was = False
        self._axlock_t = -1e9
        self._axlock_active = False
        self._axlock_p0 = None
        self._axlock_u = None
        self._axlock_R = None
        if AXIS_LOCK and AXIS_LOCK_MODE not in ("hold", "toggle"):
            self.get_logger().warn(f"axis_lock_mode {AXIS_LOCK_MODE!r} unknown -> treated as 'hold'")
        self._ori_aw_prev = None
        self._ori_aw_t = 0.0
        self._ori_aw_sum = 0.0
        self.mark = False
        self._blend_from = None
        self._blend_n = 0
        self._joints_lost = False
        self._last_cmd = None
        self._qarm_prev = None
        self._qarm_prev_t = 0.0
        self._clamp_overshoot = False
        self._cmd_hist = []
        self._lag_t0 = None
        self._pos_filter = OneEuroFilter(FILTER_MIN_CUTOFF, FILTER_BETA)
        self._steptimes = []
        self._steptime_last = time.monotonic()
        self._retreats_last = 0
        self.recorder = None
        if bool(RECORD_CFG.get("enabled", True)):
            try:
                from lerobot_recorder import EpisodeRecorder
                self.recorder = EpisodeRecorder(self, RECORD_CFG)
            except Exception as exc:
                self.get_logger().error(
                    f"episode recorder DISABLED ({type(exc).__name__}: {exc}) "
                    f"-> MENU falls back to the console RECORD marker")
        self.get_logger().info("Bridge up. Waiting for robot...")

    def shoulder_origin(self):
        """World position of the shoulder-lift joint at neutral, used as the teleop workspace center."""
        q0 = self.ik.neutral()
        fk_data = self.model.createData()
        pin.forwardKinematics(self.model, fk_data, q0)
        pin.updateFramePlacements(self.model, fk_data)
        jid = self.model.getJointId(SHOULDER_JOINT)
        return fk_data.oMi[jid].translation.copy()

    def reach_shell(self, n=60000, seed=0):
        """Monte-Carlo estimate of the min/max EE distance from the shoulder, i.e. the reach envelope."""
        rng = np.random.default_rng(seed)
        lo_lim = self.model.lowerPositionLimit
        hi_lim = self.model.upperPositionLimit
        qidx = [self.ik.qindex(j) for j in REACH_SHELL_JOINTS]
        q = self.ik.neutral()
        lo_d, hi_d = 1e9, 0.0
        samples = rng.uniform([lo_lim[i] for i in qidx], [hi_lim[i] for i in qidx], size=(n, len(qidx)))
        fk_data = self.model.createData()
        fid = self.model.getFrameId(EE_FRAME)
        for s in samples:
            for k, i in enumerate(qidx):
                q[i] = s[k]
            pin.forwardKinematics(self.model, fk_data, q)
            pin.updateFramePlacements(self.model, fk_data)
            d = np.linalg.norm(fk_data.oMf[fid].translation - self.shoulder)
            lo_d = min(lo_d, d)
            hi_d = max(hi_d, d)
        t = float(np.linalg.norm(self.ik.tcp))
        return max(0.0, lo_d - t), hi_d + t

    def on_vive_pose(self, msg):
        """Cache the controller pose: one-euro-filtered position + orthonormalized rotation."""
        d = msg.data
        if len(d) < 12:
            return
        arr = np.asarray(d[:12], float)
        if not np.isfinite(arr).all():
            return
        self._pose_msg_t = time.monotonic()
        Rc = arr[3:].reshape(3, 3)
        U, _, Vt = np.linalg.svd(Rc)
        Rc = U @ Vt
        if np.linalg.det(Rc) < 0.0:
            Rc[:, -1] *= -1.0
        now = time.monotonic()
        if self._cur is not None:
            gap = now - self._pose_t
            self._pose_dts.append(gap * 1000.0)
            if gap >= POSE_TIMEOUT:
                self._pose_drops += 1
                self._pose_worst = max(self._pose_worst, gap)
                self._pos_filter.t_prev = None
                self._med_buf = []
            elif gap > 1e-6 and self._raw_p is not None:
                v_lin = float(np.linalg.norm(arr[:3] - self._raw_p)) / gap
                v_ang = float(np.linalg.norm(pin.log3(Rc @ self._raw_R.T))) / gap
                self._pose_vlin.append(v_lin)
                self._pose_vang.append(v_ang)
                if ((POSE_JUMP_LIN > 0.0 and v_lin > POSE_JUMP_LIN)
                        or (POSE_JUMP_ANG > 0.0 and v_ang > POSE_JUMP_ANG)):
                    self._pose_glitches += 1
                    self._pose_glitch_worst = max(self._pose_glitch_worst,
                                                  max(v_lin / max(POSE_JUMP_LIN, 1e-9),
                                                      v_ang / max(POSE_JUMP_ANG, 1e-9)))
                    if DEGRADE_MAX_RATE > 0.0:
                        self._glitch_t.append(now)
                        while self._glitch_t and (now - self._glitch_t[0]) > DEGRADE_WINDOW:
                            self._glitch_t.pop(0)
                        if len(self._glitch_t) >= DEGRADE_MAX_RATE * DEGRADE_WINDOW:
                            self._track_bad_until = now + DEGRADE_HOLD
                    return
                self._pose_vlin_ok.append(v_lin)
                self._pose_vang_ok.append(v_ang)
        if self._piv_R is None:
            self._piv_p, self._piv_R = arr[:3].copy(), Rc.copy()
        else:
            dth = float(np.linalg.norm(pin.log3(Rc @ self._piv_R.T)))
            if dth >= PIVOT_MIN_ROT:
                A = Rc - self._piv_R
                dp = arr[:3] - self._piv_p
                self._piv_M += A.T @ A
                self._piv_b += -A.T @ dp
                self._piv_n += 1
                self._piv_dp2 += float(dp @ dp)
                self._piv_p, self._piv_R = arr[:3].copy(), Rc.copy()
        self._raw_p = arr[:3].copy()
        self._raw_R = Rc.copy()
        p_in = arr[:3]
        if MEDIAN_N >= 3:
            self._med_buf.append(p_in.copy())
            if len(self._med_buf) > MEDIAN_N:
                self._med_buf.pop(0)
            if len(self._med_buf) == MEDIAN_N:
                p_in = np.median(np.array(self._med_buf), axis=0)
        self._cur = self._pos_filter(p_in, now)
        self._pose_t = now
        if ORI_ANTIALIAS:
            if self._ori_acc_R is None:
                self._ori_acc_R = Rc.copy()
                self._ori_acc = np.zeros(3)
                self._ori_acc_n = 0
            self._ori_acc = self._ori_acc + pin.log3(Rc @ self._ori_acc_R.T)
            self._ori_acc_n += 1
        self.shared["Rc"] = Rc

    def _consume_Rc(self):
        """Mean controller rotation since the previous tick (see on_vive_pose)."""
        last = self.shared["Rc"]
        if not ORI_ANTIALIAS or self._ori_acc_n == 0 or self._ori_acc_R is None:
            return last.copy()
        R = pin.exp3(self._ori_acc / self._ori_acc_n) @ self._ori_acc_R
        self._ori_acc_R = last.copy()
        self._ori_acc = np.zeros(3)
        self._ori_acc_n = 0
        return R

    def on_vive_buttons(self, msg):
        d = msg.data
        if len(d) < 3:
            return
        self._btn_t = time.monotonic()
        self._trig_now = float(d[0])
        self._pad_now = d[1] > 0.5
        self._menu_now = d[2] > 0.5
        self._click_now = (d[3] > 0.5) if len(d) >= 4 else (self._trig_now >= CLICK_THRESH)
        self._home_now = (d[4] > 0.5) if len(d) >= 5 else False
        self._axlock_now = (d[5] > 0.5) if len(d) >= 6 else False

    def yaw_frame(self, Rc):
        """Heading-only frame from the controller orientation, so the position mapping ignores its pitch and roll."""
        Rc = np.asarray(Rc)
        up = np.array([0.0, 1.0, 0.0])
        nose = -Rc[:, 2]
        fwd = nose
        if np.linalg.norm(fwd - (fwd @ up) * up) < YAW_FALLBACK_SIN:
            fwd = -np.sign(nose @ up) * Rc[:, 1]
            self.get_logger().warn("yaw_frame: nose near-vertical -> heading taken from controller top axis")
        back_h = (fwd @ up) * up - fwd
        n = np.linalg.norm(back_h)
        if n < 1e-6:
            return np.eye(3)
        back_h = back_h / n
        right = np.cross(up, back_h)
        right = right / np.linalg.norm(right)
        return np.column_stack([right, up, back_h])

    def _capture_refs(self):
        """Anchor the position and orientation refs to the current controller pose and EE, atomically."""
        self.shared["ref"] = self._cur.copy()
        self._R_ref = self.yaw_frame(self.shared["Rc"])
        self.shared["anchor"] = self.shared["target"].copy()
        self.Rc_ref = self.shared["Rc"].copy()
        self.R_anchor = self.ik.fk_rotation()
        self._R_des_prev = self.R_anchor.copy()
        self._R_tgt_prev = self.R_anchor.copy()
        self._R_tcp = self.R_anchor.copy()
        self._w_err = np.zeros(3)
        self._roll_disc = 0.0
        self._roll_conf_run = 0
        self._roll_dropped = 0.0
        self._ee_prev = None
        self._ori_aw_prev = None
        self._cmd_hist = []
        self._shk = []
        self._qarm_prev = None
        self._shk_t = []

    def _joints_fresh(self):
        return isinstance(self.pos, dict) and (time.monotonic() - self._js_t) < JOINT_TIMEOUT and all(j in self.pos for j in ARM_CMD_JOINTS)

    def _on_engage(self):
        self.shared["engaged"] = True
        self._pose_lost = False
        self._blend_n = 0
        self._lag_t0 = None
        self._capture_refs()
        self.get_logger().info("ENGAGED")

    def _on_disengage(self):
        """Freeze: re-sync IK to the measured robot, park the target on that EE pose and blend the command over a few ticks."""
        self._axlock_clear("freeze")
        self.shared["engaged"] = False
        if self._joints_fresh():
            self._blend_from = self.ik.arm_positions().copy()
            self._blend_n = BLEND_TICKS
            q = self.ik.neutral()
            for j in ARM:
                q[self.ik.qindex(j)] = self.pos[j]
            self.ik.reset_to(q)
        self.shared["target"] = self.ik.fk_grasp()
        self._R_des_prev = None
        self._cmd_hist = []
        self.get_logger().info("FROZEN")

    def _axlock_engage(self):
        """Axis lock on: freeze the orientation target and constrain translation to the line the gripper points along."""
        R0 = self._R_des_prev if self._R_des_prev is not None else self.ik.fk_rotation()
        if AXIS_LOCK_AXIS is not None:
            u = R0 @ np.asarray(AXIS_LOCK_AXIS, float).reshape(3)
        elif float(np.linalg.norm(self.ik.tcp)) > 1e-6:
            u = R0 @ self.ik.tcp
        else:
            u = R0[:, 2].copy()
        n = float(np.linalg.norm(u))
        if n < 1e-9:
            self.get_logger().warn("AXIS LOCK refused: lock axis is zero (check axis_lock_axis)")
            return
        self._axlock_u = u / n
        self._axlock_p0 = self.shared["target"].copy()
        self._axlock_R = R0.copy()
        self._w_err = np.zeros(3)
        self._R_tgt_prev = R0.copy()
        self._axlock_active = True
        self.get_logger().info(
            "AXIS LOCK on: translation along [%+.2f %+.2f %+.2f] (world), rotation frozen"
            % tuple(self._axlock_u))

    def _axlock_clear(self, reason):
        """Drop the lock state (no re-anchor — the caller decides the handoff)."""
        if not self._axlock_active:
            return False
        self._axlock_active = False
        self._axlock_p0 = self._axlock_u = self._axlock_R = None
        self.get_logger().info(f"AXIS LOCK off ({reason})")
        return True

    def _axlock_release(self, reason, fresh):
        """Axis lock off: re-anchor the hand-to-tool registration so motion discarded while locked never replays."""
        if not self._axlock_clear(reason):
            return
        if self.shared["engaged"] and fresh:
            self._capture_refs()

    def _cap_werr(self):
        """Bound the orientation winding accumulator, so a blocked arm cannot replay a long backlog of twist."""
        n = float(np.linalg.norm(self._w_err))
        if n > W_ERR_CAP:
            self._w_err = self._w_err * (W_ERR_CAP / n)

    def _home_path_clear(self, q_from):
        """Check the straight joint segment to HOME_Q against the barrier floors; returns (ok, worst margin, pair)."""
        if self.ik.geom is None:
            return True, float("inf"), ""
        delta = HOME_Q - q_from
        n = int(np.ceil(float(np.max(np.abs(delta))) / max(HOME_CHECK_STEP, 1e-3))) + 1
        worst, worst_pair = float("inf"), ""
        for s in np.linspace(0.0, 1.0, min(max(n, 2), 400)):
            q = self.ik.neutral()
            for j, v in zip(ARM, q_from + s * delta):
                q[self.ik.qindex(j)] = float(v)
            m, pair = self.ik.margin_at(q)
            if m < worst:
                worst, worst_pair = m, pair
            if m < 0.0:
                return False, worst, worst_pair
        return True, worst, worst_pair

    def _start_home(self, stop_recording_on_end=False):
        """Start the slow joint ramp to HOME_Q, or refuse it with a reason; returns True if the ramp started."""
        if self._home_active:
            return False
        if self.phase != "teleop" or not self._joints_fresh():
            self.get_logger().warn("HOME ignored: robot not ready (joint_states stale or startup)")
            return False
        if not stop_recording_on_end and self.recorder is not None and self.recorder.recording:
            self.get_logger().warn("HOME ignored: episode recording — stop it first (MENU)")
            return False
        if self.shared["engaged"]:
            self._on_disengage()
            self._blend_from = None
            self._blend_n = 0
        q_now = np.array([self.pos[j] for j in ARM], float)
        for j, v in zip(ARM, HOME_Q):
            qi = self.ik.qindex(j)
            if not (self.model.lowerPositionLimit[qi] - 1e-6 <= v
                    <= self.model.upperPositionLimit[qi] + 1e-6):
                self.get_logger().error(f"HOME refused: home_q_deg puts {j} at "
                                        f"{v:+.3f} rad, outside its limits")
                return False
        ok, m, pair = self._home_path_clear(q_now)
        if not ok:
            self.get_logger().error(
                f"HOME refused: straight joint path collides ({pair} margin "
                f"{1000.0 * m:.1f}mm) — move the arm clear by hand/teleop first")
            return False
        start = q_now
        if self._last_cmd is not None and len(self._last_cmd) == len(q_now) \
                and float(np.max(np.abs(self._last_cmd - q_now))) <= MAX_JOINT_LEAD:
            start = np.asarray(self._last_cmd, float)
        self._home_v = HOME_VEL
        if self.shaper is not None:
            self._home_v = min(HOME_VEL, 0.9 * OUT_VEL, 0.5 * MAX_JOINT_LEAD * OUT_KP)
            if self._home_v < HOME_VEL:
                self.get_logger().warn(
                    f"home_vel {HOME_VEL:.2f} exceeds what the shaper can track -> "
                    f"clamped to {self._home_v:.2f} rad/s")
        self._home_cur = start.copy()
        self._home_done_t = None
        self._home_stop_rec = bool(stop_recording_on_end)
        self._home_active = True
        if self._grip_want_closed:
            self._grip_want_closed = False
            self.get_logger().info("HOME: gripper -> OPEN")
        t_est = float(np.max(np.abs(HOME_Q - start))) / max(self._home_v, 1e-6)
        self.get_logger().info(
            f"HOME: moving to home pose (~{t_est:.1f}s at {self._home_v:.2f} rad/s, "
            f"path clear, worst margin {1000.0 * m:.1f}mm)")
        return True

    def _end_home(self, reason, done):
        """Stop the home ramp, re-sync IK to the measured robot and park the target on that EE pose."""
        self._home_active = False
        self._home_cur = None
        self._home_done_t = None
        if self._home_stop_rec:
            self._home_stop_rec = False
            if self.recorder is not None:
                self.recorder.stop()
        self._blend_from = None
        self._blend_n = 0
        if self._joints_fresh():
            q = self.ik.neutral()
            for j in ARM:
                q[self.ik.qindex(j)] = self.pos[j]
            self.ik.reset_to(q)
        self.shared["target"] = self.ik.fk_grasp()
        self.shared["anchor"] = self.shared["target"].copy()
        self._R_des_prev = None
        self._R_tcp = None
        self._cmd_hist = []
        self._qarm_prev = None
        self._ee_prev = None
        if done:
            self.get_logger().info(f"HOME: {reason}")
        else:
            self.get_logger().warn(f"HOME: {reason}")

    def _home_tick(self, dt, now):
        """One step of the home ramp: synchronized linear interpolation with the measured-pose safety gate kept live."""
        if self.ik.geom is not None and MEAS_GATE_FRAC > 0.0 and (now - self._gate_t) >= 0.01:
            self._gate_t = now
            q_meas = self.ik.neutral()
            for j in ARM:
                q_meas[self.ik.qindex(j)] = self.pos[j]
            m_meas, m_pair = self.ik.margin_at(q_meas, MEAS_GATE_FRAC)
            if m_meas < 0.0:
                self._end_home(f"ABORTED — measured pose inside safety gate "
                               f"({m_pair} {1000.0 * m_meas:.1f}mm)", done=False)
                return
        delta = HOME_Q - self._home_cur
        dmax = float(np.max(np.abs(delta)))
        step = self._home_v * dt
        if dmax > step:
            self._home_cur = self._home_cur + delta * (step / dmax)
            self._last_cmd = self._home_cur.copy()
            self.send_arm(self._home_cur)
            return
        self._home_cur = HOME_Q.copy()
        self._last_cmd = self._home_cur.copy()
        self.send_arm(self._home_cur)
        if self._home_done_t is None:
            self._home_done_t = now
        q_meas = np.array([self.pos[j] for j in ARM], float)
        lag = float(np.max(np.abs(q_meas - HOME_Q)))
        if lag < HOME_DONE_TOL:
            self._end_home("reached", done=True)
        elif (now - self._home_done_t) > HOME_SETTLE_GRACE:
            self._end_home(f"command at home but the robot settled {lag:.3f} rad "
                           f"away after {HOME_SETTLE_GRACE:.0f}s — check the arm", done=False)

    def _roll_gate(self, R_t):
        """Drop the tool roll the roll joint cannot execute, so the arm does not reconfigure mid-pour; see _roll_unwind."""
        R_t = R_t @ pin.exp3(np.array([0.0, 0.0, -self._roll_disc]))
        if self._R_tgt_prev is None:
            return R_t
        a3 = self.ik.roll_axis()
        pending = float((self._w_err + pin.log3(R_t @ self._R_tgt_prev.T)) @ a3)
        excess = pending - float(np.clip(pending, -ROLL_GATE_ALLOW, ROLL_GATE_ALLOW))
        if excess == 0.0 or not self.ik.roll_conflict(np.sign(excess)):
            self._roll_conf_run = 0
            return self._roll_unwind(R_t, a3)
        self._roll_conf_run += 1
        if self._roll_conf_run < ROLL_GATE_HOLD:
            return R_t
        s = float(R_t[:, 2] @ a3)
        if abs(s) < 0.5:
            return R_t
        excess = float(np.clip(excess, -MAX_ANG_STEP, MAX_ANG_STEP))
        step = excess if s > 0.0 else -excess
        self._roll_disc += step
        self._roll_dropped += abs(excess)
        self.get_logger().info(
            f"roll gate: conflicted tool roll dropped ({self.ik.roll_conflict_info}; "
            f"net {np.degrees(self._roll_disc):+.0f}deg owed, "
            f"{self._roll_dropped:.2f} rad dropped this engage)",
            throttle_duration_sec=2.0)
        return R_t @ pin.exp3(np.array([0.0, 0.0, -step]))

    def _roll_unwind(self, R_t, a3):
        """Let a counter-roll of the hand pay off the roll discarded by the gate, keeping the hand-to-tool map registered."""
        if self._roll_disc == 0.0 or self._R_tgt_prev is None:
            return R_t
        s = float(R_t[:, 2] @ a3)
        if abs(s) < 0.5:
            return R_t
        inc = float(pin.log3(R_t @ self._R_tgt_prev.T) @ a3) * (1.0 if s > 0.0 else -1.0)
        if inc * self._roll_disc >= 0.0:
            return R_t
        d = float(np.clip(inc, -abs(self._roll_disc), abs(self._roll_disc)))
        self._roll_disc += d
        if self._roll_disc == 0.0:
            self.get_logger().info("roll gate: discard paid off by the hand -> tool roll re-registered")
        return R_t @ pin.exp3(np.array([0.0, 0.0, -d]))

    def _ori_step(self, R_target, max_step=MAX_ANG_STEP):
        """Winding-aware orientation servo: accumulated error vector, one-euro smoothing and a slew cap."""
        R_target = np.asarray(R_target, float)
        if self._R_des_prev is None:
            self._R_des_prev = R_target.copy()
            self._R_tgt_prev = R_target.copy()
            self._w_err = np.zeros(3)
            self._ori_what = np.zeros(3)
            return self._R_des_prev
        if self._R_tgt_prev is None:
            self._R_tgt_prev = self._R_des_prev.copy()
        self._w_err = self._w_err + pin.log3(R_target @ self._R_tgt_prev.T)
        self._R_tgt_prev = R_target.copy()
        self._cap_werr()
        if float(np.linalg.norm(self._w_err)) < 0.5 * np.pi:
            self._w_err = pin.log3(R_target @ self._R_des_prev.T)
        if ORI_MIN_CUTOFF > 0.0:
            cutoff = ORI_MIN_CUTOFF
            if ORI_BETA > 0.0:
                a_d = OneEuroFilter._alpha(ORI_D_CUTOFF, DT)
                self._ori_what = self._ori_what + a_d * (self._w_err / DT - self._ori_what)
                cutoff = ORI_MIN_CUTOFF + ORI_BETA * float(np.linalg.norm(self._ori_what))
                if ORI_CUTOFF_MAX > 0.0:
                    cutoff = min(cutoff, ORI_CUTOFF_MAX)
            w = OneEuroFilter._alpha(cutoff, DT) * self._w_err
        else:
            w = ORI_ALPHA * self._w_err
        ang = float(np.linalg.norm(w))
        if ang > max_step and ang > 1e-9:
            w = w * (max_step / ang)
        R_new = pin.exp3(w) @ self._R_des_prev
        self._w_err = self._w_err - w
        self._R_des_prev = R_new
        return R_new

    @staticmethod
    def _shake_bands(A, dt):
        """Split a sample window into a slow and a fast oscillation band; returns (pp, f) of each band plus t_eff per column."""
        A = np.asarray(A, dtype=float)
        n = len(A)
        u = (np.arange(n, dtype=float) - 0.5 * (n - 1)) / max(n, 1)
        V = np.vstack([np.ones_like(u), u, u * u]).T
        A = A - V @ np.linalg.lstsq(V, A, rcond=None)[0]
        w = np.hanning(n)
        X = np.abs(np.fft.rfft(A * w[:, None], axis=0)) ** 2
        f = np.fft.rfftfreq(n, dt)
        P = X * (2.0 / (n * n * max(float(np.mean(w * w)), 1e-30)))

        def band(lo, hi):
            m = (f >= lo) & (f <= hi)
            if not np.any(m):
                return np.zeros(A.shape[1]), np.zeros(A.shape[1])
            p = P[m]
            rms2 = np.maximum(p.sum(axis=0), 0.0)
            fc = (f[m][:, None] * p).sum(axis=0) / np.maximum(rms2, 1e-30)
            return 2.0 * np.sqrt(2.0) * np.sqrt(rms2), fc

        pps, fs = band(SHAKE_MIN_HZ, SHAKE_SPLIT_HZ)
        ppf, ff = band(np.nextafter(SHAKE_SPLIT_HZ, np.inf), SHAKE_MAX_HZ)
        return pps, fs, ppf, ff, n * dt

    def vr_tick(self):
        """250 Hz: move the IK target with the controller while engaged, and handle dropouts, clutch, axis lock and buttons."""
        now = time.monotonic()
        dt_vr = VR_DT if self._vr_t_last is None else min(max(now - self._vr_t_last, 0.25 * VR_DT), 4.0 * VR_DT)
        self._vr_t_last = now
        degraded = DEGRADE_MAX_RATE > 0.0 and now < self._track_bad_until
        fresh = (self._cur is not None and (now - self._pose_t) < POSE_TIMEOUT
                 and not degraded)

        if self.shared["ready"] and self.shared["engaged"]:
            if not fresh:
                if not self._pose_lost:
                    self._pose_lost = True
                    if degraded:
                        self._degrade_holds += 1
                        self.get_logger().warn(
                            f"TRACKING DEGRADED ({len(self._glitch_t)} rejects in "
                            f"{DEGRADE_WINDOW:.1f} s) -> target held")
                    else:
                        self.get_logger().warn("pose stale -> target frozen")
            else:
                if self._pose_lost:
                    self._pose_lost = False
                    self._capture_refs()
                    self.get_logger().warn("pose recovered -> re-anchored")
                dl = self._R_ref.T @ (self._cur - self.shared["ref"])
                d = dl[AXIS_MAP]
                newp = self.shared["anchor"] + SCALE * AXIS_SIGN * d
                if self._axlock_active:
                    s = float((newp - self._axlock_p0) @ self._axlock_u)
                    newp = self._axlock_p0 + s * self._axlock_u
                    raw_cmd = newp.copy()
                else:
                    raw_cmd = newp.copy()
                    arel = self.shared["anchor"] - self.shoulder
                    rel0 = newp - self.shoulder
                    az0 = np.arctan2(arel[1], arel[0])
                    az = np.arctan2(rel0[1], rel0[0])
                    daz_max = np.pi / max(AZ_GAIN, 1.0)
                    daz = np.clip((az - az0 + np.pi) % (2 * np.pi) - np.pi, -daz_max, daz_max)
                    naz = az0 + AZ_GAIN * daz
                    rh = np.hypot(rel0[0], rel0[1])
                    if rh > 1e-3:
                        newp = self.shoulder + np.array([rh * np.cos(naz), rh * np.sin(naz), rel0[2]])
                rel = newp - self.shoulder
                r = np.linalg.norm(rel)
                if r < 1e-6:
                    newp = self.shared["target"]
                elif r > self.r_max:
                    newp = self.shoulder + rel * (self.r_max / r)
                elif r < self.r_min:
                    newp = self.shoulder + rel * (self.r_min / r)
                prev = self.shared["target"]
                stepv = newp - prev
                sn = np.linalg.norm(stepv)
                max_step = MAX_TARGET_SPEED * dt_vr
                if sn > max_step:
                    newp = prev + stepv * (max_step / sn)
                ee = self.ik.fk_grasp(self._R_tcp if TCP_CUTOFF > 0.0 else None)
                lead = newp - ee
                ln = np.linalg.norm(lead)
                if ln > MAX_LEAD:
                    newp = ee + lead * (MAX_LEAD / ln)
                if ANTIWIND_RATE > 0.0:
                    rej = newp - raw_cmd
                    rn = float(np.linalg.norm(rej))
                    self._ee_dt += dt_vr
                    if self._ee_prev is None:
                        self._ee_prev, self._ee_dt = ee.copy(), 0.0
                        self._antiwind_prog = 0.0
                    elif self._ee_dt >= 0.9 * DT:
                        if ln > 1e-9:
                            self._antiwind_prog = float((ee - self._ee_prev) @ (lead / ln)) / self._ee_dt
                        self._ee_prev, self._ee_dt = ee.copy(), 0.0
                    if rn > 1e-9 and self._antiwind_prog < ANTIWIND_STUCK_SPEED:
                        cap = ANTIWIND_RATE * dt_vr
                        self.shared["anchor"] = self.shared["anchor"] + rej * (min(rn, cap) / rn)
                self.shared["target"] = newp

        pad = self._pad_now
        menu = self._menu_now
        if pad and not self._pad_was:
            if (now - self._pad_t) < PAD_BOUNCE:
                self.get_logger().warn(
                    f"PAD bounce ignored ({1000.0 * (now - self._pad_t):.0f} ms "
                    f"since the last toggle)", throttle_duration_sec=1.0)
            elif self._home_active:
                self._pad_t = now
                self._end_home("cancelled by clutch", done=False)
            elif self.shared["engaged"]:
                self._pad_t = now
                self._on_disengage()
            elif (now - self._pad_t) > PAD_DEBOUNCE:
                if self.shared["ready"] and fresh:
                    self._pad_t = now
                    self._on_engage()
                else:
                    msg_age = now - self._pose_msg_t
                    if not self.shared["ready"]:
                        why = "teleop not ready yet (waiting for /joint_states)"
                    elif degraded:
                        why = (f"TRACKING DEGRADED hold active ({len(self._glitch_t)} rejects in "
                               f"{DEGRADE_WINDOW:.1f}s) -> fix the tracking, not the teleop")
                    elif msg_age > POSE_TIMEOUT:
                        why = (f"NO /vive/pose message for {msg_age:.2f}s -> vive_pub down, or the "
                               f"controller is asleep / out of base-station view (its warn is on the "
                               f"HOST terminal; buttons keep working without tracking)")
                    else:
                        why = (f"messages ARE arriving ({1000.0 * msg_age:.0f}ms ago) but the glitch "
                               f"gate is rejecting them ({self._pose_glitches} this window, worst "
                               f"{self._pose_glitch_worst:.1f}x) -> tracking quality is bad")
                    self.get_logger().warn(
                        f"PAD ignored: ready={self.shared['ready']} fresh={fresh} "
                        f"pose_age={now - self._pose_t:.2f}s | {why}")
        self._pad_was = pad
        if menu and not self._menu_was and (now - self._menu_t) > MENU_DEBOUNCE:
            self._menu_t = now
            if self.recorder is not None:
                if self.recorder.recording:
                    homed = False
                    if bool(RECORD_CFG.get("home_on_stop", True)):
                        homed = self._start_home(stop_recording_on_end=True)
                    if not homed:
                        self.recorder.stop()
                        if self.shared["engaged"]:
                            self._on_disengage()
                elif self._home_active:
                    self.get_logger().warn("EPISODE not started: homing in progress")
                else:
                    if not self.shared["engaged"] and self.shared["ready"] and fresh:
                        self._on_engage()
                    if self.shared["engaged"]:
                        if not self.recorder.start():
                            self._on_disengage()
                            self.get_logger().warn(
                                "EPISODE refused -> arm FROZEN as the signal; fix the "
                                "cause above, then press MENU again")
                    else:
                        self.get_logger().warn(
                            f"EPISODE not started: cannot engage first (ready="
                            f"{self.shared['ready']} fresh={fresh}) — press the clutch to see why")
            else:
                self.mark = not self.mark
                self.get_logger().info(f"RECORD {'ON' if self.mark else 'OFF'}")
        self._menu_was = menu
        homeb = self._home_now
        if homeb and not self._home_was and (now - self._home_t) > HOME_DEBOUNCE:
            self._home_t = now
            if self._home_active:
                self._end_home("cancelled by button", done=False)
            else:
                self._start_home()
        self._home_was = homeb
        axl = AXIS_LOCK and self._axlock_now
        if axl and not self._axlock_was and (now - self._axlock_t) > AXIS_LOCK_DEBOUNCE:
            self._axlock_t = now
            if self._axlock_active:
                if AXIS_LOCK_MODE == "toggle":
                    self._axlock_release("button", fresh)
            elif self._home_active or not self.shared["engaged"]:
                self.get_logger().warn("AXIS LOCK ignored: %s" % (
                    "homing in progress" if self._home_active else "engage the clutch first"))
            else:
                self._axlock_engage()
        if self._axlock_active and AXIS_LOCK_MODE != "toggle" and not axl:
            self._axlock_release("button released", fresh)
        self._axlock_was = axl

    def full_cmd(self, positions):
        """Full ARM_CMD_JOINTS vector: teleoperated joints from `positions`, the rest held parked; None while any is unknown."""
        cmd = dict(zip(ARM, positions))
        data = []
        for j in ARM_CMD_JOINTS:
            if j in cmd:
                data.append(float(cmd[j]))
            elif HOLD_PARKED and j in self._parked_cmd:
                data.append(self._parked_cmd[j])
            elif isinstance(self.pos, dict) and j in self.pos:
                data.append(float(self.pos[j]))
            else:
                return None
        return data

    def send_arm(self, positions, v_ff=None):
        """Hand the command to the output shaper, or publish it directly when the shaper is off (out_rate: 0)."""
        data = self.full_cmd(positions)
        if data is None:
            return
        if self.shaper is not None:
            ff = None
            if v_ff is not None:
                vmap = dict(zip(ARM, v_ff))
                ff = [float(vmap.get(j, 0.0)) for j in ARM_CMD_JOINTS]
            self.shaper.set_target(data, ff)
            return
        msg = Float64MultiArray()
        msg.data = data
        self.pub.publish(msg)

    def grip_tick(self):
        """Gripper loop, deliberately independent of the IK tick (see the timer)."""
        now = time.monotonic()
        if (now - self._btn_t) < BUTTONS_TIMEOUT:
            self._grip_update(now)

    def send_grip(self, pos):
        """Send a gripper position goal via the action client; no-op if the server is not ready."""
        if not self.grip.server_is_ready():
            self.get_logger().warn("gripper action server not ready", throttle_duration_sec=2.0)
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(pos)
        goal.command.max_effort = GRIP_EFFORT
        self.grip.send_goal_async(goal).add_done_callback(self._on_grip_response)
        return True

    def _on_grip_response(self, fut):
        """Clear _grip_sent when the action server rejects a goal, so the next tick re-sends it."""
        try:
            handle = fut.result()
        except Exception as exc:
            handle = None
            self.get_logger().warn(f"gripper goal failed: {exc}", throttle_duration_sec=2.0)
        if handle is None or not getattr(handle, "accepted", True):
            self._grip_sent = None
            self.get_logger().warn("gripper goal REJECTED -> will re-send", throttle_duration_sec=2.0)

    def _grip_update(self, now):
        """Trigger-click toggle: one click closes the gripper, the next opens it; debounced and re-sent until accepted."""
        click = self._click_now
        if click and not self._click_was and (now - self._grip_toggle_t) > GRIP_DEBOUNCE:
            self._grip_want_closed = not self._grip_want_closed
            self._grip_toggle_t = now
            self.get_logger().info("GRIP -> " + ("CLOSE" if self._grip_want_closed else "OPEN"))
        self._click_was = click
        want = GRIP_CLOSE if self._grip_want_closed else GRIP_OPEN
        if want != self._grip_sent:
            if self.send_grip(want):
                self._grip_sent = want
                self._grip_send_t = now
                self._grip_last_w = None
        elif not self._grip_want_closed and GRIP_JOINT and isinstance(self.pos, dict) and GRIP_JOINT in self.pos:
            meas = float(self.pos[GRIP_JOINT])
            if abs(meas - GRIP_OPEN) > GRIP_RETRY_TOL and (now - self._grip_send_t) > GRIP_RETRY:
                moved = abs(meas - self._grip_last_w) if self._grip_last_w is not None else 1e9
                self._grip_last_w = meas
                if moved < GRIP_RETRY_TOL:
                    if self.send_grip(GRIP_OPEN):
                        self._grip_send_t = now
                        self.get_logger().warn(f"GRIP OPEN stalled at width {meas:.3f} -> goal re-sent")
                else:
                    self._grip_send_t = now

    def on_joint_states(self, msg):
        """Cache measured joint positions and efforts; on the first complete state sync IK to it and enter teleop."""
        nm = dict(zip(msg.name, msg.position))
        if isinstance(self.pos, dict):
            self.pos.update(nm)
        else:
            self.pos = dict(nm)
        if msg.effort:
            em = dict(zip(msg.name, msg.effort))
            if isinstance(self.eff, dict):
                self.eff.update(em)
            else:
                self.eff = em
        if all(j in nm for j in ARM):
            self._js_t = time.monotonic()
        if not all(j in self.pos for j in ARM_CMD_JOINTS):
            return
        if self.phase == "wait":
            mism = {} if self._parked_measured else {
                n: self.pos[n] for n, v in self.ik.locked_ref.items()
                if n in self.pos and abs(self.pos[n] - v) > 0.1}
            if mism:
                self.get_logger().warn(f"parked joints differ from locked_q by >0.1 rad ({sorted(mism)}); rebuilding collision model at the measured parked pose...")
                locked_q = dict(self.ik.locked_ref)
                locked_q.update({n: self.pos[n] for n in self.ik.locked_ref if n in self.pos})
                self.ik = PinkIK(URDF, EE_FRAME, ARM, srdf_path=srdf_path(), package_dirs=mesh_pkg_dirs(), locked_q=locked_q, logger=self.get_logger())
                self.model = self.ik.model
            self._parked_cmd = {j: float(self.pos[j]) for j in ARM_CMD_JOINTS
                                if j not in ARM and j in self.pos}
            if HOLD_PARKED and self._parked_cmd:
                self.get_logger().info(f"parked joints latched at their measured pose "
                                       f"({len(self._parked_cmd)}); they are commanded to hold, not to follow")
            q0 = self.ik.neutral()
            for name in ARM:
                q0[self.ik.qindex(name)] = self.pos[name]
            self.ik.reset_to(q0)
            self.shared["home"] = self.ik.fk_grasp()
            self.shared["target"] = self.shared["home"].copy()
            self.shared["anchor"] = self.shared["home"].copy()
            self.shared["engaged"] = False
            self.shared["ready"] = True
            self.phase = "teleop"
            self.get_logger().info("Teleop ready. Click trackpad to engage.")

    def tick(self):
        """Main 100 Hz loop: diff-IK toward the VR target, clamp against the measured robot, then drive the arm and gripper."""
        if self.phase != "teleop":
            return
        t_tick0 = time.perf_counter()
        now_tick = time.monotonic()
        dt_tick = DT if self._tick_t_last is None else min(max(now_tick - self._tick_t_last, 0.5 * DT), 2.0 * DT)
        self._tick_t_last = now_tick

        if not self._joints_fresh():
            self._joints_lost = True
            self.get_logger().warn("joint_states stale -> IK held", throttle_duration_sec=1.0)
            return
        if self._joints_lost:
            self._joints_lost = False
            self._blend_from = self.ik.arm_positions().copy()
            self._blend_n = BLEND_TICKS
            q = self.ik.neutral()
            for j in ARM:
                q[self.ik.qindex(j)] = self.pos[j]
            self.ik.reset_to(q)
            self.shared["target"] = self.ik.fk_grasp()
            self._R_des_prev = self.ik.fk_rotation()
            self._R_tgt_prev = None
            self._w_err = np.zeros(3)
            self._ee_prev = None
            self._ori_aw_prev = None
            self._cmd_hist = []
            if self.shared["engaged"]:
                self._pose_lost = True
            self.get_logger().warn("joint_states recovered -> re-synced to measured")

        if self._home_active:
            self._consume_Rc()
            self._home_tick(dt_tick, now_tick)
            self._steptimes.append((time.perf_counter() - t_tick0) * 1000.0)
            return

        tgt = self.shared["target"].copy()
        Rc = self._consume_Rc()
        engaged = self.shared["engaged"]

        if engaged and self.ik.geom is not None and MEAS_GATE_FRAC > 0.0 \
                and (now_tick - self._gate_t) >= 0.01:
            self._gate_t = now_tick
            q_meas = self.ik.neutral()
            for j in ARM:
                q_meas[self.ik.qindex(j)] = self.pos[j]
            m_meas, m_pair = self.ik.margin_at(q_meas, MEAS_GATE_FRAC)
            if m_meas < 0.0:
                self.get_logger().warn(
                    f"MEASURED pose inside safety gate: {m_pair} {1000.0*m_meas:.1f}mm "
                    f"below {MEAS_GATE_FRAC:.0%} of its floor -> auto-freeze")
                self._on_disengage()
                return

        w_hand = None
        if engaged and self._axlock_active and self._axlock_R is not None:
            R_des = self._axlock_R
        elif engaged and self.Rc_ref is not None:
            dR = self._R_ref.T @ (Rc @ self.Rc_ref.T) @ self._R_ref
            w = ORI_SIGN * pin.log3(M @ dR @ M.T)
            w_hand = w
            R_des = pin.exp3(w) @ self.R_anchor
            if ROLL_GATE:
                R_des = self._roll_gate(R_des)
        else:
            R_des = self.ik.fk_rotation()
        R_des = self._ori_step(R_des, MAX_ANG_SPEED * dt_tick)
        R_ee = self.ik.fk_rotation()
        w_lead = pin.log3(R_des @ R_ee.T)
        a_lead = float(np.linalg.norm(w_lead))
        if a_lead > MAX_ANG_LEAD and a_lead > 1e-9:
            R_clamped = pin.exp3(w_lead * (MAX_ANG_LEAD / a_lead)) @ R_ee
            self._w_err = self._w_err + pin.log3(R_des @ R_clamped.T)
            self._cap_werr()
            R_des = R_clamped
            self._R_des_prev = R_des

        R_tcp = R_des
        if TCP_CUTOFF > 0.0:
            if self._R_tcp is None:
                self._R_tcp = R_des.copy()
            a_t = OneEuroFilter._alpha(TCP_CUTOFF, dt_tick)
            self._R_tcp = pin.exp3(a_t * pin.log3(R_des @ self._R_tcp.T)) @ self._R_tcp
            R_tcp = self._R_tcp
        tgt = tgt - R_tcp @ self.ik.tcp
        q_arm = self.ik.step(tgt, R_des, dt_tick)

        q_ik = q_arm.copy()
        mlead_now = 0.0
        if MAX_JOINT_LEAD > 0 and self._joints_fresh() and all(j in self.pos for j in ARM):
            meas = np.array([self.pos[j] for j in ARM], float)
            lead = q_arm - meas
            j_star = int(np.argmax(np.abs(lead)))
            mlead = float(abs(lead[j_star]))
            mlead_now = mlead
            self._clamp_overshoot = False
            if mlead > MAX_JOINT_LEAD:
                dcmd = (q_ik[j_star] - self._cmd_hist[0][j_star]) if len(self._cmd_hist) >= CLAMP_HIST else 0.0
                overshoot = (CLAMP_ASYM and abs(dcmd) > CLAMP_DCMD_EPS
                             and lead[j_star] * dcmd < 0.0)
                self._clamp_overshoot = overshoot
                if overshoot:
                    self.get_logger().warn(
                        "robot overshot a reversal -> letting it brake (no IK yank)",
                        throttle_duration_sec=1.0)
                else:
                    self.ik.set_arm(meas + lead * (MAX_JOINT_LEAD / mlead))
                    if self.ik.geom is not None and self.ik.margin_at(self.ik.q)[0] < 0 and self._last_cmd is not None:
                        self.ik.set_arm(self._last_cmd)
                    q_arm = self.ik.arm_positions()
                    self._qarm_prev = None
                    if CLAMP_REANCHOR:
                        R_ee2 = self.ik.fk_rotation()
                        w2 = pin.log3(R_des @ R_ee2.T)
                        a2 = float(np.linalg.norm(w2))
                        if a2 > MAX_ANG_LEAD and a2 > 1e-9:
                            R_re = pin.exp3(w2 * (MAX_ANG_LEAD / a2)) @ R_ee2
                            self._w_err = self._w_err + pin.log3(R_des @ R_re.T)
                            self._cap_werr()
                            self._R_des_prev = R_re
                    self.get_logger().warn(
                        "cmd ran ahead of robot (lag) -> clamped to measured + max_joint_lead",
                        throttle_duration_sec=1.0)
        self._cmd_hist.append(q_ik)
        if len(self._cmd_hist) > CLAMP_HIST:
            self._cmd_hist.pop(0)

        if (ORI_ANTIWIND_RATE > 0.0 and engaged and not self._axlock_active
                and w_hand is not None):
            n_err = float(np.linalg.norm(self._w_err))
            if (n_err > ORI_ANTIWIND_DEADBAND and self._ori_aw_prev is not None
                    and (now_tick - self._ori_aw_t) < 2.5 * DT):
                u = self._w_err / n_err
                prog = float(pin.log3(R_ee @ self._ori_aw_prev.T) @ u) \
                    / max(now_tick - self._ori_aw_t, 1e-6)
                if prog < ORI_ANTIWIND_STUCK:
                    step = min(ORI_ANTIWIND_RATE * dt_tick, n_err)
                    self.R_anchor = pin.exp3(pin.exp3(w_hand).T @ (-step * u)) @ self.R_anchor
                    self._ori_aw_sum += step
        self._ori_aw_prev = R_ee.copy()
        self._ori_aw_t = now_tick

        if SHAKE_WATCH > 0.0 and engaged:
            self._shk.append(q_ik)
            self._shk_t.append(tgt)
            n_win = max(8, int(SHAKE_WATCH / max(dt_tick, 1e-6)))
            if len(self._shk) > n_win:
                self._shk.pop(0)
                self._shk_t.pop(0)
            self._shk_i += 1
            if len(self._shk) >= n_win and self._shk_i % max(1, n_win // 4) == 0:
                pps, fss, ppf, ffs, teff = self._shake_bands(np.array(self._shk), dt_tick)
                hit_s = pps > SHAKE_MIN_PP
                hit_f = ppf > SHAKE_MIN_PP
                score = np.maximum(pps * hit_s, ppf * hit_f)
                if float(score.max()) > 0.0:
                    j = int(np.argmax(score))
                    slow_led = bool(pps[j] * hit_s[j] >= ppf[j] * hit_f[j])
                    pp, freq = (pps[j], fss[j]) if slow_led else (ppf[j], ffs[j])
                    cap = float(self.ik.model.velocityLimit[
                        self.ik.model.joints[self.ik.model.getJointId(ARM[j])].idx_v])
                    rate_pk = np.pi * freq * pp
                    pair = ""
                    if self.ik.blocked and self.ik.active_pairs:
                        pair = " pair=" + self.ik._pair_name(self.ik.active_pairs[0])
                    tps, tfs, tpf, tff, _ = self._shake_bands(np.array(self._shk_t), dt_tick)
                    tpp, tfr = (tps, tfs) if slow_led else (tpf, tff)
                    jt = int(np.argmax(tpp))
                    src = ("TARGET (input/filter)" if float(tpp[jt]) > 3e-4
                           else "IK (target is smooth)")
                    self.get_logger().warn(
                        f"SHAKE {ARM[j]} {np.degrees(pp):.2f}deg pp @ ~{freq:.1f} Hz "
                        f"({'SLOW' if slow_led else 'FAST'} band; slow "
                        f"{np.degrees(pps[j]):.2f}deg fast {np.degrees(ppf[j]):.2f}deg) "
                        f"| needs {rate_pk:.2f} of {cap:.2f} rad/s "
                        f"| SOURCE={src} tgt={1000.0*float(tpp[jt]):.1f}mm pp @ "
                        f"~{float(tfr[jt]):.1f} Hz "
                        f"| blocked={self.ik.blocked}{pair} sigma={self.ik.sigma_min:.3f} "
                        f"| lead={np.degrees(mlead_now):.2f}deg of "
                        f"{np.degrees(MAX_JOINT_LEAD):.1f} | roll_disc="
                        f"{np.degrees(self._roll_disc):.2f}deg",
                        throttle_duration_sec=1.0)

        if engaged and NOT_FOLLOW_T > 0.0 and all(j in self.pos for j in ARM):
            meas_now = np.array([self.pos[j] for j in ARM], float)
            dlead = q_arm - meas_now
            k_lag = int(np.argmax(np.abs(dlead)))
            if float(abs(dlead[k_lag])) > NOT_FOLLOW_LEAD:
                if self._lag_t0 is None:
                    self._lag_t0 = now_tick
                elif (now_tick - self._lag_t0) > NOT_FOLLOW_T:
                    self.get_logger().warn(
                        f"robot NOT FOLLOWING ({ARM[k_lag]} cmd-meas {dlead[k_lag]:+.3f} rad > "
                        f"{NOT_FOLLOW_LEAD:.3f} for {now_tick - self._lag_t0:.1f}s | "
                        f"clamp={'OVERSHOOT (robot braking, likely a false alarm)' if self._clamp_overshoot else 'lag'} "
                        f"blocked={self.ik.blocked} sigma={self.ik.sigma_min:.3f}) -> auto-freeze")
                    self._on_disengage()
                    return
            else:
                self._lag_t0 = None

        if self._blend_n > 0 and self._blend_from is not None:
            a = 1.0 - (self._blend_n - 1) / float(BLEND_TICKS)
            q_arm = (1.0 - a) * self._blend_from + a * q_arm
            self._blend_n -= 1

        if DEBUG_PERIOD > 0.0 and (now_tick - self._debug_t) >= DEBUG_PERIOD:
            self._debug_t = now_tick
            gap_d, gap_m, gap_pair = self.ik.min_gap()
            print("blocked=%s min-gap=%+.4f margin=%+.4f sigma=%.4f %s"
                  % (self.ik.blocked, gap_d, gap_m, self.ik.sigma_min, gap_pair), flush=True)
        now_m = time.monotonic()
        if self._steptimes and now_m - self._steptime_last >= 10.0:
            a = np.array(self._steptimes)
            d_ret = self.ik.retreats - self._retreats_last
            self._retreats_last = self.ik.retreats
            aw_txt = f" | ori-antiwind +{np.degrees(self._ori_aw_sum):.0f}deg" if self._ori_aw_sum > 0.005 else ""
            self._ori_aw_sum = 0.0
            self.get_logger().info(f"TICKTIME n={len(a)} p50={np.percentile(a,50):.2f} p95={np.percentile(a,95):.2f} max={a.max():.2f} ms | retreats +{d_ret}/10s{aw_txt}")
            if self.shaper is not None:
                st = self.shaper.stats()
                if st is not None:
                    self.get_logger().info(
                        f"OUTSTREAM n={st[0]} p50={st[1]:.2f} p95={st[2]:.2f} max={st[3]:.2f} ms"
                        + (f" | SNAPS {st[4]} (limiter bypassed — investigate)" if st[4] else ""))
            pd = np.array(self._pose_dts) if self._pose_dts else None
            if pd is None:
                self.get_logger().warn("POSESTREAM /vive/pose SILENT for 10 s (vive_pub down or controller not tracked)")
            else:
                vl = np.array(self._pose_vlin) if self._pose_vlin else np.zeros(1)
                va = np.array(self._pose_vang) if self._pose_vang else np.zeros(1)
                ol = np.array(self._pose_vlin_ok) if self._pose_vlin_ok else np.zeros(1)
                oa = np.array(self._pose_vang_ok) if self._pose_vang_ok else np.zeros(1)
                self.get_logger().info(
                    f"POSESTREAM n={len(pd)} ({len(pd)/10.0:.0f} Hz) p50={np.percentile(pd,50):.1f} "
                    f"p95={np.percentile(pd,95):.1f} max={pd.max():.1f} ms | "
                    f"dropouts(>{POSE_TIMEOUT*1000:.0f}ms) +{self._pose_drops}/10s worst={self._pose_worst:.2f}s | "
                    f"hand raw p95 {np.percentile(vl,95):.2f} m/s {np.percentile(va,95):.1f} rad/s "
                    f"max {vl.max():.2f}/{va.max():.1f} | accepted p95 {np.percentile(ol,95):.2f} m/s "
                    f"{np.percentile(oa,95):.1f} rad/s max {ol.max():.2f}/{oa.max():.1f} "
                    f"| glitches +{self._pose_glitches} "
                    f"(worst {self._pose_glitch_worst:.1f}x gate) | holds +{self._degrade_holds}")
                hz = len(pd) / 10.0
                if 0.0 < POSE_HZ_WARN and hz < POSE_HZ_WARN:
                    self.get_logger().warn(
                        f"QUEST POSE RATE LOW: {hz:.0f} Hz (< {POSE_HZ_WARN:.0f} Hz) -> the headset "
                        f"lost solid 6-DOF tracking of the controller (out of camera view or warm "
                        f"after long use). Motion turns choppy and the gripper lags; reboot the "
                        f"headset ('adb reboot') to restore ~70 Hz.")
            if self._piv_n >= 20:
                try:
                    r = np.linalg.solve(self._piv_M + 1e-9 * np.eye(3), self._piv_b)
                    res = float(np.sqrt(max(self._piv_dp2 - r @ self._piv_b, 0.0) / self._piv_n))
                    self.get_logger().info(
                        f"HANDPIVOT n={self._piv_n} estimate=[{r[0]:+.3f} {r[1]:+.3f} {r[2]:+.3f}] "
                        f"|r|={np.linalg.norm(r):.3f} m residual={1000*res:.0f} mm "
                        f"({'usable' if res < 0.01 else 'noisy - rotate in place to measure'})")
                except np.linalg.LinAlgError:
                    pass
            self._piv_M[:] = 0.0; self._piv_b[:] = 0.0
            self._piv_n = 0; self._piv_dp2 = 0.0
            self._pose_dts, self._pose_drops, self._pose_worst = [], 0, 0.0
            self._pose_vlin, self._pose_vang = [], []
            self._pose_vlin_ok, self._pose_vang_ok = [], []
            self._pose_glitches, self._pose_glitch_worst = 0, 0.0
            self._degrade_holds = 0
            self._steptimes = []
            self._steptime_last = now_m
        if self.ik.blocked:
            self._blk += 1
            if self._blk % 100 == 1:
                names = ", ".join(self.ik._pair_name(k) for k in self.ik.active_pairs[:2])
                self.get_logger().info(f"barrier holding the arm back: {names or 'IK infeasible'}",
                                       throttle_duration_sec=1.0)
        else:
            self._blk = 0
        if SING_SIGMA0 > 0.0 and self.ik.sigma_min < SING_SIGMA0:
            self.get_logger().info(
                f"near {self.ik.sing_kind} singularity (sigma={self.ik.sigma_min:.4f}, "
                f"{SING_REPORT_JOINT}={self.pos.get(SING_REPORT_JOINT, float('nan')):+.3f}) -> IK damped",
                throttle_duration_sec=1.0)
        v_ff = None
        if OUT_FF:
            q_now = np.asarray(q_arm, float)
            if (self._qarm_prev is not None and len(self._qarm_prev) == len(q_now)
                    and (now_m - self._qarm_prev_t) < 1.5 * DT):
                v_ff = (q_now - self._qarm_prev) / max(now_m - self._qarm_prev_t, 1e-6)
            self._qarm_prev = q_now.copy()
            self._qarm_prev_t = now_m
        self._last_cmd = q_arm.copy()
        self.send_arm(q_arm, v_ff)

        self._steptimes.append((time.perf_counter() - t_tick0) * 1000.0)


def snapshot_parked_pose(timeout=PARKED_SNAPSHOT_TIMEOUT):
    """Read one complete /joint_states before the Bridge is built, so the collision model bakes the real parked pose."""
    probe = rclpy.create_node("vive_mantis_parked_probe")
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    seen = {}
    probe.create_subscription(JointState, JOINT_STATES_TOPIC,
                              lambda m: seen.update(zip(m.name, m.position)), qos)
    t0 = time.monotonic()
    try:
        while rclpy.ok() and (time.monotonic() - t0) < timeout:
            rclpy.spin_once(probe, timeout_sec=0.05)
            if all(j in seen for j in ARM_CMD_JOINTS):
                return dict(seen)
    finally:
        probe.destroy_node()
    return dict(seen) if seen else {}


def main():
    """Init rclpy, spin the Bridge node, shut down ROS cleanly."""
    warnings.filterwarnings(
        "ignore",
        message=r"coroutine 'Executor\._make_handler\.<locals>\.handler' was never awaited",
        category=RuntimeWarning,
    )
    rclpy.init()
    node = None
    try:
        node = Bridge(parked=snapshot_parked_pose())
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if getattr(node, "shaper", None) is not None:
                node.shaper.stop()
            if getattr(node, "recorder", None) is not None:
                node.recorder.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
