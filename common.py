"""Shared helpers for the Piper catch-demo motion tests.

Conventions
- Joint angles: radians in code, 0.001 deg on the wire (SDK convention).
- FK uses piper_identified.urdf; verified against firmware end-pose feedback
  (flange: 0.09 mm / 0.00 deg). TCP = flange + 0.13503 m along flange z.
- Left arm = can1, right arm = can3 (from piper.json serial mapping).
"""
import math
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
URDF_PATH = HERE / "piper_identified.urdf"
ARM_CAN = {"left": "can1", "right": "can3"}  # last-seen names ONLY; use can_if() (names are not stable)
# USB-CAN adapter serials of the two FOLLOWER arms (piper.json / migration doc). The leader (master)
# arms are 0022003D4648570C20303731 and 002E004B4648570220393831 - never command those.
ARM_SERIAL = {"left": "004C002D4648571620303731", "right": "001C00284648571720303731"}
TCP_Z = 0.13503
G = 9.81

# SDK/firmware joint limits (deg) with a safety margin applied by clamp_q()
JOINT_LIMITS_DEG = np.array(
    [[-150, 150], [0, 180], [-170, 0], [-100, 100], [-70, 70], [-120, 120]], float
)
LIMIT_MARGIN_DEG = 5.0

# Default "ready" pose (rad). From piper.json reference_joints_rad (first 5; J6 = 0).
READY_Q = np.array([0.0, 0.5, -1.0, 0.0, 0.5, 0.0])


# ----------------------------------------------------------------- kinematics
def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _axis_R(axis, q):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def load_chain(path=URDF_PATH):
    root = ET.parse(str(path)).getroot()
    chain = []
    for j in root.findall("joint"):
        o = j.find("origin")
        xyz = [float(v) for v in o.get("xyz").split()] if o is not None else [0.0] * 3
        rpy = [float(v) for v in o.get("rpy").split()] if o is not None else [0.0] * 3
        ax = j.find("axis")
        axis = [float(v) for v in ax.get("xyz").split()] if ax is not None else None
        chain.append((j.get("type"), xyz, rpy, axis))
    return chain


def fk_T(chain, q):
    """4x4 pose of link6 (flange) in the arm base frame."""
    T = np.eye(4)
    qi = 0
    for typ, xyz, rpy, axis in chain:
        To = np.eye(4)
        To[:3, :3] = _rpy_to_R(*rpy)
        To[:3, 3] = xyz
        T = T @ To
        if typ == "revolute":
            Tq = np.eye(4)
            Tq[:3, :3] = _axis_R(axis, q[qi])
            T = T @ Tq
            qi += 1
    return T


def fk_tcp(chain, q):
    """TCP position (m) in the arm base frame."""
    return (fk_T(chain, q) @ np.array([0.0, 0.0, TCP_Z, 1.0]))[:3]


def clamp_q(q, margin_deg=LIMIT_MARGIN_DEG):
    lo = np.radians(JOINT_LIMITS_DEG[:, 0] + margin_deg)
    hi = np.radians(JOINT_LIMITS_DEG[:, 1] - margin_deg)
    return np.clip(q, lo, hi)


def within_limits(q):
    lo = np.radians(JOINT_LIMITS_DEG[:, 0] + LIMIT_MARGIN_DEG)
    hi = np.radians(JOINT_LIMITS_DEG[:, 1] - LIMIT_MARGIN_DEG)
    return bool(np.all(q >= lo) and np.all(q <= hi))


def path_tcp(chain, qa, qb, n=60):
    """TCP positions along a straight joint-space interpolation qa -> qb."""
    return np.array([fk_tcp(chain, qa + (qb - qa) * s) for s in np.linspace(0, 1, n)])


# ---------------------------------------------------------------- tip kinematics
def _smooth(x, n):
    """Moving average over n samples along axis 0 (edge-padded, same length)."""
    n = max(1, int(n)) | 1
    pad = n // 2
    xp = np.pad(x, [(pad, pad)] + [(0, 0)] * (x.ndim - 1), mode="edge")
    c = np.cumsum(np.insert(xp, 0, 0, axis=0), axis=0)
    return (c[n:] - c[:-n]) / n


def tip_kinematics(chain, t, q, fs=1000.0, q_win_ms=15.0, v_win_ms=25.0):
    """Timestamped joint samples -> uniform-grid kinematics.

    The recorder logs a row whenever ANY joint-pair frame arrives, so rows can be <1 ms apart with
    only part of q refreshed (naive differences then give huge fake spikes). Here: drop near-duplicate
    times, resample to a uniform grid by linear interpolation, smooth, then differentiate.

    Returns (tg[N], qs[N,6], pos[N,3], vel[N,3], speed[N], acc_mag[N]).
    """
    t = np.asarray(t, float)
    q = np.asarray(q, float)
    o = np.argsort(t)
    t, q = t[o], q[o]
    keep = np.concatenate([[True], np.diff(t) > 1e-4])
    t, q = t[keep], q[keep]
    tg = np.arange(t[0], t[-1], 1.0 / fs)
    qg = np.column_stack([np.interp(tg, t, q[:, j]) for j in range(q.shape[1])])
    qs = _smooth(qg, q_win_ms * fs / 1000.0)
    pos = np.array([fk_tcp(chain, qi) for qi in qs])
    vel = np.gradient(pos, tg, axis=0)
    vel_s = _smooth(vel, v_win_ms * fs / 1000.0)
    speed = np.linalg.norm(vel_s, axis=1)
    acc = np.linalg.norm(np.gradient(vel_s, tg, axis=0), axis=1)
    return tg, qs, pos, vel_s, speed, acc


def ballistic_from_release(pos, vel, z_catch=None):
    """If released at (pos, vel) in the base frame: return (flight_time, horizontal_range, landing_xyz).

    z is up in the arm base frame. z_catch defaults to the release height.
    """
    z0 = pos[2]
    zc = z0 if z_catch is None else z_catch
    vz = vel[2]
    disc = vz * vz + 2 * G * (z0 - zc)
    if disc < 0:
        return float("nan"), float("nan"), None
    tf = (vz + math.sqrt(disc)) / G
    land = pos + np.array([vel[0] * tf, vel[1] * tf, 0.0])
    land[2] = zc
    return tf, float(np.hypot(vel[0] * tf, vel[1] * tf)), land


# ------------------------------------------------------------------ CAN safety
def can_if(arm):
    """Resolve the SocketCAN interface for an arm by USB serial (CAN names can be re-shuffled)."""
    import subprocess

    want = ARM_SERIAL[arm]
    found = {}
    for p in sorted(Path("/sys/class/net").glob("can*")):
        try:
            props = subprocess.run(["udevadm", "info", "-q", "property", "-p", str(p)],
                                   capture_output=True, text=True, timeout=5).stdout
        except Exception:  # noqa: BLE001
            continue
        for line in props.splitlines():
            if line.startswith("ID_SERIAL_SHORT="):
                found[line.split("=", 1)[1]] = p.name
    if want not in found:
        raise RuntimeError(f"no CAN adapter with serial {want} for {arm} arm; seen {found}")
    return found[want]


def detect_foreign_commands(can_if, secs=1.0):
    """Listen passively; return set of control/config CAN IDs seen from OTHER senders.

    Call BEFORE sending anything. Control IDs: 0x150-0x15F (motion/joint/MIT/gripper),
    0x470-0x47F (enable/config). If any appears, another process is commanding this arm.
    """
    import can

    try:
        bus = can.interface.Bus(channel=can_if, interface="socketcan")
    except TypeError:
        bus = can.interface.Bus(channel=can_if, bustype="socketcan")
    seen = set()
    t0 = time.time()
    while time.time() - t0 < secs:
        m = bus.recv(0.1)
        if m is not None and (0x150 <= m.arbitration_id <= 0x15F or 0x470 <= m.arbitration_id <= 0x47F):
            seen.add(m.arbitration_id)
    bus.shutdown()
    return seen


# ------------------------------------------------------------------ SDK wrapper
def connect(arm):
    from piper_sdk import C_PiperInterface_V2

    p = C_PiperInterface_V2(can_name=can_if(arm), judge_flag=True, can_auto_init=True)
    p.ConnectPort()  # sends only query frames (motor limits / firmware version)
    time.sleep(0.5)
    return p


def get_q(p):
    j = p.GetArmJointMsgs()
    s = j.joint_state
    q = np.radians(np.array([s.joint_1, s.joint_2, s.joint_3, s.joint_4, s.joint_5, s.joint_6], float) * 1e-3)
    return q, j.time_stamp


def get_gripper_mm(p):
    g = p.GetArmGripperMsgs()
    return g.gripper_state.grippers_angle * 1e-3, g.time_stamp


def arm_status_code(p):
    st = p.GetArmStatus().arm_status
    return getattr(st, "arm_status", None)


def enable_arm(p, timeout=6.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        p.EnableArm(7)
        time.sleep(0.1)
        if all(p.GetArmEnableStatus()):
            return True
    return False


def send_joints(p, q, speed_rate, margin_deg=LIMIT_MARGIN_DEG):
    """Position-velocity (MOVE J) joint target. speed_rate in 1..100 (%)."""
    q = clamp_q(np.asarray(q, float), margin_deg)
    p.MotionCtrl_2(0x01, 0x01, int(speed_rate), 0x00)
    p.JointCtrl(*[int(round(math.degrees(x) * 1000)) for x in q])


def hold_here(p, speed_rate=10):
    q, _ = get_q(p)
    send_joints(p, q, speed_rate, margin_deg=0.0)


def move_to(p, q_target, speed_rate, tol_deg=0.6, timeout=12.0, hz=100.0, margin_deg=LIMIT_MARGIN_DEG):
    """Blocking joint move; re-sends the same target at `hz`. Returns True if reached."""
    q_target = clamp_q(np.asarray(q_target, float), margin_deg)
    t0 = time.time()
    still = 0
    last_q, _ = get_q(p)
    while time.time() - t0 < timeout:
        send_joints(p, q_target, speed_rate, margin_deg)
        time.sleep(1.0 / hz)
        q, _ = get_q(p)
        err = np.degrees(np.max(np.abs(q - q_target)))
        moving = np.degrees(np.max(np.abs(q - last_q))) > 0.02
        last_q = q
        if err < tol_deg and not moving:
            still += 1
            if still >= 10:
                return True
        else:
            still = 0
    return False


# Folded ("归拢") rest pose: J2/J3 near their folded stops, wrist straight. Joints are only powered
# down once the arm is here, so it cannot drop onto the table. (Slightly inside the stops on purpose.)
STOW_FILE = HERE / "stow_pose.json"  # per-arm rest angles (deg), saved from the arm's REAL resting pose
_DEFAULT_STOW_DEG = [0.0, 2.0, -2.0, 0.0, 0.0, 0.0]


def stow_q(arm):
    """Folded pose (rad) for this arm: the saved real resting angles, else a generic default.

    The saved angles include where the wrist (J4-J6) actually ends up after power-down, so the
    wrist does not sag further once the joints are disabled.
    """
    import json

    try:
        deg = json.loads(STOW_FILE.read_text())[arm]["deg"]
    except Exception:  # noqa: BLE001
        print(f"WARNING: no saved stow pose for {arm}; using generic default {_DEFAULT_STOW_DEG}")
        deg = _DEFAULT_STOW_DEG
    return np.radians(np.asarray(deg, float))


def is_stowed(q, ref, tol_deg=6.0):
    """True if every joint is within tol of the (limit-clamped) stow pose."""
    ref = clamp_q(ref, 0.0)
    return bool(np.degrees(np.max(np.abs(np.asarray(q) - ref))) < tol_deg)


def stow(p, arm, speed_rate=10, z_min=0.08, timeout=45.0):
    """Slowly fold the arm to its saved stow pose. Returns True if folded. Never disables by itself."""
    chain = load_chain()
    ref = stow_q(arm)
    q, _ = get_q(p)
    path = path_tcp(chain, q, clamp_q(ref, 0.0))
    print(f"stow: TCP z along path min={path[:, 2].min():.3f} m, from {np.round(np.degrees(q), 1)} to {np.round(np.degrees(ref), 1)} deg")
    if path[:, 2].min() < z_min:
        print(f"stow: ABORT, path dips below z_min={z_min} m")
        return False
    reached = move_to(p, ref, speed_rate, tol_deg=1.5, timeout=timeout, margin_deg=0.0)
    q, _ = get_q(p)
    ok = is_stowed(q, ref)
    print(f"stow: reached={reached} folded={ok} final joints(deg)={np.round(np.degrees(q), 1)}")
    return ok


def stow_then_disable(p, arm):
    """Fold the arm; power the joints down ONLY if it ended up folded."""
    ok = stow(p, arm)
    if ok:
        p.DisableArm(7)
        print("arm folded and disabled")
    else:
        print("arm NOT folded: left enabled and holding so it does not fall. Fold it manually/with the console first.")
    return ok


def send_gripper(p, mm, effort_nm=1.0):
    """Gripper target stroke in mm, effort in N*m (SDK range 0..5)."""
    mm = float(np.clip(mm, 0.0, 100.0))  # fitted gripper is the 100 mm (v100) variant per workspace README
    p.GripperCtrl(int(round(mm * 1000)), int(round(effort_nm * 1000)), 0x01, 0)


class Recorder:
    """Background poller: stores a row whenever joint or gripper feedback timestamp changes.

    rows: (t_wall, t_joint_sdk, q0..q5, t_grip_sdk, grip_mm)
    """

    def __init__(self, p, poll_s=0.0005):
        self.p, self.poll_s = p, poll_s
        self.rows = []
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        last = (None, None)
        while not self._stop.is_set():
            q, tj = get_q(self.p)
            gm, tg = get_gripper_mm(self.p)
            if (tj, tg) != last:
                self.rows.append((time.time(), tj, *q, tg, gm))
                last = (tj, tg)
            time.sleep(self.poll_s)

    def start(self):
        self._th.start()
        return self

    def stop(self):
        self._stop.set()
        self._th.join(timeout=1)
        return np.array(self.rows, float)


def confirm(prompt, assume_yes=False):
    if assume_yes:
        return True
    return input(prompt + " Type YES to continue: ").strip() == "YES"
