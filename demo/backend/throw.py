"""Offline planner for the left arm's coordinated J2/J3(/J5) up-toss, plus the right arm's catch pose.

PURE SIMULATION: nothing in here talks to CAN or to any arm. It produces a timeline the front-end plays on the URDF
model: left-arm joints, the bottle (attached to the TCP until release, then ballistic), and the right arm at an IK
catch pose. The three presets were found by a random + local search over start/end poses of J2, J3, J5 (quintic
joint paths, J1 held at the aim angle) with these constraints:

  * peak joint speed <= 3.0 rad/s (MIT measured 3.7 on J2; J3/J5 speeds are NOT yet measured on hardware),
  * peak accel J2 <= 26, J3 <= 28, J5 <= 35 rad/s^2, TCP z >= 0.18 m along the whole path,
  * peak inverse-dynamics torque <= 11 N*m wire-equivalent (the wire limit is 16; a first search without this
    constraint produced J2 torques of 18-19 N*m, which the real arm cannot deliver),
  * release between z = 0.30 and 0.55 m, upward speed >= 1.0 m/s, gripper approach axis ~horizontal (side grasp),
  * minimise |forward speed| (near-vertical toss) and track the requested apex height.
Because J2 carries the whole arm's inertia, the elbow (J3) and wrist (J5) do most of the work; J2 contributes a few degrees.

Ignored: air drag, bottle spin, release timing error (gripper opens ~29 ms +- 3.6 ms after the command), contact.
"""
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import common as C  # noqa: E402

G = 9.81
CHAIN = None

# name -> J2/J3/J5 start,end (deg), duration T (s), release fraction of T
PRESETS = {
    "low": {"label": "低抛（抛高约 12 cm）", "j2": [72.87, 70.833], "j3": [-37.077, -83.667], "j5": [-3.122, -61.445], "T": 0.696, "rho": 0.39},
    "mid": {"label": "中抛（抛高约 18 cm）", "j2": [69.796, 67.83], "j3": [-44.993, -89.638], "j5": [15.478, -28.897], "T": 0.534, "rho": 0.424},
    "high": {"label": "高抛（抛高约 25 cm）", "j2": [74.86, 61.28], "j3": [-58.943, -104.975], "j5": [48.032, 1.113], "T": 0.516, "rho": 0.431},
}
# Oblique presets: start = the user's ready pose (J4, J6 held); J1 sweeps toward the right arm while J2/J3/J5 extend.
# Found by a penalty-based evolutionary search (see module docstring) with: joint speed/accel budgets, wire torque <= 11 N*m,
# release elevation within 5 deg of the target, horizontal azimuth in [-35, 75] deg (world, 0 = +x toward the right arm),
# catch point (descending branch at 50% of apex height) 0.28-0.52 m from the right base AND solvable by the right arm's IK.
PRESETS.update({
    "ob_low": {"label": "斜抛·低（仰角约 36°，出手约 1.5 m/s）", "j1": [-3.1, -48.386], "j2": [107.5, 123.711], "j3": [-104.4, -149.48], "j4": -3.5,
               "j5": [19.7, -20.524], "j6": 39.7, "T": 0.7, "rho": 0.469, "catch_frac": 0.5},
    "ob_mid": {"label": "斜抛·中（仰角约 45°，出手约 1.5 m/s）", "j1": [-3.1, -41.348], "j2": [107.5, 112.701], "j3": [-104.4, -142.147], "j4": -3.5,
               "j5": [19.7, -18.541], "j6": 39.7, "T": 0.695, "rho": 0.462, "catch_frac": 0.5},
})
LIMITS_DEG = {"j1": (-150, 150), "j2": (0, 180), "j3": (-170, 0), "j5": (-70, 70)}


def _chain():
    global CHAIN
    if CHAIN is None:
        CHAIN = C.load_chain()
    return CHAIN


def _Rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _quintic(s):
    s = min(1.0, max(0.0, s))
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5, 30 * s ** 2 - 60 * s ** 3 + 30 * s ** 4, 60 * s - 180 * s ** 2 + 120 * s ** 3


class ArmFrame:
    """Arm base frame <-> world (position + yaw about z), as in piper.json."""

    def __init__(self, base):
        b = base or {}
        self.p = np.array(b.get("base_xyz_m", [0.0, 0.0, 0.0]), float)
        self.yaw = math.radians((b.get("base_rpy_deg", [0, 0, 90]) or [0, 0, 90])[2])

    def to_world(self, p):
        return self.p + _Rz(self.yaw) @ np.asarray(p, float)

    def to_base(self, w):
        return _Rz(-self.yaw) @ (np.asarray(w, float) - self.p)

    def dir_to_world(self, d):
        return _Rz(self.yaw) @ np.asarray(d, float)


def ik_catch(frame, world_point, tool_horizontal=True):
    """Right-arm joints that put the TCP at world_point with the approach axis horizontal and pointing outward."""
    ch = _chain()
    pb = frame.to_base(world_point)
    phi = math.atan2(pb[1], pb[0])
    r_t, z_t = math.hypot(pb[0], pb[1]), pb[2]
    best = None
    for guess in ([60, -80, -20], [90, -60, 0], [40, -100, -30], [70, -50, 20], [100, -90, -10], [30, -60, 20]):
        x = np.radians(guess)
        for _ in range(80):
            def res(v):
                q = np.array([phi, v[0], v[1], 0, v[2], 0])
                T = C.fk_T(ch, q)
                p = (T @ np.array([0, 0, C.TCP_Z, 1.0]))[:3]
                return np.array([math.hypot(p[0], p[1]) - r_t, p[2] - z_t, T[2, 2] if tool_horizontal else 0.0])
            f = res(x)
            if np.abs(f).max() < 1e-6:
                break
            J = np.column_stack([(res(x + np.eye(3)[i] * 1e-6) - f) / 1e-6 for i in range(3)])
            try:
                dx = np.linalg.solve(J + 1e-9 * np.eye(3), -f)
            except np.linalg.LinAlgError:
                break
            x = x + np.clip(dx, -0.3, 0.3)
        q = np.array([phi, x[0], x[1], 0, x[2], 0])
        T = C.fk_T(ch, q)
        radial_out = T[0, 2] * math.cos(phi) + T[1, 2] * math.sin(phi)
        err = np.abs(res(x)).max()
        lim_ok = (LIMITS_DEG["j2"][0] <= math.degrees(x[0]) <= LIMITS_DEG["j2"][1] and LIMITS_DEG["j3"][0] <= math.degrees(x[1]) <= LIMITS_DEG["j3"][1]
                  and abs(math.degrees(x[2])) <= LIMITS_DEG["j5"][1] and abs(math.degrees(phi)) <= LIMITS_DEG["j1"][1])
        score = err + (0 if radial_out > 0 else 1) + (0 if lim_ok else 1)
        if best is None or score < best[0]:
            best = (score, q, err, radial_out > 0, lim_ok)
        if score < 1e-5:
            break
    _, q, err, out_ok, lim_ok = best
    return {"q_deg": np.degrees(q).round(2).tolist(), "tcp_err_mm": round(err * 1e3, 3), "ok": bool(err < 1e-3 and out_ok and lim_ok),
            "pointing_outward": bool(out_ok), "within_limits": bool(lim_ok)}


def plan_throw(preset="mid", aim_deg=-50.0, bases=None, catch_frac=None, dt=0.01, hold_s=0.35, torques=True):
    if preset not in PRESETS:
        raise ValueError("unknown preset")
    P = PRESETS[preset]
    oblique = "j1" in P                                   # oblique presets also sweep J1 and keep J4/J6 at fixed values
    if catch_frac is None:
        catch_frac = P.get("catch_frac", 0.75)
    ch = _chain()
    left, right = ArmFrame((bases or {}).get("left")), ArmFrame((bases or {}).get("right"))
    T, rho = P["T"], P["rho"]
    j2, j3, j5 = [np.radians(P[k]) for k in ("j2", "j3", "j5")]
    aim = math.radians(aim_deg)
    j1 = np.radians(P["j1"]) if oblique else np.array([aim, aim])
    j4c, j6c = math.radians(P.get("j4", 0.0)), math.radians(P.get("j6", 0.0))

    def q_at(t):
        p, dp, ddp = _quintic(t / T)
        d = lambda a: a[1] - a[0]
        q = np.array([j1[0] + d(j1) * p, j2[0] + d(j2) * p, j3[0] + d(j3) * p, j4c, j5[0] + d(j5) * p, j6c])
        dq = np.array([d(j1) * dp / T, d(j2) * dp / T, d(j3) * dp / T, 0, d(j5) * dp / T, 0])
        ddq = np.array([d(j1) * ddp / T ** 2, d(j2) * ddp / T ** 2, d(j3) * ddp / T ** 2, 0, d(j5) * ddp / T ** 2, 0])
        if t >= T:
            dq, ddq = np.zeros(6), np.zeros(6)
        return q, dq, ddq

    # release state (world frame)
    tr = rho * T
    h = 1e-3
    p_r_b = C.fk_tcp(ch, q_at(tr)[0])
    v_r_b = (C.fk_tcp(ch, q_at(tr + h)[0]) - C.fk_tcp(ch, q_at(tr - h)[0])) / (2 * h)
    p_r, v_r = left.to_world(p_r_b), left.dir_to_world(v_r_b)
    zt = C.fk_T(ch, q_at(tr)[0])[:3, 2]
    tilt = math.degrees(math.acos(max(-1, min(1, zt[2]))))
    vz = float(v_r[2])
    v_h = float(np.hypot(v_r[0], v_r[1]))
    elevation = math.degrees(math.atan2(vz, v_h))
    azimuth = math.degrees(math.atan2(v_r[1], v_r[0]))   # world frame: 0 deg = +x (toward the right arm), 90 deg = +y (forward)
    if vz <= 0:
        raise ValueError("release velocity is not upward")
    apex_h = vz ** 2 / (2 * G)
    t_apex = vz / G
    # Catch on the way DOWN at catch_frac * apex height above the release point (a near-vertical toss falls back onto the
    # left gripper, so catching at the release height would put both grippers in the same place).
    catch_dz = max(0.0, min(0.95, catch_frac)) * apex_h
    zc = p_r[2] + catch_dz
    disc = vz ** 2 - 2 * G * (zc - p_r[2])
    if disc < 0:
        raise ValueError("catch height is above the apex")
    tau_c = (vz + math.sqrt(disc)) / G
    ball_at = lambda tau: p_r + v_r * tau + 0.5 * np.array([0, 0, -G]) * tau ** 2
    catch_pt = ball_at(tau_c)
    catch_speed = float(np.linalg.norm(v_r + np.array([0, 0, -G]) * tau_c))
    right_ik = ik_catch(right, catch_pt)

    # timeline
    t_catch = tr + tau_c
    t_end = max(T, t_catch) + hold_s
    ts = np.arange(0.0, t_end + 1e-9, dt)
    left_q, tcp_w, ball_w, left_grip, right_grip = [], [], [], [], []
    for t in ts:
        q, _, _ = q_at(t)
        left_q.append(np.degrees(q).round(3).tolist())
        tcp_w.append(left.to_world(C.fk_tcp(ch, q)).round(4).tolist())
        if t < tr:
            ball = tcp_w[-1]
        elif t < t_catch:
            ball = ball_at(t - tr).round(4).tolist()
        else:
            ball = catch_pt.round(4).tolist()
        ball_w.append(ball)
        left_grip.append(35.0 if t < tr else min(70.0, 35.0 + (t - tr) / 0.1 * 35.0))      # clamps the bottle, opens ~0.1 s after the command
        right_grip.append(70.0 if t < t_catch else max(35.0, 70.0 - (t - t_catch) / 0.15 * 35.0))  # open, closes on the bottle
    left_q_a = np.radians(np.array(left_q))

    # peak joint speed / accel and (optionally) inverse-dynamics torque along the path
    stats_j = {}
    sp = np.array([np.abs(q_at(t)[1]) for t in ts])
    ac = np.array([np.abs(q_at(t)[2]) for t in ts])
    tq = None
    if torques:
        try:
            from .dynamics import Dynamics
            dyn = Dynamics()
            tq = np.array([dyn.inverse(*q_at(t)) for t in ts])
        except Exception:  # noqa: BLE001  (mujoco only on the host)
            tq = None
    for name, i in ((("J1", 0),) if oblique else ()) + (("J2", 1), ("J3", 2), ("J5", 4)):
        stats_j[name] = {"peak_speed_rad_s": round(float(sp[:, i].max()), 2), "peak_acc_rad_s2": round(float(ac[:, i].max()), 1),
                         "peak_torque_nm": None if tq is None else round(float(np.abs(tq[:, i]).max()), 1)}

    warnings = []
    qs, qe = q_at(0)[0], q_at(T)[0]
    for name, lim, idx in (("J2", LIMITS_DEG["j2"], 1), ("J3", LIMITS_DEG["j3"], 2), ("J5", LIMITS_DEG["j5"], 4)):
        vals = np.degrees(left_q_a[:, idx])
        if vals.min() < lim[0] or vals.max() > lim[1]:
            warnings.append(f"{name} 超出关节限位 {lim}")
    z_min = min(p[2] for p in tcp_w)
    if z_min < 0.15:
        warnings.append("TCP 最低高度 %.2f m 低于 0.15 m" % z_min)
    if sp[:, 1].max() > 3.7:
        warnings.append("J2 速度超过已实测的 MIT 3.7 rad/s")
    warnings.append("J3、J5 的高速能力尚未在真机上实测（目前只测过 J2）" if not oblique else "J1 的高速能力、以及 J1/J2/J3/J5 同时运动的组合效果尚未在真机上实测")
    if oblique and sp[:, 0].max() > 3.0:
        warnings.append("J1 速度超过 3.0 rad/s")
    if tq is not None and np.abs(tq[:, [0, 1, 2, 4]]).max() > 12:
        warnings.append("某关节所需力矩超过 12 N·m（线上限幅 16 N·m 的 75%）")
    if not right_ik["ok"]:
        warnings.append("右臂无法在该接取点以水平夹爪姿态到达（IK 失败）")
    right_reach = float(np.linalg.norm(right.to_base(catch_pt)[:2]))

    return {
        "preset": preset, "label": P["label"], "kind": "oblique" if oblique else "vertical", "aim_deg": float(np.degrees(j1[0])), "catch_frac_used": catch_frac, "dt": dt, "t_end": round(t_end, 3),
        "t_release": round(tr, 3), "t_apex": round(tr + t_apex, 3), "t_catch": round(t_catch, 3), "T": T,
        "t": np.round(ts, 3).tolist(), "left_q_deg": left_q, "left_tcp_world": tcp_w, "ball_world": ball_w,
        "left_gripper_mm": np.round(left_grip, 1).tolist(), "right_gripper_mm": np.round(right_grip, 1).tolist(),
        "right_q_deg": right_ik["q_deg"], "right_ik": right_ik,
        "release": {"pos_world": p_r.round(3).tolist(), "vel_world": v_r.round(3).tolist(), "speed_m_s": round(float(np.linalg.norm(v_r)), 2),
                    "upward_m_s": round(vz, 2), "horizontal_m_s": round(float(np.hypot(v_r[0], v_r[1])), 2),
                    "angle_from_vertical_deg": round(math.degrees(math.atan2(math.hypot(v_r[0], v_r[1]), vz)), 1), "gripper_tilt_deg": round(tilt, 0),
                    "elevation_deg": round(elevation, 1), "azimuth_deg": round(azimuth, 1)},
        "flight": {"catch_above_release_m": round(catch_dz, 3), "catch_frac": catch_frac, "apex_above_release_m": round(apex_h, 3), "apex_world": ball_at(t_apex).round(3).tolist(), "time_to_catch_height_s": round(tau_c, 3),
                   "catch_world": catch_pt.round(3).tolist(), "catch_speed_m_s": round(catch_speed, 2), "right_reach_m": round(right_reach, 3)},
        "joints": stats_j, "warnings": warnings,
        "presets": {k: v["label"] for k, v in PRESETS.items()}, "kinds": {k: ("oblique" if "j1" in v else "vertical") for k, v in PRESETS.items()},
        "assumptions": "无空气阻力、无瓶身旋转；出手时刻误差 ±3.6 ms（夹爪张开延迟抖动）按末端速度折算约 ±%.1f mm" % (3.6e-3 * float(np.linalg.norm(v_r)) * 1e3),
    }
