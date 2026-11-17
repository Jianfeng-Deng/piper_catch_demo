"""Step-wise MIT bring-up tests on ONE follower arm, plus the shared "hold" primitive.

  fw     read the firmware version (query frame only; MIT needs V3 >= S-V1.8-8)
  hold   go to the ready pose (slow MOVE J), enter MIT, hold with gravity feed-forward for 15 s, exit
  track  same entry, then small J2 step + sine tracking (0.5 / 1 / 2 Hz) to measure gain, lag, error

Every motion stage has stop conditions (tracking error, joint speed, torque, arm status, stale feedback,
UI heartbeat, stop request) and ALWAYS ends in MOVE-J position hold at the current pose, never a disable.
`hold_follower()` is also what the front-end "急停保持" button calls: stop sending MIT, hold where it is.
"""
import collections
import csv
import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import common as C  # noqa: E402

from . import gravity  # noqa: E402
from .teleop import MODE_MIT, MODE_MOVE_J, fw_ok  # noqa: E402

READY = np.radians([0.0, 28.6, -57.3, 0.0, 28.6, 0.0])
DATA_DIR = Path(__file__).resolve().parents[2] / "data"
HZ = 200.0


class TestAbort(ValueError):
    pass


FAULT_FLAGS = ("voltage_too_low", "motor_overheating", "driver_overcurrent", "driver_overheating",
               "collision_status", "driver_error_status", "stall_status")


def make_profile(D, vpk, A, fs=1000.0, smooth_ms=40.0):
    """Smoothed trapezoid move of distance D (rad): peak speed vpk (rad/s), accel A (rad/s^2).

    Returns dict(pos, vel, acc, vpk) sampled at fs, starting and ending at rest, ending exactly at D. The velocity
    trapezoid is smoothed by a short moving average so acceleration (and torque) has no steps.
    """
    ta = vpk / A
    da = 0.5 * vpk * ta
    if 2 * da >= D:  # not enough distance to reach vpk: triangular
        vpk = math.sqrt(A * D)
        ta, tc = vpk / A, 0.0
    else:
        tc = (D - 2 * da) / vpk
    n = int((2 * ta + tc) * fs) + 1
    t = np.arange(n) / fs
    v = np.where(t < ta, A * t, np.where(t < ta + tc, vpk, np.maximum(0.0, vpk - A * (t - ta - tc))))
    w = max(1, int(smooth_ms * fs / 1000.0))
    vs = np.convolve(v, np.ones(w) / w, mode="full")
    pos = np.cumsum(vs) / fs
    k = D / pos[-1]
    vs, pos = vs * k, pos * k
    return {"pos": pos, "vel": vs, "acc": np.gradient(vs, 1.0 / fs), "vpk": float(vs.max())}


def hold_follower(reader, rate=1):
    """Position-hold at the CURRENT pose (MOVE J). Leaves the arm enabled; leaves MIT."""
    q, _ = C.get_q(reader.piper)
    q = C.clamp_q(q, 0.0)
    with reader.tx_lock:
        p = reader.piper
        p.MotionCtrl_2(ctrl_mode=1, move_mode=1, move_spd_rate_ctrl=rate, is_mit_mode=0)
        p.JointCtrl(*[int(round(math.degrees(x) * 1000)) for x in q])


def query_firmware(reader, timeout=2.0):
    p = reader.piper
    with reader.tx_lock:
        if hasattr(p, "_firmware_data_mtx"):
            with p._firmware_data_mtx:
                p._firmware_data = bytearray()
        p.SearchPiperFirmwareVersion()
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        v = p.GetPiperFirmwareVersion()
        if isinstance(v, str) and v.startswith("S-V"):
            return v
        time.sleep(0.05)
    return None


def _fit_sine(t, y, w):
    """Least-squares y ~ c + a*cos(wt) + b*sin(wt); returns (complex amplitude a - jb, offset)."""
    A = np.column_stack([np.ones_like(t), np.cos(w * t), np.sin(w * t)])
    c, a, b = np.linalg.lstsq(A, y, rcond=None)[0]
    return complex(a, -b), c


SPEED_CFG = {
    "speed": dict(key="speed", joint=1, name="J2", swing=50.0, levels=[0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5], accel_cap=30.0, need=None),
    "speed_j3": dict(key="speed_j3", joint=2, name="J3", swing=-50.0, levels=[0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5], accel_cap=40.0, need=2.9),
    "speed_j5": dict(key="speed_j5", joint=4, name="J5", swing=-50.0, levels=[0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0], accel_cap=60.0, need=3.0),
}


class MitTest:
    def __init__(self, follower, side, axes_cfg, params=None):
        self.f, self.side = follower, side
        self.kp, self.kd = list(axes_cfg["kp"]), list(axes_cfg["kd"])
        self.residual = list(axes_cfg.get("residual_wire", [0.0] * 6))
        self.params = dict(max_err_deg=8.0, vmax_rad_s=2.0, effort_abort_nm=14.0, hold_s=15.0, heartbeat_timeout_s=1.0,
                           z_min_m=0.10, ready_tol_deg=1.5, joint=1, consecutive=3,
                           # step 4 (speed ramp)
                           speed_levels=[0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5], swing_deg=50.0, speed_z_min_m=0.15,
                           speed_accel_max=30.0, speed_torque_budget_nm=11.0, speed_abort_err_deg=10.0,
                           speed_advance_err_deg=5.0, speed_first_err_deg=3.0, speed_min_ratio=0.8, tip_goal_m_s=1.7, speed_wire_peak_nm=12.5)
        self.params.update(params or {})
        self.chain = C.load_chain()
        self.state, self.step, self.error = "idle", None, None
        self.firmware, self.results = None, {}
        self.log = collections.deque(maxlen=40)
        self.stop_evt = threading.Event()
        self.thread = None
        self.last_heartbeat = 0.0
        self.hb_gap, self.hb_n = 0.0, 0
        self.progress = None
        self.watch_flags = False
        self._speed_rows = []

    # ---------------------------------------------------------------- bookkeeping
    def say(self, msg):
        self.log.append(time.strftime("%H:%M:%S ") + msg)

    def heartbeat(self):
        now = time.monotonic()
        if self.active():  # diagnostics: longest silence between UI heartbeats while a test is running
            self.hb_gap = max(self.hb_gap, now - self.last_heartbeat)
            self.hb_n += 1
        self.last_heartbeat = now

    def active(self):
        return self.thread is not None and self.thread.is_alive()

    def stop(self):
        self.stop_evt.set()

    def status(self):
        return {"side": self.side, "state": self.state, "step": self.step, "error": self.error, "firmware": self.firmware,
                "results": self.results, "log": list(self.log), "progress": self.progress, "params": self.params,
                "heartbeat": {"n": self.hb_n, "max_gap_ms": round(self.hb_gap * 1e3)}}

    def _q(self):
        q, ts = C.get_q(self.f.piper)
        return q, (time.time() - ts) if ts else 1e9

    def _mode(self):
        return int(self.f.piper.GetArmStatus().arm_status.mode_feed)

    def _arm_status(self):
        return int(self.f.piper.GetArmStatus().arm_status.arm_status)

    # ---------------------------------------------------------------- step 1
    def fw(self):
        if self.active():
            raise ValueError("测试进行中")
        v = query_firmware(self.f)
        self.firmware = v
        ok = bool(v) and fw_ok(v)
        self.results["fw"] = {"version": v, "ok": ok}
        self.say(f"固件 {v or '无应答'}：" + ("满足 MIT 要求" if ok else "不满足 MIT 要求（需要 V3 >= S-V1.8-8）或无应答"))
        return self.results["fw"]

    # ---------------------------------------------------------------- steps 2/3
    def start(self, step):
        if step not in ("hold", "track") and step not in SPEED_CFG:
            raise ValueError("未知步骤")
        if self.active():
            raise ValueError("已有测试在进行")
        if not self.results.get("fw", {}).get("ok"):
            raise ValueError("请先完成第 1 步固件查询且通过")
        self.heartbeat()
        self.hb_gap, self.hb_n = 0.0, 0
        self.stop_evt.clear()
        self.step, self.error, self.progress = step, None, None
        self.thread = threading.Thread(target=self._run, args=(step,), daemon=True, name=f"mittest-{self.side}-{step}")
        self.thread.start()

    def _beat(self):
        if self.stop_evt.is_set():
            raise TestAbort("收到停止请求")
        silent = time.monotonic() - self.last_heartbeat
        if silent > self.params["heartbeat_timeout_s"]:
            raise TestAbort("界面心跳丢失（已 %.0f ms 无心跳）" % (silent * 1e3))

    def _run(self, step):
        entered = False
        try:
            self.state = "checking"
            self._precheck()
            if step in SPEED_CFG:
                self.state = "checking"
                cfg = dict(SPEED_CFG[step])
                if step == "speed":  # the original J2 test keeps its original parameter names
                    cfg.update(swing=self.params["swing_deg"], levels=self.params["speed_levels"], accel_cap=self.params["speed_accel_max"])
                cfg["swing"] = self.params.get(step + "_swing_deg", cfg["swing"])
                cfg["levels"] = self.params.get(step + "_levels", cfg["levels"])
                self._cfg = cfg
                self._prep_speed()  # all heavy maths before MIT starts
            self.state = "moving"
            self._goto_ready()
            self.state = "entering"
            self._enter()
            entered = True
            self.state = step
            if step == "hold":
                self._do_hold()
            elif step in SPEED_CFG:
                self._do_speed()
            else:
                self._do_track()
            self.say("测试完成，退出 MIT 并保持当前位置")
            err = None
        except Exception as exc:  # noqa: BLE001
            err = str(exc)
            self.say("中止：" + err)
        self.state = "exiting"
        ok = True
        try:
            if entered or err:
                hold_follower(self.f)
                t0 = time.monotonic()
                while self._mode() != MODE_MOVE_J and time.monotonic() - t0 < 1.5:
                    hold_follower(self.f)
                    time.sleep(0.1)
                ok = self._mode() == MODE_MOVE_J
        except Exception as exc:  # noqa: BLE001
            ok = False
            err = (err or "") + f"；退出保持失败 {exc}"
        if not ok:
            err = (err or "") + "；固件未确认回到位置保持，请立即检查"
        if step in SPEED_CFG:
            try:
                self._post_speed()  # heavy maths only now that the arm is back in position hold
            except Exception as exc:  # noqa: BLE001
                self.say(f"速度后处理失败：{exc}")
        self.error = err
        self.state = "fault" if (err and (not ok or "停止" not in err)) else "idle"
        self.say("已回到位置保持（臂仍使能，测完请点“归拢并失能”）" if ok else "退出异常")

    def _precheck(self):
        q, age = self._q()
        if age > 0.1:
            raise TestAbort("从臂反馈不新鲜（未上电？）")
        if self._arm_status() != 0:
            raise TestAbort("从臂状态异常")
        if self._mode() == MODE_MIT:
            raise TestAbort("从臂已处于 MIT，请先点“停止并保持”")
        foreign = C.detect_foreign_commands(self.f.can, 0.5)
        if foreign:
            raise TestAbort("总线上有其它控制者 " + ",".join(hex(i) for i in sorted(foreign)))
        self.say("预检通过")

    def _goto_ready(self):
        p, pr = self.f.piper, self.params
        q, _ = self._q()
        err = float(np.degrees(np.abs(q - READY)).max())
        if err > pr["ready_tol_deg"]:
            path = C.path_tcp(self.chain, q, READY)
            if path[:, 2].min() < pr["z_min_m"]:
                raise TestAbort("到准备姿态的路径过低 (%.3f m)" % path[:, 2].min())
        with self.f.tx_lock:
            for _ in range(30):
                C.send_joints(p, q, 5, margin_deg=0.0)
                time.sleep(0.01)
        if not C.enable_arm(p):
            raise TestAbort("无法使能从臂")
        if err <= pr["ready_tol_deg"]:
            self.say("已在准备姿态（偏差 %.1f°）" % err)
            return
        self.say("慢速（10%%）移动到准备姿态，当前偏差 %.1f°" % err)
        t0 = time.monotonic()
        while time.monotonic() - t0 < 40:
            self._beat()
            with self.f.tx_lock:
                C.send_joints(p, READY, 10, margin_deg=0.0)
            time.sleep(0.01)
            q, _ = self._q()
            if np.degrees(np.abs(q - READY)).max() < 1.0:
                self.say("到达准备姿态")
                return
        raise TestAbort("未在 40 s 内到达准备姿态")

    # ---------------------------------------------------------------- MIT core
    def _feedback(self):
        p = self.f.piper
        hs = p.GetArmHighSpdInfoMsgs()
        sp = [getattr(hs, f"motor_{i}").motor_speed * 1e-3 for i in range(1, 7)]
        cu = [getattr(hs, f"motor_{i}").current * 1e-3 for i in range(1, 7)]
        ef = [getattr(hs, f"motor_{i}").effort * 1e-3 for i in range(1, 7)]
        return np.array(sp), np.array(cu), np.array(ef)

    def _faults(self):
        ls = self.f.piper.GetArmLowSpdInfoMsgs()
        out = []
        for i in range(1, 7):
            f = getattr(ls, f"motor_{i}").foc_status
            out += [f"J{i} {k}" for k in FAULT_FLAGS if getattr(f, k, False)]
        return out

    def _send(self, target, vel, qf, wire=None):
        if wire is None:  # default: static gravity from the measured pose (what the console does)
            wire = gravity.wire_references([math.degrees(x) for x in qf], self.residual)
        with self.f.tx_lock:
            p = self.f.piper
            for j in range(6):
                p.JointMitCtrl(j + 1, float(target[j]), float(vel[j]), self.kp[j], self.kd[j], float(wire[j]))
        return wire

    def _enter(self):
        p = self.f.piper
        q0, _ = self._q()
        self.base = C.clamp_q(q0, 0.0)
        with self.f.tx_lock:
            p.MotionCtrl_2(ctrl_mode=1, move_mode=MODE_MIT, move_spd_rate_ctrl=0, is_mit_mode=0xAD)
        self.say("已发送进入 MIT，等待固件确认 mode_feed=6")
        t0 = time.monotonic()
        while time.monotonic() - t0 < 1.5:
            self._beat()
            qf, age = self._q()
            self._send(self.base, np.zeros(6), qf)  # hold where it is while waiting
            if self._mode() == MODE_MIT:
                self.say("固件确认进入 MIT（%.0f ms）" % ((time.monotonic() - t0) * 1e3))
                return
            time.sleep(1 / HZ)
        raise TestAbort("固件未确认进入 MIT")

    def _loop(self, duration, target_fn, label):
        pr = self.params
        period = 1.0 / HZ
        rows = []
        t_start = time.perf_counter()
        next_t = t_start
        bad = {"err": 0, "spd": 0, "eff": 0}
        while True:
            now = time.perf_counter()
            elapsed = now - t_start
            if elapsed >= duration:
                break
            if now < next_t:
                time.sleep(max(0.0, next_t - now - 0.0004))
                continue
            if now - next_t > 5 * period:
                next_t = now
            next_t += period
            self._beat()
            qf, age = self._q()
            if age > 0.1:
                raise TestAbort("从臂反馈中断（%.0f ms 无更新）" % (age * 1e3))
            if self._arm_status() != 0:
                raise TestAbort("从臂状态异常 (%d)" % self._arm_status())
            if self._mode() != MODE_MIT:
                raise TestAbort("固件已退出 MIT 模式")
            res = target_fn(elapsed)
            target, vel = res[0], res[1]
            ff = res[2] if len(res) > 2 else None  # optional precomputed wire torque (model feed-forward)
            err = np.degrees(np.abs(target - qf))
            if self.watch_flags and len(rows) % 10 == 0:
                fl = self._faults()
                if fl:
                    raise TestAbort("驱动器故障标志：" + "，".join(fl))
            sp, cu, ef = self._feedback()
            bad["err"] = bad["err"] + 1 if err.max() > pr["max_err_deg"] else 0
            bad["spd"] = bad["spd"] + 1 if np.abs(sp).max() > pr["vmax_rad_s"] else 0
            bad["eff"] = bad["eff"] + 1 if np.abs(ef).max() > pr["effort_abort_nm"] else 0
            if bad["err"] >= pr["consecutive"]:
                raise TestAbort("跟随误差 %.1f° 超过 %.0f°（J%d）" % (err.max(), pr["max_err_deg"], int(err.argmax()) + 1))
            if bad["spd"] >= pr["consecutive"]:
                raise TestAbort("关节速度 %.2f rad/s 超过 %.1f（疑似振荡）" % (np.abs(sp).max(), pr["vmax_rad_s"]))
            if bad["eff"] >= pr["consecutive"]:
                raise TestAbort("力矩反馈 %.1f N·m 超过 %.0f" % (np.abs(ef).max(), pr["effort_abort_nm"]))
            wire = self._send(target, vel, qf, ff)
            rows.append(np.concatenate([[elapsed], qf, target, sp, cu, ef, wire]))
            self.progress = {"label": label, "elapsed": elapsed, "duration": duration, "err_max_deg": float(err.max())}
        self.progress = None
        return np.array(rows)

    def _save(self, name, rows):
        DATA_DIR.mkdir(exist_ok=True)
        path = DATA_DIR / f"mit_{self.side}_{name}_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        hdr = ["t"] + [f"{k}{j}" for k in ("q", "tgt", "spd", "cur", "eff", "wire") for j in range(1, 7)]
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(hdr)
            w.writerows(np.round(rows, 6))
        return str(path.name)

    # ---------------------------------------------------------------- step 2: hold
    def _do_hold(self):
        dur = self.params["hold_s"]
        self.say("MIT 保持 %.0f s（重力前馈，kp=%g kd=%g）" % (dur, self.kp[0], self.kd[0]))
        base = self.base
        rows = self._loop(dur, lambda t: (base, np.zeros(6)), "MIT 保持")
        m = rows[:, 0] > 1.0
        q = rows[m, 1:7]
        tgt = rows[m, 7:13]
        err = np.degrees(q - tgt)
        sp, cu, ef = rows[m, 13:19], rows[m, 19:25], rows[m, 25:31]
        res = {
            "samples": int(len(rows)), "csv": self._save("hold", rows),
            "mean_err_deg": err.mean(0).round(3).tolist(), "std_err_deg": err.std(0).round(3).tolist(),
            "max_abs_err_deg": np.abs(err).max(0).round(3).tolist(),
            "speed_std_rad_s": sp.std(0).round(4).tolist(),
            "current_mean_a": cu.mean(0).round(3).tolist(), "current_std_a": cu.std(0).round(3).tolist(),
            "effort_mean_nm": ef.mean(0).round(3).tolist(),
            "gravity_wire_mean": rows[m, 31:37].mean(0).round(3).tolist(),
        }
        ok = float(np.abs(res["mean_err_deg"]).max()) <= 1.5 and float(max(res["std_err_deg"])) <= 0.3 and float(max(res["speed_std_rad_s"])) <= 0.05
        res["verdict"] = "通过" if ok else "需检查（偏差/抖动超过阈值）"
        self.results["hold"] = res
        self.say("保持测试 %s：最大平均偏差 %.2f°，最大抖动 std %.2f°" % (res["verdict"], np.abs(res["mean_err_deg"]).max(), max(res["std_err_deg"])))

    # ---------------------------------------------------------------- step 3: small tracking
    def _do_track(self):
        j = self.params["joint"]
        deg = math.pi / 180
        # timeline of J2 offset (rad): settle, smooth step +3 deg and back, sines
        segs = [("settle", 1.0, 0, 0), ("step", 3.0, 3 * deg, 0), ("sine", 6.0, 4 * deg, 0.5), ("sine", 4.0, 4 * deg, 1.0),
                ("sine", 2.5, 3 * deg, 2.0), ("settle", 1.0, 0, 0)]
        edges = np.cumsum([0] + [s[1] for s in segs])
        total = float(edges[-1])
        base = self.base

        def smooth(x):  # quintic 0..1 and its derivative
            x = min(1.0, max(0.0, x))
            return 10 * x ** 3 - 15 * x ** 4 + 6 * x ** 5, (30 * x ** 2 - 60 * x ** 3 + 30 * x ** 4)

        def offset(t):
            k = max(0, min(len(segs) - 1, int(np.searchsorted(edges, t, side="right") - 1)))
            kind, dur, amp, f = segs[k]
            tt = t - edges[k]
            if kind == "step":  # up in the first half, back down in the second half, each with a 0.25 s smooth ramp
                half = dur / 2
                if tt < half:
                    s, ds = smooth(tt / 0.25)
                    return amp * s, amp * ds / 0.25
                s, ds = smooth((tt - half) / 0.25)
                return amp * (1 - s), -amp * ds / 0.25
            if kind == "sine":
                w = 2 * math.pi * f
                return amp * math.sin(w * tt), amp * w * math.cos(w * tt)
            return 0.0, 0.0

        def target_fn(t):
            o, do = offset(t)
            tgt = base.copy()
            vel = np.zeros(6)
            tgt[j] += o
            vel[j] = do
            return tgt, vel

        self.say("MIT 小幅跟踪：J%d 阶跃 ±3°，正弦 0.5/1/2 Hz，共 %.1f s" % (j + 1, total))
        rows = self._loop(total, target_fn, "MIT 小幅跟踪")
        t = rows[:, 0]
        q = rows[:, 1 + j]
        tg = rows[:, 7 + j]
        cur = rows[:, 19 + j]
        out = {"csv": self._save("track", rows), "joint": f"J{j + 1}", "samples": int(len(rows)), "sines": []}
        k = 1  # step segment
        m = (t >= edges[k]) & (t < edges[k + 1])
        ts, qs = t[m] - edges[k], np.degrees(q[m] - base[j])
        half = segs[k][1] / 2
        up = (ts >= 0) & (ts < half)
        fin = qs[up][-max(1, int(0.2 * HZ)):].mean() if up.any() else float("nan")
        amp = 3.0
        peak = qs[up].max() if up.any() else float("nan")
        def cross(frac):
            idx = np.where(qs[up] >= frac * amp)[0]
            return ts[up][idx[0]] if len(idx) else float("nan")
        out["step"] = {"final_deg": round(float(fin), 3), "steady_err_deg": round(float(amp - fin), 3),
                       "overshoot_pct": round(float(max(0.0, (peak - fin) / amp * 100)), 1),
                       "rise_10_90_ms": round(float((cross(0.9) - cross(0.1)) * 1e3), 0)}
        for k in (2, 3, 4):
            kind, dur, amp_r, f = segs[k]
            m = (t >= edges[k] + 1.0 / f) & (t < edges[k + 1])  # drop the first cycle
            w = 2 * math.pi * f
            zq, _ = _fit_sine(t[m], q[m], w)
            zt, _ = _fit_sine(t[m], tg[m], w)
            lag = -np.angle(zq / zt) / w
            resid = np.degrees(q[m] - tg[m])
            out["sines"].append({"freq_hz": f, "amp_deg": round(math.degrees(amp_r), 1), "gain": round(float(abs(zq) / abs(zt)), 3),
                                 "lag_ms": round(float(lag * 1e3), 1), "rms_err_deg": round(float(np.sqrt(np.mean(resid ** 2))), 3),
                                 "peak_current_a": round(float(np.abs(cur[m]).max()), 3)})
        worst = max(s["rms_err_deg"] for s in out["sines"])
        out["verdict"] = "通过" if worst <= 1.0 and all(s["gain"] > 0.8 for s in out["sines"]) else "需检查（误差或增益不理想）"
        self.results["track"] = out
        self.say("跟踪测试 %s：最大 RMS 误差 %.2f°，2 Hz 增益 %.2f，滞后 %.0f ms" % (out["verdict"], worst, out["sines"][-1]["gain"], out["sines"][-1]["lag_ms"]))

    # ---------------------------------------------------------------- steps 4-6: single-joint speed ramps (J2 / J3 / J5)
    def _speed_tables(self, dyn, to_wire, base, j, D, levels, accel, fs=1000.0):
        tables, peak = [], 0.0
        sgn = 1.0 if D >= 0 else -1.0
        for vcmd in levels:
            prof = make_profile(abs(D), vcmd, accel, fs)
            m = len(prof["pos"])
            dw0, dw1, dw2 = int(1.0 * fs), int(0.8 * fs), int(0.6 * fs)
            n = dw0 + m + dw1 + m + dw2 + 2
            off, vel, acc = np.zeros(n), np.zeros(n), np.zeros(n)
            off[dw0:dw0 + m], vel[dw0:dw0 + m], acc[dw0:dw0 + m] = prof["pos"], prof["vel"], prof["acc"]
            off[dw0 + m:dw0 + m + dw1] = abs(D)
            b0 = dw0 + m + dw1
            off[b0:b0 + m], vel[b0:b0 + m], acc[b0:b0 + m] = abs(D) - prof["pos"], -prof["vel"], -prof["acc"]
            off, vel, acc = sgn * off, sgn * vel, sgn * acc
            q = np.tile(base, (n, 1))
            q[:, j] += off
            dq = np.zeros((n, 6))
            dq[:, j] = vel
            ddq = np.zeros((n, 6))
            ddq[:, j] = acc
            wire = np.array([to_wire(dyn.inverse(q[k], dq[k], ddq[k]), self.residual) for k in range(n)])
            peak = max(peak, float(np.abs(wire).max()))
            tables.append({"vcmd": vcmd, "vpk": prof["vpk"], "off": off, "vel": vel, "wire": wire, "n": n, "total": (n - 2) / fs})
        return tables, peak

    def _prep_speed(self):
        """Heavy part, done BEFORE entering MIT (it would starve the SDK feedback thread inside the 200 Hz loop):
        inertia / accel limit, and the full feed-forward torque table of every level, planned around the ready pose.
        The acceleration is reduced until the feed-forward torque of EVERY joint stays under speed_wire_peak_nm."""
        from .dynamics import Dynamics, to_wire

        pr, cfg = self.params, self._cfg
        j = cfg["joint"]
        base = READY.copy()
        D = math.radians(cfg["swing"])
        end = base.copy()
        end[j] += D
        zmin = float(C.path_tcp(self.chain, base, end, 100)[:, 2].min())
        if zmin < pr["speed_z_min_m"]:
            raise TestAbort("摆动路径最低 TCP 高度 %.3f m 低于 %.2f m" % (zmin, pr["speed_z_min_m"]))
        dyn = Dynamics()
        M = dyn.mass_matrix(base)
        g = dyn.gravity_nm(base)
        accel = float(min(cfg["accel_cap"], max(2.0, (pr["speed_torque_budget_nm"] - abs(g[j])) / M[j, j])))
        for _ in range(8):
            tables, peak = self._speed_tables(dyn, to_wire, base, j, D, cfg["levels"], accel)
            if peak <= pr["speed_wire_peak_nm"]:
                break
            accel = max(2.0, accel * pr["speed_wire_peak_nm"] / peak * 0.95)
        else:
            raise TestAbort("无法把前馈力矩压到 %.1f N·m 以内（峰值 %.1f）" % (pr["speed_wire_peak_nm"], peak))
        self._speed = {"tables": tables, "D": D, "zmin": zmin, "accel": accel, "inertia": float(M[j, j]), "gravity": float(g[j]), "wire_peak": peak,
                       "lever": float(np.hypot(*(C.fk_tcp(self.chain, base)[[0, 2]] - np.array([0.0, 0.123])))) if j == 1 else None}
        self.say("前馈表已备好（%s）：%d 个级别，摆动 %.0f°，加速度 %.0f rad/s²，惯量 %.3f kg·m²，前馈力矩峰值 %.1f N·m，路径最低 TCP 高 %.2f m"
                 % (cfg["name"], len(tables), cfg["swing"], accel, M[j, j], peak, zmin))

    def _do_speed(self):
        """Joint out-and-back swings at growing peak speed, with inverse-dynamics feed-forward (M*ddq + C + g + friction).

        A level only counts as passed if tracking stays small AND the joint actually reaches the commanded speed.
        The ramp ends at the first level that fails either (that is the measured limit) or at any abort condition.
        """
        pr, cfg = dict(self.params), self._cfg
        sp = self._speed
        j, fs, base = cfg["joint"], 1000.0, self.base
        res = {"joint": cfg["name"], "swing_deg": cfg["swing"], "accel_rad_s2": round(sp["accel"], 1), "inertia_kgm2": round(sp["inertia"], 3),
               "gravity_nm": round(sp["gravity"], 2), "lever_m": None if sp["lever"] is None else round(sp["lever"], 2), "min_tcp_z_m": round(sp["zmin"], 3),
               "ff_peak_nm": round(sp["wire_peak"], 1), "need_rad_s": cfg["need"], "levels": [], "csv": []}
        self.results[cfg["key"]] = res
        self._speed_rows = []
        self.params.update(max_err_deg=pr["speed_abort_err_deg"], vmax_rad_s=7.0, effort_abort_nm=1e3)
        self.watch_flags = True
        try:
            for tb in sp["tables"]:
                vcmd, vpk, off, vel, wire, n = tb["vcmd"], tb["vpk"], tb["off"], tb["vel"], tb["wire"], tb["n"]

                def fn(t, off=off, vel=vel, wire=wire, n=n):
                    k = min(int(t * fs), n - 1)
                    tgt = base.copy()
                    tgt[j] += off[k]
                    v = np.zeros(6)
                    v[j] = vel[k]
                    return tgt, v, wire[k]

                self._beat()
                self.say("级别 %.1f rad/s（指令峰值 %.2f，前馈力矩峰值 %.1f N·m）" % (vcmd, vpk, float(np.abs(wire).max())))
                try:
                    rows = self._loop(tb["total"], fn, f"{cfg['name']} {vcmd:.1f} rad/s")
                except TestAbort as exc:
                    res["levels"].append({"v_cmd": round(vpk, 2), "result": "中止：" + str(exc)})
                    res["stop_reason"] = "中止：" + str(exc)
                    raise
                t, q, tgt, cur = rows[:, 0], rows[:, 1:7], rows[:, 7:13], rows[:, 19:25]
                # Cheap numpy metrics only between levels. FK and disk I/O here would starve the SDK feedback thread
                # while MIT is running; they run after the arm has left MIT (_post_speed).
                # Achieved joint speed, conservatively: the SMALLER of (a) firmware speed feedback (99.5th pct) and
                # (b) the smoothed derivative of the measured position (raw differences of quantised samples overshoot).
                jv = np.abs(C._smooth(np.gradient(C._smooth(q[:, j], 3), t), 5))
                v_q, v_fb = float(jv.max()), float(np.percentile(np.abs(rows[:, 13 + j]), 99.5))
                err = np.degrees(np.abs(q - tgt))
                v_ach = min(v_q, v_fb)
                lvl = {"v_cmd": round(vpk, 2), "v_joint": round(v_ach, 2), "v_q": round(v_q, 2), "v_fb": round(v_fb, 2),
                       "tip_m_s": None if sp["lever"] is None else round(v_ach * sp["lever"], 2), "tip_src": "估算",
                       "err_max_deg": round(float(err.max()), 2), "err_j_deg": round(float(err[:, j].max()), 2),
                       "err_other_deg": round(float(np.delete(err, j, axis=1).max()), 2),
                       "cur_j_a": round(float(np.abs(cur[:, j]).max()), 2), "cur_max_a": round(float(np.abs(cur).max()), 2)}
                lvl["cur_j2_a"], lvl["err_j2_deg"] = lvl["cur_j_a"], lvl["err_j_deg"]  # names the UI used before the generalisation
                self._speed_rows.append((vcmd, rows))
                thr = pr["speed_first_err_deg"] if vcmd <= 1.0 else pr["speed_advance_err_deg"]
                reason = None
                if lvl["err_max_deg"] > thr:
                    reason = "跟踪误差 %.1f° 超过 %.0f°" % (lvl["err_max_deg"], thr)
                elif lvl["v_joint"] < pr["speed_min_ratio"] * lvl["v_cmd"]:
                    reason = "关节只达到指令的 %.0f%%（速度饱和）" % (100 * lvl["v_joint"] / lvl["v_cmd"])
                lvl["result"] = "通过" if reason is None else "未通过：" + reason
                res["levels"].append(lvl)
                self.say("  → 实测 %s %.2f rad/s，最大误差 %.1f°（其它关节 %.1f°），本关节电流 %.1f A、最大 %.1f A：%s"
                         % (cfg["name"], lvl["v_joint"], lvl["err_max_deg"], lvl["err_other_deg"], lvl["cur_j_a"], lvl["cur_max_a"], lvl["result"]))
                if reason:
                    res["stop_reason"] = reason
                    break
            self._speed_verdict(res)
            self.say(cfg["name"] + " 速度爬升结束：" + res["verdict"])
        finally:
            self.params = pr
            self.watch_flags = False

    def _speed_verdict(self, res):
        ok = [l for l in res["levels"] if l.get("result") == "通过"]
        res["max_ok_joint_rad_s"] = max((l["v_joint"] for l in ok), default=None)
        tips = [l["tip_m_s"] for l in ok if l.get("tip_m_s") is not None]
        res["max_ok_tip_m_s"] = max(tips, default=None)
        if not ok:
            res["verdict"] = "没有通过任何级别"
        elif res.get("need_rad_s"):
            need = res["need_rad_s"]
            res["verdict"] = "%s 最高稳定 %.2f rad/s，%s上抛预设所需的约 %.1f rad/s" % (res["joint"], res["max_ok_joint_rad_s"], "达到" if res["max_ok_joint_rad_s"] >= need else "未达到", need)
        else:
            goal = self.params["tip_goal_m_s"]
            res["verdict"] = "末端 %.2f m/s，%s抛接所需的约 %.1f m/s" % (res["max_ok_tip_m_s"], "达到" if res["max_ok_tip_m_s"] >= goal else "未达到", goal)

    def _post_speed(self):
        """After the arm has left MIT: exact tip speed from FK, and the CSV files."""
        cfg = getattr(self, "_cfg", None)
        res = self.results.get(cfg["key"]) if cfg else None
        if not res:
            return
        prefix = "speed" if cfg["joint"] == 1 else "speed_" + cfg["name"].lower()
        for idx, (vcmd, rows) in enumerate(getattr(self, "_speed_rows", [])):
            if len(rows) < 20:
                continue
            try:
                _tg, _qs, _p, _v, tip, _a = C.tip_kinematics(self.chain, rows[:, 0], rows[:, 1:7])
                if idx < len(res["levels"]) and "v_joint" in res["levels"][idx]:
                    res["levels"][idx]["tip_m_s"] = round(float(tip.max()), 2)
                    res["levels"][idx]["tip_src"] = "FK"
                res["csv"].append(self._save(f"{prefix}_{vcmd:.1f}", rows))
            except Exception as exc:  # noqa: BLE001
                self.say(f"后处理失败（级别 {vcmd}）：{exc}")
        self._speed_rows = []
        self._speed_verdict(res)
        self.say("速度爬升结果（FK 精算）：" + res["verdict"])
