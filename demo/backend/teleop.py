"""Software leader -> follower teleoperation in MIT mode (one side = one master + one follower).

Reproduces the console's MIT lifecycle (explicit enter / 200 Hz hold-and-track / explicit exit to
MOVE-J position hold) with its proven parameters (kp=10, kd=0.8, gravity feed-forward from the identified
URDF, mode confirmed by fresh feedback). Everything that can move an arm is behind:

  * the server's --allow-control switch (checked in app.py),
  * a preflight (fresh feedback both ends, no foreign controller on the bus, alignment, TCP height),
  * a UI heartbeat (browser must keep posting; silence -> hold),
  * per-cycle safety: leader staleness, tracking error, joint limits, TCP floor, rate limit.

A fault or stop always ends in MOVE-J position hold at the follower's CURRENT pose (never a jump).
It never disables the joints: fold the arm with the stow action first (a raised arm would drop).
"""
import collections
import math
import re
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))  # piper_catch/ for common.py
import common as C  # noqa: E402

from . import gravity  # noqa: E402

MODE_MIT = 6
MODE_MOVE_J = 1
HZ = 200.0
DEFAULTS = dict(
    vmax_rad_s=3.0,          # cap on target speed (the 100% MOVE-J speed measured on this arm was ~3 rad/s)
    max_err_deg=12.0,        # abort when |leader - follower| exceeds this (a stuck/blocked follower, or leader jump)
    align_tol_deg=6.0,       # start only when leader and follower are already this close
    z_min_m=0.05,            # TCP height floor (base frame, z up); the target is frozen below it
    leader_timeout_s=0.08,   # stale-leader threshold; held 0.3 s then the session ends
    heartbeat_timeout_s=0.5,
    limit_margin_deg=2.0,
    vel_lpf=0.35,            # low-pass on the velocity reference
    gripper_every=4,         # send the gripper every N cycles
    gripper_max_mm=100.0,
    gripper_enabled=True,
)
FW_RE = re.compile(r"(?:S-)?V?(\d+)\.(\d+)[.-](\d+)")


class TeleopError(ValueError):
    pass


def fw_ok(text):
    m = FW_RE.fullmatch(text or "")
    if not m:
        return False
    major, minor, patch = m.groups()
    patch = patch[:-6] if len(patch) >= 7 else patch
    return (1, 8, 8) <= tuple(map(int, (major, minor, patch))) < (2, 0, 0)


class Teleop:
    def __init__(self, leader, follower, side, axes_cfg, gripper_effort_raw=1000, params=None):
        self.leader, self.follower, self.side = leader, follower, side
        self.kp = list(axes_cfg["kp"])
        self.kd = list(axes_cfg["kd"])
        self.residual = list(axes_cfg.get("residual_wire", [0.0] * 6))
        self.gripper_effort = int(gripper_effort_raw)
        self.params = {**DEFAULTS, **(params or {})}
        self.chain = C.load_chain()
        self.state = "idle"
        self.error = None
        self.stop_evt = threading.Event()
        self.thread = None
        self.lock = threading.RLock()
        self.last_heartbeat = 0.0
        self.last_preflight = None
        self.firmware = None
        self.estopped = False
        self._reset_stats()

    # ------------------------------------------------------------------ helpers
    def _reset_stats(self):
        self.stats = {"cycles": 0, "intervals": collections.deque(maxlen=2000), "max_late_ms": 0.0,
                      "target_frozen": 0, "track_err_deg": [0.0] * 6, "target_deg": None, "wire": None,
                      "started_at": None}

    def heartbeat(self):
        self.last_heartbeat = time.monotonic()

    def _q(self, arm):
        q, ts = C.get_q(arm.piper)
        return q, time.time() - ts if ts else 1e9

    def _mode_feed(self):
        return int(self.follower.piper.GetArmStatus().arm_status.mode_feed)

    def _arm_status(self):
        return int(self.follower.piper.GetArmStatus().arm_status.arm_status)

    def _set(self, state, error=None):
        with self.lock:
            self.state = state
            if error is not None:
                self.error = error

    def active(self):
        return self.state in ("aligning", "starting", "running", "exiting")

    # ------------------------------------------------------------------ preflight
    def preflight(self, check_bus=True):
        pr = self.params
        checks = []

        def add(name, ok, detail=""):
            checks.append({"name": name, "ok": bool(ok), "detail": detail})

        if not self.follower.piper or not self.leader.piper:
            add("适配器连接", False, "主臂或从臂的 USB-CAN 未连接")
            self.last_preflight = {"ok": False, "checks": checks, "align_err_deg": None}
            return self.last_preflight
        qf, agef = self._q(self.follower)
        ql, agel = self._q(self.leader)
        add("从臂反馈新鲜", agef < 0.1, f"{agef * 1e3:.0f} ms" if agef < 1e8 else "无反馈（未上电？）")
        add("主臂反馈新鲜", agel < 0.1, f"{agel * 1e3:.0f} ms" if agel < 1e8 else "无反馈（主臂未上电？）")
        add("从臂状态正常", self._arm_status() == 0 if agef < 0.5 else False, "")
        if check_bus:
            try:
                foreign = C.detect_foreign_commands(self.follower.can, 0.5) if not self.active() else set()
            except Exception as exc:  # noqa: BLE001
                foreign = {-1}
                add("从臂总线无其它控制者", False, f"监听失败: {exc}")
            else:
                add("从臂总线无其它控制者", not foreign, "" if not foreign else "检测到控制帧 " + ",".join(hex(i) for i in sorted(foreign)))
        err = None
        if agef < 0.5 and agel < 0.5:
            err = np.degrees(np.abs(ql - qf))
            add("主从已对齐", float(err.max()) <= pr["align_tol_deg"],
                "最大偏差 %.1f° (阈值 %.0f°，J%d)" % (err.max(), pr["align_tol_deg"], int(err.argmax()) + 1))
            z = float(C.fk_tcp(self.chain, C.clamp_q(ql, pr["limit_margin_deg"]))[2])
            add("主臂姿态 TCP 高度", z >= pr["z_min_m"], "%.3f m (下限 %.2f m)" % (z, pr["z_min_m"]))
        else:
            add("主从已对齐", False, "缺少反馈")
        add("未处于急停锁存", not self.estopped, "")
        ok = all(c["ok"] for c in checks)
        self.last_preflight = {"ok": ok, "checks": checks, "align_err_deg": None if err is None else [round(float(x), 2) for x in err]}
        return self.last_preflight

    # ------------------------------------------------------------------ align (slow MOVE J, follower -> leader)
    def align(self):
        if self.active():
            raise TeleopError("遥操正在进行")
        pr = self.params
        pf = self.preflight()
        for c in pf["checks"]:
            if c["name"] in ("从臂反馈新鲜", "主臂反馈新鲜", "从臂状态正常", "从臂总线无其它控制者", "未处于急停锁存") and not c["ok"]:
                raise TeleopError(f"{c['name']}: {c['detail'] or '不满足'}")
        qf, _ = self._q(self.follower)
        ql, _ = self._q(self.leader)
        target = C.clamp_q(ql, pr["limit_margin_deg"])
        path = C.path_tcp(self.chain, qf, target)
        if path[:, 2].min() < pr["z_min_m"]:
            raise TeleopError("对齐路径会低于 TCP 高度下限 %.2f m（最低 %.3f m）" % (pr["z_min_m"], path[:, 2].min()))
        self.stop_evt.clear()
        self._set("aligning", error="")
        self.thread = threading.Thread(target=self._align_run, args=(qf, target), daemon=True, name=f"align-{self.side}")
        self.thread.start()

    def _align_run(self, q_start, target):
        p = self.follower.piper
        try:
            with self.follower.tx_lock:
                for _ in range(30):
                    C.send_joints(p, q_start, 5, margin_deg=0.0)
                    time.sleep(0.01)
            if not C.enable_arm(p):
                raise TeleopError("无法使能从臂")
            t0 = time.monotonic()
            reached = False
            while time.monotonic() - t0 < 30 and not self.stop_evt.is_set():
                if time.monotonic() - self.last_heartbeat > self.params["heartbeat_timeout_s"]:
                    raise TeleopError("界面心跳丢失，已停止对齐")
                with self.follower.tx_lock:
                    C.send_joints(p, target, 10, margin_deg=0.0)
                time.sleep(0.01)
                q, _ = self._q(self.follower)
                if np.degrees(np.max(np.abs(q - target))) < 1.0:
                    reached = True
                    break
            self._hold()
            self._set("idle" if reached else "fault", error="" if reached else "对齐未在 30 s 内到位或被中止")
        except Exception as exc:  # noqa: BLE001
            self._safe_hold()
            self._set("fault", error=str(exc))

    # ------------------------------------------------------------------ start / stop / estop
    def start(self):
        if self.active():
            raise TeleopError("遥操已在进行")
        if self.estopped:
            raise TeleopError("急停已锁存，需先人工确认并恢复")
        pf = self.preflight()
        if not pf["ok"]:
            bad = [c["name"] + ((": " + c["detail"]) if c["detail"] else "") for c in pf["checks"] if not c["ok"]]
            raise TeleopError("预检未通过: " + "；".join(bad))
        self.heartbeat()
        self.stop_evt.clear()
        self._reset_stats()
        self._set("starting", error="")
        self.thread = threading.Thread(target=self._run, daemon=True, name=f"teleop-{self.side}")
        self.thread.start()

    def stop(self):
        self.stop_evt.set()
        t = self.thread
        if t and t is not threading.current_thread():
            t.join(timeout=5)

    def estop(self):
        self.stop_evt.set()
        self.estopped = True
        errs = {}
        for name, r in (("follower", self.follower),):
            try:
                with r.tx_lock:
                    r.piper.MotionCtrl_1(emergency_stop=1)
            except Exception as exc:  # noqa: BLE001
                errs[name] = str(exc)
        self._set("estopped", error="急停已发送" + (f"（错误 {errs}）" if errs else ""))
        return errs

    def resume(self):
        """Release the e-stop latch (0x02). Joints are left as the firmware leaves them; fold/hold by hand."""
        with self.follower.tx_lock:
            self.follower.piper.MotionCtrl_1(emergency_stop=2)
        self.estopped = False
        self._set("idle", error="")

    # ------------------------------------------------------------------ MIT helpers
    def _hold(self):
        """Position hold at the follower's current pose (console: begin_position_hold)."""
        q, _ = self._q(self.follower)
        with self.follower.tx_lock:
            p = self.follower.piper
            p.MotionCtrl_2(ctrl_mode=1, move_mode=1, move_spd_rate_ctrl=1, is_mit_mode=0)
            p.JointCtrl(*[int(round(math.degrees(x) * 1000)) for x in q])

    def _safe_hold(self):
        try:
            self._hold()
        except Exception:  # noqa: BLE001
            pass

    def _check_firmware(self):
        p = self.follower.piper
        with self.follower.tx_lock:
            if hasattr(p, "_firmware_data_mtx"):
                with p._firmware_data_mtx:
                    p._firmware_data = bytearray()
            p.SearchPiperFirmwareVersion()
        t0 = time.monotonic()
        while time.monotonic() - t0 < 2.0:
            v = p.GetPiperFirmwareVersion()
            if isinstance(v, str) and v.startswith("S-V"):
                self.firmware = v
                if not fw_ok(v):
                    raise TeleopError(f"MIT 需要 V3 固件 >= 1.8-8，当前 {v}")
                return v
            time.sleep(0.05)
        raise TeleopError("固件版本查询超时")

    def _wait_mode(self, mode, timeout=1.5, resend=None):
        t0 = time.monotonic()
        last = 0.0
        while time.monotonic() - t0 < timeout:
            if self._mode_feed() == mode:
                return True
            if resend and time.monotonic() - last > 0.1:
                resend()
                last = time.monotonic()
            time.sleep(0.02)
        return self._mode_feed() == mode

    # ------------------------------------------------------------------ the 200 Hz loop
    def _run(self):
        pr, p = self.params, self.follower.piper
        entered = False
        try:
            self._check_firmware()
            qf, _ = self._q(self.follower)
            with self.follower.tx_lock:
                for _ in range(30):
                    C.send_joints(p, qf, 5, margin_deg=0.0)
                    time.sleep(0.01)
            if not C.enable_arm(p):
                raise TeleopError("无法使能从臂")
            with self.follower.tx_lock:
                for _ in range(30):
                    C.send_joints(p, qf, 5, margin_deg=0.0)
                    time.sleep(0.01)
            with self.follower.tx_lock:
                p.MotionCtrl_2(ctrl_mode=1, move_mode=MODE_MIT, move_spd_rate_ctrl=0, is_mit_mode=0xAD)
            entered = True
            self._set("starting")
            # tracking starts from the follower's actual pose; the confirmation wait keeps sending holds
            target = qf.copy()
            vel_f = np.zeros(6)
            confirmed = False
            period = 1.0 / HZ
            next_t = time.perf_counter()
            stale_since = None
            frozen_since = None
            self.stats["started_at"] = time.time()
            t_enter = time.monotonic()
            k = 0
            while not self.stop_evt.is_set():
                now = time.perf_counter()
                if now < next_t:
                    time.sleep(max(0.0, next_t - now - 0.0004))
                    continue
                late = now - next_t
                self.stats["max_late_ms"] = max(self.stats["max_late_ms"], late * 1e3)
                if late > 5 * period:
                    next_t = now  # never replay a backlog
                next_t += period
                if time.monotonic() - self.last_heartbeat > pr["heartbeat_timeout_s"]:
                    raise TeleopError("界面心跳丢失，已退出 MIT 并保持当前位置")
                qf, agef = self._q(self.follower)
                ql, agel = self._q(self.leader)
                if agef > 0.1:
                    raise TeleopError("从臂反馈中断")
                if self._arm_status() != 0:
                    raise TeleopError("从臂状态异常 (%d)" % self._arm_status())
                if not confirmed:
                    if self._mode_feed() == MODE_MIT:
                        confirmed = True
                        self._set("running")
                    elif time.monotonic() - t_enter > 1.5:
                        raise TeleopError("固件未确认进入 MIT 模式")
                # stale leader: freeze, then end
                if agel > pr["leader_timeout_s"]:
                    stale_since = stale_since or time.monotonic()
                    if time.monotonic() - stale_since > 0.3:
                        raise TeleopError("主臂反馈中断")
                    desired = target
                else:
                    stale_since = None
                    desired = C.clamp_q(ql, pr["limit_margin_deg"])
                err = np.degrees(np.abs(desired - qf))
                self.stats["track_err_deg"] = [round(float(x), 2) for x in err]
                if err.max() > pr["max_err_deg"]:
                    raise TeleopError("跟随误差 %.1f° 超过 %.0f°（J%d），已退出 MIT" % (err.max(), pr["max_err_deg"], int(err.argmax()) + 1))
                # rate-limited target
                step = np.clip(desired - target, -pr["vmax_rad_s"] * period, pr["vmax_rad_s"] * period)
                new = target + step
                if float(C.fk_tcp(self.chain, new)[2]) < pr["z_min_m"]:
                    new = target  # freeze: never walk lower than the floor
                    self.stats["target_frozen"] += 1
                    frozen_since = frozen_since or time.monotonic()
                    if time.monotonic() - frozen_since > 1.0:
                        raise TeleopError("目标持续低于 TCP 高度下限，已退出 MIT")
                else:
                    frozen_since = None
                v_cmd = (new - target) / period
                vel_f = (1 - pr["vel_lpf"]) * vel_f + pr["vel_lpf"] * v_cmd
                vel = np.clip(vel_f, -pr["vmax_rad_s"], pr["vmax_rad_s"])
                target = new
                if confirmed:
                    wire = gravity.wire_references([math.degrees(x) for x in qf], self.residual)
                else:
                    wire = gravity.wire_references([math.degrees(x) for x in qf], self.residual)
                    target = qf.copy()  # until MIT is confirmed, only hold where the arm is
                    vel = np.zeros(6)
                with self.follower.tx_lock:
                    for j in range(6):
                        p.JointMitCtrl(j + 1, float(target[j]), float(vel[j]), self.kp[j], self.kd[j], float(wire[j]))
                    k += 1
                    if confirmed and pr["gripper_enabled"] and k % pr["gripper_every"] == 0:
                        try:
                            gl, tg = C.get_gripper_mm(self.leader.piper)
                            p.GripperCtrl(int(round(max(0.0, min(pr["gripper_max_mm"], gl)) * 1000)), self.gripper_effort, 1, 0)
                        except Exception:  # noqa: BLE001
                            pass
                self.stats["cycles"] += 1
                self.stats["intervals"].append(period + late)
                self.stats["target_deg"] = [round(math.degrees(x), 2) for x in target]
                self.stats["wire"] = [round(float(w), 3) for w in wire]
            self._finish(entered, None)
        except Exception as exc:  # noqa: BLE001
            self._finish(entered, str(exc))

    def _finish(self, entered, error):
        self._set("exiting")
        ok = True
        if entered:
            try:
                self._hold()
                ok = self._wait_mode(MODE_MOVE_J, 1.5, resend=self._hold)
            except Exception as exc:  # noqa: BLE001
                ok = False
                error = (error or "") + f"；退出保持失败: {exc}"
        if error:
            self._set("fault", error=error + ("" if ok else "；固件未确认回到位置保持，请立即检查"))
        elif not ok:
            self._set("fault", error="固件未确认回到位置保持，请立即检查")
        else:
            self._set("idle", error="")

    # ------------------------------------------------------------------ UI snapshot
    def status(self):
        iv = list(self.stats["intervals"])
        hz = (len(iv) / sum(iv)) if iv and sum(iv) > 0 else None
        return {"side": self.side, "state": self.state, "error": self.error, "estopped": self.estopped,
                "firmware": self.firmware, "params": self.params, "preflight": self.last_preflight,
                "loop": {"cycles": self.stats["cycles"], "hz": hz, "max_late_ms": self.stats["max_late_ms"],
                         "target_frozen": self.stats["target_frozen"]},
                "track_err_deg": self.stats["track_err_deg"], "target_deg": self.stats["target_deg"],
                "wire": self.stats["wire"], "kp": self.kp, "kd": self.kd}
