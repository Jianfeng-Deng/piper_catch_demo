"""Slow, supervised "go to this pose" for ONE follower arm (MOVE J position mode, 10% speed by default).

Used by the page's manual-pose card. Safety layers, same philosophy as the MIT tests:
  * check() is read-only: joint limits, straight-joint-path TCP height, travel/time estimate, static gravity torque;
  * a UI heartbeat is required while moving (silence -> hold where it is);
  * stop request / "急停保持" -> MOVE-J hold at the CURRENT pose (never a jump, never a disable);
  * on arrival the arm stays enabled and holding the target (fold it with "归拢并失能" when done).
"""
import collections
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import common as C  # noqa: E402

from . import gravity  # noqa: E402
from .mittest import hold_follower  # noqa: E402

SPEED_PER_PERCENT = 0.04  # rad/s of joint speed per 1% of MOVE-J speed rate (measured: 50% -> ~2 rad/s)


class PoseAbort(ValueError):
    pass


class PoseMove:
    def __init__(self, follower, side, params=None):
        self.f, self.side = follower, side
        self.params = dict(rate=10, tol_deg=0.8, timeout_s=45.0, z_min_m=0.11, heartbeat_timeout_s=1.0, limit_margin_deg=0.2, gravity_warn_wire=12.0)
        self.params.update(params or {})
        self.chain = C.load_chain()
        self.state, self.error = "idle", None
        self.log = collections.deque(maxlen=20)
        self.stop_evt = threading.Event()
        self.thread = None
        self.last_heartbeat = 0.0
        self.target = None
        self.progress = None

    def say(self, m):
        self.log.append(time.strftime("%H:%M:%S ") + m)

    def heartbeat(self):
        self.last_heartbeat = time.monotonic()

    def active(self):
        return self.thread is not None and self.thread.is_alive()

    def stop(self):
        self.stop_evt.set()

    def status(self):
        return {"side": self.side, "state": self.state, "error": self.error, "log": list(self.log), "progress": self.progress,
                "target_deg": None if self.target is None else np.round(np.degrees(self.target), 2).tolist()}

    def _q(self):
        q, ts = C.get_q(self.f.piper)
        return q, (time.time() - ts) if ts else 1e9

    # ------------------------------------------------------------------ read-only check
    def check(self, q_deg):
        pr = self.params
        reasons, warns = [], []
        q_t = np.radians(np.asarray(q_deg, float))
        if q_t.shape != (6,) or not np.isfinite(q_t).all():
            return {"ok": False, "reasons": ["关节角必须是 6 个有限数"], "warnings": []}
        lo = C.JOINT_LIMITS_DEG[:, 0] + pr["limit_margin_deg"]
        hi = C.JOINT_LIMITS_DEG[:, 1] - pr["limit_margin_deg"]
        d = np.degrees(q_t)
        for i in range(6):
            if d[i] < lo[i] or d[i] > hi[i]:
                reasons.append("J%d=%.1f° 超出限位 [%.1f, %.1f]" % (i + 1, d[i], lo[i], hi[i]))
        q_now, age = self._q() if self.f.piper else (None, 1e9)
        if q_now is None or age > 0.5:
            reasons.append("从臂没有新鲜反馈（未上电？）")
            q_now = q_t
        P = C.path_tcp(self.chain, q_now, q_t, 120)
        zmin = float(P[:, 2].min())
        if zmin < pr["z_min_m"]:
            reasons.append("直线关节路径的最低 TCP 高度 %.3f m 低于 %.2f m（可能碰桌面）" % (zmin, pr["z_min_m"]))
        travel = np.degrees(np.abs(q_t - q_now))
        est = float(np.max(np.abs(q_t - q_now)) / (pr["rate"] * SPEED_PER_PERCENT)) + 1.5
        try:
            g = gravity.wire_references(list(d))
        except ValueError:
            g = [0.0] * 6
        if max(abs(x) for x in g) > pr["gravity_warn_wire"]:
            warns.append("该姿态静态重力力矩 %.1f N·m 偏大（线上限幅 16），不宜长时间保持" % max(abs(x) for x in g))
        if self.state not in ("idle", "fault"):
            reasons.append("上一个位姿移动尚未结束")
        return {"ok": not reasons, "reasons": reasons, "warnings": warns, "tcp_z_min_m": round(zmin, 3), "max_travel_deg": round(float(travel.max()), 1),
                "est_time_s": round(est, 1), "rate_percent": pr["rate"], "gravity_wire_nm": [round(float(x), 2) for x in g],
                "from_deg": np.round(np.degrees(q_now), 1).tolist(), "to_deg": np.round(d, 1).tolist()}

    # ------------------------------------------------------------------ execution
    def start(self, q_deg, gripper_mm=None):
        if self.active():
            raise ValueError("正在移动")
        c = self.check(q_deg)
        if not c["ok"]:
            raise ValueError("；".join(c["reasons"]))
        self.target = np.radians(np.asarray(q_deg, float))
        self.heartbeat()
        self.stop_evt.clear()
        self.error, self.progress = None, None
        self.thread = threading.Thread(target=self._run, args=(self.target.copy(), gripper_mm), daemon=True, name=f"posemove-{self.side}")
        self.thread.start()
        return c

    def _beat(self):
        if self.stop_evt.is_set():
            raise PoseAbort("收到停止请求")
        silent = time.monotonic() - self.last_heartbeat
        if silent > self.params["heartbeat_timeout_s"]:
            raise PoseAbort("界面心跳丢失（已 %.0f ms 无心跳）" % (silent * 1e3))

    def _run(self, q_t, gripper_mm):
        pr, p = self.params, self.f.piper
        arrived = False
        try:
            self.state = "checking"
            q0, age = self._q()
            if age > 0.1:
                raise PoseAbort("从臂反馈不新鲜")
            if int(p.GetArmStatus().arm_status.arm_status) != 0:
                raise PoseAbort("从臂状态异常")
            foreign = C.detect_foreign_commands(self.f.can, 0.5)
            if foreign:
                raise PoseAbort("总线上有其它控制者 " + ",".join(hex(i) for i in sorted(foreign)))
            self.say("预检通过，使能（先保持当前位置）")
            with self.f.tx_lock:
                for _ in range(30):
                    C.send_joints(p, q0, 5, margin_deg=0.0)
                    time.sleep(0.01)
            if not C.enable_arm(p):
                raise PoseAbort("无法使能从臂")
            self.state = "moving"
            self.say("以 %d%% 速度移动到目标位姿" % pr["rate"])
            t0, ok_cnt = time.monotonic(), 0
            while time.monotonic() - t0 < pr["timeout_s"]:
                self._beat()
                with self.f.tx_lock:
                    C.send_joints(p, q_t, pr["rate"], margin_deg=0.0)
                time.sleep(0.01)
                q, age = self._q()
                if age > 0.2:
                    raise PoseAbort("从臂反馈中断")
                err = np.degrees(np.abs(q - q_t))
                self.progress = {"err_max_deg": round(float(err.max()), 2), "elapsed": round(time.monotonic() - t0, 1)}
                ok_cnt = ok_cnt + 1 if err.max() < pr["tol_deg"] else 0
                if ok_cnt >= 15:
                    arrived = True
                    break
            if not arrived:
                raise PoseAbort("未在 %.0f s 内到位（最大剩余偏差 %.1f°）" % (pr["timeout_s"], self.progress["err_max_deg"] if self.progress else -1))
            if gripper_mm is not None:
                self.say("夹爪 → %.0f mm" % gripper_mm)
                for _ in range(10):
                    self._beat()
                    with self.f.tx_lock:
                        C.send_gripper(p, gripper_mm, 1.0)
                    time.sleep(0.1)
            self.say("已到位并保持（臂仍使能；用完请点“归拢并失能”）")
            self.state, self.error = "idle", None
        except Exception as exc:  # noqa: BLE001
            self.error = str(exc)
            self.say("中止：" + self.error)
            try:
                hold_follower(self.f)  # hold where it IS, enabled
            except Exception:  # noqa: BLE001
                pass
            self.state = "idle" if "停止" in self.error else "fault"
        self.progress = None
