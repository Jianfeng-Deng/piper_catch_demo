"""Logic tests for the MIT teleop state machine against simulated arms. No hardware, no CAN.

    cd demo && python -m unittest discover -s tests -v
"""
import math
import sys
import threading
import time
import unittest
from pathlib import Path

import numpy as np

DEMO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DEMO))

from backend import teleop as T  # noqa: E402
from backend import gravity  # noqa: E402


class O:
    pass


class FakeArm:
    """Stands in for C_PiperInterface_V2: feedback getters + the control calls teleop uses."""

    def __init__(self, q_deg, grip_mm=30.0, simulate=False):
        self.q = np.radians(q_deg).astype(float)
        self.grip = grip_mm
        self.mode = 1
        self.fw = None
        self.log = []
        self.mit = {}
        self.stale = False
        self.arm_status = 0
        self.enabled = False
        self._stop = threading.Event()
        if simulate:
            threading.Thread(target=self._sim, daemon=True).start()

    def _sim(self):
        last = time.time()
        while not self._stop.is_set():
            time.sleep(0.001)
            now = time.time()
            dt, last = now - last, now
            if self.mode == 6 and len(self.mit) == 6:
                tgt = np.array([self.mit[j][0] for j in range(1, 7)])
                self.q += np.clip((tgt - self.q) * 25.0, -6.0, 6.0) * dt

    def close(self):
        self._stop.set()

    # feedback
    def GetArmJointMsgs(self):
        o = O()
        o.time_stamp = time.time() - (5.0 if self.stale else 0.0)
        js = O()
        for i, v in enumerate(np.degrees(self.q)):
            setattr(js, f"joint_{i + 1}", int(round(v * 1000)))
        o.joint_state = js
        return o

    def GetArmGripperMsgs(self):
        o = O()
        o.time_stamp = time.time()
        o.gripper_state = O()
        o.gripper_state.grippers_angle = int(self.grip * 1000)
        return o

    def GetArmStatus(self):
        o = O()
        o.arm_status = O()
        o.arm_status.mode_feed = self.mode
        o.arm_status.arm_status = self.arm_status
        return o

    def GetArmEnableStatus(self):
        return [self.enabled] * 6

    # control
    def EnableArm(self, motor_num=7, enable_flag=2):
        self.enabled = True
        self.log.append(("EnableArm",))

    def MotionCtrl_2(self, ctrl_mode=1, move_mode=1, move_spd_rate_ctrl=50, is_mit_mode=0, *a):
        self.log.append(("MotionCtrl_2", ctrl_mode, move_mode, move_spd_rate_ctrl, is_mit_mode))
        self.mode = 6 if is_mit_mode == 0xAD else move_mode

    def JointCtrl(self, *j):
        self.log.append(("JointCtrl", j))

    def JointMitCtrl(self, n, pos, vel, kp, kd, t):
        self.log.append(("JointMitCtrl", n, pos, vel, kp, kd, t, self.mode))
        self.mit[n] = (pos, vel)

    def GripperCtrl(self, angle, effort, code, zero):
        self.log.append(("GripperCtrl", angle, effort, code))
        self.grip = angle / 1000.0

    def MotionCtrl_1(self, emergency_stop=0, track_ctrl=0, grag_teach_ctrl=0):
        self.log.append(("MotionCtrl_1", emergency_stop))

    def SearchPiperFirmwareVersion(self):
        self.fw = "S-V1.8-9"

    def GetPiperFirmwareVersion(self):
        return self.fw


class Reader:
    def __init__(self, arm):
        self.piper = arm
        self.can = "fake"
        self.tx_lock = threading.RLock()


READY = [0.0, 28.6, -57.3, 0.0, 28.6, 0.0]
AXES = {"kp": [10.0] * 6, "kd": [0.8] * 6, "residual_wire": [0.0] * 6}


def make(leader_q=READY, follower_q=READY, params=None):
    leader = FakeArm(leader_q)
    follower = FakeArm(follower_q, simulate=True)
    t = T.Teleop(Reader(leader), Reader(follower), "left", AXES, params=params)
    T.C.detect_foreign_commands = lambda *a, **k: set()  # no bus in tests
    stop_hb = threading.Event()

    def hb():
        while not stop_hb.is_set():
            t.heartbeat()
            time.sleep(0.1)

    return t, leader, follower, stop_hb, hb


def wait_for(cond, timeout=4.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.01)
    return False


class TeleopTests(unittest.TestCase):
    def test_firmware_gate(self):
        self.assertTrue(T.fw_ok("S-V1.8-8"))
        self.assertTrue(T.fw_ok("S-V1.8-9"))
        self.assertFalse(T.fw_ok("S-V1.7-4"))
        self.assertFalse(T.fw_ok("S-V2.0-1"))
        self.assertFalse(T.fw_ok(None))

    def test_gravity_matches_console_formula_shape(self):
        nm = gravity.gravity_nm(READY)
        self.assertEqual(len(nm), 6)
        self.assertTrue(all(math.isfinite(x) for x in nm))
        wire = gravity.wire_references(READY)
        self.assertTrue(all(abs(w) <= 16 for w in wire))
        self.assertAlmostEqual(wire[3], nm[3] / 0.813252, places=6)

    def test_preflight_ok_and_misalignment_rejected(self):
        t, l, f, hb_stop, _ = make()
        try:
            self.assertTrue(t.preflight()["ok"])
            l.q = np.radians([0, 28.6 + 20, -57.3, 0, 28.6, 0])
            pf = t.preflight()
            self.assertFalse(pf["ok"])
            self.assertIn("主从已对齐", [c["name"] for c in pf["checks"] if not c["ok"]])
            with self.assertRaises(T.TeleopError):
                t.start()
        finally:
            f.close()

    def test_tracks_leader_and_exits_to_position_hold(self):
        t, l, f, hb_stop, hb = make()
        threading.Thread(target=hb, daemon=True).start()
        try:
            t.start()
            self.assertTrue(wait_for(lambda: t.state == "running"), t.error)
            t0 = time.time()
            while time.time() - t0 < 1.5:  # leader swings +-8 deg on J2 at 1 Hz
                l.q = np.radians(READY) + np.array([0, math.radians(8) * math.sin(2 * math.pi * (time.time() - t0)), 0, 0, 0, 0])
                time.sleep(0.005)
            err = np.degrees(np.abs(l.q - f.q)).max()
            self.assertLess(err, 4.0, f"follower tracking error {err:.2f} deg")
            l.grip = 55.0
            time.sleep(0.1)
            self.assertTrue(any(e[0] == "GripperCtrl" and abs(e[1] - 55000) < 1500 for e in f.log))
            t.stop()
            self.assertEqual(t.state, "idle", t.error)
            self.assertEqual(f.mode, 1)
            mit = [e for e in f.log if e[0] == "JointMitCtrl"]
            self.assertGreater(len(mit), 600)
            self.assertTrue(all(e[4] == 10.0 and e[5] == 0.8 for e in mit))          # fixed kp/kd
            self.assertTrue(all(abs(e[6]) <= 16 for e in mit))                         # wire torque range
            self.assertTrue(all(abs(e[3]) <= 3.0 + 1e-9 for e in mit))                 # vmax
            first_mit = next(i for i, e in enumerate(f.log) if e[0] == "JointMitCtrl")
            enter = next(i for i, e in enumerate(f.log) if e[0] == "MotionCtrl_2" and e[4] == 0xAD)
            self.assertLess(enter, first_mit)                                          # MIT entered before MIT frames
            self.assertEqual(f.log[-1][0], "JointCtrl")                                # ends in position hold
            self.assertGreater(t.status()["loop"]["hz"], 150)
        finally:
            hb_stop.set()
            f.close()

    def test_leader_jump_faults_and_holds_current_pose(self):
        t, l, f, hb_stop, hb = make()
        threading.Thread(target=hb, daemon=True).start()
        try:
            t.start()
            self.assertTrue(wait_for(lambda: t.state == "running"), t.error)
            l.q = l.q + np.radians([0, 40, 0, 0, 0, 0])  # leader teleports 40 deg
            self.assertTrue(wait_for(lambda: t.state == "fault"))
            self.assertIn("跟随误差", t.error)
            self.assertEqual(f.mode, 1)
            self.assertEqual(f.log[-1][0], "JointCtrl")
            held = np.array(f.log[-1][1]) * 1e-3
            self.assertLess(np.abs(held - np.degrees(f.q)).max(), 12.0)  # holds near where it is, not at the leader
        finally:
            hb_stop.set()
            f.close()

    def test_heartbeat_loss_stops(self):
        t, l, f, hb_stop, hb = make(params={"heartbeat_timeout_s": 0.3})
        try:
            t.start()  # no heartbeat thread started
            self.assertTrue(wait_for(lambda: t.state in ("fault", "idle"), 3.0))
            self.assertEqual(t.state, "fault")
            self.assertIn("心跳", t.error)
            self.assertEqual(f.mode, 1)
        finally:
            f.close()

    def test_stale_leader_ends_session(self):
        t, l, f, hb_stop, hb = make()
        threading.Thread(target=hb, daemon=True).start()
        try:
            t.start()
            self.assertTrue(wait_for(lambda: t.state == "running"), t.error)
            l.stale = True
            self.assertTrue(wait_for(lambda: t.state == "fault", 3.0))
            self.assertIn("主臂反馈中断", t.error)
        finally:
            hb_stop.set()
            f.close()

    def test_tcp_floor_freezes_target(self):
        t, l, f, hb_stop, hb = make(params={"z_min_m": 0.44})
        threading.Thread(target=hb, daemon=True).start()
        try:
            t.start()
            self.assertTrue(wait_for(lambda: t.state == "running"), t.error)
            l.q = np.radians([0, 28.6 + 11, -57.3, 0, 28.6, 0])  # leader goes to z=0.426 m, below the 0.44 m floor
            time.sleep(0.4)
            self.assertGreater(t.stats["target_frozen"], 0)
            self.assertGreater(np.degrees(np.abs(f.q - l.q)).max(), 2.0)  # follower did NOT follow all the way down
            z = float(T.C.fk_tcp(t.chain, f.q)[2])
            self.assertGreaterEqual(z, 0.44 - 0.01)                       # follower stays at/above the floor
        finally:
            hb_stop.set()
            t.stop()
            f.close()

    def test_estop_sends_quick_stop_and_latches(self):
        t, l, f, hb_stop, hb = make()
        threading.Thread(target=hb, daemon=True).start()
        try:
            t.start()
            self.assertTrue(wait_for(lambda: t.state == "running"), t.error)
            t.estop()
            self.assertTrue(any(e == ("MotionCtrl_1", 1) for e in f.log))
            wait_for(lambda: not t.thread.is_alive())
            with self.assertRaises(T.TeleopError):
                t.start()
        finally:
            hb_stop.set()
            f.close()


if __name__ == "__main__":
    unittest.main()
