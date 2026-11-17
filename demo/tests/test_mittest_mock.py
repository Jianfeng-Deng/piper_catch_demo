"""Mock tests for the MIT bring-up steps and the hold primitive (no hardware)."""
import math
import sys
import threading
import time
import unittest
from pathlib import Path

import numpy as np

DEMO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DEMO))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from backend import mittest as M  # noqa: E402
from test_teleop_mock import AXES, FakeArm, O, Reader, wait_for  # noqa: E402


class Arm(FakeArm):
    _last = None

    def GetArmHighSpdInfoMsgs(self):
        now, q = time.time(), self.q.copy()
        if self._last is None or now - self._last[0] < 2e-3:
            self._speed = np.zeros(6) if self._last is None else self._speed
            if self._last is None:
                self._last = (now, q)
        else:
            self._speed = (q - self._last[1]) / (now - self._last[0])
            self._last = (now, q)
        o = O()
        for i in range(1, 7):
            m = O()
            m.motor_speed, m.current, m.effort = int(self._speed[i - 1] * 1000), 100, 50
            setattr(o, f"motor_{i}", m)
        return o


    def GetArmLowSpdInfoMsgs(self):
        o = O()
        for i in range(1, 7):
            m = O()
            m.foc_status = O()  # no fault flag set
            setattr(o, f"motor_{i}", m)
        return o


try:
    import mujoco  # noqa: F401
    HAVE_MUJOCO = True
except ImportError:
    HAVE_MUJOCO = False

READY_DEG = [0.0, 28.6, -57.3, 0.0, 28.6, 0.0]


def make(q=READY_DEG, params=None):
    arm = Arm(q, simulate=True)
    M.C.detect_foreign_commands = lambda *a, **k: set()
    t = M.MitTest(Reader(arm), "left", AXES, params=params)
    t.results["fw"] = {"version": "S-V1.8-9", "ok": True}
    stop = threading.Event()

    def hb():
        while not stop.is_set():
            t.heartbeat()
            time.sleep(0.1)

    threading.Thread(target=hb, daemon=True).start()
    return t, arm, stop


class MitTestTests(unittest.TestCase):
    def test_hold_follower_leaves_mit_and_holds_current_pose(self):
        arm = Arm(READY_DEG)
        arm.mode = 6
        M.hold_follower(Reader(arm))
        self.assertEqual(arm.mode, 1)
        self.assertEqual(arm.log[-1][0], "JointCtrl")
        held = np.array(arm.log[-1][1]) * 1e-3
        self.assertLess(np.abs(held - np.array(READY_DEG)).max(), 0.01)

    def test_hold_clamps_to_limits(self):
        arm = Arm([0, -2.1, 1.9, 0, 0, 0])  # the real resting pose is slightly outside the soft limits
        M.hold_follower(Reader(arm))
        held = np.array(arm.log[-1][1]) * 1e-3
        self.assertGreaterEqual(held[1], 0.0)
        self.assertLessEqual(held[2], 0.0)

    def test_requires_firmware_step_first(self):
        arm = Arm(READY_DEG)
        t = M.MitTest(Reader(arm), "left", AXES)
        with self.assertRaises(ValueError):
            t.start("hold")
        self.assertEqual(t.fw()["version"], "S-V1.8-9")
        self.assertTrue(t.results["fw"]["ok"])

    def test_hold_step_passes_and_ends_in_position_hold(self):
        t, arm, stop = make(params={"hold_s": 2.5})
        try:
            t.start("hold")
            self.assertTrue(wait_for(lambda: not t.active(), 12), t.log)
            self.assertEqual(t.state, "idle", t.error)
            r = t.results["hold"]
            self.assertEqual(r["verdict"], "通过", r)
            self.assertEqual(arm.mode, 1)
            self.assertEqual(arm.log[-1][0], "JointCtrl")
            mit = [e for e in arm.log if e[0] == "JointMitCtrl"]
            self.assertGreater(len(mit), 400)
            self.assertTrue(all(e[4] == 10.0 and e[5] == 0.8 and abs(e[6]) <= 16 for e in mit))
        finally:
            stop.set()
            arm.close()

    def test_track_step_measures_gain_and_lag(self):
        t, arm, stop = make()
        try:
            t.start("track")
            self.assertTrue(wait_for(lambda: not t.active(), 40), t.log)
            self.assertEqual(t.state, "idle", t.error)
            r = t.results["track"]
            self.assertEqual([s["freq_hz"] for s in r["sines"]], [0.5, 1.0, 2.0])
            self.assertTrue(all(s["gain"] > 0.8 for s in r["sines"]), r)
            self.assertTrue(all(0 <= s["lag_ms"] < 150 for s in r["sines"]), r)  # the simulated plant lags by ~1/25 s
            self.assertLess(abs(r["step"]["steady_err_deg"]), 0.5)
            self.assertEqual(arm.mode, 1)
        finally:
            stop.set()
            arm.close()

    def test_stop_request_ends_in_hold(self):
        t, arm, stop = make(params={"hold_s": 30})
        try:
            t.start("hold")
            self.assertTrue(wait_for(lambda: t.state == "hold", 6), t.log)
            t.stop()
            self.assertTrue(wait_for(lambda: not t.active(), 6))
            self.assertEqual(arm.mode, 1)
            self.assertEqual(arm.log[-1][0], "JointCtrl")
            self.assertIn("停止", t.error or "")
        finally:
            stop.set()
            arm.close()

    def test_heartbeat_loss_aborts_to_hold(self):
        t, arm, stop = make(params={"hold_s": 30, "heartbeat_timeout_s": 0.4})
        try:
            t.start("hold")
            self.assertTrue(wait_for(lambda: t.state == "hold", 6), t.log)
            stop.set()  # UI goes silent
            self.assertTrue(wait_for(lambda: not t.active(), 6))
            self.assertIn("心跳", t.error)
            self.assertEqual(arm.mode, 1)
        finally:
            arm.close()

    def test_profile_is_smooth_and_ends_at_rest(self):
        for vpk, A, D in [(0.5, 25.0, 0.87), (3.0, 25.0, 0.87), (4.5, 25.0, 0.87), (6.0, 25.0, 0.87)]:
            p = M.make_profile(D, vpk, A)
            self.assertAlmostEqual(p["pos"][-1], D, places=6)                 # ends exactly at D
            self.assertLess(abs(p["vel"][-1]), 1e-3)                          # and at rest
            self.assertLessEqual(p["vpk"], min(vpk, math.sqrt(A * D)) * 1.001 + 1e-9)  # never exceeds what is reachable
            self.assertLessEqual(np.abs(p["acc"]).max(), A * 1.05)            # smoothing does not raise the accel limit
            self.assertGreaterEqual(p["pos"].min(), -1e-9)                    # monotone out, no backtracking

    @unittest.skipUnless(HAVE_MUJOCO, "needs mujoco (available on the host)")
    def test_speed_ramp_runs_levels_with_feedforward(self):
        t, arm, stop = make(params={"speed_levels": [0.5, 1.0, 1.5], "swing_deg": 30.0})
        try:
            t.start("speed")
            self.assertTrue(wait_for(lambda: not t.active(), 60), t.log)
            self.assertEqual(t.state, "idle", t.error)
            r = t.results["speed"]
            self.assertEqual(len(r["levels"]), 3, r)
            self.assertTrue(all(l["result"] == "通过" for l in r["levels"]), r["levels"])
            self.assertEqual(len(r["csv"]), 3)
            self.assertGreater(r["inertia_kgm2"], 0.1)                        # J2 inertia at the ready pose is ~0.3
            self.assertIn("m/s", r["verdict"])
            self.assertEqual(arm.mode, 1)
            mit = [e for e in arm.log if e[0] == "JointMitCtrl"]
            self.assertTrue(all(abs(e[6]) <= 16 for e in mit))
        finally:
            stop.set()
            arm.close()

    @unittest.skipUnless(HAVE_MUJOCO, "needs mujoco (available on the host)")
    def test_speed_ramp_stops_at_first_failing_level(self):
        # a plant that can only move 1 rad/s: levels above it must fail (saturation) and end the ramp
        t, arm, stop = make(params={"speed_levels": [0.5, 1.0, 2.5, 3.5], "swing_deg": 40.0})
        orig = arm._sim

        def slow_sim():
            last = time.time()
            while not arm._stop.is_set():
                time.sleep(0.001)
                now = time.time()
                dt, last = now - last, now
                if arm.mode == 6 and len(arm.mit) == 6:
                    tgt = np.array([arm.mit[j][0] for j in range(1, 7)])
                    arm.q += np.clip((tgt - arm.q) * 25.0, -1.0, 1.0) * dt

        try:
            arm._stop.set()
            time.sleep(0.05)
            arm._stop.clear()
            threading.Thread(target=slow_sim, daemon=True).start()
            t.start("speed")
            self.assertTrue(wait_for(lambda: not t.active(), 90), t.log)
            r = t.results["speed"]
            self.assertLess(len(r["levels"]), 4, r)                           # ramp ended early
            self.assertIn("stop_reason", r)
            self.assertEqual(arm.mode, 1)
        finally:
            stop.set()
            arm._stop.set()

    @unittest.skipUnless(HAVE_MUJOCO, "needs mujoco (available on the host)")
    def test_j3_and_j5_ramps_have_their_own_results_and_bounded_feedforward(self):
        for step, key, name in (("speed_j3", "speed_j3", "J3"), ("speed_j5", "speed_j5", "J5")):
            t, arm, stop = make(params={step + "_levels": [0.5, 1.0], step + "_swing_deg": -30.0})
            try:
                t.start(step)
                self.assertTrue(wait_for(lambda: not t.active(), 60), t.log)
                self.assertEqual(t.state, "idle", t.error)
                r = t.results[key]
                self.assertEqual(r["joint"], name)
                self.assertEqual(len(r["levels"]), 2, r)
                self.assertLessEqual(r["ff_peak_nm"], 12.5 + 1e-6)                # every joint's feed-forward stays under budget
                self.assertIsNotNone(r["need_rad_s"])
                self.assertIn("rad/s", r["verdict"])
                self.assertTrue(all(abs(e[6]) <= 16 for e in arm.log if e[0] == "JointMitCtrl"))
                self.assertEqual(arm.mode, 1)
                self.assertNotIn("speed", [k for k in t.results if k == "speed"])  # J2 result untouched
            finally:
                stop.set()
                arm.close()

    def test_runaway_tracking_error_aborts(self):
        t, arm, stop = make(params={"hold_s": 10, "max_err_deg": 4.0})
        try:
            t.start("hold")
            self.assertTrue(wait_for(lambda: t.state == "hold", 6), t.log)
            arm.q = arm.q + np.radians([0, 12, 0, 0, 0, 0])  # the arm is shoved 12 deg off the hold target
            arm._stop.set()  # and the plant stops fighting back
            self.assertTrue(wait_for(lambda: not t.active(), 6))
            self.assertIn("跟随误差", t.error)
            self.assertEqual(arm.mode, 1)
        finally:
            stop.set()


if __name__ == "__main__":
    unittest.main()
