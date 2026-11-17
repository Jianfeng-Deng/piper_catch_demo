"""Mock tests for the supervised "go to pose" mover (no hardware)."""
import sys
import threading
import time
import unittest
from pathlib import Path

import numpy as np

DEMO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(DEMO))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from backend import posemove as PM  # noqa: E402
from test_teleop_mock import FakeArm, Reader, wait_for  # noqa: E402


class MoveArm(FakeArm):
    """MOVE-J plant: when enabled the joints slew toward the last JointCtrl target at <= 0.4 rad/s."""
    tgt = None

    def JointCtrl(self, *j):
        self.log.append(("JointCtrl", j))
        self.tgt = np.radians(np.array(j, float) * 1e-3)

    def _sim(self):
        last = time.time()
        while not self._stop.is_set():
            time.sleep(0.002)
            now = time.time()
            dt, last = now - last, now
            if self.mode == 1 and self.enabled and self.tgt is not None:
                self.q += np.clip(self.tgt - self.q, -0.4 * dt, 0.4 * dt)


START = [0.0, 28.6, -57.3, 0.0, 28.6, 0.0]
TARGET = [-3.1, 40.0, -70.0, -3.5, 19.7, 39.7]


def make(q=START, params=None):
    arm = MoveArm(q, simulate=True)
    PM.C.detect_foreign_commands = lambda *a, **k: set()
    m = PM.PoseMove(Reader(arm), "left", params=params)
    stop = threading.Event()

    def hb():
        while not stop.is_set():
            m.heartbeat()
            time.sleep(0.1)

    threading.Thread(target=hb, daemon=True).start()
    return m, arm, stop


class PoseMoveTests(unittest.TestCase):
    def test_check_reports_limits_height_and_time(self):
        m, arm, stop = make()
        try:
            c = m.check(TARGET)
            self.assertTrue(c["ok"], c)
            self.assertGreater(c["est_time_s"], 1.0)
            self.assertAlmostEqual(c["max_travel_deg"], 39.7, places=1)
            bad = m.check([0, -5, 0, 0, 0, 0])  # J2 below its limit
            self.assertFalse(bad["ok"])
            self.assertTrue(any("J2" in r for r in bad["reasons"]))
            self.assertFalse(m.check([0, 0, 0])["ok"])                           # wrong length
            m.params["z_min_m"] = 0.9                                              # impossible floor -> path rejected
            self.assertTrue(any("最低 TCP 高度" in r for r in m.check(TARGET)["reasons"]))
        finally:
            stop.set()
            arm.close()

    def test_move_arrives_holds_enabled_and_sets_gripper(self):
        m, arm, stop = make()
        try:
            m.start(TARGET, gripper_mm=60.0)
            self.assertTrue(wait_for(lambda: not m.active(), 30), m.log)
            self.assertEqual(m.state, "idle", m.error)
            self.assertLess(np.degrees(np.abs(arm.q - np.radians(TARGET))).max(), 1.0)
            self.assertTrue(arm.enabled)
            self.assertEqual(arm.mode, 1)
            self.assertFalse(any(e[0] == "JointMitCtrl" for e in arm.log))        # position mode only, never MIT
            self.assertTrue(any(e[0] == "GripperCtrl" and abs(e[1] - 60000) < 1 for e in arm.log))
            rates = {e[3] for e in arm.log if e[0] == "MotionCtrl_2"}
            self.assertTrue(rates <= {5, 10}, rates)                                # only slow speed rates were ever used
        finally:
            stop.set()
            arm.close()

    def test_rejects_unsafe_target_before_any_command(self):
        m, arm, stop = make()
        try:
            with self.assertRaises(ValueError):
                m.start([0, -5, 0, 0, 0, 0])
            self.assertFalse(any(e[0] in ("EnableArm", "JointCtrl") for e in arm.log))
        finally:
            stop.set()
            arm.close()

    def test_stop_holds_where_it_is(self):
        m, arm, stop = make()
        try:
            m.start(TARGET)
            self.assertTrue(wait_for(lambda: m.state == "moving", 5), m.log)
            time.sleep(0.4)
            m.stop()
            self.assertTrue(wait_for(lambda: not m.active(), 5))
            held = np.array(arm.log[-1][1]) * 1e-3
            self.assertGreater(np.degrees(np.abs(arm.q - np.radians(TARGET))).max(), 2.0)  # stopped short of the target
            self.assertLess(np.abs(held - np.degrees(arm.q)).max(), 3.0)                    # and holds near where it is
            self.assertEqual(arm.log[-1][0], "JointCtrl")
        finally:
            stop.set()
            arm.close()

    def test_heartbeat_loss_aborts_and_holds(self):
        m, arm, stop = make(params={"heartbeat_timeout_s": 0.3})
        try:
            m.start(TARGET)
            self.assertTrue(wait_for(lambda: m.state == "moving", 5), m.log)
            stop.set()
            self.assertTrue(wait_for(lambda: not m.active(), 5))
            self.assertIn("心跳", m.error)
            self.assertEqual(m.state, "fault")
        finally:
            arm.close()


if __name__ == "__main__":
    unittest.main()
