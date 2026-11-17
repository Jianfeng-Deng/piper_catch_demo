"""Four-arm registry resolved by USB serial, plus read-only feedback sessions.

Names of SocketCAN interfaces are NOT stable (they were re-shuffled between boots), so every arm is
found by its USB-CAN adapter serial. A reader opens the SDK with piper_init=False, which sends no
frame at all; it only parses what is already on the bus.
"""
import subprocess
import threading
import time
from pathlib import Path

ARMS = {
    "left_follower": {"serial": "004C002D4648571620303731", "role": "follower", "side": "left", "label": "左从臂"},
    "right_follower": {"serial": "001C00284648571720303731", "role": "follower", "side": "right", "label": "右从臂"},
    "left_master": {"serial": "0022003D4648570C20303731", "role": "master", "side": "left", "label": "左主臂"},
    "right_master": {"serial": "002E004B4648570220393831", "role": "master", "side": "right", "label": "右主臂"},
}
MODE_FEED = {0: "MOVE P", 1: "MOVE J", 2: "MOVE L", 3: "MOVE C", 4: "MOVE M", 5: "MOVE CPV", 6: "MIT"}
CTRL_MODE = {0: "待机", 1: "CAN 控制", 2: "示教"}
ARM_STATUS = {0: "正常", 1: "急停", 2: "无解", 3: "奇异点", 4: "目标角超限", 5: "关节通信异常", 6: "抱闸未打开",
              7: "碰撞", 8: "拖动超速", 9: "关节状态异常", 10: "其它异常", 11: "示教记录", 12: "示教执行",
              13: "示教暂停", 14: "主控过温", 15: "释放电阻过温"}


def scan_can():
    """{usb_serial: ifname} for every SocketCAN adapter currently present."""
    found = {}
    for p in sorted(Path("/sys/class/net").glob("can*")):
        try:
            out = subprocess.run(["udevadm", "info", "-q", "property", "-p", str(p)],
                                 capture_output=True, text=True, timeout=5).stdout
        except Exception:  # noqa: BLE001
            continue
        for line in out.splitlines():
            if line.startswith("ID_SERIAL_SHORT="):
                found[line.split("=", 1)[1]] = p.name
    return found


def _age(ts, now):
    return None if not ts else max(0.0, now - ts)


class ArmReader:
    def __init__(self, name):
        self.name = name
        self.meta = ARMS[name]
        self.can = None
        self.piper = None
        self.error = None
        self.tx_lock = threading.RLock()  # the ONLY lock any sender must hold when using self.piper

    def connect(self, ports=None):
        ports = ports if ports is not None else scan_can()
        can = ports.get(self.meta["serial"])
        if can is None:
            self.error = "USB-CAN 适配器未找到"
            return False
        try:
            # V3 interface, like the console: it accepts move_mode=6 (MIT) and the +/-16 N*m wire range.
            # (V2 only allows 0..5 and +/-8; it must not be used for MIT.) V3 extends V2, so every getter is the same.
            from piper_sdk import C_PiperInterface_V3

            self.piper = C_PiperInterface_V3(can_name=can, judge_flag=False, can_auto_init=True, dh_is_offset=1)
            self.piper.ConnectPort(piper_init=False)  # no TX at all
            self.can = can
            self.error = None
            return True
        except Exception as exc:  # noqa: BLE001
            self.error = f"{type(exc).__name__}: {exc}"
            return False

    def snapshot(self):
        """Raw decoded feedback. No derived physics here (see app.py)."""
        now = time.time()
        s = {"name": self.name, "label": self.meta["label"], "role": self.meta["role"], "side": self.meta["side"],
             "can": self.can, "connected": self.piper is not None, "error": self.error, "present": False}
        p = self.piper
        if p is None:
            return s
        j = p.GetArmJointMsgs()
        js = j.joint_state
        q_deg = [js.joint_1 * 1e-3, js.joint_2 * 1e-3, js.joint_3 * 1e-3, js.joint_4 * 1e-3, js.joint_5 * 1e-3, js.joint_6 * 1e-3]
        g = p.GetArmGripperMsgs()
        hs = p.GetArmHighSpdInfoMsgs()
        ls = p.GetArmLowSpdInfoMsgs()
        st = p.GetArmStatus()
        sa = st.arm_status
        speed, cur, eff, vol, ft, mt, bus, en, flags = [], [], [], [], [], [], [], [], []
        for i in range(1, 7):
            h = getattr(hs, f"motor_{i}")
            l = getattr(ls, f"motor_{i}")
            speed.append(h.motor_speed * 1e-3)
            cur.append(h.current * 1e-3)
            eff.append(h.effort * 1e-3)
            vol.append(l.vol * 0.1)
            ft.append(l.foc_temp)
            mt.append(l.motor_temp)
            bus.append(l.bus_current * 1e-3)
            f = l.foc_status
            en.append(bool(f.driver_enable_status))
            flags.append([k for k in ("voltage_too_low", "motor_overheating", "driver_overcurrent", "driver_overheating",
                                      "collision_status", "driver_error_status", "stall_status") if getattr(f, k, False)])
        age_j = _age(j.time_stamp, now)
        s.update(
            present=age_j is not None and age_j < 0.5,
            age_ms={"joint": None if age_j is None else age_j * 1e3, "highspd": None if _age(hs.time_stamp, now) is None else _age(hs.time_stamp, now) * 1e3,
                    "status": None if _age(st.time_stamp, now) is None else _age(st.time_stamp, now) * 1e3},
            hz={"joint": j.Hz, "highspd": hs.Hz, "lowspd": ls.Hz},
            q_deg=q_deg, speed_rad_s=speed, current_a=cur, effort_nm=eff, voltage_v=vol,
            driver_temp_c=ft, motor_temp_c=mt, bus_current_a=bus, enabled=en, motor_flags=flags,
            gripper_mm=g.gripper_state.grippers_angle * 1e-3, gripper_effort_nm=g.gripper_state.grippers_effort * 1e-3,
            gripper_enabled=bool(g.gripper_state.foc_status.driver_enable_status),
            ctrl_mode=int(sa.ctrl_mode), ctrl_mode_text=CTRL_MODE.get(int(sa.ctrl_mode), str(int(sa.ctrl_mode))),
            arm_status=int(sa.arm_status), arm_status_text=ARM_STATUS.get(int(sa.arm_status), str(int(sa.arm_status))),
            mode_feed=int(sa.mode_feed), mode_feed_text=MODE_FEED.get(int(sa.mode_feed), str(int(sa.mode_feed))),
            motion_reached=int(sa.motion_status) == 0,
        )
        return s
