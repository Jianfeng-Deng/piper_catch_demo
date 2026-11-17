"""Terminal fallback for the dashboard's "急停保持": leave MIT and hold the arm where it is (enabled).

    python hold.py --arm left        # or: right, both

Independent of the web server. Sends MOVE-J position hold (1% speed) at the current measured pose, a few
times, and prints the firmware mode it ends up in. It never disables the joints.
"""
import argparse
import time

import common as C


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", default="both", choices=["left", "right", "both"])
    a = ap.parse_args()
    for arm in (["left", "right"] if a.arm == "both" else [a.arm]):
        p = C.connect(arm)
        for _ in range(5):
            q, _ = C.get_q(p)
            p.MotionCtrl_2(ctrl_mode=1, move_mode=1, move_spd_rate_ctrl=1, is_mit_mode=0)
            p.JointCtrl(*[int(round(x * 57295.7795)) for x in C.clamp_q(q, 0.0)])
            time.sleep(0.05)
        st = p.GetArmStatus().arm_status
        print(f"{arm}: mode_feed={int(st.mode_feed)} (1 = MOVE J position hold), arm_status={int(st.arm_status)}")


if __name__ == "__main__":
    main()
