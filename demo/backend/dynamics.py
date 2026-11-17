"""Inverse dynamics of the identified 6-axis PiPER model (MuJoCo CPU RNEA, no contact / no simulation).

Same model construction as the console's dynamics.py (identified inertial parameters + viscous damping from
piper_identified.urdf), extended with the inertial term M(q)*ddq so a planned trajectory can be fed forward:

    tau = M(q) ddq + C(q,dq) dq + g(q) + B dq          [physical N*m]

Physical torque -> V3 wire command (see gravity.py): wire = tau / c, c = 1 for J1..J3, 0.813252 for J4..J6.
"""
import math
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np

from .gravity import MODEL_FILE, WIRE_LIMIT, WIRE_SCALES


def _rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1.0]])
    Ry = np.array([[cp, 0, sp], [0, 1.0, 0], [-sp, 0, cp]])
    Rx = np.array([[1.0, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _quat_wxyz(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    return np.array(q) / np.linalg.norm(q)


def _nums(v):
    return " ".join(format(float(x), ".17g") for x in v)


def _origin(el):
    if el is None:
        return np.zeros(3), np.eye(3)
    xyz = np.fromstring(el.get("xyz", "0 0 0"), sep=" ")
    rpy = np.fromstring(el.get("rpy", "0 0 0"), sep=" ")
    return xyz, _rpy_to_R(*rpy)


class Dynamics:
    def __init__(self, path=MODEL_FILE, gravity=(0.0, 0.0, -9.81)):
        import mujoco  # imported lazily: only the speed test needs it

        self.mj = mujoco
        urdf = ET.fromstring(Path(path).read_bytes())
        xml = ET.Element("mujoco", model="piper_identified_dynamics")
        ET.SubElement(xml, "compiler", angle="radian", inertiafromgeom="false", balanceinertia="false", fusestatic="false")
        ET.SubElement(xml, "option", gravity=_nums(gravity))
        parent = ET.SubElement(xml, "worldbody")
        self.damping = []
        for i in range(1, 7):
            joint = urdf.find(f"joint[@name='joint{i}']")
            child = joint.find("child").get("link")
            xyz, R = _origin(joint.find("origin"))
            body = ET.SubElement(parent, "body", name=child, pos=_nums(xyz), quat=_nums(_quat_wxyz(R)))
            ET.SubElement(body, "joint", name=f"joint{i}", type="hinge", axis=_nums(np.fromstring(joint.find("axis").get("xyz"), sep=" ")),
                          damping="0", frictionloss="0", limited="false")
            inertial = urdf.find(f"link[@name='{child}']/inertial")
            com, Ri = _origin(inertial.find("origin"))
            v = inertial.find("inertia").attrib
            T = np.array([[float(v["ixx"]), float(v["ixy"]), float(v["ixz"])],
                          [float(v["ixy"]), float(v["iyy"]), float(v["iyz"])],
                          [float(v["ixz"]), float(v["iyz"]), float(v["izz"])]])
            T = Ri @ T @ Ri.T
            ET.SubElement(body, "inertial", pos=_nums(com), mass=str(float(inertial.find("mass").get("value"))),
                          fullinertia=_nums(T[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]]))
            self.damping.append(float(joint.find("dynamics").get("damping")))
            parent = body
        self.damping = np.asarray(self.damping)
        self.model = mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
        if self.model.nq != 6 or self.model.nv != 6:
            raise ValueError("unexpected dynamics model")
        self.data = mujoco.MjData(self.model)

    def inverse(self, q, dq, ddq):
        """Physical joint torques (N*m) that realise (q, dq, ddq), including viscous friction."""
        d = self.data
        d.qpos[:], d.qvel[:], d.qacc[:] = q, dq, ddq
        self.mj.mj_inverse(self.model, d)
        return d.qfrc_inverse.copy() + self.damping * np.asarray(dq)

    def gravity_nm(self, q):
        return self.inverse(q, np.zeros(6), np.zeros(6))

    def mass_matrix(self, q):
        """M(q) from inverse dynamics (column i = tau(q, 0, e_i) - tau(q, 0, 0)); independent of MuJoCo API versions."""
        zero = np.zeros(6)
        g = self.inverse(q, zero, zero)
        M = np.zeros((6, 6))
        for i in range(6):
            e = np.zeros(6)
            e[i] = 1.0
            M[:, i] = self.inverse(q, zero, e) - g
        return M


def to_wire(tau_nm, residual=None):
    res = np.zeros(6) if residual is None else np.asarray(residual, float)
    return np.clip(np.asarray(tau_nm) / np.asarray(WIRE_SCALES) + res, -WIRE_LIMIT, WIRE_LIMIT)
