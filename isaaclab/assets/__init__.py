"""Articulation assets module"""
import numpy as np
from typing import Optional
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


class ArticulationData:
    """Data structure for articulation"""

    def __init__(self):
        self.GRAVITY_VEC_W = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self.FORWARD_VEC_B = np.array([1.0, 0.0, 0.0], dtype=np.float32)

        self.joint_stiffness = []  # sdk order
        self.joint_damping = []  # sdk order

        self.joint_pos = np.array([], dtype=np.float32)
        self.default_joint_pos = np.array([], dtype=np.float32)
        self.joint_vel = np.array([], dtype=np.float32)

        self.root_ang_vel_b = np.zeros(3, dtype=np.float32)
        self.projected_gravity_b = np.zeros(3, dtype=np.float32)
        self.root_quat_w = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)  # w, x, y, z

        self.joint_ids_map = []

        self.joystick = None

        # Filtered joystick values for policy observations
        self.filtered_lin_vel_x = 0.0
        self.filtered_lin_vel_y = 0.0
        self.filtered_ang_vel_z = 0.0

        self.motion_loader = None


class Articulation:
    """Base articulation class"""

    def __init__(self):
        self.data = ArticulationData()

    def update(self):
        """Update articulation state"""
        pass


class UnitreeArticulation(Articulation):
    """Unitree robot articulation"""

    def __init__(self, lowstate):
        super().__init__()
        self.lowstate = lowstate
        self.data.joystick = lowstate.joystick

    def update(self):
        """Update articulation from low state"""
        if self.lowstate is None or self.lowstate.msg_ is None:
            return

        # Base angular velocity
        gyro = self.lowstate.imu_state.gyroscope
        self.data.root_ang_vel_b = np.array([gyro[0], gyro[1], gyro[2]], dtype=np.float32)

        # Quaternion (w, x, y, z)
        quat = self.lowstate.imu_state.quaternion
        self.data.root_quat_w = np.array([quat[0], quat[1], quat[2], quat[3]], dtype=np.float32)

        # Project gravity to body frame using quaternion conjugate rotation
        # C++ version: data.root_quat_w.conjugate() * GRAVITY_VEC_W
        # Conjugate of quaternion (w, x, y, z) is (w, -x, -y, -z)
        qw, qx, qy, qz = quat[0], quat[1], quat[2], quat[3]

        # Use conjugate: qvec_conj = -qvec
        qvec = np.array([-qx, -qy, -qz], dtype=np.float32)
        uv = np.cross(qvec, self.data.GRAVITY_VEC_W)
        uuv = np.cross(qvec, uv)
        self.data.projected_gravity_b = self.data.GRAVITY_VEC_W + 2.0 * (uv * qw + uuv)

        # Joint positions and velocities
        for i, joint_id in enumerate(self.data.joint_ids_map):
            self.data.joint_pos[i] = self.lowstate.motor_state[int(joint_id)].q
            self.data.joint_vel[i] = self.lowstate.motor_state[int(joint_id)].dq
