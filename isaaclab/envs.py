"""Manager-based RL Environment"""
import numpy as np
from typing import Optional
from isaaclab.assets import Articulation
from isaaclab.manager import ObservationManager
from isaaclab.manager.actions import ActionManager


class ManagerBasedRLEnv:
    """Manager-based reinforcement learning environment"""

    def __init__(self, cfg: dict, robot: Articulation):
        """
        Initialize environment

        Args:
            cfg: Configuration dictionary from policy.yaml
            robot: Robot articulation instance
        """
        self.cfg = cfg
        self.robot = robot

        # Parse configuration
        self.step_dt = cfg['step_dt']
        self.robot.data.joint_ids_map = cfg['joint_ids_map']

        num_joints = len(self.robot.data.joint_ids_map)
        self.robot.data.joint_pos = np.zeros(num_joints, dtype=np.float32)
        self.robot.data.joint_vel = np.zeros(num_joints, dtype=np.float32)

        # Default joint positions
        default_joint_pos = cfg.get('default_joint_pos', [0.0] * num_joints)
        self.robot.data.default_joint_pos = np.array(default_joint_pos, dtype=np.float32)

        # Joint stiffness and damping
        self.robot.data.joint_stiffness = cfg.get('stiffness', [40.0] * num_joints)
        self.robot.data.joint_damping = cfg.get('damping', [1.0] * num_joints)

        # Update robot
        self.robot.update()

        # Load managers
        self.action_manager = ActionManager(cfg['actions'], self)
        self.observation_manager = ObservationManager(cfg['observations'], self)

        # Episode tracking
        self.episode_length = 0
        self.global_phase = 0.0

        # Algorithm (policy)
        self.alg: Optional['Algorithms'] = None

    def reset(self):
        """Reset environment"""
        self.global_phase = 0.0
        self.episode_length = 0
        self.robot.update()

        if self.robot.data.motion_loader:
            self.robot.data.motion_loader.reset(self.robot.data)

        self.action_manager.reset()
        self.observation_manager.reset()

    def step(self):
        """Step environment"""
        self.episode_length += 1
        self.robot.update()

        if self.robot.data.motion_loader:
            self.robot.data.motion_loader.update(self.episode_length * self.step_dt)

        obs = self.observation_manager.compute()
        action = self.alg.act(obs)
        self.action_manager.process_action(action)
