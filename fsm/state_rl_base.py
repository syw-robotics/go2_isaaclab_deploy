"""
RL Base State - Runs RL policy
"""

import time
import threading
import yaml
import numpy as np
from pathlib import Path
from fsm.base_state import BaseState
from fsm.fsm_state import FSMMode, FSMState
from isaaclab.envs import ManagerBasedRLEnv
from isaaclab.assets import UnitreeArticulation
from isaaclab.algorithms import OrtRunner
from isaaclab import terminations
import isaaclab.observations  # Import to register observations


class StateRLBase(BaseState):
    """RL Base state - runs reinforcement learning policy"""

    def __init__(self, state_mode: FSMMode, state_string: str, policy_dir: str = "", policy_name: str = ""):
        """Initialize RL Base state"""
        super().__init__(state_mode, state_string)

        print(f"Initializing State_{state_string}...")

        # Load configuration
        config_path = Path(__file__).parent.parent / "config" / "config.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        cfg = config['FSM'][state_string]

        # Load joystick filter rate
        self.joystick_rate = cfg.get('joystick_rate', 0.5)
        print(f"Joystick filter rate: {self.joystick_rate}")

        # Initialize filtered velocity values
        self.filtered_lin_vel_x = 0.0
        self.filtered_lin_vel_y = 0.0
        self.filtered_ang_vel_z = 0.0

        # Use command line parameters if provided, otherwise use config
        if not policy_dir:
            policy_dir = cfg.get('policy_dir', '')
        if not policy_name:
            policy_name = cfg.get('policy_name', '')

        proj_dir = Path(__file__).parent.parent
        policy_yaml_path = proj_dir / "config" / policy_dir / "policy.yaml"
        policy_path = proj_dir / "config" / policy_dir / policy_name

        print(f"Loading policy.yaml from: {policy_yaml_path}")
        print(f"Loading policy onnx from: {policy_path}")

        # Load policy configuration
        with open(policy_yaml_path, 'r') as f:
            policy_cfg = yaml.safe_load(f)

        # Create environment
        self.env = ManagerBasedRLEnv(
            policy_cfg,
            UnitreeArticulation(FSMState.lowstate)
        )

        # Load policy
        self.env.alg = OrtRunner(str(policy_path))

        # Add termination checks
        self.registered_checks.append(
            (lambda: terminations.bad_orientation(self.env, 1.0), FSMMode.PASSIVE)
        )

        # Add L1 button protection
        self.registered_checks.append(
            (lambda: FSMState.lowstate and FSMState.lowstate.joystick and
             FSMState.lowstate.joystick.L1, FSMMode.PASSIVE)
        )

        # Policy thread
        self.policy_thread = None
        self.policy_thread_running = False

    def enter(self):
        """Enter RL Base state"""
        if FSMState.lowcmd is None or FSMState.lowstate is None:
            return

        # Set gains
        for i in range(len(self.env.robot.data.joint_stiffness)):
            FSMState.lowcmd.msg_.motor_cmd[i].kp = self.env.robot.data.joint_stiffness[i]
            FSMState.lowcmd.msg_.motor_cmd[i].kd = self.env.robot.data.joint_damping[i]
            FSMState.lowcmd.msg_.motor_cmd[i].dq = 0.0
            FSMState.lowcmd.msg_.motor_cmd[i].tau = 0.0

        self.env.robot.update()

        # Start policy thread
        self.policy_thread_running = True
        self.policy_thread = threading.Thread(target=self._policy_loop, daemon=True)
        self.policy_thread.start()

    def _policy_loop(self):
        """Policy execution loop"""
        dt = self.env.step_dt
        self.env.reset()

        while self.policy_thread_running:
            start_time = time.time()

            self.env.step()

            # Sleep to maintain frequency
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def run(self):
        """Run RL Base state"""
        if FSMState.lowcmd is None or FSMState.lowstate is None:
            return

        # Get actions and apply to motors
        action = self.env.action_manager.processed_actions()
        for i, joint_id in enumerate(self.env.robot.data.joint_ids_map):
            FSMState.lowcmd.msg_.motor_cmd[int(joint_id)].q = action[i]

        # Update filtered velocities with first-order filter
        cfg = self.env.cfg.get('commands', {}).get('base_velocity', {}).get('ranges', {})
        joystick = self.env.robot.data.joystick

        if joystick:
            raw_lin_vel_x = np.clip(joystick.ly(), cfg['lin_vel_x'][0], cfg['lin_vel_x'][1])
            raw_lin_vel_y = np.clip(-joystick.lx(), cfg['lin_vel_y'][0], cfg['lin_vel_y'][1])
            raw_ang_vel_z = np.clip(-joystick.rx(), cfg['ang_vel_z'][0], cfg['ang_vel_z'][1])

            # First-order filter: filtered = filtered + rate * (raw - filtered)
            self.filtered_lin_vel_x += self.joystick_rate * (raw_lin_vel_x - self.filtered_lin_vel_x)
            self.filtered_lin_vel_y += self.joystick_rate * (raw_lin_vel_y - self.filtered_lin_vel_y)
            self.filtered_ang_vel_z += self.joystick_rate * (raw_ang_vel_z - self.filtered_ang_vel_z)

            # Store in robot data for observations
            self.env.robot.data.filtered_lin_vel_x = self.filtered_lin_vel_x
            self.env.robot.data.filtered_lin_vel_y = self.filtered_lin_vel_y
            self.env.robot.data.filtered_ang_vel_z = self.filtered_ang_vel_z

    def exit(self):
        """Exit RL Base state"""
        self.policy_thread_running = False
        if self.policy_thread and self.policy_thread.is_alive():
            self.policy_thread.join(timeout=2.0)
