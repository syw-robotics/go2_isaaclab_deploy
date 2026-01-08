"""
FixStand State - Move robot to standing position
"""

import time
import yaml
from pathlib import Path
from fsm.base_state import BaseState
from fsm.fsm_state import FSMMode, FSMState
from utils import linear_interpolate


class StateFixStand(BaseState):
    """FixStand state - interpolate to standing position"""

    def __init__(self, state_mode: FSMMode):
        """Initialize FixStand state"""
        super().__init__(state_mode, "FixStand")

        # Load configuration
        config_path = Path(__file__).parent.parent / "config" / "config.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        fixstand_config = config['FSM']['FixStand']
        self.kp = fixstand_config.get('kp', [60.0] * 12)
        self.kd = fixstand_config.get('kd', [3.0] * 12)
        self.ts = fixstand_config.get('ts', [0, 1, 2])
        self.qs = fixstand_config.get('qs', [[], [0.0] * 12, [0.0] * 12])

        self.t0 = 0.0

        # Add L1 button protection to enter passive mode
        self.registered_checks.append(
            (lambda: FSMState.lowstate and FSMState.lowstate.joystick and
             FSMState.lowstate.joystick.L1, FSMMode.PASSIVE)
        )

    def enter(self):
        """Enter FixStand state"""
        if FSMState.lowcmd is None or FSMState.lowstate is None:
            return

        # Set gains
        for i in range(len(self.kp)):
            motor = FSMState.lowcmd.msg_.motor_cmd[i]
            motor.kp = self.kp[i]
            motor.kd = self.kd[i]
            motor.dq = 0.0
            motor.tau = 0.0

        # Set initial position (current position)
        q0 = []
        for i in range(12):
            q0.append(FSMState.lowstate.motor_state[i].q)
        self.qs[0] = q0

        # Record start time
        self.t0 = time.time()

    def run(self):
        """Run FixStand state - interpolate to target position"""
        if FSMState.lowcmd is None:
            return

        t = time.time() - self.t0
        q = linear_interpolate(t, self.ts, self.qs)

        for i in range(len(q)):
            FSMState.lowcmd.msg_.motor_cmd[i].q = q[i]
