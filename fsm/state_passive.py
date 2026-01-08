"""
Passive State - Robot in damped mode
"""

import yaml
from pathlib import Path
from fsm.base_state import BaseState
from fsm.fsm_state import FSMMode, FSMState


class StatePassive(BaseState):
    """Passive state - robot follows current position with damping"""

    def __init__(self, state_mode: FSMMode):
        """Initialize Passive state"""
        super().__init__(state_mode, "Passive")

        # Load configuration
        config_path = Path(__file__).parent.parent / "config" / "config.yaml"
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        passive_config = config['FSM']['Passive']
        self.motor_mode = passive_config.get('mode', [1] * 12)
        self.kd = passive_config.get('kd', [2.0] * 12)

        # Initialize motor modes
        if FSMState.lowcmd is not None:
            for i in range(len(self.motor_mode)):
                FSMState.lowcmd.msg_.motor_cmd[i].mode = self.motor_mode[i]

    def enter(self):
        """Enter passive state"""
        if FSMState.lowcmd is None:
            return

        # Set gains
        for i in range(len(self.kd)):
            motor = FSMState.lowcmd.msg_.motor_cmd[i]
            motor.kp = 0.0
            motor.kd = self.kd[i]
            motor.dq = 0.0
            motor.tau = 0.0

    def run(self):
        """Run passive state - follow current position"""
        if FSMState.lowcmd is None or FSMState.lowstate is None:
            return

        for i in range(len(FSMState.lowcmd.msg_.motor_cmd)):
            FSMState.lowcmd.msg_.motor_cmd[i].q = FSMState.lowstate.motor_state[i].q
