"""
FSM State base class and definitions
"""

from enum import IntEnum
from typing import Optional
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


class FSMMode(IntEnum):
    """FSM Mode enumeration"""
    INVALID = 0
    PASSIVE = 1
    FIX_STAND = 2
    VELOCITY = 3


class FSMState:
    """Base class for FSM states"""

    # Shared state across all FSM states
    lowcmd: Optional[unitree_go_msg_dds__LowCmd_] = None
    lowstate: Optional[LowState_] = None

    def __init__(self, state_mode: FSMMode, state_string: str = ""):
        """
        Initialize FSM state

        Args:
            state_mode: State mode enum
            state_string: String representation of the state
        """
        self.state_mode = state_mode
        self.state_string = state_string if state_string else state_mode.name
        self.registered_checks = []  # List of (check_function, next_state_mode) tuples

    def is_state(self, mode: FSMMode) -> bool:
        """Check if this is a specific state"""
        return self.state_mode == mode

    def get_state(self) -> FSMMode:
        """Get current state mode"""
        return self.state_mode

    def get_state_string(self) -> str:
        """Get state string representation"""
        return self.state_string

    def enter(self):
        """Called when entering this state"""
        pass

    def run(self):
        """Main state execution - called every control cycle"""
        pass

    def exit(self):
        """Called when exiting this state"""
        pass

    def pre_run(self):
        """Called before run() - used for updates"""
        pass

    def post_run(self):
        """Called after run() - used for cleanup"""
        pass
