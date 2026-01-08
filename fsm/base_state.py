"""
Base State class
"""

from fsm.fsm_state import FSMState, FSMMode


class BaseState(FSMState):
    """Base state implementation"""

    def __init__(self, state_mode: FSMMode, state_string: str = ""):
        """Initialize base state"""
        super().__init__(state_mode, state_string)
