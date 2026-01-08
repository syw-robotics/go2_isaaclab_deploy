"""
Control FSM (Finite State Machine)
Manages state transitions and execution
"""

import time
import threading
from typing import List, Optional
from fsm.base_state import BaseState
from fsm.fsm_state import FSMMode


class CtrlFSM:
    """Control Finite State Machine"""

    def __init__(self, init_state: BaseState):
        """
        Initialize FSM

        Args:
            init_state: Initial state to start with
        """
        self.states: List[BaseState] = []
        self.current_state: Optional[BaseState] = None
        self.dt = 0.001  # 1kHz control loop
        self.running = False
        self.fsm_thread: Optional[threading.Thread] = None

        # Add initial state
        self.states.append(init_state)
        self.current_state = init_state
        self.current_state.enter()

        # Start FSM thread
        self.running = True
        self.fsm_thread = threading.Thread(target=self._run_loop, daemon=True)
        self.fsm_thread.start()

        print(f"FSM: Start {self.current_state.get_state_string()}")

    def add(self, state: BaseState):
        """
        Add a new state to FSM

        Args:
            state: State to add
        """
        # Check if state already exists
        for s in self.states:
            if s.is_state(state.get_state()):
                raise ValueError(f"FSM: State_{state.get_state_string()} already exists")

        self.states.append(state)

    def _run_loop(self):
        """Main FSM loop running at specified frequency"""
        while self.running:
            start_time = time.time()

            # Execute current state
            self.current_state.pre_run()
            self.current_state.run()
            self.current_state.post_run()

            # Check for state transitions
            next_state_mode = FSMMode.INVALID
            for check_func, target_mode in self.current_state.registered_checks:
                if check_func():
                    next_state_mode = target_mode
                    break

            # Perform state transition if needed
            if next_state_mode != FSMMode.INVALID and not self.current_state.is_state(next_state_mode):
                for state in self.states:
                    if state.is_state(next_state_mode):
                        print(f"FSM: Change state from {self.current_state.get_state_string()} to {state.get_state_string()}")
                        self.current_state.exit()
                        self.current_state = state
                        self.current_state.enter()
                        break

            # Sleep to maintain control frequency
            elapsed = time.time() - start_time
            sleep_time = self.dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def shutdown(self):
        """Shutdown FSM"""
        self.running = False
        if self.fsm_thread and self.fsm_thread.is_alive():
            self.fsm_thread.join(timeout=2.0)
        if self.current_state:
            self.current_state.exit()
