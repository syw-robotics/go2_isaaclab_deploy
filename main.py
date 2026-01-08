#!/usr/bin/env python3
"""
Go2 Isaac Lab Deploy - Python Implementation
Main entry point for the controller
"""

import sys
import time
import argparse
import yaml
from pathlib import Path

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from lowlevel_wrapper import LowCmdWrapper, LowStateWrapper
from fsm.ctrl_fsm import CtrlFSM
from fsm.state_passive import StatePassive
from fsm.state_fixstand import StateFixStand
from fsm.state_rl_base import StateRLBase
from fsm.fsm_state import FSMMode, FSMState


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(description='Go2 Controller')
    parser.add_argument('--config', type=str, default='config/config.yaml',
                        help='Path to config file')
    parser.add_argument('--policy_dir', type=str, default='',
                        help='Policy directory name')
    parser.add_argument('--policy_name', type=str, default='',
                        help='Policy file name (e.g., policy.onnx)')
    parser.add_argument('--network', type=str, default='eth0',
                        help='Network interface name')
    return parser.parse_args()


def load_config(config_path):
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def init_fsm_state():
    """Initialize FSM state (lowcmd and lowstate)"""
    # Check if another process is using lowcmd channel
    lowcmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
    lowcmd_sub.Init(lambda msg: None, 10)
    time.sleep(0.2)

    # Initialize low command and state
    FSMState.lowcmd = LowCmdWrapper()
    FSMState.lowstate = LowStateWrapper()

    print("Waiting for connection to robot...")
    FSMState.lowstate.wait_for_connection()
    print("Connected to robot.")


def main():
    """Main function"""
    args = parse_arguments()

    print(" --- Unitree Robotics --- ")
    print("     Go2 Controller ")
    print()

    # Load configuration
    config = load_config(args.config)

    # Initialize Unitree SDK2
    ChannelFactoryInitialize(0, args.network)

    # Initialize FSM state
    init_fsm_state()

    # Initialize FSM
    fsm = CtrlFSM(StatePassive(FSMMode.PASSIVE))

    # Add state transition checks for passive state
    # Transition: L2 + A button -> FixStand
    fsm.states[0].registered_checks.append(
        (lambda: FSMState.lowstate and FSMState.lowstate.joystick and
         FSMState.lowstate.joystick.L2 and FSMState.lowstate.joystick.A, FSMMode.FIX_STAND)
    )

    # Add FixStand state
    fix_stand_state = StateFixStand(FSMMode.FIX_STAND)
    # Transition: Start button -> RL
    fix_stand_state.registered_checks.append(
        (lambda: FSMState.lowstate and FSMState.lowstate.joystick and
         FSMState.lowstate.joystick.Start, FSMMode.VELOCITY)
    )
    fsm.add(fix_stand_state)

    # Add RL Base state
    policy_dir = args.policy_dir if args.policy_dir else config['FSM']['Velocity'].get('policy_dir', '')
    policy_name = args.policy_name if args.policy_name else config['FSM']['Velocity'].get('policy_name', '')

    rl_state = StateRLBase(FSMMode.VELOCITY, "Velocity", policy_dir, policy_name)
    fsm.add(rl_state)

    print("\n=====================================")
    print("Press [L2 + A] to enter FixStand mode.")
    print("Then press [Start] to start RL controller.")
    print("=====================================")
    print("Press [L1] to enter Passive mode.")
    print("=====================================\n")

    # Main loop
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
        fsm.shutdown()


if __name__ == '__main__':
    main()
