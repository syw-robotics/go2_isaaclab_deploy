# Go2 Isaac Lab Deploy - Python

Python implementation of the Go2 Isaac Lab deployment system for running reinforcement learning policies on Unitree Go2 robot.

## Overview

Sim2Sim & Sim2Real for Unitree Go2, designed for velovity tracking policy trained in IsaacLab. This deployment is compatible with both x86 and arm64 platforms.

This is a Python port of deployment code. It provides:
- Finite State Machine (FSM) based control
- RL policy execution using ONNX Runtime
- Low-level motor control
- Real-time joystick input handling
- Observation and action management

## Requirements

- Python 3.8+
- Unitree SDK2 Python
- Dependencies listed in `requirements.txt`

## Installation

### Quick Install (Recommended)

Run the installation script:
```bash
cd go2_isaaclab_deploy_python
./install.sh
```

This will:
- Check Python version (requires 3.8+)
- Install all required dependencies
- Verify installation
- Check for Unitree SDK2 Python

### Manual Install

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Ensure Unitree SDK2 Python is installed and available in your Python path.

### Dependencies

Required packages:
- `numpy >= 1.20.0`
- `pyyaml >= 5.4.0`
- `onnxruntime >= 1.10.0`
- `unitree_sdk2py` (from Unitree SDK2)

## Usage

Basic usage:
```bash
python main.py --network eth0
```

With custom policy:
```bash
python main.py --policy_dir unitree_rl_lab --policy_name policy.onnx

# as a shortcut
./run.sh

```

### Command Line Arguments

- `--policy_dir`: Policy directory name (e.g., `unitree_rl_lab`)
- `--policy_name`: Policy file name (e.g., `policy.onnx`)
- `--network`: Network interface name (default: `eth0`)

## Control

- **[L2 + A]**: Enter FixStand mode (robot stands up)
- **[Start]**: Start RL controller
- **[L1]**: Enter Passive mode (emergency stop)

Use the left and right joysticks to control the robot velocity when in RL mode.

## Project Structure

```
go2_isaaclab_deploy_python/
├── main.py                 # Main entry point
├── fsm/                    # Finite State Machine
│   ├── ctrl_fsm.py        # FSM controller
│   ├── fsm_state.py       # State base classes
│   ├── state_passive.py   # Passive state
│   ├── state_fixstand.py  # Standing state
│   └── state_rl_base.py   # RL controller state
├── isaaclab/              # Isaac Lab environment
│   ├── assets/            # Robot articulation
│   ├── manager/           # Action and observation managers
│   ├── algorithms.py      # ONNX policy runner
│   ├── envs.py           # RL environment
│   ├── observations.py    # Observation functions
│   └── terminations.py    # Safety terminations
├── config/                # Configuration files
│   ├── config.yaml       # Main configuration
│   └── */policy.yaml     # Policy configurations
└── utils.py              # Utility functions
```

## Configuration

The system uses YAML configuration files:

- `config/config.yaml`: FSM configuration, PD gains, trajectory waypoints
- `config/*/policy.yaml`: Policy-specific configuration including observations, actions, and parameters

## Safety

The system includes multiple safety checks:
- Bad orientation detection (robot tipped over)
- Emergency stop via L1 button
- Automatic transition to passive mode on failure

## Comparison with C++ Version

This Python implementation maintains feature parity with the C++ version:
- ✅ FSM-based control flow
- ✅ ONNX policy inference
- ✅ Real-time motor control
- ✅ Joystick command filtering
- ✅ Observation history buffering
- ✅ Action scaling and clipping
- ✅ Safety termination conditions

Performance considerations:
- Python has slightly higher latency than C++ (~1-2ms)
- ONNX Runtime provides efficient inference in both languages
- For most RL policies, the performance difference is negligible

## License

Copyright (c) 2025, Unitree Robotics Co., Ltd.
