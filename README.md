# Unitree Go2 Deploy

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.0.0-blue.svg)](https://docs.omniverse.nvidia.com/isaacsim/latest/overview.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.2.0-green)](https://isaac-sim.github.io/IsaacLab)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg)](https://opensource.org/license/apache-2-0)


## Overview

Sim2Sim & Sim2Real for Unitree Go2, designed for velovity tracking policy trained in IsaacLab.

## Setup

```bash
# Install dependencies
sudo apt install -y libyaml-cpp-dev libboost-all-dev libeigen3-dev libspdlog-dev libfmt-dev
# Install unitree_sdk2
git clone git@github.com:unitreerobotics/unitree_sdk2.git
cd unitree_sdk2
mkdir build && cd build
cmake .. -DBUILD_EXAMPLES=OFF # Install on the /usr/local directory
sudo make install
# Compile the robot_controller
cd go2_isaaclab_deploy/go2 # or other robots
mkdir build && cd build
cmake .. && make
```

### Sim2Sim

Installing the [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco?tab=readme-ov-file#installation).

- Set the `robot` at `/simulate/config.yaml` to go2
- Set `domain_id` to 0
- Set `enable_elastic_hand` to 0, set to 1 for humanoid
- Set `use_joystck` to 1.

```bash
# start simulation
cd unitree_mujoco/simulate/build
./unitree_mujoco
# ./unitree_mujoco -i 0 -n eth0 -r g1 -s scene_29dof.xml # alternative
```

```bash
cd go2_isaaclab_deploy/go2/build
./go2_ctrl --network lo
# 1. press [L2 + Down] to set the robot to stand up
# 2. Click the mujoco window, and then press 8 to make the robot feet touch the ground.
# 3. Press [start] to run the policy.
# 4. Click the mujoco window, and then press 9 to disable the elastic band.
```

### Sim2Real

You can use this program to control the robot directly, but make sure the on-borad control program has been closed.

```bash
./go2_ctrl --network eth0 # eth0 is the network interface name.
```

## Acknowledgements

This repository is adapted from [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab.git)
