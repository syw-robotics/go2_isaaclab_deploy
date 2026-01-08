"""Observation functions"""
import numpy as np
from isaaclab.manager import ObservationManager


@ObservationManager.register_observation('base_ang_vel')
def base_ang_vel(env):
    """Base angular velocity"""
    data = env.robot.data.root_ang_vel_b
    return data.tolist()


@ObservationManager.register_observation('projected_gravity')
def projected_gravity(env):
    """Projected gravity"""
    data = env.robot.data.projected_gravity_b
    return data.tolist()


@ObservationManager.register_observation('joint_pos')
def joint_pos(env):
    """Joint positions"""
    cfg = env.cfg.get('observations', {}).get('joint_pos', {})
    params = cfg.get('params', {})
    asset_cfg = params.get('asset_cfg', {})
    joint_ids = asset_cfg.get('joint_ids')

    if joint_ids:
        data = [env.robot.data.joint_pos[i] for i in joint_ids]
    else:
        data = env.robot.data.joint_pos.tolist()

    return data


@ObservationManager.register_observation('joint_pos_rel')
def joint_pos_rel(env):
    """Joint positions relative to default"""
    cfg = env.cfg.get('observations', {}).get('joint_pos_rel', {})
    params = cfg.get('params', {})
    asset_cfg = params.get('asset_cfg', {})
    joint_ids = asset_cfg.get('joint_ids')

    if joint_ids:
        data = [
            env.robot.data.joint_pos[i] - env.robot.data.default_joint_pos[i]
            for i in joint_ids
        ]
    else:
        data = (env.robot.data.joint_pos - env.robot.data.default_joint_pos).tolist()

    return data


@ObservationManager.register_observation('joint_vel_rel')
def joint_vel_rel(env):
    """Joint velocities"""
    data = env.robot.data.joint_vel.tolist()
    return data


@ObservationManager.register_observation('last_action')
def last_action(env):
    """Last action"""
    data = env.action_manager.action()
    return data


@ObservationManager.register_observation('velocity_commands')
def velocity_commands(env):
    """Velocity commands from joystick"""
    robot_data = env.robot.data
    cfg = env.cfg.get('commands', {}).get('base_velocity', {}).get('ranges', {})

    # Use filtered joystick values if available
    if (robot_data.filtered_lin_vel_x != 0.0 or
        robot_data.filtered_lin_vel_y != 0.0 or
        robot_data.filtered_ang_vel_z != 0.0):
        obs = [
            np.clip(robot_data.filtered_lin_vel_x, cfg['lin_vel_x'][0], cfg['lin_vel_x'][1]),
            np.clip(robot_data.filtered_lin_vel_y, cfg['lin_vel_y'][0], cfg['lin_vel_y'][1]),
            np.clip(robot_data.filtered_ang_vel_z, cfg['ang_vel_z'][0], cfg['ang_vel_z'][1]),
        ]
    else:
        # Fallback to raw joystick values
        joystick = robot_data.joystick
        obs = [
            np.clip(joystick.ly(), cfg['lin_vel_x'][0], cfg['lin_vel_x'][1]),
            np.clip(-joystick.lx(), cfg['lin_vel_y'][0], cfg['lin_vel_y'][1]),
            np.clip(-joystick.rx(), cfg['ang_vel_z'][0], cfg['ang_vel_z'][1]),
        ]

    return obs


@ObservationManager.register_observation('gait_phase')
def gait_phase(env):
    """Gait phase (sinusoidal)"""
    cfg = env.cfg.get('observations', {}).get('gait_phase', {})
    params = cfg.get('params', {})
    period = params.get('period', 1.0)

    delta_phase = env.step_dt * (1.0 / period)
    env.global_phase += delta_phase
    env.global_phase = env.global_phase % 1.0

    obs = [
        np.sin(env.global_phase * 2 * np.pi),
        np.cos(env.global_phase * 2 * np.pi),
    ]
    return obs
