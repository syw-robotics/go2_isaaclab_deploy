"""Termination conditions for RL"""
import numpy as np


def bad_orientation(env, limit_angle: float = 1.0) -> bool:
    """
    Check if robot orientation is bad (tilted too much)

    Args:
        env: Environment instance
        limit_angle: Limit angle in radians

    Returns:
        True if orientation is bad
    """
    projected_gravity = env.robot.data.projected_gravity_b
    # Clamp to [-1, 1] to handle numerical errors
    value = np.clip(-projected_gravity[2], -1.0, 1.0)
    angle = np.abs(np.arccos(value))
    is_bad = angle > limit_angle
    if is_bad:
        print(f"[TERMINATION] Bad orientation detected! projected_gravity: {projected_gravity}, angle: {angle:.3f} rad ({np.degrees(angle):.1f} deg), limit: {limit_angle:.3f} rad")
    return is_bad
