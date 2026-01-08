"""
Linear interpolation utility
"""

from typing import List


def linear_interpolate(t: float, ts: List[float], qs: List[List[float]]) -> List[float]:
    """
    Linear interpolation between waypoints

    Args:
        t: Current time
        ts: List of time waypoints
        qs: List of position waypoints (each is a list of joint positions)

    Returns:
        Interpolated joint positions
    """
    if t <= ts[0]:
        return qs[0].copy()

    if t >= ts[-1]:
        return qs[-1].copy()

    # Find the segment
    for i in range(len(ts) - 1):
        if ts[i] <= t <= ts[i + 1]:
            # Interpolate
            alpha = (t - ts[i]) / (ts[i + 1] - ts[i])
            result = []
            for j in range(len(qs[i])):
                val = (1.0 - alpha) * qs[i][j] + alpha * qs[i + 1][j]
                result.append(val)
            return result

    return qs[-1].copy()
