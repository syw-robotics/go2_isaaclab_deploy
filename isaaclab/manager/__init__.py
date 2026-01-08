"""Observation manager and term configuration"""
from collections import deque
from typing import Callable, List, Dict, Optional
import numpy as np


class ObservationTermCfg:
    """Configuration for observation term"""

    def __init__(self):
        self.func: Optional[Callable] = None
        self.clip: List[float] = []
        self.scale: List[float] = []
        self.history_length: int = 1
        self._buff = deque()

    def reset(self, obs: List[float]):
        """Reset observation buffer"""
        self._buff.clear()
        for _ in range(self.history_length):
            self.add(obs)

    def add(self, obs: List[float]):
        """Add observation to buffer"""
        self._buff.append(obs)
        if len(self._buff) > self.history_length:
            self._buff.popleft()

    def get(self) -> List[float]:
        """Get processed observation with history"""
        obs = []
        for obs_i in self._buff:
            obs_scaled = []
            for j, val in enumerate(obs_i):
                # Apply clipping
                if self.clip:
                    val = np.clip(val, self.clip[0], self.clip[1])
                # Apply scaling
                if self.scale:
                    val *= self.scale[j]
                obs_scaled.append(val)
            obs.extend(obs_scaled)
        return obs


class ObservationManager:
    """Manages observation computation"""

    # Registry of observation functions
    _observations_map: Dict[str, Callable] = {}

    @classmethod
    def register_observation(cls, name: str):
        """Decorator to register observation function"""
        def decorator(func):
            cls._observations_map[name] = func
            return func
        return decorator

    def __init__(self, cfg: dict, env):
        """
        Initialize observation manager

        Args:
            cfg: Configuration dictionary
            env: Environment instance
        """
        self.cfg = cfg
        self.env = env
        self.obs_term_cfgs: List[ObservationTermCfg] = []
        self._prepare_terms()

    def _prepare_terms(self):
        """Prepare observation terms from configuration"""
        for term_name, term_cfg in self.cfg.items():
            obs_term = ObservationTermCfg()
            obs_term.history_length = term_cfg.get('history_length', 1)

            # Get observation function
            if term_name not in self._observations_map:
                raise ValueError(f"Observation term '{term_name}' is not registered.")
            obs_term.func = self._observations_map[term_name]

            # Initialize observation buffer
            obs = obs_term.func(self.env)
            obs_term.reset(obs)

            # Set scale and clip
            obs_term.scale = term_cfg.get('scale', [])
            obs_term.clip = term_cfg.get('clip', [])

            self.obs_term_cfgs.append(obs_term)

    def reset(self):
        """Reset all observation terms"""
        for term in self.obs_term_cfgs:
            obs = term.func(self.env)
            term.reset(obs)

    def compute(self) -> List[float]:
        """Compute all observations"""
        obs = []
        for term in self.obs_term_cfgs:
            term.add(term.func(self.env))
            term_obs_scaled = term.get()
            obs.extend(term_obs_scaled)
        return obs
