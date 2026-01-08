"""Action manager and action terms"""
from typing import List, Dict, Optional
import numpy as np


class ActionTerm:
    """Base class for action terms"""

    def __init__(self, cfg: dict, env):
        """Initialize action term"""
        self.cfg = cfg
        self.env = env

    def action_dim(self) -> int:
        """Get action dimension"""
        raise NotImplementedError

    def raw_actions(self) -> List[float]:
        """Get raw actions"""
        raise NotImplementedError

    def processed_actions(self) -> List[float]:
        """Get processed actions"""
        raise NotImplementedError

    def process_actions(self, actions: List[float]):
        """Process actions"""
        raise NotImplementedError

    def reset(self):
        """Reset action term"""
        pass


class JointAction(ActionTerm):
    """Joint action base class"""

    def __init__(self, cfg: dict, env):
        """Initialize joint action"""
        super().__init__(cfg, env)

        if cfg.get('joint_ids') is None:
            self._action_dim = len(env.robot.data.joint_ids_map)
        else:
            self._joint_ids = cfg['joint_ids']
            self._action_dim = len(self._joint_ids)

        self._raw_actions = [0.0] * self._action_dim
        self._processed_actions = [0.0] * self._action_dim
        self._scale = cfg.get('scale', [1.0] * self._action_dim)
        self._offset = cfg.get('offset', [0.0] * self._action_dim)
        self._clip = cfg.get('clip', [])

    def process_actions(self, actions: List[float]):
        """Process actions with scale and offset"""
        self._raw_actions = actions.copy()
        for i in range(self._action_dim):
            self._processed_actions[i] = self._raw_actions[i] * self._scale[i] + self._offset[i]

        # Apply clipping
        if self._clip:
            for i in range(self._action_dim):
                self._processed_actions[i] = np.clip(
                    self._processed_actions[i],
                    self._clip[i][0],
                    self._clip[i][1]
                )

    def action_dim(self) -> int:
        return self._action_dim

    def raw_actions(self) -> List[float]:
        return self._raw_actions.copy()

    def processed_actions(self) -> List[float]:
        return self._processed_actions.copy()

    def reset(self):
        self._raw_actions = [0.0] * self._action_dim


class JointPositionAction(JointAction):
    """Joint position action"""
    pass


class JointVelocityAction(JointAction):
    """Joint velocity action"""
    pass


class ActionManager:
    """Manages action processing"""

    # Registry of action classes
    _actions_map: Dict[str, type] = {
        'JointPositionAction': JointPositionAction,
        'JointVelocityAction': JointVelocityAction,
    }

    def __init__(self, cfg: dict, env):
        """
        Initialize action manager

        Args:
            cfg: Configuration dictionary
            env: Environment instance
        """
        self.cfg = cfg
        self.env = env
        self._terms: List[ActionTerm] = []
        self._prepare_terms()
        self._action = [0.0] * self.total_action_dim()

    def _prepare_terms(self):
        """Prepare action terms from configuration"""
        for action_name, action_cfg in self.cfg.items():
            if action_name not in self._actions_map:
                raise ValueError(f"Action term '{action_name}' is not registered.")

            action_class = self._actions_map[action_name]
            term = action_class(action_cfg, self.env)
            self._terms.append(term)

    def reset(self):
        """Reset all action terms"""
        self._action = [0.0] * self.total_action_dim()
        for term in self._terms:
            term.reset()

    def action(self) -> List[float]:
        """Get current action"""
        return self._action.copy()

    def processed_actions(self) -> List[float]:
        """Get all processed actions"""
        actions = []
        for term in self._terms:
            actions.extend(term.processed_actions())
        return actions

    def process_action(self, action: List[float]):
        """Process action through all terms"""
        self._action = action.copy()
        idx = 0
        for term in self._terms:
            term_action = action[idx:idx + term.action_dim()]
            term.process_actions(term_action)
            idx += term.action_dim()

    def total_action_dim(self) -> int:
        """Get total action dimension"""
        return sum(term.action_dim() for term in self._terms)

    def action_dim(self) -> List[int]:
        """Get action dimensions for each term"""
        return [term.action_dim() for term in self._terms]
