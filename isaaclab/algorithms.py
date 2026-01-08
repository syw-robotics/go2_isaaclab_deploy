"""Algorithms and policy runners"""
import threading
import numpy as np
import onnxruntime as ort
from typing import List


class Algorithms:
    """Base algorithm class"""

    def __init__(self):
        self.action = []
        self._act_mtx = threading.Lock()

    def act(self, obs: List[float]) -> List[float]:
        """Compute action from observation"""
        raise NotImplementedError

    def get_action(self) -> List[float]:
        """Get current action (thread-safe)"""
        with self._act_mtx:
            return self.action.copy()


class OrtRunner(Algorithms):
    """ONNX Runtime policy runner"""

    def __init__(self, model_path: str):
        """
        Initialize ONNX Runtime runner

        Args:
            model_path: Path to ONNX model file
        """
        super().__init__()

        # Create ONNX Runtime session
        self.session = ort.InferenceSession(
            model_path,
            providers=['CPUExecutionProvider']
        )

        # Get input/output information
        self.input_name = self.session.get_inputs()[0].name
        self.output_name = self.session.get_outputs()[0].name

        input_shape = self.session.get_inputs()[0].shape
        output_shape = self.session.get_outputs()[0].shape

        self.input_dim = input_shape[1] if len(input_shape) > 1 else input_shape[0]
        self.output_dim = output_shape[1] if len(output_shape) > 1 else output_shape[0]

        self.action = [0.0] * self.output_dim

    def act(self, obs: List[float]) -> List[float]:
        """
        Run inference

        Args:
            obs: Observation vector

        Returns:
            Action vector
        """
        # Prepare input
        obs_array = np.array([obs], dtype=np.float32)

        # Run inference
        outputs = self.session.run(
            [self.output_name],
            {self.input_name: obs_array}
        )

        # Get action
        action_array = outputs[0][0]

        with self._act_mtx:
            self.action = action_array.tolist()

        return self.action.copy()
