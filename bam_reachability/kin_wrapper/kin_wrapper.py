#!/usr/bin/env python3


# PYTHON
from abc import ABC, abstractmethod
from typing import Tuple
import os
import numpy as np


class KinWrapper(ABC):
    
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def IK(self, pose_matrix: np.ndarray)-> Tuple[bool, np.ndarray]:

        assert pose_matrix.shape == (4, 4), f"Expected pose to be a 4x4 matrix, got shape {pose_matrix.shape}"
        return False, np.zeros((6,))

    @abstractmethod
    def FK(self, joint_positions: np.ndarray)-> Tuple[bool, np.ndarray]:
        

        return False, np.eye(4)


