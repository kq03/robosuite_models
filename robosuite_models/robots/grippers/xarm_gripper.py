import numpy as np

from robosuite.models.grippers.gripper_model import GripperModel
from robosuite.utils.mjcf_utils import xml_path_completion


class XArmGripperBase(GripperModel):

    def __init__(self, idn=0):
        super().__init__(xml_path_completion("grippers/xarm_gripper.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        """
        Initial joint positions for the gripper (two fingers).
        Adjust these values based on xarm_gripper.xml or physical gripper specs.
        Positive for left, negative for right assumes outward opening.
        """
        return np.array([0.85, 0.85, 0.85, 0.85, 0.85, 0.85])

    @property
    def _important_geoms(self):
        return {
            "left_finger": ["left_finger_collision", "left_inner_knuckle_collision", "left_outer_knuckle_collision"],
            "right_finger": ["right_finger_collision", "right_inner_knuckle_collision", "right_outer_knuckle_collision"],
            "left_fingerpad": ["left_finger_collision"],
            "right_fingerpad": ["right_finger_collision"],
        }


class XArmGripper(XArmGripperBase):

    def format_action(self, action):
        """
        Maps a single continuous action into position setpoints for two fingers.
        -1 => open, 1 => closed
        """
        assert len(action) == self.dof, f"Expected action of length {self.dof}, got {len(action)}"
        
        finger_pos = (1 - action[0]) * 0.425
        
        return np.array([finger_pos, finger_pos])

    @property
    def speed(self):
        return 0.2

    @property
    def dof(self):
        return 1