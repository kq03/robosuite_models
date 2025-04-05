import numpy as np
from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel
from robosuite.utils.mjcf_utils import xml_path_completion
from robosuite.robots import register_robot_class

from robosuite_models.utils import robosuite_model_path_completion

class XArm7(ManipulatorModel):
    arms = ["right"]

    def __init__(self, idn=0, gripper=None):
        super().__init__(xml_path_completion("robots/xarm7/robot.xml"), idn=idn)
        self._gripper = gripper if gripper else "XArmGripper"

    def load_model(self):
        super().load_model()

    @property
    def default_base(self):
        return "RethinkMount"

    @property
    def default_gripper(self):
        return {"right": self._gripper}

    @property
    def default_controller_config(self):
        return {"right": "default_xarm7"}

    @property
    def init_qpos(self):
        return np.array([0.0, -0.0, 0.0, 0.909, 0.0, 1.15644, 0.0])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array([0, 0, 0])

    @property
    def _horizontal_radius(self):
        return 0.7

    @property
    def arm_type(self):
        return "single"