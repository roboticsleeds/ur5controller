from openravepy import RaveInitialize
from openravepy import Environment
from openravepy import RaveCreateModule
from openravepy import RaveCreateIkSolver
from ur5_robot import UR5_Robot

class UR5_Factory:
    def __init__(self):
        # TODO: Add support for robotiq_three_finger
        self._available_grippers = ["robotiq_two_finger",]
        self._available_viewers = ["qtcoin", "rviz"]

    def create_ur5_and_env(self,
                           is_simulation=True,
                           has_ridgeback=True,
                           gripper_name="robotiq_two_finger",
                           has_force_torque_sensor=True,
                           env_path=None,
                           viewer_name="qtcoin",
                           urdf_path="package://ur5controller/ur5_description/urdf/",
                           srdf_path="package://ur5controller/ur5_description/srdf/"):

        if gripper_name not in self._available_grippers:
            raise ValueError("Gripper {} is not supported. The only available gripper names are: {}".format(gripper_name, ', '.join(self._available_grippers)))

        if viewer_name not in self._available_viewers:
            raise ValueError("Viewer {} is not supported. The only available viewer names are: {}".format(viewer_name, ', '.join(self._available_viewers)))

        # TODO: Create URDFs that do not include the ridgeback.
        if not has_ridgeback:
            raise Exception("Robot configuration without ridgeback is not yet available. Robot configuration without the Clearpath Ridgeback is under development at the moment (please check that you also have the latest version of this code).")

        RaveInitialize(True)
        env = self._create_environment()
        robot = self._load_ur5_from_urdf(env, gripper_name, has_ridgeback, has_force_torque_sensor, urdf_path, srdf_path)

        # Add class UR5_Robot to the robot.
        robot.__class__ = UR5_Robot
        robot.__init__(is_simulation)

        # Required for or_rviz to work with the robot's interactive marker.
        ik_solver = RaveCreateIkSolver(env, robot.ikmodel.getikname())
        robot.manipulator.SetIkSolver(ik_solver)

        self._set_viewer(env, viewer_name)

        return env, robot

    def _get_file_name_from_specification(gripper_name, has_ridgeback, has_force_torque_sensor):
        file_name = None

        if gripper_name == "robotiq_two_finger" and has_ridgeback and has_force_torque_sensor:
            file_name = "clearpath_ridgeback__ur5__robotiq_two_finger_gripper__robotiq_fts150"
        if gripper_name == "robotiq_two_finger" and has_ridgeback and not has_force_torque_sensor:
            file_name = "clearpath_ridgeback__ur5__robotiq_two_finger_gripper"
        if gripper_name == "robotiq_three_finger" and has_ridgeback and has_force_torque_sensor:
            file_name = "clearpath_ridgeback__ur5__robotiq_three_finger_gripper__robotiq_fts150"
        if gripper_name == "robotiq_three_finger" and has_ridgeback and not has_force_torque_sensor:
            file_name = "clearpath_ridgeback__ur5__robotiq_three_finger_gripper"

        return file_name

    def _load_ur5_from_urdf(self, env, gripper_name, has_ridgeback, has_force_torque_sensor, urdf_path, srdf_path):
        file_name = self._get_file_name_from_specification(gripper_name, has_ridgeback, has_force_torque_sensor)
        urdf_path = urdf_path + "{}.urdf".format(file_name)
        srdf_path = srdf_path + "{}.srdf".format(file_name)

        urdf_module = RaveCreateModule(env, 'urdf')
        if urdf_module is None:
            raise Exception("Unable to load or_urdf module. Make sure you have or_urdf installed in your Catkin check: https://github.com/personalrobotics/or_urdf")

        ur5_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
        if ur5_name is None:
            raise Exception("Something went wrong while trying to load UR5 from file. Is the path correct?")

        robot = env.GetRobot(ur5_name)

        if robot is None:
            raise Exception("Unable to find robot with name '{}'.".format(ur5_name))

        return robot

    def _create_environment(self):
        env = Environment()

        if env_path is not None:
            if not env.Load(env_path):
                raise ValueError("Unable to load environment file: '{}'".format(env_path))

        return env

    def _set_viewer(self, env, viewer_name):
        env.SetViewer(viewer_name)
        if env.GetViewer() is None:
            raise Exception("There was something wrong when loading the {} viewer.".format(viewer_name))
