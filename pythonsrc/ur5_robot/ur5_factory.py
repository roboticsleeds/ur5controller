from ur5_robot import UR5_Robot

class UR5_Factory:
    def __init__(self):
        self._available_grippers = ["robotiq_two_finger",
                                    "robotiq_three_finger"]

        self._available_viewers = ["qtcoin",
                                   "rviz"]

    def initialize_ur5_with_configuration(self,
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

        if gripper_name == "robotiq_two_finger" and has_ridgeback and has_force_torque_sensor:
            file = "clearpath_ridgeback__ur5__robotiq_two_finger_gripper__robotiq_fts150"
        if gripper_name == "robotiq_two_finger" and has_ridgeback and not has_force_torque_sensor:
            file = "clearpath_ridgeback__ur5__robotiq_two_finger_gripper"
        if gripper_name == "robotiq_three_finger" and has_ridgeback and has_force_torque_sensor:
            file = "clearpath_ridgeback__ur5__robotiq_three_finger_gripper__robotiq_fts150"
        if gripper_name == "robotiq_three_finger" and has_ridgeback and not has_force_torque_sensor:
            file = "clearpath_ridgeback__ur5__robotiq_three_finger_gripper"

        urdf_path = urdf_path + "{}.urdf".format(file)
        srdf_path = srdf_path + "{}.srdf".format(file)

        RaveInitialize(True)
        env = Environment()

        if env_path is not None:
            if not env.Load(env_path):
                raise ValueError(
                    'Unable to load environment from path {:s}'.format(env_path))

        urdf_module = RaveCreateModule(env, 'urdf')
        if urdf_module is None:
            print 'Unable to load or_urdf module. Do you have or_urdf' \
                  ' built and installed in one of your Catkin workspaces?'
            raise ValueError('Unable to load or_urdf plugin.')

        ur5_name = urdf_module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
        if ur5_name is None:
            raise ValueError('Failed loading HERB model using or_urdf.')

        robot = env.GetRobot(ur5_name)

        if robot is None:
            raise ValueError('Unable to find robot with name "{:s}".'.format(
                             ur5_name))

        robot.__class__ = UR5_Robot
        robot.__init__(is_simulation)

        # Required for or_rviz to work with the robot's interactive marker.
        ik_solver = RaveCreateIkSolver(env, robot.ikmodel.getikname())
        robot.manipulator.SetIkSolver(ik_solver)

        env.SetViewer(viewer_name)
        if env.GetViewer() is None:
            raise Exception("There was something wrong when loading the {} viewer.".format(viewer_name))

        return env, robot
