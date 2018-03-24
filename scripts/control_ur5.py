# Copyright (C) 2017 The University of Leeds
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# Author: Rafael Papallas (http://papallas.me)

from openravepy import *
from numpy import *
import IPython

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

        urdf_module = RaveCreateModule(self.env, 'urdf')
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

        env.SetViewer(viewer_name)
        if env.GetViewer() is None:
            raise Exception("There was something wrong when loading the {} viewer.".format(viewer_name))

        return env, robot

class UR5_Robot(openravepy.Robot):
    def __init__(self, is_simulation):
        self.robot_name = "UR5"

        if not is_simulation:
            self.multicontroller = RaveCreateMultiController(self.env, "")
            self.SetController(self.multicontroller)

            robot_controller = RaveCreateController(self.env, 'ur5controller')
            hand_controller = RaveCreateController(self.env, 'robotiqcontroller')

            self.multicontroller.AttachController(robot_controller, [2, 1, 0, 4, 5, 6], 0)
            self.multicontroller.AttachController(hand_controller, [3], 0)

        self.manipulator = self.SetActiveManipulator(self.GetManipulators()[0])
        self.task_manipulation = interfaces.TaskManipulation(self)
        self.base_manipulation = interfaces.BaseManipulation(self)

        ikmodel = databases.inversekinematics.InverseKinematicsModel(self, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

    @property
    def end_effector_transform(self):
        return self.GetManipulators()[0].GetTransform()




class Ur5Robot:
    def __init__(self, has_two_finger_gripper=True, has_force_torque_sensor=True):
        self.OFFSET = 0.07
        self.env = Environment()
        self.env.Load('environment.xml')
        self.env.SetViewer('qtcoin')

        self._load_robot_from_urdf(has_two_finger_gripper, has_force_torque_sensor)
        # self._attach_controllers_to_robot()

        self.manipulator = self.robot.SetActiveManipulator(self.robot.GetManipulators()[0])
        self.task_manipulation = interfaces.TaskManipulation(self.robot)
        self.base_manipulation = interfaces.BaseManipulation(self.robot)

        ikmodel = databases.inversekinematics.InverseKinematicsModel(self.robot, iktype=IkParameterization.Type.Transform6D)
        if not ikmodel.load():
            ikmodel.autogenerate()

        self.robot.SetTransform(array([[ 9.99945700e-01, -1.04210156e-02, -1.24229304e-09, 3.03276211e-01],
                                       [ 1.04210156e-02,  9.99945700e-01, -6.45894224e-12, -1.31079838e-01],
                                       [ 1.24229289e-09, -6.48736357e-12,  1.00000000e+00, 2.52894759e-02],
                                       [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00, 1.00000000e+00]]))

        # Add the robot to the environment
        self.env.Add(self.robot, True)

    def _load_robot_from_urdf(self, has_two_finger_gripper, has_force_torque_sensor):
        if has_two_finger_gripper and has_force_torque_sensor:
            file = "clearpath_ridgeback__ur5__robotiq_two_finger_gripper__robotiq_fts150"
        if has_two_finger_gripper and not has_force_torque_sensor:
            file = "clearpath_ridgeback__ur5__robotiq_two_finger_gripper"
        if not has_two_finger_gripper and has_force_torque_sensor:
            file = "clearpath_ridgeback__ur5__robotiq_three_finger_gripper__robotiq_fts150"
        if not has_two_finger_gripper and not has_force_torque_sensor:
            file = "clearpath_ridgeback__ur5__robotiq_three_finger_gripper"

        urdf_path = "package://ur5controller/ur5_description/urdf/{}.urdf".format(file)
        srdf_path = "package://ur5controller/ur5_description/srdf/{}.srdf".format(file)

        module = RaveCreateModule(self.env, 'urdf')

        with self.env:
            name = module.SendCommand('LoadURI {} {}'.format(urdf_path, srdf_path))
            self.robot = self.env.GetRobot(name)

    def _attach_controllers_to_robot(self):
        self.multicontroller = RaveCreateMultiController(self.env, "")
        self.robot.SetController(self.multicontroller)

        robot_controller = RaveCreateController(self.env,'ur5controller')
        hand_controller = RaveCreateController(self.env, 'robotiqcontroller')

        self.multicontroller.AttachController(robot_controller, [2, 1, 0, 4, 5, 6], 0)
        self.multicontroller.AttachController(hand_controller, [3], 0)

    def get_current_hand_transform(self):
        return self.robot.GetManipulators()[0].GetTransform()

    def is_trajectory_safe(self, trajectory_object):
        waypoints = trajectory_object.GetAllWaypoints2D()
        waypoint_sums = [0, 0, 0, 0, 0, 0]
        for i in range(0, len(waypoints) - 1):
            for j in range(0, 6):
                difference = abs(waypoints[i][j] - waypoints[i+1][j])
                waypoint_sums[j] += difference

        waypoint_below_threshold = [x < numpy.pi/3 for x in waypoint_sums]

        if all(waypoint_below_threshold):
            return True

        return False

    def execute_trajectory_object(self, trajectory_object):
        try:
            self.robot.GetController().SetPath(trajectory_object)
            self.robot.WaitForController(0)
        except planning_error, e:
            print "ERROR: There was an error when executing the trajectory."
            print e

    def rotate_hand(self, rotation_matrix):
        current_hand_transform = self.get_current_hand_transform()
        goal_transform = numpy.dot(current_hand_transform, rotation_matrix)
        ik_solution = self.manipulator.FindIKSolution(goal_transform, IkFilterOptions.CheckEnvCollisions)

        trajectory_object = self.base_manipulation.MoveManipulator(goal=ik_solution, execute=False, outputtrajobj=True)

        if self.is_trajectory_safe(trajectory_object):
            self.execute_trajectory_object(trajectory_object)
        else:
            print "This trajectory failed the safety check."

    def move_hand(self, x_offset, y_offset):
        current_hand_transform = self.get_current_hand_transform()
        current_hand_transform[0, 3] += x_offset
        current_hand_transform[1, 3] += y_offset

        ik_solution = self.manipulator.FindIKSolution(current_hand_transform, IkFilterOptions.CheckEnvCollisions)

        trajectory_object = self.base_manipulation.MoveManipulator(goal=ik_solution, execute=False, outputtrajobj=True)

        if self.is_trajectory_safe(trajectory_object):
            self.execute_trajectory_object(trajectory_object)
        else:
            print "The trajectory failed the safety check."

    def keyboard_control(self):
        print ""
        print " q     w    e "
        print ""
        print " a     +    d "
        print ""
        print "       s      "
        print ""
        print "'exit' to stop the demo."

        while True:
            action = raw_input("Menu Action: ")
            if action == "exit": # EXIT Demo
                break
            if action == "q": # Rotate Anti-clockwise
                rotation_matrix = matrixFromAxisAngle([0, 0, -numpy.pi/6])
                self.rotate_hand(rotation_matrix)
            elif action == "e": # Rotate Clockwise
                rotation_matrix = matrixFromAxisAngle([0, 0, numpy.pi/6])
                self.rotate_hand(rotation_matrix)
            elif action == "o": # Open Gripper
                pass
            elif action == "c": # Close Gripper
                task_manipulation.CloseFingers()
            elif action in ["w", "s", "a", "d"]:
                x_offset = 0
                y_offset = 0

                if action == "w": x_offset = self.OFFSET  # Move Forward
                if action == "s": x_offset = -self.OFFSET # Move Backwards
                if action == "a": y_offset = -self.OFFSET # Move Left
                if action == "d": y_offset = self.OFFSET  # Move Right

                self.move_hand(x_offset, y_offset)


if __name__ == "__main__":
    # robot = Ur5Robot(has_two_finger_gripper=True,
    #                  has_force_torque_sensor=True)

    # robot.keyboard_control()

    IPython.embed()
