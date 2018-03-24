from openravepy import Robot
from openravepy import RaveCreateMultiController
from openravepy import RaveCreateController
from openravepy import interfaces
from openravepy import databases
from openravepy import IkParameterization

class UR5_Robot(Robot):
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

        # Needed for "find a grasp" function (not parsed using or_urdf hence
        # needs manual setting)
        self.GetManipulators()[0].SetChuckingDirection([1.0])
        self.GetManipulators()[0].SetLocalToolDirection([1.0, 0, 0])

        self.ikmodel = databases.inversekinematics.InverseKinematicsModel(self, iktype=IkParameterization.Type.Transform6D)
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()

    @property
    def end_effector_transform(self):
        return self.GetManipulators()[0].GetTransform()
