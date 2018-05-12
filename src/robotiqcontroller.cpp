//  Copyright (C) 2017 The University of Leeds
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//  Author: Rafael Papallas (www.papallas.me)

#include <stdlib.h>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "robotiq_c_model_control/CModel_robot_input.h"
#include "robotiq_c_model_control/CModel_robot_output.h"
#include "plugindefs.h"

using namespace std;
using namespace OpenRAVE;

typedef robotiq_c_model_control::CModel_robot_output Gripper_Output;
typedef robotiq_c_model_control::CModel_robot_input Gripper_Input;

/**
	This is a controller for the Robotiq Two-Finger Gripper. This is an
  OpenRAVE controller that will set the DOF value of the gripper in
  OpenRAVE and will also execute trajectories/actions to the actual
  gripper.

  This gripper is also working with the OpenRAVE MultiController.

	@author Rafael Papallas
*/
class RobotiqController : public ControllerBase {
    public:
        RobotiqController(EnvironmentBasePtr penv,
                          std::istream &sinput) : ControllerBase(penv) {
            __description = ":Interface Authors: Rafael Papallas & Dr Mehmet Dogar, The University of Leeds";

            _nControlTransformation = 0;
            _penv = penv;
            _paused = false;
            _initialized = false;
        }

        /**
            Initialiser when the robot is attached to the controller.

            Subscribes to the topic /joint_states that listens to changes
            to the robot joints and calls a callback to act on the new
            values.
        */
        virtual bool Init(RobotBasePtr robot, const std::vector<int> &dofindices, int nControlTransformation) {
            _probot = robot;

            if (!!_probot) {
                _dofindices = dofindices;
                _nControlTransformation = nControlTransformation;
            }

            _pn = new ros::NodeHandle();

            // Subscribe to the topic that the robot publishes changes to joint values.
            _gripper_subscriber = _pn->subscribe("CModelRobotInput",
                                                 1,
                                                 &RobotiqController::GripperStateCallback,
                                                 this);

            // Publisher to /arm_controller/command, will publish to the robot the new joint values.
            _gripper_publisher = _pn->advertise<Gripper_Output>("CModelRobotOutput", 1);

            _initialized = true;

            // Usually when starting the gripper for the first time, it
            // requires to rest and activate it before you can operate it
            // by running reset and activate commands.
            reset();
            activate();

            return true;
        }

        /**
            Callback when the joint values changes. Responsible to
            change the robot model in OpenRAVE.

            @param msg the message sent by the subscriber (i.e the new
                       joint values)
        */
        void GripperStateCallback(const Gripper_Input::ConstPtr &msg) {
            if (_paused) {
                return;
            }

            _current_status = msg->gSTA;
            _is_gripper_activated = msg->gACT;

            std::vector<double> gripper_value;
            gripper_value.push_back(RobotValueToModelValue(msg->gPR));

            // Set DOF Values of the joint angles just received from message
            // to the robot in OpenRAVE.
            OpenRAVE::EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
            _probot->SetDOFValues(gripper_value,
                                  KinBody::CLA_CheckLimitsSilent,
                                  _dofindices);
        }

        /**
          Given a value in the range of 0 - OPENRAVE_GRIPPER_MAX_VALUE
          will map this value to a number in the range of
          0 - ROBOT_GRIPPER_MAX_VALUE.

          OpenRAVE uses a float representation for the gripper joint
          from 0 - ~0.87.. where the actual gripper joint limits
          are 0 - 255.
        */
        int ModelValueToRobotValue(double value) {
          return (int) round(ROBOT_GRIPPER_MAX_VALUE / OPENRAVE_GRIPPER_MAX_VALUE * abs(value));
        }

        /**
          Given a value in the range of 0 - ROBOT_GRIPPER_MAX_VALUE will map
          this value to a number in the range of
          0 - OPENRAVE_GRIPPER_MAX_VALUE.

          OpenRAVE uses a float representation for the gripper joint
          from 0 - ~0.87.. where the actual gripper joint limits
          are 0 - 255.
        */
        double RobotValueToModelValue(int value) {
          return OPENRAVE_GRIPPER_MAX_VALUE / ROBOT_GRIPPER_MAX_VALUE * abs(value);
        }

        virtual void Reset(int options) {
          _command.rACT = 0;
          publish_command();
        }

        virtual const std::vector<int> &GetControlDOFIndices() const {
            return _dofindices;
        }

        virtual int IsControlTransformation() const {
            return _nControlTransformation;
        }

        virtual bool SetDesired(const std::vector <OpenRAVE::dReal> &values,
                                TransformConstPtr trans) {
            return setValue(ModelValueToRobotValue(values[0]));
        }

        virtual bool SetPath(TrajectoryBaseConstPtr ptraj) {
            return true;
        }

        virtual void SimulationStep(dReal fTimeElapsed) {
            if (!_initialized) {
                return;
            }

            ros::spinOnce();
        }

        virtual bool IsDone() {
          return _current_status == 3 && _is_gripper_activated == 1;
        }

        virtual OpenRAVE::dReal GetTime() const {
            return 0;
        }

        virtual RobotBasePtr GetRobot() const {
            return _probot;
        }

        void publish_command() {
          _gripper_publisher.publish(_command);
        }

        bool setValue(int value) {
          if (value >= 0 && value <= 255) {
            _command.rPR = value;
            _command.rACT = 1;
            _command.rGTO = 1;
            _command.rATR = 0;
            _command.rSP = 255;
            _command.rFR = 150;

            publish_command();
            return true;
          }
          else {
            ROS_ERROR("The value for the gripper was out of limits 0-255.");
            return false;
          }
        }

        void reset() {
          _command.rACT = 0;
          _command.rGTO = 0;
          _command.rATR = 0;
          _command.rPR = 0;
          _command.rSP = 0;
          _command.rFR = 0;

          publish_command();
        }

        void activate() {
          _command.rACT = 1;
          _command.rGTO = 0;
          _command.rSP = 255;
          _command.rFR = 150;

          publish_command();
        }

    private:
        bool _initialized;
        bool _paused;
        int _nControlTransformation;
        int _current_status;
        int _is_gripper_activated;

        std::vector<int> _dofindices;
        Gripper_Output _command;

        ros::Subscriber _gripper_subscriber;
        ros::Publisher _gripper_publisher;
        ros::NodeHandle *_pn;

        RobotBasePtr _probot;
        EnvironmentBasePtr _penv;

        const float OPENRAVE_GRIPPER_MAX_VALUE = 0.77;
        const int ROBOT_GRIPPER_MAX_VALUE = 255;
};

ControllerBasePtr CreateRobotiqController(EnvironmentBasePtr penv, std::istream &sinput) {
    return ControllerBasePtr(new RobotiqController(penv, sinput));
}
