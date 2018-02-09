//  Copyright (C) 2017 Rafael Papallas
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


/**
	This is a controller for the UR5 robot. The main purpose of this class is to
	subscribe to ROS topic that publishes the joint angles of the robot. By listening
	to those values we can make the OpenRAVE robot model change in real-time as with
	the robot or vice versa.

	@author Rafael Papallas
*/
class RobotiqController : public ControllerBase
{
    public:
        RobotiqController(EnvironmentBasePtr penv, std::istream &sinput) : ControllerBase(penv)
        {
            __description = ":Interface Authors: Rafael Papallas & Dr Mehmet Dogar, The University of Leeds";

            _nControlTransformation = 0;
            _penv = penv;
            _paused = false;
            _initialized = false;
        }

        /**
            Callback when the joint values changes. Responsible to change the robot
            model in OpenRAVE.

            @param msg the message sent by the subscriber (i.e the new joint values)
        */
        void JointStateCallback(const robotiq_c_model_control::CModel_robot_output::ConstPtr &msg)
        {
            if (_paused)
            {
                return;
            }

            std::vector<double> gripper_value;
            gripper_value.push_back(RobotValueToModelValue(msg->rPR));

            // Set DOF Values of the joint angles just received from message to the
            // robot in OpenRAVE.
            OpenRAVE::EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
            _probot->SetDOFValues(gripper_value,
                                  KinBody::CLA_CheckLimitsSilent,
                                  _dofindices);
        }

        /**
          Given a value in the range of 0 - OPENRAVE_GRIPPER_MAX_VALUE will map
          this value to a number in the range of 0 - ROBOT_GRIPPER_MAX_VALUE.

          OpenRAVE uses a float representation for the gripper joint from 0 - ~0.87..
          where the actual gripper joint limits are 0 - 255.
        */
        int RobotValueToModelValue(double value) {
          return (int) round(OPENRAVE_GRIPPER_MAX_VALUE / ROBOT_GRIPPER_MAX_VALUE * abs(value));
        }

        /**
          Given a value in the range of 0 - ROBOT_GRIPPER_MAX_VALUE will map
          this value to a number in the range of 0 - OPENRAVE_GRIPPER_MAX_VALUE.

          OpenRAVE uses a float representation for the gripper joint from 0 - ~0.87..
          where the actual gripper joint limits are 0 - 255.
        */
        double ModelValueToRobotValue(int value) {
          return ROBOT_GRIPPER_MAX_VALUE / OPENRAVE_GRIPPER_MAX_VALUE * abs(value);
        }

        /**
            Initialiser when the robot is attached to the controller.

            Subscribes to the topic /joint_states that listens to changes to the
            robot joints and calls a callback to act on the new values.
        */
        virtual bool Init(RobotBasePtr robot, const std::vector<int> &dofindices, int nControlTransformation)
        {
            _probot = robot;

            if (!!_probot)
            {
                _dofindices = dofindices;
                _nControlTransformation = nControlTransformation;
            }

            _pn = new ros::NodeHandle();

            // Subscribe to the topic that the robot publishes changes to joint values.
            _gripper_subscriber = _pn->subscribe("/CModelRobotOutput", 1, &RobotiqController::JointStateCallback, this);

            // Publisher to /arm_controller/command, will publish to the robot the new joint values.
            _gripper_publisher = _pn->advertise<robotiq_c_model_control::CModel_robot_input>("/CModelRobotInput", 1);

            _traj = RaveCreateTrajectory(_penv, "");
            _traj->Init(robot->GetConfigurationSpecification());
            _initialized = true;

            return true;
        }

        virtual void Reset(int options)
        {
        }

        virtual const std::vector<int> &GetControlDOFIndices() const
        {
            return _dofindices;
        }

        virtual int IsControlTransformation() const
        {
            return _nControlTransformation;
        }

        virtual bool SetDesired(const std::vector <OpenRAVE::dReal> &values, TransformConstPtr trans)
        {
            return true;
        }

        virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
        {
            if (ptraj != NULL)
            {
                _traj = RaveCreateTrajectory(GetEnv(), ptraj->GetXMLId());
                _traj->Clone(ptraj, Clone_Bodies);
            }

            return true;
        }

        virtual void SimulationStep(dReal fTimeElapsed)
        {
            if (!_initialized)
            {
                return;
            }

            if (_traj->GetNumWaypoints() > 0)
            {
                vector <dReal> waypoint;

                _traj->GetWaypoint(0, waypoint);

                static const int arr[] = {0, 1, 2, 3, 4, 5};
                std::vector<int> arm_indices(arr, arr + sizeof(arr) / sizeof(arr[0]));
                std::vector <dReal> arm_goal(6);
                bool arm_at_waypoint = true;

                if (_traj->GetConfigurationSpecification().ExtractJointValues(arm_goal.begin(),
                                                                              waypoint.begin(),
                                                                              _probot,
                                                                              arm_indices))
                {
                    // arm_at_waypoint = MoveArmTowards(arm_goal);
                }

                if (arm_at_waypoint)
                {
                    // Remove the reached (first) way-point. Now the next way-point is the first way-point.
                    _traj->Remove(0, 1);
                }
            }

            ros::spinOnce();
        }

        virtual bool IsDone()
        {
            return _traj->GetNumWaypoints() == 0;
        }

        virtual OpenRAVE::dReal GetTime() const
        {
            return 0;
        }

        virtual RobotBasePtr GetRobot() const
        {
            return _probot;
        }


    private:
        bool _initialized;
        bool _paused;
        int _nControlTransformation;

        std::vector<int> _dofindices;
        robotiq_c_model_control::CModel_robot_input _command = robotiq_c_model_control::CModel_robot_input();

        ros::Subscriber _gripper_subscriber;
        ros::Publisher _gripper_publisher;
        ros::NodeHandle *_pn;

        RobotBasePtr _probot;
        EnvironmentBasePtr _penv;
        TrajectoryBasePtr _traj;

        const float OPENRAVE_GRIPPER_MAX_VALUE = 0.87266444;
        const int ROBOT_GRIPPER_MAX_VALUE = 255;
};

ControllerBasePtr CreateRobotiqController(EnvironmentBasePtr penv, std::istream &sinput)
{
    return ControllerBasePtr(new RobotiqController(penv, sinput));
}
