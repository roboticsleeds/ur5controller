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

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
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
class Ur5Controller : public ControllerBase
{
    public:
        Ur5Controller(EnvironmentBasePtr penv, std::istream &sinput) : ControllerBase(penv)
        {
            __description = ":Interface Authors: Rafael Papallas & Mehmet Dogar, The University of Leeds";

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
        void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg)
        {
            if (_paused)
            {
                return;
            }

            // Create a joint vector for angles and assign the new values from message.
            std::vector<double> joint_angles(6);
            for (unsigned int i = 0; i < (msg->position).size(); i++)
            {
                joint_angles[i] = (msg->position).at(i);
            }

            // Set DOF Values of the joint angles just received from message to the
            // robot in OpenRAVE.
            OpenRAVE::EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
            _probot->SetDOFValues(joint_angles,
                                  KinBody::CLA_CheckLimitsSilent,
                                  _dofindices);
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
            _joint_angles_sub = _pn->subscribe("/joint_states", 1, &Ur5Controller::JointStateCallback, this);

            // Publisher to /arm_controller/command, will publish to the robot the new joint values.
            _move_arm_pub = _pn->advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);

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

        double IsSameArmConfig(vector<double> &config1, vector<double> &config2)
        {
            for (unsigned int i = 0; i < config1.size(); i++)
            {
                if (std::fabs(config1[i] - config2[i]) > 0.01)
                {
                    return false;
                }
            }

            return true;
        }

        bool IsArmAtConfig(vector<double> &config)
        {
            std::vector<double> current_arm_config;
            static const int arr[] = {0, 1, 2, 3, 4, 5};
            vector<int> dofindices(arr, arr + sizeof(arr) / sizeof(arr[0]));
            _probot->GetDOFValues(current_arm_config, dofindices);
            return IsSameArmConfig(current_arm_config, config);
        }

        bool MoveArm(vector<double> values)
        {
            if (ros::ok() && !_paused)
            {
                trajectory_msgs::JointTrajectory trajectory;
                trajectory_msgs::JointTrajectoryPoint points_n;

                trajectory.header.stamp = ros::Time::now();
                trajectory.header.frame_id = "base_link";
                trajectory.joint_names.resize(6);
                trajectory.points.resize(1);

                trajectory.points[0].positions.resize(6);

                trajectory.joint_names[0] = "elbow_joint";
                trajectory.joint_names[1] = "shoulder_lift_joint";
                trajectory.joint_names[2] = "shoulder_pan_joint";
                trajectory.joint_names[3] = "wrist_1_joint";
                trajectory.joint_names[4] = "wrist_2_joint";
                trajectory.joint_names[5] = "wrist_3_joint";

                for (int i = 0; i < 6; i++)
                {
                    trajectory.points[0].positions[i] = values[i];
                }

                trajectory.points[0].time_from_start = ros::Duration(1);

                ROS_INFO("The new values are %f %f %f %f %f", values[0], values[1], values[2], values[3], values[4], values[5]);

                // Publish Changes
                _move_arm_pub.publish(trajectory);
                ros::spinOnce();

                // Store last values for later.
                _last_arm_command = values;

                return true;
            }

            return false;
        }

        bool MoveArmTowards(vector<double> &config)
        {
            if (IsArmAtConfig(config))
            {
                return true; // already there.
            }

            if (_last_arm_command.empty() || !IsSameArmConfig(_last_arm_command, config))
            {
                MoveArm(config);
                _last_arm_command = config;
            }

            return false;
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
                    arm_at_waypoint = MoveArmTowards(arm_goal);
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
        std::vector<double> _last_arm_command;

        ros::Subscriber _joint_angles_sub;
        ros::NodeHandle *_pn;
        ros::Publisher _move_arm_pub;

        RobotBasePtr _probot;
        EnvironmentBasePtr _penv;
        TrajectoryBasePtr _traj;
};

ControllerBasePtr CreateUr5Controller(EnvironmentBasePtr penv, std::istream &sinput)
{
    return ControllerBasePtr(new Ur5Controller(penv, sinput));
}
