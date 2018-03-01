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

#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"
#include "ros/time.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "control_msgs/FollowJointTrajectoryGoal.h"
#include "control_msgs/FollowJointTrajectoryAction.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "plugindefs.h"
#include <openrave/planningutils.h>

using namespace std;
using namespace OpenRAVE;

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

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
            _velocity_maximum_limit_per_joint = 0.2;
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
            robot joints and calls a callback to act on the new values. Will 
            also crient an action client to send trajectories to the action
            server.
        */
        virtual bool Init(RobotBasePtr robot, const std::vector<int> &dofindices, int nControlTransformation)
        {
            _probot = robot;

            if (!!_probot)
            {
                _dofindices = dofindices;
                _nControlTransformation = nControlTransformation;
            }

            // Add velocity limits.
            OpenRAVE::EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
            vector<dReal> velocity_limits;

            for(int i=0; i<7; i++) {
                velocity_limits.push_back(_velocity_maximum_limit_per_joint);
            }

            _probot->SetDOFVelocityLimits(velocity_limits);

            _pn = new ros::NodeHandle();

            // Subscribe to the topic that the robot publishes changes to joint values.
            _joint_angles_sub = _pn->subscribe("/joint_states", 1, &Ur5Controller::JointStateCallback, this);
            _initialized = true;

            _ac = new TrajClient("follow_joint_trajectory", true);
            _ac->waitForServer();

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

        TrajectoryBasePtr SimplifyTrajectory(TrajectoryBaseConstPtr ptraj)
        {
            TrajectoryBasePtr traj = RaveCreateTrajectory(_penv, ptraj->GetXMLId());
            traj->Init(_probot->GetConfigurationSpecification());
            traj->Clone(ptraj, Clone_Bodies);

            std::vector<ConfigurationSpecification::Group>::const_iterator it = ptraj->GetConfigurationSpecification().FindCompatibleGroup("iswaypoint",true);
            if (it == ptraj->GetConfigurationSpecification()._vgroups.end())
            {
                return traj;
            }
            
            for(int i=ptraj->GetNumWaypoints()-1; i >= 0; i--) 
            {
                std::vector <dReal> values(1);
                ptraj->GetWaypoint(i,values,*it);
                if(values[0] == 0)
                {
                    traj->Remove(i,i+1);
                }
            }
            return traj;
        }

        virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
        {
            if (ptraj != NULL)
            {
                TrajectoryBasePtr traj = SimplifyTrajectory(ptraj);

                PlannerStatus status = planningutils::RetimeTrajectory(traj, false, 1.0, 1.0, "LinearTrajectoryRetimer");
                if (status != PS_HasSolution)
                {
                    ROS_ERROR("Not executing trajectory because retimer failed.");
                    return false;
                }

                trajectory_msgs::JointTrajectory trajectory = FromOpenRaveToRosTrajectory(traj);
                control_msgs::FollowJointTrajectoryGoal goal;
                goal.trajectory = trajectory;
                _ac->sendGoal(goal);
            }

            return true;
        }

        trajectory_msgs::JointTrajectory FromOpenRaveToRosTrajectory(TrajectoryBasePtr traj) 
        {
            trajectory_msgs::JointTrajectory trajectory;
            trajectory.header.stamp = ros::Time::now();
            trajectory.header.frame_id = "base_link";
            trajectory.joint_names.resize(6);
            trajectory.points.resize(traj->GetNumWaypoints());
            trajectory.joint_names[0] = "shoulder_pan_joint";
            trajectory.joint_names[1] = "shoulder_lift_joint";
            trajectory.joint_names[2] = "elbow_joint";
            trajectory.joint_names[3] = "wrist_1_joint";
            trajectory.joint_names[4] = "wrist_2_joint";
            trajectory.joint_names[5] = "wrist_3_joint";

            for(int i=0; i < traj->GetNumWaypoints(); i++) 
            {
                trajectory_msgs::JointTrajectoryPoint ros_waypoint;
                vector <dReal> or_waypoint;
                traj->GetWaypoint(i, or_waypoint);
                std::vector <dReal> values(6);
                traj->GetConfigurationSpecification().ExtractJointValues(values.begin(),
                                                                         or_waypoint.begin(),
                                                                         _probot,
                                                                         _dofindices);

                dReal deltatime;
                traj->GetConfigurationSpecification().ExtractDeltaTime(deltatime,
                                                                        or_waypoint.begin());

                trajectory.points[i].positions.resize(6);
                trajectory.points[i].velocities.resize(6);

                for (int j = 0; j < 6; j++)
                {
                  trajectory.points[i].positions[j] = values[j];
                  trajectory.points[i].velocities[j] = 0.0;
                }

                if (i>0)
                  trajectory.points[i].time_from_start = trajectory.points[i-1].time_from_start + ros::Duration(deltatime);
                else
                  trajectory.points[0].time_from_start = ros::Duration(0);
            }

            return trajectory;
        }
        
        virtual void SimulationStep(dReal fTimeElapsed)
        {
            if (!_initialized)
            {
                return;
            }

            ros::spinOnce();
        }

        virtual bool IsDone()
        {
          return _ac->waitForResult(ros::Duration(0.05));
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
        dReal _velocity_maximum_limit_per_joint;

        std::vector<int> _dofindices;

        ros::Subscriber _joint_angles_sub;
        ros::NodeHandle *_pn;
        TrajClient* _ac;

        RobotBasePtr _probot;
        EnvironmentBasePtr _penv;
};

ControllerBasePtr CreateUr5Controller(EnvironmentBasePtr penv, std::istream &sinput)
{
    return ControllerBasePtr(new Ur5Controller(penv, sinput));
}
