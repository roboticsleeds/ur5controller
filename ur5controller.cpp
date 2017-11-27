#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/JointValue.h"
#include "brics_actuator/JointPositions.h"
#include "plugindefs.h"

using namespace std;
using namespace OpenRAVE;

/**
	This is a controller for the UR5 robot. The main purpose of this class is to
	subscribe to ROS topic that publishes the joint angles of the robot. By listening
	to those values we can make the OpenRAVE robot model change in real-time as with
	the robot or vice versa.

	@author Rafael Papallas
	@version 1.0 14/11/2017
*/
class Ur5Controller : public ControllerBase
{
	public:
		Ur5Controller(EnvironmentBasePtr penv, std::istream& sinput) : ControllerBase(penv)
		{
			__description = ":Interface Authors: Rafael Papallas, Mehmet Dogar";

			_nControlTransformation = 0;
			_penv = penv;
			_paused = false;
			_initialized = false;
			_current_waypoint_index = 0;
			_current_waypoint_executed = false;
			//RAVELOG_ERROR("Test 1");
		}

		/**
			Callback when the joint values changes. Responsible to change the robot
			model in OpenRAVE.

			@param msg the message sent by the subscriber (i.e the new joint values)
		*/
		void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
		{
			//RAVELOG_ERROR("Test 2a");
		  if (_paused)
		  {
		  	return;
		  }

			// Create a joint vector for angles and assign the new values from message.
		  std::vector <double> joint_angles;
		  for (unsigned int i = 0; i < (msg->position).size(); i++)
		  {
		    joint_angles.push_back((msg->position).at(i));
		  }

			// Set DOF Values of the joint angles just received from message to the
			// robot in OpenRAVE.
      OpenRAVE::EnvironmentMutex::scoped_lock lockenv(_penv->GetMutex());
      _probot->SetDOFValues(joint_angles,
														KinBody::CLA_CheckLimitsSilent);

			//RAVELOG_ERROR("Test 2");
		}

		/**
			Initialiser when the robot is attached to the controller.

			Subscribes to the topic /joint_states that listens to changes to the
			robot joints and calls a callback to act on the new values.
		*/
		virtual bool Init(RobotBasePtr robot, const std::vector<int>& dofindices, int nControlTransformation)
		{
			_probot = robot;

      if ( !!_probot )
      {
	      _nControlTransformation = nControlTransformation;
      }

			int dofindices_array[] = {1,2,3,4,5,6};

			for (int i = 0; i<=5;i++)
			{
				_dofindices.push_back(dofindices_array[i]);
			}

			_pn = new ros::NodeHandle();
			_joint_angles_sub = _pn->subscribe("/joint_states", 1, &Ur5Controller::JointStateCallback, this);
			_move_arm_pub = _pn->advertise<trajectory_msgs::JointTrajectory>("/arm_controller/command", 1);

			// Class attribute indicating that the classs has been intialised.
			_initialized = true;

			//RAVELOG_ERROR("Test 3");

			return true;
		}

		virtual void Reset(int options)
		{
			_traj.reset();
			//RAVELOG_ERROR("Test 4");
		}

		virtual const std::vector<int>& GetControlDOFIndices() const
		{
			//RAVELOG_ERROR("Test 5");
			return _dofindices;
		}

		virtual int IsControlTransformation() const
		{
			//RAVELOG_ERROR("Test 6");
			return _nControlTransformation;
		}

		virtual bool SetDesired(const std::vector<OpenRAVE::dReal>& values, TransformConstPtr trans)
		{
			//RAVELOG_ERROR("Test 7");
			return true;
		}

		virtual bool SetPath(TrajectoryBaseConstPtr ptraj)
		{
			//RAVELOG_ERROR("Test 8a");
			_traj.reset();
			if (ptraj != NULL)
			{
				_traj = RaveCreateTrajectory(GetEnv(),ptraj->GetXMLId());
				_traj->Clone(ptraj, Clone_Bodies);
			}
			//RAVELOG_ERROR("Test 8");
			return true;
		}

		double IsSameArmConfig(vector<double> &config1, vector<double> &config2)
		{
			//RAVELOG_ERROR("Test 9a");
				for (unsigned int i = 0; i < config1.size(); i++)
				{
						if (std::fabs( config1[i] - config2[i] ) > 0.01)
						{
								return false;
						}
				}
				//RAVELOG_ERROR("Test 9");
				return true;
		}

		bool IsArmAtConfig(vector<double> &config)
		{
				//RAVELOG_ERROR("Test 10a");
				std::vector<double> current_arm_config;
				static const int arr[] = {0, 1, 2, 3, 4, 5};
				vector<int> dofindices (arr, arr + sizeof(arr) / sizeof(arr[0]) );
				_probot->GetDOFValues(current_arm_config, dofindices);

				//RAVELOG_ERROR("Test 10");
				return IsSameArmConfig(current_arm_config,config);
		}

		bool MoveArm(vector<double> values)
		{	//RAVELOG_ERROR("Test 11a");
				if (ros::ok() && !_paused){

						// for (size_t i = 0; i < _offset.size(); ++i)
						// {
						// 		values[i] += _offset.at(i);
						// }
						trajectory_msgs::JointTrajectory joint_state;

           joint_state.header.stamp = ros::Time::now();
           joint_state.header.frame_id = "test";
           joint_state.joint_names.resize(6);
           joint_state.points.resize(6);

           joint_state.joint_names[0] ="elbow_joint";
           joint_state.joint_names[1] ="shoulder_lift_joint";
           joint_state.joint_names[2] ="shoulder_pan_joint";
					 joint_state.joint_names[2] ="wrist_1_joint";
					 joint_state.joint_names[2] ="wrist_2_joint";
					 joint_state.joint_names[2] ="wrist_3_joint";


           size_t size = 5;
           for(size_t i=0;i<=size;i++) {
              trajectory_msgs::JointTrajectoryPoint points_n;
              int j = i%6;
              points_n.positions.push_back(values[i]);
              // points_n.positions.push_back(j+1);
              // points_n.positions.push_back(j*2);
              joint_state.points.push_back(points_n);
              joint_state.points[i].time_from_start = ros::Duration(0.01);
           }

						//
						// trajectory_msgs::JointTrajectory command;
						// vector <float> armJointPositions(6);
						//
						//
						// vector <std::string> armJointNames(6);
						// armJointNames[0] = "elbow_joint";
						// armJointNames[1] = "shoulder_lift_joint";
						// armJointNames[2] = "shoulder_pan_joint"command;
						// armJointNames[3] = "wrist_1_joint";
						// armJointNames[4] = "wrist_2_joint";
						// armJointNames[5] = "wrist_3_joint";
						//
						// for (int i = 0; i < 6; ++i)
						// {
						// 		armJointPositions[i] = values[i];
						// }
						// command.joint_names.resize(6);
						// // command.joint_names = armJointNames;
						// command.points[0].positions = armJointPositions;
						ROS_INFO("the values are %f %f %f %f %f", values[0], values[1], values[2], values[3], values[4]);
						_move_arm_pub.publish(joint_state);

						// Store last values for later.
						// _last_arm_command = values;
						//RAVELOG_ERROR("Test 11");
						return true;
				}
				//RAVELOG_ERROR("Test 12");
				return false;
		}

		bool MoveArmTowards(vector<double> &config)
		{
			//RAVELOG_ERROR("Test 13a");
				if (IsArmAtConfig(config))
				{
						return true; // already there.
				}

				if (_current_waypoint_executed == false)
				{
						MoveArm(config);
						_current_waypoint_executed = true;
				}

				//RAVELOG_ERROR("Test 13");

				return false;
		}

		virtual void SimulationStep(dReal fTimeElapsed)
		{
			//RAVELOG_ERROR("Test 14a");
			if(!_initialized)
			{
				return;
			}

			TrajectoryBaseConstPtr ptraj = _traj;
			////RAVELOG_ERROR("Test 14b");
			//
			// ////RAVELOG_ERROR("Test ", ptraj->GetNumWaypoints());
			// ////RAVELOG_ERROR("Test 14c");
			if(ptraj != NULL && ptraj->GetNumWaypoints() >= _current_waypoint_index && _current_waypoint_executed)
			{
				////RAVELOG_ERROR("Test 14c");
				vector<dReal> waypoint;
				ptraj->GetWaypoint(_current_waypoint_index, waypoint);
				_current_waypoint_executed = false;
				_current_waypoint_index += 1;

				//RAVELOG_ERROR("Test 14d");

				static const int arr[] = {0, 1, 2, 3, 4, 5};
				std::vector< int > arm_indices(arr,arr+sizeof(arr)/sizeof(arr[0]));
				std::vector< dReal > arm_goal(6);
				bool arm_at_waypoint = true;

				if( ptraj->GetConfigurationSpecification().ExtractJointValues(arm_goal.begin(),waypoint.begin(),_probot,arm_indices) )
				{
						arm_at_waypoint = MoveArmTowards(arm_goal);
				}
			}
			//RAVELOG_ERROR("Test 14");
			ros::spinOnce();
		}

		virtual bool IsDone()
		{
			//RAVELOG_ERROR("Test 15");
			return _traj->GetNumWaypoints() < _current_waypoint_index;
		}

		virtual OpenRAVE::dReal GetTime() const
		{
			//RAVELOG_ERROR("Test 16");
			return 0;
		}

		virtual RobotBasePtr GetRobot() const
		{
			//RAVELOG_ERROR("Test 17");
			return _probot;
		}


	private:
		bool _initialized;
		RobotBasePtr _probot;
		std::vector<int> _dofindices;
		int _nControlTransformation;
		bool _paused;
		ros::Subscriber _joint_angles_sub;

		ros::NodeHandle* _pn;
		EnvironmentBasePtr _penv;
		TrajectoryBasePtr _traj;

		// Keeps track of the current waypoint in progress.
		unsigned int _current_waypoint_index;
		bool _current_waypoint_executed;
		ros::Publisher _move_arm_pub;

};

ControllerBasePtr CreateUr5Controller(EnvironmentBasePtr penv, std::istream& sinput)
{
	return ControllerBasePtr(new Ur5Controller(penv,sinput));
}
