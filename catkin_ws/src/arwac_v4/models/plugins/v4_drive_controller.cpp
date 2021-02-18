#include <stdio.h>
#include <math.h>
#include <string>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
using namespace std;

//Podcar Gazebo-to-ROS plugin.   Simulates sensors and recieves wheel commands.
//Build with:  cmake . ; make    (NB not catkin -- just including it to get ROS libraries in)

namespace gazebo
{
  class MyPlugin : public ModelPlugin
  {
		public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			int argc=0;
			char** argv;		
			ros::init(argc, argv, "GazeboPlugin");

			this->model = _parent;   //*er to my physical model, that I'm controlling
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&MyPlugin::OnUpdate, this, _1));      //GAZEBO (not ROS) callback
			this->twistSub = _n.subscribe("cmd_vel",  1, &MyPlugin::twistCallback,  (MyPlugin*)this);      
			this->_pub_gnd  = _n.advertise<nav_msgs::Odometry>("odometry/groundTruth", 1000);   //TODO more acurate to sim the IMU and GPS ten fuse in EKF later on

			this->fwdCmd = 0;
			this->angleCmd = 0;
			
			this-> timeOut = 2;
			this-> lastCmdTime = time(NULL);

			//These joint names don't make sense as the robot model is reversed
			this->fR = this->model->GetJoint("front_right_wheel_joint");
			this->fL = this->model->GetJoint("front_left_wheel_joint");
			this->bR = this->model->GetJoint("back_right_wheel_joint");
			this->bL = this->model->GetJoint("back_left_wheel_joint");
			this->steering = this->model->GetJoint("back_right_wheel_steering_joint");

			this->angleController = common::PID(50, 5, 10);
			this->model->GetJointController()->SetPositionPID(steering->GetScopedName(), this->angleController);
		}

		public: void twistCallback(const geometry_msgs::Twist& msg){
			this->lastCmdTime = time(NULL);

			// These are all *-1 because the robot model is backwards
			this->fwdCmd = -msg.linear.x/0.2224185; //divide by wheel radius
			this->angleCmd = -msg.angular.z;
		}

		public: void OnUpdate(const common::UpdateInfo & /*_info*/){
			//cout << "update " << endl;

			if(time(NULL)-this->lastCmdTime >= this->timeOut){
				this->fwdCmd = 0;
				this->angleCmd = 0;
			}

			bL->SetVelocity(0, this->fwdCmd); 
			bR->SetVelocity(0, this->fwdCmd);
			fL->SetVelocity(0, this->fwdCmd);
			fR->SetVelocity(0, this->fwdCmd);
			this->model->GetJointController()->SetPositionTarget(steering->GetScopedName(), this->angleCmd);

			ignition::math::Pose3d pose = this->model->WorldPose();
 			ignition::math::Vector3<double> position = pose.Pos();
			auto orientation = pose.Rot();
			nav_msgs::Odometry odomOut;
			odomOut.header.stamp = ros::Time::now(); 
			odomOut.header.frame_id = "map";
			odomOut.pose.pose.position.x = position.X();
			odomOut.pose.pose.position.y = position.Y();
			odomOut.pose.pose.position.z = position.Z();
			odomOut.pose.pose.orientation.x = orientation.X();
			odomOut.pose.pose.orientation.y = orientation.Y();
			odomOut.pose.pose.orientation.z = orientation.Z();
			odomOut.pose.pose.orientation.w = orientation.W();
			this->_pub_gnd.publish(odomOut);
		}

		private: ros::NodeHandle _n;
		protected: ros::Subscriber twistSub;
		protected: ros::Publisher _pub_gnd;
		private: physics::ModelPtr model;
		private: event::ConnectionPtr updateConnection;
		
		private: std::time_t lastCmdTime;
		private: std::time_t timeOut;
		private: double fwdCmd;
		private: double angleCmd;

		public: common::PID angleController;

		private: physics::JointPtr fR;
		private: physics::JointPtr fL;
		private: physics::JointPtr bR;
		private: physics::JointPtr bL;
		private: physics::JointPtr steering;
	};

	GZ_REGISTER_MODEL_PLUGIN(MyPlugin)   //tell Gazebo I'm here
}
