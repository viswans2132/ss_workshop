
#ifndef SS_WORKSHOP_VEL_NODE_H_
#define SS_WORKSHOP_VEL_NODE_H_

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>
#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <std_msgs/Int8.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/Thrust.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include "ss_workshop/vel_controller.h"

namespace ss_workshop{
	class VelNode{
	public:
		VelNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
		~VelNode();

		void initParams();
	private:
		ros::NodeHandle nh_;
		ros::NodeHandle private_nh_;
		VelController position_controller_;
		
		bool actuator_enabled_;
  		bool received_home_pose;

		geometry_msgs::PoseStamped home_pose_;

		mavros_msgs::State current_state_;
		mavros_msgs::SetMode offb_set_mode_;
		mavros_msgs::CommandBool arm_cmd_;

		ros::ServiceClient arming_client_;
		ros::ServiceClient set_mode_client_;
		ros::ServiceServer ctrltriggerServ_;
		ros::ServiceServer land_service_;


		ros::Subscriber odom_sub_;
		ros::Subscriber pose_sub_;
		ros::Subscriber land_sub_;
		ros::Subscriber vel_sub_;
		ros::Subscriber mavState_sub_;

		ros::Publisher thr_pub_;
		ros::Publisher att_pub_;
		ros::Publisher rpm_pub_;
		ros::Publisher tgt_pub_;

		mav_msgs::EigenTrajectoryPointDeque com_;

		std::deque<ros::Duration> com_wt_;
		ros::Timer com_timer_;
		ros::Timer pub_timer_;
		ros::Timer mavCom_timer_;
		ros::Timer sts_timer_;

		ros::Time last_request_;
		ros::Time reference_request_now_;
		ros::Time reference_request_last_;

		enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED } node_state_;

		void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
		void OdomCallback(const nav_msgs::OdometryConstPtr& msg);
		// void TrajCallback(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg);
		void VeloCallback(const geometry_msgs::TwistStampedConstPtr& msg);
		void LandCallback(const std_msgs::Int8ConstPtr& msg);

		// void ComCallback(const ros::TimerEvent &e);
		void PubCallback(const ros::TimerEvent &e);
		void MavComCallback(const ros::TimerEvent &e);
		void MavStateCallback(const mavros_msgs::State::ConstPtr &msg);
		void StsCallback(const ros::TimerEvent &e);
	};
}

#endif
