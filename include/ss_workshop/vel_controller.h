#ifndef SS_WORKSHOP_VEL_CONTROLLER_H_
#define SS_WORKSHOP_VEL_CONTROLLER_H_

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
// #include <geometric_msgs/Pose.h>

#include "ss_workshop/common_operations.h"

namespace ss_workshop{
	class VelController{
	public:
		VelController();
		~VelController();
		void initParams();
		void getRPMs(Eigen::VectorXd* rpms) const;
		void getAttThrust(Eigen::VectorXd* att_thrust) const;
		void setVelo(Eigen::Vector4d* com_traj);
		// void setOdometryFromPose(const geometric_msgs::Pose& pose);
		void setOdometry(const EigenOdometry& odom);
		void disableControl();

		Eigen::Vector3d Kp_p_;
		Eigen::Vector3d Kv_p_;
		Eigen::Vector3d Ki_p_;
		Eigen::Vector3d Kp_q_;
		Eigen::Vector3d Kv_q_;
		Eigen::Vector4d mass_inertia_;
		Eigen::Matrix4Xd allocation_matrix_;
		RotorConfiguration rotor_config_;
		double gravity_;
		double max_thrust_;

		bool controller_active_;

	private:
		bool initialized_params_;

		Eigen::Vector3d normalized_attitude_gain_;
		Eigen::Vector3d normalized_angular_rate_gain_;
		Eigen::MatrixX4d ang_acc_rpms_;

		//double test_mass; //Added by Viswa

		Eigen::Vector3d com_vel_;
		double com_yaw_rate_;
		EigenOdometry odometry_;


		void ComputeDesiredAngularAcc(const Eigen::Vector3d& forces,
		                            Eigen::Vector3d* angular_acceleration) const;
		void ComputeDesiredForces(Eigen::Vector3d* forces) const;

	};




}

#endif