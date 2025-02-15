#include "ss_workshop/pid_controller.h"


namespace ss_workshop{
	PidController::PidController()
		: initialized_params_(false),
		  controller_active_(false)	{
	  	initParams();
	}

	PidController::~PidController() {}

	void PidController::initParams(){
		calculateAllocationMatrix(rotor_config_, &(allocation_matrix_));

		Eigen::Matrix3d inertia = mass_inertia_.tail(3).asDiagonal();

		// To make the tuning independent of the inertia matrix we divide here.
		normalized_attitude_gain_ = Kp_q_.transpose()
		  * inertia.inverse();
		// To make the tuning independent of the inertia matrix we divide here.
		normalized_angular_rate_gain_ = Kv_q_.transpose()
		  * inertia.inverse();


		Eigen::Matrix4d I;
		I.setIdentity();
		I.block<3, 3>(0, 0) = inertia;
		I(3, 3) = 1;

		ang_acc_rpms_.resize(rotor_config_.rotors.size(), 4);
		// Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
		// A^{ \dagger} = A^T*(A*A^T)^{-1}
		ang_acc_rpms_ = allocation_matrix_.transpose()
		  * (allocation_matrix_* allocation_matrix_.transpose()).inverse() * I;

		gravity_ = 9.81;
		initialized_params_ = true;
	}

	void PidController::getRPMs(Eigen::VectorXd* rpms) const{
		assert(rpms);
		assert(initialized_params_);

		rpms->resize(rotor_config_.rotors.size());

		if(!controller_active_){
			*rpms = Eigen::VectorXd::Zero(rpms->rows());
			return;
		}


		Eigen::Vector3d forces;
		ComputeDesiredForces(&forces);

		Eigen::Vector3d angular_acceleration;
		ComputeDesiredAngularAcc(forces, &angular_acceleration);

		// ROS_INFO_STREAM("Forces: " << forces);
		// Project thrust onto body z axis.
		double thrust = forces.dot(odometry_.orientation.toRotationMatrix().col(2));
		ROS_INFO_STREAM("Thrust: " << thrust);

		rpmConversion(rpms, &ang_acc_rpms_, angular_acceleration, thrust);
	}

	void PidController::ComputeDesiredForces(Eigen::Vector3d* forces) const{
		assert(forces);

		Eigen::Vector3d position_error;
		position_error = odometry_.position - com_traj_.position_W;

		// Transform velocity to world frame.
		const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
		Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
		Eigen::Vector3d velocity_error;
		velocity_error = velocity_W - com_traj_.velocity_W;

		Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

		static Eigen::Vector3d position_error_integral(0,0,0);
		if(controller_active_){
			static ros::Time last_time = ros::Time::now();
			ros::Time current_time = ros::Time::now();  
			position_error_integral += position_error * (current_time - last_time).toSec();
			last_time = current_time;
		}



		*forces = - (position_error.cwiseProduct(Kp_p_) + velocity_error.cwiseProduct(Kv_p_)
						+ position_error_integral.cwiseProduct(Ki_p_))
						+ (gravity_ * e_3 + com_traj_.acceleration_W) * mass_inertia_[0];

		// ROS_INFO_STREAM("forces: "<< *forces);


	}

	void PidController::ComputeDesiredAngularAcc(const Eigen::Vector3d& forces,
	                                                     Eigen::Vector3d* angular_acceleration) const {
		assert(angular_acceleration);

		Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
		Eigen::Matrix3d R_des;
		desAttFromForces(forces, com_traj_.getYaw(), &R_des);


		// Angle error according to pid et al.
		Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
		Eigen::Vector3d angle_error;
		vectorFromSkewMatrix(angle_error_matrix, &angle_error);

		// TODO(burrimi) include angular rate references at some point.
		Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
		angular_rate_des[2] = com_traj_.getYawRate();

		Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;



		*angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
		                       - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
	                       + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
	  // data_out->angular_acceleration = *angular_acceleration;
	  // data_out->angle_error = angle_error;
	  // data_out->angle_rate_error = angular_rate_error;
	  // tf::vectorEigenToMsg(*angular_acceleration, data_out->angular_acceleration);
	  // tf::vectorEigenToMsg(angle_error, data_out->angle_error);
	  // tf::vectorEigenToMsg(angular_rate_error, data_out->angle_rate_error);
		ROS_INFO_STREAM("moments: "<< *angular_acceleration);
	}

	void PidController::getAttThrust(Eigen::VectorXd* att_thrust) const{
		assert(att_thrust);

		Eigen::Vector3d forces;
		ComputeDesiredForces(&forces);
		double thrust = forces.dot(odometry_.orientation.toRotationMatrix().col(2));
		double throttle = std::min(std::max(0.0, thrust/max_thrust_), 1.0);

		Eigen::Matrix3d R_des;
		desAttFromForces(forces, com_traj_.getYaw(), &R_des);

		Eigen::Quaterniond des_quat(R_des);
		(*att_thrust)[0] = throttle;
		(*att_thrust)[1] = des_quat.x();
		(*att_thrust)[2] = des_quat.y();
		(*att_thrust)[3] = des_quat.z();
		(*att_thrust)[4] = des_quat.w();
	}

	void PidController::setOdometry(const EigenOdometry& odometry){
		odometry_ = odometry;
	}

	void PidController::setTraj(const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
		com_traj_ = command_trajectory;
		controller_active_ = true;
	}


}