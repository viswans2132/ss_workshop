#include "ss_workshop/vel_controller.h"


namespace ss_workshop{
	VelController::VelController()
		: initialized_params_(false),
		  controller_active_(false)	{
	  	initParams();
	}

	VelController::~VelController() {}

	void VelController::initParams(){
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

	void VelController::getRPMs(Eigen::VectorXd* rpms) const{
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

		rpmConversion(rpms, &ang_acc_rpms_, angular_acceleration, thrust);
	}

	void VelController::ComputeDesiredForces(Eigen::Vector3d* forces) const{
		assert(forces);

		// Transform velocity to world frame.
		const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
		Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
		static Eigen::Vector3d velocity_error(0, 0, 0);		
		static Eigen::Vector3d velocity_error_integral(0,0,0);
		Eigen::Vector3d velocity_error_derivative(0,0,0);
		Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());

		if(controller_active_){
			static ros::Time last_time = ros::Time::now();
			ros::Time current_time = ros::Time::now();
			double dt = (current_time - last_time).toSec();
			dt = std::min(std::max(0.01, dt), 0.02);
			dt = 0.01;

			velocity_error_derivative = ((velocity_W - com_vel_) - velocity_error)/(dt);
			velocity_error = velocity_W - com_vel_;
			velocity_error_integral += velocity_error * dt;
			Eigen::Vector3d max_integral_limit(0.5, 0.5, 2.0);
			velocity_error_integral = velocity_error_integral.cwiseMin(max_integral_limit);
			velocity_error_integral = velocity_error_integral.cwiseMax(-max_integral_limit);
			last_time = current_time;
		}

		// Eigen::Vector3d position_component = velocity_error.cwiseProduct(Kp_p_);
		// Eigen::Vector3d derivative_component = velocity_error_derivative.cwiseProduct(Kv_p_);
		// Eigen::Vector3d integral_component = velocity_error_integral.cwiseProduct(Ki_p_);

		// ROS_INFO_STREAM("velocity_error_xyz: " << velocity_error.x() << " : " << velocity_error.y() << " : " << velocity_error.z());
		// ROS_INFO_STREAM("position_component: " << position_component.x() << " : " << position_component.y() << " : " << position_component.z());
		// ROS_INFO_STREAM("derivative_component: " << derivative_component.x() << " : " << derivative_component.y() << " : " << derivative_component.z());
		// ROS_INFO_STREAM("integral_component: " << integral_component.x() << " : " << integral_component.y() << " : " << integral_component.z());



		*forces = - (velocity_error.cwiseProduct(Kp_p_) + velocity_error_integral.cwiseProduct(Ki_p_) + velocity_error_derivative.cwiseProduct(Kv_p_))
						+ gravity_ * e_3 * mass_inertia_[0];

		// ROS_INFO_STREAM("velocity_error: " << velocity_error_integral.x() << " : " << velocity_error_integral.y() << " : " << velocity_error_integral.z());
		// ROS_INFO_STREAM("Integral Component: " << velocity_error_integral.cwiseProduct(Ki_p_));
		// ROS_INFO_STREAM("forces: "<< *forces);
	}

	void VelController::ComputeDesiredAngularAcc(const Eigen::Vector3d& forces,
	                                                     Eigen::Vector3d* angular_acceleration) const {
		assert(angular_acceleration);

		Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
		Eigen::Matrix3d R_des;
		double yaw = R.eulerAngles(0,1,2)[2];
		// ROS_INFO_STREAM("yaw: " << yaw);
		desAttFromForces(forces, 0.0, &R_des);


		// Angle error according to pid et al.
		Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
		Eigen::Vector3d angle_error;
		vectorFromSkewMatrix(angle_error_matrix, &angle_error);

		// TODO(burrimi) include angular rate references at some point.
		Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
		angular_rate_des[2] = com_yaw_rate_;

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
		// ROS_INFO_STREAM("moments: "<< *angular_acceleration);
	}

	void VelController::getAttThrust(Eigen::VectorXd* att_thrust) const{
		assert(att_thrust);

		Eigen::Vector3d forces;
		ComputeDesiredForces(&forces);
		double thrust = forces.dot(odometry_.orientation.toRotationMatrix().col(2));
		double throttle = std::min(std::max(0.0, thrust/max_thrust_), 1.0);

		Eigen::Matrix3d R_des;
		double yaw = odometry_.orientation.toRotationMatrix().eulerAngles(0,1,2)[2];
		desAttFromForces(forces, yaw, &R_des);

		Eigen::Quaterniond des_quat(R_des);
		(*att_thrust)[0] = throttle;
		(*att_thrust)[1] = des_quat.x();
		(*att_thrust)[2] = des_quat.y();
		(*att_thrust)[3] = des_quat.z();
		(*att_thrust)[4] = des_quat.w();
	}

	void VelController::setOdometry(const EigenOdometry& odometry){
		odometry_ = odometry;
	}

	void VelController::setVelo(Eigen::Vector4d* vel_yaw) {
		// ROS_INFO_STREAM("vel" << vel_yaw->z());
		com_vel_ << vel_yaw->x(), vel_yaw->y(), vel_yaw->z();
		com_yaw_rate_ = vel_yaw->w();
		// ROS_INFO_STREAM("com_yaw_rate_" << com_yaw_rate_);
		controller_active_ = true;
	}


}