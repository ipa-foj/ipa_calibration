/****************************************************************
 *
 * Copyright (c) 2021
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: ipa_calibration
 * ROS stack name: ipa_calibration
 * ROS package name: ipa_calibration_interface
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Marc Riedlinger, email: m.riedlinger@live.de
 * Author: Florian Jordan, email: florian.jordan@ipa.fraunhofer.de
 *
 * Date of creation: July 2021
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <ipa_calibration_interface/dekonbot_interface.h>
#include <ipa_manipulation_msgs/JointsMovement.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/JointTrajectory.h>


DekonbotInterface::DekonbotInterface(ros::NodeHandle* nh, CalibrationType* calib_type, CalibrationMarker* calib_marker, bool do_arm_calibration, bool load_data) :
				IPAInterface(nh, calib_type, calib_marker, do_arm_calibration, load_data), arm_state_current_(0)
{
	std::cout << "\n========== DeKonbot Interface Parameters ==========\n";

	// Adjust here: Add all needed code in here to let robot move itself, its camera and arm.
	arm_state_current_ = new sensor_msgs::JointState;
	if ( arm_state_current_ != 0 )
	{
		node_handle_.param<std::string>("arm_joint_state_topic", arm_joint_state_topic_, "");
		std::cout << "arm_joint_state_topic: " << arm_joint_state_topic_ << std::endl;
		arm_state_ = node_handle_.subscribe<sensor_msgs::JointState>(arm_joint_state_topic_, 0, &DekonbotInterface::armStateCallback, this);
		node_handle_.param<std::string>("arm_joint_controller_command", arm_joint_control_topic_, "");
		std::cout << "arm_joint_controller_command: " << arm_joint_control_topic_ << std::endl;
		arm_movement_client_ = node_handle_.serviceClient<ipa_manipulation_msgs::JointsMovement>(arm_joint_control_topic_);
		if (!arm_movement_client_.waitForExistence(ros::Duration(5.0)))
		{
			ROS_ERROR("DekonbotInterface::DekonbotInterface - arm control service server doesn't exist, please start 'ipa_arm_planning' node");
			return;
		}

	}
	else
	{
		ROS_ERROR("DekonbotInterface::DekonbotInterface - Could not create current camera state storage!");
		return;
	}

	if ( arm_calibration_ )
	{
		ROS_ERROR("DekonbotInterface::DekonbotInterface - Arm Calibration not supported!");
		return;
	}
	else
	{
		node_handle_.param<std::string>("base_controller_topic_name", base_controller_topic_name_, "");
		std::cout << "base_controller_topic_name: " << base_controller_topic_name_ << std::endl;
		base_controller_ = node_handle_.advertise<geometry_msgs::Twist>(base_controller_topic_name_, 1, false);
	}
	ros::Duration(2.0).sleep();
	ROS_INFO("DekonbotInterface::DekonbotInterface - DekonbotInterface initialized.");
}

DekonbotInterface::~DekonbotInterface()
{
	if ( arm_state_current_ != 0 )
		delete arm_state_current_;
}


// CAMERA CALIBRATION INTERFACE

//Callbacks - User defined
void DekonbotInterface::armStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	boost::mutex::scoped_lock lock(arm_state_data_mutex_);
	*arm_state_current_ = *msg;
}
// End Callbacks

void DekonbotInterface::assignNewRobotVelocity(geometry_msgs::Twist new_velocity) // Spin and move velocities
{
	// In current DeKonBot setup, arm is placed 180 degrees rotated (along z axis), so change sign of linear velocity
	new_velocity.linear.x *= -1.0;
	base_controller_.publish(new_velocity);
}

void DekonbotInterface::assignNewCameraAngles(const std::string &camera_name, std_msgs::Float64MultiArray new_camera_angles)
{
	// Assign new arm joint values
	ipa_manipulation_msgs::JointsMovementRequest service_request;
	if ( camera_name.compare("spedal_920pro")==0)
		service_request.name = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
	else
	{
		ROS_ERROR("DekonbotInterface::assignNewCameraAngles - Invalid camera name %s, cannot move anything!", camera_name.c_str());
		return;
	}
	service_request.values = new_camera_angles.data;
	// HACK: Change order of values, to match UR driver call
	service_request.values[0] = new_camera_angles.data[2];
	service_request.values[2] = new_camera_angles.data[0];
	service_request.values_type = 0; // radians
	service_request.movement_type = 0;
	service_request.update_states = true;
	std::cout << service_request << std::endl;
	ipa_manipulation_msgs::JointsMovementResponse service_response;
	arm_movement_client_.call(service_request, service_response);
}

std::vector<double>* DekonbotInterface::getCurrentCameraState(const std::string &camera_name)
{
	boost::mutex::scoped_lock lock(arm_state_data_mutex_);
	return &arm_state_current_->position;
}
// END CALIBRATION INTERFACE


// ARM CALIBRATION INTERFACE
void DekonbotInterface::assignNewArmJoints(const std::string &arm_name, std_msgs::Float64MultiArray new_arm_config)
{
	// Adjust here: Assign new joints to your robot arm
	ROS_ERROR("DekonbotInterface::DekonbotInterface - Arm Calibration not supported!");
}

std::vector<double>* DekonbotInterface::getCurrentArmState(const std::string &arm_name)
{
	boost::mutex::scoped_lock lock(arm_state_data_mutex_);
	return &arm_state_current_->position;
}
// END

std::string DekonbotInterface::getRobotName()
{
	return "DeKonBot";
}




