/*
 * ros_generate_grasps.h
 *
 *  Created on: Dec 14, 2012
 *      Author: Rui P. Figueiredo
 */

#ifndef ROSGENERATEGRASPS_H_
#define ROSGENERATEGRASPS_H_

#include <ros/ros.h>
#include <ist_generate_grasps_msgs/GetGeneratedGrasps.h>
#include <handle_msgs/GripState.h>
#include <eigen_conversions/eigen_msg.h>
#include "generate_grasps/generate_grasps.h"

#define EIGEN_DONT_ALIGN_STATICALLY
#define EIGEN_USE_NEW_STDVECTOR
#include <Eigen/StdVector>

class RosGenerateGrasps
{
	private:
		// The node handler
		ros::NodeHandle n;

		// The private node handler
		ros::NodeHandle n_priv;

		// Simple object recognition service
		ros::ServiceServer service;

		// List containing all possible canonical grips
		static const std::vector<std::string> parameters_name;

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		// Constructors
		RosGenerateGrasps();

		RosGenerateGrasps(ros::NodeHandle n_);

		// Destructor
		virtual ~RosGenerateGrasps();

		// Service call back
		bool serviceCallback(ist_generate_grasps_msgs::GetGeneratedGrasps::Request  &req, ist_generate_grasps_msgs::GetGeneratedGrasps::Response &res);

		// Method used to fill handle_msgs/GripPose.msg
		handle_msgs::GripPose fillGripPoseMessage(const Grip & grip);

		// Method used to fill handle_msgs/HandState.msg
		handle_msgs::HandState fillHandStateMessage(boost::shared_ptr<Grasp> grasp);

		boost::shared_ptr<GenerateGrasps> grasps_generator;
};

#endif /* ROSGENERATEGRASPS_H_ */
