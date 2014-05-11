/*
 * GenerateGrasps.cpp
 *
 *  Created on: Dec 14, 2012
 *      Author: Rui P. Figueiredo
 */

#include "ros_generate_grasps/ros_generate_grasps.h"

const std::vector<std::string> RosGenerateGrasps::parameters_name=boost::assign::list_of
("FFJ1") // First
("FFJ2")
("FFJ3")
("FFJ4")
("MFJ1") // Middle
("MFJ2")
("MFJ3")
("MFJ4")
("RFJ1") // Ring
("RFJ2")
("RFJ3")
("RFJ4")
("LFJ1") // Little
("LFJ2")
("LFJ3")
("LFJ4")
("THJ1") // Thumb
("THJ2")
("THJ3")
("THJ4")
("THJ5");

RosGenerateGrasps::RosGenerateGrasps(ros::NodeHandle n_) : n(n_), n_priv("~")
{

	// Size id + shape id to object id
	size_shape_object_map=boost::assign::map_list_of
	(std::pair<int,int>( 1, 1), 1)
	(std::pair<int,int>( 1, 2), 2)
	(std::pair<int,int>( 1, 3), 3)
	(std::pair<int,int>( 1, 4), 4)
	(std::pair<int,int>( 2, 2), 5)
	(std::pair<int,int>( 2, 4), 6)
	(std::pair<int,int>( 3, 2), 7)
	(std::pair<int,int>( 3, 4), 8)
	(std::pair<int,int>( 4, 3), 9)
	(std::pair<int,int>( 4, 4),10)
	(std::pair<int,int>( 5, 4),11)
	(std::pair<int,int>( 6, 1),12)
	(std::pair<int,int>( 6, 2),13)
	(std::pair<int,int>( 6, 3),14)
	(std::pair<int,int>( 6, 4),15)
	(std::pair<int,int>( 7, 3),16)
	(std::pair<int,int>( 7, 4),17)
	(std::pair<int,int>( 8, 2),18)
	(std::pair<int,int>( 8, 4),19)
	(std::pair<int,int>( 9, 3),20)
	(std::pair<int,int>( 9, 4),21)
	(std::pair<int,int>(10, 1),22)
	(std::pair<int,int>(10, 2),23)
	(std::pair<int,int>(10, 3),24)
	(std::pair<int,int>(10, 4),25);

	// Number of parts to id
	msgs_size_handle_size_map=boost::assign::map_list_of
	(1,1)
	(2,2)
	(3,3)
	(4,4)
	(6,5)
	(8,6)
	(9,7)
	(12,8)
	(18,9)
	(26,10);
	////////////////
	// Parameters //
	////////////////

	double grasp_offset_distance;
	double pre_grasp_offset_distance;

	n_priv.param<double>("grasp_offset_distance", grasp_offset_distance, 0.05);
	ROS_INFO("grasp offset distance: %f", grasp_offset_distance);

	n_priv.param<double>("pre_grasp_offset_distance", pre_grasp_offset_distance, 0.10);
	ROS_INFO("pre grasp offset distance: %f", pre_grasp_offset_distance);

	grasps_generator = boost::shared_ptr<GenerateGrasps> (new GenerateGrasps(grasp_offset_distance, pre_grasp_offset_distance));

	// Generate grasps service
  	service = n.advertiseService("ist_generate_grasps", &RosGenerateGrasps::serviceCallback,this);
}

RosGenerateGrasps::~RosGenerateGrasps()
{
	// TODO Auto-generated destructor stub
}

bool RosGenerateGrasps::serviceCallback(ist_generate_grasps_msgs::GetGeneratedGrasps::Request  &req, ist_generate_grasps_msgs::GetGeneratedGrasps::Response &res)
{
	ROS_INFO("Generate grasps...");

	Eigen::Vector3d dimensions(req.ellipsoid.data.type.size.values.x, req.ellipsoid.data.type.size.values.y, req.ellipsoid.data.type.size.values.z);

	std::vector<boost::shared_ptr<Grasp>, Eigen::aligned_allocator<Grasp> > grasps=grasps_generator->generateGrasps(dimensions);

	int shape_index=req.ellipsoid.data.type.shape.value;
	int size_index=msgs_size_handle_size_map.find(req.ellipsoid.data.type.size.id)->second;
	unsigned int object_id=size_shape_object_map.find(std::pair<int,int>(size_index,shape_index))->second;


	//res.grasps.resize(grasps.size());
	for(size_t i=0; i < grasps.size(); ++i)
	{

		if(grasps[i]->object_type->id!=object_id)
		{
			////////////////
			// GRIP STATE //
			////////////////
			handle_msgs::GripState grip_state;

			grip_state.grip_pose=fillGripPoseMessage(grasps[i]->grip);

			////////////////
			// HAND STATE //
			////////////////

			grip_state.hand_state=fillHandStateMessage(grasps[i]);

			/////////////////////////
			// SUCCESS PROBABILITY //
			/////////////////////////

			grip_state.success_probability=0.1;
			res.grasps.push_back(grip_state);
		}
	}

	ROS_INFO("Done.");

	return true;
}

handle_msgs::GripPose RosGenerateGrasps::fillGripPoseMessage(const Grip & grip)
{
	handle_msgs::GripPose msg;

	// Grip pose (relatively to the object frame)
	geometry_msgs::PoseStamped grip_pose_msg;
	tf::poseEigenToMsg(grip.pose, grip_pose_msg.pose);
	msg.pose=grip_pose_msg;

	// Object part
	msg.part.id=0; // 0 ANY_PART, 1 CENTER, etc

	// Grip distance
	msg.distance.id=0; // 0 ANY_DISTANCE, 1 NEAR, etc NEAR=2cm FAR=4cm VERY FAR=6cm

	// Grip direction type (discrete)
	msg.direction.id=grip.canonical_grip->type;

	// Grip direction (continuous)
	msg.direction.vector.x=grip.canonical_grip->direction.x();
	msg.direction.vector.y=grip.canonical_grip->direction.y();
	msg.direction.vector.z=grip.canonical_grip->direction.z();

	return msg;
}

handle_msgs::HandState RosGenerateGrasps::fillHandStateMessage(boost::shared_ptr<Grasp> grasp)
{
	handle_msgs::HandState msg;

	///////////
	// GRASP //
	///////////

	// Grasp hand pose (relatively to the object frame)
	geometry_msgs::PoseStamped grasp_pose_msg;
	tf::poseEigenToMsg(grasp->pose, grasp_pose_msg.pose);
	msg.grasp_pose.pose=grasp_pose_msg;

	// Grasp hand posture (joint space)
	msg.grasp_posture.joints.name.resize(grasp->canonical_grasp->closed_hand_posture.joint_states.size());
	msg.grasp_posture.joints.position.resize(grasp->canonical_grasp->closed_hand_posture.joint_states.size());
	for(size_t i=0; i < grasp->canonical_grasp->closed_hand_posture.joint_states.size(); ++i)
	{
		// Grasp joint names
		msg.grasp_posture.joints.name[i]=grasp->canonical_grasp->closed_hand_posture.joint_states[i].name;

		// Grasp joint angles
		msg.grasp_posture.joints.position[i]=grasp->canonical_grasp->closed_hand_posture.joint_states[i].position;
	}

	// Grasp hand posture (synergy space)
	// NONE

	// Grasp type
	msg.grasp_posture.grasp_type=grasp->canonical_grasp->number;

	///////////////
	// PRE GRASP //
	///////////////

	// Pre grasp hand pose (relatively to the object frame)
	geometry_msgs::PoseStamped grasp_pre_pose_msg;
	tf::poseEigenToMsg(grasp->pre_pose, grasp_pre_pose_msg.pose);
	msg.pregrasp_pose.pose=grasp_pre_pose_msg;

	// Pre grasp hand posture (joint space)
	msg.pregrasp_posture.joints.name.resize(grasp->canonical_grasp->open_hand_posture.joint_states.size());
	msg.pregrasp_posture.joints.position.resize(grasp->canonical_grasp->open_hand_posture.joint_states.size());
	for(size_t i=0; i < grasp->canonical_grasp->open_hand_posture.joint_states.size(); ++i)
	{
		// Pre grasp joint names
		msg.pregrasp_posture.joints.name[i]=grasp->canonical_grasp->open_hand_posture.joint_states[i].name;

		// Grasp joint angles
		msg.pregrasp_posture.joints.position[i]=grasp->canonical_grasp->open_hand_posture.joint_states[i].position;
	}

	// Pre grasp hand posture (synergy space)
	// NONE

	// Pre-grasp type (same as grasp?)
	msg.pregrasp_posture.grasp_type=grasp->canonical_grasp->number;

	return msg;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "ist_generate_grasps");
	ros::NodeHandle n;

	RosGenerateGrasps rosGenerateGrasps(n);

	ros::spin();
}

