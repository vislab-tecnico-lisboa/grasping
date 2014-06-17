/*
 * ros_generate_grasps_action_server.cpp
 *
 *  Created on: Jan 8, 2013
 *      Author: Rui P. Figueiredo
 */

#include "ros_generate_grasps_action_server/ros_generate_grasps_action_server.h"
#include <time.h>
RosGenerateGraspsActionServer::RosGenerateGraspsActionServer(std::string name) :  n_priv("~"), as_(nh_, name, boost::bind(&RosGenerateGraspsActionServer::executeCB, this, _1), false), action_name_(name)
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
	(0,0)
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


	/////////////////////
	// Service clients //
	/////////////////////

	generate_grasps_new_experiment_service_ = nh_.advertiseService("update_gaussian_processes_with_new_experiment_service", &RosGenerateGraspsActionServer::updateGaussianProcessesCallback,this);

	// Babbling Process_List service client
//	ros::service::waitForService("/Process_List",ros::Duration(10));
//	babbling_processing_list_client_ = nh_.serviceClient<ist_babbling::Process_List> ("Process_List", true);
//
//	// Babbling New_Process service client
//	ros::service::waitForService("/New_Process",ros::Duration(10));
//	babbling_new_process_client_ = nh_.serviceClient<ist_babbling::New_Process> ("New_Process", true);
//
//
//	// Babbling Find_Best service client
//	ros::service::waitForService("/Find_Best",ros::Duration(10));
//	babbling_find_best_client_ = nh_.serviceClient<ist_babbling::Find_Best> ("Find_Best", true);
//
//	// Babbling Plan service client
//	ros::service::waitForService("/Plan",ros::Duration(10));
//	babbling_plan_client_ = nh_.serviceClient<ist_babbling::Plan> ("Plan", true);
//
//	// Babbling Learn service client
//	babbling_learn_client_ = nh_.serviceClient<ist_babbling::Learn> ("Learn", true);

	///////////////////////////////////////////
	// Load parameters data from config file //
	///////////////////////////////////////////

	ROS_INFO("Global parameters: ");
	n_priv.param<std::string>("predict_mode", predict_mode, "Plan");
	ROS_INFO(" Predict mode: %s", predict_mode.c_str());

	n_priv.param<bool>("learn", learn, true);
	ROS_INFO(" Learn: %d", learn);
	XmlRpc::XmlRpcValue palm_base_to_palm_center_offset_x;
	n_priv.getParam("palm_base_to_palm_center_offset_x", palm_base_to_palm_center_offset_x);
	palm_center_offset_x=(double)palm_base_to_palm_center_offset_x;

	XmlRpc::XmlRpcValue palm_base_to_palm_center_offset_y;
	n_priv.getParam("palm_base_to_palm_center_offset_y", palm_base_to_palm_center_offset_y);
	palm_center_offset_y=(double)palm_base_to_palm_center_offset_y;

	XmlRpc::XmlRpcValue palm_base_to_palm_center_offset_z;
	n_priv.getParam("palm_base_to_palm_center_offset_z", palm_base_to_palm_center_offset_z);
	palm_center_offset_z=(double)palm_base_to_palm_center_offset_z;

	ROS_INFO_STREAM(" Palm base to palm center offset number: " << palm_base_to_palm_center_offset_x << " " << palm_base_to_palm_center_offset_y << " " << palm_base_to_palm_center_offset_z);

	XmlRpc::XmlRpcValue palm_to_tip_distance;
	n_priv.getParam("palm_to_tip_distance", Grip::palm_to_tip_distance);
	ROS_INFO_STREAM(" Palm to tip distance: " << Grip::palm_to_tip_distance);



	XmlRpc::XmlRpcValue gaussian_process_parameters;
	n_priv.getParam("gaussian_process_parameters", gaussian_process_parameters);

	ROS_INFO_STREAM(" Gaussian process parameters number: " << gaussian_process_parameters);

	////////////////////////////
	// Load object part types //
	////////////////////////////

	XmlRpc::XmlRpcValue considered_object_part_types;
	n_priv.getParam("object_part_types", considered_object_part_types);

	ROS_INFO_STREAM("Loading object part types: " << considered_object_part_types.size());

	for(int32_t i=0; i < considered_object_part_types.size(); ++i)
	{
        if(considered_object_part_types[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
			int id = considered_object_part_types[i]["id"];
			std::string name = considered_object_part_types[i]["name"];

			std::cout << "id: " << id << std::endl;
			std::cout << "name: " << name << std::endl;

			std::cout << std::endl;

			// Insert new object part type
			boost::shared_ptr<CanonicalObjectPart> object_part_type(new CanonicalObjectPart(id, name));
			CanonicalObjectPart::canonical_object_parts.insert(std::pair<unsigned int, boost::shared_ptr<CanonicalObjectPart> > (id, object_part_type));
        }
	}

	///////////////////////
	// Load object types //
	///////////////////////

	XmlRpc::XmlRpcValue considered_object_types;
	n_priv.getParam("object_types", considered_object_types);

	ROS_INFO_STREAM("Loading object types: " << considered_object_types.size());

	for(int32_t i=0; i < considered_object_types.size(); ++i)
	{
        if(considered_object_types[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
        	if(considered_object_types[i]["use"])
        	{
				int id = considered_object_types[i]["id"];
				std::string name = considered_object_types[i]["name"];

				std::cout << "id: " << id << std::endl;
				std::cout << "name: " << name << std::endl;

				std::cout << std::endl;

				XmlRpc::XmlRpcValue object_part_types=considered_object_types[i]["object_parts"];


				boost::shared_ptr<ObjectType> object_type(new ObjectType(id, name));


				for(int32_t object_part_type_index=0; object_part_type_index < object_part_types.size(); ++object_part_type_index)
				{
					int part_type_id;
                    part_type_id=static_cast<int>(object_part_types[object_part_type_index]["id"]);

					boost::shared_ptr<ObjectPart> new_part(new ObjectPart(part_type_id));
					object_type->object_parts.push_back(new_part);

				}

				// Insert new object type
				ObjectType::object_types.insert(std::pair<unsigned int, boost::shared_ptr<ObjectType> > (id, object_type));
        	}
        }
	}

	std::cout << std::endl << std::endl;

	//////////////////////////
	// Load canonical grips //
	//////////////////////////

	XmlRpc::XmlRpcValue considered_canonical_grips;
	n_priv.getParam("canonical_grips", considered_canonical_grips);

	ROS_INFO_STREAM("Loading canonical grips: " << considered_canonical_grips.size());

	for(int32_t i=0; i < considered_canonical_grips.size(); ++i)
	{
        if(considered_canonical_grips[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          if(considered_canonical_grips[i].hasMember("id") && considered_canonical_grips[i].hasMember("name") && considered_canonical_grips[i].hasMember("use"))
          {
        	  if(considered_canonical_grips[i]["use"])
        	  {
        		  int id = considered_canonical_grips[i]["id"];
        		  std::string name = considered_canonical_grips[i]["name"];
        		  Eigen::Matrix<double, 3 ,1> grip_direction(considered_canonical_grips[i]["direction"]["x"],considered_canonical_grips[i]["direction"]["y"],considered_canonical_grips[i]["direction"]["z"]);
        		  XmlRpc::XmlRpcValue rotation_matrix=considered_canonical_grips[i]["rotation_matrix"];
        		  Eigen::Matrix<double, 3 ,3> rotation_matrix_eigen;
        		  rotation_matrix_eigen << (double)rotation_matrix[0],(double)rotation_matrix[3],(double)rotation_matrix[6],
        				  	     	 	   (double)rotation_matrix[1],(double)rotation_matrix[4],(double)rotation_matrix[7],
        				  	     	 	   (double)rotation_matrix[2],(double)rotation_matrix[5],(double)rotation_matrix[8];

        		  std::cout << "id: " << id << std::endl;
        		  std::cout << "name: " << name << std::endl;

        		  Eigen::Transform<double, 3, Eigen::Affine> orientation(rotation_matrix_eigen);
        		  boost::shared_ptr<CanonicalGrip> canonical_grip(new CanonicalGrip(id,name,grip_direction,orientation));
        		  CanonicalGrip::canonical_grips.insert(std::pair<unsigned int, boost::shared_ptr<CanonicalGrip> >(id, canonical_grip));
        	  }
          }
        }
	}

	//////////////////////
	// Load grasp types //
	//////////////////////

	XmlRpc::XmlRpcValue considered_grasp_types;
	n_priv.getParam("grasp_types", considered_grasp_types);

	ROS_INFO_STREAM("Loading grasp types: " << considered_grasp_types.size());

	for(int32_t i=0; i < considered_grasp_types.size(); ++i)
	{
        if(considered_grasp_types[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
        	if(considered_grasp_types[i]["use"])
        	{
				int id = considered_grasp_types[i]["id"];
				std::string name = considered_grasp_types[i]["name"];

				std::cout << "id: " << id << std::endl;
				std::cout << "name: " << name << std::endl;

				std::cout << std::endl;

				// Insert new grasp type
				GraspType::grasp_types.push_back(GraspType(id,name));
        	}
        }
	}

	std::cout << std::endl << std::endl;

	///////////////////////////////////////
	// Load canonical reach types grasps //
	///////////////////////////////////////

	XmlRpc::XmlRpcValue considered_reach_types;
	n_priv.getParam("reach_types", considered_reach_types);

	ROS_INFO_STREAM("Loading reach types: " << considered_reach_types.size());

	for(int32_t i=0; i < considered_reach_types.size(); ++i)
	{
        if(considered_reach_types[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
            if(considered_reach_types[i].hasMember("id") && considered_reach_types[i].hasMember("name"))
            {
            	if(considered_reach_types[i]["use"])
            	{
            		int id = considered_reach_types[i]["id"];
            		std::string name = considered_reach_types[i]["name"];

            		double minimum_distance = considered_reach_types[i]["minimum_distance"];
            		double maximum_distance = considered_reach_types[i]["maximum_distance"];
            		Eigen::Vector3d direction(considered_reach_types[i]["approach_direction"]["x"], considered_reach_types[i]["approach_direction"]["y"], considered_reach_types[i]["approach_direction"]["z"]);
            		std::cout << "id: " << id << std::endl;
            		std::cout << "name: " << name << std::endl;
            		std::cout << "minimum distance: " << minimum_distance << std::endl;
            		std::cout << "maximum distance: " << maximum_distance << std::endl;
            		std::cout << "direction: " << direction.transpose() << std::endl;
            		std::cout << std::endl;

            		// Insert new reach type
            		boost::shared_ptr<ReachType> reach_type(new ReachType(id, name, minimum_distance, maximum_distance, direction));
            		ReachType::reach_types.insert(std::pair<unsigned int, boost::shared_ptr<ReachType> > (id, reach_type));
          	  }
            }
        }
	}

	std::cout << std::endl << std::endl;

	///////////////////////////
	// Load canonical grasps //
	///////////////////////////

	XmlRpc::XmlRpcValue considered_canonical_grasps;
	n_priv.getParam("canonical_grasps", considered_canonical_grasps);

	ROS_INFO_STREAM("Loading canonical grasps: " << considered_canonical_grasps.size());

	for(int32_t i=0; i < considered_canonical_grasps.size(); ++i)
	{
        if(considered_canonical_grasps[i].getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          if(considered_canonical_grasps[i].hasMember("id") && considered_canonical_grasps[i].hasMember("name") && considered_canonical_grasps[i].hasMember("use") && considered_canonical_grasps[i].hasMember("palm_offset"))
          {
        	  if(considered_canonical_grasps[i]["use"])
        	  {
        		  int id = considered_canonical_grasps[i]["id"];

        		  int type = considered_canonical_grasps[i]["grasp_type"];

        		  std::string name = considered_canonical_grasps[i]["name"];
        		  double pre_grasp_offset_distance = considered_canonical_grasps[i]["pre_grasp_offset_distance"];

        		  double palm_canonical_orientation_angle=((double)considered_canonical_grasps[i]["palm_canonical_orientation"]["angle"])*((double)DEG_TO_RAD);
        		  Eigen::Vector3d palm_canonical_orientation_axis(considered_canonical_grasps[i]["palm_canonical_orientation"]["axis"]["x"],considered_canonical_grasps[i]["palm_canonical_orientation"]["axis"]["y"],considered_canonical_grasps[i]["palm_canonical_orientation"]["axis"]["z"]);
				  Eigen::AngleAxisd palm_canonical_orientation(palm_canonical_orientation_angle, palm_canonical_orientation_axis);

				  Eigen::Translation3d palm_position_offset(considered_canonical_grasps[i]["palm_offset"]["position"]["x"],considered_canonical_grasps[i]["palm_offset"]["position"]["y"],considered_canonical_grasps[i]["palm_offset"]["position"]["z"]);
				  double palm_orientation_offset_angle=((double)considered_canonical_grasps[i]["palm_offset"]["orientation"]["angle"])*((double)DEG_TO_RAD);
				  Eigen::Vector3d palm_orientation_offset_axis(considered_canonical_grasps[i]["palm_offset"]["orientation"]["axis"]["x"],considered_canonical_grasps[i]["palm_offset"]["orientation"]["axis"]["y"],considered_canonical_grasps[i]["palm_offset"]["orientation"]["axis"]["z"]);
				  Eigen::AngleAxisd palm_orientation_offset(palm_orientation_offset_angle, palm_orientation_offset_axis);
				  Eigen::Transform<double, 3, Eigen::Affine> palm_pose_offset = palm_position_offset * palm_orientation_offset;


				  double perturbation_angle_step= ((double)considered_canonical_grasps[i]["orientation_perturbation"]["angle_step"])*((double)DEG_TO_RAD);
				  unsigned int n_perturbs = (int)considered_canonical_grasps[i]["orientation_perturbation"]["n_perturbs"];
				  Eigen::Vector3d perturbation_rotation_axis(considered_canonical_grasps[i]["orientation_perturbation"]["axis"]["x"],considered_canonical_grasps[i]["orientation_perturbation"]["axis"]["y"],considered_canonical_grasps[i]["orientation_perturbation"]["axis"]["z"]);

        		  std::cout << "id: " << id << std::endl;
        		  std::cout << "type: " << type << std::endl;

        		  std::cout << "name: " << name << std::endl;
        		  std::cout << "pre grasp offset distance: " << pre_grasp_offset_distance << std::endl;

        		  std::cout << "palm_canonical_orientation: " << std::endl;
        		  std::cout << " angle (degs): " << considered_canonical_grasps[i]["palm_canonical_orientation"]["angle"] << " rads: " << palm_canonical_orientation_angle;
        		  std::cout << " axis: " << palm_canonical_orientation_axis.transpose() << std::endl;

        		  std::cout << "palm_orientation_offset: " << std::endl;
        		  std::cout << " angle (degs): " << considered_canonical_grasps[i]["palm_offset"]["orientation"]["angle"] << " rads: " << palm_orientation_offset_angle;
        		  std::cout << " axis: " << palm_orientation_offset_axis.transpose() << std::endl;
        		  std::cout << "palm_position_offset: " << palm_position_offset.x() << " "<< palm_position_offset.y() << " " << palm_position_offset.z() << std::endl;

        		  std::cout << "orientation_perturbation: " << std::endl;
        		  std::cout << " angle_step (degs): " << considered_canonical_grasps[i]["orientation_perturbation"]["angle_step"] << " rads: " << perturbation_angle_step;
        		  std::cout << " n_perturbs: " << n_perturbs << std::endl;
        		  std::cout << " axis: " << perturbation_rotation_axis.transpose() << std::endl;


				  XmlRpc::XmlRpcValue open_posture_joint_angles=considered_canonical_grasps[i]["open_hand_posture"]["joint_angles"];
				  HandPosture open_hand_posture;
				  std::cout << "open_hand_posture:" << std::endl;
				  for(int32_t open_joint_angles_index=0; open_joint_angles_index < open_posture_joint_angles.size(); ++open_joint_angles_index)
				  {
					  JointState joint_state;
					  joint_state.name=static_cast<std::string>(open_posture_joint_angles[open_joint_angles_index]["name"]);
					  joint_state.position=static_cast<double>(open_posture_joint_angles[open_joint_angles_index]["position"])*((double)DEG_TO_RAD);

					  open_hand_posture.joint_states.push_back(joint_state);
				  }


				  XmlRpc::XmlRpcValue closed_posture_joint_angles=considered_canonical_grasps[i]["closed_hand_posture"]["joint_angles"];
				  HandPosture closed_hand_posture;
				  std::cout << "closed_hand_posture:" << std::endl;
				  for(int32_t closed_joint_angles_index=0; closed_joint_angles_index < closed_posture_joint_angles.size(); ++closed_joint_angles_index)
				  {
					  JointState joint_state;
					  joint_state.name=static_cast<std::string>(closed_posture_joint_angles[closed_joint_angles_index]["name"]);
					  joint_state.position=static_cast<double>(closed_posture_joint_angles[closed_joint_angles_index]["position"])*((double)DEG_TO_RAD);

					  closed_hand_posture.joint_states.push_back(joint_state);
				  }

        		  std::cout << std::endl;

        		  // Insert new canonical grasp
        		  boost::shared_ptr<CanonicalGrasp> canonical_grasp(new CanonicalGrasp(type,id,name,palm_canonical_orientation, palm_pose_offset, open_hand_posture, closed_hand_posture, pre_grasp_offset_distance, perturbation_angle_step, n_perturbs, perturbation_rotation_axis));
        		  CanonicalGrasp::canonical_grasps.insert(std::pair<unsigned int, boost::shared_ptr<CanonicalGrasp> >(id, canonical_grasp));
        	  }
          }
        }
	}

	////////////////////////////////////////////////////////////////////////
	// Initialize discrete grasps with continuous from configuration file //
	////////////////////////////////////////////////////////////////////////

	graspability_states=boost::shared_ptr<GenerateGrasps> (new GenerateGrasps());


	//////////////////////////////////////////////////////////////////
	// Get available Gaussian processes and create if not available //
	//////////////////////////////////////////////////////////////////

	//sleep(15);
//	ROS_INFO("Get babbling process list...");
//	if(babbling_processing_list_client_.call(srv_BabblingProcessList))
//	{
//		// Add existing gaussian processes to dictionary
//		for(unsigned int i=0; i < srv_BabblingProcessList.response.List.size(); ++i)
//		{
//			grasp_to_process_name.insert(std::pair<std::string, bool>(srv_BabblingProcessList.response.List[i],true));
//		}
//
//		for(std::map<unsigned int, boost::shared_ptr<Grasp> >::iterator graspability_states_it=graspability_states->generated_grasps.begin(); graspability_states_it!=graspability_states->generated_grasps.end(); ++graspability_states_it)
//		{
//			std::stringstream ss;
//			ss << graspability_states_it->second->id;
//
//			// Add grasp to dictionary
//			graspability_state_gaussian_process_bimap.insert( boost::bimap<unsigned int, std::string>::value_type(graspability_states_it->second->id, ss.str() ) );
//			//boost::bimap<unsigned int,std::string>::left_const_iterator graspability_gp_it=graspability_state_gaussian_process_bimap.left.find(graspability_states_it->second->id);
//
//			if(grasp_to_process_name.find(ss.str())==grasp_to_process_name.end())
//			{
//				ROS_INFO("Gaussian Process with name %s does not exist.", ss.str().c_str());
//				ist_babbling::ProcessInfo process_info_msg;
//				process_info_msg.numParam=gaussian_process_parameters;
//				process_info_msg.description=ss.str();
//				process_info_msg.user_defined_name=ss.str();
//				srv_BabblingNewProcess.request.ProcessInfo=process_info_msg;
//				if(babbling_new_process_client_.call(srv_BabblingNewProcess))
//				{
//					ROS_INFO("Gaussian process with name %s created.", ss.str().c_str());
//				}
//				else
//				{
//					ROS_INFO("Failed to call babbling new process service.");
//				}
//			}
//			else
//			{
//				ROS_INFO("Available process name %s: ", ss.str().c_str());
//			}
//		}
//		ROS_INFO("Done.");
//	}
//	else
//	{
//		ROS_ERROR("Failed to call %s service.", babbling_processing_list_client_.getService().c_str());
//	}

// Debug tests
//
//	double _yaw, _pitch, _roll, _x, _y, _z, _reach_distance, _object_x, _object_y, _object_z;
//	graspability_states->generated_grasps.begin()->second->getNormalizedParameters(_yaw, _pitch, _roll, _x, _y, _z, _reach_distance, _object_x, _object_y, _object_z);
//	std::cout << _yaw << " " << _pitch << " " << _roll << " " << _x << " " << _y << " " << _z << " " << _reach_distance << " " << _object_x << " " << _object_y << " " << _object_z << std::endl;
	as_.start();
}

RosGenerateGraspsActionServer::~RosGenerateGraspsActionServer(void)
{

}

// Close feedback loop
bool RosGenerateGraspsActionServer::updateGaussianProcessesCallback(ist_grasp_generation_msgs::NewExperiment::Request & req, ist_grasp_generation_msgs::NewExperiment::Response & res)
{
	if(!learn)
	{
		ROS_INFO("Not learning.");
		return true;
	}

//	unsigned int grasp_id=req.grasp.hand_state.grasp_posture.grasp_type;
//	unsigned int grip_id=req.grasp.grip_pose.direction.id;
//	//unsigned int part=req.grasp.grip_pose.part.id;
//	unsigned int reach_distance_id=req.grasp.grip_pose.distance.id;
//
//	unsigned int shape_id=req.object.data.type.shape.value;
//	//unsigned int symmetry=req.object.data.type.symmetry.value;
//
//	unsigned int size_id=msgs_size_handle_size_map.find(req.object.data.type.size.id)->second;
//
////	double metric=req.metric;
//
//	// Get object class
//	unsigned int object_id=size_shape_object_map.find(std::pair<int,int>(size_id,shape_id))->second;

	// Get graspability state
	//unsigned int graspability_state_id=Grasp::computeId(object_id,grasp_id,grip_id,reach_distance_id);

	////////////////////////////////////////////////
	// Update graspability state gaussian process //
	////////////////////////////////////////////////

	// Get previous normalized parameters for the experiment graspability state...
//	double _normalized_yaw;
//	double _normalized_pitch;
//	double _normalized_roll;
//	double _normalized_x;
//	double _normalized_y;
//	double _normalized_z;
//	double _normalized_reach_distance;
//	double _normalized_object_dimension_x; // static
//	double _normalized_object_dimension_y; // static
//	double _normalized_object_dimension_z; // static

//	std::map<unsigned int, boost::shared_ptr<Grasp> >::iterator graspability_state = graspability_states->generated_grasps.find(graspability_state_id);
//	graspability_state->second->getNormalizedParameters(_normalized_yaw,
//														_normalized_pitch,
//														_normalized_roll,
//														_normalized_x,
//														_normalized_y,
//														_normalized_z,
//														_normalized_reach_distance,
//														_normalized_object_dimension_x,
//														_normalized_object_dimension_y,
//														_normalized_object_dimension_z
//	);

	// Update using learning babbling service
//	boost::bimap<unsigned int,std::string>::left_const_iterator graspability_state_gaussian_process_it=graspability_state_gaussian_process_bimap.left.find(graspability_state_id);
//
//	srv_BabblingLearn.request.ProcessName=graspability_state_gaussian_process_it->second;
//	srv_BabblingLearn.request.data.Metric=metric;
//
//	srv_BabblingLearn.request.data.DynParam.push_back(_normalized_yaw);
//	srv_BabblingLearn.request.data.DynParam.push_back(_normalized_pitch);
//	srv_BabblingLearn.request.data.DynParam.push_back(_normalized_roll);
//	srv_BabblingLearn.request.data.DynParam.push_back(_normalized_x);
//	srv_BabblingLearn.request.data.DynParam.push_back(_normalized_y);
//	srv_BabblingLearn.request.data.DynParam.push_back(_normalized_z);
//	srv_BabblingLearn.request.data.DynParam.push_back(_normalized_reach_distance);
//
//
//	if(!babbling_learn_client_.call(srv_BabblingLearn))
//	{
//		ROS_ERROR("Failed to update Gaussian process using babbling Learn service.");
//		return false;
//	}
//
//	ROS_INFO("Gaussian process updated.");

	return true;
}

// Grasps generation
void RosGenerateGraspsActionServer::executeCB(const ist_grasp_generation_msgs::GetGeneratedGraspsGoalConstPtr &goal)
{
	ROS_INFO("Generate grasps...");

	// helper variables
	ros::Rate r(10);
	bool success = true;

	// start executing the action

	// push_back the seeds for the fibonacci sequence
	feedback_.progress=0;

	result_.grip_list.grip_states.clear();

	ROS_DEBUG_STREAM("object type id:" <<(int)goal->object.data.type.id);
	for(int i =0; i< goal->object.data.actionable_parts_data.size(); ++i)

	ROS_DEBUG_STREAM("object part id:"<<(int)goal->object.data.actionable_parts_data[i].part.id);// update part
	// publish info to the console for the user
//	ROS_INFO("%s: Executing, compute grasps object of with size dimensions (%f,%f,%f)", action_name_.c_str(), dimensions.x(),dimensions.y(),dimensions.z());

	unsigned int object_id=goal->object.data.type.id;
	// Predict objects with similar ID
	unsigned int object_grasps_so_far=0;


	// For each graspability state
	for(std::map<unsigned int, boost::shared_ptr<Grasp> >::iterator graspability_states_it=graspability_states->generated_grasps.begin(); graspability_states_it!=graspability_states->generated_grasps.end(); ++graspability_states_it)
	{
		// check that preempt has not been requested by the client
		if (as_.isPreemptRequested() || !ros::ok())
		{
			ROS_INFO("%s: Preempted", action_name_.c_str());
			// set the action state to preempted
			as_.setPreempted();
			success = false;
			break;
		}

//		std::cout << graspability_states_it->second->object_type->id<<" " <<object_id << std::endl;
		// If grasp associated object type is not the same... continue
		if((unsigned int)graspability_states_it->second->object_type->id!=object_id)
		{
			continue;
		}

		// FOR EACH OBJECT PART
		for(unsigned int r=0; r < goal->object.data.actionable_parts_data.size(); ++r)
		{
			if((int)graspability_states_it->second->object_part->id!=(int)goal->object.data.actionable_parts_data[r].part.id)
			{
				continue;
			}

			if(goal->object.data.actionable_parts_data[r].part.id!=2) // only use middle
			{		
				continue;
			
			} // Only generate grasps for usable object parts
			/*if(!goal->object.data.actionable_parts_data[r].part.use) // Only generate grasps for usable object parts
			{
				continue;
			}
			else
				ROS_DEBUG_STREAM("Im gonna use part" << (int)goal->object.data.actionable_parts_data[r].part.id);*/



            Eigen::Vector3d dimensions(goal->object.data.actionable_parts_data[r].part.bounding_box.x,
                                       goal->object.data.actionable_parts_data[r].part.bounding_box.y,
                                       goal->object.data.actionable_parts_data[r].part.bounding_box.z);
//			if(goal->object.data.actionable_parts_data[r].part.id!=4)
//				continue;
			Eigen::Transform<double, 3, Eigen::Affine> object_part_pose;
            tf::poseMsgToEigen(goal->object.data.actionable_parts_data[r].part.pose.pose, object_part_pose);

            // Update grasp
			graspability_states_it->second->updateGrasp(dimensions, object_part_pose);

			double _normalized_yaw;
			double _normalized_pitch;
			double _normalized_roll;
			double _normalized_x;
			double _normalized_y;
			double _normalized_z;
			double _normalized_reach_distance;
			double _normalized_object_dimension_x; // static
			double _normalized_object_dimension_y; // static
			double _normalized_object_dimension_z; // static

			std::vector<double> gaussian_process_new_parameters;
			gaussian_process_new_parameters.resize(10);

				//std::cout << "generating graspability state:"<< graspability_states_it->second->object_type->id << std::endl;
				// Get process name
	//			boost::bimap<unsigned int,std::string>::left_const_iterator graspability_gp_it=graspability_state_gaussian_process_bimap.left.find(graspability_states_it->second->id);
				if(predict_mode=="Babbling")
				{
					// Predict using babbling Plan service
	//				srv_BabblingPlan.request.ProcessName=graspability_gp_it->second;
	//				if(!babbling_plan_client_.call(srv_BabblingPlan))
	//				{
	//					ROS_ERROR("Failed to predict Gaussian process using babbling Plan service.");
	//					return;
	//				}
	//
	//				gaussian_process_new_parameters=srv_BabblingPlan.response.NewParam;
				}
				else if(predict_mode=="Exploit")
				{
					// Predict using babbling Find_Best service
	//				srv_BabblingFindBest.request.ProcessName=graspability_gp_it->second;
	//				if(!babbling_find_best_client_.call(srv_BabblingFindBest))
	//				{
	//						ROS_ERROR("Failed to predict Gaussian process using babbling Find Best service.");
	//						return;
	//				}
	//
	//				gaussian_process_new_parameters=srv_BabblingFindBest.response.BestParam;
				}
				else if(predict_mode=="Random")
				{
					gaussian_process_new_parameters[0] = ((double) rand() / (RAND_MAX));
					gaussian_process_new_parameters[1] = ((double) rand() / (RAND_MAX));
					gaussian_process_new_parameters[2] = ((double) rand() / (RAND_MAX));
					gaussian_process_new_parameters[3] = ((double) rand() / (RAND_MAX));
					gaussian_process_new_parameters[4] = ((double) rand() / (RAND_MAX));
					gaussian_process_new_parameters[5] = ((double) rand() / (RAND_MAX));
					gaussian_process_new_parameters[6] = ((double) rand() / (RAND_MAX));
					gaussian_process_new_parameters[7] = ((double) rand() / (RAND_MAX));
					gaussian_process_new_parameters[8] = ((double) rand() / (RAND_MAX));
					gaussian_process_new_parameters[9] = ((double) rand() / (RAND_MAX));
				}
				else if(predict_mode=="Canonical")
				{
					// Jump...
					gaussian_process_new_parameters[0] = 0.5;
					gaussian_process_new_parameters[1] = 0.5;
					gaussian_process_new_parameters[2] = 0.5;
					gaussian_process_new_parameters[3] = 0.5;
					gaussian_process_new_parameters[4] = 0.5;
					gaussian_process_new_parameters[5] = 0.5;
					gaussian_process_new_parameters[6] = 0.5;
					gaussian_process_new_parameters[7] = 0.5;
					gaussian_process_new_parameters[8] = 0.5;
					gaussian_process_new_parameters[9] = 0.5;
				}
				else if(predict_mode=="Manual")
				{
//					std::cout << "ENTROU MANUAL" << std::endl;
					graspability_states_it->second->getNormalizedParameters(_normalized_yaw,
																			_normalized_pitch,
																			_normalized_roll,
																			_normalized_x,
																			_normalized_y,
																			_normalized_z,
																			_normalized_reach_distance,
																			_normalized_object_dimension_x,
																			_normalized_object_dimension_y,
																			_normalized_object_dimension_z);
					gaussian_process_new_parameters[0]=_normalized_yaw;
					gaussian_process_new_parameters[1]=_normalized_pitch;
					gaussian_process_new_parameters[2]=_normalized_roll;
					gaussian_process_new_parameters[3]=_normalized_x;
					gaussian_process_new_parameters[4]=_normalized_y;
					gaussian_process_new_parameters[5]=_normalized_z;
					gaussian_process_new_parameters[6]=_normalized_reach_distance;
					gaussian_process_new_parameters[7]=_normalized_object_dimension_x;
					gaussian_process_new_parameters[8]=_normalized_object_dimension_y;
					gaussian_process_new_parameters[9]=_normalized_object_dimension_z;
				}

				// Incorporate gaussian process knowledge
				if(learn)
				{
					_normalized_yaw=				gaussian_process_new_parameters[0];
					_normalized_pitch=				gaussian_process_new_parameters[1];
					_normalized_roll=				gaussian_process_new_parameters[2];
					_normalized_x=					gaussian_process_new_parameters[3];
					_normalized_y=					gaussian_process_new_parameters[4];
					_normalized_z=					gaussian_process_new_parameters[5];
					_normalized_reach_distance=		gaussian_process_new_parameters[6];
					_normalized_object_dimension_x=	gaussian_process_new_parameters[7]; // static
					_normalized_object_dimension_y=	gaussian_process_new_parameters[8]; // static
					_normalized_object_dimension_z=	gaussian_process_new_parameters[9]; // static

					// Set graspabability state predicted parameters
					graspability_states_it->second->setParameters(_normalized_yaw,
																  _normalized_pitch,
																  _normalized_roll,
																  _normalized_x,
																  _normalized_y,
																  _normalized_z,
																  _normalized_reach_distance,
																  _normalized_object_dimension_x,
																  _normalized_object_dimension_y,
																  _normalized_object_dimension_z);
				}

				////////////////////////////////
				// Fill result output message //
				////////////////////////////////

				ist_msgs::GripState grip_state_msg;
				grip_state_msg.palm_to_tip_distance=Grip::palm_to_tip_distance; 
				grip_state_msg.object_id=goal->object.object_id;
				grip_state_msg.grip_pose= fillGripPoseMessage(graspability_states_it->second, goal->object.state.graspable_object.reference_frame_id, goal->object.data.actionable_parts_data[r].part);
				grip_state_msg.hand_state=fillHandStateMessage(graspability_states_it->second, goal->object.state.graspable_object.reference_frame_id);
				//grip_state_msg.success_probability=0.1; // IST IS 0.1

				result_.grip_list.grip_states.push_back(grip_state_msg);
				feedback_.progress=(float)(++object_grasps_so_far)/ObjectType::object_types.size();

		}
	}


	// Predict objects with similar ID
//	object_grasps_so_far=0;
//	for(std::map<unsigned int, boost::shared_ptr<GraspPerturbation> >::iterator grasps_perturbations_states_it=graspability_states->generated_grasps_perturbations.begin(); grasps_perturbations_states_it!=graspability_states->generated_grasps_perturbations.end(); ++grasps_perturbations_states_it)
//	{
//		grasps_perturbations_states_it->second->updateGraspPerturbation(dimensions);
//		// check that preempt has not been requested by the client
//		if (as_.isPreemptRequested() || !ros::ok())
//		{
//			ROS_INFO("%s: Preempted", action_name_.c_str());
//			// set the action state to preempted
//			as_.setPreempted();
//			success = false;
//			break;
//		}
//
//
//		if(grasps_perturbations_states_it->second->grasp->object_type->id==object_id)
//		{
//			std::cout << "generating graspability state:"<< grasps_perturbations_states_it->second->grasp->object_type->id << std::endl;
//
//			////////////////////////////////
//			// Fill result output message //
//			////////////////////////////////
//
//			ist_msgs::GripState grip_state_msg;
//			grip_state_msg.grip_pose= fillGripPoseMessage(grasps_perturbations_states_it->second, goal->object.state.graspable_object.reference_frame_id);
//			grip_state_msg.hand_state=fillHandStateMessage(grasps_perturbations_states_it->second, goal->object.state.graspable_object.reference_frame_id);
//			grip_state_msg.success_probability=0.1; // IST IS 0.1
//
//			result_.grip_list.grip_states.push_back(grip_state_msg);
//			feedback_.progress=(float)(++object_grasps_so_far)/ObjectType::object_types.size();
//		}
//	}

	if(success)
	{
		ROS_INFO("%s: Succeeded", action_name_.c_str());
		// set the action state to succeeded
		as_.setSucceeded(result_);
	}
	else
	{
		ROS_INFO("%s: Succeeded but no database connection", action_name_.c_str());
		as_.setSucceeded(result_);
	}
}

ist_msgs::GripPose RosGenerateGraspsActionServer::fillGripPoseMessage(const boost::shared_ptr<Grasp> grasp, const std::string & frame_id, const ist_msgs::ObjectPart & object_part)
{
	ist_msgs::GripPose msg;

	// Grip pose (relatively to the object frame)
	geometry_msgs::PoseStamped grip_pose_msg;
	tf::poseEigenToMsg(grasp->grip.pose, grip_pose_msg.pose);
	grip_pose_msg.header.frame_id=frame_id;

	msg.pose=grip_pose_msg;

	// Object part
	msg.part=object_part; // 0 ANY_PART, 1 CENTER, etc

	//ROS_INFO_STREAM("OBJECT PART IDDDDDDDDDDDDDDDDDDD: "<<grasp->object_part->id);
	// Grip reach distance type (discrete)
	msg.distance.id=grasp->reach.reach_type->id; // 0 ANY_DISTANCE, 1 NEAR, etc NEAR=2cm FAR=4cm VERY FAR=6cm

	// Grip reach distance (continuous)
	msg.distance.value=grasp->reach.distance;

	// Grip direction type (discrete)
	msg.direction.id=grasp->grip.canonical_grip->type;

	// Grip direction (continuous)
	msg.direction.vector.x=grasp->grip.canonical_grip->direction.x();
	msg.direction.vector.y=grasp->grip.canonical_grip->direction.y();
	msg.direction.vector.z=grasp->grip.canonical_grip->direction.z();

	return msg;
}

//ist_msgs::GripPose RosGenerateGraspsActionServer::fillGripPoseMessage(const boost::shared_ptr<GraspPerturbation> grasp_perturbation, const std::string & frame_id, const ist_msgs::ObjectPart & object_part)
//{
//	ist_msgs::GripPose msg;
//
//	// Grip pose (relatively to the object frame)
//	geometry_msgs::PoseStamped grip_pose_msg;
//	tf::poseEigenToMsg(grasp_perturbation->grasp->grip.pose, grip_pose_msg.pose);
//	grip_pose_msg.header.frame_id=frame_id;
//
//	msg.pose=grip_pose_msg;
//
//	// Object part
//	msg.part=object_part; // 0 ANY_PART, 1 CENTER, etc
//
//	// Grip reach distance type (discrete)
//	msg.distance.id=grasp_perturbation->grasp->reach.reach_type->id; // 0 ANY_DISTANCE, 1 NEAR, etc NEAR=2cm FAR=4cm VERY FAR=6cm
//
//	// Grip reach distance (continuous)
//	msg.distance.value=grasp_perturbation->grasp->reach.distance;
//
//	// Grip direction type (discrete)
//	msg.direction.id=grasp_perturbation->grasp->grip.canonical_grip->type;
//
//	// Grip direction (continuous)
//	msg.direction.vector.x=grasp_perturbation->grasp->grip.canonical_grip->direction.x();
//	msg.direction.vector.y=grasp_perturbation->grasp->grip.canonical_grip->direction.y();
//	msg.direction.vector.z=grasp_perturbation->grasp->grip.canonical_grip->direction.z();
//
//	return msg;
//}

ist_msgs::HandState RosGenerateGraspsActionServer::fillHandStateMessage(const boost::shared_ptr<Grasp> grasp, std::string frame_id)
{
	ist_msgs::HandState msg;

	///////////
	// GRASP //
	///////////

    // Transform from handle frame definition to ros frame definition
//	Eigen::Matrix3d handle_to_ros_palm_rotation_matrix;
//	handle_to_ros_palm_rotation_matrix << 0, 0, 1,
//								          0,-1, 0,
//								          1, 0, 0; // BOLAS È ISTO QUE ESTA A LIXAR ISTO TUDO!!!!!!!

    //ROS_INFO("valores offsets: %f, %f, %f", palm_center_offset_x,palm_center_offset_y,palm_center_offset_z);
//	Eigen::Transform<double, 3, Eigen::Affine> handle_to_ros_palm_transform=Eigen::Translation3d(palm_center_offset_x ,palm_center_offset_y , palm_center_offset_z)*Eigen::AngleAxisd(handle_to_ros_palm_rotation_matrix);

    //Eigen::Transform<double, 3, Eigen::Affine> ros_palm_to_handle_transform=handle_to_ros_palm_transform.inverse();


//	Eigen::Transform<double, 3, Eigen::Affine>	grasp_pose=grasp->pose*ros_palm_to_handle_transform;


    Eigen::Transform<double, 3, Eigen::Affine>	grasp_pose=grasp->pose;

	// Grasp hand pose (relatively to the object frame)
	geometry_msgs::PoseStamped grasp_pose_msg;
	tf::poseEigenToMsg(grasp_pose, grasp_pose_msg.pose);
	grasp_pose_msg.header.frame_id=frame_id;
	msg.grasp_pose.pose=grasp_pose_msg;

	// Grasp hand posture (joint space)
//	msg.grasp_posture.joints.name.resize(grasp->canonical_grasp->closed_hand_posture.joint_states.size());
//	msg.grasp_posture.joints.position.resize(grasp->canonical_grasp->closed_hand_posture.joint_states.size());
//	for(size_t i=0; i < grasp->canonical_grasp->closed_hand_posture.joint_states.size(); ++i)
//	{
//		// Grasp joint names
//		msg.grasp_posture.joints.name[i]=grasp->canonical_grasp->closed_hand_posture.joint_states[i].name;
//
//		// Grasp joint angles
//		msg.grasp_posture.joints.position[i]=grasp->canonical_grasp->closed_hand_posture.joint_states[i].position;
//	}

	// Grasp hand posture (synergy space)
	// NONE

	// Grasp type
	msg.grasp_posture.grasp_type=grasp->canonical_grasp->number;

	///////////////
	// PRE GRASP //
	///////////////

	//Eigen::Transform<double, 3, Eigen::Affine>	pre_grasp_pose=grasp->pre_pose*ros_palm_to_handle_transform;
	Eigen::Transform<double, 3, Eigen::Affine>	pre_grasp_pose=grasp->pre_pose;

	// Pre grasp hand pose (relatively to the object frame)
	geometry_msgs::PoseStamped grasp_pre_pose_msg;
	tf::poseEigenToMsg(pre_grasp_pose, grasp_pre_pose_msg.pose);
	grasp_pre_pose_msg.header.frame_id=frame_id;
	msg.pregrasp_pose.pose=grasp_pre_pose_msg;

	// Pre grasp hand posture (joint space)
//	msg.pregrasp_posture.joints.name.resize(grasp->canonical_grasp->open_hand_posture.joint_states.size());
//	msg.pregrasp_posture.joints.position.resize(grasp->canonical_grasp->open_hand_posture.joint_states.size());
//	for(size_t i=0; i < grasp->canonical_grasp->open_hand_posture.joint_states.size(); ++i)
//	{
//		// Pre grasp joint names
//		msg.pregrasp_posture.joints.name[i]=grasp->canonical_grasp->open_hand_posture.joint_states[i].name;
//
//		// Grasp joint angles
//		msg.pregrasp_posture.joints.position[i]=grasp->canonical_grasp->open_hand_posture.joint_states[i].position;
//	}

	// Pre grasp hand posture (synergy space)
	// NONE

	// Pre-grasp type (same as grasp?)
	msg.pregrasp_posture.grasp_type=grasp->canonical_grasp->number;

	return msg;
}

//ist_msgs::HandState RosGenerateGraspsActionServer::fillHandStateMessage(const boost::shared_ptr<GraspPerturbation> grasp_perturbation, std::string frame_id)
//{
//	ist_msgs::HandState msg;
//
//	///////////
//	// GRASP //
//	///////////
//
//	// Transform from handle frame definition to ros frame definition
//	Eigen::Matrix3d handle_to_ros_palm_rotation_matrix;
//	handle_to_ros_palm_rotation_matrix << 0, 0, 1,
//								          0,-1, 0,
//								          1, 0, 0;
//
//	//ROS_INFO("valores offsets: %f, %f, %f", palm_center_offset_x,palm_center_offset_y,palm_center_offset_z);
//	Eigen::Transform<double, 3, Eigen::Affine> handle_to_ros_palm_transform=Eigen::Translation3d(palm_center_offset_x ,palm_center_offset_y , palm_center_offset_z)*Eigen::AngleAxisd(handle_to_ros_palm_rotation_matrix);
//
//	//Eigen::Transform<double, 3, Eigen::Affine> ros_palm_to_handle_transform=handle_to_ros_palm_transform.inverse();
//
//
////	Eigen::Transform<double, 3, Eigen::Affine>	grasp_pose=grasp_perturbation->pose*ros_palm_to_handle_transform;
//
//	Eigen::Transform<double, 3, Eigen::Affine>	grasp_pose=grasp_perturbation->pose;
//
//
//	// Grasp hand pose (relatively to the object frame)
//	geometry_msgs::PoseStamped grasp_pose_msg;
//	tf::poseEigenToMsg(grasp_pose, grasp_pose_msg.pose);
//	grasp_pose_msg.header.frame_id=frame_id;
//	msg.grasp_pose.pose=grasp_pose_msg;
//
//	// Grasp hand posture (joint space)
////	msg.grasp_posture.joints.name.resize(grasp->canonical_grasp->closed_hand_posture.joint_states.size());
////	msg.grasp_posture.joints.position.resize(grasp->canonical_grasp->closed_hand_posture.joint_states.size());
////	for(size_t i=0; i < grasp->canonical_grasp->closed_hand_posture.joint_states.size(); ++i)
////	{
////		// Grasp joint names
////		msg.grasp_posture.joints.name[i]=grasp->canonical_grasp->closed_hand_posture.joint_states[i].name;
////
////		// Grasp joint angles
////		msg.grasp_posture.joints.position[i]=grasp->canonical_grasp->closed_hand_posture.joint_states[i].position;
////	}
//
//	// Grasp hand posture (synergy space)
//	// NONE
//
//	// Grasp type
//	msg.grasp_posture.grasp_type=grasp_perturbation->grasp->canonical_grasp->number;
//
//	///////////////
//	// PRE GRASP //
//	///////////////
//
////	Eigen::Transform<double, 3, Eigen::Affine>	pre_grasp_pose=grasp_perturbation->grasp->pre_pose*ros_palm_to_handle_transform;
//	Eigen::Transform<double, 3, Eigen::Affine>	pre_grasp_pose=grasp_perturbation->grasp->pre_pose;
//
//	// Pre grasp hand pose (relatively to the object frame)
//	geometry_msgs::PoseStamped grasp_pre_pose_msg;
//	tf::poseEigenToMsg(pre_grasp_pose, grasp_pre_pose_msg.pose);
//	grasp_pre_pose_msg.header.frame_id=frame_id;
//	msg.pregrasp_pose.pose=grasp_pre_pose_msg;
//
//	// Pre grasp hand posture (joint space)
////	msg.pregrasp_posture.joints.name.resize(grasp->canonical_grasp->open_hand_posture.joint_states.size());
////	msg.pregrasp_posture.joints.position.resize(grasp->canonical_grasp->open_hand_posture.joint_states.size());
////	for(size_t i=0; i < grasp->canonical_grasp->open_hand_posture.joint_states.size(); ++i)
////	{
////		// Pre grasp joint names
////		msg.pregrasp_posture.joints.name[i]=grasp->canonical_grasp->open_hand_posture.joint_states[i].name;
////
////		// Grasp joint angles
////		msg.pregrasp_posture.joints.position[i]=grasp->canonical_grasp->open_hand_posture.joint_states[i].position;
////	}
//
//	// Pre grasp hand posture (synergy space)
//	// NONE
//
//	// Pre-grasp type (same as grasp?)
//	msg.pregrasp_posture.grasp_type=grasp_perturbation->grasp->canonical_grasp->number;
//
//	return msg;
//}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ist_generate_grasps");
	ros::NodeHandle n;

	RosGenerateGraspsActionServer rosGenerateGraspsActionServer(ros::this_node::getName());

	ros::spin();
}
