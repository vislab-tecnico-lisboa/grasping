/*
 * GraspCandidateListManager.cpp
 *
 *  Created on: Dec 15, 2012
 *      Author: Rui P. Figueiredo
 */


#include "grasp_candidate_list_manager/grasp_candidate_list_manager.h"

std::vector<std::vector<bool> > GraspCandidateListManager::grip_equivalence; // map of size type and shape type (we don't consider symmetry)

std::vector<std::vector<double> > GraspCandidateListManager::symmetry_1ALL_grip_equivalence_rules;
std::vector<std::vector<double> > GraspCandidateListManager::symmetry_2ALL_grip_equivalence_rules;
std::vector<std::vector<double> > GraspCandidateListManager::symmetry_3Z_grip_equivalence_rules;


GraspCandidateListManager::GraspCandidateListManager(ros::NodeHandle n_) :
		n(n_),
		n_priv("~"),
		it_(n_)
{

	///////////////
	// ROS STUFF //
	///////////////

//	// Get graspability map client

//    get_database_graspability_map_client_ = n.serviceClient<database_access_msgs::GetGraspabilityMap>("GetGraspabilityMap");

	// Grasp candidate list manager service
	grasp_candidate_list_manager_service = n.advertiseService("ist_grasps_manager", &GraspCandidateListManager::serviceCallback,this);

	// Grasp candidate list manager update knowledge from experiment service
	grasp_candidate_list_manager_new_experiment_service = n.advertiseService("ist_grasps_manager_update_knowledge", &GraspCandidateListManager::newExperimentServiceCallback,this);

	n_priv.param<std::string>("world_frame_id", world_frame_id, "base_link");
	ROS_INFO("World frame id: %s", world_frame_id.c_str());

    n_priv.param<std::string>("table_frame_id", table_frame_id, "/table_frame");
	ROS_INFO("Table frame id: %s", table_frame_id.c_str());

	std::string human_knowledge_file_name;
    n_priv.param<std::string>("human_knowledge_file_name", human_knowledge_file_name, "human_readings.csv");
	ROS_INFO("Human knowledge file name: %s", human_knowledge_file_name.c_str());

	std::string human_knowledge_file_directory;
    n_priv.param<std::string>("human_knowledge_file_directory", human_knowledge_file_directory, "");
	ROS_INFO("Human knowledge file directory: %s", human_knowledge_file_directory.c_str());

	std::string similar_grasps_kernel_file_name;
    n_priv.param<std::string>("similar_grasps_kernel_file_name", similar_grasps_kernel_file_name, "human_readings.csv");
	ROS_INFO("Similar grasps kernel file name: %s", similar_grasps_kernel_file_name.c_str());

	std::string similar_grasps_kernel_file_directory;
    n_priv.param<std::string>("similar_grasps_kernel_file_directory", similar_grasps_kernel_file_directory, "");
	ROS_INFO("Similar grasps kernel file directory: %s", similar_grasps_kernel_file_directory.c_str());

    n_priv.param<double>("gripper_size", gripper_size, 0.05);
	ROS_INFO("Gripper size: %f", gripper_size);

	graspability_map_image_pub_ = it_.advertise("graspability_map_image",1);
	graspability_object_map_image_pub_ = it_.advertise("graspability_object_map_image",1);

    n_priv.param<bool>("use_reachability_map", use_reachability_map, false);
	ROS_INFO("Use reachability map: %d", use_reachability_map);
	if(use_reachability_map)
	{
		reachability_map=ReachabilityMap(n_);
	}



	//////////
	// RVIZ //
	//////////

	// Object details markers
	marker_pub = n.advertise<visualization_msgs::MarkerArray>("grasp_candidate_list_markers_out",1);

	///////////
	// RULES //
	///////////

	// EQUIVALENT GRIPS ON SYMMETRY 1ALL (BRICK, ERASER)
	symmetry_1ALL_grip_equivalence_rules.resize(6);
	symmetry_1ALL_grip_equivalence_rules[0]=boost::assign::list_of( 1)( 3)( 9)(11); // 1  RI-FR = RI-BA = LE-BA = LE-FR
	symmetry_1ALL_grip_equivalence_rules[1]=boost::assign::list_of( 2)( 4)(10)(12); // 2  RI-UP = RI-DO = LE-UP = LE-DO
	symmetry_1ALL_grip_equivalence_rules[2]=boost::assign::list_of( 5)( 7)(13)(15); // 3  FR-LE = FR-RI = BA-RI = BA-LE
	symmetry_1ALL_grip_equivalence_rules[3]=boost::assign::list_of( 6)( 8)(14)(16); // 4  FR-UP = FR-DO = BA-UP = BA-DO
	symmetry_1ALL_grip_equivalence_rules[4]=boost::assign::list_of(17)(19)(21)(23); // 5  TO-FR = TO-BA = BO-FR = BO-BA
	symmetry_1ALL_grip_equivalence_rules[5]=boost::assign::list_of(18)(20)(22)(24); // 6  TO-LE = TO-RI = BO-RI = BO-LE

	// EQUIVALENT GRIPS ON SYMMETRY 2ALL (SPHERE, CUBE)
	symmetry_2ALL_grip_equivalence_rules.resize(1);
	symmetry_2ALL_grip_equivalence_rules[0]=boost::assign::list_of(1)(2)(3)(4)(5)(6)(7)(8)(9)(10)(11)(12)(13)(14)(15)(16)(17)(18)(19)(20)(21)(22)(23)(24); // 1  RI-FR = RI-BA = LE-BA = LE-FR

	// EQUIVALENT GRIPS ON SYMMETRY 3Z (CHALK STICK, ROLLING PIN)
	symmetry_3Z_grip_equivalence_rules.resize(3);
	symmetry_3Z_grip_equivalence_rules[0]=boost::assign::list_of( 1)( 3)( 5)( 7)( 9)(11)(13)(15); // 1  RI-FR = RI-BA = FR-LE = FR-RI = LE-BA = LE-FR = BA-RI = BA-LE
	symmetry_3Z_grip_equivalence_rules[1]=boost::assign::list_of( 2)( 4)( 6)( 8)(10)(12)(14)(16); //	2  RI-UP = RI-DO = FR-UP = FR-DO = LR-UP = LR-DO = BA-UP = BA-DO
	symmetry_3Z_grip_equivalence_rules[2]=boost::assign::list_of(17)(18)(19)(20)(21)(22)(23)(24); //	3  TO-FR = TO-LE = TO-BA = TO-RI = BO-FR = BO-RI = BO-BA = BO-LE

	distance_kernel.resize(4);
	distance_kernel[0]=boost::assign::list_of( 0.0 )( 0.0 )( 0.0 )( 0.0 );
	distance_kernel[1]=boost::assign::list_of( 0.0 )((double)1.0 )((double) 0.5 )((double)0.0 );
	distance_kernel[2]=boost::assign::list_of( 0.0 )((double)0.5 )((double) 1.0 )((double)0.5);
	distance_kernel[3]=boost::assign::list_of( 0.0 )((double)0.0 )((double) 0.5 )((double)1.0 );

	// Parse similar grasps kernel
	GraspsKernelParser similar_grasps_kernel_parser;
	std::string similar_grasps_kernel_file=similar_grasps_kernel_file_directory+similar_grasps_kernel_file_name;

	std::cout << similar_grasps_kernel_file << std::endl;

	grasp_similarity_kernel=similar_grasps_kernel_parser.parse(similar_grasps_kernel_file);

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

	// Object id to shape id
	object_shape_map=boost::assign::map_list_of
	( 1, 1)
	( 2, 2)
	( 3, 3)
	( 4, 4)
	( 5, 2)
	( 6, 4)
	( 7, 2)
	( 8, 4)
	( 9, 3)
	(10, 4)
	(11, 4)
	(12, 1)
	(13, 2)
	(14, 3)
	(15, 4)
	(16, 3)
	(17, 4)
	(18, 2)
	(19, 4)
	(20, 3)
	(21, 4)
	(22, 1)
	(23, 2)
	(24, 3)
	(25, 4);

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


	//////////////////////////////////////////
	// Initialize graspability accumulators //
	//////////////////////////////////////////

	temp_successes.resize(GRASP_SPACE_SIZE+1);
	temp_failures.resize(GRASP_SPACE_SIZE+1);
	graspability_human_successes.resize(GRASP_SPACE_SIZE+1);
	graspability_human_failures.resize(GRASP_SPACE_SIZE+1);
	graspability_robot_successes.resize(GRASP_SPACE_SIZE+1);
	graspability_robot_failures.resize(GRASP_SPACE_SIZE+1);
	graspability_map.resize(GRASP_SPACE_SIZE+1);

	for(unsigned int grasp_index=0; grasp_index < GRASP_SPACE_SIZE+1; ++grasp_index)
	{
		temp_successes[grasp_index].resize(GRIP_SPACE_SIZE+1);
		temp_failures [grasp_index].resize(GRIP_SPACE_SIZE+1);
		graspability_human_successes[grasp_index].resize(GRIP_SPACE_SIZE+1);
		graspability_human_failures[grasp_index].resize(GRIP_SPACE_SIZE+1);
		graspability_robot_successes[grasp_index].resize(GRIP_SPACE_SIZE+1);
		graspability_robot_failures[grasp_index].resize(GRIP_SPACE_SIZE+1);
		graspability_map[grasp_index].resize(GRIP_SPACE_SIZE+1);
		for(unsigned int grip_index=0; grip_index < GRIP_SPACE_SIZE+1; ++grip_index)
		{
			temp_successes[grasp_index][grip_index].resize(PART_SPACE_SIZE+1);
			temp_failures [grasp_index][grip_index].resize(PART_SPACE_SIZE+1);
			graspability_human_successes[grasp_index][grip_index].resize(PART_SPACE_SIZE+1);
			graspability_human_failures[grasp_index][grip_index].resize(PART_SPACE_SIZE+1);
			graspability_robot_successes[grasp_index][grip_index].resize(PART_SPACE_SIZE+1);
			graspability_robot_failures[grasp_index][grip_index].resize(PART_SPACE_SIZE+1);
			graspability_map[grasp_index][grip_index].resize(PART_SPACE_SIZE+1);
			for(unsigned int part_index=0; part_index < PART_SPACE_SIZE+1; ++part_index)
			{
				temp_successes[grasp_index][grip_index][part_index].resize(DISTANCE_SPACE_SIZE+1);
				temp_failures [grasp_index][grip_index][part_index].resize(DISTANCE_SPACE_SIZE+1);
				graspability_human_successes[grasp_index][grip_index][part_index].resize(DISTANCE_SPACE_SIZE+1);
				graspability_human_failures[grasp_index][grip_index][part_index].resize(DISTANCE_SPACE_SIZE+1);
				graspability_robot_successes[grasp_index][grip_index][part_index].resize(DISTANCE_SPACE_SIZE+1);
				graspability_robot_failures[grasp_index][grip_index][part_index].resize(DISTANCE_SPACE_SIZE+1);
				graspability_map[grasp_index][grip_index][part_index].resize(DISTANCE_SPACE_SIZE+1);
				for(unsigned int distance_index=0; distance_index < DISTANCE_SPACE_SIZE+1; ++distance_index)
				{
					temp_successes[grasp_index][grip_index][part_index][distance_index].resize(OBJECT_SPACE_SIZE+1);
					temp_failures [grasp_index][grip_index][part_index][distance_index].resize(OBJECT_SPACE_SIZE+1);
					graspability_human_successes[grasp_index][grip_index][part_index][distance_index].resize(OBJECT_SPACE_SIZE+1);
					graspability_human_failures[grasp_index][grip_index][part_index][distance_index].resize(OBJECT_SPACE_SIZE+1);
					graspability_robot_successes[grasp_index][grip_index][part_index][distance_index].resize(OBJECT_SPACE_SIZE+1);
					graspability_robot_failures[grasp_index][grip_index][part_index][distance_index].resize(OBJECT_SPACE_SIZE+1);
					graspability_map[grasp_index][grip_index][part_index][distance_index].resize(OBJECT_SPACE_SIZE+1);
					for(unsigned int object_index=0; object_index < OBJECT_SPACE_SIZE+1; ++object_index)
					{
						temp_successes[grasp_index][grip_index][part_index][distance_index][object_index].resize(SYMMETRY_SPACE_SIZE+1);
						temp_failures [grasp_index][grip_index][part_index][distance_index][object_index].resize(SYMMETRY_SPACE_SIZE+1);
						graspability_human_successes[grasp_index][grip_index][part_index][distance_index][object_index].resize(SYMMETRY_SPACE_SIZE+1);
						graspability_human_failures[grasp_index][grip_index][part_index][distance_index][object_index].resize(SYMMETRY_SPACE_SIZE+1);
						graspability_robot_successes[grasp_index][grip_index][part_index][distance_index][object_index].resize(SYMMETRY_SPACE_SIZE+1);
						graspability_robot_failures[grasp_index][grip_index][part_index][distance_index][object_index].resize(SYMMETRY_SPACE_SIZE+1);
						graspability_map[grasp_index][grip_index][part_index][distance_index][object_index].resize(SYMMETRY_SPACE_SIZE+1);
						for(unsigned int symmetry_index=0; symmetry_index < SYMMETRY_SPACE_SIZE+1; ++symmetry_index)
						{
							graspability_human_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]=1;
							graspability_human_failures[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]=1;
							graspability_robot_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]=1;
							graspability_robot_failures[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]=1;
							//graspability_map[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]=0.5;
						}
					}
				}
			}
		}
	}

	///////////////////////////////////////////
	// Parse and incorporate human knowledge //
	///////////////////////////////////////////

	std::vector<std::vector<double> > human_knowledge;

	HumanGraspsParser human_grasps;
	std::string human_knowledge_file=human_knowledge_file_directory+human_knowledge_file_name;

	std::cout << human_knowledge_file << std::endl;

	human_knowledge=human_grasps.parse(human_knowledge_file);

	for(unsigned int i=0; i < human_knowledge.size(); ++i)
	{
		unsigned int grasp = human_knowledge[i][0];
		unsigned int grip  = human_knowledge[i][1];
		unsigned int distance   = human_knowledge[i][2];
		unsigned int shape = human_knowledge[i][3];
		unsigned int size  = human_knowledge[i][4];
		double trials      = human_knowledge[i][5];

		unsigned int object=size_shape_object_map.find(std::pair<int,int>(size,shape))->second;

		incorporateKnowledge(grasp, grip, 0, distance, object, 0, trials, trials, HUMAN);
//		sleep(4);
	}

	//////////////////////////////////////////////////////////////////
	// Retrieve and incorporate knowledge from previous experiments //
	//////////////////////////////////////////////////////////////////

//	ROS_INFO("Incorporate robot knowledge from previous experiments...");
//
//	if(get_database_graspability_map_client_.call(srv_GetGraspabilityMap))
//	{
//		for(unsigned int i=0; i < srv_GetGraspabilityMap.response.graspability_map.size(); ++i)
//		{
//			double successes=srv_GetGraspabilityMap.response.graspability_map[i].nb_successes;
//			double trials=srv_GetGraspabilityMap.response.graspability_map[i].nb_trials;
//
//			unsigned int shape=srv_GetGraspabilityMap.response.graspability_map[i].object_type.shape.value;
//			unsigned int size=msgs_size_handle_size_map.find(srv_GetGraspabilityMap.response.graspability_map[i].object_type.size.id)->second;
//			//unsigned int symmetry=srv_GetGraspabilityMap.response.graspability_map[i].object_type.symmetry.value;
//
//			unsigned int grasp=	  srv_GetGraspabilityMap.response.graspability_map[i].grip_state.hand_state.grasp_posture.grasp_type;
//			unsigned int grip=	  srv_GetGraspabilityMap.response.graspability_map[i].grip_state.grip_pose.direction.id;
//			unsigned int distance=srv_GetGraspabilityMap.response.graspability_map[i].grip_state.grip_pose.distance.id;
//
//			unsigned int object=size_shape_object_map.find(std::pair<int,int>(size,shape))->second;
//
//			std::cout << "Robot experiment number " << i+1 << " loaded: " << std::endl << " object: " << object << " size: " << size << " grasp: " << grasp << " grip: "<< grip << " distance: "<< distance << " trials: " << trials << " successes: " << successes << std::endl;
//
//			incorporateKnowledge(grasp, grip, 0, distance, object, 0, trials, successes, ROBOT); // incorporate new experiment knowledge
//		}
//	}
//	else
//	{
//		ROS_ERROR("Failed to call service %s and load previous experiments knowledge from database.",get_database_graspability_map_client_.getService().c_str());
//	}
//	ROS_INFO("Done.");


	// Save data in file
//	std::ofstream myfile;
//	myfile.open ("gm.txt");
//	for(unsigned int object_index=0; object_index < OBJECT_SPACE_SIZE+1; ++object_index)
//	{
//		for(unsigned int grip_index=0; grip_index < GRIP_SPACE_SIZE+1; ++grip_index)
//		{
//			for(unsigned int grasp_index=0; grasp_index < GRASP_SPACE_SIZE+1; ++grasp_index)
//			{
//				for(unsigned int distance_index=0; distance_index < DISTANCE_SPACE_SIZE+1; ++distance_index)
//				{
//					for(unsigned int part_index=0; part_index < PART_SPACE_SIZE+1; ++part_index)
//					{
//						for(unsigned int symmetry_index=0; symmetry_index < SYMMETRY_SPACE_SIZE+1; ++symmetry_index)
//						{
//							myfile << "object type:" << object_index << " grasp type:" << grasp_index <<  " grip type:" << grip_index << " distance_type: "<< distance_index << " gm:"<< graspability_map[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index] << " " << std::endl;
//						}
//					}
//				}
//			}
//			myfile << std::endl << " ";
//		}
//	}
//	myfile.close();



}

void GraspCandidateListManager::incorporateKnowledge(
		unsigned int grasp,
		unsigned int grip,
		unsigned int part,
		unsigned int distance,
		unsigned int object,
		unsigned int symmetry,
		double trials,
		double successes,
		bool is_robot,
		bool success)
{

	double failures=trials-successes;
	//ROS_INFO("FAILURES: %f", failures);
	// Temp init
	for(unsigned int grasp_index=0; grasp_index < GRASP_SPACE_SIZE+1; ++grasp_index)
	{
		for(unsigned int grip_index=0; grip_index < GRIP_SPACE_SIZE+1; ++grip_index)
		{
			for(unsigned int part_index=0; part_index < PART_SPACE_SIZE+1; ++part_index)
			{
				for(unsigned int distance_index=0; distance_index < DISTANCE_SPACE_SIZE+1; ++distance_index)
				{
					for(unsigned int object_index=0; object_index < OBJECT_SPACE_SIZE+1; ++object_index)
					{
						for(unsigned int symmetry_index=0; symmetry_index < SYMMETRY_SPACE_SIZE+1; ++symmetry_index)
						{
							temp_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]=0;
							temp_failures[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]=0;

						}
					}
				}
			}
		}
	}

	temp_successes[grasp][grip][0][distance][object][symmetry]+=successes;
	temp_failures[grasp][grip][0][distance][object][symmetry]+=failures;

	///////////////////////////////////////////
	// Propagate knowledge to symmetry group //
	///////////////////////////////////////////

	unsigned int shape=object_shape_map.find(object)->second;

	// Rules for shape type 1 according to symmetries (2ALL)
	if(shape==1)
	{
		// Find grip in the set of rules
		unsigned int rule_index=0;
		for(; rule_index < symmetry_2ALL_grip_equivalence_rules.size(); ++rule_index)
		{
			bool rule_found=false;
			// Find grip...
			for(unsigned int equivalent_grip_index=0; equivalent_grip_index < symmetry_2ALL_grip_equivalence_rules[rule_index].size(); ++equivalent_grip_index)
			{
				if(symmetry_2ALL_grip_equivalence_rules[rule_index][equivalent_grip_index]==grip)
				{
					rule_found=true;
					break;// rule found!
				}
			}

			if(rule_found)
				break;
		}

		// Propagate found rule
		for(unsigned int equivalent_grip_index=0; equivalent_grip_index < symmetry_2ALL_grip_equivalence_rules[rule_index].size(); ++equivalent_grip_index)
		{
			unsigned int equivalent_grip=symmetry_2ALL_grip_equivalence_rules[rule_index][equivalent_grip_index];
			if(grip!=equivalent_grip)
			{
				temp_successes[grasp][equivalent_grip][part][distance][object][symmetry]+=0.8*successes;
				temp_failures [grasp][equivalent_grip][part][distance][object][symmetry]+=0.8*failures;
			}
		}
	}
	// Rules for shape type 2 A and B according to symmetries (3Z)
	if(shape==2 || shape==3)
	{
		// Find grip in the set of rules
		unsigned int rule_index=0;
		for(; rule_index < symmetry_3Z_grip_equivalence_rules.size(); ++rule_index)
		{
			bool rule_found=false;
			// Find grip...
			for(unsigned int equivalent_grip_index=0; equivalent_grip_index < symmetry_3Z_grip_equivalence_rules[rule_index].size(); ++equivalent_grip_index)
			{
				if(symmetry_3Z_grip_equivalence_rules[rule_index][equivalent_grip_index]==grip)
				{
					rule_found=true;
					break;// rule found!
				}
			}

			if(rule_found)
				break;
		}

		// Propagate found rule
		for(unsigned int equivalent_grip_index=0; equivalent_grip_index < symmetry_3Z_grip_equivalence_rules[rule_index].size(); ++equivalent_grip_index)
		{
			unsigned int equivalent_grip=symmetry_3Z_grip_equivalence_rules[rule_index][equivalent_grip_index];
			if(grip!=equivalent_grip)
			{
				temp_successes[grasp][equivalent_grip][part][distance][object][symmetry]+=0.8*successes;
				temp_failures [grasp][equivalent_grip][part][distance][object][symmetry]+=0.8*failures;
			}


		}
	}
	// Rules for shape type 3 according to symmetries (1ALL)
	if(shape==4)
	{
		// Find grip in the set of rules
		unsigned int rule_index=0;
		for(; rule_index < symmetry_1ALL_grip_equivalence_rules.size(); ++rule_index)
		{
			bool rule_found=false;
			// Find grip...
			for(unsigned int equivalent_grip_index=0; equivalent_grip_index < symmetry_1ALL_grip_equivalence_rules[rule_index].size(); ++equivalent_grip_index)
			{
				if(symmetry_1ALL_grip_equivalence_rules[rule_index][equivalent_grip_index]==grip)
				{
					rule_found=true;
					break;// rule found!
				}
			}

			if(rule_found)
				break;
		}

		// Propagate found rule
		for(unsigned int equivalent_grip_index=0; equivalent_grip_index < symmetry_1ALL_grip_equivalence_rules[rule_index].size(); ++equivalent_grip_index)
		{
			unsigned int equivalent_grip=symmetry_1ALL_grip_equivalence_rules[rule_index][equivalent_grip_index];
			if(grip!=equivalent_grip)
			{
				temp_successes[grasp][equivalent_grip][part][distance][object][symmetry]+=0.8*successes;
				temp_failures [grasp][equivalent_grip][part][distance][object][symmetry]+=0.8*failures;
			}
		}
	}

	/////////////////////////////////////////////////////////////////
	// Propagate knowledge given reach distance equivalence kernel //
	/////////////////////////////////////////////////////////////////

	for(unsigned int grasp_index=0; grasp_index < GRASP_SPACE_SIZE+1; ++grasp_index)
	{
		for(unsigned int grip_index=0; grip_index < GRIP_SPACE_SIZE+1; ++grip_index)
		{
			for(unsigned int part_index=0; part_index < PART_SPACE_SIZE+1; ++part_index)
			{
				for(unsigned int object_index=0; object_index < OBJECT_SPACE_SIZE+1; ++object_index)
				{
					for(unsigned int symmetry_index=0; symmetry_index < SYMMETRY_SPACE_SIZE+1; ++symmetry_index)
					{
						if(temp_successes[grasp_index][grip_index][part_index][distance][object_index][symmetry_index]>0.000001)
						{
							for(unsigned int kernel_index=0; kernel_index < DISTANCE_SPACE_SIZE+1; ++kernel_index)
							{
								//std::cout << "grasp:" <<grasp_index<< " grip:" <<grip_index <<" part:" << part_index <<" distance:"<<distance_index << "object:" << object_index <<" " <<symmetry_index << std::endl;
								//std::cout << "temp before: " << temp_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]<<std::endl;
								temp_successes[grasp_index][grip_index][part_index][kernel_index][object_index][symmetry_index]=distance_kernel[distance][kernel_index]*temp_successes[grasp_index][grip_index][part_index][distance][object_index][symmetry_index];
								//std::cout << "kernel ("<< distance_index << "," <<  kernel_index<< ")="<< distance_kernel[distance_index][kernel_index]<< " temp after: " << temp_successes[grasp_index][grip_index][part_index][kernel_index][object_index][symmetry_index]<<std::endl;
								temp_failures [grasp_index][grip_index][part_index][kernel_index][object_index][symmetry_index]=distance_kernel[distance][kernel_index]*temp_failures [grasp_index][grip_index][part_index][distance][object_index][symmetry_index];
							}
						}
					}
				}
			}
		}
	}


	////////////////////////////////////////////////////////
	// Propagate knowledge given grasp equivalence kernel //
	////////////////////////////////////////////////////////


	for(unsigned int grip_index=0; grip_index < GRIP_SPACE_SIZE+1; ++grip_index)
	{
		for(unsigned int part_index=0; part_index < PART_SPACE_SIZE+1; ++part_index)
		{
			for(unsigned int distance_index=0; distance_index < DISTANCE_SPACE_SIZE+1; ++distance_index)
			{
				for(unsigned int object_index=0; object_index < OBJECT_SPACE_SIZE+1; ++object_index)
				{
					for(unsigned int symmetry_index=0; symmetry_index < SYMMETRY_SPACE_SIZE+1; ++symmetry_index)
					{
						if(temp_successes[grasp][grip_index][part_index][distance_index][object_index][symmetry_index]>0.00001)
						{
							for(unsigned int kernel_index=0; kernel_index < GRASP_SPACE_SIZE+1; ++kernel_index)
							{
								//std::cout <<" grasp:" <<grasp_index<< " grip:" <<grip_index <<" part:" << part_index <<" distance:"<<distance_index << "object:" << object_index <<" " <<symmetry_index << std::endl;
								//std::cout << "temp before: " << temp_successes[grasp][grip_index][part_index][distance_index][object_index][symmetry_index]<<std::endl;
								temp_successes[kernel_index][grip_index][part_index][distance_index][object_index][symmetry_index]=grasp_similarity_kernel[grasp][kernel_index]*temp_successes[grasp][grip_index][part_index][distance_index][object_index][symmetry_index];
								//std::cout << "kernel ("<< grasp << "," <<  kernel_index<< ")="<< grasp_similarity_kernel[grasp][kernel_index]<< " temp after: " << temp_successes[kernel_index][grip_index][part_index][distance_index][object_index][symmetry_index]<<std::endl;
							}
						}
						if(temp_failures[grasp][grip_index][part_index][distance_index][object_index][symmetry_index]>0.00001)
						{
							for(unsigned int kernel_index=0; kernel_index < GRASP_SPACE_SIZE+1; ++kernel_index)
							{
								//std::cout <<" grasp:" <<grasp_index<< " grip:" <<grip_index <<" part:" << part_index <<" distance:"<<distance_index << "object:" << object_index <<" " <<symmetry_index << std::endl;
								//std::cout << "temp before: " << temp_failures[grasp][grip_index][part_index][distance_index][object_index][symmetry_index]<<std::endl;
								temp_failures[kernel_index][grip_index][part_index][distance_index][object_index][symmetry_index]=grasp_similarity_kernel[grasp][kernel_index]*temp_failures[grasp][grip_index][part_index][distance_index][object_index][symmetry_index];
								//std::cout << "kernel ("<< grasp << "," <<  kernel_index<< ")="<< grasp_similarity_kernel[grasp][kernel_index]<< " temp after: " << temp_failures[kernel_index][grip_index][part_index][distance_index][object_index][symmetry_index]<<std::endl;
							}
						}
					}
				}
			}
		}
	}


	/////////////////////////////
	// Update graspability map //
	/////////////////////////////

	for(unsigned int grasp_index=0; grasp_index < GRASP_SPACE_SIZE+1; ++grasp_index)
	{
		for(unsigned int grip_index=0; grip_index < GRIP_SPACE_SIZE+1; ++grip_index)
		{
			for(unsigned int part_index=0; part_index < PART_SPACE_SIZE+1; ++part_index)
			{
				for(unsigned int distance_index=0; distance_index < DISTANCE_SPACE_SIZE+1; ++distance_index)
				{
					for(unsigned int object_index=0; object_index < OBJECT_SPACE_SIZE+1; ++object_index)
					{
						for(unsigned int symmetry_index=0; symmetry_index < SYMMETRY_SPACE_SIZE+1; ++symmetry_index)
						{
							if(is_robot)
							{
								graspability_robot_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]+=temp_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index];
								graspability_robot_failures[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index] +=temp_failures [grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index];

							}
							else if(!is_robot)
							{
								graspability_human_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]+=temp_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index];
								graspability_human_failures[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index] +=temp_failures [grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index];
							}

							double __successes=graspability_human_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]+graspability_robot_successes[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index];
							double __failures=graspability_human_failures[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]+graspability_robot_failures[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index];
							graspability_map[grasp_index][grip_index][part_index][distance_index][object_index][symmetry_index]=__successes/(__successes+__failures);
						}
					}
				}
			}
		}
	}


	// SHOW graspability map image
	graspabilityMapToImage();

	graspabilityMapSingleObjectToImage(object);

}


GraspCandidateListManager::~GraspCandidateListManager()
{
	// TODO Auto-generated destructor stub
}

ist_msgs::GripList GraspCandidateListManager::handObjectRules(ist_msgs::GripList & grasps, ist_msgs::Object & object)
{

	unsigned int discarded_grasps=0;

	////////////////////////////////////////////////////////////////
	// RULE: discard impossible grasps due to being inside object //
	////////////////////////////////////////////////////////////////


	std::cout << "GRASPS BEFORE:" << grasps.grip_states.size() << std::endl;
	for(std::vector<ist_msgs::ActionablePartData>::iterator r=object.data.actionable_parts_data.begin(); r != object.data.actionable_parts_data.end(); ++r)
	{
		Eigen::Vector3d min_pt(-r->part.bounding_box.x,-r->part.bounding_box.y,-r->part.bounding_box.z);
		Eigen::Vector3d max_pt( r->part.bounding_box.x, r->part.bounding_box.y, r->part.bounding_box.z);


		for(std::vector<ist_msgs::GripState>::iterator i=grasps.grip_states.begin(); i != grasps.grip_states.end(); )
		{
			if( (i->hand_state.grasp_pose.pose.pose.position.x<min_pt.x()) || (i->hand_state.grasp_pose.pose.pose.position.x>max_pt.x()) )
			{
				++i;
				continue;
			}

			if( (i->hand_state.grasp_pose.pose.pose.position.y<min_pt.y()) || (i->hand_state.grasp_pose.pose.pose.position.y>max_pt.y()) )
			{
				++i;
				continue;
			}

			if( (i->hand_state.grasp_pose.pose.pose.position.z<min_pt.z()) || (i->hand_state.grasp_pose.pose.pose.position.z>max_pt.z()) )
			{
				++i;
				continue;
			}
//			std::cout << "MAX POINT: " << max_pt << std::endl;
//			std::cout << "POINT: " << i->hand_state.grasp_pose.pose.pose.position << std::endl;

			++discarded_grasps;
			i=grasps.grip_states.erase(i);
		}
	}

	ROS_INFO_STREAM("Number of grasps discarded (Rule: impossible due to being inside object):" << discarded_grasps);

	////////////////////////////////////////////////////////
	// RULE: discard impossible grasps due to object size //
	////////////////////////////////////////////////////////

	discarded_grasps=0;

	double hand_opening_threshold=gripper_size*0.5;

	//std::cout << object.data.type.size.values.x << " " << object.data.type.size.values.y << " " << object.data.type.size.values.z << std::endl;
	// Rule 1: Discard all grasps which are impossible due to object size
	for(std::vector<ist_msgs::ActionablePartData>::iterator r=object.data.actionable_parts_data.begin(); r != object.data.actionable_parts_data.end(); ++r)
	{
		if(r->part.id!=4)
		{
			for(std::vector<ist_msgs::GripState>::iterator i=grasps.grip_states.begin(); i != grasps.grip_states.end(); )
			{
				bool discard=false;
				int id = i->grip_pose.direction.id;
				// thumb-front (1,11,17,21) && thumb-back (3,9,19,23)   -> X size
				if((id==1||id==11||id==17||id==21)||(id==3||id==9||id==19||id==23))
				{
					if(object.data.type.size.values.x > hand_opening_threshold)
					{
						discard=true;
					}
				}
				// thumb-left  (5,15,18,24) && thumb-right (7,13,20,22) -> Y size
				if((id==5||id==15||id==18||id==24)||(id==7||id==13||id==20||id==22))
				{
					if(object.data.type.size.values.y > hand_opening_threshold)
					{
						discard=true;
					}
				}
				// thumb-up    (2,6,10,14)  && thumb-down(4,8,12,16)    -> Z size
				if((id==2||id==6||id==10||id==14)||(id==4||id==8||id==12||id==16))
				{
					if(object.data.type.size.values.y > hand_opening_threshold)
					{
						discard=true;
					}
				}

				if(discard)
				{
					i=grasps.grip_states.erase(i);
					++discarded_grasps;
				}
				else
					++i;
			}
		}
		else // HANDLE REGION RULES
		{
			for(std::vector<ist_msgs::GripState>::iterator i=grasps.grip_states.begin(); i != grasps.grip_states.end(); )
			{
				bool discard=false;
				int id = i->grip_pose.direction.id;
				// thumb-front (1,11,17,21) && thumb-back (3,9,19,23)   -> X size
				if((id==1||id==11||id==17||id==21)||(id==3||id==9||id==19||id==23))
				{
					if(r->part.bounding_box.x > hand_opening_threshold)
					{
						discard=true;
					}
				}
				// thumb-left  (5,15,18,24) && thumb-right (7,13,20,22) -> Y size
				if((id==5||id==15||id==18||id==24)||(id==7||id==13||id==20||id==22))
				{
					if(r->part.bounding_box.y > hand_opening_threshold)
					{
						discard=true;
					}
				}
				// thumb-up    (2,6,10,14)  && thumb-down(4,8,12,16)    -> Z size
				if((id==2||id==6||id==10||id==14)||(id==4||id==8||id==12||id==16))
				{
					if(r->part.bounding_box.z > hand_opening_threshold)
					{
						discard=true;
					}
				}

				if(discard)
				{
					i=grasps.grip_states.erase(i);
					++discarded_grasps;
				}
				else
					++i;
			}
		}
	}


	ROS_INFO_STREAM("Number of grasps discarded (Rule: impossible due to object size):" << discarded_grasps);

	return grasps;
}


ist_msgs::GripList GraspCandidateListManager::handWorldRules(ist_msgs::GripList & grasps, ist_msgs::Object & object)
{

	unsigned int discarded_grasps=0;

	// A tf transform listener
	tf::TransformListener listener;

	tf::StampedTransform world_to_table_transform; // Table pose in world frame
	try
	{
		listener.waitForTransform(world_frame_id, table_frame_id, ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform(world_frame_id, table_frame_id, ros::Time(0), world_to_table_transform);
	}
	catch (tf::TransformException & ex)
	{
		ROS_INFO("Transform not acquired.");
		exit(-1);
	}

	ROS_INFO("Transform acquired.");
	ROS_INFO("%s Z: %f ",table_frame_id.c_str(), world_to_table_transform.getOrigin().getZ());

	tf::Transform world_to_object_transform; // Object pose in world frame
	if(object.state.graspable_object.potential_models.size()<1)
		return grasps;
	tf::poseMsgToTF(object.state.graspable_object.potential_models[0].pose.pose, world_to_object_transform);

	ROS_INFO_STREAM("number of grasps before:"<<grasps.grip_states.size());

	for(std::vector<ist_msgs::GripState>::iterator i=grasps.grip_states.begin(); i != grasps.grip_states.end(); )
	{
		bool discard=false;

		tf::Transform object_to_hand_transform; // Hand pose in object frame
		tf::poseMsgToTF(i->hand_state.grasp_pose.pose.pose, object_to_hand_transform);

		tf::Transform world_to_hand_transform; // Hand pose in world frame
		world_to_hand_transform.mult(world_to_object_transform, object_to_hand_transform);

		//////////////////////////////////////
		// RULE: discard grasps under table //
		//////////////////////////////////////


        if((world_to_hand_transform.getBasis()*tf::Vector3(0,1,0)).dot(tf::Vector3(0,0,1))>0.0)
		{
			//ROS_INFO("DISCARDED (angle...)!!!");
			discard=true;
		}

		/*ROS_INFO_STREAM("object pose: " << world_to_object_transform.getOrigin().getX() << " " 
		<< world_to_object_transform.getOrigin().getY() << " "
		<< world_to_object_transform.getOrigin().getZ());

		ROS_INFO_STREAM("hand pose: " << world_to_hand_transform.getOrigin().getX() << " " 
		<< world_to_hand_transform.getOrigin().getY() << " "
		<< world_to_hand_transform.getOrigin().getZ());
		if((double)world_to_hand_transform.getOrigin().getZ()<(double)world_to_table_transform.getOrigin().getZ())
		{
			discard=true;
		}*/


		/////////////////////////////////////////////////////////////////////////////////
		// RULE: Discard all grasps which are "almost" impossible due to kinematics... //
		/////////////////////////////////////////////////////////////////////////////////

		/*if((double)world_to_object_transform.getOrigin().getX()<(double)world_to_hand_transform.getOrigin().getX())
		{
			ROS_INFO("DISCARDED !!!");
			discard=true;
		}
		ROS_INFO_STREAM("object pose: " << world_to_object_transform.getOrigin().getX());

		ROS_INFO_STREAM("hand pose: " << world_to_hand_transform.getOrigin().getX());



		if((world_to_hand_transform.getBasis()*btVector3(0,1,0)).dot(btVector3(1,0,0))<0.0)
		{
			//ROS_INFO("DISCARDED (angle...)!!!");
			discard=true;
		}*/


		if(discard)
		{
			i=grasps.grip_states.erase(i);
			++discarded_grasps;
		}
		else
			++i;

	}
	ROS_INFO_STREAM("number of grasps after:"<<grasps.grip_states.size());

	ROS_INFO_STREAM("Number of grasps discarded (Rule: under the table, etc):" << discarded_grasps);


	return grasps;
}

ist_msgs::GripList GraspCandidateListManager::discardGrasps(ist_msgs::GripList & grasps, ist_msgs::Object & object)
{
	// Discard grasps which are impossible due to hand-object relationship
	//handObjectRules(grasps, object);

	// Discard grasps which are impossible due to hand-world relationship
	//handWorldRules(grasps, object);

	// Weight grasps with previous knowledge
	/*for(std::vector<ist_msgs::GripState>::iterator i=grasps.grip_states.begin(); i != grasps.grip_states.end(); ++i)
	{
		unsigned int grasp=i->hand_state.grasp_posture.grasp_type;
		//std::cout << "grasp:" << grasp << std::endl;

		unsigned int grip=i->grip_pose.direction.id;
		//std::cout << "direction:" << grip << std::endl;

		//unsigned int part=req.grasp.grip_pose.part.id;
		unsigned int distance=i->grip_pose.distance.id;
		//std::cout << "distance:" << distance << std::endl;

		unsigned int shape=object.data.type.shape.value;
		//std::cout << "shape:" << shape << std::endl;

		//unsigned int symmetry=req.object.data.type.symmetry.value;
		unsigned int size=msgs_size_handle_size_map.find(object.data.type.size.id)->second;
		//std::cout << "size:" << size << std::endl;

		unsigned int object=size_shape_object_map.find(std::pair<int,int>(size,shape))->second;
		//std::cout << "object:" << object << std::endl;

		i->success_probability*=graspability_map[grasp][grip][0][distance][object][0];
	}*/

	// Ros markers for debug
	fillGraspsMarkers(grasps, object);

	marker_pub.publish(marker_array);

	return grasps;
}


bool GraspCandidateListManager::serviceCallback(ist_grasp_generation_msgs::GetEvaluatedGrasps::Request  &req, ist_grasp_generation_msgs::GetEvaluatedGrasps::Response &res)
{
	ROS_INFO("Grasp candidate list manager service called...");
	// Discard grasps according to some basic rules
	//if(req.object.object_id==0)
		res.grip_list=discardGrasps(req.grip_list, req.object); // for known object -> don't prune!!!
	//else
		//res.grip_list=req.grip_list;

	return true;
}

void GraspCandidateListManager::graspabilityMapToImage()
{
    // Allocate a 1-channel byte image
	int lines=  (GRIP_SPACE_SIZE)*(OBJECT_SPACE_SIZE);
	int columns=(GRASP_SPACE_SIZE)*(DISTANCE_SPACE_SIZE);

	//std::cout << "LINES: " << lines << std::endl;
	//std::cout << "COLUMNS: " << columns << std::endl;

	IplImage* graspability_map_image=cvCreateImage(cvSize(columns, lines),IPL_DEPTH_8U,1);

	for(unsigned int grasp_index=0; grasp_index < GRASP_SPACE_SIZE; ++grasp_index)
	{
		for(unsigned int distance_index=0; distance_index < DISTANCE_SPACE_SIZE; ++distance_index)
		{
			for(unsigned int object_index=0; object_index < OBJECT_SPACE_SIZE; ++object_index)
			{
				for(unsigned int grip_index=0; grip_index < GRIP_SPACE_SIZE; ++grip_index)
				{
					CvScalar s;
					unsigned int line_index=grip_index+object_index*GRIP_SPACE_SIZE;
					unsigned int column_index=distance_index+grasp_index*DISTANCE_SPACE_SIZE;

					s.val[0]=(int)floor(255*graspability_map[grasp_index+1][grip_index+1][0][distance_index+1][object_index+1][0]);
//					std::cout << s.val[0] << std::endl;
					cvSet2D(graspability_map_image, line_index, column_index ,s); // set the (i, j) pixel value
				}
			}
		}
	}
    cv::waitKey(300);

    /*try
	{
		graspability_map_image_pub_.publish(bridge_.cvToImgMsg(graspability_map_image, "mono8"));
		ROS_INFO("Graspability map image published.");

	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error publishing image.");
    }*/

}



void GraspCandidateListManager::graspabilityMapSingleObjectToImage(const unsigned int object_index)
{
    // Allocate a 1-channel byte image
	int lines=  (GRIP_SPACE_SIZE);
	int columns=(GRASP_SPACE_SIZE)*(DISTANCE_SPACE_SIZE);

//	std::cout << "LINES: " << lines << std::endl;
//	std::cout << "COLUMNS: " << columns << std::endl;

	IplImage* graspability_map_image=cvCreateImage(cvSize(columns, lines),IPL_DEPTH_8U,1);

	for(unsigned int grasp_index=0; grasp_index < GRASP_SPACE_SIZE; ++grasp_index)
	{
		for(unsigned int distance_index=0; distance_index < DISTANCE_SPACE_SIZE; ++distance_index)
		{
			for(unsigned int grip_index=0; grip_index < GRIP_SPACE_SIZE; ++grip_index)
			{
				CvScalar s;
				unsigned int line_index=grip_index;
				unsigned int column_index=distance_index+grasp_index*DISTANCE_SPACE_SIZE;
				s.val[0]=(int)floor(255*graspability_map[grasp_index+1][grip_index+1][0][distance_index+1][object_index][0]);
				cvSet2D(graspability_map_image, line_index, column_index ,s); // set the (i, j) pixel value
			}
		}
	}


	double percent=100*(640/columns);
	// declare a destination IplImage object with correct size, depth and channels
    IplImage *destination = cvCreateImage( cvSize((int)((graspability_map_image->width*percent)/100) , (int)((graspability_map_image->height*percent)/100) ),
			graspability_map_image->depth, graspability_map_image->nChannels );

//	IplImage *destination = cvCreateImage( cvSize((int)(640) , (int)((640.0/480.0)*graspability_map_image->height) ),
//			graspability_map_image->depth, graspability_map_image->nChannels );


	//use cvResize to resize source to a destination image
	cvResize(graspability_map_image, destination,CV_INTER_NN );
  //  cvShowImage("graspability map (OpenCV)", destination );                   // Show our image inside it.

    cv::waitKey(300);
    /*try
	{
		graspability_object_map_image_pub_.publish(bridge_.cvToImgMsg(destination, "mono8"));
		ROS_INFO("Publish image for object %d.",object_index);
	}
	catch (sensor_msgs::CvBridgeException error)
	{
		ROS_ERROR("error publishing image.");
    }*/
}

bool GraspCandidateListManager::newExperimentServiceCallback(ist_grasp_generation_msgs::NewExperiment::Request &req, ist_grasp_generation_msgs::NewExperiment::Response &res)
{
	unsigned int grasp=req.grasp.hand_state.grasp_posture.grasp_type;
	unsigned int grip=req.grasp.grip_pose.direction.id;
	//unsigned int part=req.grasp.grip_pose.part.id;
	unsigned int distance=req.grasp.grip_pose.distance.id;

	unsigned int shape=req.object.data.type.shape.value;
	//unsigned int symmetry=req.object.data.type.symmetry.value;

	unsigned int size=msgs_size_handle_size_map.find(req.object.data.type.size.id)->second;

	/*bool success=req.success;

	unsigned int object=size_shape_object_map.find(std::pair<int,int>(size,shape))->second;

	double experiment_success=0;
	if(success==true)
	{
		experiment_success=1;
	}

	incorporateKnowledge(grasp, grip, 0, distance, object, 0, 1, experiment_success, ROBOT); // incorporate new experiment knowledge

	return true;*/
}

void GraspCandidateListManager::fillGraspsMarkers(ist_msgs::GripList & grasps, ist_msgs::Object & object)
{
	////////////////////////
	// Grasps rviz marker //
	////////////////////////

	marker_array.markers.clear();

	unsigned int marker_id=0;

	tf::Transform world_to_object_transform; // Object pose in world frame
	if(object.state.graspable_object.potential_models.size()<1)
		return;
	tf::poseMsgToTF(object.state.graspable_object.potential_models[0].pose.pose, world_to_object_transform);
	for(std::vector<ist_msgs::GripState>::iterator i=grasps.grip_states.begin(); i != grasps.grip_states.end(); ++i)
	{
		tf::Transform object_to_hand_transform; // Hand pose in object frame
		tf::poseMsgToTF(i->hand_state.grasp_pose.pose.pose, object_to_hand_transform);

		tf::Transform world_to_hand_transform; // Hand pose in world frame
		world_to_hand_transform.mult(world_to_object_transform, object_to_hand_transform);

		visualization_msgs::Marker marker;
		marker.id = marker_id++;
		marker.header.frame_id = world_frame_id;
		marker.header.stamp = ros::Time::now();

		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;

		//std::cout << world_to_hand_transform.getRotation().getX() << world_to_hand_transform.getRotation().getY() << world_to_hand_transform.getRotation().getZ() << world_to_hand_transform.getRotation().getW() << std::endl;
		marker.pose.orientation.x=world_to_hand_transform.getRotation().getX();
		marker.pose.orientation.y=world_to_hand_transform.getRotation().getY();
		marker.pose.orientation.z=world_to_hand_transform.getRotation().getZ();
		marker.pose.orientation.w=world_to_hand_transform.getRotation().getW();

		marker.pose.position.x=world_to_hand_transform.getOrigin().getX();
		marker.pose.position.y=world_to_hand_transform.getOrigin().getY();
		marker.pose.position.z=world_to_hand_transform.getOrigin().getZ();

		marker.scale.x=0.05;
		marker.scale.y=0.05;
		marker.scale.z=0.05;

		marker.lifetime=ros::Duration(30);
		marker.color.a=(float)marker_id/grasps.grip_states.size();
		marker.color.r=1.0;
		marker.color.g=0.0;
		marker.color.b=0.0;

		marker.ns = "grasp_arrows";
		marker_array.markers.push_back(marker);
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ist_grasp_candidate_list_manager");

	ros::NodeHandle n;

	GraspCandidateListManager GraspCandidateListManager(n);

	ros::spin();
}
