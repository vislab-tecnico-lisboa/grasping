/*
 * ReachabilityMap.cpp
 *
 *  Created on: Oct 23, 2013
 *      Author: Rui P. Figueiredo
 */

#include "ReachabilityMap.h"


//double ReachabilityMap::x_length_;
//double ReachabilityMap::y_length_;
//double ReachabilityMap::z_length_;
//
//unsigned int ReachabilityMap::x_bins_;
//unsigned int ReachabilityMap::y_bins_;
//unsigned int ReachabilityMap::z_bins_;
//unsigned int ReachabilityMap::yaw_bins_;
//unsigned int ReachabilityMap::pitch_bins_;
//unsigned int ReachabilityMap::roll_bins_;
//
//double ReachabilityMap::x_step_;
//double ReachabilityMap::y_step_;
//double ReachabilityMap::z_step_;
//double ReachabilityMap::yaw_step_;
//double ReachabilityMap::pitch_step_;
//double ReachabilityMap::roll_step_;
//
//unsigned int ReachabilityMap::x_perturbs_;
//unsigned int ReachabilityMap::y_perturbs_;
//unsigned int ReachabilityMap::z_perturbs_;
//unsigned int ReachabilityMap::yaw_perturbs_;
//unsigned int ReachabilityMap::pitch_perturbs_;
//unsigned int ReachabilityMap::roll_perturbs_;
//
//double ReachabilityMap::x_perturb_step_;
//double ReachabilityMap::y_perturb_step_;
//double ReachabilityMap::z_perturb_step_;
//double ReachabilityMap::yaw_perturb_step_;
//double ReachabilityMap::pitch_perturb_step_;
//double ReachabilityMap::roll_perturb_step_;
double fps;
clock_t time1;
void tic(){ time1=clock(); }
double tac(){ return (double)(clock()-time1)/(double)CLOCKS_PER_SEC; }
void tac_print()
{
	double time = (double)(clock()-time1)/(double)CLOCKS_PER_SEC;
	ROS_INFO_STREAM ("time(s):"<<time);
}

ReachabilityMap::ReachabilityMap(ros::NodeHandle & n) :
		n_(n),
		n_priv_("~")

{

	/////////////////////////////
	// Reachability map params //
	/////////////////////////////

    n_priv_.param<std::string>("arm_frame_id", arm_frame_id_, "manipulator_base_frame");
	ROS_INFO("arm_frame_id: %s", arm_frame_id_.c_str());

	std::string file_name;
    n_priv_.param<std::string>("reachability_file_name",  file_name, "reachability");
	ROS_INFO("reachability_file_name: %s", file_name.c_str());

	std::string reachability_file_directory;
    n_priv_.param<std::string>("reachability_file_directory",  reachability_file_directory, "reachability_file_directory");
	ROS_INFO("reachability_file_directory: %s", reachability_file_directory.c_str());

    n_priv_.param<double>("reachability_x_length", x_length_, 1.5);
	ROS_INFO_STREAM("reachability_x_length: " << x_length_);

    n_priv_.param<double>("reachability_y_length", y_length_, 1.5);
	ROS_INFO_STREAM("reachability_y_length: " << y_length_);

    n_priv_.param<double>("reachability_z_length", z_length_, 1.5);
	ROS_INFO_STREAM("reachability_z_length: " << z_length_);

    n_priv_.param<int>("reachability_x_bins", x_bins_, 10);
	ROS_INFO_STREAM("reachability_x_bins: " << x_bins_);

    n_priv_.param<int>("reachability_y_bins", y_bins_, 10);
	ROS_INFO_STREAM("reachability_y_bins: " << y_bins_);

    n_priv_.param<int>("reachability_z_bins", z_bins_, 10);
	ROS_INFO_STREAM("reachability_z_bins: " << z_bins_);

    n_priv_.param<int>("reachability_x_perturbs", x_perturbs_, 1);
	ROS_INFO_STREAM("reachability_x_perturbs: " << x_perturbs_);

    n_priv_.param<int>("reachability_y_perturbs", y_perturbs_, 1);
	ROS_INFO_STREAM("reachability_y_perturbs: " << y_perturbs_);

    n_priv_.param<int>("reachability_z_perturbs", z_perturbs_, 10);
	ROS_INFO_STREAM("reachability_z_perturbs: " << z_perturbs_);

    n_priv_.param<int>("reachability_roll_perturbs", roll_perturbs_, 1);
	ROS_INFO_STREAM("reachability_roll_perturbs: " << roll_perturbs_);

    n_priv_.param<int>("reachability_pitch_perturbs", pitch_perturbs_, 1);
	ROS_INFO_STREAM("reachability_pitch_perturbs: " << pitch_perturbs_);

    n_priv_.param<int>("reachability_yaw_perturbs", yaw_perturbs_, 1);
	ROS_INFO_STREAM("reachability_yaw_perturbs: " << yaw_perturbs_);

	x_step_=x_length_/x_bins_;
	y_step_=y_length_/y_bins_;
	z_step_=z_length_/z_bins_;

	if(x_perturbs_!=0)
	{
		x_perturb_step_=0.5*x_step_/x_perturbs_;
	}
	else
		x_perturb_step_=0.0;

	if(y_perturbs_!=0)
	{
		y_perturb_step_=0.5*y_step_/y_perturbs_;
	}
	else
		y_perturb_step_=0.0;

	if(z_perturbs_!=0)
	{
		z_perturb_step_=0.5*z_step_/z_perturbs_;
	}
	else
		z_perturb_step_=0.0;


	if (roll_perturbs_!=0)
	{
		roll_perturb_step_=0.25*PI/roll_perturbs_;
	}
	else
		roll_perturb_step_=0.0;
	if(pitch_perturbs_!=0)
	{
		pitch_perturb_step_=0.25*PI/pitch_perturbs_;
	}
	else
		pitch_perturb_step_=0.0;

	if(yaw_perturbs_!=0)
	{
		yaw_perturb_step_=0.25*PI/yaw_perturbs_;
	}
	else
		yaw_perturb_step_=0.0;

	inverse_kinematics_client_ = n_.serviceClient<ist_grasp_generation_msgs::InverseKinematics>("inverse_kinematics");

	inverse_kinematics_batch_client_ = n_.serviceClient<ist_grasp_generation_msgs::InverseKinematicsBatch>("inverse_kinematics_batch");

	reachability_grid_pub_ = n_.advertise<visualization_msgs::Marker>("reachability_grid", 2);

	tf::TransformListener listener;
	try
	{
//		ros::Time now = ros::Time::now();
		ros::Time now = ros::Time(0);
		ROS_INFO("Wait for transform from %s to %s", world_frame_id_.c_str(),arm_frame_id_.c_str());
		listener.waitForTransform(world_frame_id_, arm_frame_id_, now, ros::Duration(30.0));
		listener.lookupTransform(world_frame_id_, arm_frame_id_, now, world_to_arm_transform_);
		ROS_INFO("Transform acquired.");
	}
	catch (tf::TransformException & ex)
	{
		ROS_INFO("Transform not acquired.");
		exit(-1);
	}

	tf::poseTFToMsg(world_to_arm_transform_,world_to_arm_msg_);
	Eigen::Transform<double,3,Eigen::Affine> world_to_arm_eigen;
	tf::poseMsgToEigen (world_to_arm_msg_, world_to_arm_eigen);
	Eigen::Matrix4d world_to_arm_eigen_matrix(world_to_arm_eigen.matrix());

	offset_=Eigen::Vector3d(world_to_arm_eigen_matrix(0,3),world_to_arm_eigen_matrix(1,3),world_to_arm_eigen_matrix(2,3));

	//////////////////////////
	// Load canonical grips //
	//////////////////////////

	XmlRpc::XmlRpcValue considered_canonical_grips;
	n_priv_.getParam("canonical_grips", considered_canonical_grips);

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
        		  std::cout << rotation_matrix_eigen <<std::endl;
        		  Eigen::Transform<double, 3, Eigen::Affine> orientation(rotation_matrix_eigen);
        		  boost::shared_ptr<CanonicalGrip> canonical_grip(new CanonicalGrip(id,name,grip_direction,orientation));
        		  CanonicalGrip::canonical_grips.insert(std::pair<unsigned int, boost::shared_ptr<CanonicalGrip> >(id, canonical_grip));
        	  }
          }
        }
	}

	std::cout << CanonicalGrip::canonical_grips.size() << std::endl;

	std::string file=reachability_file_directory+file_name;
	if(!loadData(file))
	{
		ROS_INFO("Train...");
		batchTrain();
		//train();
		saveData(file_name);
	}
	else
	{
		std::ofstream myfile;
		myfile.open ("reachability_map.txt");
		myfile << "x_length: " << x_length_ << std::endl;
		myfile << "y_length: " << y_length_ << std::endl;
		myfile << "z_length: " << z_length_ << std::endl;

		myfile << "x_bins: " <<x_bins_ << std::endl;
		myfile << "y_bins: " <<y_bins_ << std::endl;
		myfile << "z_bins: " <<z_bins_ << std::endl;
		myfile << "grips: " <<CanonicalGrip::canonical_grips.size() << std::endl;
		myfile << "total_iterations: " << total_iterations_ << std::endl;
		myfile << "total_time: " << total_time_ << std::endl;
		std::cout << "VOU IMPRIMIR PARA FICHEIRO!!!!" << std::endl;
		for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
		{
			for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
			{
				for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
				{
					for(unsigned int grip_index=0; grip_index < CanonicalGrip::canonical_grips.size(); ++grip_index)
					{

						myfile << reachability_map[x_index][y_index][z_index][grip_index] << " ";
						//myfile << ( static_cast<double>(reachability_successes[x_index][y_index][z_index][grip_index])+static_cast<double>(reachability_failures[x_index][y_index][z_index][grip_index]) ) <<" ";
					}

				}
			}
		}
		std::cout << "ja esta!" << std::endl;
		myfile.close();
	}


	displayGrid();
}


ReachabilityMap::~ReachabilityMap()
{
	for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
	{
		for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
		{
			for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
			{
					reachability_successes[x_index][y_index][z_index].clear();
					reachability_failures[x_index][y_index][z_index].clear();
					reachability_map[x_index][y_index][z_index].clear();
			}
			reachability_successes[x_index][y_index].clear();
			reachability_failures[x_index][y_index].clear();
			reachability_map[x_index][y_index].clear();
		}
		reachability_successes[x_index].clear();
		reachability_failures[x_index].clear();
		reachability_map[x_index].clear();
	}
	reachability_successes.clear();
	reachability_failures.clear();
	reachability_map.clear();
}




void ReachabilityMap::displayGrid()
{
	visualization_msgs::Marker reachability_cells;
	reachability_cells.type=visualization_msgs::Marker::POINTS;
	reachability_cells.action=visualization_msgs::Marker::ADD;
	reachability_cells.scale.x=x_step_;
	reachability_cells.scale.y=y_step_;
	reachability_cells.scale.z=z_step_;
    // Points are green
	reachability_cells.color.a = 1.0;
	tf::StampedTransform world_to_arm_transform_temp; // arm pose in world frame

	world_to_arm_transform_temp=world_to_arm_transform_;
	//world_to_arm_transform_temp.setOrigin(btVector3(0.0,0.0,0.0));
	tf::poseTFToMsg(world_to_arm_transform_temp.inverse(),reachability_cells.pose);

	std::cout << reachability_cells.pose.orientation.x <<" "<< reachability_cells.pose.orientation.y<<" " <<reachability_cells.pose.orientation.z<< " "<<reachability_cells.pose.orientation.w <<std::endl;

	std::cout << reachability_cells.pose.position.x <<" "<< reachability_cells.pose.position.y<<" " <<reachability_cells.pose.position.z<<std::endl;


//	reachability_cells.pose.orientation.w=1.0;
//

	reachability_cells.header.frame_id="/manipulator_base_frame";
	reachability_cells.header.stamp= ros::Time::now();
	for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
	{
		for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
		{
			for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
			{
				for(unsigned int grip_index=0; grip_index < CanonicalGrip::canonical_grips.size(); ++grip_index)
				{
					if(z_index!=9)
					//if(reachability_map[x_index][y_index][z_index][grip_index]<=0.5)
						continue;
					geometry_msgs::Point point;
					point.x=x_index*x_step_ - x_length_/2.0 + x_step_/2.0 + offset_[0];
					point.y=y_index*y_step_ - y_length_/2.0 + y_step_/2.0 + offset_[1];
					point.z=z_index*z_step_ - z_length_/2.0 + z_step_/2.0 + offset_[2];

					std::cout <<"x:"<<point.x<<" y:"<< point.y<<" z:"<<point.z << " prob:" << reachability_map[x_index][y_index][z_index][grip_index]<< std::endl;

					reachability_cells.points.push_back(point);

					std_msgs::ColorRGBA color;
					color.r=reachability_map[x_index][y_index][z_index][grip_index];
					//color.g=reachability_map[x_index][y_index][z_index][grip_index];
					//color.b=reachability_map[x_index][y_index][z_index][grip_index];
				    color.a = 0.5;

					reachability_cells.colors.push_back(color);
				}
			}
		}
	}
	ros::Rate r(0.5);

	ROS_INFO("VOU PUBLICAR");
	int i=0;
	while(i<3)
	{
		reachability_grid_pub_.publish(reachability_cells);
		ros::spinOnce();
		ROS_INFO("Publiquei");
		r.sleep();
		i++;
	}

	while(1)
	{		r.sleep();
	}

}


void ReachabilityMap::batchTrain()
{
	///////////////////////////////////////////////////////////////////
	// Get necessary transformation to center the arm in world frame //
	///////////////////////////////////////////////////////////////////

	tf::StampedTransform world_to_arm_transform_; // arm pose in world frame

	tf::TransformListener listener;
	try
	{
//		ros::Time now = ros::Time::now();
		ros::Time now = ros::Time(0);
		ROS_INFO("Wait for transform from %s to %s", world_frame_id_.c_str(),arm_frame_id_.c_str());
		listener.waitForTransform(world_frame_id_, arm_frame_id_, now, ros::Duration(30.0));
		listener.lookupTransform(world_frame_id_, arm_frame_id_, now, world_to_arm_transform_);
		ROS_INFO("Transform acquired.");
	}
	catch (tf::TransformException & ex)
	{
		ROS_INFO("Transform not acquired.");
		exit(-1);
	}

	tf::poseTFToMsg(world_to_arm_transform_,world_to_arm_msg_);
	Eigen::Transform<double,3,Eigen::Affine> world_to_arm_eigen;
	tf::poseMsgToEigen (world_to_arm_msg_, world_to_arm_eigen);

	///////////////////////
	// Memory allocation //
	///////////////////////
	reachability_successes.resize(x_bins_);
	reachability_failures.resize(x_bins_);
	reachability_map.resize(x_bins_);
	for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
	{
		reachability_successes[x_index].resize(y_bins_);
		reachability_failures[x_index].resize(y_bins_);
		reachability_map[x_index].resize(y_bins_);
		for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
		{
			reachability_successes[x_index][y_index].resize(z_bins_);
			reachability_failures[x_index][y_index].resize(z_bins_);
			reachability_map[x_index][y_index].resize(z_bins_);
			for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
			{
				reachability_successes[x_index][y_index][z_index].resize(CanonicalGrip::canonical_grips.size());
				reachability_failures[x_index][y_index][z_index].resize(CanonicalGrip::canonical_grips.size());
				reachability_map[x_index][y_index][z_index].resize(CanonicalGrip::canonical_grips.size());
				for(unsigned int grip_index=0; grip_index < CanonicalGrip::canonical_grips.size(); ++grip_index)
				{
					reachability_successes[x_index][y_index][z_index][grip_index]=0;
					reachability_failures[x_index][y_index][z_index][grip_index]=0;
					reachability_map[x_index][y_index][z_index][grip_index]=1;
				}
			}
		}
	}

	//////////////
	// Training //
	//////////////

	int iteration=0;
	tic();

	Eigen::Matrix4d world_to_arm_eigen_matrix(world_to_arm_eigen.matrix());

	total_iterations_=0;
	int total_bins=x_bins_*y_bins_*z_bins_*CanonicalGrip::canonical_grips.size();
	// X
	for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
	{
		double x_lower_bound=x_index*x_step_ - x_length_/2.0;
		double x_middle=x_lower_bound + x_step_/2.0;
		// Y
		for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
		{
			double y_lower_bound=(double)y_index*y_step_ - y_length_/2.0;
			double y_middle=y_lower_bound + y_step_/2.0;

			// Z
			for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
			{
				double z_lower_bound=(double)z_index*z_step_ - z_length_/2.0;
				double z_middle=z_lower_bound + z_step_/2.0;

				// Grip
				int grip_index=0;
				for(std::map<unsigned int, boost::shared_ptr<CanonicalGrip> >::const_iterator canonical_grip_it=CanonicalGrip::canonical_grips.begin(); canonical_grip_it!=CanonicalGrip::canonical_grips.end(); ++canonical_grip_it)
				{
					int perturb_num=0;

					// Perturbs
					for(int x_perturb_index=static_cast<int>(-x_perturbs_); x_perturb_index <= static_cast<int>(x_perturbs_); ++x_perturb_index)
					{

						if(x_perturb_index==x_perturbs_ && x_perturbs_!=0)
							break;

//						double x=x_lower_bound + x_perturb_index*x_perturb_step_;
						double x=x_middle + x_perturb_index*x_perturb_step_;

						for(int y_perturb_index=static_cast<int>(-y_perturbs_); y_perturb_index <= static_cast<int>(y_perturbs_); ++y_perturb_index)
						{

							if(y_perturb_index==y_perturbs_ && y_perturbs_!=0)
								break;
//							double y=y_lower_bound + y_perturb_index*y_perturb_step_;
							double y=y_middle + y_perturb_index*y_perturb_step_;

							for(int z_perturb_index=static_cast<int>(-z_perturbs_); z_perturb_index <= static_cast<int>(z_perturbs_); ++z_perturb_index)
							{

								if(z_perturb_index==z_perturbs_ && z_perturbs_!=0)
									break;
//								double z=z_lower_bound + z_perturb_index*z_perturb_step_;
								double z=z_middle + z_perturb_index*z_perturb_step_;

								for(int roll_perturb_index=static_cast<int>(-roll_perturbs_); roll_perturb_index <= static_cast<int>(roll_perturbs_); ++roll_perturb_index)
								{
									if(roll_perturb_index==roll_perturbs_ && roll_perturbs_!=0)
										break;
									double roll=static_cast<double>(roll_perturb_index)*roll_perturb_step_;
									for(int pitch_perturb_index=static_cast<int>(-pitch_perturbs_); pitch_perturb_index <= static_cast<int>(pitch_perturbs_); ++pitch_perturb_index)
									{
										if(pitch_perturb_index==pitch_perturbs_ && pitch_perturbs_!=0)
											break;
										double pitch=static_cast<double>(pitch_perturb_index)*pitch_perturb_step_;
										for(int yaw_perturb_index=static_cast<int>(-yaw_perturbs_); yaw_perturb_index <= static_cast<int>(yaw_perturbs_); ++yaw_perturb_index)
										{
											if(yaw_perturb_index==yaw_perturbs_ && yaw_perturbs_!=0)
												break;

											double yaw=static_cast<double>(yaw_perturb_index)*yaw_perturb_step_;
											Eigen::Translation3d translation(x+offset_[0],y+offset_[1],z+offset_[2]);
											//Eigen::Translation3d translation(x,y,z);

											Eigen::Matrix3d perturb_rotation;
											fixedAnglesToRotationMatrix(roll, pitch, yaw, perturb_rotation);

											Eigen::Matrix3d rotation=(canonical_grip_it->second)->orientation.rotation() * perturb_rotation;

											Eigen::Transform<double,3,Eigen::Affine> pose=translation*rotation;
											geometry_msgs::PoseStamped pose_msg;
											pose_msg.header.frame_id="/base_link";
											tf::poseEigenToMsg (pose, pose_msg.pose); // BASE FRAME AND MANIPULATOR FRAME SHOULD HAVE SOME ORIENTATION

											InverseKinematicsBatchSrv_.request.pose.push_back(pose_msg);

											std::cout << "iteration:"<< iteration << " perturb:" << perturb_num << std::endl;

											++iteration;
										}
									}
								}
							}
						}
					}
					++grip_index;
				}
			}
		}
	}
	tac_print();

	bool service_called=false;
	tic();
	while(!service_called)
	{
		if(inverse_kinematics_batch_client_.call(InverseKinematicsBatchSrv_))
		{
			service_called=true;
		}
	}
	total_time_=tac();
	tac_print();



	iteration=0;
	for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
	{
		for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
		{
			for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
			{
				// Grip
				int grip_index=0;
				for(std::map<unsigned int, boost::shared_ptr<CanonicalGrip> >::const_iterator canonical_grip_it=CanonicalGrip::canonical_grips.begin(); canonical_grip_it!=CanonicalGrip::canonical_grips.end(); ++canonical_grip_it)
				{

					// Perturbs
					for(int x_perturb_index=static_cast<int>(-x_perturbs_); x_perturb_index <= static_cast<int>(x_perturbs_); ++x_perturb_index)
					{
						if(x_perturb_index==x_perturbs_ && x_perturbs_!=0)
							break;
						for(int y_perturb_index=static_cast<int>(-y_perturbs_); y_perturb_index <= static_cast<int>(y_perturbs_); ++y_perturb_index)
						{
							if(y_perturb_index==y_perturbs_ && y_perturbs_!=0)
								break;
							for(int z_perturb_index=static_cast<int>(-z_perturbs_); z_perturb_index <= static_cast<int>(z_perturbs_); ++z_perturb_index)
							{
								if(z_perturb_index==z_perturbs_ && z_perturbs_!=0)
									break;
								for(int roll_perturb_index=static_cast<int>(-roll_perturbs_); roll_perturb_index <= static_cast<int>(roll_perturbs_); ++roll_perturb_index)
								{
									if(roll_perturb_index==roll_perturbs_ && roll_perturbs_!=0)
										break;
									double roll=static_cast<double>(roll_perturb_index)*roll_perturb_step_;
									for(int pitch_perturb_index=static_cast<int>(-pitch_perturbs_); pitch_perturb_index <= static_cast<int>(pitch_perturbs_); ++pitch_perturb_index)
									{
										if(pitch_perturb_index==pitch_perturbs_ && pitch_perturbs_!=0)
											break;

										double pitch=static_cast<double>(pitch_perturb_index)*pitch_perturb_step_;
										for(int yaw_perturb_index=static_cast<int>(-yaw_perturbs_); yaw_perturb_index <= static_cast<int>(yaw_perturbs_); ++yaw_perturb_index)
										{
											if(yaw_perturb_index==yaw_perturbs_ && yaw_perturbs_!=0)
													break;
//
//											int index=
//													(yaw_perturb_index+yaw_perturbs_)+
//													(pitch_perturb_index+pitch_perturbs_)*(2*yaw_perturbs_)+
//													(roll_perturb_index+roll_perturbs_)  *(2*yaw_perturbs_)*(2*pitch_perturbs_)+
//													z_perturb_index                      *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)+
//													y_perturb_index                      *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)*z_perturbs_+
//													x_perturb_index                      *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)*z_perturbs_*y_perturbs_+
//													grip_index         					 *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)*z_perturbs_*y_perturbs_*x_perturbs_+
//													z_index            			 		 *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)*z_perturbs_*y_perturbs_*x_perturbs_*CanonicalGrip::canonical_grips.size()+
//													y_index            			 		 *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)*z_perturbs_*y_perturbs_*x_perturbs_*CanonicalGrip::canonical_grips.size()*z_bins_+
//													x_index            			 		 *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)*z_perturbs_*y_perturbs_*x_perturbs_*CanonicalGrip::canonical_grips.size()*z_bins_*y_bins_;
//
//											if (index!=iteration)
//											{
//												std::cout << "ERRO!!!!!!!" << std::endl;
//												exit(-1);
//											}

											if(InverseKinematicsBatchSrv_.response.success[iteration])
//											if(InverseKinematicsBatchSrv_.response.success[index])
											{
												++reachability_successes[x_index][y_index][z_index][grip_index];
												ROS_INFO("SUCCESS");
											}
											else
											{
												++reachability_failures[x_index][y_index][z_index][grip_index];										//ROS_INFO("NOT");
											}
											++iteration;
										}
									}
								}
							}
						}
					}
					++grip_index;
				}
			}
		}
	}

	total_iterations_=iteration;





	//////////////////////////////
	// Compute reachability map //
	//////////////////////////////

	for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
	{
		for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
		{
			for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
			{
				for(unsigned int grip_index=0; grip_index < CanonicalGrip::canonical_grips.size(); ++grip_index)
				{
					reachability_map[x_index][y_index][z_index][grip_index]=static_cast<double>(reachability_successes[x_index][y_index][z_index][grip_index]) / ( static_cast<double>(reachability_successes[x_index][y_index][z_index][grip_index])+static_cast<double>(reachability_failures[x_index][y_index][z_index][grip_index]) );
				}
			}
		}
	}

}










void ReachabilityMap::train()
{
	///////////////////////////////////////////////////////////////////
	// Get necessary transformation to center the arm in world frame //
	///////////////////////////////////////////////////////////////////


	tf::TransformListener listener;
	try
	{
//		ros::Time now = ros::Time::now();
		ros::Time now = ros::Time(0);
		ROS_INFO("Wait for transform from %s to %s", world_frame_id_.c_str(),arm_frame_id_.c_str());
		listener.waitForTransform(world_frame_id_, arm_frame_id_, now, ros::Duration(30.0));
		listener.lookupTransform(world_frame_id_, arm_frame_id_, now, world_to_arm_transform_);
		ROS_INFO("Transform acquired.");
	}
	catch (tf::TransformException & ex)
	{
		ROS_INFO("Transform not acquired.");
		exit(-1);
	}

	tf::poseTFToMsg(world_to_arm_transform_,world_to_arm_msg_);
	Eigen::Transform<double,3,Eigen::Affine> world_to_arm_eigen;
	tf::poseMsgToEigen (world_to_arm_msg_, world_to_arm_eigen);

	///////////////////////
	// Memory allocation //
	///////////////////////

	reachability_successes.resize(x_bins_);
	reachability_failures.resize(x_bins_);
	reachability_map.resize(x_bins_);
	for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
	{
		reachability_successes[x_index].resize(y_bins_);
		reachability_failures[x_index].resize(y_bins_);
		reachability_map[x_index].resize(y_bins_);
		for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
		{
			reachability_successes[x_index][y_index].resize(z_bins_);
			reachability_failures[x_index][y_index].resize(z_bins_);
			reachability_map[x_index][y_index].resize(z_bins_);
			for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
			{
				reachability_successes[x_index][y_index][z_index].resize(CanonicalGrip::canonical_grips.size());
				reachability_failures[x_index][y_index][z_index].resize(CanonicalGrip::canonical_grips.size());
				reachability_map[x_index][y_index][z_index].resize(CanonicalGrip::canonical_grips.size());
				for(unsigned int grip_index=0; grip_index < CanonicalGrip::canonical_grips.size(); ++grip_index)
				{
					reachability_successes[x_index][y_index][z_index][grip_index]=0;
					reachability_failures[x_index][y_index][z_index][grip_index]=0;
					reachability_map[x_index][y_index][z_index][grip_index]=1;
				}
			}
		}
	}

	//////////////
	// Training //
	//////////////

	int iteration=0;
	tic();

	Eigen::Matrix4d world_to_arm_eigen_matrix(world_to_arm_eigen.matrix());

	total_iterations_=0;
	int total_bins=x_bins_*y_bins_*z_bins_*CanonicalGrip::canonical_grips.size();
	// X
	for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
	{
		double x_lower_bound=x_index*x_step_ - x_length_/2.0;
		double x_middle=x_lower_bound + x_step_/2.0;
		// Y
		for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
		{
			double y_lower_bound=(double)y_index*y_step_ - y_length_/2.0;
			double y_middle=y_lower_bound + y_step_/2.0;

			// Z
			for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
			{
				double z_lower_bound=(double)z_index*z_step_ - z_length_/2.0;
				double z_middle=z_lower_bound + z_step_/2.0;

				// Grip
				int grip_index=0;
				for(std::map<unsigned int, boost::shared_ptr<CanonicalGrip> >::const_iterator canonical_grip_it=CanonicalGrip::canonical_grips.begin(); canonical_grip_it!=CanonicalGrip::canonical_grips.end(); ++canonical_grip_it)
				{
					int perturb_num=0;

					// Perturbs
					for(int x_perturb_index=static_cast<int>(-x_perturbs_); x_perturb_index <= static_cast<int>(x_perturbs_); ++x_perturb_index)
					{

						if(x_perturb_index==x_perturbs_ && x_perturbs_!=0)
							break;

//						double x=x_lower_bound + x_perturb_index*x_perturb_step_;
						double x=x_middle + x_perturb_index*x_perturb_step_;
						for(int y_perturb_index=static_cast<int>(-y_perturbs_); y_perturb_index <= static_cast<int>(y_perturbs_); ++y_perturb_index)
						{
							if(y_perturb_index==y_perturbs_ && y_perturbs_!=0)
								break;

//							double y=y_lower_bound + y_perturb_index*y_perturb_step_;
							double y=y_middle + y_perturb_index*y_perturb_step_;
							for(int z_perturb_index=static_cast<int>(-z_perturbs_); z_perturb_index <= static_cast<int>(z_perturbs_); ++z_perturb_index)
							{
								if(z_perturb_index==z_perturbs_ && z_perturbs_!=0)
									break;

//								double z=z_lower_bound + z_perturb_index*z_perturb_step_;
								double z=z_middle + z_perturb_index*z_perturb_step_;
								for(int roll_perturb_index=static_cast<int>(-roll_perturbs_); roll_perturb_index <= static_cast<int>(roll_perturbs_); ++roll_perturb_index)
								{
									if(roll_perturb_index==roll_perturbs_ && roll_perturbs_!=0)
										break;

									double roll=static_cast<double>(roll_perturb_index)*roll_perturb_step_;
									for(int pitch_perturb_index=static_cast<int>(-pitch_perturbs_); pitch_perturb_index <= static_cast<int>(pitch_perturbs_); ++pitch_perturb_index)
									{
										if(pitch_perturb_index==pitch_perturbs_ && pitch_perturbs_!=0)
											break;

										double pitch=static_cast<double>(pitch_perturb_index)*pitch_perturb_step_;
										for(int yaw_perturb_index=static_cast<int>(-yaw_perturbs_); yaw_perturb_index <= static_cast<int>(yaw_perturbs_); ++yaw_perturb_index)
										{
											if(yaw_perturb_index==yaw_perturbs_ && yaw_perturbs_!=0)
												break;
											std::cout << "roll:" << roll_perturb_index << " pitch:" << pitch_perturb_index <<  " yaw:" << yaw_perturb_index <<std::endl;

											double yaw=static_cast<double>(yaw_perturb_index)*yaw_perturb_step_;
											Eigen::Translation3d translation(x+offset_[0],y+offset_[1],z+offset_[2]);

											Eigen::Matrix3d perturb_rotation;
											fixedAnglesToRotationMatrix(roll, pitch, yaw, perturb_rotation);

											Eigen::Matrix3d rotation=(canonical_grip_it->second)->orientation.rotation() * perturb_rotation;
											Eigen::Transform<double,3,Eigen::Affine> pose=translation*rotation;
											InverseKinematicsSrv_.request.pose.header.frame_id="base_frame";
											tf::poseEigenToMsg (pose, InverseKinematicsSrv_.request.pose.pose);
//											tf::poseEigenToMsg (world_to_arm_eigen*pose, InverseKinematicsSrv_.request.pose.pose);

											inverse_kinematics_client_.call(InverseKinematicsSrv_);

											bool service_called=false;
											while(!service_called)
											{
												tic();
												if(inverse_kinematics_client_.call(InverseKinematicsSrv_))
												{
													service_called=true;
													total_time_+=tac();
												}
											}

											if(InverseKinematicsSrv_.response.success)
											{
												++reachability_successes[x_index][y_index][z_index][grip_index];
												ROS_INFO("SUCCESSSSSSSSSSS");
											}
											else
											{
												++reachability_failures[x_index][y_index][z_index][grip_index];										//ROS_INFO("NOT");
											}

											int perturb_num=
													(yaw_perturb_index+yaw_perturbs_)+
													(pitch_perturb_index+pitch_perturbs_)*(2*yaw_perturbs_)+
													(roll_perturb_index+roll_perturbs_)  *(2*yaw_perturbs_)*(2*pitch_perturbs_)+
													z_perturb_index                      *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)+
													y_perturb_index                      *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)*z_perturbs_+
													x_perturb_index                      *(2*yaw_perturbs_)*(2*pitch_perturbs_)*(2*roll_perturbs_)*z_perturbs_*y_perturbs_;

											std::cout << "iteration:"<< iteration << std::endl;
											std::cout << "perturb number: " << perturb_num << " grid box: " << poseToIndex(pose,grip_index) << " of " << total_bins << std::endl;
											++iteration;
										}
									}
								}
							}
						}
					}
					++grip_index;
				}
			}
		}
	}

	total_iterations_=iteration;

	//////////////////////////////
	// Compute reachability map //
	//////////////////////////////

	for(unsigned int x_index=0; x_index < x_bins_; ++x_index)
	{
		for(unsigned int y_index=0; y_index < y_bins_; ++y_index)
		{
			for(unsigned int z_index=0; z_index < z_bins_; ++z_index)
			{
				for(unsigned int grip_index=0; grip_index < CanonicalGrip::canonical_grips.size(); ++grip_index)
				{
					reachability_map[x_index][y_index][z_index][grip_index]=static_cast<double>(reachability_successes[x_index][y_index][z_index][grip_index]) / ( static_cast<double>(reachability_successes[x_index][y_index][z_index][grip_index])+static_cast<double>(reachability_failures[x_index][y_index][z_index][grip_index]) );
				}
			}
		}
	}
}




void ReachabilityMap::fixedAnglesToRotationMatrix(const double & roll, const double & pitch, const double & yaw, Eigen::Matrix3d & perturb_rotation)
{
	double c1 = cos(yaw);
	double s1 = sin(yaw);
    double c2 = cos(pitch);
   	double s2 = sin(pitch);
    double c3 = cos(roll);
    double s3 = sin(roll);
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;
    double c1s2 = c1*s2;



    perturb_rotation << c1c2,   c1s2*s3-s1*c3, c1s2*c3+s1*s3,
    					s1*c2,  s1s2*s3+c1*c3, s1s2*c3-c1*s3,
    					-s2,    c2*s3,          c2*c3;
}


/*void ReachabilityMap::getMapIndex(const double & x, const double & y, const double & z, double & roll, double & pitch, double & yaw)
{

}*/

// convert quaternion to euler

void ReachabilityMap::quaternionToEulerTest(const Eigen::Quaternion<double> & q, double & roll, double & pitch, double & yaw)
{
    double sqx = q.x()*q.x();
    double sqy = q.y()*q.y();
    double sqz = q.z()*q.z();
    double sqw = q.w()*q.w();

    roll = atan2f(2*(q.y()*q.z() + q.w()*q.x()), sqw - sqx - sqy + sqz);
    pitch = asinf(-2*( q.x()*q.z() - q.w()*q.y() ));
    yaw = atan2f(2*(q.x()*q.y() - q.w()*q.z()) , sqw + sqx - sqy - sqz);
}

void ReachabilityMap::quaternionToEuler(const Eigen::Quaternion<double> & q, double & roll, double & pitch, double & yaw)
{
	double test = q.x()*q.z() + q.y()*q.w();
	if (test > 0.49999)
	{ // singularity at north pole
		std::cout << "entrei aqui2" << std::endl;

		//roll = 0.0; // symmetric!!
		roll = 2 * atan2(q.z(),q.w());
		pitch = PI/2.0;
		yaw = 0.0;
		return;
	}
	if (test < -0.49999)
	{ // singularity at south pole
		std::cout << "entrei aqui" << std::endl;
		//roll = 0.0; // symmetric!!
		roll = -2 * atan2(q.z(),q.w());
		pitch = - PI/2.0;
		yaw = 0.0;
		return;
	}
    double sqx = q.x()*q.x();
    double sqy = q.y()*q.y();
    double sqz = q.z()*q.z();

    roll = atan2f(2*q.x()*q.w()-2*q.y()*q.z() , 1 - 2*sqx - 2*sqy);
    pitch = asinf(2*test);
    yaw = atan2f(2*q.z()*q.w()-2*q.y()*q.x() , 1 - 2*sqz - 2*sqy);

}


// convert pose to an index
int ReachabilityMap::poseToIndex(Eigen::Transform<double,3,Eigen::Affine> & pose, int & orientation_index)
{
	double x,y,z;
//	double roll, pitch, yaw;

	Eigen::Matrix4d transformation_matrix(pose.matrix());

	x=transformation_matrix(0,3);
	y=transformation_matrix(1,3);
	z=transformation_matrix(2,3);

//	roll=transformation_matrix(0,0);
//	pitch=transformation_matrix(1,1);
//	yaw=transformation_matrix(2,2);

	// orientation
	/*int rollDiscrete=floor(((roll+PI)/(2*PI)) * roll_bins_); // roll [-pi;pi]
	if(rollDiscrete==(int)roll_bins_) --rollDiscrete;
	else if((int)rollDiscrete==-1) ++rollDiscrete;

	int pitchDiscrete=floor(((pitch+PI)/(2*PI)) * pitch_bins_); // pitch [-pi/2; pi/2]
	if(pitchDiscrete==(int)pitch_bins_) --pitchDiscrete;
	else if((int)pitchDiscrete==-1) ++pitchDiscrete;

	int yawDiscrete=floor(((yaw+PI)/(2*PI)) * yaw_bins_); // yaw [-pi;pi]
	if(yawDiscrete==(int)yaw_bins_) --yawDiscrete;
	else if((int)yawDiscrete==-1) ++yawDiscrete;*/

	double delta = 0.000001; //stability
	int xDiscrete=floor(((x-offset_[0]+(x_length_/2.0))/x_length_)*x_bins_+delta);
	if(xDiscrete==(int)x_bins_) --xDiscrete;

	int yDiscrete=floor(((y-offset_[1]+(y_length_/2.0))/y_length_)*y_bins_+delta);
	if(yDiscrete==(int)y_bins_) --yDiscrete;

	int zDiscrete=floor(((z-offset_[2]+(z_length_/2.0))/z_length_)*z_bins_+delta);
	if(zDiscrete==(int)z_bins_) --zDiscrete;


//	std::cout << "DEBUG:" <<x-offset_[0]+(x_length_/2.0)<<" "<<y-offset_[1]+(y_length_/2.0)<<" "<<z-offset_[2]+(z_length_/2.0)<<std::endl;
//	std::cout << "DEBUG:" <<((x-offset_[0]+(x_length_/2.0))/x_length_)<<" "<<((y-offset_[1]+(y_length_/2.0))/y_length_)<<" "<<((z-offset_[2]+(z_length_/2.0))/z_length_)<<std::endl;
	//std::cout << "DEBUG:" <<((x-offset_[0]+(x_length_/2.0))/x_length_)*x_bins_+delta<<" "<<((y-offset_[1]+(y_length_/2.0))/y_length_)*y_bins_+delta<<" "<<((z-offset_[2]+(z_length_/2.0))/z_length_)*z_bins_+delta<<std::endl;

	//std::cout << "DEBUG:" <<xDiscrete<<" "<<yDiscrete<<" "<<zDiscrete<<std::endl;

//	unsigned long long index=(unsigned long long)((yawDiscrete) +
//		  (pitchDiscrete *static_cast<unsigned long long>(yaw_bins_)) +
//		  (rollDiscrete  *static_cast<unsigned long long>(yaw_bins_)*static_cast<unsigned long long>(pitch_bins_)) +
//		  (zDiscrete     *static_cast<unsigned long long>(yaw_bins_)*static_cast<unsigned long long>(pitch_bins_)*static_cast<unsigned long long>(roll_bins_)) +
//		  (yDiscrete     *static_cast<unsigned long long>(yaw_bins_)*static_cast<unsigned long long>(pitch_bins_)*static_cast<unsigned long long>(roll_bins_)*static_cast<unsigned long long>(z_bins_)) +
//		  (xDiscrete     *static_cast<unsigned long long>(yaw_bins_)*static_cast<unsigned long long>(pitch_bins_)*static_cast<unsigned long long>(roll_bins_)*static_cast<unsigned long long>(z_bins_)*static_cast<unsigned long long>(y_bins_)));


		unsigned long long index=(unsigned long long)(orientation_index) +
			  (zDiscrete*static_cast<unsigned long long>(CanonicalGrip::canonical_grips.size()) +
			  (yDiscrete*static_cast<unsigned long long>(CanonicalGrip::canonical_grips.size())*static_cast<unsigned long long>(z_bins_)) +
			  (xDiscrete*static_cast<unsigned long long>(CanonicalGrip::canonical_grips.size())*static_cast<unsigned long long>(z_bins_)*static_cast<unsigned long long>(y_bins_)));


//	unsigned long long maxIndex=static_cast<unsigned long long>(roll_bins_)*static_cast<unsigned long long>(pitch_bins_)*static_cast<unsigned long long>(yaw_bins_)*static_cast<unsigned long long>(x_bins_)*static_cast<unsigned long long>(y_bins_)*static_cast<unsigned long long>(z_bins_);
//
//	//std::cout << index << " " << maxIndex << std::endl;
//	if(index > maxIndex)
//	{
//
//		std::cout << "ups:" << std::endl;
//		std::cout << "angle bins:" << roll_bins_ << " " <<  pitch_bins_ << " "<< yaw_bins_<< std::endl;
//		std::cout << "roll:" << roll << " " << rollDiscrete << std::endl;
//		std::cout << "pitch:" << pitch << " " <<  pitchDiscrete << std::endl;
//		std::cout << "yaw:" << yaw << " " << yawDiscrete << std::endl << std::endl;
//		std::cout << "position x bins:" << x_bins_ << std::endl;
//		std::cout << "position y bins:" << y_bins_ << std::endl;
//		std::cout << "position z bins:" << z_bins_ << std::endl;
//		std::cout << "x:" << x << " " << xDiscrete << std::endl;
//		std::cout << "y:" << y << " " << yDiscrete << std::endl;
//		std::cout << "z:" << z << " " << zDiscrete << std::endl;
//		std::cout << "index: " << index << " max index: " << maxIndex << std::endl;
//		exit(-1);
//	}

	return index;
}


// convert euler angles to quaternion
void ReachabilityMap::eulerToQuaternion(Eigen::Quaternion<double> & q, const double & roll, const double & pitch, const double & yaw)
{
	double roll2=roll*0.5;
	double pitch2=pitch*0.5;
	double yaw2=yaw*0.5;
	// Assuming the angles are in radians.
	double c1 = cos(roll2);
	double s1 = sin(roll2);
    double c2 = cos(pitch2);
   	double s2 = sin(pitch2);
    double c3 = cos(yaw2);
    double s3 = sin(yaw2);
    double c1c2 = c1*c2;
    double s1s2 = s1*s2;
    q.w() =c1c2*c3 - s1s2*s3;
   	q.x() =s1*c2*c3 + c1*s2*s3;
   	q.y() =c1*s2*c3 - s1*c2*s3;
	q.z() =c1c2*s3 + s1s2*c3;

    /*q.w() =c1c2*c3 - s1s2*s3;
   	q.x() =s1s2*c3 + c1c2*s3;
   	q.y() =s1*c2*c3 + c1*s2*s3;
	q.z() =c1*s2*c3 - s1*c2*s3;*/

	return;
}

