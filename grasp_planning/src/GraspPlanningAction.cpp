#include <grasp_planning/GraspPlanningAction.h>
#include <tf/transform_broadcaster.h>
GraspPlanningAction::GraspPlanningAction(std::string name) : as_(nh_, name, false), action_name_(name), group("arm"), collision_delta(0.0005)
{
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&GraspPlanningAction::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&GraspPlanningAction::preemptCB, this));

    //subscribe to the data topic of interest
    as_.start();

    group.setPoseReferenceFrame("base_link");
    //group.setEndEffectorLink("palm_frame");
    group.setEndEffectorLink("end_effector");

    // (Optional) Create a publisher for visualizing plans in Rviz.
    display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);



    tf::TransformListener listener;
    tf::StampedTransform end_effector_palm_transform;
    //object_to_grasp.state.graspable_object.potential_models[0].pose.pose;

    try
    {
        ros::Time now = ros::Time::now();
        listener.waitForTransform("palm_frame", "end_effector",
                                  ros::Time(0), ros::Duration(3.0));

        listener.lookupTransform("palm_frame", "end_effector",
                                 ros::Time(0), end_effector_palm_transform);

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    tf::transformTFToEigen(end_effector_palm_transform, transform_end_effector_to_palm);

    close_gripper_client = nh_.serviceClient<std_srvs::Empty>("close_gripper");
    open_gripper_client = nh_.serviceClient<std_srvs::Empty>("open_gripper");
    add_object_collision_server =  nh_.advertiseService("add_objects_collision", &GraspPlanningAction::objectsToCollisionEnvironment, this);
    attached_object_publisher = nh_.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 1);
}

void GraspPlanningAction::goalCB()
{
    ist_grasp_generation_msgs::GenerateTrajectoriesGoalConstPtr goal_;
    goal_ = as_.acceptNewGoal();

    ist_msgs::Object object_to_grasp=goal_->object_to_grasp;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    try
    {
        ros::Time now = ros::Time::now();
        listener.waitForTransform("base_link", object_to_grasp.collision_name,
                                  ros::Time(0), ros::Duration(3.0));

        listener.lookupTransform("base_link", object_to_grasp.collision_name,
                                 ros::Time(0), transform);
        std::cout << transform.getOrigin().getX() << std::endl;

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }

    Eigen::Affine3d transform_object_eigen;
    tf::transformTFToEigen(transform, transform_object_eigen);

    bool good_plan=false;
    moveit::planning_interface::MoveGroup::Plan my_plan;

    ist_msgs::GripList grip_list=goal_->grip_list;
    group.setPlanningTime(0.5);
    ist_msgs::GripState chosen_grip;

    for(int i=0; i< grip_list.grip_states.size(); ++i)
    {
        //ROS_INFO("trying grip:",grip_list.grip_states[i].);
        Eigen::Affine3d transform_grip_eigen;

        tf::poseMsgToEigen(grip_list.grip_states[i].hand_state.grasp_pose.pose.pose,transform_grip_eigen);

        Eigen::Affine3d final_transform = transform_object_eigen*transform_grip_eigen*transform_end_effector_to_palm;

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(final_transform.translation().x(), final_transform.translation().y(), final_transform.translation().z()) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "gripper_pose"));

        //std::cout << final_transform.matrix() << std::endl;
        group.setPoseTarget(final_transform);

        bool good_plan=group.plan(my_plan);

        if(good_plan)
        {
            ROS_INFO("GOOD PLAN");
            //group.asyncExecute(my_plan);
            chosen_grip=grip_list.grip_states[i];
            print_grip((int)chosen_grip.grip_pose.direction.id);
            break;
        }
        else
        {
            ROS_INFO("BAD PLAN");
        }
    }

    bool success=group.execute(my_plan);

    if(!success)
    {
        ROS_INFO("FAILED MOVING...");
        as_.setAborted();
        return;
    }

    ///////////////////
    // Close Gripper //
    ///////////////////

    std_srvs::Empty srv;
    if (!close_gripper_client.call(srv))
    {
        as_.setAborted(result_);
        ROS_ERROR("Failed to call service close gripper");
        return;
    }

    sleep(6.0);

    //////////////////////////////
    // Attach object to gripper //
    //////////////////////////////

    std::vector<std::string> ids;
    ids.push_back(collision_objects[0].id);
    planning_scene_interface.removeCollisionObjects(ids);

    while(attached_object_publisher.getNumSubscribers() < 1)
    {
      ros::WallDuration sleep_t(0.5);
      sleep_t.sleep();
    }



    moveit_msgs::AttachedCollisionObject attached_object;
    attached_object.link_name = "finger_1";
    attached_object.object.header.frame_id = "finger_1";
    attached_object.object=collision_objects[0];
    attached_object_publisher.publish(attached_object);

    ///////////////////////////////////
    // Goto predefined last position //
    ///////////////////////////////////

    Eigen::Affine3d transformation;
    Eigen::Vector3d p(0.0216893, -0.414892, 0.341879);
    Eigen::Quaterniond quaternion(-0.00290399, 0.7214, -0.00289372, 0.692506);
    transformation = Eigen::Translation3d(p) * quaternion;
    group.setPoseTarget(transformation);
    good_plan=group.plan(my_plan);

    if(good_plan)
    {
        group.execute(my_plan);
        ROS_INFO("GOOD PLAN!!!!");
    }
    else
    {
        ROS_INFO("BAD PLAN");
    }

    sleep(6.0);

    ///////////////////////////////
    // Drop object: Open Gripper //
    ///////////////////////////////

    if (!open_gripper_client.call(srv))
    {
        as_.setAborted(result_);
    }
    else
    {
        as_.setSucceeded(result_);
    }

    attached_object.object.operation = attached_object.object.REMOVE;
    attached_object_publisher.publish(attached_object);
    planning_scene_interface.removeCollisionObjects(ids);

    return;
}

void GraspPlanningAction::preemptCB()
{
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void GraspPlanningAction::analysisCB(const ist_msgs::GripList& msg)
{
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
        return;

    //        data_count_++;
    //        feedback_.sample = data_count_;
    //        feedback_.data = msg->data;
    //        //compute the std_dev and mean of the data
    //        sum_ += msg->data;
    //        feedback_.mean = sum_ / data_count_;
    //        sum_sq_ += pow(msg->data, 2);
    //        feedback_.std_dev = sqrt(fabs((sum_sq_/data_count_) - pow(feedback_.mean, 2)));
    //        as_.publishFeedback(feedback_);
    // specify that our target will be a random one
    group.setRandomTarget();
    // plan the motion and then move the group to the sampled target
    bool success=group.move();
    if(success)
        as_.setSucceeded(result_);
    else
        as_.setAborted();
    ROS_INFO("INSIDE ANALYSIS");

}

bool GraspPlanningAction::objectsToCollisionEnvironment(ist_grasp_generation_msgs::AddObjectCollision::Request  &req,
                                                        ist_grasp_generation_msgs::AddObjectCollision::Response &res)
{
    ROS_INFO("Add objects to collision environment... ");

    // for each old dynamic object
    //        for(uint32_t c = 0; c < collision_objects.size(); ++c)
    //        {
    //            deleteCollisionObject(collision_objects[c]);
    //        }

    collision_objects.clear();
    std::string brain_frame_id="base_link";

    int box_index=0;
    // For each object
    for(uint32_t o = 0; o < req.object_list.objects.size(); ++o)
    {
        //            if(object_list.objects[o].data.type.id==3)
        //            {
        //                std::cout << " DON T HAD TOOL TO THE COLLISION ENVIRONMENT (SKIP): " << o << std::endl;
        //                continue;
        //            }

        moveit_msgs::CollisionObject collision_object;

        collision_object.id = req.object_list.objects[o].collision_name; // equal to object reference frame name
        collision_object.header = req.object_list.objects[o].data.actionable_parts_data[0].part.pose.header;
        //collision_object.header.frame_id="/" + getCollisionModel()->getWorldFrameId();
        collision_object.header.frame_id=brain_frame_id;
        collision_object.operation = moveit_msgs::CollisionObject::ADD;

        // For each semantical part
        for(uint32_t r = 0; r < req.object_list.objects[o].data.actionable_parts_data.size(); ++r)
        {
            //add the box into the collision space
            shape_msgs::SolidPrimitive box_object;
            box_object.type=shape_msgs::SolidPrimitive::BOX;
            box_object.dimensions.resize(3);
            box_object.dimensions[0] = 2*(req.object_list.objects[o].data.actionable_parts_data[r].part.bounding_box.x - collision_delta);
            box_object.dimensions[1] = 2*(req.object_list.objects[o].data.actionable_parts_data[r].part.bounding_box.y - collision_delta);
            box_object.dimensions[2] = 2*(req.object_list.objects[o].data.actionable_parts_data[r].part.bounding_box.z - collision_delta);
            collision_object.primitives.push_back(box_object);
            geometry_msgs::PoseStamped pose_out;
            tf::TransformListener listener;
            try
            {
                //ROS_INFO("Wait for transform from %s to %s", object_list.objects[o].data.actionable_parts_data[r].part.pose.header.frame_id.c_str(), brain_frame_id);
                listener.waitForTransform(brain_frame_id,req.object_list.objects[o].data.actionable_parts_data[r].part.pose.header.frame_id, ros::Time(0), ros::Duration(5.0));
                listener.transformPose(brain_frame_id,req.object_list.objects[o].data.actionable_parts_data[r].part.pose,pose_out);
            }
            catch (tf::TransformException & ex)
            {
                ROS_INFO("Transform not acquired.");
            }
            collision_object.primitive_poses.push_back(pose_out.pose);
        }

        std::string object_name=collision_object.id;
        //collision_objects.insert (std::pair<std::string, moveit_msgs::CollisionObject>(object_name, collision_object) );
        collision_objects.push_back(collision_object);

        // Now, let's add the collision object into the world
        ROS_INFO("Add an object into the world");

        planning_scene_interface.addCollisionObjects(collision_objects);


    }

    ROS_INFO("Done.");

    return true;
}


void GraspPlanningAction::removeObjectsFromCollisionEnvironment(std::vector<std::string> & ids)
{

    planning_scene_interface.removeCollisionObjects(ids);
//    for(uint32_t id = 0; id < ids.size(); ++id)
//    {
//        std::map<std::string, moveit_msgs::CollisionObject>::iterator it;
//        it=collision_objects.find(ids[id]);
//        if(it!=collision_objects.end())
//            collision_objects.erase(it);                   // erasing by iterator
//    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_planning_server");

    GraspPlanningAction grasp_planning_server(ros::this_node::getName());
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate rate(20.0);
    while (ros::ok())
    {
        rate.sleep();
    }

    spinner.stop();


    return 0;
}

