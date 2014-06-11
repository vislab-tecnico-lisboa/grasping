#include <grasp_planning/GraspPlanningAction.h>
#include <tf/transform_broadcaster.h>
GraspPlanningAction::GraspPlanningAction(std::string name) : as_(nh_, name, false), action_name_(name), group("arm")
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
        //std::cout << "TESTE:"<<end_effector_palm_transform.getOrigin().getZ() << std::endl;

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
    }
    tf::transformTFToEigen(end_effector_palm_transform, transform_end_effector_to_palm);

    close_gripper_client = nh_.serviceClient<std_srvs::Empty>("close_gripper");


}

void GraspPlanningAction::goalCB()
{
    ist_grasp_generation_msgs::GenerateTrajectoriesGoalConstPtr goal_;
    goal_ = as_.acceptNewGoal();

    ist_msgs::Object object_to_grasp=goal_->object_to_grasp;

    tf::TransformListener listener;
    tf::StampedTransform transform;
    //object_to_grasp.state.graspable_object.potential_models[0].pose.pose;

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


    //ist_msgs::Object object_to_grasp= goal_->object_to_grasp;
    ist_msgs::GripList grip_list=goal_->grip_list;
    moveit::planning_interface::MoveGroup::Plan my_plan;
       group.setPlanningTime(0.5);
    ist_msgs::GripState chosen_grip;
    bool good_plan=false;
    for(int i=0;i< grip_list.grip_states.size();++i)
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

    group.setPoseTarget(transform_object_eigen);
    //ist_msgs::Object[] collision_objects
    // plan the motion and then move the group to the sampled target
    //bool success=group.asyncMove();
    bool success=group.execute(my_plan);

    if(!success)
    {
        ROS_INFO("FAILED MOVING...");
        as_.setAborted();
        return;
    }

    std_srvs::Empty srv;
    if (!close_gripper_client.call(srv))
    {
        as_.setAborted(result_);
        ROS_ERROR("Failed to call service close gripper");
    }
    else
    {
      return;
    }







    // MOVE TO HARDCODED CONTA
    // 0.70456; 0.21208; 0.60488
    // -0.23468; 0.66706; 0.23463; 0.66701
    Eigen::Affine3d transformation;
    transformation.rot

    group.setPoseTarget(transform_object_eigen);
    //ist_msgs::Object[] collision_objects
    // plan the motion and then move the group to the sampled target
    //bool success=group.asyncMove();
    bool success=group.execute(my_plan);

    if(!success)
    {
        ROS_INFO("FAILED MOVING...");
        as_.setAborted();
        return;
    }

    std_srvs::Empty srv;
    if (!close_gripper_client.call(srv))
    {
        as_.setAborted(result_);
        ROS_ERROR("Failed to call service close gripper");
    }
    else
    {
      return;
    }

    as_.setSucceeded(result_);

    /*moveit::core::RobotModel robot_model;

    robot_state::RobotState goal_state(robot_model);
    robot_state::JointStateGroup* goal_group = goal_state.getJointStateGroup("right_arm");
    std::vector<double> joint_values(7, 0.0);
    joint_values[0] = -2.0;
    joint_values[3] = -0.2;
    joint_values[5] = -0.15;
    goal_group->setVariableValues(joint_values);
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_group);
    */

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

