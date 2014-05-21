#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <ist_grasp_generation_msgs/GenerateTrajectoriesAction.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>


void print_grip(int grip_id)
{

    switch (grip_id)
    {
    case ist_msgs::GripDirection::BACK_THUMB_DOWN:
        std::cout << "BACK THUMB DOWN" << std::endl;
        break;
    case ist_msgs::GripDirection::BACK_THUMB_LEFT:
        std::cout << "BACK THUMB LEFT" << std::endl;
        break;
    case ist_msgs::GripDirection::BACK_THUMB_RIGHT:
        std::cout << "BACK THUMB RIGHT" << std::endl;
        break;
    case ist_msgs::GripDirection::BACK_THUMB_UP:
        std::cout << "BACK THUMB UP" << std::endl;
        break;

    case ist_msgs::GripDirection::BOTTOM_THUMB_BACK:
        std::cout << "BOTTOM THUMB BACK" << std::endl;
        break;
    case ist_msgs::GripDirection::BOTTOM_THUMB_FRONT:
        std::cout << "BOTTOM THUMB FRONT" << std::endl;
        break;
    case ist_msgs::GripDirection::BOTTOM_THUMB_LEFT:
        std::cout << "BOTTOM THUMB LEFT" << std::endl;
        break;
    case ist_msgs::GripDirection::BOTTOM_THUMB_RIGHT:
        std::cout << "BOTTOM THUMB RIGHT" << std::endl;
        break;

    case ist_msgs::GripDirection::FRONT_THUMB_DOWN:
        std::cout << "FRONT THUMB DOWN" << std::endl;
        break;
    case ist_msgs::GripDirection::FRONT_THUMB_LEFT:
        std::cout << "FRONT THUMB LEFT" << std::endl;
        break;
    case ist_msgs::GripDirection::FRONT_THUMB_RIGHT:
        std::cout << "FRONT THUMB RIGHT" << std::endl;
        break;
    case ist_msgs::GripDirection::FRONT_THUMB_UP:
        std::cout << "FRONT THUMB UP" << std::endl;
        break;

    case ist_msgs::GripDirection::LEFT_THUMB_BACK:
        std::cout << "LEFT THUMB BACK" << std::endl;
        break;
    case ist_msgs::GripDirection::LEFT_THUMB_DOWN:
        std::cout << "LEFT THUMB DOWN" << std::endl;
        break;
    case ist_msgs::GripDirection::LEFT_THUMB_FRONT:
        std::cout << "LEFT THUMB FRONT" << std::endl;
        break;
    case ist_msgs::GripDirection::LEFT_THUMB_UP:
        std::cout << "LEFT THUMB UP" << std::endl;
        break;

    case ist_msgs::GripDirection::RIGHT_THUMB_BACK:
        std::cout << "RIGHT THUMB BACK" << std::endl;
        break;
    case ist_msgs::GripDirection::RIGHT_THUMB_DOWN:
        std::cout << "RIGHT THUMB DOWN" << std::endl;
        break;
    case ist_msgs::GripDirection::RIGHT_THUMB_FRONT:
        std::cout << "RIGHT THUMB FRONT" << std::endl;
        break;
    case ist_msgs::GripDirection::RIGHT_THUMB_UP:
        std::cout << "RIGHT THUMB UP" << std::endl;
        break;

    case ist_msgs::GripDirection::TOP_THUMB_BACK:
        std::cout << "TOP THUMB BACK" << std::endl;
        break;
    case ist_msgs::GripDirection::TOP_THUMB_FRONT:
        std::cout << "TOP THUMB FRONT" << std::endl;
        break;
    case ist_msgs::GripDirection::TOP_THUMB_LEFT:
        std::cout << "TOP THUMB LEFT" << std::endl;
        break;
    case ist_msgs::GripDirection::TOP_THUMB_RIGHT:
        std::cout << "TOP THUMB RIGHT" << std::endl;
        break;
    }
}

class GraspPlanningAction
{
public:

    GraspPlanningAction(std::string name) : as_(nh_, name, false), action_name_(name), group("arm")
    {
        //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&GraspPlanningAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&GraspPlanningAction::preemptCB, this));

        //subscribe to the data topic of interest
        as_.start();

        group.setPoseReferenceFrame("base_link");
        group.setEndEffectorLink("end_effector");

        // (Optional) Create a publisher for visualizing plans in Rviz.
        display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

    }

    ~GraspPlanningAction(void)
    {}

    void goalCB()
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

        ist_msgs::GripState chosen_grip;

        for(int i=0;i< grip_list.grip_states.size();++i)
        {
            //ROS_INFO("trying grip:",grip_list.grip_states[i].);
            Eigen::Affine3d transform_grip_eigen;

            tf::poseMsgToEigen(grip_list.grip_states[i].hand_state.grasp_pose.pose.pose,transform_grip_eigen);

            Eigen::Affine3d final_transform = transform_object_eigen*transform_grip_eigen;

            group.setPoseTarget(final_transform);

            bool good_plan=group.plan(my_plan);

            if(good_plan)
            {
                group.asyncExecute(my_plan);
                chosen_grip=grip_list.grip_states[i];
                print_grip((int)chosen_grip.grip_pose.direction.id);
                break;
            }
        }


        group.setPoseTarget(transform_object_eigen);
        //ist_msgs::Object[] collision_objects
        // plan the motion and then move the group to the sampled target
        bool success=group.asyncMove();

        if(success)
            as_.setSucceeded(result_);
        else
            as_.setAborted();
        ROS_INFO("INSIDE ANALYSIS3");

    }

    void preemptCB()
    {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
    }

    void analysisCB(const ist_msgs::GripList& msg)
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

protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<ist_grasp_generation_msgs::GenerateTrajectoriesAction> as_;
    std::string action_name_;
    int data_count_;
    ist_grasp_generation_msgs::GenerateTrajectoriesGoalPtr  goal_;
    float sum_, sum_sq_;
    ist_grasp_generation_msgs::GenerateTrajectoriesFeedback feedback_;
    ist_grasp_generation_msgs::GenerateTrajectoriesResult result_;
    ros::Subscriber sub_;

    moveit::planning_interface::MoveGroup group;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    ros::Publisher display_publisher;
    moveit_msgs::DisplayTrajectory display_trajectory;
};

