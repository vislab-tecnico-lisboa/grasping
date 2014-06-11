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
#include <std_srvs/Empty.h>

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

    GraspPlanningAction(std::string name);

    ~GraspPlanningAction(void)
    {}

    void goalCB();

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
    Eigen::Affine3d transform_end_effector_to_palm;

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

    ros::ServiceClient close_gripper_client;
};

