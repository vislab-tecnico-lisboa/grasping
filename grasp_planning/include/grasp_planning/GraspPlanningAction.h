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

class GraspPlanningAction
{
public:

    GraspPlanningAction(std::string name) : as_(nh_, name, false), action_name_(name), group("arm")
    {
        ROS_INFO("AQUI1");

        //register the goal and feeback callbacks
        as_.registerGoalCallback(boost::bind(&GraspPlanningAction::goalCB, this));
        as_.registerPreemptCallback(boost::bind(&GraspPlanningAction::preemptCB, this));

        //subscribe to the data topic of interest
        //sub_ = nh_.subscribe("/random_number", 1, &GraspPlanningAction::analysisCB, this);
        as_.start();

        group.setPoseReferenceFrame("base_link");
        group.setEndEffectorLink("end_effector");

        // We will use the :planning_scene_interface:`PlanningSceneInterface`
        // class to deal directly with the world.


        // (Optional) Create a publisher for visualizing plans in Rviz.
        display_publisher = nh_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

        ROS_INFO("AQUI2");


    }

    ~GraspPlanningAction(void)
    {}

    void goalCB()
    {
        ist_grasp_generation_msgs::GenerateTrajectoriesGoalConstPtr goal_;
        goal_ = as_.acceptNewGoal();
        //ist_msgs::Object object_to_grasp=as_.acceptNewGoal()->object_to_grasp;


        //ist_msgs::Object object_to_grasp= goal_->object_to_grasp;
        //ist_msgs::GripList grip_list=goal_->grip_list;

        //ist_msgs::Object[] collision_objects
        ROS_INFO("INSIDE ANALYSIS");
        group.setRandomTarget();
        // plan the motion and then move the group to the sampled target
        while(group.move());
        ros::waitForShutdown();

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
        ROS_INFO("INSIDE ANALYSIS");
        group.setRandomTarget();
        // plan the motion and then move the group to the sampled target
        group.move();
        ros::waitForShutdown();
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

