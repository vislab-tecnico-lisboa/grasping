#include <grasp_planning/GraspPlanningAction.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_planning_server");

    GraspPlanningAction grasp_planning_server(ros::this_node::getName());
    ros::spin();

    return 0;
}

