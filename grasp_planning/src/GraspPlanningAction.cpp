#include <grasp_planning/GraspPlanningAction.h>

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

