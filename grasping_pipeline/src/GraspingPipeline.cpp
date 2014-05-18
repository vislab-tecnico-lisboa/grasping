#include <grasping_pipeline/GraspingPipeline.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasping_pipeline_server");

    GraspingPipelineAction grasping_pipeline(ros::this_node::getName());
    ros::spin();

    return 0;
}
