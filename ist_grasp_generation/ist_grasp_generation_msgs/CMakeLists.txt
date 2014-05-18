cmake_minimum_required(VERSION 2.8.3)
project(ist_grasp_generation_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
	ist_msgs
	actionlib
	actionlib_msgs
        trajectory_msgs
        tabletop_object_segmentation_online
	perception_msgs
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   ObjectInference.srv
   NewExperiment.srv
   MoveRobot.srv
   MoveGripper.srv
   MotionPlan.srv
   InverseKinematicsBatch.srv
   InverseKinematics.srv
   GraspingPointPrediction.srv
   GetGeneratedGrasps.srv
   GetEvaluatedGrasps.srv
   CheckGrasping.srv
   AttachObject.srv
   AddObjectCollision.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  GetGeneratedGrasps.action
  GenerateTrajectories.action
  GenerateGrasps.action
  Grasping.action
)

generate_messages(DEPENDENCIES 
	ist_msgs
	actionlib
	actionlib_msgs
        trajectory_msgs
        tabletop_object_segmentation_online
        perception_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES perception_msgs
  CATKIN_DEPENDS ist_msgs actionlib actionlib_msgs trajectory_msgs tabletop_object_segmentation_online
#  DEPENDS system_lib
)


