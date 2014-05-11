cmake_minimum_required(VERSION 2.8.3)
project(ist_generate_grasp_candidates)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  ist_grasp_generation_msgs
  roscpp
  eigen_conversions
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES generate_grip_states
  CATKIN_DEPENDS  ist_grasp_generation_msgs roscpp eigen_conversions
#  DEPENDS system_lib
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_library(generate_grip_states 	src/object_type/object_type.cpp
					src/shape_type/shape_type.cpp
					src/size_type/size_type.cpp
											
					src/object_part/object_part.cpp
					src/canonical_object_part/canonical_object_part.cpp
											src/object_part/object_part.cpp
											
											src/canonical_grip/canonical_grip.cpp 
					  						src/grip/grip.cpp 
					  						
					  						src/joint_state/joint_state.cpp
					  						src/hand_posture/hand_posture.cpp
					  						
					  						src/reach/reach.cpp
					  						src/grasp_type/grasp_type.cpp
											src/canonical_grasp/canonical_grasp.cpp 
											src/grasp/grasp.cpp
											src/reach_type/reach_type.cpp
											src/generate_grasps/generate_grasps.cpp
											src/normalizing_functions.cpp
											src/grasp_perturbation/grasp_perturbation.cpp)

#rosbuild_add_executable(ist_generate_grasps src/ros_generate_grasps/ros_generate_grasps.cpp)
add_executable(ist_generate_grasp_candidates_action_server src/ros_generate_grasps_action_server/ros_generate_grasps_action_server.cpp)

#target_link_libraries(ist_generate_grasps generate_grip_states)

#rosbuild_add_library(reachability src/ReachabilityMap.cpp)

target_link_libraries(ist_generate_grasp_candidates_action_server generate_grip_states -lboost_serialization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES} ${Boost_LIBRARIES})



