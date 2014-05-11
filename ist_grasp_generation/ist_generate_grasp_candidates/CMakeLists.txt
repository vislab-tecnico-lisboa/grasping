cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(generate_grip_states 	src/object_type/object_type.cpp
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
rosbuild_add_executable(ist_generate_grasp_candidates_action_server src/ros_generate_grasps_action_server/ros_generate_grasps_action_server.cpp)

#target_link_libraries(ist_generate_grasps generate_grip_states)

#rosbuild_add_library(reachability src/ReachabilityMap.cpp)

target_link_libraries(ist_generate_grasp_candidates_action_server generate_grip_states -lboost_serialization)



