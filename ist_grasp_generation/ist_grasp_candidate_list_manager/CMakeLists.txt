cmake_minimum_required(VERSION 2.8.3)
project(ist_grasp_candidate_list_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  cv_bridge
  std_msgs
  eigen_conversions
  ist_grasp_generation_msgs
  ist_generate_grasp_candidates
  image_transport
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES reachability
  CATKIN_DEPENDS  ist_grasp_generation_msgs visualization_msgs cv_bridge std_msgs eigen_conversions    ist_grasp_generation_msgs ist_generate_grasp_candidates
#  DEPENDS system_lib
)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_library(reachability src/ReachabilityMap.cpp)
add_executable(grasp_candidate_list_manager src/grasp_candidate_list_manager/grasp_candidate_list_manager.cpp)
target_link_libraries(grasp_candidate_list_manager generate_grip_states reachability ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${catkin_LIBRARIES})
