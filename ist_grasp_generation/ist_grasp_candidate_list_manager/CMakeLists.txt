cmake_minimum_required(VERSION 2.8.3)
project(ist_grasp_candidate_list_manager)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ist_grasp_generation_msgs
  visualization_msgs
  cv_bridge
  std_msgs
  image_transport
  eigen_conversions
  ist_generate_grasp_candidates
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)


include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


add_library(reachability src/ReachabilityMap.cpp)
add_executable(grasp_candidate_list_manager src/grasp_candidate_list_manager/grasp_candidate_list_manager.cpp)
target_link_libraries(grasp_candidate_list_manager generate_grip_states reachability ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${Eigen_LIBRARIES} ${catkin_LIBRARIES})
