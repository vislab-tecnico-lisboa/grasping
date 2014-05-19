cmake_minimum_required(VERSION 2.8.3)
project(ist_grasping_point_prediction)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  ist_grasp_generation_msgs
  cv_bridge
  std_msgs
  image_transport
  eigen_conversions
)

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grasping_point_prediction
  CATKIN_DEPENDS ist_grasp_generation_msgs cv_bridge
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Add link directories
link_directories(
  ${Boost_LIBRARY_DIRS}
  ${Eigen_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)




#common commands for building c++ executables and libraries
add_library(grasping_point_prediction src/grasping_point_prediction.cpp)
add_executable(grasping_point_prediction_ros src/grasping_point_prediction_ros.cpp)
target_link_libraries(grasping_point_prediction_ros grasping_point_prediction svm  ${vtk_LIBRARIES}  libvtkCommon.so libvtkFiltering.so libvtkRendering.so ${PCL_LIBRARIES} ${catkin_LIBRARIES} )
