# Point-cloud & part-based grasping
ROS package that selects where to grasp an object using 3D point information and bounding boxes
## Installation
### Dependencies
+ rosdep install ist_msgs grasping_pipeline
+ sudo apt-get install libsvm-dev

### ROS ist messages
+ catkin_make --pkg ist_msgs

### ROS grasp generation messages
+ catkin_make --pkg ist_grasp_generation_msgs

### Package compilation
+ catkin_make --pkg grasping
