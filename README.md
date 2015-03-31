# install dependencies
rosdep install ist_msgs grasping_pipeline

sudo apt-get install libsvm-dev

# Install ist messages

catkin_make --pkg ist_msgs

# Install grasp generation messages

catkin_make --pkg ist_grasp_generation_msgs

# Compile package

catkin_make --pkg grasping
