#! /usr/bin/env python

import rospy
import numpy
import actionlib
import tf

from ist_grasp_generation_msgs.srv import *
from ist_grasp_generation_msgs.msg import *


def numpyToMsg (hmatrix):
    pose=geometry_msgs.msg.Pose() 
    quaternion_array=tf.transformations.quaternion_from_matrix(hmatrix)
    pose.orientation=geometry_msgs.msg.Quaternion()
    pose.orientation.x=quaternion_array[0]
    pose.orientation.y=quaternion_array[1]
    pose.orientation.z=quaternion_array[2]
    pose.orientation.w=quaternion_array[3]
    pose.position=geometry_msgs.msg.Point()
    position_array=tf.transformations.translation_from_matrix(hmatrix)
    pose.position.x=position_array[0]
    pose.position.y=position_array[1]
    pose.position.z=position_array[2]
    return pose


class GenerateTrajectoriesAction(object):
  # create messages that are used to publish feedback/result
  _feedback = ist_grasp_generation_msgs.msg.GenerateTrajectoriesFeedback()
  _result   = ist_grasp_generation_msgs.msg.GenerateTrajectoriesResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, ist_grasp_generation_msgs.msg.GenerateTrajectoriesAction, execute_cb=self.execute_cb)
    self._as.start()


    
  def execute_cb(self, goal):

    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._result.status=-1
        self._as.set_preempted()
        return
        
    ####################################
    # start executing the action steps #
    ####################################

    self.execution_steps(goal)
    if self._result.status==1:
        rospy.loginfo('%s: Succeeded' % self._action_name)
    else:
        rospy.loginfo('%s: Planner failed for all grasps' % self._action_name)
    self._as.set_succeeded(self._result)

  def execution_steps(self,goal):

    # 1. Grasp motion planning (kinematics and collisions)
    
    # publish the feedback
    self._feedback.state="Grasps motion planning..." 
    self._feedback.progress=0.0
    self._as.publish_feedback(self._feedback)

    # Service call
    self.generate_trajectories(goal.grip_list,goal.object_to_grasp.state.graspable_object.potential_models[0].pose.pose, goal.object_to_grasp, goal.collision_objects)

    
  def generate_trajectories(self,grip_list,object_pose, object_to_grasp, collision_objects):
    print 'Generating trajectories...'
    object_world_quaternion_array=[object_pose.orientation.x,object_pose.orientation.y,object_pose.orientation.z,object_pose.orientation.w]
    object_world_translation_array=[object_pose.position.x,object_pose.position.y,object_pose.position.z]
    object_world_hmatrix=numpy.mat(tf.transformations.quaternion_matrix(object_world_quaternion_array)+tf.transformations.translation_matrix(object_world_translation_array)-tf.transformations.identity_matrix())
    num=0
    good_trajectory=False
    # Plan trajectory for each grasp candidate
    progress_before=self._feedback.progress
    for graspsCounter in range(len(grip_list.grip_states)):
        # check that preempt has not been requested by the client
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted(self._result)
	    self._result.status=-1
            return
        
        plan_trajectory_resp=self.plan_trajectory(grip_list.grip_states[graspsCounter],grip_list.grip_states[graspsCounter].hand_state.grasp_pose.pose.pose, object_world_hmatrix,  object_to_grasp,collision_objects)
        if plan_trajectory_resp.hand_state.grasp_pose.trajectory_status==-1:
            continue

        grip_list.grip_states[graspsCounter]=plan_trajectory_resp

        # publish the feedback
        if(plan_trajectory_resp.hand_state.grasp_pose.trajectory_status==1):
          good_trajectory=True
          rospy.loginfo('Possible grasp')
          self._feedback.state="Possible grasp."
          self._result.status=1
          self._result.grip_list=grip_list
          return # SALTA LOGO MAL ENCONTRA UM GRASP BOM
        else:
	  rospy.loginfo('Impossible grasp')
          self._feedback.state="Impossible grasp."
        
        self._feedback.progress=float(graspsCounter+1)/float(len(grip_list.grip_states))
        self._as.publish_feedback(self._feedback)
    
    self._result.grip_list=grip_list
    if(good_trajectory):
        self._result.status=1
    else: 
        self._result.status=-1

            
  def plan_trajectory(self,grip_state,hand_object_pose,object_world_hmatrix, object_to_grasp, collision_objects):
    rospy.wait_for_service('motion_planning')
    response=MotionPlanResponse()
    hand_object_quaternion_array=[hand_object_pose.orientation.x,hand_object_pose.orientation.y,hand_object_pose.orientation.z,hand_object_pose.orientation.w]
    hand_object_translation_array=[hand_object_pose.position.x,hand_object_pose.position.y,hand_object_pose.position.z]
    hand_object_hmatrix=numpy.mat(tf.transformations.quaternion_matrix(hand_object_quaternion_array)+tf.transformations.translation_matrix(hand_object_translation_array)-tf.transformations.identity_matrix())
    hand_world_hmatrix=object_world_hmatrix*hand_object_hmatrix
    
    pose_stamped=geometry_msgs.msg.PoseStamped()
    pose_stamped.header.frame_id='base_link'
    pose_stamped.header.stamp=rospy.Time.now()
    pose_stamped.pose=numpyToMsg(hand_world_hmatrix)

    try:      
            plan_trajectory_srv = rospy.ServiceProxy('motion_planning',MotionPlan)
            myReq = MotionPlanRequest()
            myReq.mode=1

            myReq.pose=pose_stamped
            response=plan_trajectory_srv(myReq)
            if response.success==True:
                grip_state.hand_state.grasp_pose.trajectory_status=1;
                grip_state.hand_state.grasp_pose.trajectory=response.trajectory

            else:
                grip_state.hand_state.grasp_pose.trajectory_status=-1;
    except rospy.ServiceException, e:
            print "Motion planning service call failed: %s"%e
    return grip_state

  def arne_client(self,pose_stamped, object_to_grasp, collision_objects):
    # Creates the SimpleActionClient, passing the type of the action (DetectObjectsAction) to the constructor.
    client = actionlib.SimpleActionClient('/tub/motion_interface/plan_trajectory', motion_interface_msgs.msg.PlanTrajectoryAction)
    print 'Waiting for arne planner'

    # Waits until the action server has started up and started listening for goals.
    client.wait_for_server()


    # Creates a goal to send to the action server.
    goal = motion_interface_msgs.msg.PlanTrajectoryGoal(targetFrame=pose_stamped,  objectToGrasp=object_to_grasp, objectsToAvoid=collision_objects, graspedObject=0)

    # Sends the goal to the action server.
    client.send_goal(goal, self.motion_planning_done_cb, self.motion_planning_active_cb, self.motion_planning_feedback_cb)

    #client.cancel_goal()
    client.get_state()

    # Waits for the server to finish performing the action.
    success=client.wait_for_result()
    if success==False:
      return False

    # Prints out the result of executing the action
    return client.get_result()

  # Called once when the goal completes
  def motion_planning_done_cb(self,state,result):
    print 'Done'

  # Called once when the goal becomes active
  def motion_planning_active_cb(self):
    print 'Generating trajectory...\n'

  # Called every time feedback is received for the goal
  def motion_planning_feedback_cb(self,feedback):
    print 'Got Feedback:\n' , feedback;

if __name__ == '__main__':
    rospy.init_node('generate_trajectories_server')
    GenerateTrajectoriesAction(rospy.get_name())
    rospy.spin()
