#! /usr/bin/env python

import roslib; roslib.load_manifest('ist_generate_grasps')
import rospy
import numpy
import actionlib
import tf

from ist_grasp_generation_msgs.srv import *
from ist_grasp_generation_msgs.msg import *

DEG_TO_RAD=3.14159265359/180

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

  _perturb_angle_step = 0.0
  _perturbs_number = 0
  
  def __init__(self, name):
    self._perturb_angle_step = rospy.get_param("~perturb_angle_step")
    print 'perturb angle step: ', self._perturb_angle_step
    self._perturbs_number = rospy.get_param("~perturbs_number")
    print 'perturbs number: ', self._perturbs_number

#     self._perturb_angle_step = perturb_angle_step
#     self._perturbs_number = perturbs_number
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, ist_grasp_generation_msgs.msg.GenerateTrajectoriesAction, execute_cb=self.execute_cb)
    self._as.start()
    
  def execute_cb(self, goal):

    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
        
    ####################################
    # start executing the action steps #
    ####################################

    grasp_list=self.execution_steps(goal)
#     if grasp_list:
    self._result.grip_list = grasp_list
    rospy.loginfo('%s: Succeeded' % self._action_name)
    self._as.set_succeeded(self._result)
    
    
  def execution_steps(self,goal):

    # 1. Grasp motion planning (kinematics and collisions)
    
    # publish the feedback
    self._feedback.state="Grasps motion planning..." 
    self._feedback.progress=0.0
    self._as.publish_feedback(self._feedback)

    # Service call
    grip_list=self.generate_trajectories(goal.grip_list,goal.object.state.graspable_object.potential_models[0].pose.pose)
    if grip_list==False:
        self._as.set_aborted(self._result)
        return False
    print

    return grip_list
    
  def generate_trajectories(self,grip_list,object_pose):
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
            return False
        
        # Planner service
        plan_trajectory_resp=self.plan_trajectory(grip_list.grip_states[graspsCounter], object_world_hmatrix)

        if plan_trajectory_resp==False:
            continue

        # Update grip (perturb)
        grip_list.grip_states[graspsCounter]=plan_trajectory_resp

        # publish the feedback
        if(grip_list.grip_states[graspsCounter].hand_state.grasp_pose.trajectory_status==1):
          good_trajectory=True
          self._feedback.state="Possible grasp." 
          print 'POSSIBLE GRASP'
        else:
          self._feedback.state="Impossible grasp."
          print 'IMPOSSIBLE GRASP'

        
        self._feedback.progress=float(graspsCounter+1)/float(len(grip_list.grip_states))
        self._as.publish_feedback(self._feedback)

    if(good_trajectory):
        return grip_list
    else: 
        #return False
        return grip_list
            

  def plan_trajectory(self,grip_state,object_world_hmatrix):
    hand_object_pose = grip_state.hand_state.grasp_pose.pose.pose
    rospy.wait_for_service('motion_planning')
    response=MoveArmResponse()
    hand_object_quaternion_array=[hand_object_pose.orientation.x,hand_object_pose.orientation.y,hand_object_pose.orientation.z,hand_object_pose.orientation.w]
    hand_object_translation_array=[hand_object_pose.position.x,hand_object_pose.position.y,hand_object_pose.position.z]
    hand_object_hmatrix=numpy.mat(tf.transformations.quaternion_matrix(hand_object_quaternion_array)+tf.transformations.translation_matrix(hand_object_translation_array)-tf.transformations.identity_matrix())
    hand_world_hmatrix=object_world_hmatrix*hand_object_hmatrix
    # Canonical grasp
    try:      
        plan_trajectory_srv = rospy.ServiceProxy('motion_planning',MoveArm)
        myReq=MoveArmRequest()
        myReq.mode=1
        myReq.pose.pose=numpyToMsg(hand_world_hmatrix)
        myReq.pose.header.frame_id="/base_link"
        myReq.pose.header.stamp=rospy.Time.now()    
        response=plan_trajectory_srv(myReq)
        if response.success==True:
            grip_state.hand_state.grasp_pose.trajectory_status=1;
            grip_state.hand_state.grasp_pose.trajectory=response.trajectory
            return grip_state
    except rospy.ServiceException, e:
        print "Motion planning service call failed: %s"%e
        return False
    
    # Grasp perturbations
    for p in range(1,self._perturbs_number):
        i=-1
        while i<=1:
            grip_perturb_pose=geometry_msgs.msg.PoseStamped()
            grasp_perturb_pose=geometry_msgs.msg.PoseStamped()
            grasp_perturb_pose_object_frame=geometry_msgs.msg.PoseStamped()
            perturbation_angle=i*p*self._perturb_angle_step*DEG_TO_RAD;
            perturb_rot_axis=numpy.array([1,0,0]);
            perturb_hmatrix=self.rotation_matrix(perturb_rot_axis,perturbation_angle)
            
            hand_world_perturb_hmatrix=hand_world_hmatrix*perturb_hmatrix

            grasp_perturb_pose=geometry_msgs.msg.PoseStamped()            
            grasp_perturb_pose.pose=numpyToMsg(hand_world_perturb_hmatrix);   
            grasp_perturb_pose.header.frame_id='/base_link'
            grasp_perturb_pose.header.stamp=rospy.Time.now()     
            

            try:      
                plan_trajectory_srv = rospy.ServiceProxy('motion_planning',MoveArm)
                myReq=MoveArmRequest()
                myReq.mode=1
                myReq.pose=grasp_perturb_pose
                response=plan_trajectory_srv(myReq)
                if response.success==True:
                    grip_state.hand_state.grasp_pose.trajectory_status=1;
                    grip_state.grip_pose.perturb=True;
                    grip_state.grip_pose.perturb_pose.header=grip_state.hand_state.grasp_pose.pose.header
                    grip_state.grip_pose.perturb_pose.pose=numpyToMsg(hand_object_hmatrix*perturb_hmatrix)
                    grip_state.hand_state.grasp_pose.pose.pose=numpyToMsg(hand_world_perturb_hmatrix)
                    grip_state.hand_state.grasp_pose.trajectory=response.trajectory
                    return grip_state
            except rospy.ServiceException, e:
                print "Motion planning service call failed: %s"%e
                return False
          
            i+=2
    
    grip_state.hand_state.grasp_pose.trajectory_status=-1;

    return grip_state

  def rotation_matrix(self,axis,theta):
    axis = axis/numpy.sqrt(numpy.dot(axis,axis))
    a = numpy.cos(theta/2)
    b,c,d = -axis*numpy.sin(theta/2)
    return numpy.array([[a*a+b*b-c*c-d*d, 2*(b*c-a*d)    , 2*(b*d+a*c),      0.0],
                        [2*(b*c+a*d)    , a*a+c*c-b*b-d*d, 2*(c*d-a*b),      0.0],
                        [2*(b*d-a*c)    , 2*(c*d+a*b)    , a*a+d*d-b*b-c*c, 0.0],
                        [0.0, 0.0, 0.0, 1.0]])
    
if __name__ == '__main__':
    rospy.init_node('generate_trajectories_server')

    GenerateTrajectoriesAction(rospy.get_name())
    rospy.spin()
