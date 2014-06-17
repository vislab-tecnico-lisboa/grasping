#! /usr/bin/env python

import roslib; roslib.load_manifest('ist_generate_grasps')
import rospy
import actionlib

from ist_grasp_generation_msgs.srv import *
from ist_grasp_generation_msgs.msg import *
from ist_msgs.msg import *
from std_srvs.srv import *

use_plinio=True

class GenerateGraspsAction(object):
  # create messages that are used to publish feedback/result
  _feedback = ist_grasp_generation_msgs.msg.GenerateGraspsFeedback()
  _result   = ist_grasp_generation_msgs.msg.GenerateGraspsResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, ist_grasp_generation_msgs.msg.GenerateGraspsAction, execute_cb=self.execute_cb)
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

    self.execution_steps(goal)

    if self._result.status==1:
        rospy.loginfo('%s: Succeeded' % self._action_name)
    else:
        rospy.loginfo('%s: Planner failed for all grasps' % self._action_name)
    self._as.set_succeeded(self._result)

  def execution_steps(self,goal):

    #goal.object_list.objects[goal.object_to_grasp_id]
    
    # 1. Generate grasp candidates

    # publish the feedback
    self._feedback.state="Generating grasp candidates..." 
    self._feedback.progress=0.0
    self._as.publish_feedback(self._feedback)
    
    # Action call
    grasp_candidates_resp=self.generate_grasp_candidates(goal.object_list.objects[goal.object_to_grasp_id])
    if grasp_candidates_resp==False:
        self._result.status=-1
        #self._as.set_aborted(self._result,text="No grasp candidates generated.")
        self._as.set_aborted(self._result)
        return

    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return
    
    # 2. Filter grasp candidates

    # publish the feedback
    self._feedback.state="Filtering grasp candidates..." 
    self._feedback.progress=0.50
    self._as.publish_feedback(self._feedback)
   
		
    # Service call
    filter_grasp_candidates_resp=self.filter_grasp_candidates(grasp_candidates_resp.grip_list,goal.object_list.objects[goal.object_to_grasp_id])
    if filter_grasp_candidates_resp==False:
        self._result.status=-1
        return 
    grip_list=filter_grasp_candidates_resp  
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
    
    # 3. Grasp evaluation
    
    # publish the feedback
    self._feedback.state="Evaluate grasps..." 
    self._feedback.progress=0.75
    self._as.publish_feedback(self._feedback)
    
    # check that preempt has not been requested by the client
    if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        self._as.set_preempted()
        return False
    
    # Service call
    grip_list=self.grasp_evaluation(goal.object_list.objects[goal.object_to_grasp_id], grip_list)
    
    if grip_list==False:
        self._result.status=-1
        return       
    
    
    ##############
    # Sort grips #
    ##############
    
    grip_list=self.getSortedGripList(grip_list,goal.object_list.objects[goal.object_to_grasp_id])

    # 4. Add objects to collision environment
    
    # publish the feedback
    self._feedback.state="Add objects to collision environment..." 
    self._feedback.progress=0.50
    self._as.publish_feedback(self._feedback)
   
    self.collision_environment(goal.object_list, goal.table, 1)
    
    # 5. Grasp motion planning (kinematics and collisions)
    
    # publish the feedback
    self._feedback.state="Grasps motion planning..." 
    self._feedback.progress=0.75
    self._as.publish_feedback(self._feedback)
    #print 'size before:'+str(len(goal.collision_objects.objects))
    #collision_objects=list(goal.collision_objects.objects)
    #collision_objects.pop(goal.object_to_grasp_id)
    #print 'size after:'+str(len(goal.collision_objects.objects))

    collision_objects=[] #TIRAR ISTO

    print ' GOING TO GENERATE GRASPS FOR OBJECT ', goal.object_to_grasp_id
    # Action call
    plan_grasp_trajectories_resp=self.plan_grasp_trajectories(grip_list,goal.object_list.objects[goal.object_to_grasp_id],collision_objects)
    
    if plan_grasp_trajectories_resp.status==-1: #or len(plan_grasp_trajectories_resp.grip_list.grip_states) == 0: # or len(plan_grasp_trajectories_resp.grip_list.grip_states)==0:
        self._result.status=-1
	print ' NOOOOOOO', str(plan_grasp_trajectories_resp.status), ' ', str(len(plan_grasp_trajectories_resp.grip_list.grip_states))
        return
    else:
      print ' YEEEEES'
      self._result.status=1

    self._result.grip_list=plan_grasp_trajectories_resp.grip_list

    # publish the feedback
    self._feedback.state="Done." 
    self._feedback.progress=1.0
    self._as.publish_feedback(self._feedback)


  def getSortedGripList(self, grip_list, obj):#,object_id):
    probabilities=[]
   
    #print ' NUMB GRIPS: ' + len((grip_list.grip_states))
    for grip_index in range(0,len(grip_list.grip_states)):
    #print ' NUMB PARTS: ' + str(len(object.data.actionable_parts_data))
        grip=grip_list.grip_states[grip_index]

        # get part
        part_id=grip.grip_pose.part.id # part associated with given grip
        for part_index in range(0,len(obj.data.actionable_parts_data)):
            if(obj.data.actionable_parts_data[part_index].part.id == part_id):
                part=obj.data.actionable_parts_data[part_index] # part associated with given grip
                break

        # get task
        #for task_index in range(0,len(part.tasks)):
        #    if part.tasks[task_index].id == obj.task.id:
        #        task=par.tasks[task_index] # task for the given object part
        #        break
                
                               
#         print 'part id:' +str(part_id)+ ' task id: ' + str(task.id) + ' task likelihood: ' + str(task.likelihood) + ' grip likeli: ' + str(grip.success_probability)
        if use_plinio:
            probability=grip.success_probability#*task.likelihood
        else:
            probability=task.likelihood
    
        probabilities.append(probability)

    sorted_indexes = [i[0] for i in sorted(enumerate(probabilities), key=lambda x:x[1], reverse=True)]
    sorted_grip_list=ist_msgs.msg.GripList()
    for sorted_indexes_index in range(0,len(sorted_indexes)):
#         print 'prob:' + str(probabilities[sorted_indexes[sorted_indexes_index]]) + ' '
        sorted_grip_list.grip_states.append(grip_list.grip_states[sorted_indexes[sorted_indexes_index]])
       
    print 'grip list size: ' , int(len(sorted_grip_list.grip_states))     
    print 'sorted grip list size: ' , int(len(sorted_grip_list.grip_states))     
    return sorted_grip_list
        #print '\n'
    
        #return {'grip_ids':grip_indexes, 'part_ids':part_indexes,'task_ids':task_indexes,'probabilities':probabilities, 'indexes':sorted_indexes}

  def generate_grasp_candidates(self,object_to_grasp):
    print "Waiting for generate grasp candidates action server to start...";
    # Creates the SimpleActionClient, passing the type of the action
    # (GetGeneratedGraspsAction) to the constructor.
    client = actionlib.SimpleActionClient('ist_generate_grasp_candidates', ist_grasp_generation_msgs.msg.GetGeneratedGraspsAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
     # Creates a goal to send to the action server.
    goal = ist_grasp_generation_msgs.msg.GetGeneratedGraspsGoal(object=object_to_grasp)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    if len(client.get_result().grip_list.grip_states) == 0:
        print 'Generated 0 grasps'
        return False
    #print len(client.get_result().grip_list.grip_states)
    
    return client.get_result()
    
  def filter_grasp_candidates(self,grip_list,object_to_grasp):
    print 'Waiting for grasps manager service...'
    rospy.wait_for_service('ist_grasps_manager')
    try:
        grasps_manager_srv = rospy.ServiceProxy('ist_grasps_manager',ist_grasp_generation_msgs.srv.GetEvaluatedGrasps)
        myReq = ist_grasp_generation_msgs.srv.GetEvaluatedGraspsRequest()
        myReq.object=object_to_grasp
        myReq.grip_list=grip_list
        resp=grasps_manager_srv(myReq)
    except rospy.ServiceException, e:
        print "Grasps manager service call failed: %s"%e
        return False
    return resp.grip_list


  def collision_environment(self,object_list, table, action):
    print 'Waiting for collision environment service...'
    rospy.wait_for_service('add_objects_collision')
    try:
        collision_srv = rospy.ServiceProxy('add_objects_collision' , ist_grasp_generation_msgs.srv.AddObjectCollision)
        myReq = ist_grasp_generation_msgs.srv.AddObjectCollisionRequest()
        myReq.table=table
        myReq.object_list=object_list
        #myReq.action=action
        collision_srv(myReq)
    except rospy.ServiceException, e:
        print "Collision environment service call failed: %s"%e
        return False
    return True
        

    
  def plan_grasp_trajectories(self,grip_list,object_to_grasp, collision_objects):
    print "Waiting for generate trajectories action server to start...\n";
    # Creates the SimpleActionClient, passing the type of the action
    # (GetGeneratedGraspsAction) to the constructor.
    client = actionlib.SimpleActionClient('grasp_planning_server', ist_grasp_generation_msgs.msg.GenerateTrajectoriesAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
     # Creates a goal to send to the action server.
    #print object_to_grasp 
    #print 'ola'
    goal = ist_grasp_generation_msgs.msg.GenerateTrajectoriesGoal(grip_list=grip_list,object_to_grasp=object_to_grasp,collision_objects=collision_objects)

    # Sends the goal to the action server.
    client.send_goal(goal, self.plan_grasp_trajectories_done_cb, self.plan_grasp_trajectories_active_cb, self.plan_grasp_trajectories_feedback_cb)

    # Waits for the server to finish performing the action.
    client.wait_for_result()

    #print len(client.get_result().grip_list.grip_states)

    return client.get_result()
         
  # Called once when the goal completes
  def plan_grasp_trajectories_done_cb(self,state,result):
    print 'Done.\n'
    
      # Called once when the goal becomes active
  def plan_grasp_trajectories_active_cb(self):
    print 'Generating grasps...\n'

  # Called every time feedback is received for the goal
  def plan_grasp_trajectories_feedback_cb(self,feedback):
    print 'Got Feedback:\n' , feedback;
        
  def grasp_evaluation(self,object, grip_list):
    print 'waiting for grasping point prediction service...\n'
    rospy.wait_for_service('grasping_point_prediction')
    try:
        grasping_point_prediction_srv = rospy.ServiceProxy('grasping_point_prediction' , GraspingPointPrediction)
        myReq = GraspingPointPredictionRequest()
        myReq.object = object
        myReq.grips =  grip_list
        response = grasping_point_prediction_srv(myReq)
        grip_list = response.evaluated_grips
    except rospy.ServiceException, e:
        print "Object inference Service call failed: %s"%e
        return False

    return grip_list


if __name__ == '__main__':
  rospy.init_node('generate_grasps_server')
  GenerateGraspsAction(rospy.get_name())
  rospy.spin()
