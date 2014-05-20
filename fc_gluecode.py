#! /usr/bin/env python

import roslib; roslib.load_manifest('fetch_and_carry')
import rospy
import sys, traceback
import actionlib
import random

import first_mm_actions.msg as fmm_msg
import geometry_msgs.msg as geo_msg
import wsg_gripper.msg as wsg_msg
from actionlib_msgs.msg import GoalStatus
from omnirob_bottle_sorter_srv import *
import ist_msgs
import ist_perception_msgs
import geometry_msgs

localized = False  # global status flag
global actionTimeout, preemptTimeout
actionTimeout = rospy.Duration(1020)
preemptTimeout = rospy.Duration(1020)

class ShelfFilterClient:
  def __init__(self):
    actionservername = '/BoxFilter' # defined in ROSTrajectoryPlanner
    self.client = actionlib.SimpleActionClient(actionservername, fmm_msg.Box_FilterAction)

  def filter_shelf(self, name):
    global actionTimeout, preemptTimeout
    if not self.client.wait_for_server(actionTimeout):
      print "Warning: Action Server for BoxFilter not up yet"
    goal = fmm_msg.Box_FilterGoal()
    goal.header.frame_id = name
    #goal.size_x = 0.48
    goal.size_x = 0.35
    goal.size_y = 0.65
    goal.size_z = 0.35
    status = self.client.send_goal_and_wait(goal, execute_timeout = actionTimeout, preempt_timeout = preemptTimeout)
    if status != GoalStatus.SUCCEEDED:
      raise StandardError("Filtering for shelf " + name + " failed due to action server timeout")
    else:
      print "Filtering shelf", name , "succeeded."


  def filter_table(self, name):
    global actionTimeout, preemptTimeout
    if not self.client.wait_for_server(actionTimeout):
      print "Warning: Action Server for BoxFilter not up yet"
    goal = fmm_msg.Box_FilterGoal()
    goal.header.frame_id = name
    #goal.size_x = 0.48
    goal.size_x = 0.75
    goal.size_y = 1.70
    goal.size_z = 0.75
    status = self.client.send_goal_and_wait(goal, execute_timeout = actionTimeout, preempt_timeout = preemptTimeout)
    if status != GoalStatus.SUCCEEDED:
      raise StandardError("Filtering for shelf " + name + " failed due to action server timeout")
    else:
      print "Filtering shelf", name , "succeeded."

class WsgActionClient:
  def __init__(self):
    actionservername = '/WSG_Gripper_Action'
    self.client =  actionlib.SimpleActionClient(actionservername, wsg_msg.GripperCommandAction)

  def open(self):
    global actionTimeout, preemptTimeout
    if not self.client.wait_for_server(actionTimeout):
      print "Warning: Action Server for WSGActionServer not up yet"
    goal = fmm_msg.GripperCommandActionGoal()
    goal.command_code = 33
    goal.width = 100
    goal.speed = 100
    goal.acc = 200
    goal.force = 80

    status = self.client.send_goal_and_wait(goal, execute_timeout = actionTimeout, preempt_timeout = preemptTimeout)
    if status != GoalStatus.SUCCEEDED:
      raise StandardError("WSG Action failed due to action server timeout")
    else:
      print "Opening the WSG gripper succeeded."
    

class BottleActionClient:
  def __init__(self):
    actionservername = '/bottle_action_server' # defined in ROSTrajectoryPlanner
    self.client = actionlib.SimpleActionClient(actionservername, fmm_msg.BottleHandlingAction)

  def place_in_box(self, bucket):
    global actionTimeout, preemptTimeout
    if not self.client.wait_for_server(actionTimeout):
      print "Warning: Action Server for BottleActionServer not up yet"
    goal = fmm_msg.BottleHandlingGoal()
    goal.command =  0
    goal.bucket = bucket

    status = self.client.send_goal_and_wait(goal, execute_timeout = actionTimeout, preempt_timeout = preemptTimeout)
    if status != GoalStatus.SUCCEEDED:
      raise StandardError("Bottle Action failed due to action server timeout")
    else:
      print "Placing of the bottle succeeded."


class RelMotionClient:
  '''Encapsulates the relative motion action client'''
  def __init__(self):
    actionservername = '/relativeMotion' # defined in ROSTrajectoryPlanner
    self.client = actionlib.SimpleActionClient(actionservername, fmm_msg.Omnirob_Relative_MotionAction)

  def relative_motion(self, dx, dy, dtheta):
    if not self.client.wait_for_server(rospy.Duration.from_sec(5.0)):
      print "Warning: Action Server", actionservername, "not up yet"

    goal = fmm_msg.Omnirob_Relative_MotionGoal()
    goal.relative_offset_ego.x = dx
    goal.relative_offset_ego.y = dy
    goal.relative_offset_ego.theta = dtheta
    global actionTimeout, preemptTimeout
    status = self.client.send_goal_and_wait(goal, execute_timeout = actionTimeout, preempt_timeout = preemptTimeout)
#     if status != GoalStatus.SUCCEEDED:
#       raise StandardError("Relative motion did not succeed")
#     else:
#       print "Relative motion succeeded."
#       result = self.client.get_result()
#       print result
#       return result


def localized_callback(feedback):
  '''(Re-)set global flag upon localizer feedback'''
  global localized
  localized = feedback.is_localized
  print "Got localization status: ", localized

def localize():
    global localized
    localized = False
    client = actionlib.SimpleActionClient('StartLocalizer', fmm_msg.RunAISLocalizerAction)
    #API: See http://www.ros.org/doc/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html
    print "Waiting for Action client for StartLocalizer",
    client.wait_for_server()
    print "Ok"

    goal = fmm_msg.RunAISLocalizerActionGoal()
    # Fill in the goal here
    goal.use_scan_matching = True # just as an example
    client.send_goal(goal, feedback_cb = localized_callback)

    wait = 0
    while not localized:
      client.wait_for_result(rospy.Duration.from_sec(5.0))
      wait += 5
      print "Waiting for result since", wait, "seconds"
    else:
      print "Robot is localized, proceeding to fetch & carry"
      return True

def perception_plus_inference_and_grasping(object_category, task_type, pre_place_pose, mode):
    print "Waiting for task dependent grasping action server to start...";
    # Creates the SimpleActionClient, passing the type of the action
    # (GetGeneratedGraspsAction) to the constructor.
    client = actionlib.SimpleActionClient('task_dependent_grasping_server', fmm_msg.TaskDependentGraspingAction)
    # Waits until the action server has started up and started
    # listening for goals.
    client.wait_for_server()
    # Creates a goal to send to the action server.
    
    # Region bounding the plannar surface and the objects of interest 
    table_region=ist_perception_msgs.msg.TableRegion() 
    table_region=ist_perception_msgs.msg.TableRegion()

    #UPPER SHELF
    table_region.x_filter_max=2.5
    table_region.x_filter_min=-2.5
    table_region.y_filter_max=2.5
    table_region.y_filter_min=-2.3
    table_region.z_filter_max=2.5
    table_region.z_filter_min=-2.9
    
        # LOWER SHELF
#     table_region.x_filter_max=1.4
#     table_region.x_filter_min=0.7
#     table_region.y_filter_max=0.2
#     table_region.y_filter_min=-0.3
#     table_region.z_filter_max=0.8
#     table_region.z_filter_min=0.5




    
    goal = fmm_msg.TaskDependentGraspingGoal(object_category=object_category,task_type=task_type,table_region=table_region,placement_pose=pre_place_pose, mode=mode)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    #if client.get_result().status:
    #    return True
    #else:
    #    return False
    #print len(client.get_result().grip_list.grip_states)
    
    return client.get_result().status
    

def go_to_place(place_id, relMotionClient):
    locations = retrieve_locations()
    # TODO extract index from location list
    for name, pose in zip(locations.place_id, locations.pose):
      if place_id == name:
        print "Found the following pose for place id", name
        print pose
        break
    else: # for-loop didn't break
      raise ValueError("Did not find place id "+ name+ " in list of predefined places: " + str(locations.place_id))
      

    actionservername = '/WideRangeTravel' # defined in ROSTrajectoryPlanner
    navclient = actionlib.SimpleActionClient(actionservername, fmm_msg.AISPathPlanner_Motion_RequestAction)
    navclient.wait_for_server(rospy.Duration.from_sec(5.0))
    #pose is still defined from within the above loop
    result = navigate_to_next_pose(pose.x,pose.y,pose.theta, navclient) 
    
    if True: #result.error_code == fmm_msg.AISPathPlanner_Motion_RequestResult.GOAL_REACHED:
        actionservername = '/LocalizerToReference' # defined in ROSTrajectoryPlanner
        locclient = actionlib.SimpleActionClient(actionservername, fmm_msg.AISLocalizer_Localize_To_Reference_ScanAction)
        locclient.wait_for_server(rospy.Duration.from_sec(5.0))

        goal = fmm_msg.AISLocalizer_Localize_To_Reference_ScanGoal()
        goal.place_id = place_id
        global actionTimeout, preemptTimeout

        attempts = 5
        dx_a = -0.02
        dx_b =  0.02
        dy_a = -0.02
        dy_b =  0.02
        dtheta_a = -0.1
        dtheta_b =  0.1
        
        for i in range(attempts):
          status = locclient.send_goal_and_wait(goal, execute_timeout = actionTimeout, preempt_timeout = preemptTimeout)
          if status != GoalStatus.SUCCEEDED:
            print "Can not localize to refscan attempt #", i, "failed."
            relMotionClient.relative_motion(random.uniform(dx_a, dx_b),random.uniform(dy_a, dy_b) , random.uniform(dtheta_a, dtheta_b));
          else:          
            offset = locclient.get_result()
            # TODO need to check offset. if did not work do again, maybe issue a random motion?
            print "offset is", offset
            relMotionClient.relative_motion(offset.offsetFromRobotToReferenceScan.x, offset.offsetFromRobotToReferenceScan.y, offset.offsetFromRobotToReferenceScan.theta)
            return True
        print "Can not localize to refscan all attempts failed."
    else:
        print "error is", result.error_code, navclient.get_status().text
        raise StandardError(navclient.get_status().text)
    
    return False
    

def navigate_to_next_pose(x,y,theta, client): 
  goal = fmm_msg.AISPathPlanner_Motion_RequestActionGoal
  pose2d = geo_msg.Pose2D()
  pose2d.x= x
  pose2d.y= y
  pose2d.theta= theta
  goal.goal_pose = pose2d
  print "now sending goal",
  global actionTimeout, preemptTimeout
  status = client.send_goal_and_wait(goal, execute_timeout = actionTimeout, preempt_timeout = preemptTimeout)
  if status != GoalStatus.SUCCEEDED:
    print #to break the above line
    raise StandardError("Navigation failed due to timeout")
  else:
    print "done."
    #FIXME (in localizer): result is zero pose
    #result = client.get_result()
    #print result
    #return result
    

def search(list_of_locations): #shall be ((x1,y1,theta1), (x2,y2,theta2)...)
  actionservername = '/WideRangeTravel' # defined in ROSTrajectoryPlanner
  client = actionlib.SimpleActionClient(actionservername, fmm_msg.AISPathPlanner_Motion_RequestAction)
  while not client.wait_for_server(rospy.Duration.from_sec(5.0)):
    print "Waiting for Action client for path planning. Maybe check the name of the actionserver:", actionservername, 
    print "or have a look at the 'ros_trajectory_planner' node in rxgraph"

  for x,y,theta in list_of_locations:
    navigate_to_next_pose(x,y,theta, client)
    #finepositioning

def retrieve_locations():
  '''Fetches the predefined locations from the localizer.
  Returns an object containing, e.g.,
  place_id: ['placeone', 'placetwo']
  pose: 
    - 
      x: 5.86889418179
      y: 4.08205141412
      theta: -0.635565089966
    - 
      x: 10.0712686626
      y: 5.16956170103
      theta: -0.00160621709658
  '''
  actionservername = '/LocalizerSendPlaceNames'
  client = actionlib.SimpleActionClient(actionservername, fmm_msg.AISLocalizer_Place_NamesAction)
  while not client.wait_for_server(rospy.Duration.from_sec(5.0)):
    print "Waiting for Action client for place retrieval. Maybe check the name of the actionserver:", actionservername, 

  goal = fmm_msg.AISLocalizer_Place_NamesGoal()
  client.send_goal_and_wait(goal)
  result = client.get_result()
  return result

def place_object(cmd,bucket):
    print 'Calling bottle sorter...'
    rospy.wait_for_service('/bottle_sorter_cmd')
    try:
        bottle_sorter=rospy.ServiceProxy('/bottle_sorter_cmd', bottleSorterCmd)
        response = bottle_sorter(cmd,bucket)
    except rospy.ServiceException, e:
        print ' Service call failed: %s'%e
        return False
    return True

if __name__ == '__main__':
    import argparse
    shelf_pos=['_Upper','_Lower']
    print shelf_pos
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This scripts orchestrates the first-mm skills to a fetch & carry application''')
    parser.add_argument('place_id', nargs='+', help='goal id(s) to navigate to and fine-position')
    args = parser.parse_args()

    rospy.init_node('fetch_and_carry_application')

    shelfFilterClient = ShelfFilterClient()
    bottleActionClient = BottleActionClient()
    relMotionClient = RelMotionClient()

    # fetch and carry
    for place_id in args.place_id: # for each shelf
      print "Warning: Wrong assumption that fine-localization-place-names are equal to shelf-names made"
      if go_to_place(place_id, relMotionClient): #
        for place_id_pos in shelf_pos:
        #try:          

          # Felix's bottle sorter frame
	  pre_place_pose=geometry_msgs.msg.PoseStamped()
	  pre_place_pose=geometry_msgs.msg.PoseStamped()
	  pre_place_pose.header.frame_id="/base_link"
	  pre_place_pose.pose.position.x=-0.225
	  pre_place_pose.pose.position.y=0.234
	  pre_place_pose.pose.position.z=1.17
	  pre_place_pose.pose.orientation.x=0.977
	  pre_place_pose.pose.orientation.y=-0.21
	  pre_place_pose.pose.orientation.z=-0.013
	  pre_place_pose.pose.orientation.w=0.004

	  #pre_place_pose=geometry_msgs.msg.PoseStamped()
	  #pre_place_pose.header.frame_id="/base_link"
	  #pre_place_pose.pose.position.x=-0.285
	  #pre_place_pose.pose.position.y=0.092
	  #pre_place_pose.pose.position.z=1.29
	  #pre_place_pose.pose.orientation.x=1.0
	  #pre_place_pose.pose.orientation.y=-0.017
	  #pre_place_pose.pose.orientation.z=0.005
	  #pre_place_pose.pose.orientation.w=0.01

          if place_id == 'Table': # relational grasping stuff
            mode=fmm_msg.TaskDependentGraspingGoal.PICK_AND_PLACE
            shelfFilterClient.filter_table("/Table")
            rospy.sleep(2.0)
            rospy.set_param('/ist_generate_trajectories_server/use_arne_planner',False)
            # Look for objects of the given category
            object_category=ist_msgs.msg.ObjectCategory() 
            object_category.id=object_category.BOTTLE
    
            # Task type
            task_type=ist_msgs.msg.TaskType()
            task_type.id=task_type.PICK_PLACE_ON 

            perception_plus_inference_and_grasping(object_category,task_type,pre_place_pose,mode) # pre_place_pose_ignored since mode=PICK_PLACE_ON 
          else: # fetch and carry stuff
            mode=fmm_msg.TaskDependentGraspingGoal.FETCH_AND_CARRY
            shelfFilterClient.filter_shelf(place_id+place_id_pos)
            rospy.sleep(2.0)


            # Look for objects of the given category
            object_category=ist_msgs.msg.ObjectCategory() 
            object_category.id=object_category.BOTTLE
    
            # Task type
            task_type=ist_msgs.msg.TaskType()
            task_type.id=task_type.PICK_PLACE_ON 


            perception_plus_inference_and_grasping(object_category,task_type,pre_place_pose,mode)



            #status=1
            #while status!=fmm_msg.TaskDependentGraspingResult.NO_OBJECT_OF_GIVEN_CATEGORY_FOUND: # while we detect objects of the given class...
            status=perception_plus_inference_and_grasping(object_category,task_type,pre_place_pose)
            print str(status)
            if status==fmm_msg.TaskDependentGraspingResult.SUCCESS: # 
              print 'Debug: Everything went fine'
            else:
              print 'error could not get to goal'



#          if success:
#              continue

#           if success:
#               #if appropriate Grasp
#               #and put in Box
#               cmd=2
#               bucket=-1
#               place_object(cmd,bucket)
#               return
          


#         except ValueError as ve:
#           print "Exception:", ve.message
#           traceback.extract_stack()
#           #Can't do much here, except skip that task
#         except StandardError as se:
#           print "Exception:", se.message
#           #More Error Handling, e.g.
#           #Should go back to initial pose, try again
#           traceback.extract_stack()
#         finally:
#           shelfFilterClient.filter_shelf("")


    # locations = retrieve_locations()
    # pose = locations.pose[0]

    # actionservername = '/WideRangeTravel' # defined in ROSTrajectoryPlanner
    # client = actionlib.SimpleActionClient(actionservername, fmm_msg.AISPathPlanner_Motion_RequestAction)
    # print "gumbaq"
    # client.wait_for_server(rospy.Duration.from_sec(5.0))
    # navigate_to_next_pose(pose.x,pose.y,pose.theta, client) 


    
#    localize()

