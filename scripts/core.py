#!/usr/bin/env python
from __future__ import print_function

import rospy
import math
import tf
import actionlib

from geometry_msgs.msg import *
from actionlib_msgs.msg import GoalStatus, GoalID
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from navigator.srv import LocationPose

from benchbot_teleop.srv import Command

def is_done():
    return {'result': False}

def dispatch(velocity):
    rospy.wait_for_service('/command')
    try:
        send_command = rospy.ServiceProxy('/command', Command)
        resp = send_command(velocity)

        if (resp.result == 0):
            return {'result': 0}
        else:
            return {'result': resp.result, 'error': resp.message}

    except rospy.ServiceException as e:
        print('Service call failed:', e)

def heading(context, route_id):
    current = rospy.wait_for_message('/amcl_pose', Pose, timeout=5)   
    current = current.pose.pose.orientation

    result = tf.transformations.euler_from_quaternion((current.x, current.y, current.z, current.w))
    return {'x': result[0], 'y': result[1], 'z': result[2]}
    
def forward(context, route_id, x=0.5):
    if x <= 0:
        return {'result': 1, 'error': 'Distance must be greater than 0'}
        
    return dispatch(Twist(linear=Vector3(x, 0, 0)))    

def left(context, route_id, r=1.0):
    if r <= 0:
        return {'result': 1, 'error': 'Radians must be greater than 0'}

    return dispatch(Twist(angular=Vector3(0, 0, r)))

def right(context, route_id, r=1.0):
    if r <= 0:
        return {'result': 1, 'error': 'Radians must be greater than 0'}
        
    return dispatch(Twist(angular=Vector3(0, 0, -r)))

def goto(context, route_id, location_id):
    rospy.wait_for_service('/navigation/get_location_pose')
    get_location = rospy.ServiceProxy('/navigation/get_location_pose', LocationPose)

    location = get_location(location_id)
    print(location)
    pose_stamped = PoseStamped()

    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = '/map'

    pose_stamped.pose = location.pose

    move_goal = MoveBaseGoal()
    move_goal.target_pose = pose_stamped

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    client.send_goal_and_wait(move_goal)
    client.stop_tracking_goal()
    return  {'result': 0}

def goto_next(context, route_id):
    if not hasattr(context, 'goto_next'):
        setattr(context, 'goto_next', [])

    idx = len(context.goto_next)

    try:
        location_id = context.config['routes'][route_id]['post']['locations'][idx]
        
        rospy.wait_for_service('/navigation/get_location_pose')
        get_location = rospy.ServiceProxy('/navigation/get_location_pose', LocationPose)

        location = get_location(location_id)
        print(location)
        pose_stamped = PoseStamped()

        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = '/map'
        
        pose_stamped.pose = location.pose

        move_goal = MoveBaseGoal()
        move_goal.target_pose = pose_stamped

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        client.send_goal_and_wait(move_goal)
        client.stop_tracking_goal()

        context.goto_next.append(location_id)
        return  {'result': 0}

    except IndexError:
        return {'result': 1, 'error': 'All locations have been visted'}
    #send_command = rospy.ServiceProxy('/navigator', Command)

def current_location(context, route_id):
  if not hasattr(context, 'goto_next'):
        setattr(context, 'goto_next', [])
  return {'location_id': context.goto_next[-1] if len(context.goto_next) > 0 else 'home'}

def get_locations_from_task(context, route_id):
  return context.config['locations']
  
def get_locations_from_navigator(context, route_id):
  rospy.wait_for_service('/navigation/get_locations')
  get_locations = rospy.ServiceProxy('/navigation/get_locations', Locations)
  
  locations = []
  for location in get_locations().result.locations:
    if location.location_id not in context.config['locations']:
      continue
    
    locations.append({
      'id': location.location_id,
      'position': {
        'x': location.marker.pose.position.x, 
        'y': location.marker.pose.position.y, 
        'z': location.marker.pose.position.z
      },
      'orientation': {
        'x': location.marker.pose.orientation.x, 
        'y': location.marker.pose.orientation.y, 
        'z': location.marker.pose.orientation.z, 
        'w': location.marker.pose.orientation.w,
      }
    })

  return locations

def get_location_from_navigator(context, route_id, location_id):
  if location_id not in context.config['locations']:
    raise Exception('Unknown location - ' + location_id)

  rospy.wait_for_service('/navigation/get_location_pose')
  get_location = rospy.ServiceProxy('/navigation/get_location_pose', LocationPose)

  location = get_location(location_id)
  
  return {
    'id': location_id,
    'position': {
      'x': location.pose.position.x, 
      'y': location.pose.position.y, 
      'z': location.pose.position.z
    },
    'orientation': {
      'x': location.pose.orientation.x, 
      'y': location.pose.orientation.y, 
      'z': location.pose.orientation.z, 
      'w': location.pose.orientation.w,
    }
  }