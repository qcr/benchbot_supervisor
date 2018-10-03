#!/usr/bin/env python
from __future__ import print_function

import rospy
import math

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, Vector3
from benchbot_teleop.srv import Command
from pepper_navigation.srv import LocationPose

def is_done():
    return {'result': False}

def at_position():
    rospy.wait_for_service('/navigation/get_location_pose')
    try:
        current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)

        position = (
            current_pose.pose.pose.position.x,
            current_pose.pose.pose.position.y,
            current_pose.pose.pose.position.z
        )
        
        get_location = rospy.ServiceProxy('/navigation/get_location_pose', LocationPose)
        pose = get_location('goal')
        
        goal_position = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        )
        distance = math.sqrt((position[0] - goal_position[0]) ** 2 + (position[1] - goal_position[1])  ** 2)
        return {'result': distance < 0.3}

    except rospy.ROSException as e:
        print('wait_for_message call failed:', e)
        return {'result': False}
    except rospy.ServiceException as e:
        print('Service call failed:', e)
        return {'result': False}
    
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

def forward(x=0.5):
    if x <= 0:
        return {'result': 1, 'error': 'Distance must be greater than 0'}
        
    return dispatch(Twist(linear=Vector3(x, 0, 0)))    

def left(r=1.0):
    if r <= 0:
        return {'result': 1, 'error': 'Radians must be greater than 0'}

    return dispatch(Twist(angular=Vector3(0, 0, r)))

def right(r=1.0):
    if r <= 0:
        return {'result': 1, 'error': 'Radians must be greater than 0'}
        
    return dispatch(Twist(angular=Vector3(0, 0, -r)))