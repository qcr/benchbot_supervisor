#!/usr/bin/env python
from __future__ import print_function

import rospy
import math
import tf

from geometry_msgs.msg import Twist, Vector3, Pose
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

def heading():
    current = rospy.wait_for_message('/amcl_pose', Pose, timeout=5)   
    current = current.pose.pose.orientation

    result = tf.transformations.euler_from_quaternion((current.x, current.y, current.z, current.w))
    return {'x': result[0], 'y': result[1], 'z': result[2]}
    

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