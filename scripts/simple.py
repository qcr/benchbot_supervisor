#!/usr/bin/env python
from __future__ import print_function

import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped

def at_position(config, objective):
    try:
        objective = config['objectives'][objective_id]

        current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
        tolerance = objective['tolerance'] if 'tolerance' in objective else 0.3

        position = (
            current_pose.pose.pose.position.x,
            current_pose.pose.pose.position.y,
            current_pose.pose.pose.position.z
        )
        
        goal_position = (
            objective['position']['x'],
            objective['position']['y'],
            position[2]
        )
        
        distance = math.sqrt((position[0] - goal_position[0]) ** 2 + (position[1] - goal_position[1])  ** 2)
        return {'result': distance < tolerance, 'e': distance}

    except rospy.ROSException as e:
        print('wait_for_message call failed:', e)
    
    return {'result': False}

def at_location(config, objective):
    try:
        objective = config['objectives'][objective]
        location = objective['location']

        current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
        tolerance = objective['tolerance'] if 'tolerance' in config else 0.3

        position = (
            current_pose.pose.pose.position.x,
            current_pose.pose.pose.position.y,
        )

        if hasattr(location, 'bounds'):
            raise Exception('bounds not yet supported')

        goal_position = (
            location['position']['x'],
            location['position']['y'],
        )
        
        distance = math.sqrt((position[0] - goal_position[0]) ** 2 + (position[1] - goal_position[1])  ** 2)
        return {'result': distance < tolerance, 'e': distance}

    except rospy.ROSException as e:
        print('wait_for_message call failed:', e)
    
    return {'result': False}

def detected_objects_at_marker(config, objective, marker, detected):
    expected = {}
    
    for item in config['items']:
        if 'marker' in item and config['markers'][item['marker']]['location'] == marker:
            item_type = item['type']
            
            if item_type not in expected:
                expected[item_type] = 0
            expected[item_type] += 1

    missed = {}
    keys = list(set(expected.keys()).union(set(detected.keys())))

    for key in keys:
        missed[key] = expected[key] if key in expected else 0

        if key in detected:
            missed[key] = missed[key] - detected[key]

    return missed

def detected_objects_at_location(config, objective, location, detected):
    expected = {}
    for item in config['items']:
        if 'location' in item and item['location'] == location:
            item_type = item['type']
            
            if item_type not in expected:
                expected[item_type] = 0
            expected[item_type] += 1

        elif 'marker' in item and config['markers'][item['marker']]['location'] == location:
            item_type = item['type']
            
            if item_type not in expected:
                expected[item_type] = 0
            expected[item_type] += 1

        else:
            print('Warning: no location or marker specified for item:', item)

    missed = {}
    keys = list(set(expected.keys()).union(set(detected.keys())))

    for key in keys:
        missed[key] = expected[key] if key in expected else 0

        if key in detected:
            missed[key] = missed[key] - detected[key]

    return missed


if __name__ == '__main__':
    from benchbot_supervisor import Supervisor

    supervisor = Supervisor('tasks/example.json')
    print(detected_objects_at_location(supervisor.spec, 'moo', 'kitchen', {'duck': 1, 'toy_car': 1}))
    print(detected_objects_at_marker(supervisor.spec, 'moo', 'kitchen', {'duck': 1}))
