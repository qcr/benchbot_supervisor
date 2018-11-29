#!/usr/bin/env python
from __future__ import print_function

import rospy
import math

from collections import Counter
from geometry_msgs.msg import PoseWithCovarianceStamped

def comparer(context, id, data):
  try:
    objective = context.config['objectives'][id]
    expected = objective['expected']
    result = {'accuracy': 0, 'tp': 0, 'fp': 0, 'fn': 0, 'tn': 0}
    
    for idx in expected:
      a = Counter(expected[idx])
      b = Counter(data[idx] if idx in data else [])

      result['tp'] += len(list((a & b).elements()))
      result['fn'] += len(list((a - b).elements()))
      result['fp'] += len(list((b - a).elements()))

    result['accuracy'] = result['tp'] / float(result['tp'] + result['fn'] + result['fp'])
    return result
        
  except Exception as e:
    print(str(e))
  
  return  {'accuracy': 0}

def equals(context, id, data):
  score = 0

  try:
    objective = context.config['objectives'][id]
    expected = objective['expected']
    
    score += 1 if expected == data else 0

  except Exception as e:
    print(str(e))
  
  return  {'score': score}

def at_position(context, objective):
    try:
        objective = context.config['objectives'][objective]

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

def at_location(context, objective):
    try:
        objective = context.config['objectives'][objective]
        location = objective['location']

        current_pose = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5)
        tolerance = objective['tolerance'] if 'tolerance' in context.config else 0.3

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

def detected_objects_at_marker(context, objective, marker, detected):
    expected = {}
    
    for item in context.config['items']:
        if 'marker' in item and context.config['markers'][item['marker']]['location'] == marker:
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

def detected_objects_at_location(context, objective, location, detected):
    expected = {}
    for item in context.config['items']:
        if 'location' in item and item['location'] == location:
            item_type = item['type']
            
            if item_type not in expected:
                expected[item_type] = 0
            expected[item_type] += 1

        elif 'marker' in item and context.config['markers'][item['marker']]['location'] == location:
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

    supervisor = Supervisor('tasks/guiabot_demo.json')
    print(comparer(supervisor.context, 'main', {
      "location_3": ["bottle"]
    }))

    supervisor = Supervisor('tasks/guiabot_demo_simple.json')
    print(equals(supervisor.context, 'main', 'location_1'))
    #print(detected_objects_at_marker(supervisor.spec, 'moo', 'kitchen', {'duck': 1}))
