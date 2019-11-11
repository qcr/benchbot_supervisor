import base64
import cv2
import jsonpickle
import jsonpickle.ext.numpy as jet
import numpy as np
import ros_numpy
import rospy
import transforms3d as t3

from geometry_msgs.msg import Twist, Vector3

jet.register_handlers()

_MOVE_ANGLE_SPEED = 0.5
_MOVE_DISTANCE_SPEED = 0.5
_MOVE_HZ = 20


def __transform_stamped_to_matrix(tfs):
    # FFS ROS... how do you still not have a method for this in 2019...
    return t3.affines.compose([
        tfs.transform.translation.x, tfs.transform.translation.y,
        tfs.transform.translation.z
    ],
                              t3.quaternions.quat2mat([
                                  tfs.transform.rotation.w,
                                  tfs.transform.rotation.x,
                                  tfs.transform.rotation.y,
                                  tfs.transform.rotation.z
                              ]), [1, 1, 1])


def __pose_diff_matrices(pose_1, pose_2):
    # Returns pose_2 - pose_1
    return np.matmul(np.linalg.inv(pose_1), pose_2)


def current_pose(data, supervisor):
    # TODO REMOVE HARDCODED FRAME NAMES!!!
    return __transform_stamped_to_matrix(
        supervisor.tf_buffer.lookup_transform('map', 'robot', rospy.Time()))


def encode_color_image(data, supervisor):
    return {
        'encoding':
            data.encoding,
        'data':
            base64.b64encode(cv2.imencode('.png', ros_numpy.numpify(data))[1])
    }


def encode_depth_image(data, supervisor):
    return jsonpickle.encode(ros_numpy.numpify(data))


def encode_laserscan(data, supervisor):
    return jsonpickle.encode({
        'scans':
            np.array(
                [[data.ranges[i], data.angle_min + i * data.angle_increment]
                 for i in range(0, len(data.ranges))]),
        'range_min':
            data.range_min,
        'range_max':
            data.range_max
    })


def encode_pose(data, supervisor):
    # TODO
    pass


def move_angle(data, pubisher, supervisor):
    pass


def move_distance(data, publisher, supervisor):
    # TODO handle collisions
    print("Received: %s" % data)
    vel_msg = Twist(linear=Vector3(x=_MOVE_DISTANCE_SPEED))
    hz_rate = rospy.Rate(_MOVE_HZ)
    distance = (data['distance']
                if type(data) is dict and 'distance' in data else 0)
    print("Moving distance of: %f" % distance)
    try:
        start = current_pose(data, supervisor)
        current = start
        while (t3.affines.decompose(__pose_diff_matrices(start, current))[0][0]
               < distance):
            publisher.publish(vel_msg)
            hz_rate.sleep()
            current = current_pose(data, supervisor)
    except Exception as e:
        rospy.logerr("move_distance: failed with exception '%s'" % e)
    publisher.publish(Twist())
