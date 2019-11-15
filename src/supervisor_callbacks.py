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


def __pi_wrap(angle):
    return np.mod(angle + np.pi, 2 * np.pi) - np.pi


def __safe_dict_get(d, key, default):
    return d[key] if type(d) is dict and key in d else default


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


def move_angle(data, publisher, supervisor):
    # TODO handle collisions
    angle = __pi_wrap(__safe_dict_get(data, 'angle', 0))
    vel_msg = Twist(angular=Vector3(z=np.sign(angle) * _MOVE_ANGLE_SPEED))
    hz_rate = rospy.Rate(_MOVE_HZ)
    try:
        # Yes, this is pretty gross top to bottom. We have used a gross sign
        # factor to make our while case consistent between +ve & -ve angles,
        # and used some wrapping to hope our flimsy sign assumptions will hold
        # for us.  Valid simplifying, or bug introduction? Time will tell...
        start = current_pose(data, supervisor)
        current = start
        positive = (1 if angle >= 0 else -1)  # Yes, gross way to handle -ves
        print("Moving to angle %f (hack factor: %d)" % (angle, positive))
        print("Moved angle is: %f" %
              t3.euler.mat2euler(__pose_diff_matrices(start, current))[2])
        while (positive * __pi_wrap(angle - t3.euler.mat2euler(
                __pose_diff_matrices(start, current))[2]) > 0):
            publisher.publish(vel_msg)
            hz_rate.sleep()
            current = current_pose(data, supervisor)
    except Exception as e:
        rospy.logerr("move_angle: failed with exception '%s'" % e)
    publisher.publish(Twist())


def move_distance(data, publisher, supervisor):
    # TODO handle collisions
    distance = __safe_dict_get(data, 'distance', 0)
    vel_msg = Twist(linear=Vector3(x=np.sign(distance) * _MOVE_DISTANCE_SPEED))
    hz_rate = rospy.Rate(_MOVE_HZ)
    try:
        # This is relatively gross code at this stage...
        start = current_pose(data, supervisor)
        current = start
        positive = (1 if distance >= 0 else -1)
        while (positive * (distance - t3.affines.decompose(
                __pose_diff_matrices(start, current))[0][0]) > 0):
            publisher.publish(vel_msg)
            hz_rate.sleep()
            current = current_pose(data, supervisor)
    except Exception as e:
        rospy.logerr("move_distance: failed with exception '%s'" % e)
    publisher.publish(Twist())
