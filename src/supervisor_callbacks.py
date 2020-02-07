import base64
import cv2
import jsonpickle
import jsonpickle.ext.numpy as jet
import numpy as np
import ros_numpy
import rospy
from scipy.spatial.transform import Rotation as Rot

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
    # FFS ROS... how do you still not have a method for this in 2020...
    t_mat = np.array([tfs.transform.translation.x, 
                      tfs.transform.translation.y,
                      tfs.transform.translation.z]).reshape(3,1)
    rot_obj = Rot.from_quat([tfs.transform.rotation.x,
                             tfs.transform.rotation.y,
                             tfs.transform.rotation.z,
                             tfs.transform.rotation.w])
    rot_mat = rot_obj.as_dcm()
    h_mat = np.vstack((np.hstack((rot_mat, t_mat)), [0,0,0,1]))
    return h_mat


def __pose_diff_matrices(pose_1, pose_2):
    # Returns pose_2 - pose_1
    return np.matmul(np.linalg.inv(pose_1), pose_2)


def create_pose_list(data, supervisor):
    # TODO REMOVE HARDCODED TREE STRUCTURE!!!
    # TODO REMOVE HACK FOR FIXING CAMERA NAME!!!
    HARDCODED_POSES = ['odom', 'robot', 'left_camera', 'lidar']
    tfs = {
        p: __transform_stamped_to_matrix(
            supervisor.tf_buffer.lookup_transform('map', p, rospy.Time()))
        for p in HARDCODED_POSES
    }
    return jsonpickle.encode({
        'camera' if 'camera' in k else k: {
            'parent_frame':
                'map',
            'translation_xyz':
                v[:-1, -1],
            'rotation_rpy':
                Rot.from_dcm(v[:-1,:-1]).as_euler('XYZ'),
            'rotation_xyzw':
                Rot.from_dcm(v[:-1,:-1]).as_quat()
        } for k, v in tfs.items()
    })


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
        # Even messier using replacement for t3
        start = current_pose(data, supervisor)
        current = start
        positive = (1 if angle >= 0 else -1)  # Yes, gross way to handle -ves
        print("Moving to angle %f (hack factor: %d)" % (angle, positive))
        print("Moved angle is: %f" %
              Rot.from_dcm(__pose_diff_matrices(start, current)[:-1,:-1]).as_euler('XYZ')[2])
        while (positive * __pi_wrap(angle - Rot.from_dcm(
                __pose_diff_matrices(start, current)[:-1,:-1]).as_euler('XYZ')[2]) > 0):
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
        while (positive * (distance - __pose_diff_matrices(start, current)[0,-1]) > 0):
            publisher.publish(vel_msg)
            hz_rate.sleep()
            current = current_pose(data, supervisor)
    except Exception as e:
        rospy.logerr("move_distance: failed with exception '%s'" % e)
    publisher.publish(Twist())
