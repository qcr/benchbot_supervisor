import base64
import cv2
import jsonpickle
import jsonpickle.ext.numpy as jet
import numpy as np
import pprint
import ros_numpy
import rospy
from scipy.spatial.transform import Rotation as Rot

from geometry_msgs.msg import Twist, Vector3

jet.register_handlers()

_MOVE_HZ = 20

_MOVE_TOL_DIST = 0.01
_MOVE_TOL_YAW = np.deg2rad(1)

_MOVE_ANGLE_K = 3

_MOVE_POINT_K_RHO = 2
_MOVE_POINT_K_ALPHA = 5


def __ang_to_b_wrt_a(matrix_a, matrix_b):
    # Computes the 2D angle to the location of homogenous transformation matrix
    # b, wrt a
    diff = __tr_b_wrt_a(matrix_a, matrix_b)
    return np.arctan2(diff[1, 3], diff[0, 3])


def __dist_from_a_to_b(matrix_a, matrix_b):
    # Computes 2D distance from homogenous transformation matrix a to b
    return np.linalg.norm(__tr_b_wrt_a(matrix_a, matrix_b)[0:2, 3])


def __pi_wrap(angle):
    return np.mod(angle + np.pi, 2 * np.pi) - np.pi


def __pose_vector_to_tf_matrix(pv):
    # Expected format is [x,y,z,w,X,Y,Z]
    return np.vstack((np.hstack((Rot.from_quat(pv[0:4]).as_dcm(),
                                 np.array(pv[4:]).reshape(3,
                                                          1))), [0, 0, 0, 1]))


def __safe_dict_get(d, key, default):
    return d[key] if type(d) is dict and key in d else default


def __tf_ros_stamped_to_tf_matrix(tfs):
    # FFS ROS... how do you still not have a method for this in 2020...
    return __pose_vector_to_tf_matrix([
        tfs.transform.rotation.x, tfs.transform.rotation.y,
        tfs.transform.rotation.z, tfs.transform.rotation.w,
        tfs.transform.translation.x, tfs.transform.translation.y,
        tfs.transform.translation.z
    ])


def __tr_b_wrt_a(matrix_a, matrix_b):
    # Computes matrix_b - matrix_a (transform b wrt to transform a)
    return np.matmul(np.linalg.inv(matrix_a), matrix_b)


def __transrpy_to_tf_matrix(trans, rpy):
    # Takes a translation vector & roll pitch yaw vector
    return __pose_vector_to_tf_matrix(
        np.hstack((Rot.from_euler('XYZ', rpy).as_quat(), trans)))


def __yaw_b_wrt_a(matrix_a, matrix_b):
    # Computes the yaw diff of homogenous transformation matrix b w.r.t. a
    return Rot.from_dcm(__tr_b_wrt_a(matrix_a,
                                     matrix_b)[0:3, 0:3]).as_euler('XYZ')[2]


def _current_pose(supervisor):
    # TODO REMOVE HARDCODED FRAME NAMES!!!
    return __tf_ros_stamped_to_tf_matrix(
        supervisor.tf_buffer.lookup_transform('map', 'robot', rospy.Time()))


def _move_to_angle(goal, publisher, supervisor):
    # Servo until orientation matches that of the requested goal
    vel_msg = Twist()
    hz_rate = rospy.Rate(_MOVE_HZ)
    while not supervisor._query_simulator('is_collided')['is_collided']:
        # Get latest orientation error
        orientation_error = __yaw_b_wrt_a(_current_pose(supervisor), goal)

        # Bail if exit conditions are met
        if np.abs(orientation_error) < _MOVE_TOL_YAW:
            break

        # Construct & send velocity msg
        vel_msg.angular.z = _MOVE_ANGLE_K * orientation_error
        publisher.publish(vel_msg)
        hz_rate.sleep()
    publisher.publish(Twist())


def _move_to_pose(goal, publisher, supervisor):
    # Servo to the point represented by goal, then rotate on spot to match pose
    # NOTE: this was more feasible due to instabilities in pose servoing if
    # poses are extremely close together (if "direction to goal" flips...
    # angles all of a sudden have 180 degree error)

    # Figure out direction
    direction = (1 if np.abs(__ang_to_b_wrt_a(_current_pose(supervisor), goal))
                 < np.pi else -1)

    # Servo to point
    # rho = distance from current to goal
    # alpha = angle of goal vector in vehicle frame
    vel_msg = Twist()
    hz_rate = rospy.Rate(_MOVE_HZ)
    while not supervisor._query_simulator('is_collided')['is_collided']:
        # Get latest position error
        current = _current_pose(supervisor)
        rho = __dist_from_a_to_b(current, goal)
        alpha = __ang_to_b_wrt_a(current, goal)

        # Bail if exit conditions are met
        if (rho < _MOVE_TOL_DIST):
            break

        # Construct & send velocity msg
        vel_msg.linear.x = direction * _MOVE_POINT_K_RHO * rho
        vel_msg.angular.z = direction * _MOVE_POINT_K_ALPHA * alpha
        publisher.publish(vel_msg)
        hz_rate.sleep()
    publisher.publish(Twist())

    # Servo to match requested orientation
    _move_to_angle(goal, publisher, supervisor)


def create_pose_list(data, supervisor):
    # TODO REMOVE HARDCODED TREE STRUCTURE!!!
    # TODO REMOVE HACK FOR FIXING CAMERA NAME!!!
    HARDCODED_POSES = ['odom', 'robot', 'left_camera', 'lidar']
    tfs = {
        p: __tf_ros_stamped_to_tf_matrix(
            supervisor.tf_buffer.lookup_transform('map', p, rospy.Time()))
        for p in HARDCODED_POSES
    }
    return jsonpickle.encode({
        'camera' if 'camera' in k else k: {
            'parent_frame': 'map',
            'translation_xyz': v[:-1, -1],
            'rotation_rpy': Rot.from_dcm(v[:-1, :-1]).as_euler('XYZ'),
            'rotation_xyzw': Rot.from_dcm(v[:-1, :-1]).as_quat()
        } for k, v in tfs.items()
    })


def encode_camera_info(data, supervisor):
    return jsonpickle.encode({
        'frame_id': data.header.frame_id,
        'height': data.height,
        'width': data.width,
        'matrix_intrinsics': np.reshape(data.K, (3, 3)),
        'matrix_projection': np.reshape(data.P, (3, 4))
    })


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
            np.array([[
                data.ranges[i],
                __pi_wrap(data.angle_min + i * data.angle_increment)
            ] for i in range(0, len(data.ranges))]),
        'range_min':
            data.range_min,
        'range_max':
            data.range_max
    })


def move_angle(data, publisher, supervisor):
    # Derive a corresponding goal pose & send the robot there
    _move_to_pose(
        np.matmul(
            _current_pose(supervisor),
            __transrpy_to_tf_matrix([0, 0, 0], [
                0, 0,
                __pi_wrap(np.deg2rad(__safe_dict_get(data, 'angle', 0)))
            ])), publisher, supervisor)


def move_distance(data, publisher, supervisor):
    # Derive a corresponding goal pose & send the robot there
    _move_to_pose(
        np.matmul(
            _current_pose(supervisor),
            __transrpy_to_tf_matrix(
                [__safe_dict_get(data, 'distance', 0), 0, 0], [0, 0, 0])),
        publisher, supervisor)


def move_next(data, publisher, supervisor):
    # Configure if this is our first step
    if supervisor.environment_name is None:
        supervisor.environment_name = (
            supervisor.config['environment_names'][supervisor._query_simulator(
                'map_selection_number')['map_selection_number']])
    if ('trajectory_pose_next' not in supervisor.environment_data[
            supervisor.environment_name]):
        supervisor.environment_data[
            supervisor.environment_name]['trajectory_pose_next'] = 0

    # Servo to the goal pose
    _move_to_pose(
        __pose_vector_to_tf_matrix(
            np.take(
                np.fromstring(
                    supervisor.environment_data[
                        supervisor.environment_name]['trajectory_poses']
                    [supervisor.environment_data[supervisor.environment_name]
                     ['trajectory_pose_next']].strip()[1:-1],
                    sep=", "), [1, 2, 3, 0, 4, 5, 6])), publisher, supervisor)

    # Register that we completed this goal
    supervisor.environment_data[
        supervisor.environment_name]['trajectory_pose_next'] += 1
    if (supervisor.environment_data[supervisor.environment_name]
        ['trajectory_pose_next'] >= len(supervisor.environment_data[
            supervisor.environment_name]['trajectory_poses'])):
        rospy.logerr("You have run out of trajectory poses!")
