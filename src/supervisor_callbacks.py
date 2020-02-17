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

_MOVE_HZ = 20

_MOVE_ANGLE_SPEED = 0.5
_MOVE_DISTANCE_SPEED = 0.5

_MOVE_NEXT_TOL_DIST = 0.1
_MOVE_NEXT_TOL_YAW = 30 * np.pi / 180.0
_MOVE_NEXT_K_DIST = 2.5
_MOVE_NEXT_K_DIRECTION = 7.5
_MOVE_NEXT_K_UMMM = -5


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
    # Expected format is [x,y,z,w,X,Y,Z] (to match how they have been dumped
    # into the environment metadata files)
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


def __yaw_b_wrt_a(matrix_a, matrix_b):
    # Computes the yaw diff of homogenous transformation matrix b w.r.t. a
    return Rot.as_rotvec(
        Rot.from_dcm(__tr_b_wrt_a(matrix_a, matrix_b)[0:3, 0:3]))[2]


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


def current_pose(data, supervisor):
    # TODO REMOVE HARDCODED FRAME NAMES!!!
    return __tf_ros_stamped_to_tf_matrix(
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
        while (positive * __pi_wrap(angle - Rot.from_dcm(
                __tr_b_wrt_a(start, current)[:-1, :-1]).as_euler('XYZ')[2]) >
               0):
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
        while (positive *
               (distance - __tr_b_wrt_a(start, current)[0, -1]) > 0):
            publisher.publish(vel_msg)
            hz_rate.sleep()
            current = current_pose(data, supervisor)
    except Exception as e:
        rospy.logerr("move_distance: failed with exception '%s'" % e)
    publisher.publish(Twist())


def move_next(data, publisher, supervisor):
    # Configure if this is out first step
    if 'trajectory_pose_next' not in supervisor.environment_data:
        supervisor.environment_data['trajectory_pose_next'] = 0

    # Get the goal
    goal = __pose_vector_to_tf_matrix(
        np.fromstring(supervisor.environment_data['trajectory_poses'][
            supervisor.environment_data['trajectory_pose_next']].strip()[1:-1],
                      sep=", "))
    # rospy.logwarn("Servoing to goal: %s" % goal)

    # Servo to the goal
    vel_msg = Twist()
    hz_rate = rospy.Rate(_MOVE_HZ)
    while True:
        # Get latest data
        current = current_pose(data, supervisor)
        goal_dist = __dist_from_a_to_b(current, goal)
        goal_direction = __ang_to_b_wrt_a(current, goal)

        # rospy.logwarn(
        #     "Current: %s, %s" %
        #     (Rot.as_rotvec(Rot.from_dcm(current[0:3, 0:3])), current[0:3, -1]))
        # rospy.logwarn(
        #     "Goal: %s, %s" %
        #     (Rot.as_rotvec(Rot.from_dcm(goal[0:3, 0:3])), goal[0:3, -1]))
        rospy.logwarn("Values: %f, %f" % (goal_dist, goal_direction))

        # Bail if exit conditions are met
        if (goal_dist < _MOVE_NEXT_TOL_DIST):  # and
            # np.abs(rel_yaw) < _MOVE_NEXT_TOL_YAW):
            break

        # Construct & send velocity msg
        vel_msg.linear.x = _MOVE_NEXT_K_DIST * goal_dist
        vel_msg.angular.z = _MOVE_NEXT_K_DIRECTION * goal_direction  # +
        # _MOVE_NEXT_GAIN_YAW_DIFF * BETA
        publisher.publish(vel_msg)
        hz_rate.sleep()
    publisher.publish(Twist())

    # Register that we completed this goal
    supervisor.environment_data['trajectory_pose_next'] += 1
    if (supervisor.environment_data['trajectory_pose_next'] >= len(
            supervisor.environment_data['trajectory_poses'])):
        rospy.logerr("You have run out of trajectory poses!")
