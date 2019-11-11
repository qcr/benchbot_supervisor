import base64
import cv2
import jsonpickle
import jsonpickle.ext.numpy as jet
import numpy as np
import ros_numpy

jet.register_handlers()


def encode_color_image(data):
    return {
        'encoding':
            data.encoding,
        'data':
            base64.b64encode(cv2.imencode('.png', ros_numpy.numpify(data))[1])
    }


def encode_depth_image(data):
    return jsonpickle.encode(ros_numpy.numpify(data))


def encode_laserscan(data):
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


def encode_pose(data):
    # TODO
    pass
