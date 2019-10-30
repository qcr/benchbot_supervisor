import base64
import cv2
import ros_numpy


def encode_ros_image(data):
    return {
        'encoding':
            data.encoding,
        'data':
            base64.b64encode(cv2.imencode('.png', ros_numpy.numpify(data))[1])
    }
