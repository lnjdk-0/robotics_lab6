#!/usr/bin/env python3
"""
A ROS node that fits a sphere to a set of 3D points received from a subscriber.
The fitted sphere parameters (center coordinates and radius) are published to a topic.

@author Leon Neverov
@credits https://github.com/hsaeidi-uncw/robot_vision_lectures
"""

import numpy as np
import rospy
from robot_vision_lectures.msg import XYZarray, SphereParams

msg_received = False


def get_msg(data):
    """
       Callback function for the 'xyz_cropped_ball' subscriber. Extracts and stores the 3D points.

       Args:
           data (XYZarray): The received message containing the 3D points.
    """


    global msg_received
    global point_arr
    msg_received = True
    point_arr = [(point.x, point.y, point.z) for point in data.points]


def get_radius(POINTS):
    """
    Calculates the radius of the fitted sphere.

    Args:
        POINTS (numpy.ndarray): The fitted sphere parameters.

    Returns:
        radius (float): The radius of the fitted sphere.
    """


    xc, yc, zc, _ = POINTS[0]
    radius = np.sqrt(POINTS[0][3] + xc ** 2 + yc ** 2 + zc ** 2)
    return radius


def sphere_fit(points):
    """
    Fits a sphere to the given 3D points using linear least squares.

    Args:
        points (list): A list of 3D points as tuples (x, y, z).

    Returns:
        POINTS (numpy.ndarray): The fitted sphere parameters.
    """


    A = []
    B = []

    for point in points:
        x, y, z = point
        B.append([x ** 2 + y ** 2 + z ** 2])
        A.append([2 * x, 2 * y, 2 * z, 1])

    A = np.array(A)
    B = np.array(B)

    B = B.reshape(len(B), 1)
    A = A.reshape(len(B), 4)

    POINTS = np.linalg.lstsq(A, B, rcond=None)
    return POINTS

if __name__ == '__main__':
    # define the node and subcribers and publishers
    rospy.init_node('sphere_fit', anonymous=True)
    # define a subscriber to XYZ array
    sub = rospy.Subscriber('xyz_cropped_ball', XYZarray, get_msg)
    # define a publisher to publish Sphere params
    pub = rospy.Publisher("/sphere_params", SphereParams, queue_size=1)
    # set the loop frequency
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if msg_received:
            POINTS = sphere_fit(point_arr)
            radius = get_radius(POINTS)
            xc, yc, zc, _ = POINTS[0]

            sphere_params = SphereParams(float(xc), float(yc), float(zc), radius)
            pub.publish(sphere_params)

        rate.sleep()
