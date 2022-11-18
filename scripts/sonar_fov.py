#! /usr/bin/env python3
"""
Copyright 2022 University of Washington Applied Physics Laboratory
Author: Marc Micatka
"""
import os
import rospy
import rospkg
import numpy as np

from std_msgs.msg import Header
from acoustic_msgs.msg import ProjectedSonarImage
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


def sonar_params(image_msg):
    """
    Store ssonar message parameters in a dictionary to make comparison between messages easy.
    """

    param_dict = {}

    # Find azimuth parameters:
    azimuth_ranges = np.array(
        [beam_dir.y for beam_dir in image_msg.beam_directions])

    num_azimuth_steps = 100  # too many and it looks crazy, too few and it doesn't render well

    azimuth_steps = np.linspace(min(azimuth_ranges), max(azimuth_ranges),
                                num_azimuth_steps)
    elevation_ranges = np.array(
        [beam_dir.z for beam_dir in image_msg.beam_directions])

    elevations = np.linspace(np.arccos(max(elevation_ranges)),
                             np.arccos(min(elevation_ranges)), 2)

    elevations = elevations
    param_dict['range'] = max(image_msg.ranges)
    param_dict['elevations'] = elevations
    param_dict['azimuth_steps'] = azimuth_steps
    param_dict['azimuth_ranges'] = azimuth_ranges
    param_dict['elevation_ranges'] = elevation_ranges
    return param_dict


def build_vector_list(params):
    """
    From sonar parameters, build the wedge as list of vectors
    """
    xx = []
    yy = []
    zz = []

    # Calculate ending arc of the FOV in 3D space
    for elevation in params['elevations']:
        ce = np.cos(elevation)
        se = np.sin(elevation)
        azimuth = np.arctan2(-1 * params['azimuth_ranges'],
                             params['elevation_ranges'])
        ca = np.cos(azimuth)
        sa = np.sin(azimuth)

        x = params['range'] * ce * ca
        y = -1 * params['range'] * ce * sa
        z = np.full_like(x, fill_value=params['range'] * se)
        xx.extend(x)
        yy.extend(y)
        zz.extend(z)

    # Add a point at the origin
    xx_new = np.insert(np.array(xx), 0, 0)
    yy_new = np.insert(np.array(yy), 0, 0)
    zz_new = np.insert(np.array(zz), 0, 0)

    # Vertices are all the points on the edge of the wedge
    # Points are at the origin and then at the end of every arc
    vertices = np.zeros(shape=(len(xx_new), 3))
    vertices[:, 0] = xx_new
    vertices[:, 1] = yy_new
    vertices[:, 2] = zz_new

    # Number of faces:
    vertex_cnt = len(xx_new) + 1
    face_cnt = (vertex_cnt // 2 - 2)
    split = (len(xx_new) - 1) // 2

    pt0 = np.zeros(shape=(face_cnt), dtype=np.uint)
    pt1 = np.arange(1, face_cnt + 1, dtype=np.uint)
    pt2 = pt1 + 1
    pt3 = pt2 + face_cnt
    pt4 = pt3 + 1
    top_face = np.stack([pt0, pt1, pt2]).T
    btm_face = np.stack([pt4, pt3, pt0]).T

    # Add triangles on sides:
    left_face = np.array([[0, split + 1, 1]], dtype=np.uint)
    right_face = np.array([[0, split, len(xx_new) - 1]], dtype=np.uint)

    # Add triangles between elevations:
    first_pt = np.arange(1, split, dtype=np.uint)
    second_pt = first_pt + 1
    third_pt = second_pt + split - 1
    fourth_pt = third_pt + 1

    first_connection = np.stack([first_pt, third_pt, fourth_pt]).T
    second_connection = np.stack([first_pt, fourth_pt, second_pt]).T
    elevation_faces = np.vstack([first_connection, second_connection])

    faces = np.vstack(
        [top_face, btm_face, left_face, right_face, elevation_faces])

    # Create array of vectors
    vectors = []
    for f in faces:
        for j in range(3):
            x, y, z = vertices[f[j], :]
            pt = Point(x, y, z)
            vectors.append(pt)

    return vectors


class SonarFOV():

    def __init__(self):
        self.sub = rospy.Subscriber("sonar_image",
                                    ProjectedSonarImage,
                                    self.callback,
                                    queue_size=1,
                                    buff_size=1000000)
        self.pub_fov = rospy.Publisher('/sonar_fov',
                                       MarkerArray,
                                       queue_size=10)

        # Alpha transparency for wedge
        self.alpha = rospy.get_param("~alpha", 1)
        # RGB color for wedge
        self.color = rospy.get_param("~color", [0.0, 1.0, 0.0])

        self.vector_list = None
        self.generate_fov = False
        self.sonar_params = None

    def generate_marker_array(self):
        """
        Load the FOV wedge from an STL
        Sets the color and alpha from rosparams
        """
        global stl_name

        obj = Marker()
        obj.type = Marker.TRIANGLE_LIST
        obj.id = 1
        obj.points = self.vector_list
        obj.frame_locked = True

        obj.scale.x = 1.0
        obj.scale.y = 1.0
        obj.scale.z = 1.0

        obj.color.r = self.color[0]
        obj.color.g = self.color[1]
        obj.color.b = self.color[2]
        obj.color.a = self.alpha

        self.world = MarkerArray()
        self.world.markers = [obj]

    def callback(self, image_msg: ProjectedSonarImage):
        """
        Callback to publish the marker array containing the FOV wedge.
        Only updates the vectors if parameters change
        """
        rospy.logdebug("Received new image, seq %d at %f" %
                       (image_msg.header.seq, image_msg.header.stamp.to_sec()))

        if self.sonar_params is None:
            self.sonar_params = sonar_params(image_msg)
            self.generate_fov_flag = True  # generate the fov stl
        else:
            new_params = sonar_params(image_msg)
            # Because the dictionary contains numpy arrays, a simple check for a == b does not work.
            # Using allclose because the range occasionally changes by fractions
            message_equality = [
                np.allclose(self.sonar_params[key], new_params[key], atol=1e-2)
                for key in self.sonar_params.keys()
            ]

            if not np.all(message_equality):
                # things have changed, generate the fov stl
                rospy.logwarn("Updating Parameters of FOV")
                self.generate_fov_flag = True
            else:
                # No change in parameters,
                # no need to regenerate the fov stl
                self.generate_fov_flag = False

        if self.generate_fov_flag:
            rospy.logdebug("Generating FOV mesh...")
            self.vector_list = build_vector_list(self.sonar_params)

        header = Header()
        header = image_msg.header

        frame_id = rospy.get_param("~frame_id", None)
        if frame_id:
            header.frame_id = frame_id
        self.generate_marker_array()
        for obj in self.world.markers:
            obj.header = image_msg.header
        self.pub_fov.publish(self.world)


if __name__ == "__main__":
    rospy.init_node("sonar_pointcloud")
    fov_publisher = SonarFOV()
    rospy.spin()
