#! /usr/bin/env python3
"""
Copyright 2023 University of Washington Applied Physics Laboratory
Author: Marc Micatka & Laura Lindzey
"""
from __future__ import annotations  # use type of class in member function annotation.
import numpy as np
import rospy

from acoustic_msgs.msg import ProjectedSonarImage
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from sonar_image_proc.sonar_msg_metadata import SonarImageMetadata


def build_vector_list(msg_metadata):
    """
    From sonar parameters, build the wedge as list of vectors
    """
    xx = []
    yy = []
    zz = []

    # Calculate ending arc of the FOV in 3D space
    for elevation in [msg_metadata.min_elevation, msg_metadata.max_elevation]:
        ce = np.cos(elevation)
        se = np.sin(elevation)
        ca = np.cos(msg_metadata.azimuths)
        sa = np.sin(msg_metadata.azimuths)

        z = msg_metadata.max_range * ce * ca
        y = -1 * msg_metadata.max_range * ce * sa
        x = np.full_like(z, fill_value=msg_metadata.max_range * se)
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

    # The vertices, faces, and connections are directional in nature
    # Because the mesh is one-sided and will be invisible certain directions
    # if the order is incorrect

    # Number of faces:
    vertex_cnt = len(xx_new) + 1
    face_cnt = vertex_cnt // 2 - 2
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

    faces = np.vstack([top_face, btm_face, left_face, right_face, elevation_faces])

    # Create array of vectors
    vectors = []
    for f in faces:
        for j in range(3):
            x, y, z = vertices[f[j], :]
            pt = Point(x, y, z)
            vectors.append(pt)
    return vectors


class SonarFOV:
    def __init__(self):
        self.sub = rospy.Subscriber(
            "sonar_image",
            ProjectedSonarImage,
            self.sonar_image_callback,
            queue_size=1,
            buff_size=1000000,
        )
        self.pub_fov = rospy.Publisher("sonar_fov", MarkerArray, queue_size=10)

        # Alpha transparency for wedge
        self.alpha = rospy.get_param("~alpha", 1.0)
        if isinstance(self.alpha, str):
            self.alpha = float(self.alpha)

        # RGB color for wedge
        self.color = rospy.get_param("~color", [0.0, 1.0, 0.0])
        if isinstance(self.color, str):
            self.color = eval(self.color)

        self.vector_list = None
        self.sonar_msg_metadata = None

    def generate_marker_array(self, vector_list) -> MarkerArray:
        """
        Create MarkerArray from the pre-computed FOV wedge.
        Sets the color and alpha from rosparams
        """
        obj = Marker()
        obj.type = Marker.TRIANGLE_LIST
        obj.id = 1
        obj.points = vector_list
        obj.frame_locked = True

        obj.scale.x = 1.0
        obj.scale.y = 1.0
        obj.scale.z = 1.0

        obj.color.r = self.color[0]
        obj.color.g = self.color[1]
        obj.color.b = self.color[2]
        obj.color.a = self.alpha

        wedge = MarkerArray()
        wedge.markers = [obj]
        return wedge

    def sonar_image_callback(self, sonar_image_msg: ProjectedSonarImage):
        """
        Callback to publish the marker array containing the FOV wedge.
        Only updates the geometry if parameters change.
        """
        rospy.logdebug(
            "Received new image, seq %d at %f"
            % (sonar_image_msg.header.seq, sonar_image_msg.header.stamp.to_sec())
        )

        # For the sonar_fov script, the elevation steps need to be 2
        # The logic to draw the mesh is not generic to step counts
        new_metadata = SonarImageMetadata(sonar_image_msg)
        generate_fov_flag = False
        if self.sonar_msg_metadata is None:
            self.sonar_msg_metadata = new_metadata
            generate_fov_flag = True  # generate the fov stl
        else:
            # Because the dictionary contains numpy arrays, a simple check for a == b does not work.
            # Using allclose because the range occasionally changes by fractions
            if (
                self.sonar_msg_metadata is None
                or self.sonar_msg_metadata != new_metadata
            ):
                # things have changed, regenerate the fov
                rospy.logdebug("Updating Parameters of FOV")
                self.sonar_msg_metadata = new_metadata
                generate_fov_flag = True

        if generate_fov_flag:
            rospy.logdebug("Generating FOV mesh...")
            self.vector_list = build_vector_list(self.sonar_msg_metadata)

        wedge = self.generate_marker_array(self.vector_list)
        for obj in wedge.markers:
            obj.header = sonar_image_msg.header
        self.pub_fov.publish(wedge)


if __name__ == "__main__":
    rospy.init_node("sonar_fov")
    fov_publisher = SonarFOV()
    rospy.spin()
