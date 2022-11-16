#! /usr/bin/env python3
import os
import rospy
import time
import rospkg

import numpy as np
import open3d as o3d

from stl import mesh
from std_srvs.srv import Empty
from std_msgs.msg import Header, Float32
from acoustic_msgs.msg import ProjectedSonarImage, SonarImageData, PingInfo
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker, MarkerArray


r = rospkg.RosPack()
path = r.get_path('sonar_image_proc')
stl_name = "sonar_fov.stl"
temp_mesh_file = os.path.join(path, "scripts", stl_name)


def sonar_params(image_msg):

    param_dict = {}
    ranges = image_msg.ranges
    azimuth_ranges = [image_msg.beam_directions[0].y,
                      image_msg.beam_directions[-1].y]

    azimuth_start = min(azimuth_ranges)
    azimuth_end = max(azimuth_ranges)
    num_azimuth_steps = len(image_msg.beam_directions)

    azimuth_steps = np.linspace(
        azimuth_start, azimuth_end, num_azimuth_steps)
    azimuth_steps[azimuth_steps < 0] += 2*np.pi

    elevation_ranges = [beam_dir.z for beam_dir in image_msg.beam_directions]
    elevation_start = np.arccos(max(elevation_ranges))
    elevation_end = np.arccos(min(elevation_ranges))

    print(elevation_start, elevation_end)
    elevation_steps = 2
    elevations = np.linspace(
        elevation_start, elevation_end, elevation_steps)
    elevations = elevations + np.pi/2

    param_dict['range'] = max(ranges)
    param_dict['elevations'] = elevations
    param_dict['azimuth_steps'] = azimuth_steps
    return param_dict


def build_stl_mesh(params):
    """
    From sonar parameters, build the wedge
    Save it in a temp file that gets removed after each run (or if parameters change mid-run)
    """
    global temp_mesh_file

    xx = []
    yy = []
    zz = []

    # Calculate points in 3d space
    for elevation in params['elevations']:
        x = params['range'] * np.sin(elevation) * \
            np.cos(params['azimuth_steps'])
        y = params['range'] * np.sin(elevation) * \
            np.sin(params['azimuth_steps'])
        z = np.full_like(x, fill_value=params['range']*np.cos(elevation))
        xx.extend(x)
        yy.extend(y)
        zz.extend(z)

    xx_new = np.insert(np.array(xx), 0, 0)
    yy_new = np.insert(np.array(yy), 0, 0)
    zz_new = np.insert(np.array(zz), 0, 0)

    vertices = np.zeros(shape=(len(xx_new), 3))
    vertices[:, 0] = xx_new
    vertices[:, 1] = yy_new
    vertices[:, 2] = zz_new

    # First, connect all the wedges on each face:
    split = (len(xx_new) - 1)//2
    first_pt = np.zeros(shape=(len(xx_new) - 2), dtype=np.uint)
    second_pt = np.arange(1, len(xx_new) - 1, dtype=np.uint)
    third_pt = np.arange(2, len(xx_new), dtype=np.uint)
    first_faces = np.stack([first_pt, second_pt, third_pt]).T

    # Delete the cross-over wedge:
    first_faces = np.delete(first_faces, (split - 1), axis=0)

    # Add triangles between elevations:
    first_pt = np.arange(1, split, dtype=np.uint)
    second_pt = first_pt + 1
    third_pt = second_pt + split - 1
    fourth_pt = third_pt + 1

    first_connection = np.stack([first_pt, second_pt, third_pt]).T
    second_connection = np.stack([second_pt, third_pt, fourth_pt]).T
    second_faces = np.vstack([first_connection, second_connection])

    faces = np.vstack([first_faces, second_faces])

    # Add triangles on sides:
    side_faces = np.array([[0, 1, split + 1],
                           [0, split, (len(xx_new) - 1)]], dtype=np.uint)

    faces = np.vstack([faces, side_faces])

    # Create the mesh
    wedge = mesh.Mesh(np.zeros(faces.shape[0], dtype=mesh.Mesh.dtype))
    for i, f in enumerate(faces):
        for j in range(3):
            wedge.vectors[i][j] = vertices[f[j], :]

    # Write the mesh to file
    wedge.save(temp_mesh_file)
    print(f"Saving fov to {temp_mesh_file}")


def clean_up_mesh():
    """
    Delete temporary mesh file when program closes
    """
    global temp_mesh_file
    os.remove(temp_mesh_file)


class SonarFOV():

    def __init__(self):
        # NB: if queue_size is set, have to be sure buff_size is sufficiently large,
        #     since it can only discard messages that are fully in the buffer.
        # https://stackoverflow.com/questions/26415699/ros-subscriber-not-up-to-date
        # (200k didn't work, and sys.getsizeof doesn't return an accurate size of the whole object)
        self.sub = rospy.Subscriber("sonar_image",
                                    ProjectedSonarImage,
                                    self.callback,
                                    queue_size=1,
                                    buff_size=1000000)
        self.pub_fov = rospy.Publisher('/sonar_fov',
                                       MarkerArray,
                                       queue_size=10)

        self.generate_fov = False
        self.sonar_params = None

    def generate_marker_array(self):
        # This is the FOV object
        global stl_name

        obj = Marker()
        obj.type = Marker.MESH_RESOURCE
        obj.id = 1
        obj.mesh_resource = f"package://sonar_image_proc/scripts/{stl_name}"
        obj.frame_locked = True

        # obj.pose.position.x = 0
        # obj.pose.position.y = 0
        # obj.pose.position.z = 0
        # obj.pose.orientation.y = 0
        # obj.pose.orientation.w = 0

        obj.scale.x = 1.0
        obj.scale.y = 1.0
        obj.scale.z = 1.0

        obj.color.r = 1.0
        obj.color.g = 1.0
        obj.color.b = 1.0
        obj.color.a = 1.0

        self.world = MarkerArray()
        self.world.markers = [obj]

    def callback(self, image_msg: ProjectedSonarImage):
        """
        Generate FOV wedge based on parameters
        """
        rospy.logdebug("Received new image, seq %d at %f" %
                       (image_msg.header.seq, image_msg.header.stamp.to_sec()))

        if self.sonar_params is None:
            self.sonar_params = sonar_params(image_msg)
            self.generate_fov_flag = True  # generate the fov stl
        else:
            new_params = sonar_params(image_msg)
            dict_equality = np.all([np.all(self.sonar_params[key] == new_params[key])
                                   for key in self.sonar_params.keys()])
            if not dict_equality:
                rospy.logdebug("Updating Parameters of FOV")
                self.generate_fov_flag = True  # generate the fov stl
            else:
                self.generate_fov_flag = False  # no need to generate the fov stl

        if self.generate_fov_flag:
            rospy.logdebug("Generating FOV mesh")
            build_stl_mesh(self.sonar_params)

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
    rospy.on_shutdown(clean_up_mesh)

    fov_publisher = SonarFOV()
    rospy.spin()
