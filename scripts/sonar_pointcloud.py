#! /usr/bin/env python3
"""
Copyright 2023 University of Washington Applied Physics Laboratory
Author: Marc Micatka & Laura Lindzey
"""

from __future__ import annotations  # use type of class in member function annotation.
from matplotlib import cm
import numpy as np
import rospy
import time

from acoustic_msgs.msg import ProjectedSonarImage, SonarImageData
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from sonar_image_proc.sonar_msg_metadata import SonarImageMetadata


class SonarTranslator(object):

    def __init__(self):
        # NB: if queue_size is set, have to be sure buff_size is sufficiently large,
        #     since it can only discard messages that are fully in the buffer.
        # https://stackoverflow.com/questions/26415699/ros-subscriber-not-up-to-date
        # (200k didn't work, and sys.getsizeof doesn't return an accurate size of the whole object)
        self.sub = rospy.Subscriber("sonar_image",
                                    ProjectedSonarImage,
                                    self.sonar_image_callback,
                                    queue_size=1,
                                    buff_size=1000000)
        self.pub = rospy.Publisher("sonar_cloud", PointCloud2, queue_size=1)

        # Flag to determine whether we publish ALL points or only non-zero points
        self.publish_all_points = rospy.get_param("~publish_all_points", False)

        # threshold range is a float, [0-1] which scales the pointcloud to
        # between intensity_threshold and 1
        self.intensity_threshold = rospy.get_param("~intensity_threshold", 0)

        # x,y,z coordinates of points
        # [ELEVATION_IDX, INTENSITY_IDX, DIMENSION_IDX]
        # Shape chosen for ease of mapping coordinates to intensities
        self.geometry = None

        # We need to detect whether the geometry has changed.
        self.sonar_msg_metadata = None

        self.color_lookup = None
        self.output_points = None

    def make_geometry(self):
        """
        Vectorized geometry generation. 
        Regenerates when there are changing parameters from the sonar.
        """
        rospy.loginfo("Making Geometry")
        begin_time = time.time()
        # Pre-compute these values to speed up the loop
        # Compute the idxs to properly index into the points array
        idxs = np.arange(
            0, self.sonar_msg_metadata.num_angles *
            self.sonar_msg_metadata.num_ranges)
        idxs = idxs.reshape(
            self.sonar_msg_metadata.num_ranges,
            self.sonar_msg_metadata.num_angles).flatten(order='F')

        # Compute the cosines and sines of elevations and azimuths
        ces = np.cos(self.sonar_msg_metadata.elevations)
        ses = np.sin(self.sonar_msg_metadata.elevations)
        cas = np.cos(self.sonar_msg_metadata.azimuths)
        sas = np.sin(self.sonar_msg_metadata.azimuths)

        # Allocate points
        new_shape = (len(self.sonar_msg_metadata.elevations),
                     self.sonar_msg_metadata.num_ranges *
                     self.sonar_msg_metadata.num_angles, 3)
        points = np.zeros(shape=(new_shape))

        x_temp = np.tile(self.sonar_msg_metadata.ranges[np.newaxis, :] *
                         ses[:, np.newaxis],
                         reps=self.sonar_msg_metadata.num_angles).flatten()
        y_temp = (self.sonar_msg_metadata.ranges[np.newaxis, np.newaxis, :] *
                  ces[:, np.newaxis, np.newaxis] *
                  sas[np.newaxis, :, np.newaxis]).flatten()
        z_temp = (self.sonar_msg_metadata.ranges[np.newaxis, np.newaxis, :] *
                  ces[:, np.newaxis, np.newaxis] *
                  cas[np.newaxis, :, np.newaxis]).flatten()

        points[:, idxs, :] = np.stack([x_temp, y_temp, z_temp],
                                      axis=1).reshape(new_shape)

        self.geometry = points
        total_time = time.time() - begin_time
        rospy.loginfo(f"Creating geometry took {1000*total_time:0.2f} ms")

    def make_color_lookup(self):
        """
        Generates a lookup table from matplotlib inferno colormapping
        """
        self.color_lookup = np.zeros(shape=(256, 4))

        for aa in range(256):
            r, g, b, _ = cm.inferno(aa)
            alpha = (aa / 256)

            self.color_lookup[aa, :] = [r, g, b, alpha]

    def process_intensity_array(self, image: SonarImageData):
        """
        Process an intensity array into a parseable format for pointcloud generation
        Can handle 8bit or 32bit input data, will log scale output data
        """
        if image.dtype == image.DTYPE_UINT8:
            data_type = np.uint8
        elif image.dtype == image.DTYPE_UINT32:
            data_type = np.uint32
        else:
            raise Exception("Only 8 bit and 32 bit data is supported!")

        intensities = np.frombuffer(image.data, dtype=data_type)
        new_intensites = intensities.astype(np.float32)
        # Scaling calculation is taken from sonar_image_proc/ros/src/sonar_postprocessor_nodelet.cpp
        # First we log scale the intensities

        # Avoid log(0) and add a small offset
        v = np.log(new_intensites + 1e-6) / np.log(np.iinfo(data_type).max)

        # Then we shift the intensity range by intensity_offset and scale by intensity_scaling
        # In the original implementation, the scaling is done by (vmax - intensity_offset)
        intensity_offset = 0.74
        vmax = 1.0
        intensity_scaling = vmax - intensity_offset
        v = (v - intensity_offset) / intensity_scaling
        # Finally we clip to 0 to 1.
        # No values shoule be above 1, but plenty of values are below zero.

        v = np.clip(v, a_min=0.0, a_max=1.0)
        intensities = (np.iinfo(np.uint8).max * v).astype(np.uint8)

        return intensities

    def sonar_image_callback(self, sonar_image_msg: ProjectedSonarImage):
        """
        Convert img_msg into point cloud with color mappings via numpy.
        """
        begin_time = time.time()
        rospy.logdebug("Received new image, seq %d at %f" %
                       (sonar_image_msg.header.seq,
                        sonar_image_msg.header.stamp.to_sec()))
        header = Header()
        header = sonar_image_msg.header

        frame_id = rospy.get_param("~frame_id", None)
        if frame_id:
            header.frame_id = frame_id

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('r', 12, PointField.FLOAT32, 1),
            PointField('g', 16, PointField.FLOAT32, 1),
            PointField('b', 20, PointField.FLOAT32, 1),
            PointField('a', 24, PointField.FLOAT32, 1)
        ]

        new_params = SonarImageMetadata(sonar_image_msg)
        if self.sonar_msg_metadata is None or self.sonar_msg_metadata != new_params:
            print("Metadata updated! \nOld: {} \nNew: {}".format(
                self.sonar_msg_metadata, new_params))
            self.sonar_msg_metadata = new_params
            self.make_geometry()

        if self.color_lookup is None:
            self.make_color_lookup()
        t0 = time.time()

        # np.ndarray, shape = (npts,)
        intensities = self.process_intensity_array(sonar_image_msg.image)

        # QUESTION(lindzey): I could really use some comments on what a
        #     negative intensity means at this point, and why we effectively filter
        #     by intensity twice. Once here, and once when setting points[:, 3:]

        # If you're not publishing all values (if publish_all_points is false)
        # then the pointcloud is masked and only values above the threshold value are published\
        if self.publish_all_points:
            # uint8_threshold = 0
            selected_intensities = intensities
            geometry = self.geometry
        else:
            uint8_threshold = self.intensity_threshold * np.iinfo(np.uint8).max
            pos_intensity_idx = np.where(intensities > uint8_threshold)
            selected_intensities = intensities[pos_intensity_idx]
            geometry = self.geometry[:, pos_intensity_idx[0]]

            num_points = np.sum(intensities > uint8_threshold)
            rospy.logdebug(
                f"Filtering Results: (Pts>Thresh, Total): {num_points, intensities.size}. Frac: {(num_points/intensities.size):.3f}"
            )

        npts = len(selected_intensities)

        # Allocate our output points
        self.output_points = np.zeros(
            (len(self.sonar_msg_metadata.elevations) * npts, 7),
            dtype=np.float32)

        # Expand out intensity array (for fast comparison)
        # The np.where call setting colors requires an array of shape (npts, 4).
        # selected_intensities has shape (npts,), but np.repeat requires (npts, 1).
        expanded_intensities = np.repeat(selected_intensities[..., np.newaxis],
                                         4,
                                         axis=1)
        # NOTE(lindzey): Intensity doesn't change as a function of elevation,
        #    so I moved this to be computed outside of the elevations loop.
        #    (Since we're using numpy arrays, we don't need to reallocate
        #    points every time.)
        elev_points = np.empty((npts, 7))
        elev_points[:, 3:] = self.color_lookup[expanded_intensities[:, 0]]

        # Fill the output array
        for i in range(len(self.sonar_msg_metadata.elevations)):
            elev_points[:, 0:3] = geometry[i, :, :]
            step = i * npts
            next_step = step + npts
            self.output_points[step:next_step, :] = elev_points

        t1 = time.time()
        N = len(self.output_points)
        cloud_msg = PointCloud2(header=header,
                                height=1,
                                width=N,
                                is_dense=True,
                                is_bigendian=False,
                                fields=fields,
                                point_step=7 * 4,
                                row_step=7 * 4 * N,
                                data=self.output_points.tobytes())

        dt1 = time.time() - t1
        dt0 = t1 - t0
        total_time = time.time() - begin_time
        self.pub.publish(cloud_msg)

        rospy.logdebug(
            f"published pointcloud: npts = {npts}, Find Pts = {dt0:0.5f} sec, "
            f"Convert to Cloud = {dt1:0.5f} sec. "
            f"Total Time = {total_time:0.3f} sec")


if __name__ == "__main__":
    rospy.init_node("sonar_pointcloud")
    translator = SonarTranslator()
    rospy.spin()
