#! /usr/bin/env python3
"""
Copyright 2022 University of Washington Applied Physics Laboratory
Author: Marc Micatka & Laura Lindzey
"""

from __future__ import annotations  # use type of class in member function annotation.
from matplotlib import cm
import numpy as np
import rospy
import time
import typing
import pickle

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

        # threshold range is a float, [0-1]
        self.intensity_threshold = rospy.get_param("~intensity_threshold",
                                                   0.74)
        # x,y,z coordinates of points
        # [ELEVATION_IDX, INTENSITY_IDX, DIMENSION_IDX]
        # Shape chosen for ease of mapping coordinates to intensities
        self.geometry = None
        # We need to detect whether the geometry has changed.
        self.sonar_msg_metadata = None

        self.color_lookup = None
        self.output_points = None

        self.save_pickle = True

    def make_geometry(self):
        rospy.loginfo("make_geometry")
        # points = [[[0, 0, 0]
        #            for _ in range(self.sonar_msg_metadata.num_ranges *
        #                           self.sonar_msg_metadata.num_angles)]
        #           for _ in range(len(self.sonar_msg_metadata.elevations))]
        t0 = time.time()
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
        dt = time.time() - t0
        rospy.logerr("Creating geometry took {:0.2f} ms".format(1000 * dt))

    def make_color_lookup(self):
        self.color_lookup = np.zeros(shape=(256, 4))

        for aa in range(256):
            r, g, b, _ = cm.inferno(aa)
            alpha = (aa / 256)

            self.color_lookup[aa, :] = [r, g, b, alpha]

    def process_intensity_array(self, image: SonarImageData):
        '''
        process an intensity array into a parseable format for pointcloud generation
        Can handle 8bit or 32bit input data, will log scale output data
        '''
        if image.dtype == image.DTYPE_UINT8:
            data_type = np.uint8
        elif image.dtype == image.DTYPE_UINT32:
            data_type = np.uint32
        else:
            raise Exception("Only 8 bit and 32 bit data is supported!")

        intensities = np.frombuffer(image.data, dtype=data_type)
        # Log scaling modified from sonar_postprocessor_nodelet.cpp
        # Avoid log(0)
        new_intensites = intensities.astype(np.float32) + 1e-6
        # NOTE(lindzey): This calculation needs more comments.
        v = np.log(new_intensites) / np.log(np.iinfo(data_type).max)
        vmax = 1.0
        v = (v - self.intensity_threshold) / (vmax - self.intensity_threshold)

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

        image_metadata = SonarImageMetadata(sonar_image_msg)
        if self.sonar_msg_metadata is None or not self.sonar_msg_metadata.equals(
                image_metadata):
            print("Metadata updated! \nOld: {} \nNew: {}".format(
                self.sonar_msg_metadata, image_metadata))
            self.sonar_msg_metadata = image_metadata
            self.make_geometry()

        if self.color_lookup is None:
            self.make_color_lookup()
        t0 = time.time()

        # np.ndarray, shape = (npts,)
        intensities = self.process_intensity_array(sonar_image_msg.image)

        # Make a copy of geometry, if not, indexing into positive values will
        # change the array after one iteration
        # QUESTION(lindzey): I'm confused by the above statement, given that
        #     geometry is immediately assigned to a slice of self.geometry
        geometry = self.geometry.copy()

        # QUESTION(lindzey): I could really use some comments on what a
        #     negative intensity means at this point, and why we effectively filter
        #     by intensity twice. Once here, and once when setting points[:, 3:]
        if not self.publish_all_points:
            pos_intensity_idx = np.where(intensities > 0)
            thresholded_intensities = intensities[pos_intensity_idx]
            geometry = self.geometry[:, pos_intensity_idx[0]]

        npts = len(thresholded_intensities)

        # Allocate our output points
        self.output_points = np.zeros(
            (len(self.sonar_msg_metadata.elevations) * npts, 7),
            dtype=np.float32)

        # Expand out intensity array (for fast comparison)
        # The np.where call setting colors requires an array of shape (npts, 4).
        # thresholded_intensities has shape (npts,), but np.repeat requires (npts, 1).
        expanded_intensities = np.repeat(thresholded_intensities[...,
                                                                 np.newaxis],
                                         4,
                                         axis=1)
        # NOTE(lindzey): Intensity doesn't change as a function of elevation,
        #    so I moved this to be computed outside of the elevations loop.
        #    (Since we're using numpy arrays, we don't need to reallocate
        #    points every time.)
        elev_points = np.empty((npts, 7))
        elev_points[:, 3:] = np.where(
            expanded_intensities > self.intensity_threshold,
            self.color_lookup[expanded_intensities[:, 0]], np.zeros((npts, 4)))

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
