#! /usr/bin/env python3

import rospy
import struct
import time

from matplotlib import cm
import numpy as np

from acoustic_msgs.msg import SonarImage
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs import point_cloud2


class SonarTranslator(object):

    def __init__(self):
        # NB: if queue_size is set, have to be sure buff_size is sufficiently large,
        #     since it can only discard messages that are fully in the buffer.
        # https://stackoverflow.com/questions/26415699/ros-subscriber-not-up-to-date
        # (200k didn't work, and sys.getsizeof doesn't return an accurate size of the whole object)
        self.sub = rospy.Subscriber("sonar_image",
                                    SonarImage,
                                    self.callback_fast,
                                    queue_size=1,
                                    buff_size=1000000)
        self.pub = rospy.Publisher("sonar_cloud", PointCloud2, queue_size=1)

        min_elev_deg = rospy.get_param("~min_elev_deg", 0)
        max_elev_deg = rospy.get_param("~max_elev_deg", 20)
        assert (max_elev_deg >= min_elev_deg)
        elev_step_deg = rospy.get_param("~elev_step_deg", 10.0)
        self.min_elev = np.radians(min_elev_deg)
        self.max_elev = np.radians(max_elev_deg)
        self.elev_step = np.radians(elev_step_deg)
        self.elevations = np.arange(self.min_elev,
                                    self.max_elev + self.elev_step,
                                    self.elev_step)

        # TODO: assert that this is in range of uint8? (Or, msg.data_size)
        self.intensity_threshold = rospy.get_param("~intensity_threshold", 100)

        # x,y,z coordinates of points
        # [ELEVATION_IDX, INTENSITY_IDX, DIMENSION_IDX]
        # Shape chosen for ease of mapping coordinates to intensities
        self.geometry = None
        self.geometry_np = None
        self.intensity_lookup = None
        self.intensity_lookup_np = None
        self.color_lookup = None
        self.rgba_zero = struct.unpack('I', struct.pack('BBBB', 0, 0, 0,
                                                        255))[0]

    # TODO: This assumes that the intensity data is single byte
    def make_intensity_lookup(self):
        rospy.loginfo("make_intensity_lookup")
        self.intensity_lookup = [0 for _ in range(256)]

        for aa in range(256):
            r, g, b, _ = cm.inferno(aa)
            rr = int(255 * r)
            gg = int(255 * g)
            bb = int(255 * b)
            rgba = struct.unpack('I', struct.pack('BBBB', bb, gg, rr, aa))[0]
            self.intensity_lookup[aa] = rgba

        self.intensity_lookup_np = np.array(self.intensity_lookup)

    def make_color_lookup(self):
        self.color_lookup = np.zeros(shape=(256, 3))

        for aa in range(256):
            r, g, b, _ = cm.inferno(aa)
            self.color_lookup[aa, :] = [r, g, b]

    def make_geometry(self, image_msg):
        rospy.loginfo("make_geometry")
        nranges = len(image_msg.ranges)
        nangles = len(image_msg.azimuth_angles)
        points = [[[0, 0, 0] for _ in range(nranges * nangles)]
                  for _ in range(len(self.elevations))]

        for kk, elevation in enumerate(self.elevations):
            ce = np.cos(elevation)
            se = np.sin(elevation)
            for ii, azimuth in enumerate(image_msg.azimuth_angles):
                # Pre-compute these values to speed up the loop
                ca = np.cos(azimuth)
                sa = np.sin(azimuth)
                for jj, distance in enumerate(image_msg.ranges):
                    idx = ii + jj * nangles
                    xx = distance * ce * ca
                    yy = distance * ce * sa
                    zz = distance * se
                    points[kk][idx] = [xx, yy, zz]

        self.geometry = points
        self.geometry_np = np.array(points)

    def callback(self, image_msg):

        rospy.logdebug("Received new image, seq %d at %f" %
                       (image_msg.header.seq, image_msg.header.stamp.to_sec()))

        header = Header()
        header = image_msg.header

        frame_id = rospy.get_param("~frame_id", None)
        if frame_id:
            header.frame_id = frame_id

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1)
        ]

        t0 = time.time()
        nranges = len(image_msg.ranges)
        nangles = len(image_msg.azimuth_angles)
        npts = nranges * nangles / image_msg.data_size

        # Trying to get the types right slowed it back down to 400 ms / msg
        # points = [[np.float32(0.0), np.float32(0.0), np.float32(0.0), np.int32(0)] for _ in range(npts)]
        # TODO: Would be faster to pre-compute the geometry for the FIRST message,
        #       then just look it up after that.
        # TODO: Consider using a different color map? (in addition to the intensity data)
        # TODO: Add range of elevations.

        if self.geometry is None:
            self.make_geometry(image_msg)
        if self.intensity_lookup is None:
            self.make_intensity_lookup()

        if image_msg.data_size == 1:
            intensities = np.frombuffer(image_msg.intensities, dtype=np.uint8)
        if image_msg.data_size == 4:
            intensities = np.frombuffer(image_msg.intensities, dtype=np.uint32)
            # intensities = np.log(intensities + 1e-6)
            intensities = (intensities / 256).astype(np.uint8)
        print(intensities.max(), intensities.mean())
        # Construction of PointCloud2 cribbed from:
        # https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
        points = []

        return_idxs, = np.where(
            np.array([int(ii)
                      for ii in intensities]) > self.intensity_threshold)

        rospy.logwarn("{} (out of {}) points are above threshold".format(
            len(return_idxs), len(intensities)))

        for geometry in self.geometry:
            pts = [[
                xx, yy, zz,
                self.intensity_lookup_np[aa - self.intensity_threshold]
                if aa > self.intensity_threshold else 0
            ] for (xx, yy, zz), aa in zip(geometry, intensities)]
            points.extend(pts)

        t1 = time.time()
        cloud_msg = point_cloud2.create_cloud(header, fields, points)
        dt0 = t1 - t0
        dt1 = time.time() - t1

        total_time = time.time()
        self.pub.publish(cloud_msg)

        rospy.loginfo(
            f"published pointcloud: npts = {npts}, Find Pts = {dt0:0.3f}, Convert to Cloud = {dt1:0.3f}. Total Time = {(total_time - t0):0.3f}"
        )

    def callback_fast(self, image_msg):
        """
        Convert img_msg into point cloud with intensity mappings.
        """
        header = Header()
        header = image_msg.header

        # If specified, rewrite the frame in th
        frame_id = rospy.get_param("~frame_id", None)
        if frame_id:
            header.frame_id = frame_id

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgba', 12, PointField.UINT32, 1)
        ]
        t0 = time.time()
        nranges = len(image_msg.ranges)
        nangles = len(image_msg.azimuth_angles)
        npts = nranges * nangles

        if self.geometry is None:
            self.make_geometry(image_msg)

        if self.color_lookup is None:
            self.make_color_lookup()

        if self.intensity_lookup is None:
            self.make_intensity_lookup()

        # Construction of PointCloud2 from: https://gist.github.com/pgorczak/5c717baa44479fa064eb8d33ea4587e0
        if image_msg.data_size == 1:
            intensities = np.frombuffer(image_msg.intensities, dtype=np.uint8)
        if image_msg.data_size == 4:
            intensities = np.frombuffer(image_msg.intensities, dtype=np.uint32)
            # intensities = np.log(intensities)
            intensities = (intensities / 256).astype(np.uint8)

        output_points = np.empty((len(self.elevations) * nranges * nangles, 4))
        # expanded_intensities = np.repeat(intensities[..., np.newaxis],
        #                                  3,
        #                                  axis=1)

        for i in range(len(self.elevations)):
            points = np.empty((nranges * nangles, 4))
            if points[:, 0:3].shape != self.geometry_np[i, :, :].shape:
                break
            points[:, 0:3] = self.geometry_np[i, :, :]
            # print(points.shape, intensities.shape, self.color_lookup.shape)
            points[:, 3] = np.where(
                intensities > self.intensity_threshold,
                self.intensity_lookup_np[intensities -
                                         self.intensity_threshold], 0)
            # points[:, 3:] = np.where(
            #     expanded_intensities > self.intensity_threshold,
            #     self.intensity_lookup_np[expanded_intensities[:, 0]],
            #     np.zeros((nranges * nangles, 3)))

            step = i * (nranges * nangles)
            next_step = step + (nranges * nangles)
            output_points[step:next_step, :] = points

        print(
            'lookup',
            hex(self.intensity_lookup_np[intensities[0] -
                                         self.intensity_threshold]))
        print(intensities.max(), intensities.mean())
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        data = output_points.astype(dtype).tobytes()
        t1 = time.time()
        cloud_msg = PointCloud2(header=header,
                                height=1,
                                width=output_points.shape[0],
                                is_dense=False,
                                is_bigendian=False,
                                fields=fields,
                                point_step=(itemsize * 4),
                                row_step=(itemsize * 4 *
                                          output_points.shape[0]),
                                data=data)

        dt0 = t1 - t0
        dt1 = time.time() - t1
        total_time = time.time()
        self.pub.publish(cloud_msg)

        rospy.loginfo(
            f"published pointcloud: npts = {npts}, Find Pts = {dt0:0.5f} sec, Convert to Cloud = {dt1:0.5f} sec. Total Time = {(total_time - t0):0.3f} sec"
        )


if __name__ == "__main__":
    rospy.init_node("sonar_pointcloud")

    translator = SonarTranslator()
    rospy.spin()
