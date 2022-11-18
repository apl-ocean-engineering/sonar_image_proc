#! /usr/bin/env python3
import rospy
import time

from matplotlib import cm
import numpy as np

from acoustic_msgs.msg import ProjectedSonarImage, SonarImageData
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class SonarTranslator(object):

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
        self.pub = rospy.Publisher("sonar_cloud", PointCloud2, queue_size=1)

        # Flag to determine whether we publish ALL points or only non-zero points
        self.publish_all_points = rospy.get_param("~publish_all_points", False)

        min_elev_deg = rospy.get_param("~min_elev_deg", 0)
        max_elev_deg = rospy.get_param("~max_elev_deg", 20)
        assert (max_elev_deg >= min_elev_deg)
        self.elev_steps = rospy.get_param("~elev_steps", 2)
        self.min_elev = np.radians(min_elev_deg)
        self.max_elev = np.radians(max_elev_deg)
        self.elevations = np.linspace(self.min_elev, self.max_elev,
                                      self.elev_steps)

        # threshold range is a float, [0-1]
        self.intensity_threshold = rospy.get_param("~intensity_threshold",
                                                   0.74)

        # x,y,z coordinates of points
        # [ELEVATION_IDX, INTENSITY_IDX, DIMENSION_IDX]
        # Shape chosen for ease of mapping coordinates to intensities
        self.geometry = None
        self.color_lookup = None
        self.output_points = None

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
        v = np.log(new_intensites) / np.log(np.iinfo(data_type).max)
        vmax = 1.0
        v = (v - self.intensity_threshold) / (vmax - self.intensity_threshold)

        v = np.clip(v, a_min=0.0, a_max=1.0)
        intensities = (np.iinfo(np.uint8).max * v).astype(np.uint8)

        return intensities

    def make_geometry(self, image_msg: ProjectedSonarImage):
        rospy.loginfo("make_geometry")
        nranges = len(image_msg.ranges)
        nangles = len(image_msg.beam_directions)
        points = [[[0, 0, 0] for _ in range(nranges * nangles)]
                  for _ in range(len(self.elevations))]

        for kk, elevation in enumerate(self.elevations):
            ce = np.cos(elevation)
            se = np.sin(elevation)
            for ii, beam_dir in enumerate(image_msg.beam_directions):
                # Pre-compute these values to speed up the loop
                azimuth = np.arctan2(-1 * beam_dir.y, beam_dir.z)
                ca = np.cos(azimuth)
                sa = np.sin(azimuth)
                for jj, distance in enumerate(image_msg.ranges):
                    idx = ii + jj * nangles
                    xx = distance * ce * ca
                    yy = -1 * distance * ce * sa
                    zz = distance * se
                    points[kk][idx] = [xx, yy, zz]

        self.geometry = np.array(points)

    def callback(self, image_msg: ProjectedSonarImage):
        """
        Convert img_msg into point cloud with color mappings via numpy.
        """
        begin_time = time.time()
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
            PointField('r', 12, PointField.FLOAT32, 1),
            PointField('g', 16, PointField.FLOAT32, 1),
            PointField('b', 20, PointField.FLOAT32, 1),
            PointField('a', 24, PointField.FLOAT32, 1)
        ]

        nranges = len(image_msg.ranges)
        nangles = len(image_msg.beam_directions)
        npts = nranges * nangles

        if self.geometry is None:
            self.make_geometry(image_msg)

        if self.color_lookup is None:
            self.make_color_lookup()
        t0 = time.time()

        intensities = self.process_intensity_array(image_msg.image)
        # Make a copy of geometry, if not, indexing into positive values will
        # change the array after one iteration
        geometry = self.geometry.copy()

        if not self.publish_all_points:
            pos_intensity_idx = np.where(intensities > 0)
            intensities = intensities[pos_intensity_idx]
            geometry = self.geometry[:, pos_intensity_idx[0]]

        # Calculate the number of points we need:
        npts = len(intensities)

        # Allocate our output points
        self.output_points = np.zeros(
            (len(self.elevations) * npts, 7), dtype=np.float32)\

        # Expand out intensity array (for fast comparison)
        expanded_intensities = np.repeat(intensities[..., np.newaxis],
                                         4,
                                         axis=1)
        # Fill the output array
        for i in range(len(self.elevations)):
            points = np.empty((npts, 7))
            if points[:, 0:3].shape != geometry[i, :, :].shape:
                # Occassionally the sonar message has a changing geometry
                # that really screws stuff up until it's resolved. Fix it here
                rospy.logdebug(
                    'Change in image size! Remaking geometry...')
                self.make_geometry(image_msg)

            points[:, 0:3] = geometry[i, :, :]

            points[:, 3:] = np.where(
                expanded_intensities > self.intensity_threshold,
                self.color_lookup[expanded_intensities[:, 0]],
                np.zeros((npts, 4)))

            step = i * npts
            next_step = step + npts
            self.output_points[step:next_step, :] = points

        t1 = time.time()
        N = len(self.output_points)
        cloud_msg = PointCloud2(header=header,
                                height=1,
                                width=N,
                                is_dense=True,
                                is_bigendian=False,
                                fields=fields,
                                point_step=28,
                                row_step=28 * N,
                                data=self.output_points.tobytes())

        dt1 = time.time() - t1
        dt0 = t1 - t0
        total_time = time.time()
        self.pub.publish(cloud_msg)

        rospy.logdebug(
            f"published pointcloud: npts = {npts}, Find Pts = {dt0:0.5f} sec, Convert to Cloud = {dt1:0.5f} sec. Total Time = {(total_time - begin_time):0.3f} sec"
        )


if __name__ == "__main__":
    rospy.init_node("sonar_pointcloud")

    translator = SonarTranslator()
    rospy.spin()
