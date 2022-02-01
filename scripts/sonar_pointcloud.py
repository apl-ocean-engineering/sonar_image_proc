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

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

class SonarTranslator(object):
    def __init__(self):
        # NB: if queue_size is set, have to be sure buff_size is sufficiently large,
        #     since it can only discard messages that are fully in the buffer.
        # https://stackoverflow.com/questions/26415699/ros-subscriber-not-up-to-date
        # (200k didn't work, and sys.getsizeof doesn't return an accurate size of the whole object)
        self.sub = rospy.Subscriber("sonar_image", SonarImage, self.callback, queue_size=1, buff_size=1000000)
        self.pub = rospy.Publisher("sonar_cloud", PointCloud2, queue_size=1)

        min_elev_deg = rospy.get_param("~min_elev_deg", 0)
        max_elev_deg = rospy.get_param("~max_elev_deg", 0)
        assert(max_elev_deg >= min_elev_deg)
        elev_step_deg = rospy.get_param("~elev_step_deg", 5.0)
        self.min_elev = np.radians(min_elev_deg)
        self.max_elev = np.radians(max_elev_deg)
        self.elev_step = np.radians(elev_step_deg)
        self.elevations = np.arange(self.min_elev, self.max_elev+self.elev_step, self.elev_step)

        # TODO: assert that this is in range of uint8? (Or, msg.data_size)
        self.intensity_threshold = rospy.get_param("~intensity_threshold", 100)

        # x,y,z coordinates of points
        # [ELEVATION_IDX, INTENSITY_IDX, DIMENSION_IDX]
        # Shape chosen for ease of mapping coordinates to intensities
        self.geometry = None
        self.geometry_np = None
        self.intensity_lookup = None
        self.intensitity_lookup_np = None

    # TODO: This assumes that the intensity data is single byte
    def make_intensity_lookup(self):
        rospy.loginfo("make_intensity_lookup")
        self.intensity_lookup = [0 for _ in range(256)]
        for aa in range(256):
            r,g,b,_ = cm.inferno(aa)
            rr = int(255*r)
            gg = int(255*g)
            bb = int(255*b)
            rgba = struct.unpack('I', struct.pack('BBBB', bb, gg, rr, aa))[0]
            self.intensity_lookup[aa] = rgba

        self.intensitity_lookup_np = np.array(self.intensity_lookup)

    def make_geometry(self, image_msg):
        rospy.loginfo("make_geometry")
        nranges = len(image_msg.ranges)
        nangles = len(image_msg.azimuth_angles)
        points = [[[0, 0, 0] for _ in range(nranges * nangles)] for _ in range(len(self.elevations))]

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

    def make_geometry_fast(self, image_msg):
        """
        Vectorizing make_geometry() for faster creation
        """
        rospy.loginfo("make_geometry")
        nranges = len(image_msg.ranges)
        nangles = len(image_msg.azimuth_angles)
        
        azimuth_angles = np.arange(nangles)
        ranges = np.arange(nranges)

        x, y = np.meshgrid(ranges, azimuth_angles)
        ce = np.cos(self.elevations)
        se = np.sin(self.elevations)

        tranges = x.flatten()
        tangles = y.flatten()
        idxs = tangles + tranges * nangles

        t_ca = np.cos(tangles)
        t_sa = np.sin(tangles)

        xx = tranges * ce * t_ca
        yy = tranges * ce * t_sa
        zz = tranges * se

        output = np.empty((nranges*nranges, 3))
        output[idxs, :] = np.stack([xx, yy, zz], axis=1)
        output = output[np.newaxis, :, :]

        self.geometry = list(output)

    def callback(self, image_msg):
        header = Header()
        header = image_msg.header

        # If specified, rewrite the frame in th
        frame_id = rospy.get_param("~frame_id", None)
        if frame_id:
            header.frame_id = frame_id

        if image_msg.data_size != 1:
            raise Exception("NYI: mult-byte intensity data")
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]

        # t0 = time.time()
        nranges = len(image_msg.ranges)
        nangles = len(image_msg.azimuth_angles)
        npts = nranges * nangles
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

        # Construction of PointCloud2 cribbed from:
        # https://answers.ros.org/question/289576/understanding-the-bytes-in-a-pcl2-message/
        # points = []
        # return_idxs, = np.where(np.array([int(ii) for ii in image_msg.intensities]) > self.intensity_threshold)
        # rospy.loginfo("{} (out of {}) points are above threshold".format(len(return_idxs), len(image_msg.intensities)))
        start_time = time.time()
        intensities = np.frombuffer(image_msg.intensities, dtype=np.uint8)
        output_points = np.empty((nranges * nangles, 4))
        output_points[:, 0:3] = self.geometry_np
        output_points[:, -1] = np.where(intensities > self.intensity_threshold, 
                                        self.intensitity_lookup_np[intensities- self.intensity_threshold],  
                                        0)
        dtype = np.float32
        itemsize = np.dtype(dtype).itemsize
        data = output_points.astype(dtype).tobytes()
        # cloud_msg = PointCloud2(
        #                         header=header,
        #                         height=1,
        #                         width=output_points.shape[0],
        #                         is_dense=False,
        #                         is_bigendian=False,
        #                         fields=fields,
        #                         point_step=(itemsize * 3),
        #                         row_step=(itemsize * 4 * output_points.shape[0]),
        #                         data=data
        #                     )
        t0 = time.time() - start_time        
         
        # start_time = time.time()
        points = []                     
        for geometry in self.geometry:
            pts = [[xx, yy, zz, self.intensity_lookup[aa-self.intensity_threshold] if aa > self.intensity_threshold else 0]
                   for (xx, yy, zz), aa in zip(geometry, image_msg.intensities)]
            points.extend(pts)

        # t1= time.time() - start_time

        # t2 = time.time()
        

        cloud_msg = point_cloud2.create_cloud(header, fields, points)
        # start = time.time()
        # test_msg = point_cloud(output_points, frame_id)
        # rospy.loginfo(f'{(time.time() - start):.3f}')
        # t2 = time.time() - start_time
        
        self.pub.publish(cloud_msg)
        # rospy.loginfo(f'Created Cloud (fast): {t0:.3f}s')

def point_cloud(points, parent_frame):
    """ Creates a point cloud message.
    Args:
        points: Nx4 array of xyz positions (m) and a colors
        parent_frame: frame in which the point cloud is defined
    Returns:
        sensor_msgs/PointCloud2 message
    """
    ros_dtype = PointField.FLOAT32
    dtype = np.float32
    itemsize = np.dtype(dtype).itemsize
    data = points.astype(dtype).tobytes()

    # fields = [PointField(
    #     name=n, offset=i*itemsize, datatype=ros_dtype, count=1)
    #     for i, n in enumerate('xyza')]
    fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgba', 12, PointField.UINT32, 1)]
    

    header = Header(frame_id=parent_frame, stamp=rospy.Time.now())

    return PointCloud2(
        header=header,
        height=1,
        width=points.shape[0],
        is_dense=False,
        is_bigendian=False,
        fields=fields,
        point_step=(itemsize * 7),
        row_step=(itemsize * 7 * points.shape[0]),
        data=data
    )

if __name__ == "__main__":
    rospy.init_node("sonar_pointcloud")

    translator = SonarTranslator()
    rospy.spin()
