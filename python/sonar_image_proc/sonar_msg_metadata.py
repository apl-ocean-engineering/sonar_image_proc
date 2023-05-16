#! /usr/bin/env python3
"""
Copyright 2023 University of Washington Applied Physics Laboratory
Author: Marc Micatka & Laura Lindzey
"""

from __future__ import annotations  # use type of class in member function annotation.

import numpy as np
import typing
from marine_acoustic_msgs.msg import ProjectedSonarImage


class SonarImageMetadata(object):
    def __init__(self, sonar_image_msg: ProjectedSonarImage):
        """
        Metadata for a sonar image, containing all information necessary
        to compute its geometry.
        NOTE(lindzey): excludes beamwidths because those are not used when
            deciding what elevation angles to publish.
        """
        self.num_angles = len(sonar_image_msg.beam_directions)
        self.num_ranges = len(sonar_image_msg.ranges)
        self.ranges = np.array(sonar_image_msg.ranges)
        self.min_range = np.min(sonar_image_msg.ranges)
        self.max_range = np.max(sonar_image_msg.ranges)

        xx = np.array([dir.x for dir in sonar_image_msg.beam_directions])
        yy = np.array([dir.y for dir in sonar_image_msg.beam_directions])
        zz = np.array([dir.z for dir in sonar_image_msg.beam_directions])
        self.azimuths = np.arctan2(-1 * yy, np.sqrt(xx**2 + zz**2))
        self.min_azimuth = np.min(self.azimuths)
        self.max_azimuth = np.max(self.azimuths)
        elev_beamwidth = np.median(sonar_image_msg.ping_info.tx_beamwidths)
        self.min_elevation = -0.5 * elev_beamwidth
        self.max_elevation = 0.5 * elev_beamwidth

    def __eq__(self, other: SonarImageMetadata) -> bool:
        """
        Overrides the default implementation of == and != (along with is and is not)
        Determine whether all fields are "close enough" for the
        metadata to be the same.
        """
        if self.num_angles != other.num_angles:
            return False
        if self.num_ranges != other.num_ranges:
            return False
        return np.allclose(
            [self.min_range, self.max_range, self.min_azimuth, self.max_azimuth],
            [other.min_range, other.max_range, other.min_azimuth, other.max_azimuth],
        )

    def __str__(self) -> typing.String:
        """
        Overrides the default implementation of print(SonarImageMetadata)
        """
        ss = "SonarImageMetadata: {} beams, {} ranges, {:0.2f}=>{:0.2f} m, {:0.1f}=>{:0.1f} deg".format(
            self.num_angles,
            self.num_ranges,
            self.min_range,
            self.max_range,
            np.degrees(self.min_azimuth),
            np.degrees(self.max_azimuth),
        )
        return ss
