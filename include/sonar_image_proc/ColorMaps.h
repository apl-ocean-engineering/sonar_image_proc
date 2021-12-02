// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include <opencv2/core.hpp>

#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

using cv::Scalar;

struct SonarColorMap {
    /// All color maps map sonar data to a Scalar (an OpenCV Vec<float,4>)
    /// RGB values are all in the [0,1] interval so you must
    /// multiply by 255 to use in CV_8UC3 images!
    ///
    virtual Scalar scalarLookup(float bearing,
                            float range,
                            uint8_t intensity) const = 0;


    template <typename tp>
    tp lookup(float bearing,
                            float range,
                            uint8_t intensity) const {
      return tp(scalarLookup(bearing,range,intensity));
    }

    // Overload which works on the ping type directly
    //
    template <typename tp>
    tp lookup(const sonar_image_proc::AbstractSonarInterface &ping,
                            int bearing_idx, int range_idx) const {
      return lookup<tp>(ping.bearing(bearing_idx),
                          ping.range(range_idx),
                          ping.intensity(bearing_idx, range_idx));
    }

};


// explicit specialization for Vec<uchar,3>.
// When returning unsigned int, scale by 255
template <>
inline cv::Vec3b SonarColorMap::lookup(float bearing,
                                float range,
                                uint8_t intensity) const {
    const Scalar result = 255 * scalarLookup(bearing, range, intensity);
    return cv::Vec3b(result[0], result[1], result[2]);
}

// explicit specialization for Vec<float,3>.
template <>
inline cv::Vec3f SonarColorMap::lookup(float bearing,
                                float range,
                                uint8_t intensity) const {
    const Scalar result = scalarLookup(bearing, range, intensity);
    return cv::Vec3f(result[0], result[1], result[2]);
}

//=== Implementations of specific color maps ===

struct MitchellColorMap : public SonarColorMap {
  virtual cv::Scalar scalarLookup(float bearing,
                                float range,
                                uint8_t intensity) const {
    const float i = intensity / 256;
    return cv::Scalar( 1-i, i, i );
  }
};


// Color maps from
//    https://github.com/BIDS/colormap/blob/master/colormaps.py
//
// As used in matplotlib
// As released under a CC0 license

struct InfernoColorMap : public SonarColorMap {
  static const float _inferno_data[][3];

  virtual Scalar scalarLookup(float bearing,
                            float range,
                            uint8_t intensity) const {
    return Scalar(_inferno_data[intensity][0],
                  _inferno_data[intensity][1],
                  _inferno_data[intensity][2]);
  }
};

}  // namespace sonar_image_proc
