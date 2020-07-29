
#pragma once

#include <opencv2/core.hpp>

namespace draw_sonar {

using cv::Scalar;

struct SonarColorMap {

  /// All color maps map sonar data to a Scalar.  RGB values are all in
  /// the [0,1] interval so you must multiply by 255 to use in CV_8UC3 images!
  ///
  virtual Scalar operator()( float bearing, float range, uint8_t intensity ) const = 0;
};

struct MitchellColorMap : public SonarColorMap {

  virtual cv::Scalar operator()( float bearing, float range, uint8_t intensity ) const {
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

  virtual Scalar operator()( float bearing, float range, uint8_t intensity ) const {
    return Scalar( _inferno_data[intensity][0],_inferno_data[intensity][1],_inferno_data[intensity][2] );
  }
};


}
