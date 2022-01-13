// Copyright 2021 University of Washington Applied Physics Laboratory
//
// An implementation of an AbstractSonarInterface which
// wraps a ROS acoustic_msgs::SonarImage
//

#include <vector>
#include "sonar_image_proc/AbstractSonarInterface.h"


namespace sonar_image_proc {

using namespace std;

struct SonarImageMsgInterface : public sonar_image_proc::AbstractSonarInterface {

  explicit SonarImageMsgInterface(const acoustic_msgs::SonarImage::ConstPtr &ping)
    : _ping(ping) {
          // Vertical field of view is determined by comparing
    // z / sqrt(x^2 + y^2) to tan(elevation_beamwidth/2)
    _verticalTanSquared = std::pow(std::tan(ping->elevation_beamwidth/2.0), 2);
  }

  const std::vector<float> &ranges() const override {
    return _ping->ranges;
  }

  const std::vector<float> &azimuths() const override {
    return _ping->azimuth_angles;
  }

  float verticalTanSquared() const { return _verticalTanSquared; }

  uint8_t intensity( const AzimuthRangeIndices &idx ) const override {

    if (_ping->data_size == 1) {
      const size_t i = idx.azimuth() * nAzimuth() + idx.range();
      return _ping->intensities[i];
    } else if (_ping->data_size == 2) {
      uint16_t d;
      const size_t i = (idx.azimuth() * nAzimuth() + idx.range()) * 2;

      if (_ping->is_bigendian)
        d = (_ping->intensities[i] << 8) | _ping->intensities[i + 1];
      else
        d = (_ping->intensities[i + 1] << 8) | _ping->intensities[i];

      // Hacky
      const int shift = 6;
      if (d >= (0x1 << (shift+8))) return 0xFF;

      return (d >> shift);
    } else {
      ROS_ERROR_STREAM("SonarImage has unsupported data_size = " << _ping->data_size);
      return 0;
    }
  }

  acoustic_msgs::SonarImage::ConstPtr _ping;

  float _verticalTanSquared;
};

}  // namespace draw_sonar