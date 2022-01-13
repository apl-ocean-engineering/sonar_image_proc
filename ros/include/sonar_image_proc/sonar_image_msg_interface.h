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

  // Optimized version...
  uint8_t intensity_uint8(const AzimuthRangeIndices &idx) const override {
      const auto i = index(idx);

      if (_ping->data_size == 1) {
          return _ping->intensities[i];
      } else if (_ping->data_size == 2) {
          return 255 * intensity_float(idx);
      }

      return 0;
  }

  uint16_t intensity_uint16(const AzimuthRangeIndices &idx) const override {
      const auto i = index(idx);

      if (_ping->data_size == 1) {
          return _ping->intensities[i] << 8;
      } else if (_ping->data_size == 2) {
          return _ping->intensities[i] << 8 | _ping->intensities[i+1];
      }
      return 0;
  }

  virtual float intensity_float(const AzimuthRangeIndices &idx) const {
      const auto i = index(idx);

      if (_ping->data_size == 1) {
          return _ping->intensities[i]/255;
      } else if (_ping->data_size == 2) {
          return (_ping->intensities[i] << 8 | _ping->intensities[i+1])/65535;
      }
      return 0.0;
  }

  acoustic_msgs::SonarImage::ConstPtr _ping;

  float _verticalTanSquared;

  size_t index(const AzimuthRangeIndices &idx) const {
    assert( (_ping->data_size == 1) || (_ping->data_size == 2));
    return _ping->data_size*((idx.range() * nBearings()) + idx.azimuth());
  }
};

}  // namespace draw_sonar