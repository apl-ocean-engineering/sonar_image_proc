// Copyright 2021 University of Washington Applied Physics Laboratory
//
// An implementation of an AbstractSonarInterface which
// wraps a ROS acoustic_msgs::SonarImage
//

#include <vector>
#include "sonar_image_proc/AbstractSonarInterface.h"


namespace sonar_image_proc {

using std::vector;
using sonar_image_proc::AbstractSonarInterface;
using acoustic_msgs::SonarImage;

struct SonarImageMsgInterface : public sonar_image_proc::AbstractSonarInterface {
  explicit SonarImageMsgInterface(const acoustic_msgs::SonarImage::ConstPtr &ping)
    : _ping(ping) {
          // Vertical field of view is determined by comparing
    // z / sqrt(x^2 + y^2) to tan(elevation_beamwidth/2)
    _verticalTanSquared = std::pow(std::tan(ping->elevation_beamwidth/2.0), 2);
  }

    AbstractSonarInterface::DataType_t data_type() const override { 
        if (_ping->data_size == 1)
            return AbstractSonarInterface::TYPE_UINT8;
        else if (_ping->data_size == 2)
            return AbstractSonarInterface::TYPE_UINT16;
        else if (_ping->data_size == 4)
            return AbstractSonarInterface::TYPE_UINT32;
    }

  const std::vector<float> &ranges() const override {
    return _ping->ranges;
  }

  const std::vector<float> &azimuths() const override {
    return _ping->azimuth_angles;
  }

    float verticalTanSquared() const { return _verticalTanSquared; }

    // When the underlying data is 8-bit, this optimized version
    // returns that exact value.
    //
    // If the underlying data is 16-bit, it returns a scaled value.
    uint8_t intensity_uint8(const AzimuthRangeIndices &idx) const override {
        const auto i = index(idx);

        if (_ping->data_size == 1) {
            return _ping->intensities[i];
        } else if (_ping->data_size == 2) {
            return intensity_float(idx) * UINT8_MAX;
        } else if (_ping->data_size == 4) {
            return intensity_uint32(idx) >> 24;
        }

        return 0;
    }

    uint16_t intensity_uint16(const AzimuthRangeIndices &idx) const override {
        const auto i = index(idx);

        if (_ping->data_size == 1) {
            return _ping->intensities[i];
        } else if (_ping->data_size == 2) {
            return (static_cast<uint16_t>(_ping->intensities[i]) |
                   (static_cast<uint16_t>(_ping->intensities[i+1]) << 8));
        } else if (_ping->data_size == 4) {
            return intensity_uint32(idx) >> 16;
        }
        return 0;
    }

    uint32_t intensity_uint32(const AzimuthRangeIndices &idx) const override {
        const auto i = index(idx);

        if (_ping->data_size == 1) {
            return intensity_uint8(idx);
        } else if (_ping->data_size == 2) {
            return intensity_uint16(idx);
        } else if (_ping->data_size == 4) {
            const uint32_t v = (static_cast<uint32_t>(_ping->intensities[i]) |
                                (static_cast<uint32_t>(_ping->intensities[i+1]) << 8) |
                                (static_cast<uint32_t>(_ping->intensities[i+2]) << 16) |
                                (static_cast<uint32_t>(_ping->intensities[i+3]) << 24));
            return v;
        }
        return 0;
    }

    float intensity_float(const AzimuthRangeIndices &idx) const override {
        const auto i = index(idx);

        // !! n.b.  Need to ensure these calls aren't circular!!
        if (_ping->data_size == 1) {
            return static_cast<float>(intensity_uint8(idx)) / UINT8_MAX;
        } else if (_ping->data_size == 2) {
            // Data is stored LSB
            return static_cast<float>(intensity_uint16(idx)) / UINT16_MAX;
        } else if (_ping->data_size == 4) {
            return static_cast<float>(intensity_uint32(idx)) / UINT32_MAX;
        }
        return 0.0;
    }

 protected:
  acoustic_msgs::SonarImage::ConstPtr _ping;

  float _verticalTanSquared;

  size_t index(const AzimuthRangeIndices &idx) const {
    assert( (_ping->data_size == 1) || (_ping->data_size == 2) || (_ping->data_size == 4));
    return _ping->data_size*((idx.range() * nBearings()) + idx.azimuth());
  }
};

}  // namespace draw_sonar