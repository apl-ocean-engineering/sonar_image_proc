// Copyright 2021 University of Washington Applied Physics Laboratory
//
// An implementation of an AbstractSonarInterface which
// wraps a ROS acoustic_msgs::SonarImage
//

#pragma once

#include "sonar_image_proc/AbstractSonarInterface.h"
#include <vector>

namespace sonar_image_proc {

using acoustic_msgs::SonarImage;
using sonar_image_proc::AbstractSonarInterface;
using std::vector;

struct SonarImageMsgInterface
    : public sonar_image_proc::AbstractSonarInterface {
  explicit SonarImageMsgInterface(
      const acoustic_msgs::SonarImage::ConstPtr &ping)
      : _ping(ping), do_log_scale_(false) {
    // Vertical field of view is determined by comparing
    // z / sqrt(x^2 + y^2) to tan(elevation_beamwidth/2)
    _verticalTanSquared =
        std::pow(std::tan(ping->elevation_beamwidth / 2.0), 2);
  }

  // n.b. log_scale *does not* cause the intensity_* functions to return
  // 10*log(sonar value)
  //
  // Instead, the 10*log(sonar value) value is (linearly) mapped onto the
  // interval [min_db, max_db] and scaled into the range of the return value
  // e.g. [0,255] for uint8, [0,1] for float, etc.
  void do_log_scale(float min_db, float max_db) {
    do_log_scale_ = true;
    min_db_ = min_db;
    max_db_ = max_db;
    range_db_ = max_db - min_db;
  }

  AbstractSonarInterface::DataType_t data_type() const override {
    if (_ping->data_size == 1)
      return AbstractSonarInterface::TYPE_UINT8;
    else if (_ping->data_size == 2)
      return AbstractSonarInterface::TYPE_UINT16;
    else if (_ping->data_size == 4)
      return AbstractSonarInterface::TYPE_UINT32;

    //
    return AbstractSonarInterface::TYPE_NONE;
  }

  const std::vector<float> &ranges() const override { return _ping->ranges; }

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
      return intensity_uint16(idx) >> 8;
    } else if (_ping->data_size == 4) {
      if (do_log_scale_) {
        return intensity_float(idx) * UINT8_MAX;
      } else {
        return intensity_uint32(idx) >> 24;
      }
    }

    return 0;
  }

  uint16_t intensity_uint16(const AzimuthRangeIndices &idx) const override {
    const auto i = index(idx);

    if (_ping->data_size == 1) {
      return _ping->intensities[i];
    } else if (_ping->data_size == 2) {
      return (static_cast<uint16_t>(_ping->intensities[i]) |
              (static_cast<uint16_t>(_ping->intensities[i + 1]) << 8));
    } else if (_ping->data_size == 4) {
      if (do_log_scale_) {
        return intensity_float(idx) * UINT16_MAX;
      } else {
        return intensity_uint32(idx) >> 16;
      }
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
      if (do_log_scale_) {
        return intensity_float(idx) * UINT32_MAX;
      } else {
        return intensity_uint32_nonlog(i);
      }
    }
    return 0;
  }

  uint32_t intensity_uint32_nonlog(const size_t i) const {
    if (_ping->data_size == 4) {
      const uint32_t v =
          (static_cast<uint32_t>(_ping->intensities[i]) |
           (static_cast<uint32_t>(_ping->intensities[i + 1]) << 8) |
           (static_cast<uint32_t>(_ping->intensities[i + 2]) << 16) |
           (static_cast<uint32_t>(_ping->intensities[i + 3]) << 24));
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
      if (do_log_scale_) {
        const auto intensity = intensity_uint32_nonlog(i);
        const auto v =
            log(static_cast<float>(std::max((uint)1, intensity)) / UINT32_MAX) *
            10; // dbm

        const auto min_db =
            (min_db_ == 0 ? log(1.0 / UINT32_MAX) * 10 : min_db_);

        return std::min(1.0, std::max(0.0, (v - min_db) / range_db_));
      } else {
        return static_cast<float>(intensity_uint32(idx)) / UINT32_MAX;
      }
    }
    return 0.0;
  }

protected:
  acoustic_msgs::SonarImage::ConstPtr _ping;

  float _verticalTanSquared;

  size_t index(const AzimuthRangeIndices &idx) const {
    assert((_ping->data_size == 1) || (_ping->data_size == 2) ||
           (_ping->data_size == 4));
    return _ping->data_size * ((idx.range() * nBearings()) + idx.azimuth());
  }

  bool do_log_scale_;
  float min_db_, max_db_, range_db_;
};

} // namespace sonar_image_proc