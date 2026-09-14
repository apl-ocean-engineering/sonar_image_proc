// Copyright 2021 University of Washington Applied Physics Laboratory
//
// An implementation of an AbstractSonarInterface that wraps a ROS2
// marine_acoustic_msgs/msg/ProjectedSonarImage message.
//

#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <cassert>
#include <algorithm>
#include <cstdint>

#include "sonar_image_proc/AbstractSonarInterface.h"
#include "marine_acoustic_msgs/msg/projected_sonar_image.hpp"

namespace sonar_image_proc {

using marine_acoustic_msgs::msg::ProjectedSonarImage;
using sonar_image_proc::AbstractSonarInterface;
using std::vector;

struct SonarImageMsgInterface
    : public sonar_image_proc::AbstractSonarInterface {
  explicit SonarImageMsgInterface(
      std::shared_ptr<ProjectedSonarImage> ping)
    : _ping(std::move(ping)), do_log_scale_(false) {
    // Vertical field of view is determined by comparing
    // z / sqrt(x^2 + y^2) to tan(elevation_beamwidth/2)
    _verticalTanSquared =
        std::pow(std::tan(_ping->ping_info.tx_beamwidths[0] / 2.0), 2);

    for (const auto& pt : _ping->beam_directions) {
      auto az = atan2(-1 * pt.y, pt.z);
      _ping_azimuths.push_back(az);
    }
  }

  void do_log_scale(float min_db, float max_db) {
    do_log_scale_ = true;
    min_db_ = min_db;
    max_db_ = max_db;
    range_db_ = max_db - min_db;
  }

  AbstractSonarInterface::DataType_t data_type() const override {
    if (_ping->image.dtype == _ping->image.DTYPE_UINT8)
      return AbstractSonarInterface::TYPE_UINT8;
    else if (_ping->image.dtype == _ping->image.DTYPE_UINT16)
      return AbstractSonarInterface::TYPE_UINT16;
    else if (_ping->image.dtype == _ping->image.DTYPE_UINT32)
      return AbstractSonarInterface::TYPE_UINT32;
    return AbstractSonarInterface::TYPE_NONE;
  }

  const std::vector<float>& ranges() const override { return _ping->ranges; }
  const std::vector<float>& azimuths() const override { return _ping_azimuths; }
  float verticalTanSquared() const { return _verticalTanSquared; }

  uint8_t intensity_uint8(const AzimuthRangeIndices& idx) const override {
    if (do_log_scale_ && (_ping->image.dtype == _ping->image.DTYPE_UINT32)) {
      return intensity_float_log(idx) * UINT8_MAX;
    }
    if (_ping->image.dtype == _ping->image.DTYPE_UINT8) {
      return read_uint8(idx);
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT16) {
      return read_uint16(idx) >> 8;
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT32) {
      return read_uint32(idx) >> 24;
    }
    return 0;
  }

  uint16_t intensity_uint16(const AzimuthRangeIndices& idx) const override {
    if (do_log_scale_ && (_ping->image.dtype == _ping->image.DTYPE_UINT32)) {
      return intensity_float_log(idx) * UINT16_MAX;
    }
    if (_ping->image.dtype == _ping->image.DTYPE_UINT8) {
      return read_uint8(idx) << 8;
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT16) {
      return read_uint16(idx);
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT32) {
      return read_uint32(idx) >> 16;
    }
    return 0;
  }

  uint32_t intensity_uint32(const AzimuthRangeIndices& idx) const override {
    if (do_log_scale_ && (_ping->image.dtype == _ping->image.DTYPE_UINT32)) {
      return intensity_float_log(idx) * UINT32_MAX;
    }
    if (_ping->image.dtype == _ping->image.DTYPE_UINT8) {
      return read_uint8(idx) << 24;
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT16) {
      return read_uint16(idx) << 16;
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT32) {
      return read_uint32(idx);
    }
    return 0;
  }

  float intensity_float(const AzimuthRangeIndices& idx) const override {
    if (do_log_scale_ && (_ping->image.dtype == _ping->image.DTYPE_UINT32)) {
      return intensity_float_log(idx);
    }
    if (_ping->image.dtype == _ping->image.DTYPE_UINT8) {
      return static_cast<float>(read_uint8(idx)) / UINT8_MAX;
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT16) {
      return static_cast<float>(read_uint16(idx)) / UINT16_MAX;
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT32) {
      return static_cast<float>(read_uint32(idx)) / UINT32_MAX;
    }
    return 0.0;
  }

protected:
  std::shared_ptr<ProjectedSonarImage> _ping;
  float _verticalTanSquared;
  std::vector<float> _ping_azimuths;

  size_t index(const AzimuthRangeIndices& idx) const {
    int data_size;
    if (_ping->image.dtype == _ping->image.DTYPE_UINT8) {
      data_size = 1;
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT16) {
      data_size = 2;
    } else if (_ping->image.dtype == _ping->image.DTYPE_UINT32) {
      data_size = 4;
    } else {
      assert(false);
    }
    return data_size * ((idx.range() * azimuths().size()) + idx.azimuth());
  }

  // "raw" read functions. Assumes the data type has already been checked
  uint32_t read_uint8(const AzimuthRangeIndices& idx) const {
    assert(_ping->image.dtype == _ping->image.DTYPE_UINT8);
    const auto i = index(idx);
    return (_ping->image.data[i]);
  }

  uint32_t read_uint16(const AzimuthRangeIndices& idx) const {
    assert(_ping->image.dtype == _ping->image.DTYPE_UINT16);
    const auto i = index(idx);

    return (static_cast<uint16_t>(_ping->image.data[i]) |
            (static_cast<uint16_t>(_ping->image.data[i + 1]) << 8));
  }

  uint32_t read_uint32(const AzimuthRangeIndices& idx) const {
    assert(_ping->image.dtype == _ping->image.DTYPE_UINT32);
    const auto i = index(idx);

    const uint32_t v =
        (static_cast<uint32_t>(_ping->image.data[i]) |
         (static_cast<uint32_t>(_ping->image.data[i + 1]) << 8) |
         (static_cast<uint32_t>(_ping->image.data[i + 2]) << 16) |
         (static_cast<uint32_t>(_ping->image.data[i + 3]) << 24));
    return v;
  }

  float intensity_float_log(const AzimuthRangeIndices& idx) const {
    const auto intensity = read_uint32(idx);
    const auto v =
        log(static_cast<float>(std::max((uint32_t)1, intensity)) / UINT32_MAX) * 10;  // dbm

    const auto min_db = (min_db_ == 0 ? log(1.0 / UINT32_MAX) * 10 : min_db_);

    return std::min(1.f, std::max(0.f, static_cast<float>((v - min_db) / range_db_)));
  }

  bool do_log_scale_;
  float min_db_ = 0.0f, max_db_ = 0.0f, range_db_ = 0.0f;
};

}  // namespace sonar_image_proc