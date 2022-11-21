// Copyright 2021 University of Washington Applied Physics Laboratory
//
// An implementation of an AbstractSonarInterface which
// wraps a ROS acoustic_msgs::SonarImage
//

#pragma once

#include <vector>

#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

using acoustic_msgs::ProjectedSonarImage;
using sonar_image_proc::AbstractSonarInterface;
using std::vector;

struct SonarImageMsgInterface
    : public sonar_image_proc::AbstractSonarInterface {
  explicit SonarImageMsgInterface(
      const acoustic_msgs::ProjectedSonarImage::ConstPtr &ping)
      : _ping(ping), do_log_scale_(false) {
    // Vertical field of view is determined by comparing
    // z / sqrt(x^2 + y^2) to tan(elevation_beamwidth/2)
    _verticalTanSquared =
        // NOTE(lindzey): The old message assumed a constant elevation
        // beamwidth;
        //     the multibeam people insisted that it be per-beam. For now, this
        //     assumes that they're all the same.
        // TODO(lindzey): Look into whether averaging would be better, or if we
        //     should create an array of verticalTanSquared.
        // TODO(lindzey): Handle empty-array case.
        std::pow(std::tan(ping->ping_info.tx_beamwidths[0] / 2.0), 2);

    for (const auto pt : ping->beam_directions) {
      auto az = atan2(-1 * pt.y, pt.z);
      _ping_azimuths.push_back(az);
    }
  }

  // n.b. do_log_scale is a mode switch which causes the intensity_*
  // functions in this SonarImageMsgInterface use a log (db) scale
  // for 32-bit data **ONLY**  -- functionality is unchanged for 8-
  // and 16-bit data
  //
  // It maps a given 32-bit intensity value i to a fraction of the
  // full scale of a 32-bit uint in the interval [0.0, 1.0]
  // It then takes the db value of that:
  //
  //     l = 10 * log( i / (2^32-1) )
  //
  // Such that l is between
  //      ~ -96 for i = 1
  //          and
  //        0.0 for i = UINT32_MAX
  //
  // The intensity_* functions create a linear map m() from [min_db, max_db]
  // to the full range of the returned type (e.g. uint8, uint16, etc),
  // and returns m(l)
  //
  // If min_db is 0.0, then the full range value
  //                       min_db = (10*log(1/(2^32-1))) is used
  // If max_db is 0.0, then the full range value
  //                       max_db = 0.0 is used
  //
  // So for example if i = 2^31 (half the full range of a uint32), l = -3 db
  //  This is approx 0.97 of the full scale in logspace
  //
  // With min_db = 0, max_db = 0 (full range)
  //      intensity_uint8(i) will return approx 247  (0.97 * 255)
  //
  // With min_db = -10, max_db = 0
  //      intensity_uint8(i) will return approx 178  (0.7 * 255)
  //
  // With min_db = -1, max_db = 0
  //      l(i) is less than min_db and intensity_uint8(i) will return 0
  //
  // With min_db = 0, max_db = -10
  //      l(i) is greater than max_db and intensity_uint8(i) will return 255
  //
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

    //
    return AbstractSonarInterface::TYPE_NONE;
  }

  const std::vector<float> &ranges() const override { return _ping->ranges; }

  const std::vector<float> &azimuths() const override { return _ping_azimuths; }

  float verticalTanSquared() const { return _verticalTanSquared; }

  // When the underlying data is 8-bit, returns that exact value
  // from the underlying data
  //
  // If the underlying data is 16-bit, it returns a scaled value.
  uint8_t intensity_uint8(const AzimuthRangeIndices &idx) const override {
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

  uint16_t intensity_uint16(const AzimuthRangeIndices &idx) const override {
    // Truncate 32bit intensities
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

  uint32_t intensity_uint32(const AzimuthRangeIndices &idx) const override {
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

  float intensity_float(const AzimuthRangeIndices &idx) const override {
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
  acoustic_msgs::ProjectedSonarImage::ConstPtr _ping;

  float _verticalTanSquared;
  std::vector<float> _ping_azimuths;

  size_t index(const AzimuthRangeIndices &idx) const {
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

    return data_size * ((idx.range() * nBearings()) + idx.azimuth());
  }

  // "raw" read functions.  Assumes the data type has already been checked
  uint32_t read_uint8(const AzimuthRangeIndices &idx) const {
    assert(_ping->image.dtype == _ping->image.DTYPE_UINT8);
    const auto i = index(idx);

    return (_ping->image.data[i]);
  }

  uint32_t read_uint16(const AzimuthRangeIndices &idx) const {
    assert(_ping->image.dtype == _ping->image.DTYPE_UINT16);
    const auto i = index(idx);

    return (static_cast<uint16_t>(_ping->image.data[i]) |
            (static_cast<uint16_t>(_ping->image.data[i + 1]) << 8));
  }

  uint32_t read_uint32(const AzimuthRangeIndices &idx) const {
    assert(_ping->image.dtype == _ping->image.DTYPE_UINT32);
    const auto i = index(idx);

    const uint32_t v =
        (static_cast<uint32_t>(_ping->image.data[i]) |
         (static_cast<uint32_t>(_ping->image.data[i + 1]) << 8) |
         (static_cast<uint32_t>(_ping->image.data[i + 2]) << 16) |
         (static_cast<uint32_t>(_ping->image.data[i + 3]) << 24));
    return v;
  }

  float intensity_float_log(const AzimuthRangeIndices &idx) const {
    const auto intensity = read_uint32(idx);
    const auto v =
        log(static_cast<float>(std::max((uint)1, intensity)) / UINT32_MAX) *
        10;  // dbm

    const auto min_db = (min_db_ == 0 ? log(1.0 / UINT32_MAX) * 10 : min_db_);

    return std::min(1.0, std::max(0.0, (v - min_db) / range_db_));
  }

  bool do_log_scale_;
  float min_db_, max_db_, range_db_;
};

}  // namespace sonar_image_proc
