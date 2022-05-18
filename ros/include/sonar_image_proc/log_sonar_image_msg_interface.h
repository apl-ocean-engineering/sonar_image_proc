// Copyright 2021 University of Washington Applied Physics Laboratory
//
// An implementation of an AbstractSonarInterface which
// wraps a ROS acoustic_msgs::SonarImage
//

#pragma once

#include "sonar_image_msg_interface.h"

namespace sonar_image_proc {

struct LogScaleSonarImageMsgInterface : public SonarImageMsgInterface {
  explicit LogScaleSonarImageMsgInterface(
      const acoustic_msgs::SonarImage::ConstPtr &ping, float min_db = 0,
      float max_db = 0)
      : SonarImageMsgInterface(ping), min_db_(min_db), max_db_(max_db) {
    range_db_ = max_db - min_db;
  }

  // Log scale is only useful for 32bit data
  // For 8- and 16-bit data, just pass it through.
  uint8_t intensity_uint8(const AzimuthRangeIndices &idx) const override {
    const auto i = index(idx);

    if ((_ping->data_size == 1) || (_ping->data_size == 2)) {
      return SonarImageMsgInterface::intensity_uint8(idx);
    } else if (_ping->data_size == 4) {
      return intensity_float(idx) * UINT8_MAX;
    }

    return 0;
  }

  uint16_t intensity_uint16(const AzimuthRangeIndices &idx) const override {
    const auto i = index(idx);

    if ((_ping->data_size == 1) || (_ping->data_size == 2)) {
      return SonarImageMsgInterface::intensity_uint16(idx);
    } else if (_ping->data_size == 4) {
      return intensity_float(idx) * UINT16_MAX;
    }
    return 0;
  }

  uint32_t intensity_uint32(const AzimuthRangeIndices &idx) const override {
    const auto i = index(idx);

    if ((_ping->data_size == 1) || (_ping->data_size == 2)) {
      return SonarImageMsgInterface::intensity_uint32(idx);
    } else if (_ping->data_size == 4) {
      return intensity_float(idx) * UINT32_MAX;
    }
    return 0;
  }

  float intensity_float(const AzimuthRangeIndices &idx) const override {
    const auto i = index(idx);

    // !! n.b.  Need to ensure these calls aren't circular!!
    if ((_ping->data_size == 1) || (_ping->data_size == 2)) {
      return SonarImageMsgInterface::intensity_float(idx);
    } else if (_ping->data_size == 4) {
      const auto intensity = SonarImageMsgInterface::intensity_uint32(idx);
      const auto v =
          log(static_cast<float>(std::max((uint)1, intensity)) / UINT32_MAX) *
          10; // dbm

      const auto min_db = (min_db_ == 0 ? log(1.0 / UINT32_MAX) * 10 : min_db_);

      return std::min(1.0, std::max(0.0, (v - min_db) / range_db_));
    }
    return 0.0;
  }

  float min_db_, max_db_, range_db_;
};

} // namespace sonar_image_proc