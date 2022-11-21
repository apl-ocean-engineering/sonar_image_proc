// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include "sonar_image_proc/HistogramGenerator.h"

#include <iostream>

namespace sonar_image_proc {

std::vector<unsigned int> HistogramGenerator::Generate(
    const AbstractSonarInterface &ping) {
  if (ping.data_type() == AbstractSonarInterface::TYPE_UINT8)
    return HistogramGenerator::GenerateUint8(ping);
  else if (ping.data_type() == AbstractSonarInterface::TYPE_UINT16)
    return HistogramGenerator::GenerateUint16(ping);
  else if (ping.data_type() == AbstractSonarInterface::TYPE_UINT32)
    return HistogramGenerator::GenerateUint32(ping);

  return std::vector<unsigned int>();
}

// \todo Some repetition, but these functions are pretty simple
std::vector<unsigned int> HistogramGenerator::GenerateUint8(
    const AbstractSonarInterface &ping) {
  std::vector<unsigned int> result(256, 0);

  for (int r = 0; r < ping.nRanges(); r++) {
    for (int b = 0; b < ping.nBearings(); b++) {
      const auto val = ping.intensity_uint8(AzimuthRangeIndices(b, r));
      result[val]++;
    }
  }

  return result;
}

std::vector<unsigned int> HistogramGenerator::GenerateUint16(
    const AbstractSonarInterface &ping) {
  std::vector<unsigned int> result(65536, 0);

  for (int r = 0; r < ping.nRanges(); r++) {
    for (int b = 0; b < ping.nBearings(); b++) {
      const auto val = ping.intensity_uint16(AzimuthRangeIndices(b, r));

      result[val]++;
    }
  }

  return result;
}

std::vector<unsigned int> HistogramGenerator::GenerateUint32(
    const AbstractSonarInterface &ping) {
  std::vector<unsigned int> result(256, 0);

  const float logMax = log10(UINT32_MAX);

  for (int r = 0; r < ping.nRanges(); r++) {
    for (int b = 0; b < ping.nBearings(); b++) {
      const auto val = ping.intensity_uint32(AzimuthRangeIndices(b, r));

      if (val == 0) continue;

      const float l = log10(val);

      const size_t idx = UINT8_MAX * (l / logMax);

      // std::cerr << "val = " << val << "; l = " << l << "; idx = " << idx <<
      // std::endl;

      if ((idx < 0) || (idx > result.size())) continue;
      result[idx]++;
    }
  }

  return result;
}

}  // namespace sonar_image_proc
