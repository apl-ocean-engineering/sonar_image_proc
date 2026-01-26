// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include "libdrawsonar/AbstractSonarInterface.h"

namespace libdrawsonar {

class HistogramGenerator {
public:
  static std::vector<unsigned int> Generate(const AbstractSonarInterface &ping);

  static std::vector<unsigned int>
  GenerateUint8(const AbstractSonarInterface &ping);
  static std::vector<unsigned int>
  GenerateUint16(const AbstractSonarInterface &ping);

  // Histogram generator for 32bit data produces 256 bins of log10(intensity)
  static std::vector<unsigned int>
  GenerateUint32(const AbstractSonarInterface &ping);
};

} // namespace libdrawsonar
