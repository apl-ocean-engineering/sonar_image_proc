// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

class HistogramGenerator {
 public:
  static std::vector<unsigned int> Generate( const AbstractSonarInterface &ping );

  static std::vector<unsigned int> GenerateUint8( const AbstractSonarInterface &ping );
  static std::vector<unsigned int> GenerateUint16( const AbstractSonarInterface &ping );

};

}  // namespace sonar_image_proc
