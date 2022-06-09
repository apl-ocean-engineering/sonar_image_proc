// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include <algorithm>
#include <utility>

#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

const Bounds_t UnsetBounds = Bounds_t(-1, -1);

AbstractSonarInterface::AbstractSonarInterface()
    : _rangeBounds(UnsetBounds), _azimuthBounds(UnsetBounds) {}

Bounds_t AbstractSonarInterface::azimuthBounds() const {
  checkAzimuthBounds();
  return _azimuthBounds;
}

Bounds_t AbstractSonarInterface::rangeBounds() const {
  checkRangeBounds();
  return _rangeBounds;
}

void AbstractSonarInterface::checkRangeBounds() const {
  if (_rangeBounds == UnsetBounds) {
    auto results = std::minmax_element(ranges().begin(), ranges().end());
    _rangeBounds = std::make_pair(*(results.first), *(results.second));

    _maxRangeSquared = _rangeBounds.second * _rangeBounds.second;
  }
}

void AbstractSonarInterface::checkAzimuthBounds() const {
  if (_azimuthBounds == UnsetBounds) {
    auto results = std::minmax_element(azimuths().begin(), azimuths().end());
    _azimuthBounds = std::make_pair(*(results.first), *(results.second));

    _minAzimuthTan = std::tan(_azimuthBounds.first);
    _maxAzimuthTan = std::tan(_azimuthBounds.second);
  }
}

} // namespace sonar_image_proc