// Copyright 2021 University of Washington Applied Physics Laboratory
//

#include <utility>

#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

const std::pair<float, float> AbstractSonarInterface::UnsetPair = std::pair<float,float>(-1, -1);


std::pair<float, float> AbstractSonarInterface::azimuthBounds() const {
    if (_azimuthBounds == UnsetPair) {
        float minAzimuth = std::numeric_limits<float>::max(),
              maxAzimuth = -std::numeric_limits<float>::max();

        // Ah, wish I had iterators
        for (int i = 0; i < nAzimuth(); i++) {
        const auto a = azimuth(i);
        if (a < minAzimuth) minAzimuth = a;
        if (a > maxAzimuth) maxAzimuth = a;
        }

        _azimuthBounds = std::make_pair(minAzimuth, maxAzimuth);
    }

    return _azimuthBounds;
}



std::pair<float, float> AbstractSonarInterface::rangeBounds() const {
    if (_rangeBounds == UnsetPair) {
        float minRange = std::numeric_limits<float>::max(),
              maxRange = -std::numeric_limits<float>::max();

        // Ah, wish I had iterators
        for (int i = 0; i < nRanges(); i++) {
        if (range(i) < minRange) minRange = range(i);
        if (range(i) > maxRange) maxRange = range(i);
        }

        _rangeBounds = std::make_pair(minRange, maxRange);
    }

    return _rangeBounds;
}


}