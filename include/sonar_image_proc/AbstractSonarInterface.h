// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>
#include <limits>
#include <utility>

namespace sonar_image_proc {

typedef std::pair<float, float> Bounds_t;
extern const Bounds_t UnsetBounds;

// Abstract class strictly for drawing sonar images
// Designed as a "common abstact type" between the Blueprint
// Subsea SimplePingResult and ROS ImagingSonarMsg
struct AbstractSonarInterface {
 public:
    AbstractSonarInterface()
        : _rangeBounds(UnsetBounds), _azimuthBounds(UnsetBounds)
        {;}

    //
    // azimuths are in **radians**
    //
    virtual const std::vector<float> &azimuths() const = 0;

    int nAzimuth() const        { return azimuths().size(); }
    float azimuth(int n) const  { return azimuths().at(n); }

    Bounds_t azimuthBounds() const;

    float minAzimuth() const    { return azimuthBounds().first; }
    float maxAzimuth() const    { return azimuthBounds().second; }

    // Keep the "bearing" functions as a legacy API which is
    // synonymous with azimuth
    int nBearings() const       { return azimuths().size(); }
    float bearing(int n) const  { return azimuths().at(n);  }

    //
    // ranges are in **meters**
    //
    virtual const std::vector<float> &ranges() const = 0;

    int nRanges() const         { return ranges().size(); }
    float range(int n) const    { return ranges().at(n); }

    Bounds_t rangeBounds() const;

    float minRange() const      { return rangeBounds().first; }
    float maxRange() const      { return rangeBounds().second; }

    //
    // Rather than dive into template magic, it's up to the
    // end user to understand the underlying data type
    // stored in a sonar data format.
    //
    // The "native" format is float ... so intensity_float()
    // is the only pure virtual.  The default implementations of
    // _unit8 and _uint16 calculate from the float;  but
    // depending on the original data format, it may be more efficient
    // to override these implementations
    //
    virtual float intensity_float(int b, int r) const = 0;

    virtual uint8_t intensity_uint8(int b, int r) const {
        return 255*intensity_float(b, r);
    }

    virtual uint16_t intensity_uint16(int b, int r) const {
        return 65535*intensity_float(b, r);
    }

    enum DataType_t {
        TYPE_UINT8,
        TYPE_UINT16,
        TYPE_FLOAT32
    };

    virtual DataType_t data_type() const = 0;

 private:
    // Since we search extensively for the bounds
    // (rather than assuming the first and last are the bounds),
    // cache the results
    mutable Bounds_t _rangeBounds, _azimuthBounds;
};

}  // namespace sonar_image_proc
