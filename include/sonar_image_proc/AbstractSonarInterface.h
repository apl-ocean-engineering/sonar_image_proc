// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>
#include <limits>
#include <utility>

namespace sonar_image_proc {

// Abstract class strictly for drawing sonar images
// Designed as a "common type" between SimplePingResults and ROS ImagingSonarMsg
struct AbstractSonarInterface {
  typedef std::pair<float, float> Bounds_t;

  static const Bounds_t UnsetBounds;

  AbstractSonarInterface()
    : _rangeBounds(UnsetBounds), _azimuthBounds(UnsetBounds)
    {;}

  //
  // azimuths are in **radians**
  //
  virtual const std::vector<float> &azimuths() const = 0;

  int nBearings() const       { return azimuths().size(); }
  float bearing(int n) const  { return azimuths().at(n);  }

  int nAzimuth() const        { return azimuths().size(); }
  float azimuth(int n) const  { return azimuths().at(n); }

  Bounds_t azimuthBounds() const;

  float minAzimuth() const { return azimuthBounds().first; }
  float maxAzimuth() const { return azimuthBounds().second; }

  //
  // ranges are in **meters**
  //
  virtual const std::vector<float> &ranges() const = 0;

  int nRanges() const      { return ranges().size(); }
  float range(int n) const { return ranges().at(n); }

  Bounds_t rangeBounds() const;

  float minRange() const   { return rangeBounds().first; }
  float maxRange() const   { return rangeBounds().second; }

  virtual uint8_t intensity(int b, int r) const
    { return intensity( (r * nBearings()) + b ); }

 protected:
  // I'm not hugely enamored with this API, might deprecate this
  // and use intensity(b,r) above as the main point of entry.
  //  i _must_ be bearing-major
  virtual uint8_t intensity(int i) const  = 0;

 private:
  // Since we search extensively for the bounds
  // (rather than assuming the first and last are the bounds),
  // cache the results
  mutable Bounds_t _rangeBounds, _azimuthBounds;
};

}  // namespace sonar_image_proc
