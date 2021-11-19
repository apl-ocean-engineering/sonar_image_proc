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

  typedef std::pair<float,float> Bounds_t;

  static const Bounds_t UnsetBounds;

  AbstractSonarInterface()
    : _rangeBounds(UnsetBounds), _azimuthBounds(UnsetBounds)
    {;}

  // bearings are in **radians**
  virtual int nBearings() const = 0;
  virtual float bearing(int n) const = 0;

  int nAzimuth() const       { return nBearings(); }
  float azimuth(int n) const { return bearing(n); }

  Bounds_t azimuthBounds() const;

  float minAzimuth() const { return azimuthBounds().first; }
  float maxAzimuth() const { return azimuthBounds().second; }


  // ranges are in **meters**
  virtual int nRanges() const = 0;
  virtual float range(int n) const = 0;

  Bounds_t rangeBounds() const;

  float minRange() const { return rangeBounds().first; }
  float maxRange() const { return rangeBounds().second; }

  virtual uint8_t intensity(int b, int r) const
    { return intensity( (r * nBearings()) + b ); }

 protected:
  //  i _must_ be bearing-major
  virtual uint8_t intensity(int i) const  = 0;

 private:
  mutable Bounds_t _rangeBounds, _azimuthBounds;
};

}  // namespace sonar_image_proc
