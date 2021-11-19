// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include <stdint.h>
#include <math.h>
#include <vector>
#include <limits>

namespace sonar_image_proc {

// Abstract class strictly for drawing sonar images
// Designed as a "common type" between SimplePingResults and ROS ImagingSonarMsg
struct AbstractSonarInterface {

  static const std::pair<float,float> UnsetPair;

  AbstractSonarInterface()
    : _rangeBounds(UnsetPair), _azimuthBounds(UnsetPair)
    {;}

  // bearings are in **radians**
  virtual int nBearings() const = 0;
  virtual float bearing(int n) const = 0;

  int nAzimuth() const       { return nBearings(); }
  float azimuth(int n) const { return bearing(n); }

  std::pair<float,float> azimuthBounds() const;

  float minAzimuth() const { return azimuthBounds().first; }
  float maxAzimuth() const { return azimuthBounds().second; }


  // ranges are in **meters**
  virtual int nRanges() const = 0;
  virtual float range(int n) const = 0;

  std::pair<float,float> rangeBounds() const;

  float minRange() const { return rangeBounds().first; }
  float maxRange() const { return rangeBounds().second; }

  virtual uint8_t intensity(int b, int r) const
    { return intensity( (r * nBearings()) + b ); }

 protected:

   // i _must_ be bearing-major
  virtual uint8_t intensity(int i) const  = 0;

 private:
  mutable std::pair<float,float> _rangeBounds, _azimuthBounds;
};

}  // namespace sonar_image_proc
