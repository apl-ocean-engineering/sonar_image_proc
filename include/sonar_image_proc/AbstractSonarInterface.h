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

struct AzimuthRangeIndices {
  AzimuthRangeIndices(size_t a, size_t r)
    : _rangeIdx(r), _azimuthIdx(a) {;}

  size_t range() const   { return _rangeIdx;}
  size_t azimuth() const { return _azimuthIdx; }

  size_t _rangeIdx, _azimuthIdx;
};

// Abstract class strictly for drawing sonar images
// Designed as a "common abstact type" between the Blueprint
// Subsea SimplePingResult and ROS ImagingSonarMsg
struct AbstractSonarInterface {
 public:
  AbstractSonarInterface();

  enum DataType_t {
        TYPE_UINT8,
        TYPE_UINT16,
        TYPE_UINT32,
        TYPE_FLOAT32
  };

  virtual DataType_t data_type() const = 0;

  //
  // azimuths are in **radians**
  //
  virtual const std::vector<float> &azimuths() const = 0;

  int nBearings() const       { return azimuths().size(); }    __attribute__ ((deprecated));
  float bearing(int n) const  { return azimuths().at(n);  }    __attribute__ ((deprecated));

  int nAzimuth() const        { return azimuths().size(); }
  int nAzimuths() const       { return azimuths().size(); } // Whoops, should be consistent
  float azimuth(int n) const  { return azimuths().at(n); }

  Bounds_t azimuthBounds() const;

  float minAzimuth() const { return azimuthBounds().first; }
  float maxAzimuth() const { return azimuthBounds().second; }

  float minAzimuthTan() const { checkAzimuthBounds();  return _minAzimuthTan; }
  float maxAzimuthTan() const { checkAzimuthBounds();  return _maxAzimuthTan; }

  //
  // ranges are in **meters**
  //
  virtual const std::vector<float> &ranges() const = 0;

  int nRanges() const      { return ranges().size(); }
  float range(int n) const { return ranges().at(n); }

  Bounds_t rangeBounds() const;

  float minRange() const   { return rangeBounds().first; }
  float maxRange() const   { return rangeBounds().second; }

  float maxRangeSquared() const {
    checkRangeBounds();
    return _maxRangeSquared;
  }

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
  virtual float intensity_float(const AzimuthRangeIndices &idx) const = 0;

  virtual uint8_t intensity_uint8(const AzimuthRangeIndices &idx) const {
      return INT8_MAX*intensity_float(idx);
  }

  virtual uint16_t intensity_uint16(const AzimuthRangeIndices &idx) const {
      return INT16_MAX*intensity_float(idx);
  }

  virtual uint32_t intensity_uint32(const AzimuthRangeIndices &idx) const {
      return INT32_MAX*intensity_float(idx);
  }

  // Trivial wrappers.  These will be deprecated eventually
  float intensity_float(size_t a, size_t r)     const { return intensity_float(AzimuthRangeIndices(a,r)); }     __attribute__ ((deprecated));
  uint8_t intensity_uint8(size_t a, size_t r)   const { return intensity_uint8(AzimuthRangeIndices(a,r)); }     __attribute__ ((deprecated));
  uint16_t intensity_uint16(size_t a, size_t r) const { return intensity_uint16(AzimuthRangeIndices(a,r)); }    __attribute__ ((deprecated));

 private:
  // In a few cases, need to "check and potentially calculate cached
  // value" without actually getting the value
  void checkRangeBounds() const;
  void checkAzimuthBounds() const;

 private:
  // Since we search extensively for the bounds
  // (rather than assuming the first and last are the bounds),
  // cache the results
  mutable Bounds_t _rangeBounds, _azimuthBounds;
  mutable float _minAzimuthTan, _maxAzimuthTan;
  mutable float _maxRangeSquared;
};

}  // namespace sonar_image_proc
