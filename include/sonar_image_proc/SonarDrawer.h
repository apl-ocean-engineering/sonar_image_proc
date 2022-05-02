// Copyright 2021 University of Washington Applied Physics Laboratory
//
// This file contains the class-based API (which is more efficient)
// as it can store and reuse intermediate results.
//
// See "DrawSonar.h" for the function-based API

#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "sonar_image_proc/AbstractSonarInterface.h"
#include "sonar_image_proc/ColorMaps.h"

namespace sonar_image_proc {

using sonar_image_proc::AbstractSonarInterface;

// Function in lib/OverlayImage.cpp
void overlayImage(const cv::Mat &background, const cv::Mat &foreground,
                  cv::Mat &output);

#define OVERLAY_RW(var, tp, set, get)                                          \
  OverlayConfig &set(tp i) {                                                   \
    var = i;                                                                   \
    return *this;                                                              \
  }                                                                            \
  tp get() const { return var; }

class SonarDrawer {
public:
  struct OverlayConfig {
  public:
    int DEFAULT_LINE_THICKNESS = 1;
    float DEFAULT_LINE_ALPHA = 0.5;

    // range spacing of 0 means "calculate automatically"
    float DEFAULT_RANGE_SPACING = 0;

    float DEFAULT_RADIAL_SPACING = 20; // degrees
    bool DEFAULT_RADIAL_AT_ZERO = false;

    float DEFAULT_FONT_SCALE = 1.0;

    OverlayConfig()
        : line_thickness_(DEFAULT_LINE_THICKNESS),
          line_alpha_(DEFAULT_LINE_ALPHA),
          range_spacing_m_(DEFAULT_RANGE_SPACING),
          radial_spacing_deg_(DEFAULT_RADIAL_SPACING),
          radial_at_zero_(DEFAULT_RADIAL_AT_ZERO),
          font_scale_(DEFAULT_FONT_SCALE), line_color_(255, 255, 255) {}

    OVERLAY_RW(line_alpha_, float, setLineAlpha, lineAlpha)
    OVERLAY_RW(line_thickness_, int, setLineThickness, lineThickness)
    OVERLAY_RW(range_spacing_m_, float, setRangeSpacing, rangeSpacing)

    OVERLAY_RW(radial_spacing_deg_, float, setRadialSpacing, radialSpacing)
    OVERLAY_RW(radial_at_zero_, bool, setRadialAtZero, radialAtZero)
    OVERLAY_RW(font_scale_, float, setFontScale, fontScale)

    OverlayConfig &setLineColor(const cv::Vec3b &v) {
      line_color_ = v;
      return *this;
    }
    cv::Vec3b lineColor() const { return line_color_; }

    bool operator!=(const OverlayConfig &other) const {
      return (lineThickness() != other.lineThickness()) ||
             (lineAlpha() != other.lineAlpha()) ||
             (rangeSpacing() != other.rangeSpacing()) ||
             (radialSpacing() != other.radialSpacing()) ||
             (radialAtZero() != other.radialAtZero()) ||
             (fontScale() != other.fontScale()) ||
             (lineColor() != other.lineColor());
    }

  private:
    int line_thickness_;
    float line_alpha_;
    float range_spacing_m_;
    float radial_spacing_deg_;
    bool radial_at_zero_;
    float font_scale_;

    cv::Vec3b line_color_;
  };

  SonarDrawer();

  // Calls drawRectSonarImage followed by remapRectSonarImage inline
  // The intermediate rectangular image is not returned, if required,
  // use the two functions individually...
  cv::Mat drawSonar(const AbstractSonarInterface &ping,
                    const SonarColorMap &colorMap = InfernoColorMap(),
                    const cv::Mat &image = cv::Mat(0, 0, CV_8UC3),
                    bool addOverlay = false);

  // Maps the sonar ping to an RGB image.
  // rectImage is reshaped to be numRanges rows x numBearings columns
  //
  // If rectImage is either 8UC3 or 32FC3, it retains that type, otherwise
  // rectImage is converted to 8UC3
  //
  // Cell (0,0) is the color mapping of the data with the smallest range and
  // smallest (typically, most negative) bearing in the ping.
  //
  // Cell (nRange,0) is the data at the max range, most negative bearing
  //
  // Cell (nRange,nBearing) is the data at the max range, most positive
  // bearing
  //
  cv::Mat drawRectSonarImage(const AbstractSonarInterface &ping,
                             const SonarColorMap &colorMap = InfernoColorMap(),
                             const cv::Mat &rectImage = cv::Mat(0, 0, CV_8UC3));

  cv::Mat remapRectSonarImage(const AbstractSonarInterface &ping,
                              const cv::Mat &rectImage);

  // Creates a copy of sonarImage with the graphical overlay using the
  // configuration in overlayConfig
  cv::Mat drawOverlay(const AbstractSonarInterface &ping,
                      const cv::Mat &sonarImage);

  OverlayConfig &overlayConfig() { return overlay_config_; }

private:
  OverlayConfig overlay_config_;

  // Utility class which can generate and store the two cv::Mats
  // required for the cv::remap() function
  //
  // Also stores meta-information to determine if the map is
  // invalid and needs to be regenerated.
  struct Cached {
  public:
    Cached() { ; }

  protected:
    virtual bool isValid(const AbstractSonarInterface &ping) const;

    // Meta-information to validate map
    std::pair<float, float> _rangeBounds, _azimuthBounds;
    int _numRanges, _numAzimuth;
  };

  struct CachedMap : public Cached {
  public:
    CachedMap() : Cached() { ; }
    typedef std::pair<cv::Mat, cv::Mat> MapPair;

    MapPair operator()(const AbstractSonarInterface &ping);

  private:
    bool isValid(const AbstractSonarInterface &ping) const override;
    void create(const AbstractSonarInterface &ping);

    cv::Mat _scMap1, _scMap2;
  } _map;

  struct CachedOverlay : public Cached {
  public:
    CachedOverlay() : Cached() { ; }

    const cv::Mat &operator()(const AbstractSonarInterface &ping,
                              const cv::Mat &sonarImage,
                              const OverlayConfig &config);

  private:
    bool isValid(const AbstractSonarInterface &ping, const cv::Mat &sonarImage,
                 const OverlayConfig &config) const;

    void create(const AbstractSonarInterface &ping, const cv::Mat &sonarImage,
                const OverlayConfig &config);

    cv::Mat _overlay;
    OverlayConfig _config_used;

  } _overlay;

}; // namespace sonar_image_procclassSonarDrawer

} // namespace sonar_image_proc
