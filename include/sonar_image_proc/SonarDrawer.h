// Copyright 2021 University of Washington Applied Physics Laboratory
//
// This file contains the "functional" API.
//
// See SonarDrawer.h for a class-based API (which is more efficients)
// as it can store and reuse intermediate results

#pragma once

#include <memory>

#include <opencv2/core/core.hpp>

#include "sonar_image_proc/ColorMaps.h"
#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {


class SonarDrawer {
 public:
    SonarDrawer();

    void drawSonar(const sonar_image_proc::AbstractSonarInterface &ping,
                        cv::Mat &image,
                        const SonarColorMap &colorMap = InfernoColorMap(),
                        const cv::Mat &rectImage = cv::Mat());

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
    void drawSonarRectImage(const sonar_image_proc::AbstractSonarInterface &ping,
                        cv::Mat &rectImage,
                        const SonarColorMap &colorMap = InfernoColorMap());

 private:

    struct CachedMap {
     public:
        CachedMap()
            {;}

        typedef std::pair<cv::Mat,cv::Mat> MapPair;

        MapPair operator()(const sonar_image_proc::AbstractSonarInterface &ping);
        void create(const sonar_image_proc::AbstractSonarInterface &ping);

        bool isValid(const sonar_image_proc::AbstractSonarInterface &ping) const;

     private:
        cv::Mat _scMap1, _scMap2;

        // Meta-information to validate map
        std::pair<float, float> _rangeBounds, _azimuthBounds;
        int _numRanges, _numAzimuth;
    } _map;

};

}  // namespace sonar_image_proc
