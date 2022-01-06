// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include <opencv2/core.hpp>

#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

using sonar_image_proc::AbstractSonarInterface;

struct SonarColorMap {
    // A ColorMap is a mapping from one pixel of sonar data to
    // one pixel in the image.  Each virtual function in
    // this class corresponds to one possible OpenCV pixel format.
    //
    // By default "lookup_cv32fc1()" is the only operation
    // implemented in this version.  It returns the sonar pixel
    // as a float (in the interval [0,1])
    // which OpenCV will interpret as a greyscale.
    //
    // The other virtuals are naive wrappers which convert that float
    // to other OpenCV formats ... though still representing the data
    // as a greyscale.
    //
    // Inherited classes can override any/all of the classes to
    // cover other input/output combinations.

    // The "native" operation returns a single greyscale values a as float
    virtual float lookup_cv32fc1(const AbstractSonarInterface &ping,
                                int bearing_idx, int range_idx) const {
        return ping.intensity_float(bearing_idx, range_idx);
    }

    virtual cv::Vec3b lookup_cv8uc3(const AbstractSonarInterface &ping,
                                    int bearing_idx, int range_idx) const {
        const auto f = lookup_cv32fc1(ping, bearing_idx, range_idx);
        return cv::Vec3b(f*255, f*255, f*255);
    }

    virtual cv::Vec3f lookup_cv32fc3(const AbstractSonarInterface &ping,
                                    int bearing_idx, int range_idx) const {
        const auto f = lookup_cv32fc1(ping, bearing_idx, range_idx);
        return cv::Vec3f(f, f, f);
    }
};

//=== Implementations of specific color maps ===
struct MitchellColorMap : public SonarColorMap {
    cv::Vec3b lookup_cv8uc3(const AbstractSonarInterface &ping,
                            int bearing_idx, int range_idx) const override {
        const auto i = ping.intensity_float(bearing_idx, range_idx);
        return cv::Vec3b( 1-i, i, i );
    }
};

// Use color map from
//    https://github.com/BIDS/colormap/blob/master/colormaps.py
//
// As used in matplotlib
// As released under a CC0 license
struct InfernoColorMap : public SonarColorMap {
    static const float _inferno_data_float[][3];
    static const float _inferno_data_uint8[][3];

    // Minor optimization ... don't go through the intermediate Scalar
    cv::Vec3b lookup_cv8uc3(const AbstractSonarInterface &ping,
                           int bearing_idx, int range_idx) const override {
        const auto i = ping.intensity_uint8(bearing_idx, range_idx);
        return cv::Vec3b(_inferno_data_uint8[i][0],
                            _inferno_data_uint8[i][1],
                            _inferno_data_uint8[i][2]);
    }

    cv::Vec3f lookup_cv32fc3(const AbstractSonarInterface &ping,
                           int bearing_idx, int range_idx) const override {
        const auto i = ping.intensity_uint8(bearing_idx, range_idx);
        return cv::Vec3f(_inferno_data_float[i][0],
                            _inferno_data_float[i][1],
                            _inferno_data_float[i][2]);
    }
};


struct InfernoSaturationColorMap : public InfernoColorMap {

    // Minor optimization ... don't go through the intermediate Scalar
    cv::Vec3b lookup_cv8uc3(const AbstractSonarInterface &ping,
                           int bearing_idx, int range_idx) const override {
        const auto i = ping.intensity_uint8(bearing_idx, range_idx);
        if (i == 255) {
            return cv::Vec3b(0, 255, 0);
        } else {
            return cv::Vec3b(_inferno_data_uint8[i][0],
                                _inferno_data_uint8[i][1],
                                _inferno_data_uint8[i][2]);
        }
    }

    cv::Vec3f lookup_cv32fc3(const AbstractSonarInterface &ping,
                           int bearing_idx, int range_idx) const override {
        const auto i = ping.intensity_uint8(bearing_idx, range_idx);
        if (i == 255) {
            return cv::Vec3f(0.0, 1.0, 0.0);
        } else {
            return cv::Vec3f(_inferno_data_float[i][0],
                                _inferno_data_float[i][1],
                                _inferno_data_float[i][2]);
        }
    }
};



}  // namespace sonar_image_proc
