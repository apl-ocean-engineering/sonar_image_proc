// Copyright 2021 University of Washington Applied Physics Laboratory
//

#pragma once

#include <opencv2/core.hpp>

#include "sonar_image_proc/AbstractSonarInterface.h"

namespace sonar_image_proc {

using sonar_image_proc::AbstractSonarInterface;

struct SonarColorMap {
    // The "native" operation is a simple greyscale based on floats
    virtual float lookup_float(const AbstractSonarInterface &ping,
                                int bearing_idx, int range_idx) const {
        return ping.intensity_float(bearing_idx, range_idx);
    }

    // The 3-channel implementations are simple copies of the
    // greyscale version
    virtual cv::Scalar lookup_scalar(const AbstractSonarInterface &ping,
                                    int bearing_idx, int range_idx) const {
        const float f = lookup_float(ping, bearing_idx, range_idx);
        return cv::Scalar(f, f, f);
    }

    virtual cv::Vec3b lookup_vec3b(const AbstractSonarInterface &ping,
                                    int bearing_idx, int range_idx) const {
        const auto f = lookup_float(ping, bearing_idx, range_idx);
        return cv::Vec3b(f*255, f*255, f*255);
    }

    virtual cv::Vec3f lookup_vec3f(const AbstractSonarInterface &ping,
                                    int bearing_idx, int range_idx) const {
        const auto f = lookup_float(ping, bearing_idx, range_idx);
        return cv::Vec3f(f, f, f);
    }
};

//=== Implementations of specific color maps ===
struct MitchellColorMap : public SonarColorMap {
    cv::Scalar lookup_scalar(const AbstractSonarInterface &ping,
                            int bearing_idx, int range_idx) const override {
        const auto i = ping.intensity_float(bearing_idx, range_idx);
        return cv::Scalar( 1-i, i, i );
    }
};

// Use color map from
//    https://github.com/BIDS/colormap/blob/master/colormaps.py
//
// As used in matplotlib
// As released under a CC0 license
struct InfernoColorMap : public SonarColorMap {
    static const float _inferno_data[][3];

    cv::Scalar lookup_scalar(const AbstractSonarInterface &ping,
                            int bearing_idx, int range_idx) const override {
        const auto i = ping.intensity_uint8(bearing_idx, range_idx);
        return cv::Scalar(_inferno_data[i][0],
                            _inferno_data[i][1],
                            _inferno_data[i][2]);
    }

    // Minor optimization ... don't go through the intermediate Scalar
    cv::Vec3b lookup_vec3b(const AbstractSonarInterface &ping,
                           int bearing_idx, int range_idx) const override {
        const auto i = ping.intensity_uint8(bearing_idx, range_idx);
        return cv::Vec3b(_inferno_data[i][0]*255,
                            _inferno_data[i][1]*255,
                            _inferno_data[i][2]*255);
    }

    cv::Vec3f lookup_vec3f(const AbstractSonarInterface &ping,
                           int bearing_idx, int range_idx) const override {
        const auto i = ping.intensity_uint8(bearing_idx, range_idx);
        return cv::Vec3f(_inferno_data[i][0],
                            _inferno_data[i][1],
                            _inferno_data[i][2]);
    }
};

}  // namespace sonar_image_proc
