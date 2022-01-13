// Copyright 2021 University of Washington Applied Physics Laboratory
//
// An implementation of an AbstractSonarInterface which
// wraps a ROS acoustic_msgs::SonarImage
//

#include <vector>

#include "sonar_image_proc/AbstractSonarInterface.h"


namespace draw_sonar {

using std::vector;
using sonar_image_proc::AbstractSonarInterface;
using acoustic_msgs::SonarImage;

struct SonarImageMsgInterface : public AbstractSonarInterface {
    explicit SonarImageMsgInterface(const SonarImage::ConstPtr &ping)
        : _ping(ping), _gain(1)
    {;}

    const std::vector<float> &ranges() const override {
      return _ping->ranges;
    }

    const std::vector<float> &azimuths() const override {
      return _ping->azimuth_angles;
    }

    // Optimized version...
    uint8_t intensity_uint8(int b, int r) const override {
        const auto i = index(b, r);

        if (_ping->data_size == 1) {
            return _ping->intensities[i] * _gain;
        } else if (_ping->data_size == 2) {
            return 255 * intensity_float(b, r);
        }

        return 0;
    }

    uint16_t intensity_uint16(int b, int r) const override {
        const auto i = index(b, r);

        if (_ping->data_size == 1) {
            return (_ping->intensities[i] << 8) * _gain;
        } else if (_ping->data_size == 2) {
            return (static_cast<uint16_t>(_ping->intensities[i]) |
                    (static_cast<uint16_t>(_ping->intensities[i+1]) << 8)) * _gain;
        }
        return 0;
    }

    virtual float intensity_float(int b, int r) const {
        const auto i = index(b, r);

        if (_ping->data_size == 1) {
            return static_cast<float>(_ping->intensities[i]) * _gain/255.0;
        } else if (_ping->data_size == 2) {
            // Data is stored LSB
            const uint16_t v = (static_cast<uint16_t>(_ping->intensities[i]) |
                               (static_cast<uint16_t>(_ping->intensities[i+1]) << 8));
            return static_cast<float>(v)*_gain / 65535.0;
        }
        return 0.0;
    }

 protected:
    SonarImage::ConstPtr _ping;

    unsigned int _gain;

    size_t index(int b, int r) const {
        assert( (_ping->data_size == 1) || (_ping->data_size == 2));
        return _ping->data_size*((r * nBearings()) + b);
    }
};

}  // namespace draw_sonar