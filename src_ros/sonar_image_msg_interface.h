// Copyright 2021 University of Washington Applied Physics Laboratory
//
// An implementation of an AbstractSonarInterface which
// wraps a ROS acoustic_msgs::SonarImage
//

#include "sonar_image_proc/AbstractSonarInterface.h"


namespace draw_sonar {

using namespace std;
using namespace cv;

struct SonarImageMsgInterface : public sonar_image_proc::AbstractSonarInterface {
    explicit SonarImageMsgInterface(const acoustic_msgs::SonarImage::ConstPtr &ping)
      : _ping(ping) {;}

    const std::vector<float> &ranges() const override   { return _ping->ranges; }
    const std::vector<float> &azimuths() const override { return _ping->azimuth_angles; }

 protected:
    virtual uint8_t intensity(int i) const override {
      if (_ping->data_size == 1) {
        return _ping->intensities[i];
      } else if (_ping->data_size == 2) {
        uint16_t d;

        if (_ping->is_bigendian)
          d = (_ping->intensities[i * 2] << 8) | _ping->intensities[i * 2 + 1];
        else
          d = (_ping->intensities[i * 2 + 1] << 8) | _ping->intensities[i * 2];

        // Hacky
        const int shift = 6;
        if (d >= (0x1 << (shift+8))) return 0xFF;

        return (d >> shift);
      } else {
        ROS_ERROR_STREAM("SonarImage has unsupported data_size = " << _ping->data_size);
        return 0;
      }
    }

    acoustic_msgs::SonarImage::ConstPtr _ping;
};

}  // namespace draw_sonar