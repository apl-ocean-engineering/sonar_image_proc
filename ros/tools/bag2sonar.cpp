#include <cv_bridge/cv_bridge.h>
#include <marine_acoustic_msgs/ProjectedSonarImage.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sonar_image_proc/sonar_image_msg_interface.h>

#include <boost/program_options.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

#include "sonar_image_proc/SonarDrawer.h"

namespace po = boost::program_options;

using std::string;
using std::vector;

class OutputWrapper {
 public:
  virtual void write(
      const marine_acoustic_msgs::ProjectedSonarImage::ConstPtr &msg,
      const cv::Mat &mat) = 0;
};

class BagOutput : public OutputWrapper {
 public:
  BagOutput(const std::string &bagfile, const std::string topic)
      : _bag(bagfile, rosbag::bagmode::Write), _topic(topic) {}

  void write(const marine_acoustic_msgs::ProjectedSonarImage::ConstPtr &msg,
             const cv::Mat &mat) override {
    cv_bridge::CvImage img_bridge(msg->header,
                                  sensor_msgs::image_encodings::RGB8, mat);

    sensor_msgs::Image output_msg;
    img_bridge.toImageMsg(output_msg);

    // Retain original message's timestamp
    _bag.write(_topic, msg->header.stamp, output_msg);
  }

  rosbag::Bag _bag;
  std::string _topic;
};

void print_help(const po::options_description &description) {
  std::cout << "Usage:" << std::endl;
  std::cout << std::endl;
  std::cout << "   bag2sonar [options] <input file(s)>" << std::endl;
  std::cout << std::endl;
  std::cout << description;
  exit(0);
}

int main(int argc, char **argv) {
  po::options_description public_description("Draw sonar from a bagfile");

  // The following code with three differente po::option_descriptions in a
  // slight-of-hand to hide the positional argment "input-files"
  // otherwise it shows up in print_help() which is ugly
  //
  // clang-format off
  public_description.add_options()
    ("help,h", "Display this help message")
    ("logscale,l", po::bool_switch()->default_value(true), "Do logscale")
    ("min-db", po::value<float>()->default_value(0), "Min db")
    ("max-db", po::value<float>()->default_value(0), "Max db")
    ("osd", po::bool_switch()->default_value(true), "If set, include the on-screen display in output")
    ("output-bag,o", po::value<string>(), "Name of output bagfile")
    ("output-topic,t", po::value<string>()->default_value("/drawn_sonar"), "Topic for images in output bagfile");

  po::options_description hidden_description("");
  hidden_description.add_options()
    ("input-files", po::value<std::vector<std::string>>()->required(), "Input files");
  // clang-format on

  po::options_description full_description("");
  full_description.add(public_description).add(hidden_description);

  po::positional_options_description p;
  p.add("input-files", -1);

  po::variables_map vm;

  try {
    po::store(po::command_line_parser(argc, argv)
                  .options(full_description)
                  .positional(p)
                  .run(),
              vm);
    po::notify(vm);

  } catch (const boost::program_options::required_option &e) {
    // This exception occurs when a required option (input-files) isn't supplied
    // Catch that and display a help message instead.
    print_help(public_description);
  } catch (const std::exception &e) {
    // Catch any other random errors and exit
    std::cerr << e.what() << std::endl;
    exit(-1);
  }

  if (vm.count("help")) {
    print_help(public_description);
  } else if (vm.count("input-files")) {
    if (vm.count("input-files") > 1) {
      std::cerr << "Can only process one file at a time" << std::endl;
      exit(-1);
    }

    std::vector<std::shared_ptr<OutputWrapper>> outputs;

    if (vm.count("output-bag")) {
      outputs.push_back(std::make_shared<BagOutput>(
          vm["output-bag"].as<string>(), vm["output-topic"].as<string>()));
    }

    sonar_image_proc::SonarDrawer sonar_drawer;
    std::unique_ptr<sonar_image_proc::SonarColorMap> color_map(
        new sonar_image_proc::InfernoColorMap);

    std::vector<std::string> files =
        vm["input-files"].as<std::vector<std::string>>();
    for (std::string file : files) {
      std::cout << "Processing input file " << file << std::endl;

      rosbag::Bag bag(file, rosbag::bagmode::Read);

      std::cout << "Bagfile " << file << " is " << bag.getSize() << " bytes"
                << std::endl;

      rosbag::View view(
          bag, rosbag::TypeQuery("marine_acoustic_msgs/ProjectedSonarImage"));

      int count = 0;

      BOOST_FOREACH (rosbag::MessageInstance const m, view) {
        marine_acoustic_msgs::ProjectedSonarImage::ConstPtr msg =
            m.instantiate<marine_acoustic_msgs::ProjectedSonarImage>();

        sonar_image_proc::SonarImageMsgInterface interface(msg);
        if (vm["logscale"].as<bool>()) {
          interface.do_log_scale(vm["min-db"].as<float>(),
                                 vm["max-db"].as<float>());
        }

        cv::Mat rectMat =
            sonar_drawer.drawRectSonarImage(interface, *color_map);

        cv::Mat sonarMat = sonar_drawer.remapRectSonarImage(interface, rectMat);

        cv::Mat outMat;
        if (vm["osd"].as<bool>()) {
          outMat = sonar_drawer.drawOverlay(interface, sonarMat);
        } else {
          outMat = sonarMat;
        }

        for (auto output : outputs) {
          output->write(msg, outMat);
        }

        count++;

        if ((count % 100) == 0) {
          std::cout << "Processed " << count << " sonar frames" << std::endl;
        }
      }

      bag.close();
    }
  }

  exit(0);
}
