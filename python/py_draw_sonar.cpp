// #include <pybind11/pybind11.h>
//
// #include <opencv2/highgui/highgui.hpp>
//
// // #include "ndarray_converter.h"
// // #include "serdp_common/DrawSonar.h"
//
// namespace py = pybind11;
//
//
// cv::Mat read_image(std::string image_name) {
// #if CV_MAJOR_VERSION < 4
//   cv::Mat image = cv::imread(image_name, CV_LOAD_IMAGE_COLOR);
// #else
//   cv::Mat image = cv::imread(image_name, cv::IMREAD_COLOR);
// #endif
//   return image;
// }
//
// // class AddClass {
// // public:
// //   AddClass(int value) : value(value) {}
// //
// //   cv::Mat add(cv::Mat input) {
// //     return input + this->value;
// //   }
// //
// // private:
// //   int value;
// // };
//

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <string>

#include "ndarray_converter.h"

#include "draw_sonar/DrawSonar.h"
#include "draw_sonar/DataStructures.h"

namespace py = pybind11;


// Sample functions from examples used to bootstrap the project

cv::Mat read_image(std::string image_name) {
#if CV_MAJOR_VERSION < 4
  cv::Mat image = cv::imread(image_name, CV_LOAD_IMAGE_COLOR);
#else
  cv::Mat image = cv::imread(image_name, cv::IMREAD_COLOR);
#endif
  return image;
}

cv::Mat passthru(cv::Mat image) {
  return image;
}

cv::Mat cloneimg(cv::Mat image) {
  return image.clone();
}

int add(int i, int j) {
    return i + j;
}

class AddClass {
public:
  AddClass(int value) : value(value) {}

  cv::Mat add(cv::Mat input) {
    return input + this->value;
  }

private:
  int value;
};


///
PYBIND11_MODULE(py_serdp_common, m) {

  NDArrayConverter::init_numpy();

  m.def("read_image", &read_image, "A function that read an image",
        py::arg("image"));

  m.def("passthru", &passthru, "Passthru function", py::arg("image"));
  m.def("clone", &cloneimg, "Clone function", py::arg("image"));

  m.def("cppadd", &add, "A function which adds two numbers");

  py::class_<AddClass>(m, "AddClass")
    .def(py::init<int>())
    .def("add", &AddClass::add);


}
