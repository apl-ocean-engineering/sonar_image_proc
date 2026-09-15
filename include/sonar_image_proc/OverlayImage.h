#include <opencv2/core.hpp>
#include <opencv2/core/traits.hpp>

namespace sonar_image_proc {

using cv::Mat;
using cv::Vec;

// Adapted from the code sample in the OpenCV documentation:
// https://docs.opencv.org/4.x/d3/d63/classcv_1_1Mat.html#a33ee3bc402827f587a5ad64b568d6986
//
// \todo(@amarburg):  Optimize?  Loop unrolling?
//

template <typename T>
void overlayImage(const Mat &bg, const Mat &fg, Mat &dst) {
  typedef Vec<T, 3> VB;
  typedef Vec<T, 4> VF;

  const float alpha_scale = (float)std::numeric_limits<T>::max(),
              inv_scale = 1.f / alpha_scale;

#if (CV_VERSION_MAJOR >= 4)
  CV_Assert(bg.type() == cv::traits::Type<VB>::value &&
            fg.type() == cv::traits::Type<VF>::value && bg.size() == fg.size());
#else
  CV_Assert(bg.type() == cv::DataType<VB>::type &&
            fg.type() == cv::DataType<VF>::type && bg.size() == fg.size());
#endif

  dst.create(bg.size(), bg.type());

  cv::MatConstIterator_<VF> fit = fg.begin<VF>(), fit_end = fg.end<VF>();
  cv::MatConstIterator_<VB> bit = bg.begin<VB>();
  cv::MatIterator_<VB> dst_it = dst.begin<VB>();

  for (; fit != fit_end; ++fit, ++bit, ++dst_it) {
    const auto fg_pix = *fit;
    const auto bg_pix = *bit;

    const float alpha = fg_pix[3] * inv_scale;
    const float beta = 1 - alpha;
    *dst_it = VB(cv::saturate_cast<T>(fg_pix[0] * alpha + bg_pix[0] * beta),
                 cv::saturate_cast<T>(fg_pix[1] * alpha + bg_pix[1] * beta),
                 cv::saturate_cast<T>(fg_pix[2] * alpha + bg_pix[2] * beta));
  }
}

};  // namespace sonar_image_proc
