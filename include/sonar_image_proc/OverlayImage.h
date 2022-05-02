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

  CV_Assert(bg.type() == cv::traits::Type<VB>::value &&
            fg.type() == cv::traits::Type<VF>::value && bg.size() == fg.size());

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

// void overlayImage(const cv::Mat &background, const cv::Mat &foreground,
//                   cv::Mat &output) {
//   background.copyTo(output);

//   // Only works with CV_8UC3 for background and
//   // CV_8UC4 for foreground
//   CV_Assert(bg.type() == traits::Type<VT>::value &&
//             bg.size() == fg.size());

//   MatConstIterator_<cv::Vec3b> bit = background.begin<cv::Vec3b>(),
//                                bit_end = background.end<cv::Vec3b>();
//   MatConstIterator_<cv::Vec4b> fit = foreground.begin<cv::Vec4b>();

//   MatIterator_<cv::Vec3b> oit = dst.begin<cv::Vec3b>();

//   for (; bit != bit_end; ++bit, ++fit, ++oit) {

//      fg_pix = *fit, bg_pix = *bit;
//     float alpha = fg_pix[3] * inv_scale, beta = bg_pix[3] * inv_scale;
//     *dst_it =
//         VT(saturate_cast<T>(fg_pix[0] * alpha + bg_pix[0] * beta),
//            saturate_cast<T>(fg_pix[1] * alpha + bg_pix[1] * beta),
//            saturate_cast<T>(fg_pix[2] * alpha + bg_pix[2] * beta),
//            saturate_cast<T>((1 - (1 - alpha) * (1 - beta)) * alpha_scale));
//   }

//   //   // start at the row indicated by location, or at row 0 if location.y
//   is
//   //   // negative.
//   //   for (int fY = 0; (fY < background.rows) && (fY < foreground.rows);
//   ++fY)
//   //   {

//   //     // start at the column indicated by location,

//   //     // or at column 0 if location.x is negative.
//   //     for (int fX = 0; (fX < background.cols) && (fX < foreground.cols);
//   //     ++fX) {

//   //       // determine the opacity of the foregrond pixel, using its fourth
//   //       (alpha)
//   //       // channel.
//   //       float alpha = foreground.at<float>(fY, fX, 3) / 255.;
//   //       //   ((double)foreground
//   //       //        .data[fY * foreground.step + fX * foreground.channels()
//   +
//   //       3]) /
//   //       //   255.;

//   //       // and now combine the background and foreground pixel, using the
//   //       // opacity,

//   //       // but only if opacity > 0.
//   //       for (int c = 0; alpha > 0 && c < output.channels(); ++c) {
//   //         const unsigned char foregroundPx =
//   //             foreground.at<unsigned char>(fY, fX, c);
//   //         const unsigned char backgroundPx =
//   //             background.at<unsigned char>(fY, fX, c);

//   //         output.at<unsigned char>(fY, fX, c) =
//   //             backgroundPx * (1. - alpha) + foregroundPx * alpha;
//   //       }
//   //     }
//   //   }
// }
}; // namespace sonar_image_proc