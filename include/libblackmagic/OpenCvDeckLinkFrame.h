
#include <g3log/g3log.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace libblackmagic {

  // Based on CvMatDeckLinkVideoFrame from
  // https://github.com/ull-isaatc/blackmagic-test

  class CvMatDeckLinkVideoFrame : public IDeckLinkVideoFrame {
  public:
       cv::Mat mat;

       CvMatDeckLinkVideoFrame(int row, int cols)
           : mat(row, cols, CV_8UC4)
       {}

       //
       // IDeckLinkVideoFrame
       //

       long GetWidth()
       { return mat.rows; }

       long GetHeight()
       { return mat.cols; }

       long GetRowBytes()
       { return mat.step; }

       BMDPixelFormat GetPixelFormat()
       {
         return bmdFormat10BitYUV;  //bmdFormat8BitBGRA;
       }

       BMDFrameFlags GetFlags()
       {
         return 0;
       }

       HRESULT GetBytes(void **buffer)
       {
           *buffer = mat.data;
           return S_OK;
       }

       HRESULT GetTimecode(BMDTimecodeFormat format,
           IDeckLinkTimecode **timecode)
       { *timecode = nullptr; return S_OK; }

       HRESULT GetAncillaryData(IDeckLinkVideoFrameAncillary **ancillary)
       { *ancillary = nullptr; return S_OK; }

       HRESULT QueryInterface(REFIID iid, LPVOID *ppv)
   { return E_NOINTERFACE; }

   ULONG AddRef()
   { mat.addref(); return 0; } //*mat.refcount; }

   ULONG Release()
   {
       mat.release();
       if (*mat.data == 0) delete this;
       return 0;  //*mat.refcount;
   }

  };
}
