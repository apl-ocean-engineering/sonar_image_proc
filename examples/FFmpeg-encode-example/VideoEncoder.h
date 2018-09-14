#pragma once

/*
FFmpeg simple Encoder
*/


#include "ffmpegInclude.h"
#include <string>

namespace Encoder {

  class VideoEncoder {
  public:

    struct OutputStream {
        AVStream *st;
        AVCodecContext *enc;

        /* pts of the next frame that will be generated */
        // int64_t next_pts;
        // int samples_count;

        AVFrame *frame;
        AVFrame *tmp_frame;

        // float t, tincr, tincr2;
        // struct SwsContext *sws_ctx;
        // struct SwrContext *swr_ctx;
    };


    VideoEncoder();

    virtual ~VideoEncoder()
    {
      Finish();
    }

    // init output file
    bool InitFile(const std::string &inputFile, const std::string &container);
    // Add video and audio data
    bool AddFrame(AVFrame* frame, const char* soundBuffer, int soundBufferSize);
    // end of output
    bool Finish();

  private:

    // Allocate memory
    AVFrame * CreateFFmpegPicture(AVPixelFormat pix_fmt, int nWidth, int nHeight);

    // Add video stream
    bool AddVideoStream(OutputStream *ost, AVFormatContext *pContext, AVCodecID codec_id);
    // Open Video Stream
    bool OpenVideo(AVFormatContext *oc, OutputStream *ost);

    // Close video stream
    void CloseVideo(AVFormatContext *oc, OutputStream *ost);

    // Add audio stream
    // AVStream * AddAudioStream(AVFormatContext *pContext, AVCodecID codec_id);
    // // Open audio stream
    // bool OpenAudio(AVFormatContext *pContext, AVStream *pStream);
    // // close audio stream
    // void CloseAudio(AVFormatContext *pContext, AVStream *pStream);

    //== Add frames to movie ==

    // Add video frame to stream
    bool AddVideoFrame(AVFrame *data, OutputStream *ost);

    // Add audio samples to stream
    //bool AddAudioSample(AVFormatContext *_pFormatContext, AVStream *pStream, const char* soundBuffer, int soundBufferSize);


    // Free all resourses.
    void Free();
    bool NeedConvert();


    // output file name
    std::string     outputFilename;

    // output format.
    AVOutputFormat  *_pOutFormat;

    // format context
    AVFormatContext *_pFormatContext;

    OutputStream *_vost;

    // video stream context
    // AVStream * _pVideoStream;
    // // audio streams context
    AVStream * pAudioStream;

    // convert context context
    struct SwsContext *_pImgConvertCtx;
    // encode buffer and size
    uint8_t * pVideoEncodeBuffer;
    int nSizeVideoEncodeBuffer;

    // audio buffer and size
    uint8_t * pAudioEncodeBuffer;
    int nSizeAudioEncodeBuffer;


    int numFrames;

    // count of sample
    int audioInputSampleSize;
    // current picture
    AVFrame *pCurrentPicture;

    // audio buffer. Save rest samples in audioBuffer from previous audio frame.
    char* audioBuffer;
    int   nAudioBufferSize;
    int   nAudioBufferSizeCurrent;

  };

};
