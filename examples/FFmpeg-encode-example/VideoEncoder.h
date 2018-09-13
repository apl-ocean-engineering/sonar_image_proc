#pragma once

/*
FFmpeg simple Encoder
*/


#include "ffmpegInclude.h"
#include <string>

namespace Encoder {

  class VideoEncoder {
  public:

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

    // Add video stream
    AVStream *AddVideoStream(AVFormatContext *pContext, AVCodecID codec_id);
    // Open Video Stream
    bool OpenVideo(AVFormatContext *oc, AVStream *pStream);
    // Allocate memory
    AVFrame * CreateFFmpegPicture(AVPixelFormat pix_fmt, int nWidth, int nHeight);
    // Close video stream
    void CloseVideo(AVFormatContext *pContext, AVStream *pStream);
    // Add audio stream
    AVStream * AddAudioStream(AVFormatContext *pContext, AVCodecID codec_id);
    // Open audio stream
    bool OpenAudio(AVFormatContext *pContext, AVStream *pStream);
    // close audio stream
    void CloseAudio(AVFormatContext *pContext, AVStream *pStream);
    // Add video frame
    bool AddVideoFrame(AVFrame * frame, AVCodecContext *pVideoCodec);
    // Add audio samples
    bool AddAudioSample(AVFormatContext *_pFormatContext, AVStream *pStream, const char* soundBuffer, int soundBufferSize);

    // Free resourses.
    void Free();
    bool NeedConvert();


    // output file name
    std::string     outputFilename;
    // output format.
    AVOutputFormat  *_pOutFormat;
    // format context
    AVFormatContext *_pFormatContext;
    // video stream context
    AVStream * pVideoStream;
    // audio streams context
    AVStream * pAudioStream;
    // convert context context
    struct SwsContext *pImgConvertCtx;
    // encode buffer and size
    uint8_t * pVideoEncodeBuffer;
    int nSizeVideoEncodeBuffer;

    // audio buffer and size
    uint8_t * pAudioEncodeBuffer;
    int nSizeAudioEncodeBuffer;


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
