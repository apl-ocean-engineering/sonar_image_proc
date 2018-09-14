/*
FFmpeg simple Encoder
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <algorithm>

#include <iostream>

#include "ffmpegInclude.h"


#include "VideoEncoder.h"
#include "Settings.h"


#define MAX_AUDIO_PACKET_SIZE (128 * 1024)


static AVFrame *alloc_picture(enum AVPixelFormat pix_fmt, int width, int height)
{
    AVFrame *picture;
    int ret;
    picture = av_frame_alloc();
    if (!picture)
        return NULL;
    picture->format = pix_fmt;
    picture->width  = width;
    picture->height = height;
    /* allocate the buffers for the frame data */
    ret = av_frame_get_buffer(picture, 32);
    if (ret < 0) {
        fprintf(stderr, "Could not allocate frame data.\n");
        exit(1);
    }
    return picture;
}

namespace Encoder {

	using namespace std;

	VideoEncoder::VideoEncoder()
		: _pOutFormat( nullptr ),
			_pFormatContext( nullptr ),
			_vost( new OutputStream ),
			_pImgConvertCtx( nullptr ),
	    pCurrentPicture( nullptr ),
	    pVideoEncodeBuffer( nullptr )
  {
    nSizeVideoEncodeBuffer = 0;
    pAudioEncodeBuffer = NULL;
    nSizeAudioEncodeBuffer = 0;
    nAudioBufferSize = 1024 * 1024 * 4;
    audioBuffer      = new char[nAudioBufferSize];
    nAudioBufferSizeCurrent = 0;
  }


bool VideoEncoder::InitFile( const std::string &inputFile, const std::string &container)
{
	bool res = false;

	const char * filename = inputFile.c_str();
	outputFilename = inputFile;

	// Initialize libavcodec
	avcodec_register_all();
	av_register_all();					// Initializes libavformat

	if (container == std::string("auto")) {
		_pOutFormat = av_guess_format(NULL, filename, NULL);
	} else {
		_pOutFormat = av_guess_format(container.c_str(), NULL, NULL);
	}

	if (_pOutFormat == nullptr ) {
		cerr << "Unable to determine output format." << endl;
		return false;
	}

	cout << "Using container format " << _pOutFormat->name << " (" << _pOutFormat->long_name << ")" << endl;

	// allocate context
	_pFormatContext = avformat_alloc_context();
	if (!_pFormatContext) {
		cerr << "Unable to allocate avformat context" << endl;
		return false;
	}

	_pFormatContext->oformat = _pOutFormat;
	memcpy(_pFormatContext->filename, filename, std::min(strlen(filename), sizeof(_pFormatContext->filename)));

	AVCodecID codec_id = _pOutFormat->video_codec;
	if( _pOutFormat->name == std::string("mov") ) codec_id = AV_CODEC_ID_PRORES;

	// Add video and audio stream
	auto result = AddVideoStream( _vost, _pFormatContext, codec_id );
	if( !result ) {
		cerr << "Failed to add video stream";
		return false;
	}
	//pAudioStream   = AddAudioStream(_pFormatContext, _pOutFormat->audio_codec);

	// Set the output parameters (must be done even if no
	// parameters).
	{
		// Open Video and Audio stream
		res = false;

		cout << "Opening video stream" << endl;
		res = OpenVideo(_pFormatContext, _vost );

		av_dump_format(_pFormatContext, 0, filename, 1);


		// if( pAudioStream ) {
		// 	cout << "Opening audio stream" << endl;
		// 	res = OpenAudio(_pFormatContext, pAudioStream);
		// }

		// if (res && !(_pOutFormat->flags & AVFMT_NOFILE))
		// {

			if (avio_open(&_pFormatContext->pb, filename, AVIO_FLAG_WRITE) < 0)	{
				cerr << "Cannot open file" << endl;
				return false;
			}

		// }
		//
		// if (res)
		// {
			cout << "Writing file header" << endl;

			// Write file header.
			avformat_write_header(_pFormatContext, NULL);
			res = true;
		//}
	}


	nSizeVideoEncodeBuffer = 10000000;
	pVideoEncodeBuffer = (uint8_t *)av_malloc(nSizeVideoEncodeBuffer);


	if (!res)
	{
		Free();
		cerr << "Cannot init file" << endl;
	}

	return res;
}


bool VideoEncoder::AddFrame(AVFrame* frame, const char* soundBuffer, int soundBufferSize)
{
	bool res = true;
	int nOutputSize = 0;
	AVCodecContext *pVideoCodec = NULL;

	if (_vost && frame && frame->data[0])
	{
		//pVideoCodec = _pVideoStream->codec;

		if (NeedConvert()) {

			// RGB to YUV420P.
			if (!_pImgConvertCtx)
			{
				_pImgConvertCtx = sws_getContext(_vost->enc->width, _vost->enc->height,
																					AV_PIX_FMT_RGB24,
																					_vost->enc->width, _vost->enc->height,
																					_vost->enc->pix_fmt,
																					SWS_BICUBLIN, NULL, NULL, NULL);
			}
		}

		// Allocate picture.
		pCurrentPicture = CreateFFmpegPicture(_vost->enc->pix_fmt, _vost->enc->width, _vost->enc->height);

		if (NeedConvert() && _pImgConvertCtx)	{
			// Convert RGB to YUV.
			sws_scale(_pImgConvertCtx, frame->data, frame->linesize,
				0, _vost->enc->height, pCurrentPicture->data, pCurrentPicture->linesize);
		}

		res = AddVideoFrame(pCurrentPicture, _vost);

		// Free temp frame
		av_free(pCurrentPicture->data[0]);
		av_free(pCurrentPicture);
		pCurrentPicture = NULL;
	}

	// Add sound
	// if (soundBuffer && soundBufferSize > 0)	{
	// 	res = AddAudioSample(_pFormatContext, pAudioStream, soundBuffer, soundBufferSize);
	// }

	return res;
}


bool VideoEncoder::Finish()
{
	bool res = true;
	// Todo: Maybe you need write audio samples from audioBuffer to file before cloasing.

	if (_pFormatContext)
	{
		av_write_trailer(_pFormatContext);
		Free();
	}

	if (audioBuffer)
	{
		delete[] audioBuffer;
		audioBuffer = NULL;
	}

	return res;
}


void VideoEncoder::Free()
{
	bool res = true;

	if (_pFormatContext)
	{
		// close video stream
		if (_vost) CloseVideo(_pFormatContext, _vost);

		// close audio stream.
		// if (pAudioStream)
		// {
		// 	CloseAudio(_pFormatContext, pAudioStream);
		// }

		// Free the streams.
		for(size_t i = 0; i < _pFormatContext->nb_streams; i++)
		{
			av_freep(&_pFormatContext->streams[i]->codec);
			av_freep(&_pFormatContext->streams[i]);
		}

		if (!(_pFormatContext->flags & AVFMT_NOFILE) && _pFormatContext->pb)
		{
			avio_close(_pFormatContext->pb);
		}

		// Free the stream.
		av_free(_pFormatContext);
		_pFormatContext = NULL;
	}
}

AVFrame * VideoEncoder::CreateFFmpegPicture(AVPixelFormat pix_fmt, int nWidth, int nHeight)
{
	AVFrame *picture     = NULL;
	uint8_t *picture_buf = NULL;
	int size;

	picture = av_frame_alloc();   ///avcodec_alloc_frame();
	if ( !picture)
	{
		printf("Cannot create frame\n");
		return NULL;
	}

	size = avpicture_get_size(pix_fmt, nWidth, nHeight);

	picture_buf = (uint8_t *) av_malloc(size);

	if (!picture_buf)
	{
		av_free(picture);
		printf("Cannot allocate buffer\n");
		return NULL;
	}

	avpicture_fill((AVPicture *)picture, picture_buf,
		pix_fmt, nWidth, nHeight);

	return picture;
}


bool VideoEncoder::OpenVideo(AVFormatContext *oc, OutputStream *ost)
{
	int ret;
	//AVCodecContext *c = ost->enc;

	// AVDictionary *opt = NULL;
	// av_dict_copy(&opt, opt_arg, 0);


	AVCodec *codec = avcodec_find_encoder( ost->enc->codec_id );
	if( codec == nullptr ) {
		cerr << "Unable to find encoder codec" << endl;
		return false;
	}

	/* open the codec */
	ret = avcodec_open2(ost->enc, codec, nullptr);
	//av_dict_free(&opt);
	if (ret < 0) {
			cerr << "Could not open video codec"; //: " <<  av_err2str(ret) << endl;
			return false;;
	}

	/* allocate and init a re-usable frame */
	ost->frame = alloc_picture(ost->enc->pix_fmt, ost->enc->width, ost->enc->height);
	if (!ost->frame) {
			fprintf(stderr, "Could not allocate video frame\n");
			return false;;
	}

	/* If the output format is not YUV420P, then a temporary YUV420P
	 * picture is needed too. It is then converted to the required
	 * output format. */
	ost->tmp_frame = NULL;

	// if (ost->enc->pix_fmt != AV_PIX_FMT_YUV420P) {
	// 		ost->tmp_frame = alloc_picture(AV_PIX_FMT_YUV420P, c->width, c->height);
	// 		if (!ost->tmp_frame) {
	// 				fprintf(stderr, "Could not allocate temporary picture\n");
	// 				exit(1);
	// 		}
	// }

	/* copy the stream parameters to the muxer */
	ret = avcodec_parameters_from_context(ost->st->codecpar, ost->enc);
	if (ret < 0) {
			fprintf(stderr, "Could not copy the stream parameters\n");
			return false;;
	}

	// // AVCodec *pCodec;
	// // AVCodecContext *pContext;
	//
	// AVCodecContext *pContext = stream->codec;
	//
	// // // Find the video encoder.
	// // pCodec = avcodec_find_encoder(pContext->codec_id);
	// // if (!pCodec) {
	// // 	cerr << "Cannot find video codec" << endl;
	// // 	return false;
	// // }
	//
	// // Open the codec.
	// if (avcodec_open2(pContext, pCodec, NULL) < 0)
	// {
	// 	cerr << "Cannot open video codec" << endl;
	// 	return false;
	// }
	//
	// pVideoEncodeBuffer = NULL;
	//
	// // AVFMT_RAWPICTURE is apparently no longer used, so this is always true
	// // if (!(_pFormatContext->oformat->flags & AVFMT_RAWPICTURE))
	// // {
	// 	/* allocate output buffer */

//	}

	return true;
}


void VideoEncoder::CloseVideo(AVFormatContext *pContext, OutputStream *ost)
{
	avcodec_close(ost->enc);

	if (pCurrentPicture)
	{
		if (pCurrentPicture->data)
		{
			av_free(pCurrentPicture->data[0]);
			pCurrentPicture->data[0] = NULL;
		}
		av_free(pCurrentPicture);
		pCurrentPicture = NULL;
	}

	if (pVideoEncodeBuffer)
	{
		av_free(pVideoEncodeBuffer);
		pVideoEncodeBuffer = NULL;
	}
	nSizeVideoEncodeBuffer = 0;
}


bool VideoEncoder::NeedConvert()
{
	bool res = false;
	if (_vost && _vost->enc) {
		res = (_vost->enc->pix_fmt != AV_PIX_FMT_RGB24);
	}
	return res;
}


bool VideoEncoder::AddVideoStream(OutputStream *vost, AVFormatContext *pContext, AVCodecID codec_id)
{
	//AVCodecContext *pCodecCxt = NULL;
	//AVStream *st    = NULL;

	{
		 const AVCodecDescriptor *codecDesc = avcodec_descriptor_get(codec_id);
		 if( codecDesc ) {
			  cout << "Using codec " << codec_id << ": " << codecDesc->name << " (" << codecDesc->long_name << ")"  << endl;
		 } else {
			 cerr << "Could not retrieve codec description for " << codec_id << endl;
			 return false;
		 }
	}


	AVCodec *codec = avcodec_find_encoder( codec_id );
	if( codec == nullptr ) {
		cerr << "Unable to find encoder codec" << endl;
		return false;
	}

	vost->st = avformat_new_stream( pContext, NULL );
	if (!vost->st){
		cerr << "Cannot add new video stream" << endl;
		return false;
	}
	vost->st->id = pContext->nb_streams-1;

	// Settings params, how does this get passed to the codec?

	//AVCodecParameters *params = st->codecpar;

	// st->codecpar->codec_id = codec_id;
	// st->codecpar->width = W_VIDEO;
	// st->codecpar->height = H_VIDEO;
	//
	// st->time_base.den = 25;
	// st->time_base.num = 1;

	AVCodecContext *c = avcodec_alloc_context3(codec);
	if( !c ) {
		cerr << "Cannot create codec context";
		return false;
	}
	vost->enc = c;


	// pCodecCxt->codec_id = (AVCodecID)codec_id;
	// pCodecCxt->codec_type = AVMEDIA_TYPE_VIDEO;
	// pCodecCxt->frame_number = 0;
	// // Put sample parameters.
	// pCodecCxt->bit_rate = 2000000;
	// Resolution must be a multiple of two.

	// This causes a silent crash
	//cparser->width  = W_VIDEO;
	//cparser->height = H_VIDEO;

	//AVCodecParameters *codecPars = stream->codecpar;
	c->codec_id = codec_id;
	c->width = W_VIDEO;
	c->height = H_VIDEO;

	/* time base: this is the fundamental unit of time (in seconds) in terms
	of which frame timestamps are represented. for fixed-fps content,
	timebase should be 1/framerate and timestamp increments should be
	identically 1. */
	c->time_base.den = 25;
	c->time_base.num = 1;
	vost->st->time_base = c->time_base;

	//pCodecCxt->gop_size = 12; // emit one intra frame every twelve frames at most

	const enum AVPixelFormat *pixFmt = codec->pix_fmts;
	cout << "This codec supports pix formats: ";
	for( int i = 0; pixFmt[i] != -1; i++ ) {
		cout << pixFmt[i] << " ";
	}
	cout << endl;

	c->pix_fmt = pixFmt[0];

	// //st->codecpar->format = AV_PIX_FMT_YUV420P;
	if (c->codec_id == AV_CODEC_ID_MPEG2VIDEO) {
		// Just for testing, we also add B frames
		//pCodecCxt->max_b_frames = 2;
	} else if (c->codec_id == AV_CODEC_ID_MPEG1VIDEO) {
		/* Needed to avoid using macroblocks in which some coeffs overflow.
		This does not happen with normal video, it just happens here as
		the motion of the chroma plane does not match the luma plane. */
		//pCodecCxt->mb_decision = 2;
	} else if (c->codec_id == AV_CODEC_ID_PRORES ) {

		// 0 = Proxy
		// 1 = LT
		// 2 = normal
		// 3 = HQ
		c->profile = 2;
	}

	//avcodec_parameters_to_context( st->codec, st->codecpar );

	// Some formats want stream headers to be separate.
	// if(pContext->oformat->flags & AVFMT_GLOBALHEADER)
	// {
	// 	pCodecCxt->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
	// }
	return true;
}

//
// AVStream * VideoEncoder::AddAudioStream(AVFormatContext *pContext, AVCodecID codec_id)
// {
// 	AVCodecContext *pCodecCxt = NULL;
// 	AVStream *pStream = NULL;
//
// 	// Try create stream.
// 	pStream = avformat_new_stream(pContext, NULL);
// 	if (!pStream)
// 	{
// 		printf("Cannot add new audio stream\n");
// 		return NULL;
// 	}
//
// 	// Codec.
// 	pCodecCxt = pStream->codec;
// 	pCodecCxt->codec_id = codec_id;
// 	pCodecCxt->codec_type = AVMEDIA_TYPE_AUDIO;
// 	// Set format
// 	pCodecCxt->bit_rate    = 128000;
// 	pCodecCxt->sample_rate = 44100;
// 	pCodecCxt->channels    = 1; // mono
// 	pCodecCxt->sample_fmt  = AV_SAMPLE_FMT_S16;
//
// 	nSizeAudioEncodeBuffer = 4 * MAX_AUDIO_PACKET_SIZE;
// 	if (pAudioEncodeBuffer == NULL)
// 	{
// 		pAudioEncodeBuffer = (uint8_t * )av_malloc(nSizeAudioEncodeBuffer);
// 	}
//
// 	// Some formats want stream headers to be separate.
// 	if(pContext->oformat->flags & AVFMT_GLOBALHEADER)
// 	{
// 		pCodecCxt->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
// 	}
//
// 	return pStream;
// }
//
//
// bool VideoEncoder::OpenAudio(AVFormatContext *pContext, AVStream *pStream)
// {
// 	AVCodecContext *pCodecCxt = NULL;
// 	AVCodec *pCodec = NULL;
// 	pCodecCxt = pStream->codec;
//
// 	// Find the audio encoder.
// 	pCodec = avcodec_find_encoder(pCodecCxt->codec_id);
// 	if (!pCodec)
// 	{
// 		printf("Cannot open audio codec\n");
// 		return false;
// 	}
//
// 	// Open it.
// 	if (avcodec_open2(pCodecCxt, pCodec, NULL) < 0)
// 	{
// 		printf("Cannot open audio codec\n");
// 		return false;
// 	}
//
// 	if (pCodecCxt->frame_size <= 1)
// 	{
// 		// Ugly hack for PCM codecs (will be removed ASAP with new PCM
// 		// support to compute the input frame size in samples.
// 		audioInputSampleSize = nSizeAudioEncodeBuffer / pCodecCxt->channels;
// 		switch (pStream->codec->codec_id)
// 		{
// 		case AV_CODEC_ID_PCM_S16LE:
// 		case AV_CODEC_ID_PCM_S16BE:
// 		case AV_CODEC_ID_PCM_U16LE:
// 		case AV_CODEC_ID_PCM_U16BE:
// 			audioInputSampleSize >>= 1;
// 			break;
// 		default:
// 			break;
// 		}
// 		pCodecCxt->frame_size = audioInputSampleSize;
// 	}
// 	else
// 	{
// 		audioInputSampleSize = pCodecCxt->frame_size;
// 	}
//
// 	return true;
// }
//
//
// void VideoEncoder::CloseAudio(AVFormatContext *pContext, AVStream *pStream)
// {
// 	avcodec_close(pStream->codec);
// 	if (pAudioEncodeBuffer)
// 	{
// 		av_free(pAudioEncodeBuffer);
// 		pAudioEncodeBuffer = NULL;
// 	}
// 	nSizeAudioEncodeBuffer = 0;
// }
//

bool VideoEncoder::AddVideoFrame(AVFrame *data, OutputStream *ost)
{
	bool res = false;

	// if (_pFormatContext->oformat->flags & AVFMT_RAWPICTURE)
	// {
	// 	// Raw video case. The API will change slightly in the near
	// 	// future for that.
	// 	AVPacket pkt;
	// 	av_init_packet(&pkt);
	//
	// 	pkt.flags |= AV_PKT_FLAG_KEY;
	// 	pkt.stream_index = _pVideoStream->index;
	// 	pkt.data= (uint8_t *) pOutputFrame;
	// 	pkt.size= sizeof(AVPicture);
	//
	// 	res = av_interleaved_write_frame(_pFormatContext, &pkt);
	// 	res = true;
	// }
	// else
	// {

		// Encode
		AVPacket packet;
		packet.data = pVideoEncodeBuffer;
		packet.size = nSizeVideoEncodeBuffer;

		int nOutputSize = 0;
		// Encode frame to packet.
		int error = avcodec_encode_video2(ost->enc, &packet, data, &nOutputSize);
		if (!error && nOutputSize > 0) {
			AVPacket pkt;
			av_init_packet(&pkt);

			if (ost->enc->coded_frame->pts != AV_NOPTS_VALUE) 	{
				pkt.pts = AV_NOPTS_VALUE;
			}	else {
				int a = 0;
			}

			if(ost->enc->coded_frame->key_frame) {
				pkt.flags |= AV_PKT_FLAG_KEY;
			}
			pkt.stream_index = ost->st->index;
			pkt.data         = pVideoEncodeBuffer;
			pkt.size         = packet.size;

			// Write packet with frame.
			res = (av_interleaved_write_frame(_pFormatContext, &pkt) == 0);
		}
		else
		{
			res = false;
		}
	//}

	return res;
}

//
// bool VideoEncoder::AddAudioSample(AVFormatContext *_pFormatContext, AVStream *pStream,
// 								  const char* soundBuffer, int soundBufferSize)
// {
// 	AVCodecContext *pCodecCxt = NULL;
// 	bool res = true;
//
// 	pCodecCxt       = pStream->codec;
//
// 	// Size of packet on bytes.
// 	// FORMAT s16
// 	uint32_t packSizeInSize = soundBufferSize;
//
// 	int nCountSamples    = soundBufferSize / av_get_bytes_per_sample(AV_SAMPLE_FMT_S16);
//
// 	// Add current audio frame to previos.
// 	memcpy(audioBuffer + nAudioBufferSizeCurrent, soundBuffer, soundBufferSize);
// 	nAudioBufferSizeCurrent += soundBufferSize;
//
// 	int nCurrentSize    = nAudioBufferSizeCurrent;
// 	int nWriteSamples   = 0;
// 	uint8_t *pSoundBuffer = (uint8_t *)audioBuffer;
//
// 	while (nCurrentSize >= packSizeInSize)
// 	{
// 		AVFrame*  pAudioFrame = NULL;
//
// 		pAudioFrame = av_frame_alloc();   ///avcodec_alloc_frame();
//
// 		// Audio frame should be equal or smaller pCodecCxt->frame_size.
// 		pAudioFrame->nb_samples = std::min(pCodecCxt->frame_size / av_get_bytes_per_sample(AV_SAMPLE_FMT_S16), nCountSamples);
// 		int nBufferShift        = nWriteSamples * av_get_bytes_per_sample(AV_SAMPLE_FMT_S16);
// 		int nCurrentBufferSize  = pAudioFrame->nb_samples * av_get_bytes_per_sample(AV_SAMPLE_FMT_S16);
//
// 		if (avcodec_fill_audio_frame(pAudioFrame, 1,
// 			AV_SAMPLE_FMT_S16,
// 			(uint8_t *)pSoundBuffer,
// 			nCurrentBufferSize, 1) != 0)
// 		{
// 			res = false;
// 			break;
// 		}
//
// 		AVPacket pkt;
// 		av_init_packet(&pkt);
//
// 		pkt.flags |= AV_PKT_FLAG_KEY;
// 		pkt.stream_index = pStream->index;
// 		pkt.data = pAudioEncodeBuffer;
// 		pkt.size = nSizeAudioEncodeBuffer;
//
// 		int nOutputSize = 0;
// 		// Create encoded packet from audio frame.
// 		int error   = avcodec_encode_audio2(pCodecCxt, &pkt, pAudioFrame, &nOutputSize);
//
// 		if (!error && nOutputSize > 0) {
// 			if (pCodecCxt->coded_frame && pCodecCxt->coded_frame->pts != AV_NOPTS_VALUE)
// 			{
// 				pkt.pts = av_rescale_q(pCodecCxt->coded_frame->pts, pCodecCxt->time_base, pStream->time_base);
// 			}
//
// 			pkt.stream_index = pStream->index;
//
// 			// Write the compressed frame in the media file.
// 			if (av_interleaved_write_frame(_pFormatContext, &pkt) != 0)
// 			{
// 				res = false;
// 				break;
// 			}
// 		}
//
// 		nCurrentSize  -= nCurrentBufferSize;
// 		pSoundBuffer  += nCurrentBufferSize;
//
// 		nWriteSamples += pAudioFrame->nb_samples;
// 		av_frame_free(&pAudioFrame);     //was: avcodec_free_frame(&pAudioFrame);
// 	}
//
// 	// save excess
// 	memcpy(audioBuffer, audioBuffer + nAudioBufferSizeCurrent - nCurrentSize, nCurrentSize);
// 	nAudioBufferSizeCurrent = nCurrentSize;
//
// 	return res;
// }

};
