/*
FFmpeg simple Encoder
*/

#include <stdio.h>
#include <stdlib.h>
#include "ffmpegInclude.h"
#include <math.h>
#include "VideoEncoder.h"
#include "Settings.h"


#define MAX_AUDIO_PACKET_SIZE (128 * 1024)


bool VideoEncoder::InitFile(std::string& inputFile, std::string& container)
{
	bool res = false;

	const char * filename = inputFile.c_str();
	outputFilename = inputFile;

	// Initialize libavcodec
	av_register_all();

	if (container == std::string("auto"))
	{
		// Create format
		pOutFormat = av_guess_format(NULL, filename, NULL);
	}
	else
	{
		// use contanier
		pOutFormat = av_guess_format(container.c_str(), NULL, NULL);
	}

	if (pOutFormat) 
	{
		// allocate context
		pFormatContext = avformat_alloc_context();
		if (pFormatContext) 
		{    
			pFormatContext->oformat = pOutFormat;
			memcpy(pFormatContext->filename, filename, min(strlen(filename), 
				sizeof(pFormatContext->filename)));

			// Add video and audio stream
			pVideoStream   = AddVideoStream(pFormatContext, pOutFormat->video_codec);
			pAudioStream   = AddAudioStream(pFormatContext, pOutFormat->audio_codec);

			// Set the output parameters (must be done even if no
			// parameters).
			{
				av_dump_format(pFormatContext, 0, filename, 1);

				// Open Video and Audio stream
				res = false;
				if (pVideoStream)
				{
					res = OpenVideo(pFormatContext, pVideoStream);
				}

				res = OpenAudio(pFormatContext, pAudioStream);

				if (res && !(pOutFormat->flags & AVFMT_NOFILE)) 
				{
					if (avio_open(&pFormatContext->pb, filename, AVIO_FLAG_WRITE) < 0) 
					{
						res = false;
						printf("Cannot open file\n");
					}
				}

				if (res)
				{
					// Write file header.
					avformat_write_header(pFormatContext, NULL);
					res = true;
				}
			}    
		}   
	}

	if (!res)
	{
		Free();
		printf("Cannot init file\n");
	}

	return res;
}


bool VideoEncoder::AddFrame(AVFrame* frame, const char* soundBuffer, int soundBufferSize)
{
	bool res = true;
	int nOutputSize = 0;
	AVCodecContext *pVideoCodec = NULL;

	if (pVideoStream && frame && frame->data[0])
	{
		pVideoCodec = pVideoStream->codec;

		if (NeedConvert()) 
		{
			// RGB to YUV420P.
			if (!pImgConvertCtx) 
			{
				pImgConvertCtx = sws_getContext(pVideoCodec->width, pVideoCodec->height,
					PIX_FMT_RGB24,
					pVideoCodec->width, pVideoCodec->height,
					pVideoCodec->pix_fmt,
					SWS_BICUBLIN, NULL, NULL, NULL);
			}
		}

		// Allocate picture.
		pCurrentPicture = CreateFFmpegPicture(pVideoCodec->pix_fmt, pVideoCodec->width, 
			pVideoCodec->height);

		if (NeedConvert() && pImgConvertCtx) 
		{
			// Convert RGB to YUV.
			sws_scale(pImgConvertCtx, frame->data, frame->linesize,
				0, pVideoCodec->height, pCurrentPicture->data, pCurrentPicture->linesize);      
		}

		res = AddVideoFrame(pCurrentPicture, pVideoStream->codec);

		// Free temp frame
		av_free(pCurrentPicture->data[0]);
		av_free(pCurrentPicture);
		pCurrentPicture = NULL;
	}

	// Add sound
	if (soundBuffer && soundBufferSize > 0)
	{
		res = AddAudioSample(pFormatContext, pAudioStream, soundBuffer, soundBufferSize);
	}

	return res;
}


bool VideoEncoder::Finish()
{
	bool res = true;
	// Todo: Maybe you need write audio samples from audioBuffer to file before cloasing.

	if (pFormatContext)
	{
		av_write_trailer(pFormatContext);
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

	if (pFormatContext)
	{
		// close video stream
		if (pVideoStream)
		{
			CloseVideo(pFormatContext, pVideoStream);
		}

		// close audio stream.
		if (pAudioStream)
		{
			CloseAudio(pFormatContext, pAudioStream);
		}

		// Free the streams.
		for(size_t i = 0; i < pFormatContext->nb_streams; i++) 
		{
			av_freep(&pFormatContext->streams[i]->codec);
			av_freep(&pFormatContext->streams[i]);
		}

		if (!(pFormatContext->flags & AVFMT_NOFILE) && pFormatContext->pb) 
		{
			avio_close(pFormatContext->pb);
		}

		// Free the stream.
		av_free(pFormatContext);
		pFormatContext = NULL;
	}
}

AVFrame * VideoEncoder::CreateFFmpegPicture(AVPixelFormat pix_fmt, int nWidth, int nHeight)
{
	AVFrame *picture     = NULL;
	uint8_t *picture_buf = NULL;
	int size;

	picture = avcodec_alloc_frame();
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


bool VideoEncoder::OpenVideo(AVFormatContext *oc, AVStream *pStream)
{
	AVCodec *pCodec;
	AVCodecContext *pContext;

	pContext = pStream->codec;

	// Find the video encoder.
	pCodec = avcodec_find_encoder(pContext->codec_id);
	if (!pCodec) 
	{
		printf("Cannot found video codec\n");
		return false;
	}

	// Open the codec.
	if (avcodec_open2(pContext, pCodec, NULL) < 0) 
	{
		printf("Cannot open video codec\n");
		return false;
	}

	pVideoEncodeBuffer = NULL;      
	if (!(pFormatContext->oformat->flags & AVFMT_RAWPICTURE)) 
	{
		/* allocate output buffer */
		nSizeVideoEncodeBuffer = 10000000;
		pVideoEncodeBuffer = (uint8_t *)av_malloc(nSizeVideoEncodeBuffer);
	}

	return true;
}


void VideoEncoder::CloseVideo(AVFormatContext *pContext, AVStream *pStream)
{
	avcodec_close(pStream->codec);
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
	if (pVideoStream && pVideoStream->codec)
	{
		res = (pVideoStream->codec->pix_fmt != PIX_FMT_RGB24);
	}
	return res;
}


AVStream *VideoEncoder::AddVideoStream(AVFormatContext *pContext, AVCodecID codec_id)
{
	AVCodecContext *pCodecCxt = NULL;
	AVStream *st    = NULL;

	st = avformat_new_stream(pContext, 0);
	if (!st) 
	{
		printf("Cannot add new vidoe stream\n");
		return NULL;
	}

	pCodecCxt = st->codec;
	pCodecCxt->codec_id = (AVCodecID)codec_id;
	pCodecCxt->codec_type = AVMEDIA_TYPE_VIDEO;
	pCodecCxt->frame_number = 0;
	// Put sample parameters.
	pCodecCxt->bit_rate = 2000000;
	// Resolution must be a multiple of two.
	pCodecCxt->width  = W_VIDEO;
	pCodecCxt->height = H_VIDEO;
	/* time base: this is the fundamental unit of time (in seconds) in terms
	of which frame timestamps are represented. for fixed-fps content,
	timebase should be 1/framerate and timestamp increments should be
	identically 1. */
	pCodecCxt->time_base.den = 25;
	pCodecCxt->time_base.num = 1;
	pCodecCxt->gop_size = 12; // emit one intra frame every twelve frames at most

	pCodecCxt->pix_fmt = PIX_FMT_YUV420P;
	if (pCodecCxt->codec_id == AV_CODEC_ID_MPEG2VIDEO) 
	{
		// Just for testing, we also add B frames 
		pCodecCxt->max_b_frames = 2;
	}
	if (pCodecCxt->codec_id == AV_CODEC_ID_MPEG1VIDEO)
	{
		/* Needed to avoid using macroblocks in which some coeffs overflow.
		This does not happen with normal video, it just happens here as
		the motion of the chroma plane does not match the luma plane. */
		pCodecCxt->mb_decision = 2;
	}

	// Some formats want stream headers to be separate.
	if(pContext->oformat->flags & AVFMT_GLOBALHEADER)
	{
		pCodecCxt->flags |= CODEC_FLAG_GLOBAL_HEADER;
	}

	return st;
}


AVStream * VideoEncoder::AddAudioStream(AVFormatContext *pContext, AVCodecID codec_id)
{
	AVCodecContext *pCodecCxt = NULL;
	AVStream *pStream = NULL;

	// Try create stream.
	pStream = avformat_new_stream(pContext, NULL);
	if (!pStream) 
	{
		printf("Cannot add new audio stream\n");
		return NULL;
	}

	// Codec.
	pCodecCxt = pStream->codec;
	pCodecCxt->codec_id = codec_id;
	pCodecCxt->codec_type = AVMEDIA_TYPE_AUDIO;
	// Set format
	pCodecCxt->bit_rate    = 128000;
	pCodecCxt->sample_rate = 44100;
	pCodecCxt->channels    = 1; // mono
	pCodecCxt->sample_fmt  = AV_SAMPLE_FMT_S16;

	nSizeAudioEncodeBuffer = 4 * MAX_AUDIO_PACKET_SIZE;
	if (pAudioEncodeBuffer == NULL)
	{      
		pAudioEncodeBuffer = (uint8_t * )av_malloc(nSizeAudioEncodeBuffer);
	}

	// Some formats want stream headers to be separate.
	if(pContext->oformat->flags & AVFMT_GLOBALHEADER)
	{
		pCodecCxt->flags |= CODEC_FLAG_GLOBAL_HEADER;
	}

	return pStream;
}


bool VideoEncoder::OpenAudio(AVFormatContext *pContext, AVStream *pStream)
{
	AVCodecContext *pCodecCxt = NULL;
	AVCodec *pCodec = NULL;
	pCodecCxt = pStream->codec;

	// Find the audio encoder.
	pCodec = avcodec_find_encoder(pCodecCxt->codec_id);
	if (!pCodec) 
	{
		printf("Cannot open audio codec\n");
		return false;
	}

	// Open it.
	if (avcodec_open2(pCodecCxt, pCodec, NULL) < 0) 
	{
		printf("Cannot open audio codec\n");
		return false;
	}

	if (pCodecCxt->frame_size <= 1) 
	{
		// Ugly hack for PCM codecs (will be removed ASAP with new PCM
		// support to compute the input frame size in samples. 
		audioInputSampleSize = nSizeAudioEncodeBuffer / pCodecCxt->channels;
		switch (pStream->codec->codec_id) 
		{
		case AV_CODEC_ID_PCM_S16LE:
		case AV_CODEC_ID_PCM_S16BE:
		case AV_CODEC_ID_PCM_U16LE:
		case AV_CODEC_ID_PCM_U16BE:
			audioInputSampleSize >>= 1;
			break;
		default:
			break;
		}
		pCodecCxt->frame_size = audioInputSampleSize;
	} 
	else 
	{
		audioInputSampleSize = pCodecCxt->frame_size;
	}

	return true;
}


void VideoEncoder::CloseAudio(AVFormatContext *pContext, AVStream *pStream)
{
	avcodec_close(pStream->codec);
	if (pAudioEncodeBuffer)
	{
		av_free(pAudioEncodeBuffer);
		pAudioEncodeBuffer = NULL;
	}
	nSizeAudioEncodeBuffer = 0;
}


bool VideoEncoder::AddVideoFrame(AVFrame * pOutputFrame, AVCodecContext *pVideoCodec)
{
	bool res = false;

	if (pFormatContext->oformat->flags & AVFMT_RAWPICTURE) 
	{
		// Raw video case. The API will change slightly in the near
		// future for that.
		AVPacket pkt;
		av_init_packet(&pkt);

		pkt.flags |= AV_PKT_FLAG_KEY;
		pkt.stream_index = pVideoStream->index;
		pkt.data= (uint8_t *) pOutputFrame;
		pkt.size= sizeof(AVPicture);

		res = av_interleaved_write_frame(pFormatContext, &pkt);
		res = true;
	} 
	else 
	{
		// Encode
		AVPacket packet;
		packet.data = pVideoEncodeBuffer;
		packet.size = nSizeVideoEncodeBuffer; 

		int nOutputSize = 0;
		// Encode frame to packet.
		int error = avcodec_encode_video2(pVideoCodec, &packet, pOutputFrame, &nOutputSize);
		if (!error && nOutputSize > 0) 
		{
			AVPacket pkt;
			av_init_packet(&pkt);

			if (pVideoCodec->coded_frame->pts != AV_NOPTS_VALUE)
			{
				pkt.pts = AV_NOPTS_VALUE;
			}
			else
			{
				int a = 0;
			}

			if(pVideoCodec->coded_frame->key_frame)
			{
				pkt.flags |= AV_PKT_FLAG_KEY;
			}
			pkt.stream_index = pVideoStream->index;
			pkt.data         = pVideoEncodeBuffer;
			pkt.size         = packet.size;

			// Write packet with frame.
			res = (av_interleaved_write_frame(pFormatContext, &pkt) == 0);
		}
		else 
		{
			res = false;
		}
	}

	return res;
}


bool VideoEncoder::AddAudioSample(AVFormatContext *pFormatContext, AVStream *pStream, 
								  const char* soundBuffer, int soundBufferSize)
{
	AVCodecContext *pCodecCxt = NULL;    
	bool res = true;  

	pCodecCxt       = pStream->codec;

	// Size of packet on bytes.
	// FORMAT s16
	DWORD packSizeInSize = soundBufferSize;

	int nCountSamples    = soundBufferSize / av_get_bytes_per_sample(AV_SAMPLE_FMT_S16);

	// Add current audio frame to previos.
	memcpy(audioBuffer + nAudioBufferSizeCurrent, soundBuffer, soundBufferSize);
	nAudioBufferSizeCurrent += soundBufferSize;

	int nCurrentSize    = nAudioBufferSizeCurrent;
	int nWriteSamples   = 0;
	BYTE * pSoundBuffer = (BYTE *)audioBuffer;

	while (nCurrentSize >= packSizeInSize)
	{
		AVFrame*  pAudioFrame = NULL;

		pAudioFrame = avcodec_alloc_frame();

		// Audio frame should be equal or smaller pCodecCxt->frame_size.
		pAudioFrame->nb_samples = min(pCodecCxt->frame_size / av_get_bytes_per_sample(AV_SAMPLE_FMT_S16), nCountSamples);
		int nBufferShift        = nWriteSamples * av_get_bytes_per_sample(AV_SAMPLE_FMT_S16);
		int nCurrentBufferSize  = pAudioFrame->nb_samples * av_get_bytes_per_sample(AV_SAMPLE_FMT_S16);

		if (avcodec_fill_audio_frame(pAudioFrame, 1, 
			AV_SAMPLE_FMT_S16, 
			(uint8_t *)pSoundBuffer, 
			nCurrentBufferSize, 1) != 0)
		{
			res = false;
			break;
		}

		AVPacket pkt;
		av_init_packet(&pkt);

		pkt.flags |= AV_PKT_FLAG_KEY;
		pkt.stream_index = pStream->index;
		pkt.data = pAudioEncodeBuffer;
		pkt.size = nSizeAudioEncodeBuffer;

		int nOutputSize = 0;
		// Create encoded packet from audio frame.
		int error   = avcodec_encode_audio2(pCodecCxt, &pkt, pAudioFrame, &nOutputSize);

		if (!error && nOutputSize > 0)
		{
			if (pCodecCxt->coded_frame && pCodecCxt->coded_frame->pts != AV_NOPTS_VALUE)
			{
				pkt.pts = av_rescale_q(pCodecCxt->coded_frame->pts, pCodecCxt->time_base, pStream->time_base);
			}

			pkt.stream_index = pStream->index;

			// Write the compressed frame in the media file.
			if (av_interleaved_write_frame(pFormatContext, &pkt) != 0) 
			{
				res = false;
				break;
			}
		}

		nCurrentSize  -= nCurrentBufferSize;  
		pSoundBuffer  += nCurrentBufferSize;      

		nWriteSamples += pAudioFrame->nb_samples;
		avcodec_free_frame(&pAudioFrame);
	}

	// save excess
	memcpy(audioBuffer, audioBuffer + nAudioBufferSizeCurrent - nCurrentSize, nCurrentSize);
	nAudioBufferSizeCurrent = nCurrentSize; 

	return res;
}

