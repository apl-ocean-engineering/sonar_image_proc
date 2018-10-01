#pragma once


#include "active_object/active.h"

namespace serdprecorder {

  using std::vector;
  class SerdpRecorder;

  class OpenCVDisplay {
  public:

    OpenCVDisplay( bool active, SerdpRecorder &parent );

    void callDisplay( vector< cv::Mat > mats )
    { if( _active) _thread->send( std::bind(&OpenCVDisplay::display, this, mats) ); }

    float setPreviewScale( float scale )
    { _previewScale = scale;
      return _previewScale; }


  protected:

    void display( vector< cv::Mat > mats );


    void resizeImage( const cv::Mat &rawImage, cv::Mat &scaledImage );

  private:

    bool _active;
    float _previewScale;
    SerdpRecorder &_parent;

    std::unique_ptr<active_object::Active> _thread;

  };


}
