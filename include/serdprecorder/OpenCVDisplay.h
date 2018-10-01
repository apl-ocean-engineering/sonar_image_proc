#pragma once


#include "active_object/active.h"

namespace serdprecorder {

  using std::vector;
  class SerdpRecorder;

  class OpenCVDisplay {
  public:

    OpenCVDisplay( SerdpRecorder &parent, bool enabled = true );

    bool setEnabled( bool e ) { return _enabled = e; }

    void callDisplay( vector< cv::Mat > mats )
    { if( _enabled) _thread->send( std::bind(&OpenCVDisplay::display, this, mats) ); }

    float setPreviewScale( float scale )
    { _previewScale = scale;
      return _previewScale; }


  protected:

    void display( vector< cv::Mat > mats );

    void resizeImage( const cv::Mat &rawImage, cv::Mat &scaledImage );

  private:

    bool _enabled;
    float _previewScale;
    SerdpRecorder &_parent;

    std::unique_ptr<active_object::Active> _thread;

  };


}
