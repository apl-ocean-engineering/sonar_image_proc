#pragma once

#include <atomic>

#include <DeckLinkAPI.h>

#include "libbmsdi/bmsdi_message.h"

#include "InputHandler.h"
#include "OutputHandler.h"
#include "SDICameraControl.h"


namespace libblackmagic {

  // OO-ish abstraction around a Blackmagic "DeckLink"
  // e.g. one capture card / source.
  // It's really just a container for Blackmagic API IDeckLink,
  // IDeckLinkInput and IDeckLinkOutput instances, plus the callback handlers
  // InputHandler and OutputHandler
  //
  class DeckLink {
  public:

  	DeckLink( int cardno = 0 );
    ~DeckLink();

    // Delete copy operators
    DeckLink( const DeckLink & ) = delete;
    DeckLink &operator=( const DeckLink & ) = delete;

    bool do3D() const     { return _do3D; }
    void set3D(bool s)    { _do3D = s; }

    int cardNo() const    { return _cardNo; }

    // Input and output will be created automatically with defaults unless these
    // functions are called first.
    bool  createVideoInput( const BMDDisplayMode desiredMode = bmdModeHD1080p2997 );
    bool createVideoOutput( const BMDDisplayMode desiredMode = bmdModeHD1080p2997 );

    // These start and stop the input streams
    bool startStreams();
    void stopStreams();

    // Lazy constructors
    InputHandler &inputHandler();
    OutputHandler &outputHandler();


    // // Pull images from _InputHandler
    virtual bool grab( void );
    virtual int getRawImage( int i, cv::Mat &mat );


  protected:

    // Lazy constructors
    IDeckLink *deckLink();
    IDeckLinkInput *deckLinkInput();
    IDeckLinkOutput *deckLinkOutput();

    // These can be called expicitly, otherwise a default will be
    // lazy-constructed when needed
    bool    createDeckLink();


  private:

    cv::Mat _grabbedImage;

    int _cardNo;
    bool _do3D;

    // For now assume an object uses just one Decklink board
    // Stupid COM model precludes use of auto ptrs
    IDeckLink *_deckLink;
    IDeckLinkInput *_deckLinkInput;
    IDeckLinkOutput *_deckLinkOutput;

    BMDTimeScale _outputTimeScale;
    BMDTimeValue _outputTimeValue;

    InputHandler *_inputHandler;
    OutputHandler *_outputHandler;

  };

}
