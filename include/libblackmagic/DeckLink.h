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

  	DeckLink();
    ~DeckLink();

    // Delete copy operators
    DeckLink( const DeckLink & ) = delete;
    DeckLink &operator=( const DeckLink & ) = delete;

    // These can be called expicitly, otherwise a default will be
    // lazy-constructed when needed
    bool    createDeckLink( int cardno = 0 );
    bool  createVideoInput( const BMDDisplayMode desiredMode = bmdModeHD1080p2997, bool do3D = false );
    bool createVideoOutput( const BMDDisplayMode desiredMode = bmdModeHD1080p2997, bool do3D = false );

    // These start and stop the input streams
    bool startStreams();
    void stopStreams();

    InputHandler &inputHandler()    { return *_inputHandler; }
    OutputHandler &outputHandler()  { return *_outputHandler; }

    // virtual int numFrames( void ) const { return -1; }

    // // Pull images from _InputHandler
    // virtual bool grab( void );
    //
    // virtual int getRawImage( int i, cv::Mat &mat );
    //
    // virtual ImageSize imageSize( void ) const;

  protected:

    // Lazy-constructors
    IDeckLink *deckLink();
    IDeckLinkInput *deckLinkInput();
    IDeckLinkOutput *deckLinkOutput();



  private:

    cv::Mat _grabbedImage;

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
