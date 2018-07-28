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

    // Input and output will be created automatically with defaults unless these
    // functions are called first.
    // bool  createVideoInput( const BMDDisplayMode desiredMode = bmdModeHD1080p2997 );
    // bool createVideoOutput( const BMDDisplayMode desiredMode = bmdModeHD1080p2997 );

    void listCards();
    void listInputModes();



    // These start and stop the input streams
    bool startStreams();
    void stopStreams();

    // Lazy constructors
    InputHandler &input()     { return *_inputHandler; }
    OutputHandler &output()   { return *_outputHandler; }

    // // Pull images from _InputHandler
    // virtual bool grab( void );
    // virtual int getRawImage( int i, cv::Mat &mat );


  protected:

    IDeckLink *createDeckLink( int cardNo );

  private:

    // int _cardNo;

    // For now assume an object uses just one Decklink board
    // COM model precludes use of auto ptrs
    IDeckLink *_deckLink;
    // IDeckLinkInput *_deckLinkInput;
    // IDeckLinkOutput *_deckLinkOutput;

    InputHandler *_inputHandler;
    OutputHandler *_outputHandler;

  };

}
