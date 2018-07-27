
#include <string>

#include <DeckLinkAPIVersion.h>

#include "libg3logger/g3logger.h"

#include "DeckLink.h"

namespace libblackmagic {

  using std::string;

  DeckLink::DeckLink( int cardNo )
  :
    _deckLink( createDeckLink( cardNo ) ),
    // _deckLinkInput( nullptr ),
    // _deckLinkOutput( nullptr ),
    _inputHandler( nullptr  ),
    _outputHandler( nullptr )
  {
    CHECK( _deckLink != nullptr );

    _inputHandler = new InputHandler( _deckLink );
    CHECK( _inputHandler != nullptr );

    _outputHandler = new OutputHandler( _deckLink );
    CHECK( _outputHandler  != nullptr );
  }

  DeckLink::~DeckLink()
  {
    // if( _deckLinkOutput ) {
    //   // Disable the video input interface
    //   _deckLinkOutput->DisableVideoOutput();
    //   _deckLinkOutput->Release();
    // }
    //
    // if( _deckLinkInput ) {
    //   _deckLinkInput->StopStreams();
    //   _deckLinkInput->Release();
    // }

    if( _inputHandler ) delete _inputHandler;
    if( _outputHandler) delete _outputHandler;

    _deckLink->Release();

  }


  void DeckLink::listCards() {
    IDeckLink *dl = nullptr;
    IDeckLinkIterator *deckLinkIterator = CreateDeckLinkIteratorInstance();

    int i = 0;
    while( (deckLinkIterator->Next(&dl)) == S_OK ) {
      char *modelName, *displayName;
      if( dl->GetModelName( (const char **)&modelName ) != S_OK ) {
        LOG(WARNING) << "Unable to query model name.";
      }
      if( dl->GetDisplayName( (const char **)&displayName ) != S_OK ) {
        LOG(WARNING) << "Unable to query display name.";
      }

      LOG(INFO) << i << " model name: " << modelName << "; display name: " << displayName;

      free(modelName);
      free(displayName);
      i++;
    }

    deckLinkIterator->Release();
  }

  void DeckLink::listInputModes() {

  IDeckLinkInput *input = nullptr;

    // Get the input (capture) interface of the DeckLink device
    auto result = _deckLink->QueryInterface(IID_IDeckLinkInput, (void**)&input);
    if (result != S_OK) {
      LOG(WARNING) << "Couldn't get input for Decklink";
      return;
    }

    IDeckLinkDisplayModeIterator* displayModeItr = NULL;
    IDeckLinkDisplayMode *displayMode = NULL;

    if( input->GetDisplayModeIterator(&displayModeItr) != S_OK ) {
      LOG(WARNING) << "Unable to get DisplayModeIterator";
      return;
    }

    // Iterate through available modes
    while( displayModeItr->Next( &displayMode ) == S_OK ) {

      char *displayModeName = nullptr;
      if( displayMode->GetName( (const char **)&displayModeName) != S_OK ) {
        LOG(WARNING) << "Unable to get name of DisplayMode";
        return;
      }

      BMDTimeValue timeValue = 0;
      BMDTimeScale timeScale = 0;

      if( displayMode->GetFrameRate( &timeValue, &timeScale ) != S_OK ) {
        LOG(WARNING) << "Unable to get DisplayMode frame rate";
        return;
      }

      float frameRate = (timeScale != 0) ? float(timeValue)/timeScale : float(timeValue);

      LOG(INFO) << "Card supports display mode \"" << displayModeName << "\"    "
                    << displayMode->GetWidth() << " x " << displayMode->GetHeight()
                    << ", " << 1.0/frameRate << " FPS";

  }

  input->Release();

}

  //==== Lazy constructors ====
  // IDeckLink *DeckLink::deckLink()
  // {
  //   if( !_deckLink ) createDeckLink();
  //   CHECK( (bool)_deckLink ) << "Error creating Decklink card";
  //   return _deckLink;
  // }
  //
  // IDeckLinkInput *DeckLink::deckLinkInput()
  // {
  //   if( !_deckLinkInput && !initialize() ) {
  //     LOG(FATAL) << "Error creating video input";
  //     return NULL;
  //   }
  //   CHECK( (bool)_deckLinkInput ) << "_deckLinkInput not set";
  //   return _deckLinkInput;
  // }
  //
  // IDeckLinkOutput *DeckLink::deckLinkOutput()
  // {
  //   if( !_deckLinkOutput && !createVideoOutput() ) {
  //     LOG(FATAL) << "Error creating video output";
  //     return NULL;
  //   }
  //   CHECK( (bool)_deckLinkOutput ) << "_deckLinkOutput not set";
  //   return _deckLinkOutput;
  // }
  //
  // InputHandler &DeckLink::inputHandler()
  // {
  //   // InputHandler is created in parallel with deckLinkInput
  //   if( !_inputHandler ) { deckLinkInput(); }
  //   CHECK( (bool)_inputHandler ) << "_inputHandler not set";
  //
  //   return *_inputHandler;
  // }
  //
  // OutputHandler &DeckLink::outputHandler()
  // {
  //   // OutputHandler is create in parallel with deckLinkOutput
  //   if( !_outputHandler ) { deckLinkOutput(); }
  //   CHECK( (bool)_outputHandler ) << "_outputHandler not set";
  //
  //   return *_outputHandler;
  // }

  //=================================================================
  // Configuration functions
  IDeckLink *DeckLink::createDeckLink( int cardNo )
  {
    LOG(INFO) << "Using Decklink API  " << BLACKMAGIC_DECKLINK_API_VERSION_STRING;
    //
    // if( _deckLink ) {
    //   _deckLink->Release();
    //   _deckLink = NULL;
    // }

    HRESULT result;

    IDeckLinkIterator *deckLinkIterator = CreateDeckLinkIteratorInstance();
    IDeckLink *deckLink = nullptr;

    // Index cards by number for now
    int i;
    for( i = 0; i <= cardNo; i++ ) {
      if( (result = deckLinkIterator->Next(&deckLink)) != S_OK) {
        LOG(WARNING) << "Couldn't get information on DeckLink card " << i;
        return nullptr;
      }
    }

    if( i >= cardNo ) {
      LOG(WARNING) << "System has " << cardNo << " cards";
      return nullptr;
    }

    free( deckLinkIterator );

    char *modelName, *displayName;
    if( deckLink->GetModelName( (const char **)&modelName ) != S_OK ) {
      LOG(WARNING) << "Unable to query model name.";
    }

    if( deckLink->GetDisplayName( (const char **)&displayName ) != S_OK ) {
      LOG(WARNING) << "Unable to query display name.";
    }

    LOG(INFO) << "Using card " << cardNo << " model name: " << modelName << "; display name: " << displayName;

    free(modelName);
    free(displayName);

   return deckLink;
  }


  //=================================================================

  bool DeckLink::startStreams( void )
  {

    // TODO.  Check responses
    _outputHandler->startStreams();
    _inputHandler->startStreams();


    return true;
  }

  void DeckLink::stopStreams( void )
  {
    _inputHandler->stopStreams();
    _outputHandler->stopStreams();

  }
  //
  // bool DeckLink::grab( void )
  // {
  //   // TODO.  Go back and check how many copies are being made...
  //   _grabbedImage = cv::Mat();
  //
  //   // while( inputHandler().queue().try_and_pop(_grabbedImage) ) { LOG(INFO) << "Pop!  Queue now " << inputHandler().queue().size(); }
  //   //
  //   // if( !_grabbedImage.empty() ) return true;
  //
  //   // If there was nothing in the queue, wait
  //   if( inputHandler().queue().wait_for_pop(_grabbedImage, std::chrono::milliseconds(100) ) == false ) {
  //     LOG(WARNING) << "Timeout waiting for image in image queue";
  //     return false;
  //   }
  //
  //   if( !_grabbedImage.empty() ) return true;
  //
  //   return false;
  // }
  //
  // int DeckLink::getRawImage( int i, cv::Mat &mat )
  // {
  //   switch(i) {
  //     case 0:
  //             mat = _grabbedImage;
  //             return 1;
  //             break;
  //
  //     default:
  //             return 0;
  //   }
  //
  //   return 0;
  // }

  // ImageSize DeckLink::imageSize( void ) const
  // {
  //   return _inputHandler->imageSize();
  // }

}
