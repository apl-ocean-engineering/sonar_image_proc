#pragma once

#include <DeckLinkAPI.h>

#include "libbmsdi/bmsdi_message.h"

namespace libblackmagic {

  // Makes an empty (blue) frame and inserts SDI protocol info
  IDeckLinkMutableVideoFrame* makeFrameWithSDIProtocol( IDeckLinkOutput *deckLinkOutput, BMSDIBuffer *buffer, bool do3D=false );

  // Add SDI protocol info to an existing frame
  IDeckLinkMutableVideoFrame* addSDIProtocolToFrame( IDeckLinkOutput *deckLinkOutput,
                                                        IDeckLinkMutableVideoFrame* frame, BMSDIBuffer *buffer );

  // Make a blank frame
  IDeckLinkMutableVideoFrame* makeBlueFrame( IDeckLinkOutput *deckLinkOutput, bool do3D=false );


}
