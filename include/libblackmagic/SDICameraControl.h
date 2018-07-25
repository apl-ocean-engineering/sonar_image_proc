#pragma once

#include <DeckLinkAPI.h>

#include "libbmsdi/bmsdi_message.h"

namespace libblackmagic {

  IDeckLinkMutableVideoFrame* AddSDICameraControlFrame( IDeckLinkOutput *deckLinkOutput,
                                                        IDeckLinkMutableVideoFrame* frame, BMSDIBuffer *buffer );

  IDeckLinkMutableVideoFrame* CreateBlueFrame( IDeckLinkOutput *deckLinkOutput, bool do3D );


}
