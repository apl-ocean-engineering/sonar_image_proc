#pragma once

#include <string>

#include <DeckLinkAPI.h>

enum {
  bmdModeDetect = /* 'auto' */ 0x6175746F,
};

namespace libblackmagic {

  BMDDisplayMode stringToDisplayMode( const std::string &str );

};
