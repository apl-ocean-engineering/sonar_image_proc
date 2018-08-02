

#include <algorithm>
#include <iostream>

#include <libg3logger/g3logger.h>

#include "libblackmagic/DataTypes.h"


namespace libblackmagic {

  struct StringToDisplayTableMember {
    uint32_t    mode;
    const char *name;
  } StringToDisplayTable[] = {
    {bmdModeHD1080p2997, "1080P2997"},
    {bmdModeHD1080p2997, "HD1080P2997"},
    {bmdModeDetect, "DETECT"}
  };

  BMDDisplayMode stringToDisplayMode( const std::string &str )
  {
    std::string mode;
    std::transform(str.begin(), str.end(), std::back_inserter(mode), ::toupper);

    for( unsigned int i = 0; i < sizeof(StringToDisplayTable)/sizeof(StringToDisplayTableMember); ++i ) {
      if( mode == StringToDisplayTable[i].name ) return StringToDisplayTable[i].mode;
    }

    return bmdModeUnknown;

  }

};
