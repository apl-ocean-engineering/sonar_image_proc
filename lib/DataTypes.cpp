

#include <algorithm>

#include "libblackmagic/DataTypes.h"


namespace libblackmagic {

  BMDDisplayMode stringToDisplayMode( const std::string &str )
  {
    std::string mode;
    std::transform(str.begin(), str.end(), mode.begin(), ::toupper);

    if( mode == "HD1080P5997" )
      return bmdModeHD1080p2997;
    else if (mode == "DETECT" || mode == "AUTO")
      return bmdModeDetect;
    else
      return bmdModeUnknown;

  }

};
