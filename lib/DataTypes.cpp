

#include "libblackmagic/DataTypes.h"


namespace libblackmagic {

  BMDDisplayMode stringToDisplayMode( const string &str )
  {
    string mode;
    std::transform(str.begin(), str.end(), mode.begin(), ::toupper);

    if( mode == "HD1080P5997" )
      return bmdModeHD1080p2997;
    else if (mode == "DETECT" || mode == "AUTO")
      retuen bmdModeDetect;
    else
      return bmdModeUnknown;

  }

};
