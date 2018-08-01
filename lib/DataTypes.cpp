

#include <algorithm>
#include <iostream>

#include <libg3logger/g3logger.h>

#include "libblackmagic/DataTypes.h"


namespace libblackmagic {

  BMDDisplayMode stringToDisplayMode( const std::string &str )
  {
    std::string mode;
    std::transform(str.begin(), str.end(), std::back_inserter(mode), ::toupper);

	   LOG(INFO) << "Decoding mode string:" << mode;

    if( mode == "1080P2997" || mode == "HD1080P2997"  )
      return bmdModeHD1080p2997;
    else if (mode == "DETECT" || mode == "AUTO")
      return bmdModeDetect;
    else
      return bmdModeUnknown;

  }

};
