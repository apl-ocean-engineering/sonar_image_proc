#pragma once

#include <DeckLinkAPI.h>

#include "DataTypes.h"

namespace libblackmagic {

class InputConfig {
public:
  InputConfig( BMDDisplayMode defaultMode = bmdModeDetect )
    : _do3D( false ),
      _mode( defaultMode )
  {;}

  InputConfig &set3D( bool do3D = true )
    { _do3D = do3D;  return *this; }

  InputConfig &setMode( BMDDisplayMode m )
    { _mode = m; return *this; }


  bool           do3D()         { return _do3D; }
  BMDDisplayMode mode()         { return _mode; }

private:
  bool _do3D;
  BMDDisplayMode _mode;
};

}
