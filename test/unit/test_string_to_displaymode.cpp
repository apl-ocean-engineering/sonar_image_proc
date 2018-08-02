#include <gtest/gtest.h>

#include "libblackmagic/DataTypes.h"

using namespace libblackmagic;

// Test the float to fixed16 conversion
TEST(TestStringToDisplayMode, goodData) {

  ASSERT_EQ( stringToDisplayMode("hd1080p2997"), bmdModeHD1080p2997 );
  ASSERT_EQ( stringToDisplayMode("HD1080p2997"), bmdModeHD1080p2997 );
  ASSERT_EQ( stringToDisplayMode("hd1080P2997"), bmdModeHD1080p2997 );

  ASSERT_EQ( stringToDisplayMode("detect"), bmdModeDetect );

  ASSERT_EQ( stringToDisplayMode("HD1080P31"), bmdModeUnknown );

}
