namespace serdp_common {

struct SonarPoint {
  SonarPoint(float _x, float _z) : x(_x), z(_z) { ; }
  float x;
  float z;
};

struct SonarData {
  SonarData() : timestamp(0.0), frequency(-1.0) { ; }
  unsigned int nBearings;
  unsigned int nRanges;
  float timestamp;
  float frequency;
  std::vector<float> bearings;
  std::vector<float> ranges;
  std::vector<float> intensities;
};

} // namespace serdp_common
