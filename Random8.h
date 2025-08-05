
class Random8 {
public:

  uint16_t seed = 13;

  int8_t get(int8_t min, int8_t max) {

    seed = seed * 25173u + 13849u;
    uint8_t range = uint8_t((max + 1) - min);
    uint8_t value = (seed >> 8) & 0xFF;
    uint8_t offset = (uint16_t(value) * range) >> 8;

    return min + int8_t(offset);
  }
};
