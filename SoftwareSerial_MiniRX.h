#include <inttypes.h>

#define _SS_MAX_RX_BUFF 32  // RX buffer size

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

#if GCC_VERSION > 40800
#define _rx_delay_centering 45
#define _rx_delay_intrabit 123
#define _rx_delay_stopbit 84
#else
#define _rx_delay_centering 34
#define _rx_delay_intrabit 126
#define _rx_delay_stopbit 81
#endif

class SoftwareSerial {

public:

  SoftwareSerial(uint8_t receivePin);
  ~SoftwareSerial();

  void begin(long speed);
  static void handle_interrupt();
  int16_t read();

  // dummy methods
  size_t write(uint8_t byte) {
    return true;
  }
  bool available() {
    return true;
  }

private:
  // per object data
  uint8_t _receivePin;
  uint8_t _receiveBitMask;
  volatile uint8_t *_receivePortRegister;
  volatile uint8_t *_pcint_maskreg;
  uint8_t _pcint_maskvalue;
  bool _enable;

  // static data
  volatile uint8_t _receive_buffer[_SS_MAX_RX_BUFF];
  volatile uint8_t _receive_buffer_tail;
  volatile uint8_t _receive_buffer_head;
  static SoftwareSerial *active_object;

  // private methods
  inline void recv();
  uint8_t rx_pin_read();
  void setRX(uint8_t receivePin);
};
