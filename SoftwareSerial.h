#include <inttypes.h>
#include <Stream.h>

#define _SS_MAX_RX_BUFF 64  // RX buffer size

#ifndef GCC_VERSION
#define GCC_VERSION (__GNUC__ * 10000 + __GNUC_MINOR__ * 100 + __GNUC_PATCHLEVEL__)
#endif

class SoftwareSerial : public Stream {

public:
  // public methods
  SoftwareSerial(uint8_t receivePin);
  ~SoftwareSerial();
  void begin(long speed);
  bool listen();
  void end();
  bool isListening() {
    return this == active_object;
  }
  bool stopListening();
  bool overflow() {
    bool ret = _buffer_overflow;
    if (ret) _buffer_overflow = false;
    return ret;
  }
  int peek();

  virtual size_t write(uint8_t byte);
  virtual int read();
  virtual int available();

  operator bool() {
    return true;
  }

  using Print::write;

  // public only for easy access by interrupt handlers
  static inline void handle_interrupt() __attribute__((__always_inline__));

private:
  // per object data
  uint8_t _receivePin;
  uint8_t _receiveBitMask;
  volatile uint8_t *_receivePortRegister;
  uint8_t _transmitBitMask;
  volatile uint8_t *_transmitPortRegister;
  volatile uint8_t *_pcint_maskreg;
  uint8_t _pcint_maskvalue;

  // Expressed as 4-cycle delays (must never be 0!)
  uint16_t _rx_delay_centering;
  uint16_t _rx_delay_intrabit;
  uint16_t _rx_delay_stopbit;

  uint16_t _buffer_overflow : 1;
  bool _enable : 1;

  // static data
  static uint8_t _receive_buffer[_SS_MAX_RX_BUFF];
  static volatile uint8_t _receive_buffer_tail;
  static volatile uint8_t _receive_buffer_head;
  static SoftwareSerial *active_object;

  // private methods
  inline void recv() __attribute__((__always_inline__));
  uint8_t rx_pin_read();
  void setRX(uint8_t receivePin);
  inline void setRxIntMsk(bool enable) __attribute__((__always_inline__));

  // Return num - sub, or 1 if the result would be < 1
  static uint16_t subtract_cap(uint16_t num, uint16_t sub);
  static inline void tunedDelay(uint16_t delay);
};
