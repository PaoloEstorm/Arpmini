  #include <avr/interrupt.h>
  #include <avr/pgmspace.h>
  #include <Arduino.h>
  #include "SoftwareSerial.h"
  #include <util/delay_basic.h>

SoftwareSerial *SoftwareSerial::active_object = 0;
uint8_t SoftwareSerial::_receive_buffer[_SS_MAX_RX_BUFF];
volatile uint8_t SoftwareSerial::_receive_buffer_tail = 0;
volatile uint8_t SoftwareSerial::_receive_buffer_head = 0;

inline void SoftwareSerial::tunedDelay(uint16_t delay) {
  _delay_loop_2(delay);
}

bool SoftwareSerial::listen() {
  if (!_rx_delay_stopbit)
    return false;

  if (active_object != this) {
    if (active_object)
      active_object->stopListening();

    _buffer_overflow = false;
    _receive_buffer_head = _receive_buffer_tail = 0;
    active_object = this;

    setRxIntMsk(true);
    return true;
  }
  return false;
}

bool SoftwareSerial::stopListening() {
  if (active_object == this) {
    setRxIntMsk(false);
    active_object = NULL;
    return true;
  }
  return false;
}

void SoftwareSerial::recv() {

  #if GCC_VERSION < 40302
  // Work-around for avr-gcc 4.3.0 OSX version bug
  // Preserve the registers that the compiler misses
  // (courtesy of Arduino forum user *etracer*)
  asm volatile(
    "push r18 \n\t"
    "push r19 \n\t"
    "push r20 \n\t"
    "push r21 \n\t"
    "push r22 \n\t"
    "push r23 \n\t"
    "push r26 \n\t"
    "push r27 \n\t" ::);
  #endif

  uint8_t d = 0;

  if (!rx_pin_read()) {

    setRxIntMsk(false);
    tunedDelay(_rx_delay_centering);

    for (uint8_t i = 8; i > 0; --i) {
      tunedDelay(_rx_delay_intrabit);
      d >>= 1;

      if (rx_pin_read())
        d |= 0x80;
    }

    // if buffer full, set the overflow flag and return
    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    if (next != _receive_buffer_head) {
      // save new data in buffer: tail points to where byte goes
      _receive_buffer[_receive_buffer_tail] = d;  // save new byte
      _receive_buffer_tail = next;
    } else {

      _buffer_overflow = true;
    }

    tunedDelay(_rx_delay_stopbit);
    setRxIntMsk(true);
  }

  #if GCC_VERSION < 40302
  // Work-around for avr-gcc 4.3.0 OSX version bug
  // Restore the registers that the compiler misses
  asm volatile(
    "pop r27 \n\t"
    "pop r26 \n\t"
    "pop r23 \n\t"
    "pop r22 \n\t"
    "pop r21 \n\t"
    "pop r20 \n\t"
    "pop r19 \n\t"
    "pop r18 \n\t" ::);
  #endif
}

uint8_t SoftwareSerial::rx_pin_read() {
  return *_receivePortRegister & _receiveBitMask;
}

inline void SoftwareSerial::handle_interrupt() {
  if (active_object) {
    active_object->recv();
  }
}

ISR(PCINT0_vect) {
  SoftwareSerial::handle_interrupt();
}

SoftwareSerial::SoftwareSerial(uint8_t receivePin)
  : _rx_delay_centering(0),
    _rx_delay_intrabit(0),
    _rx_delay_stopbit(0),
    _buffer_overflow(false),
    _enable(1) {
  setRX(receivePin);
}

SoftwareSerial::~SoftwareSerial() {
  end();
}

void SoftwareSerial::setRX(uint8_t rx) {
  pinMode(rx, INPUT);
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  uint8_t port = digitalPinToPort(rx);
  _receivePortRegister = portInputRegister(port);
}

uint16_t SoftwareSerial::subtract_cap(uint16_t num, uint16_t sub) {
  if (num > sub)
    return num - sub;
  else
    return 1;
}

void SoftwareSerial::begin(long speed) {

  uint16_t bit_delay = (F_CPU / speed) / 4;

  if (digitalPinToPCICR((int8_t)_receivePin)) {
  #if GCC_VERSION > 40800
    _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 75 + 17 - 23) / 4);
    _rx_delay_intrabit = subtract_cap(bit_delay, 23 / 4);
    _rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (37 + 11) / 4);
  #else
    _rx_delay_centering = subtract_cap(bit_delay / 2, (4 + 4 + 97 + 29 - 11) / 4);
    _rx_delay_intrabit = subtract_cap(bit_delay, 11 / 4);
    _rx_delay_stopbit = subtract_cap(bit_delay * 3 / 4, (44 + 17) / 4);
  #endif
    *digitalPinToPCICR((int8_t)_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
    _pcint_maskreg = digitalPinToPCMSK(_receivePin);
    _pcint_maskvalue = _BV(digitalPinToPCMSKbit(_receivePin));
  }

  listen();
}

void SoftwareSerial::setRxIntMsk(bool enable) {
  if (enable)
    *_pcint_maskreg |= _pcint_maskvalue;
  else
    *_pcint_maskreg &= ~_pcint_maskvalue;
}

void SoftwareSerial::end() {
  stopListening();
}

int SoftwareSerial::read() {
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  uint8_t d = _receive_buffer[_receive_buffer_head];  // grab next byte
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;
  return d;
}

int SoftwareSerial::available() {
  if (!isListening())
    return 0;

  return ((unsigned int)(_receive_buffer_tail + _SS_MAX_RX_BUFF - _receive_buffer_head)) % _SS_MAX_RX_BUFF;
}

size_t SoftwareSerial::write(uint8_t b) {
  b = b;
  return 1;
}

int SoftwareSerial::peek() {
  if (!isListening())
    return -1;

  // Empty buffer?
  if (_receive_buffer_head == _receive_buffer_tail)
    return -1;

  // Read from "head"
  return _receive_buffer[_receive_buffer_head];
}
