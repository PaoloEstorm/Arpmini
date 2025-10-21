#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "SoftwareSerial_MiniRX.h"
#include <util/delay_basic.h>

SoftwareSerial *SoftwareSerial::active_object = 0;

ISR(PCINT0_vect) {
  SoftwareSerial::handle_interrupt();
}

SoftwareSerial::SoftwareSerial(uint8_t receivePin)
  :
    _enable(1) {
  setRX(receivePin);
}

SoftwareSerial::~SoftwareSerial() {
}

void SoftwareSerial::begin(long speed) {

  if (digitalPinToPCICR((int8_t)_receivePin)) {

    *digitalPinToPCICR((int8_t)_receivePin) |= _BV(digitalPinToPCICRbit(_receivePin));
    _pcint_maskreg = digitalPinToPCMSK(_receivePin);
    _pcint_maskvalue = _BV(digitalPinToPCMSKbit(_receivePin));
  }

  _receive_buffer_head = 0;
  _receive_buffer_tail = 0;
  active_object = this;

  *_pcint_maskreg |= _pcint_maskvalue;
}

void SoftwareSerial::setRX(uint8_t rx) {

  pinMode(rx, INPUT);
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  uint8_t port = digitalPinToPort(rx);
  _receivePortRegister = portInputRegister(port);
}

uint8_t SoftwareSerial::rx_pin_read() {

  return *_receivePortRegister & _receiveBitMask;
}

int16_t SoftwareSerial::read() {

  if (_receive_buffer_head == _receive_buffer_tail) return -1;  // Empty buffer?

  uint8_t d = _receive_buffer[_receive_buffer_head];                    // Read from "head"
  _receive_buffer_head = (_receive_buffer_head + 1) % _SS_MAX_RX_BUFF;  // grab next byte
  return d;
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

    *_pcint_maskreg &= ~_pcint_maskvalue;

    _delay_loop_2(_rx_delay_centering);

    for (uint8_t i = 8; i > 0; --i) {
      _delay_loop_2(_rx_delay_intrabit);
      d >>= 1;
      if (rx_pin_read()) d |= 0x80;
    }

    uint8_t next = (_receive_buffer_tail + 1) % _SS_MAX_RX_BUFF;
    
    if (next != _receive_buffer_head) {
      _receive_buffer[_receive_buffer_tail] = d;  // save new data in buffer: tail points to where byte goes
      _receive_buffer_tail = next;                // next byte
    }

    _delay_loop_2(_rx_delay_stopbit);

    *_pcint_maskreg |= _pcint_maskvalue;
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

void SoftwareSerial::handle_interrupt() {

  if (active_object) active_object->recv();
}
