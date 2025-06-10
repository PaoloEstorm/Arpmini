#include "Arduino.h"

volatile long timer_toggle_count;

ISR(TIMER3_COMPA_vect) {

  if (timer_toggle_count != 0) {

    PORTF ^= (1 << PF4);  // toggle pin 21 (A3) (square wave)

    if (timer_toggle_count > 0)
      timer_toggle_count--;
  } else {
    bitClear(TIMSK3, OCIE3A);
    PORTF &= ~(1 << PF4);  // set pin 21 (A3) LOW
  }
}

class Tone8 {
public:

  void tone(unsigned int frequency, uint8_t duration) {

    uint8_t prescalarbits = 0b001;
    long toggle_count = 0;
    uint32_t ocr = F_CPU / frequency / 2 - 1;

    toneBegin();

    if (ocr > 0xffff) {
      ocr = F_CPU / frequency / 2 / 64 - 1;
      prescalarbits = 0b011;
    }

    TCCR3B = (TCCR3B & 0b11111000) | prescalarbits;

    toggle_count = 2 * frequency * duration / 1000;  // calculate the toggle count

    OCR3A = ocr;
    timer_toggle_count = toggle_count;
    bitSet(TIMSK3, OCIE3A);
  }

private:

  void toneBegin() {
    
    TCCR3A = 0;
    TCCR3B = 0;
    bitSet(TCCR3B, WGM32);
    bitSet(TCCR3B, CS30);
  }
};
