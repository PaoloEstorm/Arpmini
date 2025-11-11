#include <Arduino.h>

volatile long _number_toggles;

ISR(TIMER3_COMPA_vect) {

  if (_number_toggles) {
    if (_number_toggles % 4 == 0) PORTF ^= (1 << PF4);  // toggle pin 21 every 4th interrupt
    _number_toggles--;                                  // decrease remaining toggles
  } else {
    bitClear(TIMSK3, OCIE3A);  // disable timer
    PORTF &= ~(1 << PF4);      // set pin 21 (A3) LOW
  }
}

class Tone8 {
public:

  void init() {  // speaker initialization

    DDRF |= (1 << PF4);                   // set pin 21 (A3) as output
    PORTF &= ~(1 << PF4);                 // set pin 21 (A3) LOW
    TCCR3B = (1 << WGM32) | (1 << CS30);  // CTC mode, prescaler = 1
    TCCR3A = 0;                           // reset count
  }

  void tone(uint32_t frequency, uint16_t duration) {  // play tune

    long toggle_count = (8 * frequency * duration) / 1000;  // ×4 more interrupts
    uint16_t ocr = (F_CPU / frequency / 8) - 1;               // ISR rate ×4 slower toggle

    OCR3A = ocr;
    _number_toggles = toggle_count;
    bitSet(TIMSK3, OCIE3A);  // enable timer
  }

  uint16_t midiToFreq(uint8_t note) {  // convert MIDI note to frequency

    static const uint16_t freqs[12] PROGMEM = {    // Base frequencies
      261, 277, 293, 311, 329, 349,
      370, 392, 415, 440, 466, 494
    };

    int8_t octave = (note / 12) - 5;
    uint16_t base = pgm_read_word(&freqs[note % 12]);

    if (octave > 0) return base << octave;   // up = multiply by 2^octave
    if (octave < 0) return base >> -octave;  // down = divide by 2^(-octave)
    return base;
  }
};
