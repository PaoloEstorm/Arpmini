// Based on "GyverOled" library by Alex Gyver, alex@alexgyver.ru
// https://alexgyver.ru/

#pragma GCC optimize ("-Os")

#include <Arduino.h>
#include <Print.h>

#include "Arpmini_Font.h"

#define OLED_WIDTH 128
#define OLED_HEIGHT 64

#define OLED_DISPLAY_OFF 0xAE
#define OLED_DISPLAY_ON 0xAF
#define OLED_CONTRAST 0x81

// startup sequence
const uint8_t _oled_init[] PROGMEM = {
  OLED_DISPLAY_OFF,
  0xD5,  // CLOCKDIV
  0x80,  // value
  0x8D,  // CHARGEPUMP
  0x14,  // value
  0x20,  // ADDRESSING_MODE
  0x01,  // VERTICAL
  0xA1,  // NORMAL_H
  0xC8,  // NORMAL_V
  OLED_CONTRAST,
  0x7F,  // value
  0xDB,  // VCOMDETECT
  0x40,  // value
  0xA6,  // NORMALDISPLAY
  0xDA,  // COMPINS
  0x12,  // HEIGHT_64
  0xA8,  // MULTIPLEX
  0x3F,  // OLED 64
  OLED_DISPLAY_ON,
};

class SPI_OLED : public Print {
public:

  void init() {

    DDRF |= (1 << PF6) | (1 << PF5);  // pin 19 & 20 as OUTPUT (RST & DC)

    PORTF &= ~(1 << PF6);  // pin 19 LOW (RST)
    delayMicroseconds(200);
    PORTF |= (1 << PF6);  // pin 19 HIGH (RST)

    PORTB |= (1 << PB0);  // pin 17 HIGH (enable pull-up) (SS)
    DDRB &= ~(1 << PB0);  // pin 17 as INPUT (SS)

    DDRB |= (1 << PB2) | (1 << PB1);  // set MOSI and SCK as OUTPUT
    SPCR = (1 << SPE) | (1 << MSTR);  // Enable SPI, Master, f/4
    SPSR = (1 << SPI2X);              // Double speed, f/2 → 8 MHz

    PORTF &= ~(1 << PF5);  // pin 20 LOW (DC)
    for (uint8_t i = 0; i < 19; i++) SPI_write(pgm_read_byte(&_oled_init[i]));
  }

  void clear() {

    setWindow(0, 0, _maxX, _maxRow);
    PORTF |= (1 << PF5);  // pin 20 HIGH (DC)
    for (int i = 0; i < 1024; i++) SPI_write(0x00);
    setCursorXY(0, 0);
  }

  void clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

    y0 >>= 3;
    y1 >>= 3;
    setWindow(x0, y0, x1, y1);
    PORTF |= (1 << PF5);  // pin 20 HIGH (DC)
    for (uint8_t x = x0; x <= x1; x++)
      for (uint8_t y = y0; y <= y1; y++)
        SPI_write(0x00);
    setCursorXY(_x, _y);
  }

  void setContrast(uint8_t value) {

    sendCommand(OLED_CONTRAST, value);
  }

  void printF(const char* ptr) {  // print from flash

    char text;
    while ((text = pgm_read_byte(ptr++))) {
      print(text);
    }
  }

  void printlnF(const char* ptr) {  // println from flash

    printF(ptr);
    println();
  }

  void setCursorXY(uint8_t x, uint8_t y) {

    _x = x;
    _y = y;
    setWindowShift(x, y);
  }

  void setSize(uint8_t scale) {

    _scaleX = scale;
    _scaleY = scale * 8;
    setCursorXY(_x, _y);
  }

  void invertText(bool inv) {

    _invState = inv;
  }

  void sendCommand(uint8_t cmd1) {

    PORTF &= ~(1 << PF5);  // pin 20 LOW (DC)
    SPI_write(cmd1);
  }

  void sendCommand(uint8_t cmd1, uint8_t cmd2) {

    PORTF &= ~(1 << PF5);  // pin 20 LOW (DC)
    SPI_write(cmd1);
    SPI_write(cmd2);
  }

  void shiftX() {

    setCursorXY(_x + 6, _y);
  }

private:

  const uint8_t _maxRow = 8 - 1;
  const uint8_t _maxY = OLED_HEIGHT - 1;
  const uint8_t _maxX = OLED_WIDTH - 1;
  bool _invState = 0;
  uint8_t _scaleX = 1;
  uint8_t _scaleY = 8;
  uint8_t _x = 0;
  uint8_t _y = 0;
  uint8_t _shift = 0;

  virtual size_t write(uint8_t data) {

    switch (data) {
      case '\r':
        _x = 0;
        setCursorXY(_x, _y);
        return 1;
        break;

      case '\n':
        _y += _scaleY;
        setCursorXY(_x, _y);
        return 1;
        break;
    }

    PORTF |= (1 << PF5);  // pin 20 HIGH (DC)

    for (uint8_t col = 0; col < 6; col++) {  // 6 columns per character
      uint8_t bits = getFont(data, col);     // get the font byte
      if (_invState) bits = ~bits;           // invert bits if needed

      if (_scaleX == 1) {  // scale 1: direct output
        SPI_write(bits);
      } else {  // scale 2 or 3: stretch the font

        uint32_t newData = 0;
        uint8_t mask = (1 << _scaleX) - 1;  // e.g., for _scaleX = 3, mask becomes 0b111
        for (uint8_t i = 0; i < 8; i++) {
          if (bits & (1 << i)) {
            newData |= (uint32_t)mask << (i * _scaleX);
          }
        }

        // Output newData: split the buffer into 8-bit chunks
        for (uint8_t xOffset = 0; xOffset < _scaleX; xOffset++) {
          uint8_t prevData = 0;
          for (uint8_t j = 0; j < _scaleX; j++) {
            uint8_t dataByte = (newData >> (j * 8)) & 0xFF;  // extract an 8-bit segment
            SPI_write((prevData >> (8 - _shift)) | (dataByte << _shift));
            prevData = dataByte;
          }
          if (_shift != 0) SPI_write(prevData >> (8 - _shift));
        }
      }
      _x += _scaleX;
    }

    return 1;
  }

  void setWindowShift(uint8_t x0, uint8_t y0) {

    _shift = y0 & 0b111;
    setWindow(x0, y0 >> 3, _maxX, (y0 + _scaleY - 1) >> 3);
  }

  void setWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

    PORTF &= ~(1 << PF5);  // pin 20 LOW (DC)
    SPI_write(0x21);     // COLUMNADDR
    SPI_write(x0);
    SPI_write(x1);
    SPI_write(0x22);  // PAGEADDR
    SPI_write(y0);
    SPI_write(y1);
  }

  void SPI_write(uint8_t data) {

    SPDR = data;  // Load data into buffer
    while (!(SPSR & (1 << SPIF))) {
      ;  // Wait for transmission complete
    }
  }

  uint8_t getFont(uint8_t colum, uint8_t row) {

    if (row > 4) return 0;
    colum = colum - '0' + 16;
    return pgm_read_byte(&(ArpminiFont[colum][row]));
  }
};
