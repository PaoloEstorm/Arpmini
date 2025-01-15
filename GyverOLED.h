
//  AlexGyver, alex@alexgyver.ru
//  https://alexgyver.ru/
//  MIT License

#ifndef GyverOLED_h
#define GyverOLED_h

#define OLED_CLEAR 0
#define OLED_FILL 1

#include <Arduino.h>
#include <SPI.h>

#include "charMap.h"
#include <Print.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT_64 0x12
#define OLED_64 0x3F

#define OLED_DISPLAY_OFF 0xAE
#define OLED_DISPLAY_ON 0xAF

#define OLED_COMMAND_MODE 0x00
#define OLED_ONE_COMMAND_MODE 0x80
#define OLED_DATA_MODE 0x40
#define OLED_ONE_DATA_MODE 0xC0

#define OLED_ADDRESSING_MODE 0x20
#define OLED_HORIZONTAL 0x00
#define OLED_VERTICAL 0x01

#define OLED_NORMAL_V 0xC8
#define OLED_FLIP_V 0xC0
#define OLED_NORMAL_H 0xA1
#define OLED_FLIP_H 0xA0

#define OLED_CONTRAST 0x81
#define OLED_SETCOMPINS 0xDA
#define OLED_SETVCOMDETECT 0xDB
#define OLED_CLOCKDIV 0xD5
#define OLED_SETMULTIPLEX 0xA8
#define OLED_COLUMNADDR 0x21
#define OLED_PAGEADDR 0x22
#define OLED_CHARGEPUMP 0x8D

#define OLED_NORMALDISPLAY 0xA6
#define OLED_INVERTDISPLAY 0xA7

#ifndef OLED_SPI_SPEED
#define OLED_SPI_SPEED 1000000ul
#endif

static SPISettings OLED_SPI_SETT(OLED_SPI_SPEED, MSBFIRST, SPI_MODE0);

static const uint8_t _oled_init[] PROGMEM = {
  OLED_DISPLAY_OFF,
  OLED_CLOCKDIV,
  0x80,  // value
  OLED_CHARGEPUMP,
  0x14,  // value
  OLED_ADDRESSING_MODE,
  OLED_VERTICAL,
  OLED_NORMAL_H,
  OLED_NORMAL_V,
  OLED_CONTRAST,
  0x7F,  // value
  OLED_SETVCOMDETECT,
  0x40,  // value
  OLED_NORMALDISPLAY,
  OLED_DISPLAY_ON,
};

template<uint8_t _CS, uint8_t _DC, uint8_t _RST>
class GyverOLED : public Print {
public:

  void init() {

    SPI.begin();
    pinMode(_CS, OUTPUT);
    fastWrite(_CS, 1);
    pinMode(_DC, OUTPUT);

    pinMode(_RST, OUTPUT);
    fastWrite(_RST, 1);
    delay(1);
    fastWrite(_RST, 0);
    delay(20);
    fastWrite(_RST, 1);

    beginCommand();
    for (uint8_t i = 0; i < 15; i++) sendByte(pgm_read_byte(&_oled_init[i]));
    endTransm();
    beginCommand();
    sendByte(OLED_SETCOMPINS);
    sendByte(OLED_HEIGHT_64);
    sendByte(OLED_SETMULTIPLEX);
    sendByte(OLED_64);
    endTransm();
    setCursorXY(0, 0);
  }

  void clear() {
    fill(0);
  }

  void clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

    x1++;
    y1++;
    y0 >>= 3;
    y1 = (y1 - 1) >> 3;
    setWindow(x0, y0, x1, y1);
    beginData();
    for (uint8_t x = x0; x < x1; x++)
      for (uint8_t y = y0; y < y1 + 1; y++)
        writeData(0);
    endTransm();
    setCursorXY(_x, _y);
  }

  void setContrast(uint8_t value) {

    sendCommand(OLED_CONTRAST, value);
  }

  virtual size_t write(uint8_t data) {

    bool newPos = false;
    if (data == '\r') {
      _x = 0;
      newPos = true;
      data = 0;
    }  // received carriage return
    if (data == '\n') {
      _y += _scaleY;
      newPos = true;
      data = 0;
      _getn = 1;
    }  // received line feed
    if (_println && (_x + 6 * _scaleX) >= _maxX) {
      _x = 0;
      _y += _scaleY;
      newPos = true;
    }                                        // line overflow, move to next line and reset x
    if (newPos) setCursorXY(_x, _y);         // set cursor position
    if (_y + _scaleY > _maxY + 1) data = 0;  // display overflow
    if (_getn && _println && data == ' ' && _x == 0) {
      _getn = 0;
      data = 0;
    }  // remove the first space in the line

    if (data == 0) return 1;
    // if we haven't exited, print the character

    uint8_t newX = _x + _scaleX * 6;
    if (newX < 0 || _x > _maxX) _x = newX;  // skip output off-screen
    else {
      beginData();
      for (uint8_t col = 0; col < 6; col++) {   // 6 columns per character
        uint8_t bits = getFont(data, col);      // get byte
        if (_invState) bits = ~bits;            // inversion
        if (_scaleX == 1) {                     // if scale is 1
          if (_x >= 0 && _x <= _maxX) {         // within display
            if (_shift == 0) {                  // without line shift
              writeData(bits);                  // output
            } else {                            // with shift
              writeData(bits << _shift);        // upper part
              writeData(bits >> (8 - _shift));  // lower part
            }
          }
        } else {                 // scale 2, 3 or 4 - stretch font
          uint32_t newData = 0;  // buffer
          for (uint8_t i = 0, count = 0; i < 8; i++)
            for (uint8_t j = 0; j < _scaleX; j++, count++)
              bitWrite(newData, count, bitRead(bits, i));  // pack stretched font

          for (uint8_t i = 0; i < _scaleX; i++) {  // output horizontally
            byte prevData = 0;
            if (_x + i >= 0 && _x + i <= _maxX)                              // within display
              for (uint8_t j = 0; j < _scaleX; j++) {                        // output vertically
                byte data = newData >> (j * 8);                              // get buffer piece
                if (_shift == 0) {                                           // without line shift
                  writeData(data);                                           // output
                } else {                                                     // with shift
                  writeData((prevData >> (8 - _shift)) | (data << _shift));  // merge and output
                  prevData = data;                                           // remember previous
                }
              }
            if (_shift != 0) writeData(prevData >> (8 - _shift));  // output lower piece with shift
          }
        }
        _x += _scaleX;  // move by pixel width (1-4)
      }
      endTransm();
    }

    return 1;
  }

  void home() {

    setCursorXY(0, 0);
  }

  void setCursorXY(uint8_t x, uint8_t y) {
    _x = x;
    _y = y;
    setWindowShift(x, y, _maxX, _scaleY);
  }

  void setScale(uint8_t scale) {

    _scaleX = scale;
    _scaleY = scale * 8;
    setCursorXY(_x, _y);
  }

  void invertText(bool inv) {

    _invState = inv;
  }

  void fill(uint8_t data) {

    setWindow(0, 0, _maxX, _maxRow);
    beginData();
    for (int i = 0; i < 1024; i++) sendByte(data);
    endTransm();
    setCursorXY(_x, _y);
  }

  void writeData(byte data) {

    sendByte(data);
  }

  void setWindowShift(uint8_t x0, uint8_t y0, uint8_t sizeX, uint8_t sizeY) {

    _shift = y0 & 0b111;
    setWindow(x0, (y0 >> 3), x0 + sizeX, (y0 + sizeY - 1) >> 3);
  }

  void sendByte(uint8_t data) {

    SPI.transfer(data);
  }

  void sendCommand(uint8_t cmd1) {

    beginCommand();
    sendByte(cmd1);
    endTransm();
  }

  void sendCommand(uint8_t cmd1, uint8_t cmd2) {

    beginCommand();
    sendByte(cmd1);
    sendByte(cmd2);
    endTransm();
  }

  void setWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

    beginCommand();
    sendByte(OLED_COLUMNADDR);
    sendByte(constrain(x0, 0, _maxX));
    sendByte(constrain(x1, 0, _maxX));
    sendByte(OLED_PAGEADDR);
    sendByte(constrain(y0, 0, _maxRow));
    sendByte(constrain(y1, 0, _maxRow));
    endTransm();
  }

  void beginData() {

    startTransm();
    fastWrite(_DC, 1);
  }

  void beginCommand() {

    startTransm();
    fastWrite(_DC, 0);
  }

  void endTransm() {

    fastWrite(_CS, 1);
    SPI.endTransaction();
  }

  void startTransm() {

    SPI.beginTransaction(OLED_SPI_SETT);
    fastWrite(_CS, 0);
  }

  uint8_t getFont(uint8_t font, uint8_t row) {

    if (row > 4) return 0;
    font = font - '0' + 16;
    return pgm_read_byte(&(_charMap[font][row]));
  }

  const uint8_t _maxRow = 8 - 1;
  const uint8_t _maxY = 64 - 1;
  const uint8_t _maxX = OLED_WIDTH - 1;

private:

  void fastWrite(const uint8_t pin, bool val) {

    digitalWrite(pin, val);
  }

  bool _invState = 0;
  bool _println = false;
  bool _getn = false;
  uint8_t _scaleX = 1, _scaleY = 8;
  uint8_t _x = 0, _y = 0;
  uint8_t _shift = 0;
};
#endif
