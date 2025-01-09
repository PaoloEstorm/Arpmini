
//  AlexGyver, alex@alexgyver.ru
//  https://alexgyver.ru/
//  MIT License

#ifndef GyverOLED_h
#define GyverOLED_h

#define SSD1306_128x64 1

#define OLED_I2C 0
#define OLED_SPI 1

#define OLED_NO_BUFFER 0
#define OLED_BUFFER 1

#define OLED_CLEAR 0
#define OLED_FILL 1
#define OLED_STROKE 2

#define BUF_ADD 0
#define BUF_SUBTRACT 1
#define BUF_REPLACE 2

#define BITMAP_NORMAL 0
#define BITMAP_INVERT 1

#if defined(USE_MICRO_WIRE)
#include <microWire.h>
#else
#include <Wire.h>
#endif

#include <Arduino.h>
#include <SPI.h>

#include "charMap.h"
#include <Print.h>

#define OLED_WIDTH 128
#define OLED_HEIGHT_32 0x02
#define OLED_HEIGHT_64 0x12
#define OLED_64 0x3F
#define OLED_32 0x1F

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

#define BUFSIZE_128x64 (128 * 64 / 8)
#define BUFSIZE_128x32 (128 * 32 / 8)

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

template<int _TYPE, int _BUFF = OLED_BUFFER, int _CONN = OLED_I2C, int8_t _CS = -1, int8_t _DC = -1, int8_t _RST = -1>
class GyverOLED : public Print {
public:

  GyverOLED(uint8_t address = 0x3C)
    : _address(address) {}

  void init(int __attribute__((unused)) sda = 0, int __attribute__((unused)) scl = 0) {
    if (_CONN) {
      SPI.begin();
      pinMode(_CS, OUTPUT);
      fastWrite(_CS, 1);
      pinMode(_DC, OUTPUT);
      if (_RST > 0) {
        pinMode(_RST, OUTPUT);
        fastWrite(_RST, 1);
        delay(1);
        fastWrite(_RST, 0);
        delay(20);
        fastWrite(_RST, 1);
      }
    } else {
      Wire.begin();
    }

    beginCommand();
    for (uint8_t i = 0; i < 15; i++) sendByte(pgm_read_byte(&_oled_init[i]));
    endTransm();
    beginCommand();
    sendByte(OLED_SETCOMPINS);
    sendByte(_TYPE ? OLED_HEIGHT_64 : OLED_HEIGHT_32);
    sendByte(OLED_SETMULTIPLEX);
    sendByte(_TYPE ? OLED_64 : OLED_32);
    endTransm();

    setCursorXY(0, 0);
    if (_BUFF) setWindow(0, 0, _maxX, _maxRow);  // для буфера включаем всю область
  }

  void clear() {
    fill(0);
  }

  void clear(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) {

    if (_TYPE < 2) {  // для SSD1306
      x1++;
      y1++;
      y0 >>= 3;
      y1 = (y1 - 1) >> 3;
      setWindow(x0, y0, x1, y1);
      beginData();
      for (int x = x0; x < x1; x++)
        for (int y = y0; y < y1 + 1; y++)
          writeData(0, y, x, 2);
      endTransm();
    }
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
    }  // получен возврат каретки
    if (data == '\n') {
      _y += _scaleY;
      newPos = true;
      data = 0;
      _getn = 1;
    }  // получен перевод строки
    if (_println && (_x + 6 * _scaleX) >= _maxX) {
      _x = 0;
      _y += _scaleY;
      newPos = true;
    }                                        // строка переполненена, перевод и возврат
    if (newPos) setCursorXY(_x, _y);         // переставляем курсор
    if (_y + _scaleY > _maxY + 1) data = 0;  // дисплей переполнен
    if (_getn && _println && data == ' ' && _x == 0) {
      _getn = 0;
      data = 0;
    }  // убираем первый пробел в строке

    // фикс русских букв и некоторых символов
    if (data > 127) {
      uint8_t thisData = data;
      // data = 0 - флаг на пропуск
      if (data > 191) data = 0;
      else if (_lastChar == 209 && data == 145) data = 192;  // ё кастомная
      else if (_lastChar == 208 && data == 129) data = 149;  // Е вместо Ё
      else if (_lastChar == 226 && data == 128) data = 0;    // тире вместо длинного тире (начало)
      else if (_lastChar == 128 && data == 148) data = 45;   // тире вместо длинного тире
      _lastChar = thisData;
    }
    if (data == 0) return 1;
    // если тут не вылетели - печатаем символ

    if (_TYPE < 2 || 1) {  // для SSD1306
      int newX = _x + _scaleX * 6;
      if (newX < 0 || _x > _maxX) _x = newX;  // пропускаем вывод "за экраном"
      else {
        if (!_BUFF) beginData();
        for (uint8_t col = 0; col < 6; col++) {                // 6 стобиков буквы
          uint8_t bits = getFont(data, col);                   // получаем байт
          if (_invState) bits = ~bits;                         // инверсия
          if (_scaleX == 1) {                                  // если масштаб 1
            if (_x >= 0 && _x <= _maxX) {                      // внутри дисплея
              if (_shift == 0) {                               // если вывод без сдвига на строку
                writeData(bits, 0, 0, _mode);                  // выводим
              } else {                                         // со сдвигом
                writeData(bits << _shift, 0, 0, _mode);        // верхняя часть
                writeData(bits >> (8 - _shift), 1, 0, _mode);  // нижняя часть
              }
            }
          } else {                 // масштаб 2, 3 или 4 - растягиваем шрифт
            uint32_t newData = 0;  // буфер
            for (uint8_t i = 0, count = 0; i < 8; i++)
              for (uint8_t j = 0; j < _scaleX; j++, count++)
                bitWrite(newData, count, bitRead(bits, i));  // пакуем растянутый шрифт

            for (uint8_t i = 0; i < _scaleX; i++) {  // выводим. По Х
              byte prevData = 0;
              if (_x + i >= 0 && _x + i <= _maxX)                                           // внутри дисплея
                for (uint8_t j = 0; j < _scaleX; j++) {                                     // выводим. По Y
                  byte data = newData >> (j * 8);                                           // получаем кусок буфера
                  if (_shift == 0) {                                                        // если вывод без сдвига на строку
                    writeData(data, j, i, _mode);                                           // выводим
                  } else {                                                                  // со сдвигом
                    writeData((prevData >> (8 - _shift)) | (data << _shift), j, i, _mode);  // склеиваем и выводим
                    prevData = data;                                                        // запоминаем предыдущий
                  }
                }
              if (_shift != 0) writeData(prevData >> (8 - _shift), _scaleX, i, _mode);  // выводим нижний кусочек, если со сдвигом
            }
          }
          _x += _scaleX;  // двигаемся на ширину пикселя (1-4)
        }
        if (!_BUFF) endTransm();
      }
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

    if (_BUFF) memset(_oled_buffer, data, _bufSize);
    else {
      if (_TYPE < 2 || 1) {  // для SSD1306
        setWindow(0, 0, _maxX, _maxRow);
        beginData();
        for (int i = 0; i < (_TYPE ? 1024 : 512); i++) sendByte(data);
        endTransm();
      } else {  // для SSH1108
      }
    }
    setCursorXY(_x, _y);
  }

  void writeData(byte data, byte offsetY = 0, byte offsetX = 0, int mode = 0) {
    if (_BUFF) {
      switch (mode) {
        case 0:
          _oled_buffer[_bufIndex(_x + offsetX, _y) + offsetY] |= data;  // добавить
          break;
        case 1:
          _oled_buffer[_bufIndex(_x + offsetX, _y) + offsetY] &= ~data;  // вычесть
          break;
        case 2:
          _oled_buffer[_bufIndex(_x + offsetX, _y) + offsetY] = data;  // заменить
          break;
      }
    } else {
      if (_buf_flag) {
        int x = _x - _bufX;
        int y = _y - _bufY;
        if (x < 0 || x > _bufsizeX || y < 0 || y > (_bufsizeY << 3)) return;
        switch (mode) {
          case 0:
            _buf_ptr[(y >> 3) + x * _bufsizeY] |= data;  // добавить
            break;
          case 1:
            _buf_ptr[(y >> 3) + x * _bufsizeY] &= ~data;  // вычесть
            break;
          case 2:
            _buf_ptr[(y >> 3) + x * _bufsizeY] = data;  // заменить
            break;
        }
      } else {
        sendByte(data);
      }
    }
  }

  void setWindowShift(int x0, int y0, int sizeX, int sizeY) {

    _shift = y0 & 0b111;
    if (!_BUFF) setWindow(x0, (y0 >> 3), x0 + sizeX, (y0 + sizeY - 1) >> 3);
  }

  void sendByte(uint8_t data) {
    sendByteRaw(data);
    #if !defined(microWire_h)
    if (!_CONN) {
      _writes++;
      if (_writes >= 16) {
        endTransm();
        beginData();
      }
    }
   #endif
  }

  void sendByteRaw(uint8_t data) {

    if (_CONN) SPI.transfer(data);
    else Wire.write(data);
  }

  void sendCommand(uint8_t cmd1) {

    beginOneCommand();
    sendByteRaw(cmd1);
    endTransm();
  }

  void sendCommand(uint8_t cmd1, uint8_t cmd2) {

    beginCommand();
    sendByteRaw(cmd1);
    sendByteRaw(cmd2);
    endTransm();
  }

  void setWindow(int x0, int y0, int x1, int y1) {

    beginCommand();
    sendByteRaw(OLED_COLUMNADDR);
    sendByteRaw(constrain(x0, 0, _maxX));
    sendByteRaw(constrain(x1, 0, _maxX));
    sendByteRaw(OLED_PAGEADDR);
    sendByteRaw(constrain(y0, 0, _maxRow));
    sendByteRaw(constrain(y1, 0, _maxRow));
    endTransm();
  }

  void beginData() {

    startTransm();
    if (_CONN) fastWrite(_DC, 1);
    else sendByteRaw(OLED_DATA_MODE);
  }

  void beginCommand() {

    startTransm();
    if (_CONN) fastWrite(_DC, 0);
    else sendByteRaw(OLED_COMMAND_MODE);
  }

  void beginOneCommand() {

    startTransm();
    if (_CONN) fastWrite(_DC, 0);
    else sendByteRaw(OLED_ONE_COMMAND_MODE);
  }

  void endTransm() {

    if (_CONN) {
      fastWrite(_CS, 1);
      SPI.endTransaction();
    } else {
      Wire.endTransmission();
      _writes = 0;
      delayMicroseconds(2);  // https://github.com/GyverLibs/GyverOLED/issues/45
    }
  }

  void startTransm() {
    if (_CONN) {
      SPI.beginTransaction(OLED_SPI_SETT);
      fastWrite(_CS, 0);
    } else Wire.beginTransmission(_address);
  }

  uint8_t getFont(uint8_t font, uint8_t row) {

    if (row > 4) return 0;
    font = font - '0' + 16;
    return pgm_read_byte(&(_charMap[font][row]));
  }

  const uint8_t _address = 0x3C;
  const uint8_t _maxRow = (_TYPE ? 8 : 4) - 1;
  const uint8_t _maxY = (_TYPE ? 64 : 32) - 1;
  const uint8_t _maxX = OLED_WIDTH - 1;

  const int _bufSize = ((_BUFF == 1) ? (_TYPE ? BUFSIZE_128x64 : BUFSIZE_128x32) : 0);
  uint8_t _oled_buffer[((_BUFF == 1) ? (_TYPE ? BUFSIZE_128x64 : BUFSIZE_128x32) : 0)];

private:

  void fastWrite(const uint8_t pin, bool val) {

    digitalWrite(pin, val);
  }

  int _bufIndex(int x, int y) {
    return ((y) >> 3) + ((x) << (_TYPE ? 3 : 2));
  }

  bool _invState = 0;
  bool _println = false;
  bool _getn = false;
  uint8_t _scaleX = 1, _scaleY = 8;
  int _x = 0, _y = 0;
  uint8_t _shift = 0;
  uint8_t _lastChar;
  uint8_t _writes = 0;
  uint8_t _mode = 2;

  int _bufsizeX, _bufsizeY;
  int _bufX, _bufY;
  uint8_t* _buf_ptr;
  bool _buf_flag = false;
};
#endif
