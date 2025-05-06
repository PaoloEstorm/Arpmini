#include <Arduino.h>

class I2C_EEPROM {
public:

  bool busy = true;

  void write(uint16_t addr, uint8_t data) {

    I2C_begin(addr);
    I2C_write(data);  // Data byte
    I2C_stop();
    delay(4);
  }

  uint8_t read(uint16_t addr) {

    I2C_begin(addr);
    I2C_start();                     // Repeated start
    I2C_write(0xA1);                 // EEPROM I2C address + read
    uint8_t data = I2C_read_nack();  // Read single byte with NACK
    I2C_stop();
    return data;
  }

  void update(uint16_t address, uint8_t data) {

    if (data != read(address)) {
      write(address, data);
    }
  }

  void init() {

    DDRF |= (1 << 7);  // pin 18 as OUTPUT
    TWBR = 72;         // Sets I2C speed to 100kHz
    TWSR = 0;          // Sets prescaler to 1 (TWPS = 00) and clears status bits
  }

private:

  void I2C_begin(uint16_t addr) {

    I2C_enable();
    I2C_start();
    I2C_write(0xA0);                // EEPROM I2C address + write
    I2C_write((addr >> 8) & 0xFF);  // High byte
    I2C_write(addr & 0xFF);         // Low byte
  }

  void I2C_enable() {

    busy = true;
    PORTF |= (1 << 7);                   // pin 18 high
    DDRD &= ~((1 << PD1) | (1 << PD0));  // Set PD1 (SDA) and PD0 (SCL) as input
    PORTD |= (1 << PD1) | (1 << PD0);    // Enable internal pull-up resistors on PD1 and PD0
  }

  void I2C_start() {

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);  // Send START condition
    while (!(TWCR & (1 << TWINT)))
      ;  // Wait for TWINT flag set
  }

  void I2C_stop() {

    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  // Send STOP condition
    while (TWCR & (1 << TWSTO))
      ;  // Wait for STOP to finish

    TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));  // disable I2C
    PORTD &= ~((1 << PD1) | (1 << PD0));           // set SDA and SCL pin to LOW
    DDRD |= (1 << PD1) | (1 << PD0);               // SDA and SCL pin as outputs
    PORTF &= ~(1 << 7);                            // pin 18 low
    busy = false;
  }

  void I2C_write(uint8_t data) {

    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);  // Start transmission
    while (!(TWCR & (1 << TWINT)))
      ;  // Wait for TWINT flag
  }

  uint8_t I2C_read_nack() {

    TWCR = (1 << TWINT) | (1 << TWEN);  // Read with NACK
    while (!(TWCR & (1 << TWINT)))
      ;
    return TWDR;
  }
};