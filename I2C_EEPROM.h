#include <Arduino.h>

extern void TCTask();
extern void TCTask2();

class I2C_EEPROM {
public:

  bool busy = false;

  void init() {

    DDRF |= (1 << PF7);   // pin 18 as OUTPUT (LEDGND)
    PORTF |= (1 << PF7);  // pin 18 HIGH (LEDGND)
    // TWBR = 72;            // Sets I2C speed at 100kHz
    TWBR = 12;  // Sets I2C speed at 400kHz
    TWSR = 0;   // Sets prescaler to 1 (TWPS = 00) and clears status bits
  }

  uint8_t read(uint16_t addr) {

    I2C_begin(addr);
    I2C_start();                     // Repeated start
    I2C_write(0xA1);                 // EEPROM I2C address + read
    uint8_t data = I2C_read_nack();  // Read single byte with NACK
    I2C_stop();
    I2C_disable();
    return data;
  }

  void write(uint16_t addr, uint8_t data) {

    I2C_begin(addr);
    I2C_write(data);  // Data byte
    I2C_wait();
  }

  void update(uint16_t address, uint8_t data) {

    if (data != read(address)) {
      write(address, data);
    }
  }

private:

  void I2C_wait() { 

    I2C_stop();

    while (true) {
      I2C_start();
      I2C_write(0xA0);  // EEPROM I2C address + write
      uint8_t status = TWSR & 0xF8;
      if (status == 0x18) break;  // If ACK received, EEPROM is ready
      I2C_stop();
      TCTask();  // <-- do this while waiting for writing to complete
      TCTask2();  // <-- do this while waiting for writing to complete
    }

    I2C_stop();
    I2C_disable();
  }

  void I2C_begin(uint16_t addr) {

    I2C_enable();
    I2C_start();
    I2C_write(0xA0);                // EEPROM I2C address + write
    I2C_write((addr >> 8) & 0xFF);  // High byte
    I2C_write(addr & 0xFF);         // Low byte
  }

  void I2C_enable() {

    busy = true;
    PORTF |= (1 << PF7);                 // pin 18 HIGH (LEDGND)
    DDRD &= ~((1 << PD1) | (1 << PD0));  // Set PD1 (SDA) and PD0 (SCL) as input
    PORTD |= (1 << PD1) | (1 << PD0);    // Enable internal pull-up resistors on PD1 (SDA) and PD0 (SCL)
  }

  void I2C_disable() {

    TWCR &= ~(_BV(TWEN) | _BV(TWIE) | _BV(TWEA));  // disable I2C
    PORTD &= ~((1 << PD1) | (1 << PD0));           // set SDA and SCL pin to LOW
    DDRD |= (1 << PD1) | (1 << PD0);               // SDA and SCL pin as outputs
    PORTF &= ~(1 << PF7);                          // pin 18 LOW (LEDGND)
    busy = false;
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
