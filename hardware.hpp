#ifndef __HARDWARE_HPP
#define __HARDWARE_HPP

#include <stdint.h>

// Create a pure virtual base class to act as an interface for
// implementations using underlying hardware specialisations and libraries
class IHardwareSPI{
public:

  IHardwareSPI() ;
  
  // Open the port and device for SPI use
  virtual bool spiopen(uint32_t bus, uint32_t device) = 0;

  // Write one byte
  virtual bool write(uint8_t byte) = 0;

  // Write a string of bytes
  virtual bool write(uint8_t *bytes, uint32_t len) = 0 ;

  // Return the read bytes into the bytes buffer.
  // Should match number of bytes written
  virtual bool read(uint8_t *bytes, uint32_t len) = 0 ;

  // Configure SPI. Most libraries only partially implement these
  // and Linux drivers are limited as to what is actually available.
  virtual bool setBitOrder(bool bLSB) = 0;
  virtual bool setCSHigh(bool bHigh) = 0;
  virtual bool setSpeed(uint32_t speed) = 0;
  virtual bool setMode(uint8_t mode) = 0;
  virtual bool set3Wire(bool b3Wire) =0; 
  virtual bool setLoop(bool bLoop) =0;
  virtual bool setBPW(uint8_t bits) =0;

  // This command handles 9bit data writes with the first
  // bit being a command or data bit (1/0)
  // This works with a 9 byte buffer and flush9bit must be called to complete
  // SPI writes. Automatic flushing occurs with every full 9 byte buffer
  bool write9bit(int cmd, uint8_t byte) ;

  // Flush out incomplete buffer with noop commands. Only works where command exists
  // otherwise 9 bit cannot be supported this way. First parameter must identify the device
  // control bit to be set as high or low (1/0)
  bool flush9bit(int ctl, uint8_t noop) ;
  
  // Returns the bits reversed
  static uint8_t reversebits(uint8_t byte) ;

private:
  int m_nCurByte ;
  uint8_t m_prevByte ;
  uint8_t m_buff9bit[9] ;
};

class IHardwareGPIO{
public:
  enum enDirection{gpio_output, gpio_input} ;
  enum enValue{high, low} ;
  enum enEdge{falling, rising, both} ;
  // Don't care what numbering used. Can be chip or abstraction. 
  // This is where the user will need to code directly for the hardware
  virtual bool setup(uint32_t pin, enDirection eDir) = 0;

  virtual bool output(uint32_t pin, enValue eVal) = 0;

  virtual enValue input(uint32_t pin) = 0;

  // Helper function to toggle enValues from low to high or high to low
  static enValue toggle(enValue val) ;

  virtual bool register_interrupt(uint32_t pin, enEdge edge, void(*function)(void)) = 0;
};

class IHardwareTimer{
public:
  // Simple interface exposing micro and millisecond sleep timers
  virtual void microSleep(unsigned int nMicroSec) = 0;
  virtual void milliSleep(unsigned int nMilliSec) = 0;
};

#endif // __HARDWARE_HPP
