#include "hardware.hpp"

IHardwareSPI::IHardwareSPI()
{
  m_nCurByte = 0 ;
}

// Utility function to reverse bits (if required)
uint8_t IHardwareSPI::reversebits(uint8_t byte)
{
  byte = ((byte & 0xF0) >> 4) | ((byte & 0x0F) << 4);
  byte = ((byte & 0xCC) >> 2) | ((byte & 0x33) << 2);
  byte = ((byte & 0xAA) >> 1) | ((byte & 0x55) << 1);
  return byte ;
}

IHardwareGPIO::enValue IHardwareGPIO::toggle(IHardwareGPIO::enValue val)
{
  return val == low?high:low ;
}

#include <stdio.h>
void printBuffer(uint8_t *buf, int len)
{
  char szHex[3] ;
  int high, low ;

  printf("buffer: [") ;
  
  for (int i=0; i < len; i++){
    if (i > 0) printf(", ") ;
    
    high = buf[i] / 16 ;
    if (high > 9) szHex[0] = (high - 10) + 'A' ;
    else szHex[0] = high + '0' ;

    low = buf[i] % 16 ;
    if (low > 9) szHex[1] = (low - 10) + 'A' ;
    else szHex[1] = low + '0' ;

    szHex[2] = '\0' ;
    
    printf("0x%s", szHex) ;
  }
  printf("]\n") ;
}

bool IHardwareSPI::write9bit(int ctl, uint8_t byte)
{
  uint8_t out = 0x00 ;
  bool bRet = false ;
  
  if (m_nCurByte == 0){  // First byte to write - exception as no prior byte
    out = (byte >> (m_nCurByte+1));
  }else{
    out = (byte >> (m_nCurByte+1)) | (m_prevByte << (8-m_nCurByte)) ;
  }
  if (ctl <= 0){
    out &= ~(1<<(7-m_nCurByte)) ;
  }else{
    out |= 1 << (7-m_nCurByte) ;
  }
  
  m_buff9bit[m_nCurByte] = out ;
  m_prevByte = byte ;

  // Fix the last byte and flush to spi
  if (m_nCurByte >= 7){
    
    m_buff9bit[8] = byte ;

    //printBuffer(m_buff9bit, 9) ;
    bRet = write(m_buff9bit, 9) ;

    m_prevByte = 0x00 ; // Reset previous byte
    m_nCurByte = 0 ; // reset counter, just flushed 9 bytes
    return bRet ;
  }else{
    // Next byte
    m_nCurByte++ ;
  }
  
  return true ;
}

bool IHardwareSPI::flush9bit(int ctl, uint8_t noop)
{
  while (m_nCurByte > 0){
    if (!write9bit(ctl, noop)) return false ;
  }

  return true ;
}
