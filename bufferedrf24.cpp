#include "bufferedrf24.hpp"
#include "radioutil.h"

BufferedRF24::BufferedRF24()
{
  m_read_size = 0;  
  m_write_size = 0;
  m_front_read = 0 ;
  m_front_write = 0 ;
}

BufferedRF24::~BufferedRF24()
{
  
}

uint16_t BufferedRF24::write(uint8_t *buffer, uint16_t length, bool blocking)
{
  
}

uint16_t BufferedRF24::read(uint8_t *buffer, uint16_t length, bool blocking)
{
  uint16_t len = length ;
  uint16_t buff_size = m_read_size - m_front_read ;
  if (length > buff_size) len = buff_size ; // length larger than remaining buffer
  if (len == 0){
    if (!blocking) return 0 ; // buffer is empty
    while((len=m_read_size - m_front_read) == 0){
      nano_sleep(0,10000000) ;
    }
  }
  
  memcpy(buffer, m_read_buffer+m_front_read, len) ;

  if (length < buff_size){
    // Still buffer remaining
    m_front_read += length ;
  }else{
    m_front_read = 0 ; // reset lead pointer to buffer
    m_read_size = 0 ; // reset and reuse buffer
  }

  return len ;
}

bool BufferedRF24::data_received_interrupt()
{
  //if (!read_status()) return false ; // status already read in interrupt()
  uint8_t pipe = get_pipe_available();
  if (pipe == RF24_PIPE_EMPTY) return true ; // no pipe
  uint8_t size = get_rx_data_size(pipe) ;
  while(!is_rx_empty()){
    if ((RF24_BUFFER_READ - m_read_size) < size) return false ; // no more buffer
    if (!read_payload(m_read_buffer+m_read_size, size)) return false ; // SPI error
    m_read_size += size ;
  }
  return true ;
}

bool BufferedRF24::max_retry_interrupt()
{

}

bool BufferedRF24::data_sent_interrupt()
{

}
