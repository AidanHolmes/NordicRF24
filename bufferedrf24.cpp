#include "bufferedrf24.hpp"
#include "radioutil.h"
#include <string.h>
#include <stdio.h>

BufferedRF24::BufferedRF24()
{
  for (int i = 0; i < RF24_PIPES; i++){
    m_read_size[i] = 0;  
    m_front_read[i] = 0 ;
  }
  m_write_size = 0;
  m_front_write = 0 ;
  m_status = ok ;
}

BufferedRF24::~BufferedRF24()
{

}

uint16_t BufferedRF24::write(uint8_t *buffer, uint16_t length, bool blocking)
{
  uint16_t buffer_remaining = RF24_BUFFER_WRITE - m_write_size ;
  uint16_t len = length ;
  uint8_t packet_size = get_transmit_width() ;
  
  pthread_mutex_lock(&m_rwlock) ;
  // Write using remaining space in the data buffer
  if (buffer_remaining < length){
    len = buffer_remaining ;
    if (len == 0){
      pthread_mutex_unlock(&m_rwlock) ;
      return 0 ;
    }
  }

  memcpy(m_write_buffer+m_write_size, buffer, len) ;
  if (m_write_size == 0){
    // Fresh write
    m_status = ok ;
    flushtx() ; // TX buffer may have unsent data if previously failed
    // Initial data is smaller than a packet.
    // pad with zeros. Note that whole packets really should be used
    // by an implementer of this class.
    if (len < packet_size){
      uint8_t pktbuff[32] ; // max length buffer
      memset(pktbuff, 0, 32); // use zeros to pad packet
      memcpy(pktbuff, buffer, len) ;
      m_front_write = write_packet(pktbuff) ;
    }else{
      // Normal write of a full packet of data
      m_front_write = write_packet(m_write_buffer) ;
    }
    if (!m_front_write){
      pthread_mutex_unlock(&m_rwlock) ;
      throw BuffIOErr("write_packet failed") ;
    }
  }

  m_write_size += len ;

  if (blocking){
    uint16_t written = m_write_size ;
    // Just write this data and wait for it to complete
    // release thread locks
    pthread_mutex_unlock(&m_rwlock) ;
    while(m_write_size > 0){
      if(m_status == io_err) throw BuffIOErr("error blocking write") ;
      nano_sleep(0,100000) ; // 100 micro second wait
    }
    if (m_status == max_retry_failure) throw BuffMaxRetry() ;
    return written ;
  }else{
    // wait for the transistion settling time
    nano_sleep(0, 130000) ; // 130 micro seconds
  }
    
  pthread_mutex_unlock(&m_rwlock) ;

  return len ;
}

BufferedRF24::enStatus BufferedRF24::get_status()
{
  return m_status ;
}

bool BufferedRF24::max_retry_interrupt()
{
  pthread_mutex_lock(&m_rwlock) ;

  m_write_size = m_front_write = 0 ; // Reset
  flushtx() ;
  
  // Slightly awkward situation, what do I tell the
  // write call?
  m_status = max_retry_failure ;

  pthread_mutex_unlock(&m_rwlock) ;
  return true ;
}

bool BufferedRF24::data_sent_interrupt()
{
  uint8_t rembuf[33] ;
  uint16_t size = 0, ret = 0;

  pthread_mutex_lock(&m_rwlock) ;
  uint8_t packet_size = get_transmit_width() ;

  if (m_front_write > m_write_size) size = 0 ;
  else size = m_write_size - m_front_write ;

  if (size == 0){
    // No data to send. End of transmission
    m_front_write = m_write_size = 0;
    pthread_mutex_unlock(&m_rwlock) ;
    return true ;
  }
  
  // This could be considered an error if the size is less than the packet_size
  // as the remaining data shouldn't be in the data stream.
  // Works for dynamic data packets used in Enhanced Shockburst
  if (size <= packet_size){ 
    memset(rembuf,0,32) ; // Clear
    memcpy(rembuf, m_write_buffer+m_front_write, size) ; // Partial packet write

    ret = write_packet(rembuf) ;
  }else{
    // Write another packet
    ret = write_packet(m_write_buffer+m_front_write) ;
  }
  if (ret == 0) m_status = io_err ; // flag an error
  m_front_write += ret ;
   
  pthread_mutex_unlock(&m_rwlock) ;

  return true ;
}

uint16_t BufferedRF24::read(uint8_t *buffer, uint16_t length, uint8_t pipe, bool blocking)
{
  uint16_t len = length ;
  uint16_t buff_size = 0 ;

  if (pipe >= RF24_PIPES) return 0 ; // out of range

  // Check if there's unread data in the buffer
  buff_size = m_read_size[pipe] - m_front_read[pipe] ; 
  if (length > buff_size) len = buff_size ; // length larger than remaining buffer

  if (len == 0 && blocking){
    // Wait until there's buffer to read
    while((len=m_read_size[pipe] - m_front_read[pipe]) == 0){
      if(m_status == io_err) throw BuffIOErr("error blocking read") ;
      else if(m_status == buff_overflow) throw BuffOverflow() ;
      nano_sleep(0,10000000) ;
    }
    if (len > length) len = length ; // ensure just enough data is read
  }
  
  pthread_mutex_lock(&m_rwlock) ;

  if (len > 0){
    memcpy(buffer, m_read_buffer[pipe]+m_front_read[pipe], len) ;
    m_front_read[pipe] += len ;
  }
  
  // Check if there's anymore data in the buffer following this read
  if((m_front_read[pipe] > m_read_size[pipe]) ||
     (m_read_size[pipe] - m_front_read[pipe] == 0)){
    m_front_read[pipe] = 0 ; // reset lead pointer to buffer
    m_read_size[pipe] = 0 ; // reset and reuse buffer
    m_status = ok ;
  }

  pthread_mutex_unlock(&m_rwlock) ;

  return len ;
}

bool BufferedRF24::data_received_interrupt()
{
  pthread_mutex_lock(&m_rwlock) ;
  uint8_t pipe = get_pipe_available();
  pthread_mutex_unlock(&m_rwlock) ;
  if (pipe == RF24_PIPE_EMPTY) return true ; // no pipe

  pthread_mutex_lock(&m_rwlock) ;
  uint8_t size = get_rx_data_size(pipe) ;
  
  while(!is_rx_empty()){
    if ((RF24_BUFFER_READ - m_read_size[pipe]) < size){
      m_status = buff_overflow ;
      pthread_mutex_unlock(&m_rwlock) ;
      return false ; // no more buffer
    }
    if (!read_payload(m_read_buffer[pipe]+m_read_size[pipe], size)){
      m_status = io_err ; // SPI error
      pthread_mutex_unlock(&m_rwlock) ;
      return false ;
    }
    m_read_size[pipe] += size ;
  }
  
  pthread_mutex_unlock(&m_rwlock) ;

  return true ;
}
