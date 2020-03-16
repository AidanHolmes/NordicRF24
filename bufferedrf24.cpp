#include "bufferedrf24.hpp"
#include "radioutil.hpp"
#include <string.h>
#include <stdio.h>

#ifdef DEBUG
#define DPRINT(x,...) fprintf(stdout,x,##__VA_ARGS__)
#define EPRINT(x,...) fprintf(stderr,x,##__VA_ARGS__)
#else
#define DPRINT(x,...)
#define EPRINT(x,...)
#endif

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

bool BufferedRF24::enable_power(bool bPower)
{
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  // Enable power
  if (bPower){
    if (!is_powered_up()){
      power_up(true);
      if (!m_auto_update) write_config() ;
      m_pTimer->microSleep(130) ; // 130 micro second sleep to settle power
    }
  }else{ // Power off
    if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
 #ifndef ARDUINO
      pthread_mutex_unlock(&m_rwlock) ;
 #endif
      return false ;
    }
    if (is_powered_up()){
      power_up(false);
      if (!m_auto_update) write_config() ;
    }
  }
  
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif
  return true ;
}

bool BufferedRF24::listen_mode(bool bListen)
{
  
  
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  // Set CE low
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return false;
  }
  // Setup receiver or as sender
  receiver(bListen);
  if (!m_auto_update) write_config() ;
  
  if (bListen){
    // If listening the set CE high again to go into listen mode
    if (!m_pGPIO->output(m_ce, IHardwareGPIO::high)){
#ifndef ARDUINO
      pthread_mutex_unlock(&m_rwlock) ;
#endif
      return false ;
    }
  }
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif
  m_pTimer->microSleep(130) ; // Settle for 130 micro seconds
  
  // Enable power if not powered on
  if (!enable_power(true)) return false ;
  return true ;
}

uint16_t BufferedRF24::write(uint8_t *buffer, uint16_t length, bool blocking)
{
  uint16_t buffer_remaining = RF24_BUFFER_WRITE - m_write_size ;
  uint16_t len = length ;
  uint8_t packet_size = get_transmit_width() ;
  
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  // Write using remaining space in the data buffer
  if (buffer_remaining < length){
    len = buffer_remaining ;
    if (len == 0){
#ifndef ARDUINO
      pthread_mutex_unlock(&m_rwlock) ;
#endif
      m_status = buff_overflow ;
      return 0 ; // no more buffer left, need to transmit current buffer
    }
  }

  memcpy((void *)m_write_buffer+m_write_size, buffer, len) ;
  if (m_write_size == 0){
    // Fresh write
    m_status = ok ;
    flushtx() ; // TX buffer may have unsent data if previously failed
    // Initial data is smaller than a packet.
    // pad with zeros. Note that whole packets really should be used
    // by an implementer of this class.
    if (len < packet_size){
      uint8_t pktbuff[MAX_RXTXBUF] ; // max length buffer
      memset(pktbuff, 0, MAX_RXTXBUF); // use zeros to pad packet
      memcpy(pktbuff, buffer, len) ;
      m_front_write = write_packet(pktbuff) ;
    }else{
      // Normal write of a full packet of data
      m_front_write = write_packet((uint8_t*)m_write_buffer) ;
    }
    if (!m_front_write){
#ifndef ARDUINO
      pthread_mutex_unlock(&m_rwlock) ;
#endif
      // no date returned from write_packet
      // set as an IO error
      m_status = io_err ; 
      return 0;
    }
  }

  m_write_size += len ;
#ifndef ARDUINO
  // release thread locks
  pthread_mutex_unlock(&m_rwlock) ;
#endif

  if (blocking){
    // Just write this data and wait for it to complete
    while(m_write_size > 0){
      if(m_status == io_err) return 0 ;
      m_pTimer->microSleep(100) ; // 100 micro second wait
    }
    if (m_status == max_retry_failure) return 0 ;
  }

  // wait for the transistion settling time
  m_pTimer->microSleep(130) ; // 130 micro second wait  
    
  return len ;
}

BufferedRF24::enStatus BufferedRF24::get_status()
{
  return m_status ;
}

bool BufferedRF24::max_retry_interrupt()
{
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  m_write_size = m_front_write = 0 ; // Reset
  flushtx() ;
  
  // Set the failure status
  m_status = max_retry_failure ;
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif
  return true ;
}

bool BufferedRF24::data_sent_interrupt()
{
  uint8_t rembuf[MAX_RXTXBUF+1] ;
  uint16_t size = 0, ret = 0;

#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  uint8_t packet_size = get_transmit_width() ;

  if (m_front_write > m_write_size) size = 0 ;
  else size = m_write_size - m_front_write ;

  if (size == 0){
    // No data to send. End of transmission
    m_front_write = m_write_size = 0;
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return true ;
  }
  
  // This could be considered an error if the size is less than the packet_size
  // as the remaining data shouldn't be in the data stream.
  // Works for dynamic data packets used in Enhanced Shockburst
  if (size <= packet_size){ 
    memset(rembuf,0,MAX_RXTXBUF) ; // Clear buffer
    memcpy(rembuf, (void *)m_write_buffer+m_front_write, size) ; // Partial packet write

    ret = write_packet(rembuf) ;
  }else{
    // Write another packet
    ret = write_packet((uint8_t*)m_write_buffer+m_front_write) ;
  }
  if (ret == 0) m_status = io_err ; // flag an error
  m_front_write += ret ;
   
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif

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
      if(m_status == io_err) return 0 ;
      else if(m_status == buff_overflow) return 0 ;
      m_pTimer->microSleep(1000);
    }
    if (len > length) len = length ; // ensure just enough data is read
  }
  
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif

  if (len > 0){
    memcpy(buffer, (void *)m_read_buffer[pipe]+m_front_read[pipe], len) ;
    m_front_read[pipe] += len ;
  }
  
  // Check if there's more data in the buffer following this read
  if((m_front_read[pipe] > m_read_size[pipe]) ||
     (m_read_size[pipe] - m_front_read[pipe] == 0)){
    // No more data, reset buffer
    m_front_read[pipe] = 0 ; // reset lead pointer to buffer
    m_read_size[pipe] = 0 ; // reset and reuse buffer
    m_status = ok ;
  }

#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif

  return len ;
}

bool BufferedRF24::data_received_interrupt()
{
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  uint8_t pipe = get_pipe_available();

#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif
  if (pipe == RF24_PIPE_EMPTY) return true ; // no pipe

#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  uint8_t size = get_rx_data_size(pipe) ;
  
  while(!is_rx_empty()){
    if ((RF24_BUFFER_READ - m_read_size[pipe]) < size){
      m_status = buff_overflow ;
#ifndef ARDUINO
      pthread_mutex_unlock(&m_rwlock) ;
#endif
      return false ; // no more buffer
    }
    if (!read_payload((uint8_t*)m_read_buffer[pipe]+m_read_size[pipe], size)){
      m_status = io_err ; // SPI error
#ifndef ARDUINO
      pthread_mutex_unlock(&m_rwlock) ;
#endif
      return false ;
    }
    m_read_size[pipe] += size ;
  }
  
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif

  return true ;
}
