#include "bufferedrf24.hpp"
#include "radioutil.h"
#include <string.h>
#include <stdio.h>

BufferedRF24::BufferedRF24()
{
  m_read_size = 0;  
  m_write_size = 0;
  m_front_read = 0 ;
  m_front_write = 0 ;

  if (pthread_mutex_init(&m_rwlock, NULL) != 0)
    fprintf(stderr, "Mutex creation failed\n") ;
  
}

BufferedRF24::~BufferedRF24()
{
  pthread_mutex_destroy(&m_rwlock) ;  
}

uint16_t BufferedRF24::write(uint8_t *buffer, uint16_t length, bool blocking)
{
  uint16_t buffer_remaining = RF24_BUFFER_WRITE - m_write_size ;
  uint16_t len = length ;

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
    uint8_t packet_size = get_transmit_width() ;
    m_front_write = packet_size ;
    printf("Writing payload of size %d\n", packet_size) ;
    write_payload(m_write_buffer, packet_size) ;
    m_pGPIO->output(m_ce, IHardwareGPIO::high) ;
    nano_sleep(0,10000) ; // 10 micro seconds
    m_pGPIO->output(m_ce, IHardwareGPIO::low) ;
  }

  m_write_size += len ;

  if (blocking){
    uint16_t written = m_write_size ;
    // Just write this data and wait for it to complete
    // release thread locks
    pthread_mutex_unlock(&m_rwlock) ;
    while(m_write_size > 0){
      nano_sleep(0,100000) ; // 1 ms
    }
    return written ;
  }
    
  pthread_mutex_unlock(&m_rwlock) ;

  return len ;
}

bool BufferedRF24::max_retry_interrupt()
{
  uint16_t size = 0;

  printf ("Received max try interrupt. Cancel transmission\n") ;
  pthread_mutex_lock(&m_rwlock) ;

  //size = m_write_size - m_front_write ;

  m_write_size = m_front_write = 0 ; // Reset
  flushtx() ;
  
  // Slightly awkward situation, what do I tell the
  // write call?
  // To Do: Set error flag

  pthread_mutex_unlock(&m_rwlock) ;
  return true ;
}

bool BufferedRF24::data_sent_interrupt()
{
  uint8_t rembuf[32] ;
  uint16_t size = 0;
  uint8_t packet_size = get_transmit_width() ;
  
  fprintf(stdout, "Data sent interrupt\n") ;

  pthread_mutex_lock(&m_rwlock) ;

  if (m_front_write > m_write_size) size = 0 ;
  else size = m_write_size - m_front_write ;
  printf("%d bytes left to write\n", size) ;

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
    printf("Writing payload of size %d\n", packet_size) ;
    write_payload(rembuf, packet_size) ;
  }else{
    // Write another packet
    printf("Writing payload of size %d\n", packet_size) ;
    write_payload(m_write_buffer+m_front_write, packet_size) ;
  }
  m_front_write += packet_size ;
  // Send the packet
  m_pGPIO->output(m_ce, IHardwareGPIO::high) ;
  nano_sleep(0,10000) ; // 10 micro seconds
  m_pGPIO->output(m_ce, IHardwareGPIO::low) ;
   
  pthread_mutex_unlock(&m_rwlock) ;

  return true ;
}

uint16_t BufferedRF24::read(uint8_t *buffer, uint16_t length, uint8_t pipe, bool blocking)
{
  uint16_t len = length ;
  uint16_t buff_size = 0 ;

  if (pipe >= RF24_PIPES) return 0 ; // out of range
  
  buff_size = m_read_size - m_front_read ; 
  if (length > buff_size) len = buff_size ; // length larger than remaining buffer

  if (len == 0){
    if (!blocking){
      return 0 ; // buffer is empty
    }
    // Wait until there's buffer to read
    while((len=m_read_size - m_front_read) == 0){
      nano_sleep(0,10000000) ;
    }
  }
  
  pthread_mutex_lock(&m_rwlock) ;

  memcpy(buffer, m_read_buffer[pipe]+m_front_read, len) ;

  if (length < buff_size){
    // Still buffer remaining
    m_front_read += length ;
  }else{
    m_front_read = 0 ; // reset lead pointer to buffer
    m_read_size = 0 ; // reset and reuse buffer
  }

  pthread_mutex_unlock(&m_rwlock) ;

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
    if (!read_payload(m_read_buffer[pipe]+m_read_size, size)) return false ; // SPI error
    m_read_size += size ;
  }
  return true ;
}
