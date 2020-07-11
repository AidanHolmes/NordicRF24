#include "RF24Driver.hpp"

RF24Driver::RF24Driver()
{
  // Use the max address length and set payload to remaining packet size
  m_address_len = PACKET_DRIVER_MAX_ADDRESS_LEN ;
  m_payload_width = MAX_RXTXBUF - PACKET_DRIVER_MAX_ADDRESS_LEN ;
  m_sendstatus = Status::waiting ;
}

bool RF24Driver::initialise(uint8_t *device, uint8_t *broadcast, uint8_t length)
{
  m_address_len = length ;
  memcpy(m_device, device, length) ;
  memcpy(m_broadcast, broadcast, length) ;

  // Check if driver is missing port interfaces
  if (!m_pGPIO || !m_pSPI) return false ;

  m_pSPI->setCSHigh(false) ;
  m_pSPI->setMode(0) ;
  
  auto_update(true) ;

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
    return false ;
  }
  // Reset device
  power_up(false);
  reset_rf24() ;
  flushtx() ;
  flushrx() ;
  // Set all interrupts. This is important as they are used to read and write data
  set_use_interrupt_data_ready(true);
  set_use_interrupt_data_sent(true);
  set_use_interrupt_max_retry(true);
  
  m_address_len = length ;
  if (!set_address_width(length)) return false ; // invalid width
  set_retry(15,15);
  // User pipe 0 to receive transmitted responses or listen
  // on broadcast address
  enable_pipe(0, true) ;
  // Use pipe 1 for receiving unicast data
  enable_pipe(1, true) ;

  // Don't use ACKs - reduce radio noise and handle in protocol
  set_pipe_ack(0, false) ;
  set_pipe_ack(1, false) ;
  set_power_level(RF24_0DBM) ; // Max power
  crc_enabled(true) ;
  set_2_byte_crc(true) ;
  set_data_rate(RF24_2MBPS) ; // Highest speed

  // Set default payload width
  if (!NordicRF24::set_payload_width(0,MAX_RXTXBUF))return false ;
  if (!NordicRF24::set_payload_width(1,MAX_RXTXBUF)) return false ;
  if (!NordicRF24::set_transmit_width(MAX_RXTXBUF)) return false ;

  if (!set_rx_address(0, m_broadcast, m_address_len)){
    return false ;
  }

  if (!set_rx_address(1, m_device, m_address_len)){
    return false ;
  }

  m_pTimer->microSleep(5000) ; // 1.5 ms settle
  power_up(true) ;

  listen_mode() ;

  return true ;
}

bool RF24Driver::shutdown()
{
  // Drop CE if transition is from listen mode
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low))
    return false ;

  // Power down
  power_up(false) ;
  flushrx() ;
  flushtx() ;
  clear_interrupts() ;

  return true ;
}

bool RF24Driver::set_payload_width(uint8_t width)
{
  if (width > MAX_RXTXBUF - m_address_len) return false ;
  if (!NordicRF24::set_payload_width(0,width+m_address_len)) return false ;
  if (!NordicRF24::set_payload_width(1,width+m_address_len)) return false ;
  if (!NordicRF24::set_transmit_width(width+m_address_len)) return false ;
  m_payload_width = width ;
  return true ;
}

uint8_t RF24Driver::get_payload_width()
{
  return m_payload_width ;
}

bool RF24Driver::send_mode()
{
  if (!m_pGPIO){
    return false; // Terminal problem - GPIO interface required
  }
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return false;
  }

  receiver(false);

  m_pTimer->microSleep(130); // 130 micro second wait
  
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif

  return true;
}

bool RF24Driver::listen_mode()
{
  if (!m_pGPIO){ 
    return false ;
  }

#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return false ; // Terminal problem
  }

  // Set pipe 0 to listen on the broadcast address
  if (!set_rx_address(0, m_broadcast, m_address_len)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return false;
  }
  receiver(true);

  // 130 micro second wait recommended in RF24 spec
  m_pTimer->microSleep(130); 

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::high)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return false; // Fairly terminal error if GPIO cannot be set
  }
  m_pTimer->microSleep(4);

  // Flushing RX & TX and clearing interrupts not required to change to listen mode (or send mode)

#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif

  return true ;
}
bool RF24Driver::max_retry_interrupt()
{
  flushtx();
  m_sendstatus = Status::failed ;
  return true ;
}

bool RF24Driver::data_sent_interrupt()
{
  m_sendstatus = Status::delivered ;
  return true ;
}

bool RF24Driver::data_received_interrupt()
{
  uint8_t packet[MAX_RXTXBUF] ;
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  uint8_t pipe = get_pipe_available();
  
  if (pipe == RF24_PIPE_EMPTY){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return true ; // no pipe
  }

  uint8_t size = get_rx_data_size(pipe) ;
  
  while (!is_rx_empty()){
    bool ret = read_payload(packet, size) ;
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    if (!ret){
      return false ;
    }
    return (*m_callbackfn)(m_callbackcontext, packet, packet+m_address_len) ;
  }  
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
  return false ;
}

bool RF24Driver::send(const uint8_t *receiver, uint8_t *data, uint8_t len)
{
  uint8_t send_buff[MAX_RXTXBUF] ;
  if (get_payload_width() < len) return false ; // too long
  send_mode() ;
  
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  
  if (!set_tx_address(receiver, m_address_len)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return false ;
  }

  if (!set_rx_address(0, receiver, m_address_len)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return false ;
  }
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif

  // Copy address
  memcpy(send_buff, m_device, m_address_len) ;
  // Copy data
  if (data != NULL && len > 0)
    memcpy(send_buff+m_address_len, data, len) ;

  m_sendstatus = Status::waiting ;
  // No flushing of TX buffer required prior to write
  write_packet(send_buff) ;

  // Wait for send status to update or quit after 250 ms
  for (uint16_t i=0;m_sendstatus == Status::waiting && i < 2500; i++){
    m_pTimer->microSleep(100) ;
  }
  // loop exited after waiting too long. Set ioerr (TO DO: expose this error)
  if (m_sendstatus == Status::waiting) m_sendstatus = Status::ioerr ;
  
  listen_mode();

  return m_sendstatus == Status::delivered ;

}
