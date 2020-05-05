#include "RF24Driver.hpp"

RF24Driver::RF24Driver()
{
  // Use the max address length and set payload to remaining packet size
  m_address_len = PACKET_DRIVER_MAX_ADDRESS_LEN ;
  m_payload_width = MAX_RXTXBUF - PACKET_DRIVER_MAX_ADDRESS_LEN ;
}

bool RF24Driver::initialise(uint8_t *device, uint8_t *broadcast, uint8_t length)
{
  // Fix addresses to 5 bytes from driver
  if (length != MAX_RF24_ADDRESS_LEN) return false ;
  set_address_width(length) ;
  m_address_len = length ;
  memcpy(m_device, device, length) ;
  memcpy(m_broadcast, broadcast, length) ;

  auto_update(true) ;

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
    return false ;
  }
  m_address_len = length ;

  set_retry(15,15);
  // User pipe 0 to receive transmitted responses or listen
  // on broadcast address
  enable_pipe(0, true) ;
  // Use pipe 1 for receiving unicast data
  enable_pipe(1, true) ;

  set_pipe_ack(0, false) ;
  set_pipe_ack(1, false) ;
  set_power_level(RF24_0DBM) ;
  crc_enabled(true) ;
  set_2_byte_crc(true) ;
  set_data_rate(RF24_2MBPS) ;

  // Set default payload width
  if (!BufferedRF24::set_payload_width(0,MAX_RXTXBUF))return false ;
  if (!BufferedRF24::set_payload_width(1,MAX_RXTXBUF)) return false ;

  if (!set_rx_address(0, m_broadcast, m_address_len)){
    return false ;
  }

  if (!set_rx_address(1, m_device, m_address_len)){
    return false ;
  }

  power_up(true) ;
  m_pTimer->microSleep(150) ;

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

  return true ;
}

bool RF24Driver::set_payload_width(uint8_t width)
{
  if (width > MAX_RXTXBUF - m_address_len) return false ;
  if (!BufferedRF24::set_payload_width(0,width+m_address_len)) return false ;
  if (!BufferedRF24::set_payload_width(1,width+m_address_len)) return false ;
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

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::high)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    return false; // Fairly terminal error if GPIO cannot be set
  }
  m_pTimer->microSleep(130); // 130 micro second wait

  // Flushing RX & TX and clearing interrupts not required to change to listen mode (or send mode)
  //flushrx() ;
  //flushtx() ;
  //clear_interrupts() ;
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif

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

  // Not using blocking reads for inbound data as everything
  // is a single packet message

  // The buffered class implements a mutex to control
  // accessing data between interrupt thread and the data in the class
  
  for ( ; ; ){
    if (is_rx_empty()){
#ifndef ARDUINO
      pthread_mutex_unlock(&m_rwlock) ;
#endif
      break ;
    }
    bool ret = read_payload(packet, m_payload_width+m_address_len) ;
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    if (!ret){
      m_status = io_err ; // SPI error
      return false ;
    }
    
    return (*m_callbackfn)(m_callbackcontext, packet, packet+m_address_len) ;
         
  }
  
  return true ;

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

  memcpy(send_buff, m_device, m_address_len) ;
  if (data != NULL && len > 0)
    memcpy(send_buff+m_address_len, data, len) ;


  write(send_buff, len+m_address_len, true) ;
  enStatus sret = get_status() ;
  listen_mode();
  if (sret == io_err){
    return false ;

  }else if (sret == buff_overflow){
    return false ;
  }

  return true ;

}
