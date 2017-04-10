#include "pingRF24.hpp"

#include <iostream>
#include <time.h>

PingRF24::PingRF24()
{
  m_failed = 0;
  m_succeeded = 0;
  m_max_ping = 0;
  m_min_ping = 0;
  m_remaining = 0 ;

  if (pthread_mutex_init(&m_rwlock, NULL) != 0)
    std::cerr << "Mutex creation failed\n" ;
}

PingRF24::~PingRF24()
{
  pthread_mutex_destroy(&m_rwlock) ;  
}

bool PingRF24::data_received_interrupt()
{

  return true ;
}

bool PingRF24::max_retry_interrupt()
{

  return true ;
}

bool PingRF24::data_sent_interrupt()
{

  return true ;
}

bool PingRF24::initialise(uint8_t channel)
{

  return true ;
}

bool PingRF24::listen()
{

  return true ;
}

uint16_t PingRF24::ping(uint8_t *address, uint16_t count)
{
  pthread_mutex_lock(&m_rwlock) ;
  if (m_remaining > 0){
    pthread_mutex_unlock(&m_rwlock) ;
    return m_remaining ;
  }
  pthread_mutex_unlock(&m_rwlock) ;

  m_failed = 0;
  m_succeeded = 0;
  m_max_ping = 0;
  m_min_ping = 0;
  m_remaining = count ;

  if (count == 0) return 0; // nothing to do

  clock_t clock() ;
  if (!write_packet((uint8_t*)&clock_t)) return 0 ;
  
  return m_remaining ;
}

void PingRF24::print_summary()
{

}
