//   Copyright 2017 Aidan Holmes

//   Licensed under the Apache License, Version 2.0 (the "License");
//   you may not use this file except in compliance with the License.
//   You may obtain a copy of the License at
//
//       http://www.apache.org/licenses/LICENSE-2.0
//
//   Unless required by applicable law or agreed to in writing, software
//   distributed under the License is distributed on an "AS IS" BASIS,
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//   See the License for the specific language governing permissions and
//   limitations under the License.

#ifndef __NORDIC_PING_RF24
#define __NORDIC_PING_RF24

#include "rpinrf24.hpp"
#include <pthread.h>
#include <time.h>

class PingRF24 : public NordicRF24{
public:
  PingRF24() ;
  ~PingRF24() ;

  // Ping with 32 bytes of data (required for full time data)
  // Set channel
  // Reset queues
  bool initialise(uint8_t channel) ;
  
  // receiver
  bool listen(uint8_t *address) ;

  // sender
  // ping the address, count number of times
  uint16_t ping(uint8_t *address, uint16_t count);

  void print_summary() ;

protected:
  bool data_received_interrupt() ;
  bool max_retry_interrupt();
  bool data_sent_interrupt();

  uint16_t m_failed ;
  uint16_t m_succeeded ;
  uint16_t m_remaining ;
  uint32_t m_max_ping ; // ms
  uint32_t m_min_ping ; // ms
  clock_t m_tick ;

private:
  pthread_mutex_t m_rwlock ;  
};


#endif
