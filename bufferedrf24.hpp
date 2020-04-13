//   Copyright 2020 Aidan Holmes

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

#ifndef __BUFFERED_NORDIC_RF24
#define __BUFFERED_NORDIC_RF24

// 50 read and 50 write packets can be held. 
// Can be scaled for embedded by changing these macros (should be multiple of 32)
#define RF24_BUFFER_READ 64
#define RF24_BUFFER_WRITE 64

#include "rpinrf24.hpp"

class BufferedRF24 : public NordicRF24{
public:
  BufferedRF24();
  ~BufferedRF24();

  // Additional orchestration for the RF24 driver to simplify the interface
  // Enable or disable power with correct settling time
  bool enable_power(bool bPower);
  // Enable or disable listen mode with correct settling time and CE high
  bool listen_mode(bool bListen) ;
  
  // Writes a buffer of data to a receiver. Returns bytes written.
  // Cannot exceed RF24_BUFFER_WRITE length
  // Can throw BuffIOErr or BuffMaxRetry if blocking
  uint16_t write(uint8_t *buffer, uint16_t length, bool blocking) ;

  // Read data into a buffer. Returns bytes written. Returns 0 if no data is waiting and not blocking
  // Can throw BuffIOErr if blocking
  uint16_t read(uint8_t *buffer, uint16_t length, uint8_t pipe, bool blocking) ;

  enum enStatus {ok,max_retry_failure,io_err,buff_overflow} ;

  // Call the write_status if using non-blocking write calls and find out if the
  // read or write was successful. 
  enStatus get_status();
  
protected:
  virtual bool data_received_interrupt() ;
  virtual bool max_retry_interrupt();
  virtual bool data_sent_interrupt();

  volatile uint8_t m_read_buffer[RF24_PIPES][RF24_BUFFER_READ];
  volatile uint8_t m_write_buffer[RF24_BUFFER_WRITE];
  volatile uint16_t m_read_size[RF24_PIPES], m_front_read[RF24_PIPES] ;
  volatile uint16_t m_write_size, m_front_write ;

  volatile enStatus m_status ;  
};


#endif
