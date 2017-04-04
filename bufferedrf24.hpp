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

#ifndef __BUFFERED_NORDIC_RF24
#define __BUFFERED_NORDIC_RF24

#define RF24_BUFFER_READ 1600
#define RF24_BUFFER_WRITE 1600

#include "rpinrf24.hpp"

class BufferedRF24 : public NordicRF24{
public:
  BufferedRF24();
  ~BufferedRF24();

  uint16_t write(uint8_t *buffer, uint16_t length, bool blocking) ;
  uint16_t read(uint8_t *buffer, uint16_t length, bool blocking) ;
  
protected:
  uint8_t m_read_buffer[RF24_BUFFER_READ];
  uint8_t m_write_buffer[RF24_BUFFER_WRITE];
  uint16_t m_read_size ;
  uint16_t m_write_size ;
  
private:
  
  
};


#endif
