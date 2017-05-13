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
#include <string.h>
#include <pthread.h>
#include <exception>

const int excptlen = 1024 ;

class BuffException : public std::exception{
public:
  BuffException(){m_remain = excptlen;} ;
  BuffException(const char *szException){
    do_except("Buffer Exception: ", szException) ;
  }
  void do_except(const char *szStr, const char *szException){
    strncpy(m_szException, szStr, excptlen-1) ;
    int len = strlen(szStr) ;
    m_remain = excptlen - 1 - len ;
    strncpy(m_szException+len, szException, m_remain);
  }
  virtual const char* what() const throw(){
    return m_szException ;
  }
protected:
  char m_szException[excptlen];
  int m_remain ;
} ;

class BuffMaxRetry : public std::exception{
public:
  virtual const char* what() const throw(){
    return "Max retries reached" ;
  }
};

class BuffIOErr : public BuffException{
public:
  BuffIOErr(const char *sz){
    do_except("Buff IO Err: ", sz) ;
  }
};

class BufferedRF24 : public NordicRF24{
public:
  BufferedRF24();
  ~BufferedRF24();

  // Writes a buffer of data to a receiver. Returns bytes written.
  // Cannot exceed RF24_BUFFER_WRITE length
  // Can throw BuffIOErr or BuffMaxRetry if blocking
  uint16_t write(uint8_t *buffer, uint16_t length, bool blocking) ;

  // Read data into a buffer. Returns bytes written. Returns 0 if no data is waiting and not blocking
  // Can throw BuffIOErr if blocking
  uint16_t read(uint8_t *buffer, uint16_t length, uint8_t pipe, bool blocking) ;

  enum enStatus {ok,max_retry_failure,io_err} ;

  // Call the write_status if using non-blocking write calls and find out if the
  // read or write was successful. 
  enStatus get_status();
  
protected:
  bool data_received_interrupt() ;
  bool max_retry_interrupt();
  bool data_sent_interrupt();

  uint8_t m_read_buffer[RF24_PIPES][RF24_BUFFER_READ];
  uint8_t m_write_buffer[RF24_BUFFER_WRITE];
  uint16_t m_read_size[RF24_PIPES], m_front_read[RF24_PIPES] ;
  uint16_t m_write_size, m_front_write ;

  enStatus m_status ;
  
private:
  pthread_mutex_t m_rwlock ;  
  
};


#endif
