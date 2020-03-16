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

#include "mqttsnrf24.hpp"
#include <string.h>
#include <stdio.h>
#ifndef ARDUINO
 #include <wchar.h>
#endif
#include <stdlib.h>
#include <locale.h>

#ifndef _BV
#define _BV(x) 1 << x
#endif

#define MAX_GW_TXT 23

#define FLAG_DUP _BV(7)
#define FLAG_RETAIN _BV(4)
#define FLAG_WILL _BV(3)
#define FLAG_CLEANSESSION _BV(2)
#define FLAG_QOS0 0
#define FLAG_QOS1 _BV(5)
#define FLAG_QOS2 _BV(6)
#define FLAG_QOSN1 (_BV(5) | _BV(6))
#define FLAG_NORMAL_TOPIC_ID 0
#define FLAG_DEFINED_TOPIC_ID _BV(0)
#define FLAG_SHORT_TOPIC_NAME _BV(1)

#define MQTT_RETURN_ACCEPTED 0x00
#define MQTT_RETURN_CONGESTION 0x01
#define MQTT_RETURN_INVALID_TOPIC 0x02
#define MQTT_RETURN_NOT_SUPPORTED 0x03

#define MQTT_ADVERTISE 0x00
#define MQTT_GWINFO 0x02
#define MQTT_CONNECT 0x04
#define MQTT_WILLTOPICREQ 0x06
#define MQTT_WILLMSGREQ 0x08
#define MQTT_REGISTER 0x0A
#define MQTT_PUBLISH 0x0C
#define MQTT_PUBCOMP 0x0E
#define MQTT_PUBREL 0x10
#define MQTT_SUBSCRIBE 0x12
#define MQTT_UNSUBSCRIBE 0x14
#define MQTT_PINGREQ 0x16
#define MQTT_DISCONNECT 0x18
#define MQTT_WILLTOPICUPD 0x1A
#define MQTT_WILLMSGUPD 0x1C
#define MQTT_SEARCHGW 0x01
#define MQTT_CONNACK 0x05
#define MQTT_WILLTOPIC 0x07
#define MQTT_WILLMSG 0x09
#define MQTT_REGACK 0x0B
#define MQTT_PUBACK 0x0D
#define MQTT_PUBREC 0x0F
#define MQTT_SUBACK 0x13
#define MQTT_UNSUBACK 0x15
#define MQTT_PINGRESP 0x17
#define MQTT_WILLTOPICRESP 0x1B
#define MQTT_WILLMSGRESP 0x1D

#define MQTT_HDR_LEN 2

#ifdef DEBUG
#define DPRINT(x,...) fprintf(stdout,x,##__VA_ARGS__)
#define EPRINT(x,...) fprintf(stderr,x,##__VA_ARGS__)
#else
#define DPRINT(x,...)
#define EPRINT(x,...)
#endif

MqttSnRF24::MqttSnRF24()
{
  m_address_len = MIN_RF24_ADDRESS_LEN ; // Min length

  // Initialise the broadcast and device address with
  // default addresses.
  for (uint8_t i=0; i < MAX_RF24_ADDRESS_LEN; i++){
    m_broadcast[i] = 0xAA ;
    m_address[i] = 0xC7 ;
  }

  m_queue_head = 0 ;

  m_Tretry = 10; // sec
  m_Nretry = 5 ; // attempts
}

MqttSnRF24::~MqttSnRF24()
{

}

void MqttSnRF24::set_retry_attributes(uint16_t Tretry, uint16_t Nretry)
{
    m_Tretry = Tretry ;
    m_Nretry = Tretry ;
}

bool MqttSnRF24::data_received_interrupt()
{
  uint8_t packet[MQTT_PAYLOAD_WIDTH] ;
  uint8_t sender_addr[MAX_RF24_ADDRESS_LEN];

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
    bool ret = read_payload(packet, MQTT_PAYLOAD_WIDTH) ;
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    if (!ret){
      m_status = io_err ; // SPI error
      return false ;
    }

    // Extract the sender address
    memcpy(sender_addr, packet, m_address_len) ;
    // Read the MQTT-SN payload
    uint8_t length = packet[0+m_address_len] ;
    uint8_t messageid = packet[1+m_address_len] ;
    uint8_t *mqtt_payload = packet+MQTT_HDR_LEN+m_address_len ;
    
    if (length >= MQTT_HDR_LEN && length <= MQTT_PAYLOAD_WIDTH){
      // Valid length field. This may not be a MQTT message
      // for other values
      length -= MQTT_HDR_LEN ;
      queue_received(sender_addr, messageid, mqtt_payload, length) ;
    }      
  }
  
  return true ;
}


void MqttSnRF24::initialise(uint8_t address_len, uint8_t *broadcast, uint8_t *address)
{
  if (address_len > MAX_RF24_ADDRESS_LEN){
    EPRINT("Address too long") ;
    address_len = MAX_RF24_ADDRESS_LEN; // fix to max length, but not really fixing the programming error
  }
  
  if (broadcast) memcpy(m_broadcast, broadcast, address_len) ;
  if (address) memcpy(m_address, address, address_len) ;

  m_address_len = address_len ;

  auto_update(true);
  if (!set_address_width(m_address_len)){
    EPRINT("cannot set address width") ;
    return ;
  }

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
    EPRINT("GPIO error\n") ;
    return ;
  }
  
  // User pipe 0 to receive transmitted responses or listen
  // on broadcast address
  enable_pipe(0, true) ;
  // Use pipe 1 for receiving unicast data
  enable_pipe(1, true) ;

  // No choice for payload width. Will be 32 bytes
  set_payload_width(0,MQTT_PAYLOAD_WIDTH) ;
  set_payload_width(1,MQTT_PAYLOAD_WIDTH) ;

  if (!set_rx_address(0, m_broadcast, m_address_len)){
    EPRINT("Cannot set broadcast RX address\n") ;
  }

  if (!set_rx_address(1, m_address, m_address_len)){
    EPRINT("Cannot set RX address\n") ;
  }

  power_up(true) ;
  m_pTimer->microSleep(150); // 150 micro second wait

  // Everything is a listener by default
  listen_mode() ;
  
}

void MqttSnRF24::shutdown()
{
  // Drop CE if transition is from listen mode
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low))
    EPRINT("GPIO error, cannot shutdown\n") ;

  // Power down
  power_up(false) ;
}

void MqttSnRF24::send_mode()
{
  if (!m_pGPIO){
    EPRINT("GPIO not set\n") ;
    return ; // Terminal problem - GPIO interface required
  }
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    EPRINT("GPIO error\n") ;
    return ;
  }
  //flushtx() ;
  //flushrx() ;
  //clear_interrupts() ;

  receiver(false);
  m_pTimer->microSleep(130); // 130 micro second wait

#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif
}

void MqttSnRF24::listen_mode()
{
  if (!m_pGPIO){ 
    EPRINT("GPIO not set") ;
    return ;
  }

  //DPRINT("Listening...\n") ;
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    EPRINT("GPIO error\n") ;
    return ; // Terminal problem
  }

  // Set pipe 0 to listen on the broadcast address
  if (!set_rx_address(0, m_broadcast, m_address_len)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    EPRINT("Cannot set RX address\n") ;
    return ;
  }
  
  receiver(true);

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::high)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    EPRINT("GPIO error") ;
    return ; // Fairly terminal error if GPIO cannot be set
  }
  m_pTimer->microSleep(130); // 130 micro second wait

  //flushrx() ;
  //flushtx() ;
  //clear_interrupts() ;
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif
}

void MqttSnRF24::queue_received(const uint8_t *addr,
				uint8_t messageid,
				const uint8_t *data,
				uint8_t len)
{
#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif
  m_queue_head++ ;
  if (m_queue_head >= MAX_QUEUE) m_queue_head = 0 ;

  // Regardless of what's in the queue, either overwrite
  // an old entry or set a new one. 
  m_queue[m_queue_head].set = true ;
  m_queue[m_queue_head].messageid = messageid ;
  memcpy((void*)m_queue[m_queue_head].address, addr, m_address_len) ;
  if (len > 0 && data != NULL)
    memcpy((void*)m_queue[m_queue_head].message_data, data, len) ;
  m_queue[m_queue_head].message_len = len ;

#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif
}

bool MqttSnRF24::manage_pending_message(MqttConnection &con)
{
  // Client handling of pending message
  if(con.state_timeout(m_Tretry)){
    if(con.state_timeout_count() >= m_Nretry){
      // Exceeded number of allowed retries.
      return false ;
    }else{
      // Retry
      addrwritemqtt(con.get_address(),
		    con.get_cache_id(),
		    con.get_cache(),
		    con.get_cache_len());
    }
  }
  return true ;
}

bool MqttSnRF24::dispatch_queue()
{
  bool ret = true ;
  uint8_t queue_ptr = m_queue_head ;
  bool failed = false ;
  
  for ( ; ;){ // it's a do...while loop
    failed = false ;
    if (m_queue[queue_ptr].set){
      switch(m_queue[queue_ptr].messageid){
      case MQTT_ADVERTISE:
	// Gateway message received
	received_advertised((uint8_t*)m_queue[queue_ptr].address,
			    (uint8_t*)m_queue[queue_ptr].message_data,
			    m_queue[queue_ptr].message_len) ;
	break;
      case MQTT_SEARCHGW:
	// Message from client to gateway
	received_searchgw((uint8_t*)m_queue[queue_ptr].address,
			  (uint8_t*)m_queue[queue_ptr].message_data,
			  m_queue[queue_ptr].message_len) ;
	break;
      case MQTT_GWINFO:
	// Sent by gateways, although clients can also respond (not implemented)
	received_gwinfo((uint8_t*)m_queue[queue_ptr].address,
			(uint8_t*)m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	
	break ;
      case MQTT_CONNECT:
	received_connect((uint8_t*)m_queue[queue_ptr].address,
			 (uint8_t*)m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_CONNACK:
	received_connack((uint8_t*)m_queue[queue_ptr].address,
			 (uint8_t*)m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_WILLTOPICREQ:
	received_willtopicreq((uint8_t*)m_queue[queue_ptr].address,
			      (uint8_t*)m_queue[queue_ptr].message_data,
			      m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_WILLTOPIC:
	received_willtopic((uint8_t*)m_queue[queue_ptr].address,
			   (uint8_t*)m_queue[queue_ptr].message_data,
			   m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_WILLMSGREQ:
	received_willmsgreq((uint8_t*)m_queue[queue_ptr].address,
			    (uint8_t*)m_queue[queue_ptr].message_data,
			    m_queue[queue_ptr].message_len) ;

	break;
      case MQTT_WILLMSG:
	received_willmsg((uint8_t*)m_queue[queue_ptr].address,
			 (uint8_t*)m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_PINGREQ:
	received_pingreq((uint8_t*)m_queue[queue_ptr].address,
			 (uint8_t*)m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PINGRESP:
	received_pingresp((uint8_t*)m_queue[queue_ptr].address,
			  (uint8_t*)m_queue[queue_ptr].message_data,
			  m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_DISCONNECT:
	received_disconnect((uint8_t*)m_queue[queue_ptr].address,
			    (uint8_t*)m_queue[queue_ptr].message_data,
			    m_queue[queue_ptr].message_len) ;
	break;
      case MQTT_REGISTER:
	received_register((uint8_t*)m_queue[queue_ptr].address,
			  (uint8_t*)m_queue[queue_ptr].message_data,
			  m_queue[queue_ptr].message_len) ;
	break;
      case MQTT_REGACK:
	received_regack((uint8_t*)m_queue[queue_ptr].address,
			(uint8_t*)m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBLISH:
	received_publish((uint8_t*)m_queue[queue_ptr].address,
			 (uint8_t*)m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBACK:
	received_puback((uint8_t*)m_queue[queue_ptr].address,
			(uint8_t*)m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBREC:
	received_pubrec((uint8_t*)m_queue[queue_ptr].address,
			(uint8_t*)m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBREL:
	received_pubrel((uint8_t*)m_queue[queue_ptr].address,
			(uint8_t*)m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBCOMP:
	received_pubcomp((uint8_t*)m_queue[queue_ptr].address,
			 (uint8_t*)m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_SUBSCRIBE:
	received_subscribe((uint8_t*)m_queue[queue_ptr].address,
			   (uint8_t*)m_queue[queue_ptr].message_data,
			   m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_SUBACK:
	received_suback((uint8_t*)m_queue[queue_ptr].address,
			(uint8_t*)m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_UNSUBSCRIBE:
	received_unsubscribe((uint8_t*)m_queue[queue_ptr].address,
			     (uint8_t*)m_queue[queue_ptr].message_data,
			     m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_UNSUBACK:
	received_unsuback((uint8_t*)m_queue[queue_ptr].address,
			  (uint8_t*)m_queue[queue_ptr].message_data,
			  m_queue[queue_ptr].message_len) ;
	break ;

      default:
	// Not expected message.
	// This is not a 1.2 MQTT message
	received_unknown(m_queue[queue_ptr].messageid,
			 (uint8_t*)m_queue[queue_ptr].address,
			 (uint8_t*)m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;
      }
    }
#ifndef ARDUINO
    pthread_mutex_lock(&m_rwlock) ;
#endif
    if (!failed) m_queue[queue_ptr].set = false ;
    queue_ptr++;
    if (queue_ptr >= MAX_QUEUE) queue_ptr = 0 ;
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    if (queue_ptr == m_queue_head) break;
  }
  return ret ;
}

bool MqttSnRF24::writemqtt(MqttConnection *con,
			   uint8_t messageid,
			   const uint8_t *buff, uint8_t len)
{
  const uint8_t *address = con->get_address() ;
  if (addrwritemqtt(address,messageid,buff,len)){
    con->set_cache(messageid, buff, len) ;
    return true ;
  }
  return false;
}

bool MqttSnRF24::addrwritemqtt(const uint8_t *address,
			       uint8_t messageid,
			       const uint8_t *buff,
			       uint8_t len)
{
  uint8_t send_buff[MQTT_PAYLOAD_WIDTH] ;
  
  // includes the length field and message type
  uint8_t payload_len = len+MQTT_HDR_LEN; 

  //DPRINT("Transmitting...\n") ;

  if ((payload_len+m_address_len) > MQTT_PAYLOAD_WIDTH){
    EPRINT("Payload too long") ;
    return false ;
  }

  // Switch to write mode
  send_mode() ;

#ifndef ARDUINO
  pthread_mutex_lock(&m_rwlock) ;
#endif

  if (!set_tx_address(address, m_address_len)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    EPRINT("Cannot set TX address\n") ;
    return false ;
  }

  if (!set_rx_address(0, address, m_address_len)){
#ifndef ARDUINO
    pthread_mutex_unlock(&m_rwlock) ;
#endif
    EPRINT("Cannot set RX address\n") ;
    return false ;
  }
#ifndef ARDUINO
  pthread_mutex_unlock(&m_rwlock) ;
#endif
  memcpy(send_buff, m_address, m_address_len) ;
  send_buff[0+m_address_len] = payload_len ;
  send_buff[1+m_address_len] = messageid ;
  if (buff != NULL && len > 0)
    memcpy(send_buff+MQTT_HDR_LEN+m_address_len, buff, len) ;

  write(send_buff, payload_len+m_address_len, true) ;
  enStatus sret = get_status() ;
  if (sret == io_err){
    listen_mode();
    EPRINT("IO Error on addrwritemqtt\n") ;
    return false ;

  }else if (sret == buff_overflow){
    EPRINT("Data exceeds buffer size\n") ;
    return false ;
  }

  listen_mode() ;
  return true ;
}

#ifndef ARDUINO
size_t MqttSnRF24::wchar_to_utf8(const wchar_t *wstr, char *outstr, const size_t maxbytes)
{
  size_t len = wcslen(wstr) ;
  if (len > maxbytes){
    EPRINT("Conversion to UTF8 too long\n");
    return 0;
  }

  char *curlocale = setlocale(LC_CTYPE, NULL);
  
  if (!setlocale(LC_CTYPE, "en_GB.UTF-8")){
    EPRINT("Cannot set UTF local to en_GB\n") ;
    return 0 ;
  }
    
  size_t ret = wcstombs(outstr, wstr, maxbytes) ;

  setlocale(LC_CTYPE, curlocale) ; // reset locale

  if (ret < 0){
    EPRINT("Failed to convert wide string to UTF8\n") ;
  }
  
  return ret ;
}
#endif
#ifndef ARDUINO
size_t utf8_to_wchar(const char *str, wchar_t *outstr, const size_t maxbytes)
{
  char *curloc = setlocale(LC_CTYPE, NULL);
  size_t ret = 0;

  if (!setlocale(LC_CTYPE, "en_GB.UTF-8")){
    EPRINT("Cannot set UTF8 locale en_GB\n") ;
    return 0;
  }

  ret = mbstowcs(outstr, str, maxbytes) ;

  setlocale(LC_CTYPE, curloc) ; // reset locale
  
  if (ret < 0){
    EPRINT("Failed to convert utf8 to wide string\n") ;
  }
  
  return ret ;  
}

#endif

