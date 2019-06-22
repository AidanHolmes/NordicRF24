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
#include "radioutil.h"
#include <string.h>
#include <stdio.h>
#include <wchar.h>
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
  m_max_retries = 3 ;

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

  pthread_mutex_lock(&m_rwlock) ;
  uint8_t pipe = get_pipe_available();
  
  if (pipe == RF24_PIPE_EMPTY){
    pthread_mutex_unlock(&m_rwlock) ;
    return true ; // no pipe
  }

  // Not using blocking reads for inbound data as everything
  // is a single packet message

  // The buffered class implements a mutex to control
  // accessing data between interrupt thread and the data in the class
  
  for ( ; ; ){
    if (is_rx_empty()){
      pthread_mutex_unlock(&m_rwlock) ;
      break ;
    }
    bool ret = read_payload(packet, MQTT_PAYLOAD_WIDTH) ;
    pthread_mutex_unlock(&m_rwlock) ;
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
    throw MqttOutOfRange("Address too long") ;
  }
  
  if (broadcast) memcpy(m_broadcast, broadcast, address_len) ;
  if (address) memcpy(m_address, address, address_len) ;

  m_address_len = address_len ;

  auto_update(true);
  if (!set_address_width(m_address_len))
    throw MqttIOErr("cannot set address width") ;

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low))
    throw MqttIOErr("GPIO error") ;
  
  // User pipe 0 to receive transmitted responses or listen
  // on broadcast address
  enable_pipe(0, true) ;
  // Use pipe 1 for receiving unicast data
  enable_pipe(1, true) ;

  // No choice for payload width. Will be 32 bytes
  set_payload_width(0,MQTT_PAYLOAD_WIDTH) ;
  set_payload_width(1,MQTT_PAYLOAD_WIDTH) ;

  if (!set_rx_address(0, m_broadcast, m_address_len)){
    throw MqttIOErr("Cannot set RX address") ;
  }

  if (!set_rx_address(1, m_address, m_address_len)){
    throw MqttIOErr("Cannot set RX address") ;
  }

  power_up(true) ;
  nano_sleep(0,150000) ; // 150 micro second wait

  // Everything is a listener by default
  listen_mode() ;
  
}

void MqttSnRF24::shutdown()
{
  // Drop CE if transition is from listen mode
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low))
    throw MqttIOErr("GPIO error") ;

  // Power down
  power_up(false) ;
}

void MqttSnRF24::send_mode()
{
  if (!m_pGPIO) throw MqttIOErr("GPIO not set") ;

  pthread_mutex_lock(&m_rwlock) ;

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("GPIO error") ;
  }
  //flushtx() ;
  //flushrx() ;
  //clear_interrupts() ;

  receiver(false);
  nano_sleep(0,130000) ; // 130 micro second wait

  pthread_mutex_unlock(&m_rwlock) ;
}

void MqttSnRF24::listen_mode()
{
  if (!m_pGPIO) throw MqttIOErr("GPIO not set") ;

  //DPRINT("Listening...\n") ;
  pthread_mutex_lock(&m_rwlock) ;
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("GPIO error") ;
  }

  // Set pipe 0 to listen on the broadcast address
  if (!set_rx_address(0, m_broadcast, m_address_len)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("Cannot set RX address") ;
  }
  
  receiver(true);

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::high)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("GPIO error") ;
  }
  nano_sleep(0,130000) ; // 130 micro second wait

  //flushrx() ;
  //flushtx() ;
  //clear_interrupts() ;
  pthread_mutex_unlock(&m_rwlock) ;

}

void MqttSnRF24::queue_received(const uint8_t *addr,
				uint8_t messageid,
				const uint8_t *data,
				uint8_t len)
{
  pthread_mutex_lock(&m_rwlock) ;
  m_queue_head++ ;
  if (m_queue_head >= MAX_QUEUE) m_queue_head = 0 ;

  // Regardless of what's in the queue, either overwrite
  // an old entry or set a new one. 
  m_queue[m_queue_head].set = true ;
  m_queue[m_queue_head].messageid = messageid ;
  memcpy(m_queue[m_queue_head].address, addr, m_address_len) ;
  if (len > 0 && data != NULL)
    memcpy(m_queue[m_queue_head].message_data, data, len) ;
  m_queue[m_queue_head].message_len = len ;

  pthread_mutex_unlock(&m_rwlock) ;
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
	received_advertised(m_queue[queue_ptr].address,
			    m_queue[queue_ptr].message_data,
			    m_queue[queue_ptr].message_len) ;
	break;
      case MQTT_SEARCHGW:
	// Message from client to gateway
	received_searchgw(m_queue[queue_ptr].address,
			  m_queue[queue_ptr].message_data,
			  m_queue[queue_ptr].message_len) ;
	break;
      case MQTT_GWINFO:
	// Sent by gateways, although clients can also respond (not implemented)
	received_gwinfo(m_queue[queue_ptr].address,
			m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	
	break ;
      case MQTT_CONNECT:
	received_connect(m_queue[queue_ptr].address,
			 m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_CONNACK:
	received_connack(m_queue[queue_ptr].address,
			 m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_WILLTOPICREQ:
	received_willtopicreq(m_queue[queue_ptr].address,
			      m_queue[queue_ptr].message_data,
			      m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_WILLTOPIC:
	received_willtopic(m_queue[queue_ptr].address,
			   m_queue[queue_ptr].message_data,
			   m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_WILLMSGREQ:
	received_willmsgreq(m_queue[queue_ptr].address,
			    m_queue[queue_ptr].message_data,
			    m_queue[queue_ptr].message_len) ;

	break;
      case MQTT_WILLMSG:
	received_willmsg(m_queue[queue_ptr].address,
			 m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;

	break ;
      case MQTT_PINGREQ:
	received_pingreq(m_queue[queue_ptr].address,
			 m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PINGRESP:
	received_pingresp(m_queue[queue_ptr].address,
			  m_queue[queue_ptr].message_data,
			  m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_DISCONNECT:
	received_disconnect(m_queue[queue_ptr].address,
			    m_queue[queue_ptr].message_data,
			    m_queue[queue_ptr].message_len) ;
	break;
      case MQTT_REGISTER:
	received_register(m_queue[queue_ptr].address,
			  m_queue[queue_ptr].message_data,
			  m_queue[queue_ptr].message_len) ;
	break;
      case MQTT_REGACK:
	received_regack(m_queue[queue_ptr].address,
			m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBLISH:
	received_publish(m_queue[queue_ptr].address,
			 m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBACK:
	received_puback(m_queue[queue_ptr].address,
			m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBREC:
	received_pubrec(m_queue[queue_ptr].address,
			m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBREL:
	received_pubrel(m_queue[queue_ptr].address,
			m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_PUBCOMP:
	received_pubcomp(m_queue[queue_ptr].address,
			 m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_SUBSCRIBE:
	received_subscribe(m_queue[queue_ptr].address,
			   m_queue[queue_ptr].message_data,
			   m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_SUBACK:
	received_suback(m_queue[queue_ptr].address,
			m_queue[queue_ptr].message_data,
			m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_UNSUBSCRIBE:
	received_unsubscribe(m_queue[queue_ptr].address,
			     m_queue[queue_ptr].message_data,
			     m_queue[queue_ptr].message_len) ;
	break ;
      case MQTT_UNSUBACK:
	received_unsuback(m_queue[queue_ptr].address,
			  m_queue[queue_ptr].message_data,
			  m_queue[queue_ptr].message_len) ;
	break ;

      default:
	// Not expected message.
	// This is not a 1.2 MQTT message
	received_unknown(m_queue[queue_ptr].messageid,
			 m_queue[queue_ptr].address,
			 m_queue[queue_ptr].message_data,
			 m_queue[queue_ptr].message_len) ;
      }
    }
    pthread_mutex_lock(&m_rwlock) ;
    if (!failed) m_queue[queue_ptr].set = false ;
    queue_ptr++;
    if (queue_ptr >= MAX_QUEUE) queue_ptr = 0 ;
    pthread_mutex_unlock(&m_rwlock) ;

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
  bool ret = false ;
  uint8_t send_buff[MQTT_PAYLOAD_WIDTH] ;
  
  // includes the length field and message type
  uint8_t payload_len = len+MQTT_HDR_LEN; 

  //DPRINT("Transmitting...\n") ;

  if ((payload_len+m_address_len) > MQTT_PAYLOAD_WIDTH){
    throw MqttOutOfRange("Payload too long") ;
  }

  // Switch to write mode
  send_mode() ;

  pthread_mutex_lock(&m_rwlock) ;

  if (!set_tx_address(address, m_address_len)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("Cannot set TX address") ;
  }

  if (!set_rx_address(0, address, m_address_len)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("Cannot set RX address") ;
  }
  pthread_mutex_unlock(&m_rwlock) ;

  memcpy(send_buff, m_address, m_address_len) ;
  send_buff[0+m_address_len] = payload_len ;
  send_buff[1+m_address_len] = messageid ;
  if (buff != NULL && len > 0)
    memcpy(send_buff+MQTT_HDR_LEN+m_address_len, buff, len) ;

  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    try{
      // Write a blocking message
      write(send_buff, payload_len+m_address_len, true) ;
      ret = true ;
      break;
    }
    catch(BuffIOErr &e){
      listen_mode() ;
      throw MqttIOErr(e.what());
    }
    catch(BuffMaxRetry &e){
      nano_sleep(1,0) ;
    }
  }

  listen_mode() ;
  return ret ;
}

size_t MqttSnRF24::wchar_to_utf8(const wchar_t *wstr, char *outstr, const size_t maxbytes)
{
  size_t len = wcslen(wstr) ;
  if (len > maxbytes)
    throw MqttOutOfRange("conversion to utf8 too long") ;

  char *curlocale = setlocale(LC_CTYPE, NULL);
  
  if (!setlocale(LC_CTYPE, "en_GB.UTF-8")){
    throw MqttOutOfRange("cannot set UTF locale") ;
  }
    
  size_t ret = wcstombs(outstr, wstr, maxbytes) ;

  setlocale(LC_CTYPE, curlocale) ; // reset locale

  if (ret < 0){
    throw MqttOutOfRange("failed to convert wide string to utf8") ;
  }
  
  return ret ;
}
size_t utf8_to_wchar(const char *str, wchar_t *outstr, const size_t maxbytes)
{
  char *curloc = setlocale(LC_CTYPE, NULL);
  size_t ret = 0;

  if (!setlocale(LC_CTYPE, "en_GB.UTF-8")){
    throw MqttOutOfRange("cannot set UTF8 locale") ;

    ret = mbstowcs(outstr, str, maxbytes) ;
  }

  setlocale(LC_CTYPE, curloc) ; // reset locale
  
  if (ret < 0){
    throw MqttOutOfRange("failed to convert utf8 to wide string") ;
  }
  
  return ret ;  
}




