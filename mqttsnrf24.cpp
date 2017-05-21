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

MqttSnRF24::MqttSnRF24()
{
  m_address_len = 2 ; // Similar to ZigBee

  // Initialise the broadcast and device address with
  // default addresses.
  for (uint8_t i=0; i < MAX_RF24_ADDRESS_LEN; i++){
    m_broadcast[i] = 0xAA ;
    m_address[i] = 0xC7 ;
  }

  strcpy(m_szclient_id, "me") ;
  
  m_gwid = 0 ;

  m_mqtt_type = client ;

  m_queue_head = 0 ;
}

MqttSnRF24::~MqttSnRF24()
{
  
}

bool MqttSnRF24::data_received_interrupt()
{
  uint8_t packet[MQTT_PAYLOAD_WIDTH] ;

  pthread_mutex_lock(&m_rwlock) ;
  uint8_t pipe = get_pipe_available();
  pthread_mutex_unlock(&m_rwlock) ;

  if (pipe == RF24_PIPE_EMPTY) return true ; // no pipe

  // Not using blocking reads for inbound data as everything
  // is a single packet message

  // The buffered class implements a mutex to control
  // accessing data between interrupt thread and the data in the class
  
  for ( ; ; ){
    // Ensure locking is kept short
    pthread_mutex_lock(&m_rwlock) ;
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

    // Read the MQTT-SN payload
    uint8_t length = packet[0] ;
    uint8_t messageid = packet[1] ;
    
    if (length >= 2 && length <= MQTT_PAYLOAD_WIDTH){
      // Valid length field. This may not be a MQTT message
      // for other values
      length -= 2 ;
      switch(messageid){
      case MQTT_ADVERTISE:
	// Gateway message received
	received_advertised(packet+2, length) ;
	break;
      case MQTT_SEARCHGW:
	// Message from client to gateway
	received_searchgw(packet+2, length) ;
	break;
      case MQTT_GWINFO:
	// Sent by gateways, although clients can also respond (not implemented)
	received_gwinfo(packet+2, length) ;
	break ;
	//default:
	// Not expected message.
	// This is not a 1.2 MQTT message
      }
    }
  }
  
  return true ;
}

void MqttSnRF24::received_advertised(uint8_t *data, uint8_t len)
{
  uint16_t duration = (data[1] << 8) | data[2] ; // Assuming MSB is first
  printf("ADVERTISED: {gw = %u, duration = %u}\n", data[0], duration) ;
  
  if (m_mqtt_type == client){
    // Since RF24 doesn't actually let the receiver know who sent the
    // packet the gateway info cannot be complete. All that can be done is
    // to check if the gateway is already known and reactivate
    // Assumes address hasn't changed if it is active again.
    for (uint8_t i=0;i<MAX_GATEWAYS;i++){
      if (m_gwinfo[i].allocated &&
	  m_gwinfo[i].gw_id == data[2] &&
	  !m_gwinfo[i].active){
	m_gwinfo[i].active = true ;
	m_gwinfo[i].advertised(duration) ;
	break ;
      }
    }
  }
}


void MqttSnRF24::received_searchgw(uint8_t *data, uint8_t len)
{
  printf("SEARCHGW: {radius = %u}\n", data[0]) ;
  if (m_mqtt_type == gateway){
    uint8_t buff[MAX_RF24_ADDRESS_LEN+1] ;
    uint8_t data_len = m_address_len + 1 ;
    // Ignore radius value. This is a gw so respond with
    // a broadcast message back.
    // Since RF24 doesn't communicate the unicast address it will be in this return packet
    buff[0] = m_gwid ;
    memcpy(buff+1, m_address, m_address_len) ;
    queue_response(m_broadcast, MQTT_GWINFO, buff, data_len) ;
  }
}
void MqttSnRF24::received_gwinfo(uint8_t *data, uint8_t len)
{
  printf("GWINFO: {gw = %u, address = ", data[0]) ;
  for (uint8_t j=0; j < len - 1; j++) printf("%X", data[j+1]) ;
  printf("}\n") ;
  
  bool gw_updated = false  ;
  if (m_mqtt_type == client){
    uint8_t addr_len = len - 1 ;
    if (addr_len != m_address_len){
      // Gateway is using a different spec or hasn't set an address!
      // Cannot process this gateway
      return ;
    }
    for (uint8_t i=0; i < MAX_GATEWAYS; i++){
      if (m_gwinfo[i].allocated &&
	  m_gwinfo[i].gw_id == data[0]){
	// Already have a record of this gateway.
	// Update the details
	m_gwinfo[i].address_length = addr_len ;
	memcpy(m_gwinfo[i].address, data+1, addr_len) ;
	gw_updated = true ;
	break ;
      }
    }
    if (!gw_updated){
      // Insert new gateway. Overwrite old or expired gateways
      for (uint8_t i=0; i < MAX_GATEWAYS; i++){
	if (!m_gwinfo[i].allocated || !m_gwinfo[i].active || m_gwinfo[i].advertised_expired()){
	  m_gwinfo[i].address_length = addr_len ;
	  m_gwinfo[i].allocated = true ;
	  m_gwinfo[i].active = true ;
	  m_gwinfo[i].gw_id = data[0];
	  memcpy(m_gwinfo[i].address, data+1, addr_len);
	  m_gwinfo[i].advertised(0) ; // no idea if it's going to advertise
	}
      }
    }
    // It's possible that the gateway cannot be saved if there's already a full list
    // of gateways.
    // Why is the client requesting the info though??? Only needs one GW so not an error
  }
}

void MqttSnRF24::set_client_id(const char *szclientid)
{
  strncpy(m_szclient_id, szclientid, MAX_MQTT_CLIENTID) ;
}

const char* MqttSnRF24::get_client_id()
{
  return m_szclient_id ;
}

void MqttSnRF24::initialise(enType type, uint8_t address_len, uint8_t *broadcast, uint8_t *address)
{
  if (address_len > MAX_RF24_ADDRESS_LEN){
    throw MqttOutOfRange("Address too long") ;
  }
  
  if (broadcast) memcpy(m_broadcast, broadcast, address_len) ;
  if (address) memcpy(m_address, address, address_len) ;

  m_address_len = address_len ;
  m_mqtt_type = type ;

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

  uint8_t l = m_address_len ;
  if (!set_rx_address(0, m_broadcast, &l)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("Cannot set RX address") ;
  }

  l = m_address_len ;
  if (!set_rx_address(1, m_address, &l)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("Cannot set RX address") ;
  }

  /*
  l = m_address_len ;
  if (!set_tx_address(m_broadcast, &l)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("Cannot set TX address") ;
  }
  */
  
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
  uint8_t l = m_address_len ;

  //printf("Listening...\n") ;
  pthread_mutex_lock(&m_rwlock) ;
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("GPIO error") ;
  }

  // Set pipe 0 to listen on the broadcast address
  if (!set_rx_address(0, m_broadcast, &l)){
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

void MqttSnRF24::queue_response(uint8_t *addr,
				  uint8_t messageid,
				  uint8_t *data,
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
  memcpy(m_queue[m_queue_head].message_data, data, len) ;
  m_queue[m_queue_head].message_len = len ;

  pthread_mutex_unlock(&m_rwlock) ;
}

void MqttSnRF24::dispatch_queue()
{
  uint8_t queue_ptr = m_queue_head ;

  for ( ; ;){ // it's a do...while loop
    if (m_queue[queue_ptr].set){

      writemqtt(m_queue[queue_ptr].address,
		  m_queue[queue_ptr].messageid,
		  m_queue[queue_ptr].message_data,
		  m_queue[queue_ptr].message_len) ;
  
    }
    pthread_mutex_lock(&m_rwlock) ;
    m_queue[queue_ptr].set = false ;
    queue_ptr++;
    if (queue_ptr >= MAX_QUEUE) queue_ptr = 0 ;
    pthread_mutex_unlock(&m_rwlock) ;

    if (queue_ptr == m_queue_head) break;
  }
}

void MqttSnRF24::writemqtt(uint8_t *address,
			   uint8_t messageid,
			   uint8_t *buff, uint8_t len)
{
  uint8_t send_buff[MQTT_PAYLOAD_WIDTH] ;
  uint8_t l = m_address_len ;
  
  // includes the length field and message type
  uint8_t payload_len = len+MQTT_HDR_LEN; 
  
  //printf("Transmitting...\n") ;
  
  if (payload_len > MQTT_PAYLOAD_WIDTH){
    throw MqttOutOfRange("Payload too long") ;
  }

  pthread_mutex_lock(&m_rwlock) ;
  
  if (!set_tx_address(address, &l)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("Cannot set TX address") ;
  }
  l = m_address_len ;
  if (!set_rx_address(0, address, &l)){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttIOErr("Cannot set RX address") ;
  }
  pthread_mutex_unlock(&m_rwlock) ;

  // Switch to write mode
  send_mode() ;

  send_buff[0] = payload_len ;
  send_buff[1] = messageid ;
  memcpy(send_buff+MQTT_HDR_LEN, buff, len) ;

  try{
    // Write a blocking message
    write(send_buff, payload_len, true) ;
  }
  catch(BuffIOErr &e){
    listen_mode() ;
    //throw MqttIOErr("write call failed") ;
    throw ;
  }
  catch(BuffMaxRetry &e){
    //fprintf(stdout, "Failed to write MQTT data\n") ;
    // ignore the exception
  }

  listen_mode() ;
}

void MqttSnRF24::searchgw(uint8_t radius)
{
  uint8_t buff[1] ;
  buff[0] = radius ;
  writemqtt(m_broadcast, MQTT_SEARCHGW, buff, 1) ;
}

void MqttSnRF24::advertise(uint16_t duration)
{
  uint8_t buff[3] ;
  buff[0] = m_gwid ;
  buff[1] = duration >> 8 ; // Is this MSB first or LSB first?
  buff[2] = duration & 0x00FF;
  writemqtt(m_broadcast, MQTT_ADVERTISE, buff, 3) ;
}
