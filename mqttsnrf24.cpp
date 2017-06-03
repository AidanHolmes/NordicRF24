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

  strcpy(m_szclient_id, "RF24") ;  
  m_gwid = 0 ;
  m_mqtt_type = client ;
  m_queue_head = 0 ;
  m_max_retries = 3 ;
  m_connect_timeout = 20 ; // sec
  m_connection_head = NULL ;

  m_willmessage[0] = '\0' ;
  m_willmessagesize = 0 ;
  m_willtopic[0] = '\0' ;
  m_willtopicsize = 0 ;
  m_willtopicqos = 0;
}

MqttSnRF24::~MqttSnRF24()
{
  
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
      switch(messageid){
      case MQTT_ADVERTISE:
	// Gateway message received
	received_advertised(sender_addr, mqtt_payload, length) ;
	break;
      case MQTT_SEARCHGW:
	// Message from client to gateway
	received_searchgw(sender_addr, mqtt_payload, length) ;
	break;
      case MQTT_GWINFO:
	// Sent by gateways, although clients can also respond (not implemented)
	received_gwinfo(sender_addr, mqtt_payload, length) ;
	break ;
      case MQTT_CONNECT:
	received_connect(sender_addr, mqtt_payload, length) ;
	break ;
      case MQTT_CONNACK:
	received_connack(sender_addr, mqtt_payload, length) ;
	break ;
      case MQTT_WILLTOPICREQ:
	received_willtopicreq(sender_addr, mqtt_payload, length) ;
	break ;
      case MQTT_WILLTOPIC:
	received_willtopic(sender_addr, mqtt_payload, length) ;
	break ;
      case MQTT_WILLMSGREQ:
	received_willmsgreq(sender_addr, mqtt_payload, length) ;
	break;
      case MQTT_WILLMSG:
	received_willmsg(sender_addr, mqtt_payload, length) ;
	break ;
      case MQTT_PINGREQ:
	received_pingreq(sender_addr, mqtt_payload, length) ;
	break ;
      case MQTT_PINGRESP:
	received_pingresp(sender_addr, mqtt_payload, length) ;
	break ;
	//default:
	// Not expected message.
	// This is not a 1.2 MQTT message
      }
    }      
  }
  
  return true ;
}
void MqttSnRF24::received_pingresp(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  DPRINT("PINGRESP\n") ;
  pthread_mutex_lock(&m_rwlock) ;
  if (m_mqtt_type == gateway){
    MqttConnection *con = search_connection_address(sender_address) ;
    if (con){
      // just update the last activity timestamp
      con->lastactivity = time(NULL) ;
    }
  }else if (m_mqtt_type == client){
    MqttGwInfo *gw = get_gateway_address(sender_address);
    if (gw){
      gw->ping() ;
    }
  }
  pthread_mutex_unlock(&m_rwlock) ;

}

void MqttSnRF24::received_pingreq(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  // regardless of client or gateway just send the ACK
  #ifdef DEBUG
  if (len > 0){ //contains a client id
    char szclientid[MAX_MQTT_CLIENTID+1];
    memcpy(szclientid, data, len);
    szclientid[len] = '\0' ;
    DPRINT("PINGREQ: {clientid = %s}\n", szclientid) ;
  }else{
    DPRINT("PINGREQ\n") ;
  }
  #endif
 
  queue_response(sender_address, MQTT_PINGRESP, NULL, 0) ;
 
}

void MqttSnRF24::received_advertised(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  uint16_t duration = (data[1] << 8) | data[2] ; // Assuming MSB is first
  DPRINT("ADVERTISED: {gw = %u, duration = %u}\n", data[0], duration) ;
  
  if (m_mqtt_type == client){
    // Since RF24 doesn't actually let the receiver know who sent the
    // packet the gateway info cannot be complete. All that can be done is
    // to check if the gateway is already known and reactivate
    // Assumes address hasn't changed if it is active again.
    pthread_mutex_lock(&m_rwlock) ;

    for (uint8_t i=0;i<MAX_GATEWAYS;i++){
      if (m_gwinfo[i].allocated &&
	  m_gwinfo[i].gw_id == data[2] &&
	  !m_gwinfo[i].active){
	m_gwinfo[i].active = true ;
	m_gwinfo[i].advertised(duration) ;
	break ;
      }
    }
    pthread_mutex_unlock(&m_rwlock) ;
  }
}


void MqttSnRF24::received_searchgw(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  uint8_t buff[1] ;

  DPRINT("SEARCHGW: {radius = %u}\n", data[0]) ;

  if (m_mqtt_type == gateway){
    // Ignore radius value. This is a gw so respond with
    // a broadcast message back.
    buff[0] = m_gwid ;
    queue_response(m_broadcast, MQTT_GWINFO, buff, 1) ;
  }
}
void MqttSnRF24::received_gwinfo(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  DPRINT("GWINFO: {gw = %u, address = ", data[0]) ;
  for (uint8_t j=0; j < len - 1; j++) DPRINT("%X", data[j+1]) ;
  DPRINT("}\n") ;
  
  bool gw_updated = false  ;
  if (m_mqtt_type == client){
    pthread_mutex_lock(&m_rwlock) ;

    for (uint8_t i=0; i < MAX_GATEWAYS; i++){
      if (m_gwinfo[i].allocated &&
	  m_gwinfo[i].gw_id == data[0]){
	// Already have a record of this gateway.
	// Update the details
	m_gwinfo[i].address_length = m_address_len ;
	if (len == m_address_len+1) // Was the address populated in GWINFO?
	  memcpy(m_gwinfo[i].address, data+1, m_address_len) ;
	else // No address in GWINFO, assume sender is a gateway
	  memcpy(m_gwinfo[i].address, sender_address, m_address_len) ;
	gw_updated = true ;
	m_gwinfo[i].ping() ; // update last gw activity
	break ;
      }
    }
    if (!gw_updated){
      // Insert new gateway. Overwrite old or expired gateways
      for (uint8_t i=0; i < MAX_GATEWAYS; i++){
	if (!m_gwinfo[i].allocated || !m_gwinfo[i].active || m_gwinfo[i].advertised_expired()){
	  m_gwinfo[i].address_length = m_address_len ;
	  m_gwinfo[i].allocated = true ;
	  m_gwinfo[i].active = true ;
	  m_gwinfo[i].gw_id = data[0];
	  if (len == m_address_len+1) // Was the address populated in GWINFO?
	    memcpy(m_gwinfo[i].address, data+1, m_address_len) ;
	  else // No address in GWINFO, assume sender is a gateway
	    memcpy(m_gwinfo[i].address, sender_address, m_address_len) ;
	  m_gwinfo[i].advertised(64800) ; // no idea if it's going to advertise
	}
      }
    }
    pthread_mutex_unlock(&m_rwlock) ;

    // It's possible that the gateway cannot be saved if there's already a full list
    // of gateways.
    // Why is the client requesting the info though??? Only needs one GW so not an error
  }
}

MqttConnection* MqttSnRF24::search_connection(char *szclientid)
{
  MqttConnection *p = NULL ;
  for(p = m_connection_head; p != NULL; p=p->next){
    if (p->enabled){
      if (strcmp(p->szclientid, szclientid) == 0) break ;
    }
  }
  return p ;
}

MqttConnection* MqttSnRF24::search_connection_address(uint8_t *clientaddr)
{
  MqttConnection *p = NULL ;
  uint8_t a = 0;
  for(p = m_connection_head; p != NULL; p=p->next){
    if (p->enabled){
      for (a=0; a < m_address_len; a++){
	if(p->connect_address[a] != clientaddr[a]) break ;
      }
      if (a == m_address_len) return p ;
    }
  }
  return NULL ; // Cannot find
}

MqttConnection* MqttSnRF24::new_connection()
{
  MqttConnection *p = NULL, *prev = NULL ;
  
  for(p = m_connection_head; p != NULL; p=p->next){
    if (!p->enabled){
      // reuse this record
      return p ;
    }
    prev = p ;
  }
  // p not found, create new connection at the end of the list
  p = new MqttConnection() ;
  p->prev = prev ; // could be null if the first record
  // Set the head if first record
  if (m_connection_head == NULL) m_connection_head = p ;

  return p ;
}

void MqttSnRF24::delete_connection(char *szclientid)
{
  MqttConnection *p = NULL, *prev = NULL, *next = NULL ;

  // Search for all client id instances and remove
  while ((p = search_connection(szclientid))){
    prev = p->prev ;
    next = p->next ;
    delete p ;
    if (!prev) m_connection_head = NULL ; // this was the head
    else prev->next = next ; // Connect the head and tail records
  }
}

void MqttSnRF24::received_connect(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  char szClientID[MAX_MQTT_CLIENTID+1] ;
  if (len < 5 || len > (4 + MAX_MQTT_CLIENTID)) return ; // invalid data length
  
  if (m_mqtt_type == gateway){
    memcpy(szClientID, data+4, len - 4) ; // copy identifier
    szClientID[len-4] = '\0' ; // Create null terminated string

    DPRINT("CONNECT: {flags = %02X, protocol = %02X, duration = %u, client ID = %s\n", data[0], data[1], (data[2] << 8) | data[3], szClientID) ;

    if (data[1] != MQTT_PROTOCOL){
      EPRINT("Invalid protocol ID in CONNECT from client %s\n", szClientID);
      return ;
    }

    pthread_mutex_lock(&m_rwlock) ;

    MqttConnection *con = search_connection(szClientID) ;
    if (!con){
      DPRINT("Cannot find an existing connection, creating a new new connection for %s\n", szClientID) ;
      con = new_connection() ;
    }
    if (!con){
      EPRINT("Cannot create a new connection record for client %s\n", szClientID) ;
      pthread_mutex_unlock(&m_rwlock) ;
      return ; // something went wrong with the allocation
    }
    
    con->enabled = true ;
    con->disconnected = false ;
    con->sleep_duration = 0 ;
    con->asleep_from = 0 ;
    con->ping() ; // update activity from client
    strcpy(con->szclientid, szClientID) ;
    con->duration = (data[2] << 8) | data[3] ; // MSB assumed
    memcpy(con->connect_address, sender_address, m_address_len) ;
    
    // If WILL if flagged then set the flags for the message and topic
    bool will = ((FLAG_WILL & data[0]) > 0) ;
    con->prompt_will_topic = will ;
    con->prompt_will_message = will ;
    con->connection_complete = false ; 

    pthread_mutex_unlock(&m_rwlock) ;

    if (con->prompt_will_topic){
      // Start with will topic request
      queue_response(sender_address, MQTT_WILLTOPICREQ, NULL, 0) ;
    }else{
      // No need for WILL setup, just CONNACK
      uint8_t buff[1] ;
      buff[0] = MQTT_RETURN_ACCEPTED ;
      queue_response(sender_address, MQTT_CONNACK, buff, 1) ;
    }
    
  }
  // Clients ignore this
}

void MqttSnRF24::received_connack(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  // This message isn't for gateways so just deal with the client case
  if (m_mqtt_type == client){
    pthread_mutex_lock(&m_rwlock) ;

    // Was this client connecting?
    if (!m_client_connection.enabled) return ; // not enabled and expected

    // TO DO - confirm that the gateway sending this matches the address expected
    // not a major fault but is worth a check

    if (m_client_connection.prompt_will_topic || m_client_connection.prompt_will_message){
      EPRINT("Connection complete, but will topic or message not prompted\n") ;
    }

    // TO DO: Needs to handle the errors properly and inform client of error
    // so correct behaviour can follow.
    switch(data[0]){
    case MQTT_RETURN_ACCEPTED:
      DPRINT("CONNACK: {return code = Accepted}\n") ;
      m_client_connection.connection_complete = true ;
      break ;
    case MQTT_RETURN_CONGESTION:
      DPRINT("CONNACK: {return code = Congestion}\n") ;
      m_client_connection.enabled = false ; // Cannot connect
      break ;
    case MQTT_RETURN_INVALID_TOPIC:
      DPRINT("CONNACK: {return code = Invalid Topic}\n") ;
      // Can this still count as a connection?
      m_client_connection.enabled = false ; // don't allow?
      break ;
    case MQTT_RETURN_NOT_SUPPORTED:
      DPRINT("CONNACK: {return code = Not Supported}\n") ;
      m_client_connection.enabled = false ; // Cannot connect
      break ;
    default:
      DPRINT("CONNACK: {return code = %u}\n", data[0]) ;
      // ? Are we connected ?
      m_client_connection.enabled = false ; // Cannot connect
    }
    
    pthread_mutex_unlock(&m_rwlock) ;

  }
}

void MqttSnRF24::received_willtopicreq(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  // Only clients need to respond to this
  if (m_mqtt_type == client){
    DPRINT("WILLTOPICREQ\n") ;
    if (m_willtopicsize == 0){
      // No topic set
      queue_response(sender_address, MQTT_WILLTOPIC, NULL, 0) ;
    }else{
      uint8_t send[1+MQTT_TOPIC_MAX_BYTES];
      send[0] = 0 ;
      switch(m_willtopicqos){
      case 0:
	send[0] = FLAG_QOS0 ;
	break ;
      case 1:
	send[0] = FLAG_QOS1 ;
	break ;
      case 2:
      default: // Ignore other values and set to max QOS
	send[0] = FLAG_QOS2 ;
      }
      // Any overflow of size should have been checked so shouldn't need to check again here.
      memcpy(send+1, m_willtopic, m_willtopicsize) ;
      queue_response(sender_address, MQTT_WILLTOPIC, send, m_willtopicsize+1) ;
    }
    m_client_connection.prompt_will_topic = false ; 
  }
}

void MqttSnRF24::received_willtopic(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (m_mqtt_type == gateway){
    char utf8[MQTT_TOPIC_MAX_BYTES+1] ;
    wchar_t topic[MQTT_TOPIC_MAX_BYTES+1] ;

    MqttConnection *con = search_connection_address(sender_address) ;
    uint8_t buff[1] ;
    if (!con){
      EPRINT("WillTopic could not find the client connection\n") ;
      buff[0] = MQTT_RETURN_CONGESTION ; // There is no "who are you?" response so this will do
      queue_response(sender_address, MQTT_CONNACK, buff, 1) ;
      return ;
    }

    con->ping() ; // update known client activity
    
    memcpy(utf8, data+1, len-1) ;
    utf8[len-1] = '\0' ;
    size_t ret = mbrtowc(topic, utf8, MQTT_TOPIC_MAX_BYTES, NULL) ;
    if (ret < 0){
      con->enabled = false ; // kill connection record
      buff[0] = MQTT_RETURN_NOT_SUPPORTED ; // Assume something unsupported happened
      queue_response(sender_address, MQTT_CONNACK, buff, 1) ;
      EPRINT("WILLTOPIC failed to convert utf8 topic string\n") ;
      return ;
    }
    uint8_t qos = 0;
    if (data[0] & FLAG_QOS1) qos = 1;
    else if (data[0] & FLAG_QOS2) qos =2 ;
    
    DPRINT("WILLTOPIC: QOS = %u, Topic = %s\n", qos, utf8) ;

    // Client registered for connection
    con->prompt_will_topic = false ; // received this now
    if (con->prompt_will_message){
      queue_response(sender_address, MQTT_WILLMSGREQ, NULL, 0) ;
    }else{
      con->connection_complete = true ; 
      buff[0] = MQTT_RETURN_ACCEPTED ;
      queue_response(sender_address, MQTT_CONNACK, buff, 1) ;
    }
  }
}

void MqttSnRF24::received_willmsgreq(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  // Only clients need to respond to this
  if (m_mqtt_type == client){
    DPRINT("WILLMSGREQ\n") ;

    if (m_willmessagesize == 0){
      // No topic set
      queue_response(sender_address, MQTT_WILLMSG, NULL, 0) ;
    }else{
      uint8_t send[MQTT_MESSAGE_MAX_BYTES];

      // Any overflow of size should have been checked so shouldn't need to check again here.
      memcpy(send, m_willmessage, m_willmessagesize) ; 
      queue_response(sender_address, MQTT_WILLMSG, send, m_willmessagesize) ;
    }
    m_client_connection.prompt_will_message = false ;
  }
}

void MqttSnRF24::received_willmsg(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (m_mqtt_type == gateway){
    char utf8[MQTT_MESSAGE_MAX_BYTES+1] ;
    wchar_t message[MQTT_MESSAGE_MAX_BYTES+1] ;

    MqttConnection *con = search_connection_address(sender_address) ;
    uint8_t buff[1] ;
    if (!con){
      EPRINT("WillMsg could not find the client connection\n") ;
      buff[0] = MQTT_RETURN_CONGESTION ; // There is no "who are you?" response so this will do
      queue_response(sender_address, MQTT_CONNACK, buff, 1) ;
      return ;
    }

    memcpy(utf8, data, len) ;
    utf8[len] = '\0' ;
    size_t ret = mbrtowc(message, utf8, MQTT_MESSAGE_MAX_BYTES, NULL) ;
    if (ret < 0){
      // Conversion failed
      EPRINT("WILLMSG: failed to convert message from UTF8\n") ;
      con->enabled = false ; // kill connection record with client
      buff[0] = MQTT_RETURN_NOT_SUPPORTED ; // Assume something unsupported happened
      queue_response(sender_address, MQTT_CONNACK, buff, 1) ;
      return ;
    }
    
    DPRINT("WILLMESSAGE: Message = %s\n", utf8) ;

    // Client sent final will message
    con->prompt_will_message = false ; // received this now
    con->connection_complete = true ;
    con->ping() ;
    buff[0] = MQTT_RETURN_ACCEPTED ;
    queue_response(sender_address, MQTT_CONNACK, buff, 1) ;
  }  
}

void MqttSnRF24::received_disconnect(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

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

  // Reset will attributes
  m_willtopic[0] = '\0' ;
  m_willtopicsize = 0 ;
  m_willtopicqos = 0 ;
  m_willmessage[0] = '\0' ;
  m_willmessagesize = 0;
  
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
  if (len > 0 && data != NULL)
    memcpy(m_queue[m_queue_head].message_data, data, len) ;
  m_queue[m_queue_head].message_len = len ;

  pthread_mutex_unlock(&m_rwlock) ;
}

bool MqttSnRF24::dispatch_queue()
{
  bool ret = true ;
  uint8_t queue_ptr = m_queue_head ;
  bool failed = false ;
  
  for ( ; ;){ // it's a do...while loop
    failed = false ;
    if (m_queue[queue_ptr].set){

      for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
	try{
	  writemqtt(m_queue[queue_ptr].address,
		    m_queue[queue_ptr].messageid,
		    m_queue[queue_ptr].message_data,
		    m_queue[queue_ptr].message_len) ;
	  break ;
	}catch(BuffMaxRetry &e){
	  // Do nothing, continue in for loop

	  // continue to try other messages but this flag indicates
	  // at least 1 message could not be sent
	  if (retry == m_max_retries){
	    failed = false ;
	    ret = false ;
	  }
	}
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

void MqttSnRF24::writemqtt(uint8_t *address,
			   uint8_t messageid,
			   uint8_t *buff, uint8_t len)
{
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

  try{
    // Write a blocking message
    write(send_buff, payload_len+m_address_len, true) ;
  }
  catch(BuffIOErr &e){
    listen_mode() ;
    throw MqttIOErr(e.what());
  }
  catch(BuffMaxRetry &e){
    listen_mode() ;
    throw ;
  }

  listen_mode() ;
}

bool MqttSnRF24::searchgw(uint8_t radius)
{
  uint8_t buff[1] ;
  buff[0] = radius ;
  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    try{
      writemqtt(m_broadcast, MQTT_SEARCHGW, buff, 1) ;
      return true ;
    }catch(BuffMaxRetry &e){
      // Do nothing, continue in for loop
    }
  }
  return false ;
}

bool MqttSnRF24::advertise(uint16_t duration)
{
  uint8_t buff[3] ;
  buff[0] = m_gwid ;
  buff[1] = duration >> 8 ; // Is this MSB first or LSB first?
  buff[2] = duration & 0x00FF;
  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    try{
      writemqtt(m_broadcast, MQTT_ADVERTISE, buff, 3) ;
      return true ;
    }catch(BuffMaxRetry &e){
      // Do nothing, continue in for loop
    }
  }
  return false ;
}

bool MqttSnRF24::ping(uint8_t gwid)
{
  // Client to Gateway ping
  size_t clientid_len = strlen(m_szclient_id) ;
  MqttGwInfo *gw = get_gateway(gwid) ;
  if (!gw) return false ; // no gateway known

  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    try{
      writemqtt(gw->address, MQTT_PINGREQ, (uint8_t *)m_szclient_id, clientid_len) ;
      return true ;
    }catch(BuffMaxRetry &e){
      // Do nothing, continue in for loop
    }
  }
  return false ;
}

bool MqttSnRF24::ping(char *szclientid)
{
  MqttConnection *con = search_connection(szclientid) ;
  if (!con) return false ; // cannot ping unknown client

  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    try{
      writemqtt(con->connect_address, MQTT_PINGREQ, NULL, 0) ;
      return true ;
    }catch(BuffMaxRetry &e){
      // Do nothing, continue in for loop
    }
  }
  return false ; // failed to send the ping
}

bool MqttSnRF24::disconnect(uint16_t sleep_duration)
{
  uint8_t buff[2] ;
  uint8_t len = 0 ;
  // Already disconnected or other issue closed the connection
  if (!m_client_connection.enabled) return false ;

  if (sleep_duration > 0){
    buff[0] = sleep_duration >> 8 ; //MSB set first
    buff[1] = sleep_duration & 0x00FF ;
    len = 2 ;
  }

  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    try{
      writemqtt(m_client_connection.connect_address, MQTT_DISCONNECT, buff, len) ;
      m_client_connection.enabled = false ;
      return true ;
    }catch(BuffMaxRetry &e){
      // Do nothing, continue in for loop
    }
  }
  return false ; // failed to send the diconnect
}

bool MqttSnRF24::connect(uint8_t gwid, bool will, bool clean, uint16_t duration)
{
  MqttGwInfo *gw ;
  uint8_t buff[MQTT_PAYLOAD_WIDTH-2] ;
  buff[0] = (will?FLAG_WILL:0) | (clean?FLAG_CLEANSESSION:0) ;
  buff[1] = MQTT_PROTOCOL ;
  buff[2] = duration >> 8 ; // MSB set first
  buff[3] = duration & 0x00FF ;

  pthread_mutex_lock(&m_rwlock) ;
  if (!(gw = get_gateway(gwid))){
    pthread_mutex_unlock(&m_rwlock) ;
    return false ;
  }
  // Copy connection details
  m_client_connection.enabled = false ; // do not enable yet
  m_client_connection.gwid = gw->gw_id ;
  memcpy(m_client_connection.connect_address, gw->address, m_address_len) ;
  pthread_mutex_unlock(&m_rwlock) ;
  
  uint8_t len = strlen(m_szclient_id) ;
  if (len > MAX_MQTT_CLIENTID) throw MqttOutOfRange("Client ID too long") ;
  memcpy(buff+4, m_szclient_id, len) ; // do not copy /0 terminator

  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    try{
      writemqtt(m_client_connection.connect_address, MQTT_CONNECT, buff, 4+len) ;
      // Record the start of the connection
      m_connect_start = time(NULL) ;
      m_client_connection.enabled = true ; // Sent request, so enable
      m_client_connection.prompt_will_topic = will ;
      m_client_connection.prompt_will_message = will ;
      m_client_connection.duration = duration ; // not used
      gw->keepalive(duration) ; // set the keep alive

      return true ;
    }catch(BuffMaxRetry &e){
      // Do nothing, continue in for loop
      nano_sleep(1,0) ;
    }
  }
  return false ;
}

bool MqttSnRF24::connect_expired()
{
  if (time(NULL) > (m_connect_start+m_connect_timeout)){
    return true ;
  }
  
  return false ;
}

 void MqttSnRF24::set_willtopic(const wchar_t *topic, uint8_t qos)
{
  if (topic == NULL){
    m_willtopic[0] = '\0' ; // Clear the topic
    m_willtopicsize = 0;
  }
  size_t len = wcslen(topic) ;
  if (len > (unsigned)(MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len))){
    throw MqttOutOfRange("topic too long for payload") ;
  }
  
  size_t ret = wcstombs(m_willtopic, topic, (MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len))) ;

  if (ret < 0) throw MqttOutOfRange("topic string conversion error") ;

  m_willtopicsize = ret ;
  m_willtopicqos = qos ;
}

void MqttSnRF24::set_willmessage(const wchar_t *message)
{
  if (message == NULL){
    m_willmessage[0] = '\0' ;
    m_willmessagesize = 0 ;
  }
  
  size_t len = wcslen(message) ;
  if (len > (unsigned)(MQTT_MESSAGE_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len)))
    throw MqttOutOfRange("message too long for payload") ;

  char *curlocale = setlocale(LC_CTYPE, NULL);
  
  if (!setlocale(LC_CTYPE, "en_GB.UTF-8")){
    throw MqttOutOfRange("cannot set UTF locale") ;
  }
    
  size_t ret = wcstombs(m_willmessage, message, (MQTT_MESSAGE_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len))) ;

  setlocale(LC_CTYPE, curlocale) ; // reset locale
  
  if (ret < 0){
    DPRINT("Throwing exception - cannot convert will message to UTF-8\n") ;
    throw MqttOutOfRange("message string conversion error") ;
  }

  m_willmessagesize = ret ;
}

MqttGwInfo* MqttSnRF24::get_available_gateway()
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].allocated && m_gwinfo[i].active){
      // return the first known gateway
      return &(m_gwinfo[i]) ;
    }
  }
  return NULL ;
}

MqttGwInfo* MqttSnRF24::get_gateway_address(uint8_t *gwaddress)
{
  uint8_t addri = 0;
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].allocated && m_gwinfo[i].active){
      
      for (addri=0; addri < m_address_len; addri++){
	if (m_gwinfo[i].address[addri] != gwaddress[addri]) break;
      }
      if (addri == m_address_len){
	// address found, return the gateway info
	return &(m_gwinfo[i]) ;
      }
    }
  }
  return NULL ;
}

MqttGwInfo* MqttSnRF24::get_gateway(uint8_t gwid)
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].allocated && m_gwinfo[i].active && m_gwinfo[i].gw_id == gwid){
      return &(m_gwinfo[i]) ;
    }
  }
  return NULL ;
}

uint8_t *MqttSnRF24::get_gateway_address()
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].allocated && m_gwinfo[i].active){
      // return the first known gateway
      return m_gwinfo[i].address ;
    }
  }
  return NULL ; // No gateways are known
}

bool MqttSnRF24::get_known_gateway(uint8_t *gwid)
{
  MqttGwInfo *gw = get_available_gateway();

  if (!gw) return false;
  *gwid = gw->gw_id ;

  return true ;
}

bool MqttSnRF24::is_gateway_valid(uint8_t gwid)
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].allocated && m_gwinfo[i].active && m_gwinfo[i].gw_id == gwid){
      // Check if it has expired
      if (m_gwinfo[i].is_active()){
	m_gwinfo[i].active = false ;
	return false ;
      }
      return true ;
    }
  }
  return false ; // No gateway with this id found
}
