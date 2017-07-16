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

  strcpy(m_szclient_id, "RF24") ;  
  m_gwid = 0 ;
  m_mqtt_type = client ;
  m_queue_head = 0 ;
  m_max_retries = 3 ;
  m_connection_head = NULL ;

  m_willmessage[0] = '\0' ;
  m_willmessagesize = 0 ;
  m_willtopic[0] = '\0' ;
  m_willtopicsize = 0 ;
  m_willtopicqos = 0;

  m_last_advertised = 0 ;
  m_advertise_interval = 1500 ;
  m_pmosquitto = NULL ;

  m_mosquitto_initialised = false ;
  m_broker_connected = false ;

  m_connect_start = 0 ;
}

MqttSnRF24::~MqttSnRF24()
{
  if (m_mosquitto_initialised){
    mosquitto_lib_cleanup() ;
  }
}

void MqttSnRF24::set_advertise_interval(uint16_t t)
{
  m_advertise_interval = t ;
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
void MqttSnRF24::received_register(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (len < 4) return ;
  uint16_t topicid = (data[0] << 8) | data[1] ; // Assuming MSB is first
  uint16_t messageid = (data[2] << 8) | data[3] ; // Assuming MSB is first
  char sztopic[MQTT_TOPIC_MAX_BYTES+1] ;
  memcpy(sztopic, data+4, len-4) ;
  sztopic[len-4] = '\0';

  DPRINT("REGISTER {topicid: %u, messageid: %u, topic %s}\n", topicid, messageid, sztopic) ;
  if (m_mqtt_type == gateway){
    pthread_mutex_lock(&m_rwlock) ;
    MqttConnection *con = search_connection_address(sender_address) ;
    if (!con){
      DPRINT("REGISTER - Cannnot find a connection for the client\n") ;
      pthread_mutex_unlock(&m_rwlock) ;
      return ;
    }
    uint16_t topicid = con->add_topic(sztopic, messageid) ;
    uint8_t response[5] ;
    response[0] = topicid >> 8 ; // Write topicid MSB first
    response[1] = topicid & 0x00FF ;
    response[2] = data[2] ; // Echo back the messageid received
    response[3] = data[3] ; // Echo back the messageid received
    response[4] = MQTT_RETURN_ACCEPTED ;
    writemqtt(sender_address, MQTT_REGACK, response, 5) ;
    pthread_mutex_unlock(&m_rwlock) ;
  }
}

void MqttSnRF24::received_regack(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (len != 5) return ;
  
  uint16_t topicid = (data[0] << 8) | data[1] ; // Assuming MSB is first
  uint16_t messageid = (data[2] << 8) | data[3] ; // Assuming MSB is first
  uint8_t returncode = data[5] ;

  DPRINT("REGACK: {topicid = %u, messageid = %u, returncode = %u}\n", topicid, messageid, returncode) ;
  if (m_mqtt_type == client){
    
    switch(returncode){
    case MQTT_RETURN_ACCEPTED:
      DPRINT("REGACK: {return code = Accepted}\n") ;
      if (!m_client_connection.complete_topic(messageid, topicid)){
	DPRINT("Cannnot complete topic %u with messageid %u\n", topicid, messageid) ;
      }
      break ;
    case MQTT_RETURN_CONGESTION:
      DPRINT("REGACK: {return code = Congestion}\n") ;
      m_client_connection.del_topic_by_messageid(messageid) ;
      break ;
    case MQTT_RETURN_INVALID_TOPIC:
      DPRINT("REGACK: {return code = Invalid Topic}\n") ;
      m_client_connection.del_topic_by_messageid(messageid) ;
      break ;
    case MQTT_RETURN_NOT_SUPPORTED:
      DPRINT("REGACK: {return code = Not Supported}\n") ;
      m_client_connection.del_topic_by_messageid(messageid) ;
      break ;
    default:
      DPRINT("REGACK: {return code = %u}\n", returncode) ;
      m_client_connection.del_topic_by_messageid(messageid) ;
    }
  }
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
      gw->update_activity() ;
    }
    if (gw->gw_id == m_client_connection.gwid){
      // Ping received from connected gateway
      m_client_connection.update_activity() ;
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
 
  writemqtt(sender_address, MQTT_PINGRESP, NULL, 0) ;
 
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
    if (m_broker_connected){
      buff[0] = m_gwid ;
      writemqtt(m_broadcast, MQTT_GWINFO, buff, 1) ;
    }
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
	m_gwinfo[i].active = true ; // reactivate
	m_gwinfo[i].update_activity() ; // update last gw activity
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
	  m_gwinfo[i].update_activity() ; // update last gw activity
	  break ;
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
  if (prev) prev->next = p ;
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
      DPRINT("Cannot find an existing connection, creating a new connection for %s\n", szClientID) ;
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
    con->update_activity() ; // update activity from client
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
      writemqtt(sender_address, MQTT_WILLTOPICREQ, NULL, 0) ;
    }else{
      // No need for WILL setup, just CONNACK
      uint8_t buff[1] ;
      buff[0] = MQTT_RETURN_ACCEPTED ;
      con->connection_complete = true ;
      writemqtt(sender_address, MQTT_CONNACK, buff, 1) ;
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

    m_client_connection.update_activity() ;
    m_connect_retries = 0 ; // reset retry, despite server errors
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
      writemqtt(sender_address, MQTT_WILLTOPIC, NULL, 0) ;
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
      writemqtt(sender_address, MQTT_WILLTOPIC, send, m_willtopicsize+1) ;
    }
    m_client_connection.prompt_will_topic = false ;
    m_client_connection.update_activity() ;
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
      writemqtt(sender_address, MQTT_CONNACK, buff, 1) ;
      return ;
    }

    con->update_activity() ; // update known client activity
    
    memcpy(utf8, data+1, len-1) ;
    utf8[len-1] = '\0' ;
    size_t ret = mbrtowc(topic, utf8, MQTT_TOPIC_MAX_BYTES, NULL) ;
    if (ret < 0){
      con->enabled = false ; // kill connection record
      buff[0] = MQTT_RETURN_NOT_SUPPORTED ; // Assume something unsupported happened
      writemqtt(sender_address, MQTT_CONNACK, buff, 1) ;
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
      writemqtt(sender_address, MQTT_WILLMSGREQ, NULL, 0) ;
    }else{
      con->connection_complete = true ; 
      buff[0] = MQTT_RETURN_ACCEPTED ;
      writemqtt(sender_address, MQTT_CONNACK, buff, 1) ;
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
      writemqtt(sender_address, MQTT_WILLMSG, NULL, 0) ;
    }else{
      uint8_t send[MQTT_MESSAGE_MAX_BYTES];

      // Any overflow of size should have been checked so shouldn't need to check again here.
      memcpy(send, m_willmessage, m_willmessagesize) ; 
      writemqtt(sender_address, MQTT_WILLMSG, send, m_willmessagesize) ;
    }
    m_client_connection.prompt_will_message = false ;
    m_client_connection.update_activity() ;
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
      writemqtt(sender_address, MQTT_CONNACK, buff, 1) ;
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
      writemqtt(sender_address, MQTT_CONNACK, buff, 1) ;
      return ;
    }
    
    DPRINT("WILLMESSAGE: Message = %s\n", utf8) ;

    // Client sent final will message
    con->prompt_will_message = false ; // received this now
    con->connection_complete = true ;
    con->update_activity() ;
    buff[0] = MQTT_RETURN_ACCEPTED ;
    writemqtt(sender_address, MQTT_CONNACK, buff, 1) ;
  }  
}

void MqttSnRF24::received_disconnect(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (m_mqtt_type == gateway){
    // Disconnect request from client
    MqttConnection *con = search_connection_address(sender_address);
    if (!con){
      DPRINT("Disconnect received from unknown client. Ignoring\n") ;
      return ;
    }
    time_t time_now = time(NULL) ;
    if (len == 2){
      // Contains a duration
      con->sleep_duration = (data[0] << 8) | data[1] ; // MSB assumed
      DPRINT("DISCONNECT: sleeping for %u sec\n", con->sleep_duration) ;
      con->asleep_from = time_now ;
    }else{
      DPRINT("DISCONNECT\n") ;
      con->enabled = false ; // disconnected without sleep
      // Connection could be deleted...
    }
    con->disconnected = true ;
    con->connection_complete = false ;
    con->lastactivity = time_now ;

    writemqtt(sender_address, MQTT_DISCONNECT, NULL, 0) ;

  }else if (m_mqtt_type == client){
    // Disconnect request from server to client
    // probably due to an error
    DPRINT("DISCONNECT\n") ;
    m_client_connection.enabled = false;
    m_client_connection.connection_complete = false ;
    m_client_connection.free_topics() ;
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

  if (m_mqtt_type == gateway){
    char szgw[MAX_GW_TXT+1];
    if (!m_mosquitto_initialised) mosquitto_lib_init();
    m_mosquitto_initialised = true ;

    snprintf(szgw, MAX_GW_TXT, "Gateway %u", m_gwid) ;
    m_pmosquitto = mosquitto_new(szgw, false, this) ;
    if (!m_pmosquitto)
      throw MqttException("Cannot create new mosquitto instance") ;

    int ret = mosquitto_connect_async(m_pmosquitto, "localhost", 1883, 60) ;
    if (ret != MOSQ_ERR_SUCCESS)
      throw MqttException ("Cannot connect to broker") ;

    ret = mosquitto_loop_start(m_pmosquitto) ;
    if (ret != MOSQ_ERR_SUCCESS)
      throw MqttException ("Cannot start mosquitto loop") ;

    mosquitto_connect_callback_set(m_pmosquitto, gateway_connect_callback) ;
    mosquitto_disconnect_callback_set(m_pmosquitto, gateway_disconnect_callback);
  }
  
  power_up(true) ;
  nano_sleep(0,150000) ; // 150 micro second wait

  // Everything is a listener by default
  listen_mode() ;
  
}

void MqttSnRF24::gateway_disconnect_callback(struct mosquitto *m,
						    void *data,
						    int res)
{
  DPRINT("Mosquitto disconnect: %d\n", res) ;
  ((MqttSnRF24*)data)->m_broker_connected = false ;
}

void MqttSnRF24::gateway_connect_callback(struct mosquitto *m,
						 void *data,
						 int res)
{
  DPRINT("Mosquitto connect: %d\n", res) ;
  // Gateway connected to the broker
  if (res == 0) ((MqttSnRF24*)data)->m_broker_connected = true ;
}

void MqttSnRF24::shutdown()
{
  // Drop CE if transition is from listen mode
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low))
    throw MqttIOErr("GPIO error") ;

  // Power down
  power_up(false) ;

  if (m_mqtt_type == gateway){
    if (m_mosquitto_initialised && m_pmosquitto){
      mosquitto_disconnect(m_pmosquitto) ;
      int ret = mosquitto_loop_stop(m_pmosquitto,false) ;
      if (ret != MOSQ_ERR_SUCCESS)
	throw MqttException("failed to stop mosquitto loop") ;
      mosquitto_destroy(m_pmosquitto) ;
      m_pmosquitto = NULL ;
    }
  }
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

void MqttSnRF24::queue_received(uint8_t *addr,
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

bool MqttSnRF24::manage_connections()
{
  if (m_mqtt_type == client){
    // if connection is enabled and complete
    // excludes asleep connections
    if (m_client_connection.is_connected()){
      if (m_client_connection.lost_contact()){
	DPRINT("Client lost connection to gateway %u\n", m_client_connection.gwid) ;
	pthread_mutex_lock(&m_rwlock) ;
	// Close connection. Take down connection
	m_client_connection.enabled = false;
	m_client_connection.disconnected = true ;
	m_client_connection.connection_complete = false ;
	MqttGwInfo *gw = get_gateway(m_client_connection.gwid) ;
	if (gw){
	  gw->active = false ;
	}
	pthread_mutex_unlock(&m_rwlock) ;
      }else{
	// Another ping due?      
	if (m_client_connection.send_another_ping()){
	  ping(m_client_connection.gwid) ;
	}
      }
    }
    pthread_mutex_lock(&m_rwlock) ;
    // Refresh GW information
    for (unsigned int i=0; i < MAX_GATEWAYS;i++){
      if (m_gwinfo[i].allocated && m_gwinfo[i].active){
	// Check if it has expired
	if (m_client_connection.is_connected() &&
	    m_gwinfo[i].gw_id == m_client_connection.gwid){
	  // ignore the connected gateway as this has been checked
	  continue ;
	}
	if (m_gwinfo[i].advertised_expired()){
	  // Updated state to inactive as this gateway is not
	  // advertising
	  m_gwinfo[i].active = false ;
	}
      }
    }
    pthread_mutex_unlock(&m_rwlock) ;

    // TO DO - Issue search if no gateways. Currently managed by APP
    
  }else if (m_mqtt_type == gateway){
    time_t now = time(NULL) ;
    // TO DO. Check all client connections and
    // send disconnects if client is not alive
    MqttConnection *p = NULL ;
    for(p = m_connection_head; p != NULL; p=p->next){
      if (p->enabled){
	// Manage sleeping clients
	if (!p->is_asleep() && p->disconnected){
	  // Client should be out of sleep
	  // Clean up connection
	  p->enabled = false ;
	  p->connection_complete = false ;
	  // Attempt to send a disconnect
	  DPRINT("Disconnecting sleeping client: %s\n", p->szclientid);
	  writemqtt(p->connect_address, MQTT_DISCONNECT, NULL, 0) ;
	}
	else if (p->lost_contact()){
	  // Client is not a sleeping client and is also
	  // inactive
	  p->enabled = false ;
	  p->connection_complete = false ;
	  // Attempt to send a disconnect
	  DPRINT("Disconnecting lost client: %s\n", p->szclientid) ;
	  writemqtt(p->connect_address, MQTT_DISCONNECT, NULL, 0) ;
	}else if (p->is_connected()){
	  // Connection should be valid
	  if (p->send_another_ping())
	    ping(p->szclientid) ;
	}
      }
    }    

    if (m_broker_connected){
      // Send Advertise messages
      if (m_last_advertised+m_advertise_interval < now){
	DPRINT("Sending Advertised\n") ;
	advertise(m_advertise_interval) ;
	m_last_advertised = now ;
      }
    }
  }
  
  return dispatch_queue() ;
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
	break;
	//default:
	// Not expected message.
	// This is not a 1.2 MQTT message
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

bool MqttSnRF24::writemqtt(uint8_t *address,
			   uint8_t messageid,
			   uint8_t *buff, uint8_t len)
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

uint16_t MqttConnection::reg_topic(char *sztopic, uint16_t messageid)
{
  MqttTopic *p = NULL, *insert_at = NULL ;
  if (!topics){
    // No topics in connection.
    // Create the head topic. No index set
    topics = new MqttTopic(0, messageid, sztopic) ;
    return 0;
  }
  for (p = topics; p; p = p->next()){
    if (strcmp(p->get_topic(), sztopic) == 0){
      // topic exists
      DPRINT("Topic %s already exists for connection\n", sztopic) ;
      return p->get_id() ;
    }
    // Add 1 to be unique
    insert_at = p ; // Save last valid topic pointer
  }

  insert_at->link_tail(new MqttTopic(0, messageid, sztopic)) ;
  return 0 ;
}

bool MqttConnection::complete_topic(uint16_t messageid, uint16_t topicid)
{
  MqttTopic *p = NULL ;
  if (!topics) return false ; // no topics
  for (p = topics; p; p = p->next()){
    if (p->get_message_id() == messageid && !p->is_complete()){
      p->complete(topicid) ;
      return true ;
    }
  }
  // Cannot find an incomplete topic that needs completing
  return false ;
}

uint16_t MqttConnection::add_topic(char *sztopic, uint16_t messageid)
{
  MqttTopic *p = NULL, *insert_at = NULL ;
  uint16_t available_id = 0 ;
  if (!topics){
    // No topics in connection.
    // Create the head topic. Always index 1
    topics = new MqttTopic(1, messageid, sztopic) ;
    return 1;
  }
  for (p = topics; p; p = p->next()){
    if (strcmp(p->get_topic(), sztopic) == 0){
      // topic exists
      DPRINT("Topic %s already exists for connection\n", sztopic) ;
      return p->get_id() ;
    }
    // Add 1 to be unique
    if (p->get_id() > available_id) available_id = p->get_id() + 1 ;
    insert_at = p ; // Save last valid topic pointer
  }
  // If connection was to run a long time with creation and deletion of topics then
  // the ID count will overflow! Overflows in 18 hours if requested every second
  // TO DO: Better implementation of ID assignment and improved data structure
  p = new MqttTopic(available_id, messageid, sztopic) ;
  p->complete(available_id) ; // server completes the topic
  insert_at->link_tail(p);
  return available_id ;
}

bool MqttConnection::del_topic_by_messageid(uint16_t messageid)
{
  MqttTopic *p = NULL ;
  for (p=topics;p;p = p->next()){
    if (p->get_message_id() == messageid){
      if (p->is_head()){
	topics = p->next() ;
      }else{
	p->unlink() ;
      }
      delete p ;
      return true ;
    }
  }
  return false ;
}

bool MqttConnection::del_topic(uint16_t id)
{
  MqttTopic *p = NULL ;
  for (p=topics;p;p = p->next()){
    if (p->get_id() == id){
      if (p->is_head()){
	topics = p->next() ;
      }else{
	p->unlink() ;
      }
      delete p ;
      return true ;
    }
  }
  return false ;
}

void MqttConnection::free_topics()
{
  MqttTopic *p = topics,*delme = NULL ;

  while(p){
    // Cannot unlink the head
    if (!p->is_head()){
      p->unlink() ;
      delme = p;
      p = p->next() ;
      delete delme ;
    }else{
      p = p->next() ;
    }
  }
  if (topics){
    delete topics ;
    topics = NULL ;
  }
}

bool MqttSnRF24::searchgw(uint8_t radius)
{
  uint8_t buff[1] ;
  buff[0] = radius ;
  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    if (writemqtt(m_broadcast, MQTT_SEARCHGW, buff, 1))
      return true ;
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
    if (writemqtt(m_broadcast, MQTT_ADVERTISE, buff, 3))
      return true ;
  }
  return false ;
}

uint16_t MqttSnRF24::register_topic(const wchar_t *topic)
{
  char sztopic[MQTT_TOPIC_MAX_BYTES] ;
  uint8_t buff[4 + MQTT_TOPIC_MAX_BYTES] ;
  size_t len = 0 ;
  uint16_t ret = 0 ;
  
  try{
    len = wchar_to_utf8(topic, sztopic, (unsigned)(MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len)));
  }catch(MqttOutOfRange &e){
    DPRINT("Throwing exception - cannot convert REGISTER topic to UTF-8\n") ;
    throw ;
  }

  if (m_client_connection.is_connected()){
    pthread_mutex_lock(&m_rwlock) ;
    uint16_t mid = m_client_connection.get_new_messageid() ;
    // Register the topic
    if ((ret = m_client_connection.reg_topic(sztopic, mid)) > 0)
      return ret ; // already exists

    pthread_mutex_unlock(&m_rwlock) ;
    buff[0] = 0 ;
    buff[1] = 0 ; // topic ID set to zero
    buff[2] = mid >> 8 ; // MSB
    buff[3] = mid & 0x00FF;
    memcpy(buff+4, sztopic, len) ;
    
    for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
      if (writemqtt(m_client_connection.connect_address, MQTT_REGISTER, buff, 4+len))
	// add the topic to the client connection
	return true ;
    }
  }
  // Connection timed out or not connected
  return 0 ;
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

bool MqttSnRF24::ping(uint8_t gwid)
{
  // Client to Gateway ping
  size_t clientid_len = strlen(m_szclient_id) ;
  MqttGwInfo *gw = get_gateway(gwid) ;
  if (!gw) return false ; // no gateway known

  // Record when the ping was attempted, note that this doesn't care
  // if it worked
  if (m_client_connection.gwid == gwid)
    m_client_connection.last_ping = time(NULL) ;
  
  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    if (writemqtt(gw->address, MQTT_PINGREQ, (uint8_t *)m_szclient_id, clientid_len))
      return true ;
  }
  return false ;
}

bool MqttSnRF24::ping(char *szclientid)
{
  MqttConnection *con = search_connection(szclientid) ;
  if (!con) return false ; // cannot ping unknown client

  // Record when the ping was attempted, note that this doesn't care
  // if it worked
  con->last_ping = time(NULL) ;

  for(uint16_t retry = 0;retry < m_max_retries+1;retry++){
    if (writemqtt(con->connect_address, MQTT_PINGREQ, NULL, 0))
      return true ;
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
    if (writemqtt(m_client_connection.connect_address, MQTT_DISCONNECT, buff, len)){
      m_client_connection.enabled = false ;
      return true ;
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
    throw MqttConnectErr("Gateway ID unknown") ;
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

  if (writemqtt(m_client_connection.connect_address, MQTT_CONNECT, buff, 4+len)){
    // Record the start of the connection
    m_connect_start = time(NULL) ;
    m_connect_retries++ ;
    m_client_connection.enabled = true ; // Sent request, so enable
    m_client_connection.connection_complete = false ; // orchestration started
    m_client_connection.prompt_will_topic = will ;
    m_client_connection.prompt_will_message = will ;
    m_client_connection.duration = duration ; // keep alive timer for connection
    gw->keepalive(duration) ; // set the keep alive in GW register

    return true ;
  }

  return false ;
}

bool MqttSnRF24::connect_expired(uint16_t retry_time)
{
  if (time(NULL) > (m_connect_start+retry_time)){
    return true ;
  }
  
  return false ;
}

bool MqttSnRF24::connect_max_retry(bool reset)
{
  if (m_connect_retries >= m_max_retries){
    if (reset) m_connect_retries = 0 ;
    return true ;
  }
  return false ;
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

void MqttSnRF24::print_gw_table()
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    printf("Allocated %s, Active: %s, GWID: %u\n",
	   (m_gwinfo[i].allocated?"yes":"no"),
	   (m_gwinfo[i].active?"yes":"no"),
	   m_gwinfo[i].gw_id) ;
  }
}

bool MqttSnRF24::is_connected(uint8_t gwid)
{
  if (m_client_connection.is_connected() && 
      m_client_connection.gwid == gwid
      ){
    // Check if it has expired
    if (m_client_connection.lost_contact()){
      DPRINT("Lost contact with gateway %u\n", gwid) ;
      // gateway was connected but has expired.
      // update connection attributes
      pthread_mutex_lock(&m_rwlock) ;
      m_client_connection.connection_complete = false ;
      m_client_connection.enabled = false;
      MqttGwInfo *gw = get_gateway(gwid) ;
      if (gw){
	gw->active = false ;
      }
      pthread_mutex_unlock(&m_rwlock) ;

      return false ; 
    }else{
      return true ;
    }
  }
  return false ; // No gateway found with complete connection
}

bool MqttSnRF24::is_connected()
{
  // use the current gw id. 
  return is_connected(m_client_connection.gwid) ;
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

  char *curlocale = setlocale(LC_CTYPE, NULL);
  
  if (!setlocale(LC_CTYPE, "en_GB.UTF-8")){
    throw MqttOutOfRange("cannot set UTF locale") ;
  }
  
  size_t ret = wcstombs(m_willtopic, topic, (MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len))) ;

  setlocale(LC_CTYPE, curlocale) ; // reset locale

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
      DPRINT("Seaching for gateways found active GW %u\n", m_gwinfo[i].gw_id);
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

