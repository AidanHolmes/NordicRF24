//   Copyright 2019 Aidan Holmes

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

#include "clientmqtt.hpp"
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


ClientMqttSnRF24::ClientMqttSnRF24()
{
  strcpy(m_szclient_id, "RF24") ;  

  m_willmessage[0] = '\0' ;
  m_willmessagesize = 0 ;
  m_willtopic[0] = '\0' ;
  m_willtopicsize = 0 ;
  m_willtopicqos = 0;

  m_sleep_duration = 0 ;
}

ClientMqttSnRF24::~ClientMqttSnRF24()
{

}


void ClientMqttSnRF24::received_puback(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (len != 5) return ;
  
  uint16_t topicid = (data[0] << 8) | data[1] ; // Assuming MSB is first
  uint16_t messageid = (data[2] << 8) | data[3] ; // Assuming MSB is first
  uint8_t returncode = data[4] ;

  DPRINT("PUBACK: {topicid = %u, messageid = %u, returncode = %u}\n", topicid, messageid, returncode) ;

  // not for this client if the connection address is different
  if (!m_client_connection.address_match(sender_address)) return ; 
  m_client_connection.update_activity() ;
  if (m_client_connection.get_activity() == MqttConnection::Activity::publishing)
    m_client_connection.set_activity(MqttConnection::Activity::none) ;
    
  switch(returncode){
  case MQTT_RETURN_ACCEPTED:
    DPRINT("PUBACK: {return code = Accepted}\n") ;
    break ;
  case MQTT_RETURN_CONGESTION:
    DPRINT("PUBACK: {return code = Congestion}\n") ;
    break ;
  case MQTT_RETURN_INVALID_TOPIC:
    DPRINT("PUBACK: {return code = Invalid Topic}\n") ;
    break ;
  case MQTT_RETURN_NOT_SUPPORTED:
    DPRINT("PUBACK: {return code = Not Supported}\n") ;
    break ;
  default:
    DPRINT("PUBACK: {return code = %u}\n", returncode) ;
  }    

}

void ClientMqttSnRF24::received_pubrec(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (m_client_connection.get_activity() != MqttConnection::Activity::publishing)
    return ; // client is not expecting a pubrec

  if (!m_client_connection.is_connected()) return ;

  // Check that this is coming from the expected gateway
  if (!m_client_connection.address_match(sender_address)) return ; 

  // Note the server activity and reset timers
  m_client_connection.update_activity() ;

  // Is this a QoS == 2? To Do: Implement check

  // Check the length, does it match expected PUBREC length?
  if (len != 2) return ;
    
  uint16_t messageid = (data[0] << 8) | data[1] ; // Assuming MSB is first
  DPRINT("PUBREC {messageid = %u}\n", messageid) ;
  writemqtt(&m_client_connection, MQTT_PUBREL, data, 2) ;
}

void ClientMqttSnRF24::received_pubcomp(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (len != 2) return ; // Invalid PUBCOMP message length
  uint16_t messageid = (data[0] << 8) | data[1] ; // Assuming MSB is first
  DPRINT("PUBCOMP {messageid = %u}\n", messageid) ;

  if (m_client_connection.get_activity() != MqttConnection::Activity::publishing)
    return ; // client is not expecting a pubcomp

  // not for this client if the connection address is different
  if (!m_client_connection.address_match(sender_address)) return ; 

  // Note the server activity and reset timers
  m_client_connection.update_activity() ;

  // Reset the connection activity
  m_client_connection.set_activity(MqttConnection::Activity::none) ;

}

void ClientMqttSnRF24::received_subscribe(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

}

void ClientMqttSnRF24::received_suback(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

}

void ClientMqttSnRF24::received_unsubscribe(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

}

void ClientMqttSnRF24::received_unsuback(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

}


void ClientMqttSnRF24::received_regack(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (len != 5) return ;
  
  uint16_t topicid = (data[0] << 8) | data[1] ; // Assuming MSB is first
  uint16_t messageid = (data[2] << 8) | data[3] ; // Assuming MSB is first
  uint8_t returncode = data[4] ;

  DPRINT("REGACK: {topicid = %u, messageid = %u, returncode = %u}\n", topicid, messageid, returncode) ;

  // not for this client if the connection address is different
  if (!m_client_connection.address_match(sender_address)) return ; 
  m_client_connection.update_activity() ;
  m_client_connection.set_activity(MqttConnection::Activity::none) ;
  switch(returncode){
  case MQTT_RETURN_ACCEPTED:
    DPRINT("REGACK: {return code = Accepted}\n") ;
    if (!m_client_connection.topics.complete_topic(messageid, topicid)){
      DPRINT("Cannot complete topic %u with messageid %u\n", topicid, messageid) ;
    }
    break ;
  case MQTT_RETURN_CONGESTION:
    DPRINT("REGACK: {return code = Congestion}\n") ;
    m_client_connection.topics.del_topic_by_messageid(messageid) ;
    break ;
  case MQTT_RETURN_INVALID_TOPIC:
    DPRINT("REGACK: {return code = Invalid Topic}\n") ;
    m_client_connection.topics.del_topic_by_messageid(messageid) ;
    break ;
  case MQTT_RETURN_NOT_SUPPORTED:
    DPRINT("REGACK: {return code = Not Supported}\n") ;
    m_client_connection.topics.del_topic_by_messageid(messageid) ;
    break ;
  default:
    DPRINT("REGACK: {return code = %u}\n", returncode) ;
    m_client_connection.topics.del_topic_by_messageid(messageid) ;
  }

}

void ClientMqttSnRF24::received_pingresp(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  DPRINT("PINGRESP\n") ;
  pthread_mutex_lock(&m_rwlock) ;

  MqttGwInfo *gw = get_gateway_address(sender_address);
  if (gw){
    gw->update_activity() ;

    if (gw->get_gwid() == m_client_connection.get_gwid()){
      // Ping received from connected gateway
      m_client_connection.update_activity() ;
    }
  }
  pthread_mutex_unlock(&m_rwlock) ;

}

void ClientMqttSnRF24::received_pingreq(uint8_t *sender_address, uint8_t *data, uint8_t len)
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
 
  addrwritemqtt(sender_address, MQTT_PINGRESP, NULL, 0) ;
 
}

void ClientMqttSnRF24::received_advertised(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  uint16_t duration = (data[1] << 8) | data[2] ; // Assuming MSB is first
  DPRINT("ADVERTISED: {gw = %u, duration = %u}\n", data[0], duration) ;

  pthread_mutex_lock(&m_rwlock) ;

  // Call update gateway. This returns false if gateway is not known
  if (!update_gateway(sender_address, data[0], duration)){

    // New gateway
    bool ret = add_gateway(sender_address, data[0], duration);
    if (!ret){
      DPRINT("Cannot add gateway %u\n", data[0]) ;
    }
  }

  // If client is connected to this gateway then update client connection activity
  if (m_client_connection.is_connected() &&
      m_client_connection.get_gwid() == data[0]){
    m_client_connection.update_activity() ;
  }

  pthread_mutex_unlock(&m_rwlock) ;
  
}

bool ClientMqttSnRF24::add_gateway(uint8_t *gateway_address, uint8_t gwid, uint16_t ad_duration, bool perm)
{
  for (uint8_t i=0; i < MAX_GATEWAYS; i++){
    if (!m_gwinfo[i].is_allocated() || !m_gwinfo[i].is_active()){
      m_gwinfo[i].reset() ; // Clear gateway
      m_gwinfo[i].set_address(gateway_address, m_address_len) ;
      m_gwinfo[i].set_allocated(true) ;
      m_gwinfo[i].update_activity() ;
      m_gwinfo[i].set_gwid(gwid) ;
      m_gwinfo[i].set_permanent(perm) ;

      if (ad_duration > 0){
	m_gwinfo[i].advertised(ad_duration) ; 
      }
      return true ;
    }
  }
  return false ;
}

bool ClientMqttSnRF24::update_gateway(uint8_t *gateway_address, uint8_t gwid, uint16_t ad_duration)
{
  for (uint8_t i=0; i < MAX_GATEWAYS; i++){
    if (m_gwinfo[i].is_allocated() && m_gwinfo[i].get_gwid() == gwid){
      m_gwinfo[i].set_address(gateway_address, m_address_len) ;
      m_gwinfo[i].update_activity() ;

      // Only set new duration if not zero. Retain original advertised state
      if (ad_duration > 0){
	m_gwinfo[i].advertised(ad_duration) ; 
      }

      return true ;
    }
  }
  return false;
}

bool ClientMqttSnRF24::del_gateway(uint8_t gwid)
{
  for (uint8_t i=0; i < MAX_GATEWAYS; i++){
    if (m_gwinfo[i].get_gwid() == gwid){
      m_gwinfo[i].reset() ; // Clear all attributes. Can be reused
      return true ;
    }
  }
  return false;
}


void ClientMqttSnRF24::received_gwinfo(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  DPRINT("GWINFO: {gw = %u, address = ", data[0]) ;
  for (uint8_t j=0; j < len - 1; j++) DPRINT("%X", data[j+1]) ;
  DPRINT("}\n") ;
  
  bool gw_updated = false  ;

  pthread_mutex_lock(&m_rwlock) ;

  // Reset activity if searching was requested
  if (m_client_connection.get_activity() == MqttConnection::Activity::searching){
    m_client_connection.set_activity(MqttConnection::Activity::none) ;
  }
  
  if (len == m_address_len+1) // Was the address populated in GWINFO?
    gw_updated = update_gateway(data+1, data[0], 0);
  else
    gw_updated = update_gateway(sender_address, data[0], 0);
  
  if (!gw_updated){
    // Insert new gateway. Overwrite old or expired gateways
    if (len == m_address_len+1){ // Was the address populated in GWINFO?
      add_gateway(data+1, data[0], 0);
    }else{ // No address, but the sender address can be used
      add_gateway(sender_address, data[0], 0);
    }
  }
  pthread_mutex_unlock(&m_rwlock) ;

  // It's possible that the gateway cannot be saved if there's already a full list
  // of gateways.
  // Why is the client requesting the info though??? Only needs one GW so not an error
  
}

void ClientMqttSnRF24::received_connack(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

  pthread_mutex_lock(&m_rwlock) ;

  // Was this client connecting?
  if (m_client_connection.get_state() != MqttConnection::State::connecting){
    pthread_mutex_unlock(&m_rwlock) ;
    return ; // not enabled and expected
  }
  
  m_client_connection.update_activity() ;
  // TO DO - confirm that the gateway sending this matches the address expected
  // not a major fault but is worth a check

  if (m_client_connection.get_activity() == MqttConnection::Activity::willtopic){
    EPRINT("Connection complete, but will topic or message not processed\n") ;
  }

  // TO DO: Needs to handle the errors properly and inform client of error
  // so correct behaviour can follow.
  switch(data[0]){
  case MQTT_RETURN_ACCEPTED:
    DPRINT("CONNACK: {return code = Accepted}\n") ;
    m_client_connection.set_state(MqttConnection::State::connected);
    break ;
  case MQTT_RETURN_CONGESTION:
    DPRINT("CONNACK: {return code = Congestion}\n") ;
    m_client_connection.set_state(MqttConnection::State::disconnected); // Cannot connect
    break ;
  case MQTT_RETURN_INVALID_TOPIC:
    DPRINT("CONNACK: {return code = Invalid Topic}\n") ;
    // Can this still count as a connection?
    m_client_connection.set_state(MqttConnection::State::disconnected); // Don't allow?
    break ;
  case MQTT_RETURN_NOT_SUPPORTED:
    DPRINT("CONNACK: {return code = Not Supported}\n") ;
    m_client_connection.set_state(MqttConnection::State::disconnected); // Cannot connect
    break ;
  default:
    DPRINT("CONNACK: {return code = %u}\n", data[0]) ;
    // ? Are we connected ?
    m_client_connection.set_state(MqttConnection::State::disconnected); // Cannot connect
  }
    
  m_client_connection.set_activity(MqttConnection::Activity::none) ;
    
  pthread_mutex_unlock(&m_rwlock) ;

}

void ClientMqttSnRF24::received_willtopicreq(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

  // Check that this is coming from the expected gateway
  if (!m_client_connection.address_match(sender_address)) return ; 

  // Only clients need to respond to this
  DPRINT("WILLTOPICREQ\n") ;
  if (m_client_connection.get_state() != MqttConnection::State::connecting) return ; // Unexpected
  if (m_willtopicsize == 0){
    // No topic set
    writemqtt(&m_client_connection, MQTT_WILLTOPIC, NULL, 0) ;
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
    writemqtt(&m_client_connection, MQTT_WILLTOPIC, send, m_willtopicsize+1) ;
  }
  m_client_connection.set_activity(MqttConnection::Activity::willtopic) ;
  m_client_connection.update_activity() ;
  
}

void ClientMqttSnRF24::received_willmsgreq(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  // Check that this is coming from the expected gateway
  if (!m_client_connection.address_match(sender_address)) return ; 

  // Only clients need to respond to this
  DPRINT("WILLMSGREQ\n") ;
  if (m_client_connection.get_state() != MqttConnection::State::connecting) return ; // Unexpected

  if (m_willmessagesize == 0){
    // No topic set
    writemqtt(&m_client_connection, MQTT_WILLMSG, NULL, 0) ;
  }else{
    uint8_t send[MQTT_MESSAGE_MAX_BYTES];

    // Any overflow of size should have been checked so shouldn't need to check again here.
    memcpy(send, m_willmessage, m_willmessagesize) ; 
    writemqtt(&m_client_connection, MQTT_WILLMSG, send, m_willmessagesize) ;
  }
  m_client_connection.set_activity(MqttConnection::Activity::willmessage) ;
  m_client_connection.update_activity() ;
  
}

void ClientMqttSnRF24::received_disconnect(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  // Disconnect request from server to client
  // probably due to an error
  DPRINT("DISCONNECT\n") ;
  if (m_sleep_duration)
    m_client_connection.set_state(MqttConnection::State::asleep) ;
  else
    m_client_connection.set_state(MqttConnection::State::disconnected) ;
  m_client_connection.topics.free_topics() ; // client always forgets topics
  
}

void ClientMqttSnRF24::set_client_id(const char *szclientid)
{
  strncpy(m_szclient_id, szclientid, MAX_MQTT_CLIENTID) ;
}

const char* ClientMqttSnRF24::get_client_id()
{
  return m_szclient_id ;
}

void ClientMqttSnRF24::initialise(uint8_t address_len, uint8_t *broadcast, uint8_t *address)
{
  // Reset will attributes
  m_willtopic[0] = '\0' ;
  m_willtopicsize = 0 ;
  m_willtopicqos = 0 ;
  m_willmessage[0] = '\0' ;
  m_willmessagesize = 0;
  
  MqttSnRF24::initialise(address_len, broadcast, address) ;
}

bool ClientMqttSnRF24::manage_gw_connection()
{
  // Has connection been lost or does the connection need a keep-alive
  // ping to maintain connection?
  
  if (m_client_connection.lost_contact()){
    DPRINT("Client lost connection to gateway %u\n", m_client_connection.get_gwid()) ;
    // Close connection. Take down connection
    m_client_connection.set_state(MqttConnection::State::disconnected) ;
    m_client_connection.set_activity(MqttConnection::Activity::none) ;
    m_client_connection.topics.free_topics() ; // clear all topics
    // Disable the gateway in the client register
    MqttGwInfo *gw = get_gateway(m_client_connection.get_gwid()) ;
    if (gw){
      gw->set_active(false);
    }
    return false;
    
  }else{
    // Another ping due?      
    if (m_client_connection.send_another_ping()){
      DPRINT("Sending a ping to %u\n", m_client_connection.get_gwid()) ;
      bool r = ping(m_client_connection.get_gwid()) ;
      if (!r) DPRINT("Ping to gw failed\n") ;
    }
  }
  return true ;
}

bool ClientMqttSnRF24::manage_connections()
{
  switch (m_client_connection.get_state()){
  case MqttConnection::State::connected:
    // Manage connection to gateway. Ensure gateway is still there and connection should be open
    // Function returns false if gateway lost - no need to manage connection
    if (manage_gw_connection()){
      if (!m_client_connection.get_activity() == MqttConnection::Activity::none){
	// If connected client is doing anything then manage the connection
	if (!manage_pending_message(m_client_connection)){
	  m_client_connection.set_activity(MqttConnection::Activity::none) ;
	}
      }
    }

    break;
  case MqttConnection::State::connecting:
    if (!manage_pending_message(m_client_connection)){
      m_client_connection.set_state(MqttConnection::State::disconnected) ;
    }
    break;
  case MqttConnection::State::disconnecting:
    if (!manage_pending_message(m_client_connection)){
      m_client_connection.set_state(MqttConnection::State::disconnected) ;
    }
    break ;
  case MqttConnection::State::disconnected:
    // Retry searches if no response.
    if (m_client_connection.get_activity() == MqttConnection::Activity::searching){
      if (!manage_pending_message(m_client_connection)){
	// No search response, stop searching
	m_client_connection.set_activity(MqttConnection::Activity::none);
      }
    }
      
    break ;
  case MqttConnection::State::asleep:
    break ;
  default:
    break ; // unhandled connection state
  }
  
  // TO DO - Issue search if no gateways. Currently managed by APP
  
  return dispatch_queue() ;
}

bool ClientMqttSnRF24::searchgw(uint8_t radius)
{
  uint8_t buff[1] ;
  buff[0] = radius ;

  if (addrwritemqtt(m_broadcast, MQTT_SEARCHGW, buff, 1)){
    // Cache the message if disconnected. This changes the connection to use
    // the broadcast address for resending. 
    if (m_client_connection.get_state() == MqttConnection::State::disconnected){
      m_client_connection.set_address(m_broadcast, m_address_len);
      m_client_connection.set_cache(MQTT_SEARCHGW, buff, 1) ;
      m_client_connection.set_activity(MqttConnection::Activity::searching) ;
    }
    return true ;
  }

  return false ;
}

uint16_t ClientMqttSnRF24::register_topic(const wchar_t *topic)
{
  char sztopic[MQTT_TOPIC_MAX_BYTES] ;
   
  try{
    wchar_to_utf8(topic, sztopic, (unsigned)(MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len)));
  }catch(MqttOutOfRange &e){
    DPRINT("Throwing exception - cannot convert REGISTER topic to UTF-8\n") ;
    throw ;
  }

  return register_topic(sztopic) ;
}

uint16_t ClientMqttSnRF24::register_topic(const char *topic)
{
  uint16_t ret = 0, len = 0 ;
  uint8_t buff[4 + MQTT_TOPIC_MAX_BYTES] ;

  if (m_client_connection.is_connected()){
    pthread_mutex_lock(&m_rwlock) ;
    uint16_t mid = m_client_connection.get_new_messageid() ;
    // Register the topic. Return value is zero if topic is new
    ret = m_client_connection.topics.reg_topic(topic, mid) ;
    if(ret > 0){
      DPRINT("reg_topic returned an existing topic ID %u\n", ret) ;
      return ret ; // already exists
    }
    DPRINT("reg_topic has registered a new topic %u\n", mid) ;

    len = strlen(topic) + 1;
    pthread_mutex_unlock(&m_rwlock) ;
    buff[0] = 0 ;
    buff[1] = 0 ; // topic ID set to zero
    buff[2] = mid >> 8 ; // MSB
    buff[3] = mid & 0x00FF;
    memcpy(buff+4, topic, len) ;
    
    if (writemqtt(&m_client_connection, MQTT_REGISTER, buff, 4+len)){
      // Cache the message
      m_client_connection.set_activity(MqttConnection::Activity::registering) ;
      return 0;
    }
  }
  // Connection timed out or not connected
  return 0 ;
}

bool ClientMqttSnRF24::ping(uint8_t gwid)
{
  // Client to Gateway ping
  size_t clientid_len = strlen(m_szclient_id) ;
  MqttGwInfo *gw = get_gateway(gwid) ;
  if (!gw) return false ; // no gateway known

  // Record when the ping was attempted, note that this doesn't care
  // if it worked
  if (m_client_connection.get_gwid() == gwid)
    m_client_connection.reset_ping() ;
  
  if (addrwritemqtt(gw->get_address(), MQTT_PINGREQ, (uint8_t *)m_szclient_id, clientid_len))
    return true ;

  return false ;
}

bool ClientMqttSnRF24::disconnect(uint16_t sleep_duration)
{
  uint8_t buff[2] ;
  uint8_t len = 0 ;
  // Already disconnected or other issue closed the connection
  if (m_client_connection.is_disconnected()) return false ;
  if (m_client_connection.is_asleep()) return false ;

  if (sleep_duration > 0){
    buff[0] = sleep_duration >> 8 ; //MSB set first
    buff[1] = sleep_duration & 0x00FF ;
    len = 2 ;
  }
  m_sleep_duration = sleep_duration ; // store the duration

  if (writemqtt(&m_client_connection, MQTT_DISCONNECT, buff, len)){
    m_client_connection.set_state(MqttConnection::State::disconnecting) ;
    return true ;
  }
  
  return false ; // failed to send the diconnect
}

bool ClientMqttSnRF24::publish_noqos(uint8_t gwid, const char* sztopic, const uint8_t *payload, uint8_t payload_len, bool retain)
{
  uint16_t topicid = 0;
  // This will send Qos -1 messages with a short topic
  if (strlen(sztopic) != 2) return false ; // must be 2 bytes
  topicid = (sztopic[0] << 8) | sztopic[1] ;
  return publish_noqos(gwid,
		       topicid,
		       FLAG_SHORT_TOPIC_NAME,
		       payload, payload_len, retain) ;
}

bool ClientMqttSnRF24::publish_noqos(uint8_t gwid, uint16_t topicid, uint8_t topictype, const uint8_t *payload, uint8_t payload_len, bool retain)
{
  uint8_t buff[MQTT_PAYLOAD_WIDTH-2] ;
  buff[0] = (retain?FLAG_RETAIN:0) | FLAG_QOSN1 | topictype ;
  buff[1] = topicid >> 8 ;
  buff[2] = topicid & 0x00FF ;
  buff[3] = 0 ;
  buff[4] = 0 ;
  uint8_t len = payload_len + 5 ;
  if (len > (MQTT_PAYLOAD_WIDTH-7-m_address_len)){
    EPRINT("Payload of %u bytes is too long for publish\n", payload_len) ;
    throw MqttOutOfRange("PUBLISH payload too long") ;
    return false ;
  }
  memcpy(buff+5,payload, payload_len);

  DPRINT("NOQOS - topicid %u, topictype %u, flags %X\n",topicid,topictype,buff[0]) ;

  MqttGwInfo *gw = get_gateway(gwid) ;
  if (!gw) return false ;

  if(topictype == FLAG_SHORT_TOPIC_NAME ||
     topictype == FLAG_DEFINED_TOPIC_ID){
    if (!addrwritemqtt(gw->get_address(), MQTT_PUBLISH, buff, len)){
      EPRINT("Failed to send QoS -1 message\n") ;
      return false ;
    }
  }else if (topictype == FLAG_NORMAL_TOPIC_ID){
    throw MqttParamErr("QoS -1 cannot support normal topic IDs") ;
  }else{
    throw MqttParamErr("QoS -1 unknown topic type") ;
  }
  return true ;
}

bool ClientMqttSnRF24::publish(uint8_t qos, const char *sztopic, const uint8_t *payload, uint8_t payload_len, bool retain)
{
  uint16_t topicid = 0;
  // This will send messages with a short topic
  if (strlen(sztopic) != 2) return false ; // must be 2 bytes
  topicid = (sztopic[0] << 8) | sztopic[1] ;
  return publish(qos,
		 topicid,
		 FLAG_SHORT_TOPIC_NAME,
		 payload, payload_len, retain) ;
}

bool ClientMqttSnRF24::publish(uint8_t qos, uint16_t topicid, uint16_t topictype, const uint8_t *payload, uint8_t payload_len, bool retain)
{
  uint8_t buff[MQTT_PAYLOAD_WIDTH-2] ;
  
  // This publish call will not handle -1 QoS messages
  if (!m_client_connection.is_connected()) return false ;

  if (qos > 2) return false ; // Invalid QoS
  
  buff[0] = (retain?FLAG_RETAIN:0) |
    (qos==0?FLAG_QOS0:0) |
    (qos==1?FLAG_QOS1:0) |
    (qos==2?FLAG_QOS2:0) |
    topictype;
  buff[1] = topicid >> 8 ;
  buff[2] = topicid & 0x00FF ;
  uint16_t mid = m_client_connection.get_new_messageid() ;
  buff[3] = mid << 8 ;
  buff[4] = mid & 0x00FF ;
  uint8_t len = payload_len + 5 ;
  if (len > (MQTT_PAYLOAD_WIDTH-7-m_address_len)){
    EPRINT("Payload of %u bytes is too long for publish\n", payload_len) ;
    throw MqttOutOfRange("PUBLISH payload too long") ;
    return false ;
  }
  memcpy(buff+5,payload, payload_len);
  
  if (writemqtt(&m_client_connection, MQTT_PUBLISH, buff, len)){
    if (qos > 0) m_client_connection.set_activity(MqttConnection::Activity::publishing);
    // keep a copy for retries. Set the DUP flag for retries
    buff[0] |= FLAG_DUP;
    return true ;
  }

  return false ;
}

bool ClientMqttSnRF24::connect(uint8_t gwid, bool will, bool clean, uint16_t keepalive)
{
  MqttGwInfo *gw ;
  uint8_t buff[MQTT_PAYLOAD_WIDTH-2] ;
  buff[0] = (will?FLAG_WILL:0) | (clean?FLAG_CLEANSESSION:0) ;
  buff[1] = MQTT_PROTOCOL ;
  buff[2] = keepalive >> 8 ; // MSB set first
  buff[3] = keepalive & 0x00FF ;

  pthread_mutex_lock(&m_rwlock) ;
  if (!(gw = get_gateway(gwid))){
    pthread_mutex_unlock(&m_rwlock) ;
    throw MqttConnectErr("Gateway ID unknown") ;
    return false ;
  }
  // Copy connection details
  m_client_connection.set_state(MqttConnection::State::disconnected) ;
  m_client_connection.topics.free_topics() ; // clear all topics
  m_client_connection.set_gwid(gwid) ;
  m_client_connection.set_address(gw->get_address(), m_address_len) ;
  pthread_mutex_unlock(&m_rwlock) ;
  
  uint8_t len = strlen(m_szclient_id) ;
  if (len > MAX_MQTT_CLIENTID) throw MqttOutOfRange("Client ID too long") ;
  memcpy(buff+4, m_szclient_id, len) ; // do not copy /0 terminator

#if DEBUG
  char addrdbg[(MAX_RF24_ADDRESS_LEN*2)+1];
  addr_to_straddr(gw->get_address(), addrdbg, m_address_len) ;
  DPRINT("Connecting to gateway %d at address %s\n", gwid, addrdbg) ;
#endif
  if (writemqtt(&m_client_connection, MQTT_CONNECT, buff, 4+len)){
    // Record the start of the connection
    m_client_connection.set_state(MqttConnection::State::connecting) ;
    if(will)
      m_client_connection.set_activity(MqttConnection::Activity::willtopic) ;
    else
      m_client_connection.set_activity(MqttConnection::Activity::none) ;
    // Hold keep alive info on gateway list and in the connection parameters
    m_client_connection.duration = keepalive ; // keep alive timer for connection
    return true ;
  }

  return false ;
}

bool ClientMqttSnRF24::is_gateway_valid(uint8_t gwid)
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].is_allocated() && m_gwinfo[i].is_active() && m_gwinfo[i].get_gwid() == gwid){
      // Check if it has expired
      return m_gwinfo[i].is_active();
    }
  }
  return false ; // No gateway with this id found
}

void ClientMqttSnRF24::print_gw_table()
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].is_allocated()){
      printf("GWID: %u, Permanent: %s, Active: %s, Advertising: %s, Advertising Duration: %u\n", 
	     m_gwinfo[i].get_gwid(),
	     m_gwinfo[i].is_permanent()?"yes":"no",
	     m_gwinfo[i].is_active()?"yes":"no",
	     m_gwinfo[i].is_advertising()?"yes":"no",
	     m_gwinfo[i].advertising_duration());
    }
  }
}

bool ClientMqttSnRF24::is_disconnected()
{
  return m_client_connection.is_disconnected() ;
}

bool ClientMqttSnRF24::is_connected(uint8_t gwid)
{
  return m_client_connection.is_connected() && 
    (m_client_connection.get_gwid() == gwid) ;
}

bool ClientMqttSnRF24::is_connected()
{
  return m_client_connection.is_connected() ;
}

void ClientMqttSnRF24::set_willtopic(const char *topic, uint8_t qos)
{
  if (topic == NULL){
    m_willtopic[0] = '\0' ; // Clear the topic
    m_willtopicsize = 0;
  }else{
    strncpy(m_willtopic, topic, (MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len)));
    m_willtopic[MQTT_TOPIC_MAX_BYTES] = '\0';
    m_willtopicsize = strlen(m_willtopic)+1 ;
  }
  m_willtopicqos = qos ;
}

void ClientMqttSnRF24::set_willtopic(const wchar_t *topic, uint8_t qos)
{
  if (topic == NULL){
    m_willtopic[0] = '\0' ; // Clear the topic
    m_willtopicsize = 0;
    m_willtopicqos = qos ;
  }else{
    size_t len = wcslen(topic) ;
    if (len > (unsigned)(MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len))){
      throw MqttOutOfRange("topic too long for payload") ;
    }

    // TO DO: use wchar_to_utf8
    size_t ret = wchar_to_utf8(topic, m_willtopic, (MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len)));
    
    m_willtopicsize = ret ;
    m_willtopicqos = qos ;
  }
}

void ClientMqttSnRF24::set_willmessage(const wchar_t *message)
{
  char utf8str[MQTT_MESSAGE_MAX_BYTES+1] ;
  size_t maxlen = (MQTT_MESSAGE_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len)) ;

  size_t len = wchar_to_utf8(message, utf8str, maxlen) ;
  set_willmessage((uint8_t*)utf8str, len) ;
}

void ClientMqttSnRF24::set_willmessage(const uint8_t *message, uint8_t len)
{
  if (message == NULL || len == 0){
    m_willmessagesize = 0 ;
    return ;
  }
  
  if (len > (unsigned)(MQTT_MESSAGE_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len)))
    throw MqttOutOfRange("WILL message too long for payload") ;

  memcpy(m_willmessage, message, len) ;

  m_willmessagesize = len ;
}

MqttGwInfo* ClientMqttSnRF24::get_available_gateway()
{
  if (m_client_connection.is_connected()){
    uint8_t gwid = m_client_connection.get_gwid();
    return get_gateway(gwid) ;
  }

  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].is_allocated() && m_gwinfo[i].is_active()){
      // return the first known gateway
      DPRINT("Seaching for gateways found active GW %u\n", m_gwinfo[i].get_gwid());
      return &(m_gwinfo[i]) ;
    }
  }
  return NULL ;
}

MqttGwInfo* ClientMqttSnRF24::get_gateway_address(uint8_t *gwaddress)
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].is_allocated() && m_gwinfo[i].is_active()){
      if (m_gwinfo[i].match(gwaddress)){
	return &(m_gwinfo[i]) ;
      }
    }
  }
  return NULL ;
}

MqttGwInfo* ClientMqttSnRF24::get_gateway(uint8_t gwid)
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].is_allocated() && m_gwinfo[i].is_active() && m_gwinfo[i].get_gwid() == gwid){
      return &(m_gwinfo[i]) ;
    }
  }
  return NULL ;
}

// TO DO: Maybe have a similar function but parameter of gwid
uint8_t *ClientMqttSnRF24::get_gateway_address()
{
  for (unsigned int i=0; i < MAX_GATEWAYS;i++){
    if (m_gwinfo[i].is_allocated() && m_gwinfo[i].is_active()){
      // return the first known gateway
      return m_gwinfo[i].get_address() ;
    }
  }
  return NULL ; // No gateways are known
}

bool ClientMqttSnRF24::get_known_gateway(uint8_t *gwid)
{
  if (m_client_connection.is_connected()){
    *gwid = m_client_connection.get_gwid();
    return true ;
  }

  MqttGwInfo *gw = get_available_gateway();

  if (!gw) return false;
  *gwid = gw->get_gwid() ;

  return true ;
}


