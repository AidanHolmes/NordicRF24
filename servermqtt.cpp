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

#include "servermqtt.hpp"
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

ServerMqttSnRF24::ServerMqttSnRF24()
{
  m_gwid = 0 ;

  m_connection_head = NULL ;

  m_last_advertised = 0 ;
  m_advertise_interval = 1500 ;
  m_pmosquitto = NULL ;

  m_mosquitto_initialised = false ;
  m_broker_connected = false ;

}

ServerMqttSnRF24::~ServerMqttSnRF24()
{
  if (m_mosquitto_initialised){
    mosquitto_lib_cleanup() ;
  }
}

void ServerMqttSnRF24::set_advertise_interval(uint16_t t)
{
  m_advertise_interval = t ;
}

void ServerMqttSnRF24::received_publish(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (len < 6) return ; // not long enough to be a publish
  uint16_t topicid = (data[1] << 8) | data[2] ; // Assuming MSB is first
  uint16_t messageid = (data[3] << 8) | data[4] ; // Assuming MSB is first
  uint8_t qos = data[0] & FLAG_QOSN1 ;
  uint8_t topic_type = data[0] & (FLAG_DEFINED_TOPIC_ID | FLAG_SHORT_TOPIC_NAME);
  uint8_t payload[MQTT_MESSAGE_MAX_BYTES] ;
  int payload_len = len-5 ;
  int ret = 0;
  memcpy(payload, data+5, payload_len) ;

  uint8_t buff[5] ; // Response buffer
  buff[0] = data[1] ; // replicate topic id 
  buff[1] = data[2] ; // replicate topic id 
  buff[2] = data[3] ; // replicate message id
  buff[3] = data[4] ; // replicate message id

  DPRINT("PUBLISH: {Flags = %X, QoS = %u, Topic ID = %u, Mess ID = %u\n",
	 data[0], qos, topicid, messageid) ;

  if (!m_mosquitto_initialised){
    EPRINT("Gateway is not connected to the Mosquitto server to process Publish\n") ;
    buff[4] = MQTT_RETURN_CONGESTION;
    writemqtt(sender_address, MQTT_PUBACK, buff, 5) ;
    return ;
  }
  int mosqos = 0 ;
  int mid = 0 ;
  switch(qos){
  case FLAG_QOS0:
    mosqos = 0 ;
    break;
  case FLAG_QOS1:
    mosqos = 1 ;
    break;
  case FLAG_QOS2:
    mosqos = 2 ;
    break;
  case FLAG_QOSN1:
    mosqos = 0 ;
    // Cannot process normal topic IDs
    // An error shouldn't really be sent for -1 QoS messages, but
    // it cannot hurt as the client is likely to just ignore
    if (topic_type == FLAG_NORMAL_TOPIC_ID){
      EPRINT("Client sent normal topic ID %u for -1 QoS message\n", topicid) ;
      buff[4] = MQTT_RETURN_INVALID_TOPIC ;
      writemqtt(sender_address, MQTT_PUBACK, buff, 5) ;
      return ;
    }
    // Just publish and forget for QoS -1
    if (topic_type == FLAG_SHORT_TOPIC_NAME){
      char szshort[3] ;
      szshort[0] = (char)(buff[0]);
      szshort[1] = (char)(buff[1]) ;
      szshort[2] = '\0';
      ret = mosquitto_publish(m_pmosquitto,
			      &mid,
			      szshort,
			      payload_len,
			      payload, 0,
			      data[0] & FLAG_RETAIN) ;
    }else if(topic_type == FLAG_DEFINED_TOPIC_ID){
      MqttTopic *t = m_predefined_topics.get_topic(topicid);
      if (!t){
	DPRINT("Cannot find topic %u in the pre-defined list\n", topicid) ;
	buff[4] = MQTT_RETURN_INVALID_TOPIC ;
	writemqtt(sender_address, MQTT_PUBACK, buff, 5) ;
	return ;
      }
      ret = mosquitto_publish(m_pmosquitto,
			      &mid,
			      t->get_topic(),
			      payload_len,
			      payload, 0,
			      data[0] & FLAG_RETAIN) ;	
    }
    if (ret != MOSQ_ERR_SUCCESS)
      EPRINT("Mosquitto publish failed with code %d\n", ret) ;
    return ;
  }

  MqttConnection *con = search_connection_address(sender_address);
  if (!con){
    EPRINT("No registered connection for client\n") ;
    return;
  }
  // Store the publish entities for transmission to the server
  // The server can only handle one publish transaction at a time from a client
  con->set_pub_entities(topicid,
			messageid,
			topic_type,
			mosqos,
			payload_len,
			payload,
			data[0] & FLAG_RETAIN);

  if (!server_publish(con)){
    EPRINT("Could not complete the PUBLISH message with the MQTT server\n") ;
    buff[4] = MQTT_RETURN_CONGESTION;
    writemqtt(sender_address, MQTT_PUBACK, buff, 5) ;
    return ;      
  }
  con->set_activity(MqttConnection::Activity::publishing);
}

bool ServerMqttSnRF24::server_publish(MqttConnection *con)
{
  int mid = 0, ret = 0 ;
  uint8_t buff[5] ; // Response buffer
  if (!con){
    EPRINT("No registered connection for client\n") ;
    return false;
  }

  uint16_t topicid = con->get_pub_topicid() ;
  uint16_t messageid = con->get_pub_messageid() ;
  const uint8_t *sender_address = con->get_address() ;
  buff[0] = topicid >> 8 ; // replicate topic id 
  buff[1] = (topicid & 0x00FF) ; // replicate topic id 
  buff[2] = messageid >> 8 ; // replicate message id
  buff[3] = (messageid & 0x00FF) ; // replicate message id
  
  if (con->get_pub_topic_type() == FLAG_SHORT_TOPIC_NAME){
    char szshort[3] ;
    szshort[0] = (char)(buff[0]);
    szshort[1] = (char)(buff[1]) ;
    szshort[2] = '\0';
    ret = mosquitto_publish(m_pmosquitto,
			    &mid,
			    szshort,
			    con->get_pub_payload_len(),
			    con->get_pub_payload(),
			    con->get_pub_qos(),
			    con->get_pub_retain()) ;
    if (ret != MOSQ_ERR_SUCCESS){
      con->set_activity(MqttConnection::Activity::none);
      
      EPRINT("Mosquitto publish failed with code %d\n", ret);
      return false ;
    }
    con->set_mosquitto_mid(mid) ;
    
  }else if(con->get_pub_topic_type()  == FLAG_DEFINED_TOPIC_ID){
    MqttTopic *topic = m_predefined_topics.get_topic(topicid);
    if (!topic){
      EPRINT("Predefined topic id %u unrecognised\n", topicid);
      buff[4] = MQTT_RETURN_INVALID_TOPIC ;
      writemqtt(sender_address, MQTT_PUBACK, buff, 5) ;
      con->set_activity(MqttConnection::Activity::none);
      return false;
    }
    ret = mosquitto_publish(m_pmosquitto,
			    &mid,
			    topic->get_topic(),
			    con->get_pub_payload_len(),
			    con->get_pub_payload(),
			    con->get_pub_qos(),
			    con->get_pub_retain()) ;
    if (ret != MOSQ_ERR_SUCCESS){
      EPRINT("Mosquitto publish failed with code %d\n", ret);
      con->set_activity(MqttConnection::Activity::none);
      buff[4] = MQTT_RETURN_CONGESTION ;
      writemqtt(sender_address, MQTT_PUBACK, buff, 5) ;
      return false ;
    }
    con->set_mosquitto_mid(mid) ;
    
  }else{ // Normal Topic ID
    MqttTopic *topic = con->topics.get_topic(topicid) ;
    if (!topic){
      EPRINT("Client topic id %d unknown to gateway\n", topicid) ;
      buff[4] = MQTT_RETURN_INVALID_TOPIC ;
      writemqtt(sender_address, MQTT_PUBACK, buff, 5) ;
      con->set_activity(MqttConnection::Activity::none);
      return false;
    }
    ret = mosquitto_publish(m_pmosquitto,
			    &mid,
			    topic->get_topic(),
			    con->get_pub_payload_len(),
			    con->get_pub_payload(),
			    con->get_pub_qos(),
			    con->get_pub_retain()) ;
    if (ret != MOSQ_ERR_SUCCESS){
      EPRINT("Mosquitto publish failed with code %d\n", ret);
      buff[4] = MQTT_RETURN_CONGESTION ;
      writemqtt(sender_address, MQTT_PUBACK, buff, 5) ;
      con->set_activity(MqttConnection::Activity::none);
      return false ;
    }
    con->set_mosquitto_mid(mid) ;
    
  }
  return true ;
}

void ServerMqttSnRF24::received_pubrel(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (len != 2) return ; // Invalid PUBREL message length
  uint16_t messageid = (data[0] << 8) | data[1] ; // Assuming MSB is first
  DPRINT("PUBREL {messageid = %u}\n", messageid) ;
    
  MqttConnection *con = search_connection_address(sender_address);
  if (!con){
    EPRINT("No registered connection for client\n") ;
    return;
  }
  if (con->get_activity() != MqttConnection::Activity::publishing){
    EPRINT("PUBREL received for a non-publishing client\n") ;
    return ;
    }
  
  con->update_activity() ;
  con->set_activity(MqttConnection::Activity::none) ; // reset activity
  
  writemqtt(sender_address, MQTT_PUBCOMP, data, 2) ;
}

void ServerMqttSnRF24::received_subscribe(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

}

void ServerMqttSnRF24::received_suback(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

}

void ServerMqttSnRF24::received_unsubscribe(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

}

void ServerMqttSnRF24::received_unsuback(uint8_t *sender_address, uint8_t *data, uint8_t len)
{

}


void ServerMqttSnRF24::received_register(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (len < 4) return ;
  uint16_t topicid = (data[0] << 8) | data[1] ; // Assuming MSB is first
  uint16_t messageid = (data[2] << 8) | data[3] ; // Assuming MSB is first
  char sztopic[MQTT_TOPIC_MAX_BYTES+1] ;
  memcpy(sztopic, data+4, len-4) ;
  sztopic[len-4] = '\0';

  DPRINT("REGISTER: {topicid: %u, messageid: %u, topic %s}\n", topicid, messageid, sztopic) ;
  pthread_mutex_lock(&m_rwlock) ;
  MqttConnection *con = search_connection_address(sender_address) ;
  if (!con){
    DPRINT("REGISTER - Cannot find a connection for the client\n") ;
    pthread_mutex_unlock(&m_rwlock) ;
    return ;
  }
  topicid = con->topics.add_topic(sztopic, messageid) ;
  uint8_t response[5] ;
  response[0] = topicid >> 8 ; // Write topicid MSB first
  response[1] = topicid & 0x00FF ;
  response[2] = data[2] ; // Echo back the messageid received
  response[3] = data[3] ; // Echo back the messageid received
  response[4] = MQTT_RETURN_ACCEPTED ;
  writemqtt(sender_address, MQTT_REGACK, response, 5) ;
  pthread_mutex_unlock(&m_rwlock) ;
}

void ServerMqttSnRF24::received_regack(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  if (len != 5) return ;
  
  uint16_t topicid = (data[0] << 8) | data[1] ; // Assuming MSB is first
  uint16_t messageid = (data[2] << 8) | data[3] ; // Assuming MSB is first
  uint8_t returncode = data[4] ;

  DPRINT("REGACK: {topicid = %u, messageid = %u, returncode = %u}\n", topicid, messageid, returncode) ;

  MqttConnection *con = search_connection_address(sender_address) ;
  if (con->is_connected()){
    if (con->get_activity() == MqttConnection::Activity::registeringall){
      MqttTopic *t = NULL ;
      if ((t=con->topics.get_next_topic())){
	if (!register_topic(con, t)){
	  DPRINT("Failed to register topic id %u, name %s\n",
		 t->get_id(), t->get_topic()) ;
	}
      }else{
	// Finished all topics
	con->set_activity(MqttConnection::Activity::none) ;
      }
    }
  }

}

void ServerMqttSnRF24::received_pingresp(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  DPRINT("PINGRESP\n") ;
  pthread_mutex_lock(&m_rwlock) ;
  MqttConnection *con = search_connection_address(sender_address) ;
  if (con){
    // just update the last activity timestamp
    con->update_activity() ;
  }
  
  pthread_mutex_unlock(&m_rwlock) ;

}

void ServerMqttSnRF24::received_pingreq(uint8_t *sender_address, uint8_t *data, uint8_t len)
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

void ServerMqttSnRF24::received_searchgw(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  uint8_t buff[1] ;

  DPRINT("SEARCHGW: {radius = %u}\n", data[0]) ;

  // Ignore radius value. This is a gw so respond with
  // a broadcast message back.
  if (m_broker_connected){
    buff[0] = m_gwid ;
    writemqtt(m_broadcast, MQTT_GWINFO, buff, 1) ;
  }

}

MqttConnection* ServerMqttSnRF24::search_connection(const char *szclientid)
{
  MqttConnection *p = NULL ;
  for(p = m_connection_head; p != NULL; p=p->next){
    if (!p->is_disconnected()){
      if (p->client_id_match(szclientid)) break ;
    }
  }
  return p ;
}

MqttConnection* ServerMqttSnRF24::search_mosquitto_id(int mid)
{
  MqttConnection *p = NULL ;
  for(p = m_connection_head; p != NULL; p=p->next){
    if (p->is_connected() &&
	p->get_activity() != MqttConnection::Activity::none &&
	p->get_mosquitto_mid() == mid){
      break;
    }
  }
  return p ;
}

MqttConnection* ServerMqttSnRF24::search_cached_connection(const char *szclientid)
{
  MqttConnection *p = NULL ;
  for(p = m_connection_head; p != NULL; p=p->next){
    if (p->client_id_match(szclientid)) break ;
  }
  return p ;
}

MqttConnection* ServerMqttSnRF24::search_connection_address(const uint8_t *clientaddr)
{
  MqttConnection *p = NULL ;

  for(p = m_connection_head; p != NULL; p=p->next){
    if (!p->is_disconnected() && p->address_match(clientaddr)){
      return p ;
    }
  }
  return NULL ; // Cannot find
}

MqttConnection* ServerMqttSnRF24::search_cached_connection_address(const uint8_t *clientaddr)
{
  MqttConnection *p = NULL ;

  for(p = m_connection_head; p != NULL; p=p->next){
    if (p->address_match(clientaddr)){
      return p ;
    }
  }
  return NULL ; // Cannot find
}

MqttConnection* ServerMqttSnRF24::new_connection()
{
  MqttConnection *p = NULL, *prev = NULL ;

  p = new MqttConnection() ;

  // Set the head if first record
  if (m_connection_head == NULL) m_connection_head = p ;
  else{
    // Append connection to end of connection list
    for (prev = m_connection_head; prev->next; prev = prev->next){
      
    }
    p->prev = prev ;
  }

  return p ;
}

void ServerMqttSnRF24::delete_connection(const char *szclientid)
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

void ServerMqttSnRF24::received_connect(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  char szClientID[MAX_MQTT_CLIENTID+1] ;
  if (len < 5 || len > (4 + MAX_MQTT_CLIENTID)) return ; // invalid data length
  
  memcpy(szClientID, data+4, len - 4) ; // copy identifier
  szClientID[len-4] = '\0' ; // Create null terminated string

  DPRINT("CONNECT: {flags = %02X, protocol = %02X, duration = %u, client ID = %s\n", data[0], data[1], (data[2] << 8) | data[3], szClientID) ;

  if (data[1] != MQTT_PROTOCOL){
    EPRINT("Invalid protocol ID in CONNECT from client %s\n", szClientID);
    return ;
  }

  pthread_mutex_lock(&m_rwlock) ;

  MqttConnection *con = search_cached_connection(szClientID) ;
  if (!con){
    DPRINT("Cannot find an existing connection, creating a new connection for %s\n", szClientID) ;
    con = new_connection() ;
  }
  if (!con){
    EPRINT("Cannot create a new connection record for client %s\n", szClientID) ;
    pthread_mutex_unlock(&m_rwlock) ;
    return ; // something went wrong with the allocation
  }

  con->set_state(MqttConnection::State::connecting);
  con->sleep_duration = 0 ;
  con->asleep_from = 0 ;
  con->update_activity() ; // update activity from client
  con->set_client_id(szClientID) ;
  con->duration = (data[2] << 8) | data[3] ; // MSB assumed
  con->set_address(sender_address, m_address_len) ;

  // If clean flag is set then remove all topics
  if (((FLAG_CLEANSESSION & data[0]) > 0)){
    con->topics.free_topics() ;
  }
    
  // If WILL if flagged then set the flags for the message and topic
  bool will = ((FLAG_WILL & data[0]) > 0) ;

  pthread_mutex_unlock(&m_rwlock) ;

  if (will){
    // Start with will topic request
    writemqtt(sender_address, MQTT_WILLTOPICREQ, NULL, 0) ;
  }else{
    // No need for WILL setup, just CONNACK
    uint8_t buff[1] ;
    buff[0] = MQTT_RETURN_ACCEPTED ;
    complete_client_connection(con) ;
    writemqtt(sender_address, MQTT_CONNACK, buff, 1) ;
  }

}

void ServerMqttSnRF24::complete_client_connection(MqttConnection *p)
{
  if (!p) return ;
  p->update_activity() ;
  p->set_state(MqttConnection::State::connected) ;
  p->topics.iterate_first_topic();
  MqttTopic *t = NULL ;
  if ((t=p->topics.get_curr_topic())){
    // Topics are set on the connection
    p->set_activity(MqttConnection::Activity::registeringall) ;
    if (!register_topic(p, t)){
      EPRINT("Failed to send client topic id %u, name %s\n",
	     t->get_id(), t->get_topic()) ;
    }
  }else{
    p->set_activity(MqttConnection::Activity::none) ;
  }
}

void ServerMqttSnRF24::received_willtopic(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  char utf8[MQTT_TOPIC_MAX_BYTES+1] ;

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
    
  uint8_t qos = 0;
  bool retain = (data[0] & FLAG_RETAIN) ;
  if (data[0] & FLAG_QOS1) qos = 1;
  else if (data[0] & FLAG_QOS2) qos =2 ;
  
  DPRINT("WILLTOPIC: QOS = %u, Topic = %s, Retain = %s\n", qos, utf8, retain?"Yes":"No") ;

  if (!con->set_will_topic(utf8, qos, retain)){
    EPRINT("WILLTOPIC: Failed to set will topic for connection!\n") ;
  }
    
  writemqtt(sender_address, MQTT_WILLMSGREQ, NULL, 0) ;

}

void ServerMqttSnRF24::received_willmsg(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
  uint8_t utf8[MQTT_MESSAGE_MAX_BYTES] ;

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
    
  DPRINT("WILLMESSAGE: Message = %s\n", utf8) ;
  if (!con->set_will_message(utf8, len)){
    EPRINT("WILLMESSAGE: Failed to set the will message for connection!\n") ;
  }

  // Client sent final will message
  complete_client_connection(con) ;
  buff[0] = MQTT_RETURN_ACCEPTED ;
  writemqtt(sender_address, MQTT_CONNACK, buff, 1) ;
  
}

void ServerMqttSnRF24::received_disconnect(uint8_t *sender_address, uint8_t *data, uint8_t len)
{
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
  }
  con->set_state(MqttConnection::State::disconnected) ;
  con->update_activity() ;

  writemqtt(sender_address, MQTT_DISCONNECT, NULL, 0) ;
  
}

void ServerMqttSnRF24::initialise(uint8_t address_len, uint8_t *broadcast, uint8_t *address)
{

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
  mosquitto_publish_callback_set(m_pmosquitto, gateway_publish_callback) ;

  MqttSnRF24::initialise(address_len, broadcast, address);
}

void ServerMqttSnRF24::gateway_publish_callback(struct mosquitto *m,
						    void *data,
						    int mid)
{
  DPRINT("Mosquitto publish: message id = %d\n", mid) ;
  if (data == NULL) return ;
  
  ServerMqttSnRF24 *gateway = (ServerMqttSnRF24*)data ;
  MqttConnection *con = gateway->search_mosquitto_id(mid) ;
  if (!con) return ;

  uint8_t buff[5] ; // Response buffer
  uint16_t topicid = con->get_pub_topicid() ;
  uint16_t messageid = con->get_pub_messageid() ;
  buff[0] = topicid >> 8 ; // replicate topic id 
  buff[1] = topicid & 0x00FF ; // replicate topic id 
  buff[2] = messageid >> 8 ; // replicate message id
  buff[3] = messageid & 0x00FF ; // replicate message id

  switch(con->get_pub_qos()){
  case 1:
    buff[4] = MQTT_RETURN_ACCEPTED ;
    gateway->writemqtt(con->get_address(), MQTT_PUBACK, buff, 5) ;
    break ;
  case 2:
    gateway->writemqtt(con->get_address(), MQTT_PUBREC, buff+2, 2) ;
    break ;
  default:
    EPRINT("Invalid QoS %d\n", con->get_pub_qos()) ;
  }
}

void ServerMqttSnRF24::gateway_disconnect_callback(struct mosquitto *m,
						    void *data,
						    int res)
{
  DPRINT("Mosquitto disconnect: %d\n", res) ;
  ((ServerMqttSnRF24*)data)->m_broker_connected = false ;
}

void ServerMqttSnRF24::gateway_connect_callback(struct mosquitto *m,
						 void *data,
						 int res)
{
  DPRINT("Mosquitto connect: %d\n", res) ;
  // Gateway connected to the broker
  if (res == 0) ((ServerMqttSnRF24*)data)->m_broker_connected = true ;
}

void ServerMqttSnRF24::send_will(MqttConnection *con)
{
  if (!m_mosquitto_initialised) return ; // cannot process

  int mid = 0 ;
  if (strlen(con->get_will_topic()) > 0){
    int ret = mosquitto_publish(m_pmosquitto,
				&mid,
				con->get_will_topic(),
				con->get_will_message_len(),
				con->get_will_message(),
				con->get_will_qos(),
				con->get_will_retain()) ;
    if (ret != MOSQ_ERR_SUCCESS){
      EPRINT("Sending WILL: Mosquitto publish failed with code %d\n", ret);
    }
  }
}

void ServerMqttSnRF24::manage_client_connection(MqttConnection *p)
{
  if (p->lost_contact()){
    // Client is not a sleeping client and is also
    // inactive
    p->set_state(MqttConnection::State::disconnected) ;
    // Attempt to send a disconnect
    DPRINT("Disconnecting lost client: %s\n", p->get_client_id()) ;
    writemqtt(p->get_address(), MQTT_DISCONNECT, NULL, 0) ;
    send_will(p) ;
    return ;
  }else{
    // Connection should be valid
    if (p->send_another_ping()){
      DPRINT("Sending a ping to %s\n", p->get_client_id()) ;
      bool r = ping(p->get_client_id()) ;
      if (!r) DPRINT("Ping to client failed\n") ;
    }
  }

  switch(p->get_activity()){
  case MqttConnection::Activity::registeringall:
  case MqttConnection::Activity::registering:
    // Server registering a topic to a client
    if (!manage_pending_message(*p)){
      p->set_activity(MqttConnection::Activity::none);
    }
    break;
  default:
    break;
  }
}

bool ServerMqttSnRF24::manage_connections()
{

  // TO DO. Check all client connections and
  // send disconnects if client is not alive
  MqttConnection *p = NULL ;
  for(p = m_connection_head; p != NULL; p=p->next){
    switch(p->get_state()){
    case MqttConnection::State::connected:
      manage_client_connection(p) ;
      break;
    case MqttConnection::State::disconnected:
      break;
    case MqttConnection::State::asleep:
      break;
    default:
      break;
    }
  }
  
  if (m_broker_connected){
    // Send Advertise messages
    time_t now = time(NULL) ;
    if (m_last_advertised+m_advertise_interval < now){
      DPRINT("Sending Advertised\n") ;
      advertise(m_advertise_interval) ;
      m_last_advertised = now ;
    }
  }

  return dispatch_queue() ;
}

bool ServerMqttSnRF24::advertise(uint16_t duration)
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

bool ServerMqttSnRF24::register_topic(MqttConnection *con, MqttTopic *t)
{
  uint8_t buff[4 + MQTT_TOPIC_MAX_BYTES] ;
  // Gateway call to client
  if (!con->is_connected()) return false ; // not connected
  uint16_t mid = con->get_new_messageid();
  uint16_t topicid = t->get_id() ;
  buff[0] = topicid >> 8;
  buff[1] = topicid & 0x00FF ;
  buff[2] = mid >> 8;
  buff[3] = mid & 0x00FF ;
  char *sz = t->get_topic() ;
  size_t len = strlen(sz) ;
  memcpy(buff+4, sz, len) ; 
  if (writemqtt(con->get_address(), MQTT_REGISTER, buff, 4+len)){
    // Cache the message
    con->set_cache(MQTT_REGISTER, buff, 4+len) ;
    con->set_activity(MqttConnection::Activity::registering) ;
    return true ;
  }
  return false ;
}

bool ServerMqttSnRF24::create_predefined_topic(uint16_t topicid, const char *name)
{
  if (strlen(name) > (unsigned)(MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len))){
    DPRINT("Throwing exception - pre-defined topic too long\n") ;
    throw MqttOutOfRange("Pre-defined topic too long\n") ;
  }
  return m_predefined_topics.create_topic(name, topicid) ;
}

bool ServerMqttSnRF24::create_predefined_topic(uint16_t topicid, const wchar_t *name)
{
  char sztopic[MQTT_TOPIC_MAX_BYTES] ;
  
  try{
    wchar_to_utf8(name,
		  sztopic,
		  (unsigned)(MQTT_TOPIC_SAFE_BYTES + (MAX_RF24_ADDRESS_LEN - m_address_len)));
  }catch(MqttOutOfRange &e){
    DPRINT("Throwing exception - cannot convert pre-defined topic to UTF-8\n") ;
    throw ;
  }
  
  return m_predefined_topics.create_topic(sztopic, topicid) ;
}

bool ServerMqttSnRF24::ping(const char *szclientid)
{
  MqttConnection *con = search_connection(szclientid) ;
  if (!con) return false ; // cannot ping unknown client

  // Record when the ping was attempted, note that this doesn't care
  // if it worked
  con->reset_ping() ;

  if (writemqtt(con->get_address(), MQTT_PINGREQ, NULL, 0))
    return true ;

  return false ; // failed to send the ping
}

