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

#ifndef __MQTTSN_RF24
#define __MQTTSN_RF24

#include "bufferedrf24.hpp"
#include "rpinrf24.hpp"
#include <time.h>

#define MAX_QUEUE 50
#define MAX_GATEWAYS 5
#define MAX_MQTT_CLIENTID 21
#define MQTT_PAYLOAD_WIDTH 32
#define MQTT_PROTOCOL 0x01
#define MQTT_TOPIC_SAFE_BYTES 24
#define MQTT_TOPIC_MAX_BYTES 26
#define MQTT_MESSAGE_SAFE_BYTES 25
#define MQTT_MESSAGE_MAX_BYTES 27

class MqttException : public RF24Exception{
public:
  MqttException(const char *sz){
    do_except("MQTT exception: ", sz) ;
  }
};

class MqttOutOfRange : public RF24Exception{
public:
  MqttOutOfRange(const char *sz){
    do_except("Out of range: ", sz) ;
  }
};

class MqttIOErr : public RF24Exception{
public:
  MqttIOErr(const char *sz){
    do_except("IO Error: ", sz) ;
  }
};

class MqttMessageQueue{
public:
  MqttMessageQueue(){
    set = false ;
    address_len = 0 ;
    message_len = 0 ;
    messageid = 0;
  }
  bool set ;
  uint8_t messageid ;
  uint8_t address[MAX_RF24_ADDRESS_LEN];
  uint8_t address_len ;
  uint8_t message_data[MQTT_PAYLOAD_WIDTH] ;
  uint8_t message_len ;
};

class MqttConnection{
public:
  MqttConnection(){
    next = NULL ;
    prev = NULL ;
    prompt_will_topic = false ;
    prompt_will_message = false ;
    duration = 0 ;
    gwid = 0 ;
    enabled = false ;
    connection_complete = false ;
  }
  MqttConnection *next; // linked list of connections (gw only)
  MqttConnection *prev ; // linked list of connections (gw only)
  bool enabled ; // This record is valid
  bool connection_complete ; // protocol complete
  char szclientid[MAX_MQTT_CLIENTID+1] ; // client id for gw
  uint8_t gwid ; // gw id for client connections
  uint8_t connect_address[MAX_RF24_ADDRESS_LEN] ; // client or gw address
  bool prompt_will_topic ; // waiting for will topic
  bool prompt_will_message ; // waiting for will message
  uint16_t duration ; // duration
};

class MqttGwInfo{
public:
  MqttGwInfo(){
    allocated=false;
    active=false;
    gw_id=0;
    address_length=0;
    m_ad_time = 0 ;
    m_ad_duration = 0 ;
  }
  uint8_t address[MAX_RF24_ADDRESS_LEN] ;
  uint8_t address_length ;
  uint8_t gw_id ;
  bool active ;
  bool allocated ;
  void advertised(uint16_t t){
    m_ad_time = time(NULL) ;
    m_ad_duration = t + 60 ; // add 1 min grace
  }
  bool advertised_expired(){
    return (time(NULL) > (m_ad_time + m_ad_duration)) ;
  }
protected:
  time_t m_ad_time ;
  uint16_t m_ad_duration ;
  
};

class MqttSnRF24 : public BufferedRF24{
public:
  MqttSnRF24();
  ~MqttSnRF24() ;

  enum enType {client, gateway, forwarder} ;

  //////////////////////////////////////
  // Setup
  
  // Powers up and configures addresses. Goes into listen
  // mode
  void initialise(enType type, uint8_t address_len, uint8_t *broadcast, uint8_t *address) ;

  // Powers down the radio. Call initialise to power up again
  void shutdown() ;

  ///////////////////////////////////////
  // Settings
  
  // Set this for gateways. Defaults to zero
  void set_gateway_id(const uint8_t gwid){m_gwid = gwid;}
  uint8_t get_gateway_id(){return m_gwid;}

  // Set a client ID. This can be up to MAX_MQTT_CLIENTID.
  // String must be null terminated.
  // Will throw MqttOutOfRange if string is over MAX_MQTT_CLIENTID
  void set_client_id(const char *szclientid) ;
  const char* get_client_id() ; // Returns pointer to class objects identfier

  //////////////////////////////////////
  // MQTT messages
  
  // Send an advertise message to broadcast address
  // duration, in sec, until next advertise is broadcast
  // Returns false if connection couldn't be made due to timeout (only applies if using acks on pipes)
  bool advertise(uint16_t duration) ;

  // Send a request for a gateway. Gateway responds with gwinfo data
  // Returns false if connection couldn't be made due to timeout (only applies if using acks on pipes)
  bool searchgw(uint8_t radius) ;

  // Throws MqttOutOfRange
  // Returns false if connection couldn't be made due to timeout or
  // no known gateway to connect to. Timeouts only detected if using ACKS on pipes
  bool connect(uint8_t gwid, bool will, bool clean, uint16_t duration) ; 
  bool connect_expired() ; // has the connect request expired?
  void set_willtopic(const wchar_t *topic, uint8_t qos) ;
  void set_willmessage(const wchar_t *message) ;
  
  // send all queued responses
  // returns false if a message cannot be sent.
  // Recommend retrying later if it fails (only applies if using acks on pipes)
  bool dispatch_queue();

  // Returns true if known gateway exists. Sets gwid to known gateway handle
  // returns false if no known gateways exist
  bool get_known_gateway(uint8_t *gwid) ;

  // Check gateway handle. If gateway has been lost then this will return false,
  // otherwise it will be true
  bool is_gateway_valid(uint8_t gwid);

protected:
  virtual bool data_received_interrupt() ;
  void received_advertised(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_searchgw(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_gwinfo(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_connect(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_connack(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_willtopicreq(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_willtopic(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_willmsgreq(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_willmsg(uint8_t *sender_address, uint8_t *data, uint8_t len) ;

  MqttGwInfo* get_gateway(uint8_t gwid);
  MqttGwInfo* get_available_gateway();
  uint8_t *get_gateway_address();
  MqttConnection* search_connection(char *szclientid);
  MqttConnection* search_connection_address(uint8_t *clientaddr);
  MqttConnection* new_connection();
  void delete_connection(char *szclientid);
  
  // Queue the data responses until dispatch is called.
  // Overwrites old queue messages without error if not dispacted quickly
  // Use as alternative to writemqtt for responses from interrupt calls
  void queue_response(uint8_t *addr,
		      uint8_t messageid,
		      uint8_t *data,
		      uint8_t len) ;
    
  // Creates header and body. Writes to address
  // Throws BuffMaxRetry, MqttIOErr or MqttOutOfRange exceptions
  void writemqtt(uint8_t *address, uint8_t messageid, uint8_t *buff, uint8_t len);
  void listen_mode() ;
  void send_mode() ;
  
  uint8_t m_broadcast[MAX_RF24_ADDRESS_LEN];
  uint8_t m_address[MAX_RF24_ADDRESS_LEN];
  char m_szclient_id[MAX_MQTT_CLIENTID+1] ; // Client ID
  uint8_t m_gwid;
  uint8_t m_address_len ;
  enType m_mqtt_type ;
  MqttGwInfo m_gwinfo[MAX_GATEWAYS] ;
  MqttMessageQueue m_queue[MAX_QUEUE] ;
  uint8_t m_queue_head ;

  uint16_t m_max_retries ;

  MqttConnection *m_connection_head ;

  // Connection attributes for a client
  time_t m_connect_start ;
  uint16_t m_connect_timeout ;
  MqttConnection m_client_connection ;
  char m_willtopic[MQTT_TOPIC_MAX_BYTES+1] ;
  char m_willmessage[MQTT_MESSAGE_MAX_BYTES+1] ;
  size_t m_willtopicsize ;
  size_t m_willmessagesize ;
  uint8_t m_willtopicqos ;
};


#endif
