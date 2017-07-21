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
#include <mosquitto.h>

#define MAX_QUEUE 50
#define MAX_GATEWAYS 5
#define MAX_MQTT_CLIENTID 21
#define MQTT_PAYLOAD_WIDTH 32
#define MQTT_PROTOCOL 0x01
#define MQTT_TOPIC_SAFE_BYTES 21
#define MQTT_TOPIC_MAX_BYTES 23
#define MQTT_MESSAGE_SAFE_BYTES 25
#define MQTT_MESSAGE_MAX_BYTES 27

class MqttException : public RF24Exception{
public:
  MqttException(const char *sz){
    do_except("MQTT exception: ", sz) ;
  }
};

class MqttConnectErr : public RF24Exception{
public:
  MqttConnectErr(const char *sz){
    do_except("Connection error: ", sz) ;
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

class MqttTopic{
public:
  MqttTopic(){reset();}
  MqttTopic(uint16_t topic, uint16_t mid, char *sztopic){reset();set_topic(topic, mid, sztopic);}
  // Clients will set the messageid, servers need to set from REGISTER
  // requested value.
  void set_topic(uint16_t topic, uint16_t messageid, char *sztopic){
    m_topicid = topic ;
    strncpy(m_sztopic, sztopic, MQTT_TOPIC_MAX_BYTES) ;
    m_messageid = messageid ;
    m_registered_at = time(NULL) ;
  }
  void reset(){
    m_next = NULL;
    m_prev = NULL;
    m_sztopic[0] = '\0' ;
    m_topicid = 0;
    m_messageid = 0;
    m_acknowledged = false ;
    m_registered_at = 0 ;
    m_timeout = 5 ;
  }
  bool registration_expired(){return (m_registered_at + m_timeout) < time(NULL);}
  bool is_head(){return !m_prev;}
  uint16_t get_id(){return m_topicid;}
  uint16_t get_message_id(){return m_messageid;}
  char *get_topic(){return m_sztopic;}
  MqttTopic *next(){return m_next;}
  MqttTopic *prev(){return m_prev;}
  bool is_complete(){return m_acknowledged;}
  void complete(uint16_t tid){m_acknowledged = true ;m_topicid=tid;}
  void unlink(){if (!is_head())m_prev->m_next = m_next;}
  void link_head(MqttTopic *topic){if (m_prev)m_prev->m_next = topic;m_prev = topic;} // adds topic ahead
  void link_tail(MqttTopic *topic){if (m_next)m_next->m_prev = topic;m_next = topic;} // adds topic after
protected:
  MqttTopic *m_next ;
  MqttTopic *m_prev ;
  char m_sztopic[MQTT_TOPIC_MAX_BYTES+1];
  uint16_t m_topicid ;
  uint16_t m_messageid ; // used by clients
  bool m_acknowledged ;
  time_t m_registered_at ;
  uint16_t m_timeout ;
};

class MqttConnection{
public:
  enum State{
    disconnected, connected, 
    asleep, connecting, disconnecting
  };
  enum Activity{
    none, willtopic, willmessage,registering, publishing, subscribing
  };
  
  MqttConnection(){
    topics = NULL ;
    next = NULL ;
    prev = NULL ;
    //prompt_will_topic = false ;
    //prompt_will_message = false ;
    duration = 0 ; // keep alive timer
    m_gwid = 0 ;
    //enabled = false ; // client not heard from becomes disabled
    //disconnected = false ;
    //connection_complete = false ;
    m_lastactivity = 0 ;
    sleep_duration = 0 ;
    asleep_from = 0 ;
    m_last_ping =0;
    m_messageid = 0;
    m_address_len = 0 ;
    m_state = State::disconnected ;
    m_activity = Activity::none ;
    m_szclientid[0] = '\0' ;
  }
  // Client connection will register that it is creating a topic
  // Needs to be formally added with a complete_topic call
  uint16_t reg_topic(char *sztopic, uint16_t messageid) ;
  // Server adds the topic. a call to complete_topic is not required when a
  // server adds a topic.
  // Will return a new Topic ID or if the topic already exists, the existing Topic ID
  uint16_t add_topic(char *sztopic, uint16_t messageid=0) ;
  // Client call to complete topic and update topicid. Returns false if not found
  bool complete_topic(uint16_t messageid, uint16_t topicid) ;
  bool del_topic(uint16_t id) ;
  bool del_topic_by_messageid(uint16_t messageid) ;
  void free_topics() ; // delete all topics and free mem
  void update_activity(){ // received activity from client or server
    m_lastactivity = time(NULL) ;
    reset_ping() ;
  }
  bool send_another_ping(){
    return ((m_last_ping + (duration)) < time(NULL)) ;
  }
  void reset_ping(){m_last_ping = time(NULL) ;}
  bool is_asleep(){
    return m_state == State::asleep ;
  }

  void set_client_id(const char *sz){strncpy(m_szclientid, sz, MAX_MQTT_CLIENTID);}
  bool client_id_match(const char *sz){return (strcmp(sz, m_szclientid) == 0);}
  const char* get_client_id(){return m_szclientid;}
  State get_state(){return m_state;}
  void set_state(State s){m_state = s;}
  Activity get_activity(){return m_activity;}
  void set_activity(Activity a){m_activity = a;}

  bool is_connected(){
    return m_state == State::connected ;
  }
  bool is_disconnected(){
    return m_state == State::disconnected ;
  }

  // Not thread protected!
  uint16_t get_new_messageid(){if(m_messageid == 0xFFFF)m_messageid=0;return ++m_messageid;}
  
  bool lost_contact(){
    // Give 5 retries before failing. This mutliplies the time assuming that
    // all pings will be sent timely
    return ((m_lastactivity + (duration * 5)) < time(NULL)) ;
  }
  bool address_match(const uint8_t *addr){
    for (uint8_t a=0; a < m_address_len; a++){
      if (m_connect_address[a] != addr[a]) return false ;
    }
    return true ;
  }
  void set_address(const uint8_t *addr, uint8_t len){
    memcpy(m_connect_address, addr, len) ;
    m_address_len = len ;
  }

  const uint8_t* get_address(){return m_connect_address;}

  void set_gwid(uint8_t gwid){m_gwid = gwid;}
  uint8_t get_gwid(){return m_gwid;}
  MqttTopic *topics ;
  MqttConnection *next; // linked list of connections (gw only)
  MqttConnection *prev ; // linked list of connections (gw only)
  //bool enabled ; // This record is valid
  //bool disconnected ; // client issued a disconnect if true
  //bool connection_complete ; // protocol complete
  //bool prompt_will_topic ; // waiting for will topic
  //bool prompt_will_message ; // waiting for will message
  uint16_t duration ; // keep alive duration
  time_t asleep_from ;
  uint16_t sleep_duration ;
protected:
  uint8_t m_gwid ; // gw id for client connections
  time_t m_last_ping ;
  time_t m_lastactivity ; // when did we last hear from the client (sec)
  uint16_t m_messageid ;
  char m_szclientid[MAX_MQTT_CLIENTID+1] ; // client id for gw
  uint8_t m_connect_address[MAX_RF24_ADDRESS_LEN] ; // client or gw address
  uint8_t m_address_len ;
  State m_state ;
  Activity m_activity ;
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
    m_lastactivity = 0 ;
    m_keepalive = 0 ;
  }
  uint8_t address[MAX_RF24_ADDRESS_LEN] ;
  uint8_t address_length ;
  uint8_t gw_id ;
  bool active ;
  bool allocated ;
  void advertised(uint16_t t){
    m_ad_time = time(NULL) ;
    m_lastactivity = m_ad_time ; // update activity as well
    m_ad_duration = t + 60 ; // add 1 min grace
  }
  void keepalive(uint16_t t){
    m_keepalive = t ;
  }
  bool advertised_expired(){
    return (time(NULL) > (m_ad_time + m_ad_duration)) ;
  }
  void update_activity(){
    m_lastactivity = time(NULL);
  }
  bool is_active(){
    return (time(NULL) > m_lastactivity + m_keepalive);
  }
protected:
  time_t m_ad_time ;
  uint16_t m_ad_duration ;
  time_t m_lastactivity ;
  uint16_t m_keepalive ;
};

class MqttSnRF24 : public BufferedRF24{
public:
  MqttSnRF24();
  ~MqttSnRF24() ;

  enum enType {client, gateway, forwarder} ;
  size_t wchar_to_utf8(const wchar_t *wstr, char *outstr, const size_t maxbytes) ;
  
  //////////////////////////////////////
  // Setup
  
  // Powers up and configures addresses. Goes into listen
  // mode
  void initialise(enType type, uint8_t address_len, uint8_t *broadcast, uint8_t *address) ;

  // Gateways can define how often the advertise is sent
  void set_advertise_interval(uint16_t t) ;
  
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
  bool connect_expired(uint16_t retry_time) ; // has the connect request expired?
  bool connect_max_retry(bool reset); // Has exceeded retry counter?
  bool is_connected(uint8_t gwid) ; // are we connected to this gw?
  bool is_connected() ; // are we connected to any gateway?
  void set_willtopic(const wchar_t *topic, uint8_t qos) ;
  void set_willmessage(const wchar_t *message) ;

  // Disconnect. Optional sleep duration can be set. If zero then
  // no sleep timer will be set
  // Returns false if disconnect cannot be sent (ACK enabled)
  bool disconnect(uint16_t sleep_duration = 0) ;

  // Ping for use by a gateway to check a client is alive
  // Returns false if client is not connected or ping fails (with ACK)
  bool ping(const char *szclientid) ;
  // Ping for use by a client to check a gateway is alive
  // Returns false if gateway is unknown or transmit failed (with ACK)
  bool ping(uint8_t gw);

  // Register a topic with the server. Returns the topic id to use
  // when publishing messages. Returns 0 if the register fails.
  uint16_t register_topic(const wchar_t *topic) ;
  
  // Handles connections to gateways or to clients. Dispatches queued messages
  // Will return false if a queued message cannot be dispatched.
  bool manage_connections() ;

  // Returns true if known gateway exists. Sets gwid to known gateway handle
  // returns false if no known gateways exist
  bool get_known_gateway(uint8_t *gwid) ;

  // Check gateway handle. If gateway has been lost then this will return false,
  // otherwise it will be true
  bool is_gateway_valid(uint8_t gwid);

  void print_gw_table() ;

protected:

  static void gateway_disconnect_callback(struct mosquitto *m,
					  void *data,
					  int res);

  static void gateway_connect_callback(struct mosquitto *m,
				       void *data,
				       int res);

  // send all queued responses
  // returns false if a message cannot be sent.
  // Recommend retrying later if it fails (only applies if using acks on pipes)
  bool dispatch_queue();

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
  void received_pingresp(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_pingreq(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_disconnect(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_register(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  void received_regack(uint8_t *sender_address, uint8_t *data, uint8_t len) ;

  MqttGwInfo* get_gateway(uint8_t gwid);
  MqttGwInfo* get_gateway_address(uint8_t *gwaddress) ;
  MqttGwInfo* get_available_gateway();
  uint8_t *get_gateway_address();
  // Searches for a client connection using the client ID
  // Only returns connected clients 
  MqttConnection* search_connection(const char *szclientid);
  // Searches all cached connections connected and disconnected
  // Returns NULL if no matches found
  MqttConnection* search_cached_connection(const char *szclientid);
  // Searches for connections by address. Only returns connected
  // clients. Returns NULL if no connected clients can be found
  MqttConnection* search_connection_address(const uint8_t *clientaddr);
  // Searches for connections by address regardless of connection state.
  // Returns NULL if no connections can be found
  MqttConnection* search_cached_connection_address(const uint8_t *clientaddr);
  // Creates a new connection and appends to end of client connection list
  MqttConnection* new_connection();
  // Removes a connection from the connection cache.
  void delete_connection(const char *szclientid);
  
  // Queue the data received until dispatch is called.
  // Overwrites old queue messages without error if not dispacted quickly
  void queue_received(const uint8_t *addr,
		      uint8_t messageid,
		      const uint8_t *data,
		      uint8_t len) ;
    
  // Creates header and body. Writes to address
  // Throws MqttIOErr or MqttOutOfRange exceptions
  // Returns false if connection failed max retries
  bool writemqtt(const uint8_t *address, uint8_t messageid, const uint8_t *buff, uint8_t len);
  void listen_mode() ;
  void send_mode() ;
  
  uint8_t m_broadcast[MAX_RF24_ADDRESS_LEN];
  uint8_t m_address[MAX_RF24_ADDRESS_LEN];
  char m_szclient_id[MAX_MQTT_CLIENTID+1] ; // Client ID
  uint8_t m_address_len ;
  enType m_mqtt_type ;
  MqttGwInfo m_gwinfo[MAX_GATEWAYS] ;
  MqttMessageQueue m_queue[MAX_QUEUE] ;
  uint8_t m_queue_head ;

  uint16_t m_max_retries ;

  MqttConnection *m_connection_head ;

  // Connection attributes for a client
  time_t m_connect_start ;
  uint16_t m_connect_retries ;
  MqttConnection m_client_connection ;
  char m_willtopic[MQTT_TOPIC_MAX_BYTES+1] ;
  char m_willmessage[MQTT_MESSAGE_MAX_BYTES+1] ;
  size_t m_willtopicsize ;
  size_t m_willmessagesize ;
  uint8_t m_willtopicqos ;

  // Gateway connection attributes
  struct mosquitto *m_pmosquitto ;
  time_t m_last_advertised ;
  uint16_t m_advertise_interval ;
  bool m_mosquitto_initialised ;
  uint8_t m_gwid;
  bool m_broker_connected ;
};


#endif
