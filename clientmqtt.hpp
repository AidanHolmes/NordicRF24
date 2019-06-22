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

#ifndef __CLIENT_MQTTSN_RF24
#define __CLIENT_MQTTSN_RF24

#include "mqttsnrf24.hpp"
#include "rpinrf24.hpp"
#include "mqttconnection.hpp"
#include "mqtttopic.hpp"
#include "mqttparams.hpp"
#include <time.h>


class ClientMqttSnRF24 : public MqttSnRF24{
public:
  ClientMqttSnRF24();
  ~ClientMqttSnRF24() ;

  //////////////////////////////////////
  // Setup
  
  // Powers up and configures addresses. Goes into listen
  // mode
  void initialise(uint8_t address_len, uint8_t *broadcast, uint8_t *address) ;

  ///////////////////////////////////////
  // Settings
  
  // Set a client ID. This can be up to MAX_MQTT_CLIENTID.
  // String must be null terminated.
  // Will throw MqttOutOfRange if string is over MAX_MQTT_CLIENTID
  void set_client_id(const char *szclientid) ;
  const char* get_client_id() ; // Returns pointer to client identfier

  //////////////////////////////////////
  // MQTT messages
  
  // Send a request for a gateway. Gateway responds with gwinfo data
  // Returns false if connection couldn't be made due to timeout (only applies if using acks on pipes)
  bool searchgw(uint8_t radius) ;

  // Throws MqttOutOfRange
  // Returns false if connection couldn't be made due to timeout or
  // no known gateway to connect to. Timeouts only detected if using ACKS on pipes
  bool connect(uint8_t gwid, bool will, bool clean, uint16_t duration) ; 
  //bool connect_expired(uint16_t retry_time) ; // has the connect request expired?
  //bool connect_max_retry(bool reset); // Has exceeded retry counter?
  bool is_connected(uint8_t gwid) ; // are we connected to this gw?
  bool is_connected() ; // are we connected to any gateway?
  bool is_disconnected() ; // are we disconnected to any gateway?
  void set_willtopic(const wchar_t *topic, uint8_t qos) ;
  void set_willtopic(const char *topic, uint8_t qos) ;
  void set_willmessage(const uint8_t *message, uint8_t len) ;
  void set_willmessage(const wchar_t *message) ;
  // TO DO: Protocol also allows update of will messages during connection to server
  
  // Disconnect. Optional sleep duration can be set. If zero then
  // no sleep timer will be set
  // Returns false if disconnect cannot be sent (ACK enabled)
  bool disconnect(uint16_t sleep_duration = 0) ;

  // Publishes a -1 QoS message which do not require a connection.
  // This call publishes short topics (2 bytes).
  // Requires a known gateway (requires manual specification of GW)
  bool publish_noqos(uint8_t gwid,
		     const char* sztopic,
		     const uint8_t *payload,
		     uint8_t payload_len,
		     bool retain) ;

  // Publish a -1 QoS message. Topic ID must relate to a pre-defined
  // topic.
  bool publish_noqos(uint8_t gwid,
		     uint16_t topicid,
		     uint8_t topictype,
		     const uint8_t *payload,
		     uint8_t payload_len,
		     bool retain);
  
  // Publish for connected clients. Doesn't support -1 QoS
  // Only pulishes normal registered topics
  // Returns false if message cannot be sent or client not connected
  bool publish(uint16_t topicid,
	       uint8_t qos,
	       bool retain,
	       uint8_t *payload,
	       uint8_t payload_len);
  
  // Ping for use by a client to check a gateway is alive
  // Returns false if gateway is unknown or transmit failed (with ACK)
  bool ping(uint8_t gw);

  // Client call
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

  // Connection state handling for clients
  bool manage_gw_connection() ;

  virtual void received_advertised(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_gwinfo(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_connack(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_willtopicreq(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_willmsgreq(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_pingresp(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_pingreq(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_disconnect(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_regack(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_puback(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_pubrec(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_pubcomp(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_subscribe(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_suback(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_unsubscribe(uint8_t *sender_address, uint8_t *data, uint8_t len) ;
  virtual void received_unsuback(uint8_t *sender_address, uint8_t *data, uint8_t len) ;

  MqttGwInfo* get_gateway(uint8_t gwid);
  MqttGwInfo* get_gateway_address(uint8_t *gwaddress) ;
  MqttGwInfo* get_available_gateway();
  uint8_t *get_gateway_address();
  bool add_gateway(uint8_t *gateway_address, uint8_t gwid, uint16_t ad_duration);
  bool update_gateway(uint8_t *gateway_address, uint8_t gwid, uint16_t ad_duration);

  char m_szclient_id[MAX_MQTT_CLIENTID+1] ; // Client ID
  MqttGwInfo m_gwinfo[MAX_GATEWAYS] ;

  // Connection attributes for a client
  MqttConnection m_client_connection ;
  char m_willtopic[MQTT_TOPIC_MAX_BYTES+1] ;
  uint8_t m_willmessage[MQTT_MESSAGE_MAX_BYTES] ;
  size_t m_willtopicsize ;
  size_t m_willmessagesize ;
  uint8_t m_willtopicqos ;
  uint16_t m_sleep_duration ;
};


#endif
