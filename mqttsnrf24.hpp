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
#include "mqttconnection.hpp"
#include "mqtttopic.hpp"
#include "mqttparams.hpp"
#include <time.h>


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

class MqttParamErr : public RF24Exception{
public:
  MqttParamErr(const char *sz){
    do_except("Parameter Error: ", sz) ;
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


class MqttGwInfo{
public:
  MqttGwInfo(){
    reset();
  }

  void reset(){
    m_allocated=false;
    m_permanent = false;
    m_gwid=0;
    m_address_length=0;
    m_ad_time = 0 ;
    m_ad_duration = 0 ;
    m_advertising = false ;
    m_lastactivity = 0 ;
    m_active = false ;
  }    
  
  bool is_advertising(){
    return m_advertising;
  }
  
  void advertised(uint16_t t){
    m_advertising = true ;
    m_active = true ;
    m_ad_time = time(NULL) ;
    m_lastactivity = m_ad_time ; // update activity as well
    m_ad_duration = t ;
  
  }
  void set_permanent(bool perm){m_permanent = perm;}
  bool is_permanent(){return m_permanent ;}

  //  bool advertised_expired(){
  // // Add 1 min grace
  // return (m_advertising && time(NULL) > (m_ad_time + m_ad_duration + 60)) ;
  //}
  void update_activity(){
    m_lastactivity = time(NULL);
    m_active = true ;
  }
  bool is_active(){
    if (m_permanent) return true ; // always active

    if (m_advertising){
      // Add 1 min grace
      return time(NULL) < m_lastactivity + m_ad_duration + 60;
    } 

    return m_active ;
  }
  void set_active(bool active){m_active = active ;}

  bool match(uint8_t *addr){
    for (uint8_t addri=0; addri < m_address_length; addri++){
      if (m_address[addri] != addr[addri])
	return false;
    }
    return true ;
  }

  void set_address(uint8_t *address, uint8_t len){
    m_address_length = len ;
    memcpy(m_address, address, m_address_length) ;
  }
  uint8_t *get_address(){return m_address;}
  
  uint16_t advertising_duration(){return m_ad_duration;}

  bool is_allocated(){return m_allocated;}
  void set_allocated(bool alloc){m_allocated = alloc;}

  uint8_t get_gwid(){return m_gwid;}
  void set_gwid(uint8_t gwid){m_gwid = gwid;}
  
protected:
  bool m_active ;
  uint8_t m_address[MAX_RF24_ADDRESS_LEN] ;
  uint8_t m_address_length ;
  uint8_t m_gwid;
  time_t m_ad_time ;
  uint16_t m_ad_duration ;
  time_t m_lastactivity ;
  bool m_permanent ;
  bool m_allocated ;
  bool m_advertising ;
};

class MqttSnRF24 : public BufferedRF24{
public:
  MqttSnRF24();
  ~MqttSnRF24() ;

  size_t wchar_to_utf8(const wchar_t *wstr, char *outstr, const size_t maxbytes) ;
  size_t utf8_to_wchar(const char *str, wchar_t *outstr, const size_t maxbytes) ; 
  
  // Powers up and configures addresses. Goes into listen
  // mode
  void initialise(uint8_t address_len, uint8_t *broadcast, uint8_t *address) ;

  // Set the retry attributes. This affects all future connections
  void set_retry_attributes(uint16_t Tretry, uint16_t Nretry) ;

  // Powers down the radio. Call initialise to power up again
  void shutdown() ;

protected:

  // send all queued responses
  // returns false if a message cannot be sent.
  // Recommend retrying later if it fails (only applies if using acks on pipes)
  bool dispatch_queue();

  // Handle all MQTT messages with functions that can be overridden in base class
  virtual bool data_received_interrupt() ;
  virtual void received_unknown(uint8_t id, uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_advertised(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_searchgw(uint8_t *sender_address, uint8_t *data, uint8_t len){} 
  virtual void received_gwinfo(uint8_t *sender_address, uint8_t *data, uint8_t len){} 
  virtual void received_connect(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_connack(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_willtopicreq(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_willtopic(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_willmsgreq(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_willmsg(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_pingresp(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_pingreq(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_disconnect(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_register(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_regack(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_publish(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_puback(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_pubrec(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_pubrel(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_pubcomp(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_subscribe(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_suback(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_unsubscribe(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  virtual void received_unsuback(uint8_t *sender_address, uint8_t *data, uint8_t len){}
  
  // Queue the data received until dispatch is called.
  // Overwrites old queue messages without error if not dispacted quickly
  void queue_received(const uint8_t *addr,
		      uint8_t messageid,
		      const uint8_t *data,
		      uint8_t len) ;

  bool manage_pending_message(MqttConnection &con);

  // Creates header and body. Writes to address
  // Throws MqttIOErr or MqttOutOfRange exceptions
  // Returns false if connection failed max retries
  bool addrwritemqtt(const uint8_t *address,
		     uint8_t messageid,
		     const uint8_t *buff,
		     uint8_t len);

  bool writemqtt(MqttConnection *con, uint8_t messageid, const uint8_t *buff, uint8_t len);
  void listen_mode() ;
  void send_mode() ;

  uint8_t m_broadcast[MAX_RF24_ADDRESS_LEN];
  uint8_t m_address[MAX_RF24_ADDRESS_LEN];
  uint8_t m_address_len ;

  MqttMessageQueue m_queue[MAX_QUEUE] ;
  uint8_t m_queue_head ;

  uint16_t m_max_retries ;
  time_t m_Tretry ;
  uint16_t m_Nretry ;

};


#endif
