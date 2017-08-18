#ifndef __MQTT_TOPIC_H
#define __MQTT_TOPIC_H

#include <stdint.h>
#include "mqttparams.hpp"
#include <time.h>

class MqttTopic{
public:
  MqttTopic(){reset();}
  MqttTopic(uint16_t topic, uint16_t mid, char *sztopic){reset();set_topic(topic, mid, sztopic);}

  void set_topic(uint16_t topic, uint16_t messageid, char *sztopic) ;
  void reset() ;
  bool registration_expired(){return (m_registered_at + m_timeout) < time(NULL);}
  bool is_head(){return !m_prev;}
  uint16_t get_id(){return m_topicid;}
  uint16_t get_message_id(){return m_messageid;}
  char *get_topic(){return m_sztopic;}
  MqttTopic *next(){return m_next;}
  MqttTopic *prev(){return m_prev;}
  bool is_complete(){return m_acknowledged;}
  void complete(uint16_t tid){m_acknowledged = true ;m_topicid=tid;}
  void set_predefined(bool predefined){m_predefined = predefined;}
  bool is_predefined(){return m_predefined;}
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
  bool m_predefined;
};

class MqttTopicCollection{
public:
  MqttTopicCollection() ;
  ~MqttTopicCollection() ;
  
  // Client connection will register that it is creating a topic
  // Needs to be formally added with a complete_topic call
  uint16_t reg_topic(char *sztopic, uint16_t messageid) ;
  // Server adds the topic. a call to complete_topic is not required when a
  // server adds a topic.
  // Will return a new Topic ID or if the topic already exists, the existing Topic ID
  uint16_t add_topic(char *sztopic, uint16_t messageid=0) ;

  // Add a topic to the collection with a specific topic ID
  // returns false if the topic ID has already been allocated
  bool create_topic(char *sztopic, uint16_t topicid) ;
  
  // Client call to complete topic and update topicid. Returns false if not found
  bool complete_topic(uint16_t messageid, uint16_t topicid) ;
  bool del_topic(uint16_t id) ;
  bool del_topic_by_messageid(uint16_t messageid) ;
  void free_topics() ; // delete all topics and free mem
  void iterate_first_topic() ;
  MqttTopic* get_next_topic() ;
  MqttTopic* get_curr_topic() ;
  MqttTopic* get_topic(uint16_t topicid) ;
protected:
  MqttTopic *topics ;
  MqttTopic *m_topic_iterator ;
  
};


#endif
