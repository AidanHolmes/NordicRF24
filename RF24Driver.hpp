#include "PacketDriver.hpp"
#include "rpinrf24.hpp"

#ifndef __MQTTSN_RF24_DRIVER
#define __MQTTSN_RF24_DRIVER

// Driver macro redefinitions
#ifdef PACKET_DRIVER_MAX_ADDRESS_LEN
#undef PACKET_DRIVER_MAX_ADDRESS_LEN
#endif
#define PACKET_DRIVER_MAX_ADDRESS_LEN MAX_RF24_ADDRESS_LEN

#ifdef PACKET_DRIVER_MAX_PAYLOAD
#undef PACKET_DRIVER_MAX_PAYLOAD
#endif
#define PACKET_DRIVER_MAX_PAYLOAD (MAX_RXTXBUF - MIN_RF24_ADDRESS_LEN)

class RF24Driver : public IPacketDriver, public NordicRF24{
public:
  RF24Driver();

  // Set device address, broadcast address and required address length
  // Once length has been set the it cannot be changed
  bool initialise(uint8_t *device, uint8_t *broadcast, uint8_t length);
  bool shutdown();
  virtual bool data_received_interrupt();
  virtual bool max_retry_interrupt() ;
  virtual bool data_sent_interrupt() ;

  bool send(const uint8_t *receiver, uint8_t *data, uint8_t len) ;
  bool set_payload_width(uint8_t width);
  uint8_t get_payload_width();
  bool send_mode();
  bool listen_mode();
  uint8_t *get_broadcast(){return m_broadcast ;}
  uint8_t *get_address(){return m_device;}
protected:
  uint8_t m_device[MAX_RF24_ADDRESS_LEN] ;
  uint8_t m_broadcast[MAX_RF24_ADDRESS_LEN] ;
  uint8_t m_payload_width ;
  volatile enum Status{waiting, delivered, ioerr, failed} m_sendstatus ;
private:
  
};

#endif
