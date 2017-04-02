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

#ifndef __NORDIC_RF24
#define __NORDIC_RF24

#include "hardware.hpp"
#include <vector>
#define MAX_RXTXBUF 33
#define RF24_PIPES 6
#define RF24_250KBPS 1
#define RF24_1MBPS 2
#define RF24_2MBPS 3
#define RF24_NEG18DBM 0
#define RF24_NEG12DBM 1
#define RF24_NEG6DBM 2
#define RF24_0DBM 3
#define RF24_PIPE_EMPTY 0x07

#define AR_CONFIG if(m_auto_update)read_config()
#define AW_CONFIG if(m_auto_update)write_config()
#define AR_FIFO if(m_auto_update)read_fifo_status()
#define AR_FEAT if(m_auto_update)read_feature()
#define AW_FEAT if(m_auto_update)write_feature()

class NordicRF24{
public:
  NordicRF24();
  ~NordicRF24() ;
  bool reset_rf24() ;
  bool set_spi(IHardwareSPI *pSPI);

  void auto_update(bool update){m_auto_update = update;}

  bool set_gpio(IHardwareGPIO *pGPIO, uint8_t ce, uint8_t irq) ;

  bool send(uint8_t *buffer, uint8_t len);
  
  // Reading data functions
  uint8_t get_rx_data_size(uint8_t pipe) ;
  bool read_payload(uint8_t *buffer, uint8_t len) ;
  bool write_payload(uint8_t *buffer, uint8_t len) ;
  
  // Configuration settings
  bool read_config();
  bool write_config();
  // GET calls
  bool use_interrupt_data_ready(){AR_CONFIG;return m_mask_rx_dr;}
  bool use_interrupt_data_sent(){AR_CONFIG;return m_mask_tx_ds;}
  bool use_interrupt_max_retry(){AR_CONFIG;return m_mask_max_rt;}
  bool is_crc_enabled(){AR_CONFIG;return m_en_crc;}
  bool is_powered_up(){AR_CONFIG;return m_pwr_up;}
  bool is_receiver(){AR_CONFIG;return m_prim_rx;}
  bool is_2_byte_crc(){AR_CONFIG;return m_crc_2byte;}
  // SET calls
  void set_use_interrupt_data_ready(bool set){m_mask_rx_dr = set;AW_CONFIG;}
  void set_use_interrupt_data_sent(bool set){m_mask_tx_ds = set;AW_CONFIG;}
  void set_use_interrupt_max_retry(bool set){m_mask_max_rt = set;AW_CONFIG;}
  void crc_enabled(bool set){m_en_crc = set;AW_CONFIG;}
  void set_2_byte_crc(bool set){m_crc_2byte = set;AW_CONFIG;}
  void power_up(bool set){m_pwr_up = set;AW_CONFIG;}
  void receiver(bool set){m_prim_rx = set;AW_CONFIG;}

  // Enable Auto Acknowledgement settings
  bool read_enaa() ;
  bool write_enaa() ;
  // GET calls
  bool is_pipe_ack(uint8_t pipe) ;
  // SET calls
  void set_pipe_ack(uint8_t pipe, bool val) ;

  // Enable pipe settings
  bool read_enrxaddr();
  bool write_enrxaddr() ;
  // GET calls
  bool is_pipe_enabled(uint8_t pipe) ;
  // SET calls
  void enable_pipe(uint8_t pipe, bool enabled) ;

  // Address width settings
  bool set_address_width(uint8_t width) ;
  // gets width of 3, 4 or 5 bytes. Returns 0 if an error occurred
  uint8_t get_address_width() ;

  // Retry settings
  // SET call
  bool set_retry(uint8_t delay_multiplier, uint8_t retry_count);
  // GET calls - returns -1 on error
  int8_t get_retry_delay();
  int8_t get_retry_count();

  // Channel settings
  // SET call
  bool set_channel(uint8_t channel);
  // GET call - returns 0x80 on error as this is an invalid channel value
  uint8_t get_channel();

  // RF Setup
  bool read_setup() ;
  bool write_setup() ;
  // SET call
  void set_power_level(uint8_t level);
  void set_data_rate(uint8_t datarate) ;
  void set_continuous_carrier_transmit(bool set) ;
  // GET call
  uint8_t get_power_level() ;
  uint8_t get_data_rate() ;
  bool is_continuous_carrier_transmit() ;

  // Status register
  bool read_status();
  bool clear_interrupts();
  // Returns the pipe number which contains data to read.
  // If empty or error returns RF24_PIPE_EMPTY (0x07)
  uint8_t get_pipe_available() ;
  // Reads TX_DS interrupt flag raised. Note that this always requires the
  // read_status or another SPI call to update. It will not auto update
  bool has_data_sent();
  // Reads MAX_RT interrupt flag raised. Note that this always requires the
  // read_status or another SPI call to update. It will not auto update
  bool is_at_max_retry_limit();
  // Reads RX_DR interrupt flag raised. Note that this always requires the
  // read_status or another SPI call to update. It will not auto update
  bool has_received_data();
  // Reads TX_FULL register (no auto update on this call, use read_status())
  bool is_transmit_full();
  
  // Observe register
  bool read_observe(uint8_t &packets_lost, uint8_t &retransmitted);

  // Read carrier detect (received power detector for +)
  bool carrier_detect(bool &cd);

  // RX Address registers
  // pipe & address is read-only, len is read/write
  bool set_rx_address(uint8_t pipe, uint8_t *address, uint8_t *len);
  // pipe is read-only, address and len are read/write. If address is NULL
  // then only len is updated to address width
  bool get_rx_address(uint8_t pipe, uint8_t *address, uint8_t *len);

  // TX Address register
  // Sets transmit address.
  // If address to longer than the width then only width number
  // of bytes are set and len is updated to show actual length set.
  bool set_tx_address(uint8_t *address, uint8_t *len) ;
  
  // Gets the transmit address. Len must reflect the length of the address buffer and be long enough
  // to write the address buffer, else false is returned.
  // Len will be changed to match the actual address width if too long.
  // If address is null then len will be set to the width regardless of the value of len.
  bool get_tx_address(uint8_t *address, uint8_t *len) ;
  
  // Payload width registers
  // returns false if call failed, writes width value
  bool get_payload_width(uint8_t pipe, uint8_t *width);
  // returns false if call failed
  // returns false if width is out of range
  bool set_payload_width(uint8_t pipe, uint8_t width);

  // Fifo-status register
  bool read_fifo_status() ;
  // GET calls
  bool is_rx_empty(){AR_FIFO;return m_rx_empty;}
  bool is_rx_full(){AR_FIFO;return m_rx_full;}
  bool is_tx_empty(){AR_FIFO;return m_tx_empty;}
  bool is_tx_full(){AR_FIFO;return m_tx_full;}
  bool is_tx_reuse(){AR_FIFO;return m_tx_reuse;}

  // DYNPD register
  bool read_dynamic_payload() ;
  bool write_dynamic_payload() ;
  // GET calls
  bool is_dynamic_payload(uint8_t pipe);
  // SET
  void set_dynamic_payload(uint8_t pipe, bool set);

  // Feature register
  bool read_feature() ;
  bool write_feature() ;
  // GET calls
  bool dynamic_payloads_enabled(){AR_FEAT;return m_en_dyn_payload;}
  bool payload_ack_enabled(){AR_FEAT;return m_en_ack_payload ;}
  bool tx_noack_cmd_enabled(){AR_FEAT;return m_en_dyn_ack;}
  // SET calls
  void set_dynamic_payloads(bool enable){m_en_dyn_payload=enable;AW_FEAT;}
  void set_payload_ack(bool enable){m_en_ack_payload=enable;AW_FEAT;}
  void set_tx_noack_cmd(bool enable){m_en_dyn_ack=enable;AW_FEAT;}

  bool flushtx();
  bool flushrx();

protected:
  void reset_class() ;
  bool read_register(uint8_t addr, uint8_t *val, uint8_t len);
  bool write_register(uint8_t addr, uint8_t *val, uint8_t len);
  bool enable_features(bool enable) ; // Should this be public?
  void convert_status(uint8_t status) ;
  static void interrupt() ;
  virtual bool max_retry_interrupt() ;
  virtual bool data_sent_interrupt() ;
  virtual bool data_received_interrupt() ;

  IHardwareSPI *m_pSPI ;
  IHardwareGPIO *m_pGPIO ;
  uint8_t m_rxbuf[MAX_RXTXBUF], m_txbuf[MAX_RXTXBUF] ;
  uint8_t m_irq, m_ce;

  bool m_auto_update ; 
  bool m_is_plus ; // Is this a plus model or standard?
  
  // Config register
  bool m_mask_rx_dr ;
  bool m_mask_tx_ds ;
  bool m_mask_max_rt ;
  bool m_en_crc ;
  bool m_crc_2byte ;
  bool m_pwr_up ;
  bool m_prim_rx ;

  // Enable Auto Ack register
  bool m_en_aa[RF24_PIPES] ;
  bool m_enable_pipe[RF24_PIPES] ;

  // Setup register
  bool m_cont_wave ;
  uint8_t m_data_rate ; // 4 levels
  uint8_t m_rf_pwr ; // 4 levels

  // Status register
  bool m_tx_full;
  uint8_t m_rx_pipe_ready ;
  bool m_interrupt_max_rt ;
  bool m_interrupt_tx_ds ;
  bool m_interrupt_rx_dr ;

  // FIFO register
  bool m_tx_reuse ;
  //bool m_tx_full ; // Same meaning as status register
  bool m_tx_empty ;
  bool m_rx_full ;
  bool m_rx_empty ;

  // DYNPD register
  bool m_dyn_payload[RF24_PIPES] ;

  // Feature register
  bool m_en_dyn_payload ;
  bool m_en_ack_payload ;
  bool m_en_dyn_ack ;

  uint8_t m_read_buffer[];
private:

} ;

static std::vector<NordicRF24*> radio_instances ;

#endif
