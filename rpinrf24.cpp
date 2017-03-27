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

#include "hardware.hpp"
#include "wpihardware.hpp"
#include "spihardware.hpp"
#include "rpinrf24.hpp"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#ifndef _BV
#define _BV(x) 1 << x
#endif

// Commands
#define RF24_NOP 0xFF
#define RF24_READ_REG 0x00
#define RF24_WRITE_REG 0x20
#define R_RX_PAYLOAD 0x61
#define W_TX_PAYLOAD 0xA0
#define FLUSH_TX 0xE1
#define FLUSH_RX 0xE2
#define REUSE_TX_PL 0xE3
#define R_RX_PL_WID 0x60
#define W_ACK_PAYLOAD 0xA8
#define W_TX_PAYLOAD_NO_ACK 0xB0
// Non-plus commands
#define ACTIVATE 0x50
#define ACTIVATE_FEATURES 0x73

//Registers
#define REG_CONFIG 0x00
#define REG_EN_AA 0x01
#define REG_EN_RXADDR 0x02
#define REG_SETUP_AW 0x03
#define REG_SETUP_RETR 0x04
#define REG_RF_CH 0x05
#define REG_RF_SETUP 0x06
#define REG_STATUS 0x07
#define REG_OBSERVE_TX 0x08
#define REG_CD 0x09
#define REG_RX_ADDR_BASE 0x0A
#define REG_TX_ADDR 0x10
#define REG_RX_PW_BASE 0x11
#define REG_FIFO_STATUS 0x17
#define REG_DYNPD 0x1C
#define REG_FEATURE 0x1D

#include <unistd.h>


void NordicRF24::interrupt()
{
  uint8_t buffer[MAX_RXTXBUF+1] ;

  for (std::vector<NordicRF24*>::iterator i = radio_instances.begin(); i != radio_instances.end(); i++){
    (*i)->read_status() ;
    printf("STATUS:\t\tReceived=%s, Transmitted=%s, Max Retry=%s, RX Pipe Ready=%d, Transmit Full=%s\n",
	   (*i)->has_received_data()?"YES":"NO",
	   (*i)->has_data_sent()?"YES":"NO",
	   (*i)->is_at_max_retry_limit()?"YES":"NO",
	   (*i)->get_pipe_available(),
	   (*i)->is_transmit_full()?"YES":"NO"
	   );
    
    if ((*i)->has_received_data()){
      uint8_t size = (*i)->get_rx_data_size(0) ;
      (*i)->read_payload(buffer, size) ;
      buffer[size] = '\0' ;
      fprintf(stdout, "Pipe %d: %s hex{", (*i)->get_pipe_available(), buffer) ;
      for (uint8_t i=0; i<size;i++){
	fprintf(stdout, " %X ", buffer[i]) ;
      }
      fprintf(stdout, "}\n") ;
    }else{
      fprintf(stdout, "Other IRQ interrupt\n") ;
    }
    
    (*i)->flushtx() ;
    (*i)->flushrx() ;
    (*i)->clear_interrupts() ;
  }
}

NordicRF24::NordicRF24()
{
  m_pSPI = NULL ;
  m_pGPIO = NULL ;
  m_irq = 0;
  m_ce = 0 ;
  m_auto_update = true ;
  m_is_plus = true ;

  // Config register defaults
  m_mask_rx_dr = true ;
  m_mask_tx_ds = true ;
  m_mask_max_rt = true ;
  m_en_crc = true ;
  m_crc_2byte = false ;
  m_pwr_up = false ;
  m_prim_rx = false ;

  
  for(int i=0; i < RF24_PIPES; i++){
    m_en_aa[i] = true ; // Set auto ack defaults on all pipes
    m_enable_pipe[i] = true ;
    m_dyn_payload[i] = false ;
  }

  // Setup register defaults
  m_cont_wave = false ;
  m_data_rate = RF24_2MBPS ;
  m_rf_pwr = RF24_0DBM ;

  // Status register
  m_rx_pipe_ready = RF24_PIPE_EMPTY ;
  m_interrupt_max_rt = false ;
  m_interrupt_tx_ds = false ;
  m_interrupt_rx_dr = false ;
  
  // FIFO and Status registers
  m_tx_full = false ;
  
  // FIFO registers
  m_tx_reuse = false ;
  m_tx_empty = true ;
  m_rx_full = false ;
  m_rx_empty = true;

  // Feature register
  m_en_dyn_payload = false ;
  m_en_ack_payload = false ;
  m_en_dyn_ack = false ;

  // Support for multi instances but this is theoretical and
  // not practical since the hardware is a single instance.
  // Assumes single IRQ being used but multi instances will likely use a
  // separate IRQ GPIO pin. This just means that interrupts will unnecessarily
  // be called. 
  radio_instances.push_back(this) ;  
}
NordicRF24::~NordicRF24()
{
  // Support for multi-instances but isn't thread safe or practical since
  // hardware is a single instance
  for (std::vector<NordicRF24*>::iterator i; i != radio_instances.end(); i++){
    if (*i == this){
      radio_instances.erase(i) ;
      break;
    }
  }
}

bool NordicRF24::set_gpio(IHardwareGPIO *pGPIO, uint8_t ce, uint8_t irq)
{
  if (!pGPIO) return false ;
  m_pGPIO = pGPIO ;
  m_irq = irq;
  m_ce = ce ;

  if (!pGPIO->setup(m_ce, IHardwareGPIO::gpio_output)){
    fprintf(stderr, "Cannot set GPIO output pin for CE\n") ;
    return false ;
  }
  pGPIO->output(m_ce, IHardwareGPIO::low) ;

  if (!pGPIO->setup(m_irq, IHardwareGPIO::gpio_input)){
    fprintf(stderr, "Cannot set GPIO input pin for IRQ\n") ;
    return false ;
  }

  if (!pGPIO->register_interrupt(m_irq, IHardwareGPIO::falling, interrupt)){
    fprintf(stderr, "Cannot set GPIO interrupt pin for IRQ\n") ;
    return false ;
  }
  
  return true ;
}

bool NordicRF24::set_spi(IHardwareSPI *pSPI)
{
  m_pSPI = pSPI ;
  return m_pSPI != NULL ;
}

uint8_t NordicRF24::get_rx_data_size(uint8_t pipe)
{
  uint8_t width = 0;
  if (m_auto_update) read_dynamic_payload() ;
  if (m_dyn_payload[pipe]){
    // Payload is dynamic for this pipe. Assume features
    // are enabled because we have a dynamic payload and read the R_RX_PL_WID
    *m_txbuf = R_RX_PL_WID ;
    if (!m_pSPI->write(m_txbuf, 2)) return 0 ;
    if (!m_pSPI->read(m_rxbuf, 2)) return 0 ;
    width = *(m_rxbuf + 1) ;
    convert_status(*m_rxbuf) ;
    if (width > 32){
      flushrx() ;
      return 0 ; // Corrupt value
    }
    return width ;
  }
  if (!get_payload_width(pipe, &width)) return 0 ;
  return width ;
}

bool NordicRF24::read_payload(uint8_t *buffer, uint8_t len)
{
  if (!m_pSPI) return false ; // No SPI interface
  // Check if receiving or transmitting.
  if (!m_prim_rx) return false ;

  if (buffer == NULL || len > 32) return false ;

  *m_txbuf = R_RX_PAYLOAD ;
  if (!m_pSPI->write(m_txbuf, len+1)) return false ;
  if (!m_pSPI->read(m_rxbuf, len+1)) return false ;
  convert_status(*m_rxbuf) ;
  memcpy(buffer, m_rxbuf+1, len) ;
  return true ;
}


bool NordicRF24::read_register(uint8_t addr, uint8_t *val, uint8_t len)
{
  const uint16_t addmask = 0x01FF;
  if (!m_pSPI) return false ;

  addr &= addmask ;
  *m_txbuf = addr ;
  if (!m_pSPI->write(m_txbuf, len+1)) return false ;
  if (!m_pSPI->read(m_rxbuf, len+1)) return false ;
  memcpy(val, m_rxbuf+1, len) ;
  convert_status(*m_rxbuf) ;
  return true ;
}

bool NordicRF24::write_register(uint8_t addr, uint8_t *val, uint8_t len)
{
  const uint16_t addmask = 0x01FF;
  if (!m_pSPI) return false ;

  addr &= addmask ;
  *m_txbuf = addr | RF24_WRITE_REG;
  if (len >= MAX_RXTXBUF) return false ; // too much data
  
  memcpy(m_txbuf+1, val, len);
  if (!m_pSPI->write(m_txbuf, len+1))return false ;
  if (!m_pSPI->read(m_rxbuf, len+1)) return false ;
  convert_status(*m_rxbuf) ;
  return true ;
}

bool NordicRF24::enable_features(bool enable)
{
  if (!m_pSPI) return false ;
  if (!m_is_plus){
    m_txbuf[0] = ACTIVATE ;
    m_txbuf[1] = enable?ACTIVATE_FEATURES:0;
    return m_pSPI->write(m_txbuf, 2) ;
  }
  return true ;
}

bool NordicRF24::flushtx()
{
  *m_txbuf = FLUSH_TX;
  uint8_t status = 0 ;
  bool ret = m_pSPI->write(m_txbuf,1) ;
  if (ret){
    m_pSPI->read(&status, 1);
    convert_status(status) ;
  }
  return ret ;
}

bool NordicRF24::flushrx()
{
  *m_txbuf = FLUSH_RX;
  uint8_t status = 0 ;
  bool ret = m_pSPI->write(m_txbuf,1) ;
  if (ret){
    m_pSPI->read(&status, 1);
    convert_status(status) ;
  }
  return ret ;
}

bool NordicRF24::read_dynamic_payload()
{
  uint8_t reg = 0 ;
  if (!read_register(REG_DYNPD, &reg, 1)) return false ;

  for (int i = 0; i < RF24_PIPES; i++)
    m_dyn_payload[i] = ((reg & (1 << i)) > 0?true:false) ;
  return true ;
}

bool NordicRF24::write_dynamic_payload()
{
  uint8_t reg = 0;
  for (int i = 0; i < RF24_PIPES; i++)
    reg |= m_dyn_payload[i]?(1 << i):0;

  return write_register(REG_DYNPD, &reg, 1) ;  
}

bool NordicRF24::is_dynamic_payload(uint8_t pipe)
{
  if (pipe >= RF24_PIPES) return false ; // out of range
  if (m_auto_update) read_dynamic_payload() ;
  return m_dyn_payload[pipe] ;
}

void NordicRF24::set_dynamic_payload(uint8_t pipe, bool set)
{
  if (pipe >= RF24_PIPES) return ; // out of range
  m_dyn_payload[pipe] = set;
  if(m_auto_update) write_dynamic_payload() ;
}

bool NordicRF24::read_fifo_status()
{
  uint8_t reg = 0;
  if (!read_register(REG_FIFO_STATUS, &reg, 1)) return false ;
  m_rx_empty = ((reg & _BV(0)) > 0) ;
  m_rx_full = ((reg & _BV(1)) > 0) ;
  m_tx_empty = ((reg & _BV(4)) > 0) ;
  m_tx_full = ((reg & _BV(5)) > 0) ;
  m_tx_reuse = ((reg & _BV(6)) > 0) ;

  return true ;
}

bool NordicRF24::read_feature()
{
  uint8_t reg = 0;
  if (!read_register(REG_FEATURE, &reg, 1)) return false ;
  m_en_dyn_ack = ((reg & _BV(0)) > 0) ;
  m_en_ack_payload = ((reg & _BV(1)) > 0) ;
  m_en_dyn_payload = ((reg & _BV(2)) > 0) ;

  return true ;
}

bool NordicRF24::write_feature()
{
  uint8_t reg = 0 ;
  reg |= (m_en_dyn_ack?_BV(0):0) |
    (m_en_ack_payload?_BV(1):0) |
    (m_en_dyn_payload?_BV(2):0) ;
  return write_register(REG_FEATURE, &reg, 1) ;
}

bool NordicRF24::read_setup()
{
  uint8_t reg = 0;
  if (!read_register(REG_RF_SETUP, &reg, 1)) return false ;

  if ((_BV(5) & reg) > 0) m_data_rate = RF24_250KBPS ;
  else m_data_rate = ((_BV(3) & reg) > 0)?RF24_2MBPS:RF24_1MBPS;

  m_cont_wave = ((_BV(7) & reg) > 0)?true:false;
  m_rf_pwr = (0x06 & reg) >> 1;

  return true ;
}

bool NordicRF24::write_setup()
{
  uint8_t reg = 0;
  reg |= (m_cont_wave?_BV(7):0) |
    (m_data_rate == RF24_250KBPS?_BV(5):0) |
    (m_data_rate == RF24_2MBPS?_BV(3):0) |
    (m_rf_pwr << 1);
  return write_register(REG_RF_SETUP, &reg, 1) ;
}

void NordicRF24::set_power_level(uint8_t level)
{
  if (level > RF24_0DBM) level = RF24_0DBM ;
  m_rf_pwr = level ;
  if (m_auto_update) write_setup() ;
}

void NordicRF24::set_data_rate(uint8_t datarate)
{
  if (datarate < RF24_250KBPS || datarate > RF24_2MBPS) datarate = RF24_1MBPS;
  m_data_rate = datarate ;
  if (m_auto_update) write_setup() ;
}

void NordicRF24::set_continuous_carrier_transmit(bool set)
{
  m_cont_wave = set ;
  if (m_auto_update) write_setup() ;
}

uint8_t NordicRF24::get_power_level()
{
  if (m_auto_update) read_setup() ;
  return m_rf_pwr;
}

uint8_t NordicRF24::get_data_rate()
{
  if (m_auto_update) read_setup() ;
  return m_data_rate;
}

bool NordicRF24::is_continuous_carrier_transmit()
{
  if (m_auto_update) read_setup() ;
  return m_cont_wave;
}

void NordicRF24::convert_status(uint8_t status)
{
  m_tx_full = ((_BV(0) & status) > 0) ;
  m_rx_pipe_ready = 0x07 & (status >> 1);
  m_interrupt_max_rt = ((_BV(4) & status) > 0) ;
  m_interrupt_tx_ds = ((_BV(5) & status) > 0) ;
  m_interrupt_rx_dr = ((_BV(6) & status) > 0) ;
}

uint8_t NordicRF24::get_pipe_available()
{
  return m_rx_pipe_ready ;
}

bool NordicRF24::has_data_sent()
{
  return m_interrupt_tx_ds ;
}

bool NordicRF24::is_at_max_retry_limit()
{
  return m_interrupt_max_rt ;
}

bool NordicRF24::has_received_data()
{
  return m_interrupt_rx_dr ;
}

bool NordicRF24::is_transmit_full()
{
  return m_tx_full ;
}

bool NordicRF24::read_status()
{
  uint8_t reg = 0 ;
  if (!read_register(REG_STATUS, &reg, 1)) return false ;
  convert_status(reg) ;
}

bool NordicRF24::clear_interrupts()
{
  uint8_t reg = 0xF0 ;
  return write_register(REG_STATUS, &reg, 1) ;
}

bool NordicRF24::read_observe(uint8_t &packets_lost, uint8_t &retransmitted)
{
  uint8_t reg = 0 ;
  if (!read_register(REG_OBSERVE_TX, &reg, 1)) return false ;
  packets_lost = 0xF0 & (reg >> 4) ;
  retransmitted = 0xF0 & reg ;
  return true;
}

bool NordicRF24::carrier_detect(bool &cd)
{
  uint8_t reg = 0 ;
  if (read_register(REG_CD, &reg, 1)) return false ;
  cd = ((_BV(0) & reg) > 0);
  return true ;
}

bool NordicRF24::set_rx_address(uint8_t pipe, uint8_t *address, uint8_t *len)
{
  if (pipe >= RF24_PIPES) return false ; // out of range

  if (pipe <= 1){
    uint8_t address_width = get_address_width() ;
    if (address_width == 0 || address_width > *len) return false ;
    *len = address_width ;
    return write_register(REG_RX_ADDR_BASE+pipe, address, address_width) ;
  }
  if (*len < 1) return false ;
  *len = 1 ;
  return write_register(REG_RX_ADDR_BASE+pipe, address, 1) ;
}

bool NordicRF24::get_rx_address(uint8_t pipe, uint8_t *address, uint8_t *len)
{
  uint8_t full_address[5], address_width = 0, low_byte = 0;
  if (pipe >= RF24_PIPES) return false ; // out of range

  address_width = get_address_width() ;
  if (address_width == 0) return false ;
  
  if (address == NULL){
    *len = address_width ;
    return true ;
  }

  if (address_width > *len) return false;
  
  if (pipe > 1){
    if (!read_register(REG_RX_ADDR_BASE+1, full_address, address_width))
      return false;
    if (!read_register(REG_RX_ADDR_BASE+pipe, &low_byte, 1)) return false ;
    memcpy(address, full_address, *len);
    return true ;  
  } 

  return read_register(REG_RX_ADDR_BASE+pipe, address, address_width);
}

bool NordicRF24::set_tx_address(uint8_t *address, uint8_t *len)
{
  uint8_t address_width = get_address_width();
  if (address_width == 0 || address_width > *len) return false ;

  *len = address_width ;
  return write_register(REG_TX_ADDR, address, address_width) ;
}

bool NordicRF24::get_tx_address(uint8_t *address, uint8_t *len)
{
  uint8_t address_width = get_address_width();
  if (address_width == 0) return false ;

  if (address == NULL){
    *len = address_width ;
    return true ;
  }
  if (address_width > *len) return false ;
  *len = address_width ;
  return read_register(REG_TX_ADDR, address, address_width) ;
}

bool NordicRF24::get_payload_width(uint8_t pipe, uint8_t *width)
{
  if (width == NULL) return false ;
  if (pipe >= RF24_PIPES) return false ; 
  return read_register(REG_RX_PW_BASE+pipe, width,1) ;
}

bool NordicRF24::set_payload_width(uint8_t pipe, uint8_t width)
{
  if (pipe >= RF24_PIPES) return false ;
  if (width > 32) return false ;
  return write_register(REG_RX_PW_BASE+pipe, &width, 1) ;
}

bool NordicRF24::read_config()
{
  uint8_t reg = 0 ;
  if (!read_register(REG_CONFIG, &reg, 1)) return false ;

  m_mask_rx_dr = ((_BV(6) & reg) > 0)?false:true ;
  m_mask_tx_ds = ((_BV(5) & reg) > 0)?false:true ;
  m_mask_max_rt = ((_BV(4) & reg) > 0)?false:true ;
  m_en_crc = ((_BV(3) & reg) > 0)?true:false ;
  m_crc_2byte = ((_BV(2) & reg) > 0)?true:false ;
  m_pwr_up = ((_BV(1) & reg) > 0)?true:false ;
  m_prim_rx = ((_BV(0) & reg) > 0)?true:false ;

  return true ;
}
bool NordicRF24::write_config()
{
  uint8_t reg = 0 ;
  reg |= (m_mask_rx_dr?0:_BV(6)) |
    (m_mask_tx_ds?0:_BV(5)) |
    (m_mask_max_rt?0:_BV(4)) |
    (m_en_crc?_BV(3):0) |
    (m_crc_2byte?_BV(2):0) |
    (m_pwr_up?_BV(1):0) |
    (m_prim_rx?_BV(0):0) ;
  return write_register(REG_CONFIG, &reg, 1) ;
}

bool NordicRF24::read_enaa()
{
  uint8_t reg = 0 ;
  if (!read_register(REG_EN_AA, &reg, 1)) return false ;

  for (int i = 0; i < RF24_PIPES; i++)
    m_en_aa[i] = ((reg & (1 << i)) > 0?true:false) ;
  return true ;
}

bool NordicRF24::write_enaa()
{
  uint8_t reg = 0;
  for (int i = 0; i < RF24_PIPES; i++)
    reg |= m_en_aa[i]?(1 << i):0;

  return write_register(REG_EN_AA, &reg, 1) ;  
}

bool NordicRF24::is_pipe_ack(uint8_t pipe)
{
  if (pipe >= RF24_PIPES) return false ; // out of range
  if (m_auto_update) read_enaa() ;
  return m_en_aa[pipe] ;
}

void NordicRF24::set_pipe_ack(uint8_t pipe, bool val)
{
  if (pipe >= RF24_PIPES) return;
  m_en_aa[pipe] = val ;
  if (m_auto_update) write_enaa() ;
}

bool NordicRF24::read_enrxaddr()
{
  uint8_t reg = 0 ;
  if (!read_register(REG_EN_RXADDR, &reg, 1)) return false ;

  for (int i = 0; i < RF24_PIPES; i++)
    m_enable_pipe[i] = ((reg & (1 << i)) > 0?true:false) ;
  return true ;
}

bool NordicRF24::write_enrxaddr()
{
  uint8_t reg = 0;
  for (int i = 0; i < RF24_PIPES; i++)
    reg |= m_enable_pipe[i]?(1 << i):0;

  return write_register(REG_EN_RXADDR, &reg, 1) ;  
}

bool NordicRF24::is_pipe_enabled(uint8_t pipe)
{
  if (pipe >= RF24_PIPES) return false ; // out of range                                                             
  if (m_auto_update) read_enrxaddr() ;
  return m_enable_pipe[pipe] ;
}

void NordicRF24::enable_pipe(uint8_t pipe, bool enabled)
{
  if (pipe >= RF24_PIPES) return;
  m_enable_pipe[pipe] = enabled ;
  if (m_auto_update) write_enrxaddr() ;
}

bool NordicRF24::set_address_width(uint8_t width)
{
  // Check for valid byte address length (3, 4 or 5 bytes)
  if (width > 5 || width < 3) return false ;

  uint8_t reg = width - 2 ;
  
  return write_register(REG_SETUP_AW, &reg, 1) ;
}

uint8_t NordicRF24::get_address_width()
{
  uint8_t reg = 0 ;
  if (!read_register(REG_SETUP_AW, &reg, 1)) return 0 ;

  return reg + 2 ;
}

bool NordicRF24::set_retry(uint8_t delay_multiplier, uint8_t retry_count)
{
  // Check value limits. Only 4 bit values accepted
  if (delay_multiplier >= 0xF0 || retry_count >= 0xF0) return false ;
  uint8_t reg = retry_count + (delay_multiplier << 4);
  return write_register(REG_SETUP_RETR, &reg, 1) ;
}

int8_t NordicRF24::get_retry_delay()
{
  uint8_t reg = 0;
  if (!read_register(REG_SETUP_RETR, &reg, 1)) return -1 ;
  return (int8_t)(reg >> 4) ;
}

int8_t NordicRF24::get_retry_count()
{
  uint8_t reg = 0;
  if (!read_register(REG_SETUP_RETR, &reg, 1)) return -1 ;
  return (int8_t)(reg & 0x0F) ;
}

bool NordicRF24::set_channel(uint8_t channel)
{
  uint8_t reg = channel & ~0x80 ; // Clear top bit if set
  return write_register(REG_RF_CH, &reg, 1) ;
}

uint8_t NordicRF24::get_channel()
{
  uint8_t reg ;
  if (!read_register(REG_RF_CH, &reg, 1)) return 0x80 ; // return invalid channel on error
  return reg ;
}



int main(int argc, char **argv)
{
  NordicRF24 radio ;
  
  // BCM pin
  const int ce_pin = 12 ;
  const int irq_pin = 13 ;
  
  // Create new wiringPi instance
  wPi pi ; // init GPIO

  // Create spidev instance. wiringPi interface doesn't seem to work
  // The SPIDEV code has more configuration to handle devices. 
  spiHw spi ;

  // Pi has only one bus available on the user pins. 
  // Two devices 0,0 and 0,1 are available (CS0 & CS1). 
  if (!spi.spiopen(0,0)){ // init SPI
    fprintf(stderr, "Cannot Open SPI\n") ;
    return 1;
  }

  spi.setCSHigh(false) ;
  spi.setMode(0) ;

  // 1 KHz = 1000 Hz
  // 1 MHz = 1000 KHz
  spi.setSpeed(6000000) ;

  if (!radio.set_gpio(&pi, ce_pin, irq_pin)){
    fprintf(stderr, "Failed to initialise GPIO\n") ;
    return 1 ;
  }

  fprintf(stdout, "IRQ is set to %s\n", (pi.input(irq_pin)==IHardwareGPIO::low)?"LOW":"HIGH");
  
  // Create the radio instance

  uint8_t rx_address[5] = {0xC2,0xC2,0xC2,0xC2,0xC2} ;
  uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7} ;
  uint8_t addr_len = 5 ;
  
  radio.set_spi(&spi) ;
  radio.auto_update(true);
  if (!radio.set_address_width(addr_len)) fprintf(stderr, "Cannot set_address_width\n") ;
  radio.set_retry(15,15);
  radio.set_channel(0x60) ;
  radio.set_pipe_ack(0,true) ;
  radio.set_power_level(RF24_0DBM) ;
  radio.crc_enabled(true) ;
  radio.set_2_byte_crc(true) ;

  radio.set_payload_width(0,8) ;  // Must match senders data length
  radio.set_data_rate(RF24_250KBPS) ;

  radio.set_rx_address(0,rx_address, &addr_len) ;
  radio.set_tx_address(tx_address, &addr_len) ;

  radio.receiver(true) ;
  radio.power_up(true) ;
  sleep(1) ; // or 130 micro sec

  radio.flushrx();
  radio.flushtx();
  radio.clear_interrupts() ;
  pi.output(ce_pin, IHardwareGPIO::high) ;

  for (;;){
    // Do nothing
    
    // get the status
    uint8_t status = 0, NOP = RF24_NOP ;
    /*    
    radio.read_status();

    printf("STATUS:\t\tReceived=%s, Transmitted=%s, Max Retry=%s, RX Pipe Ready=%d, Transmit Full=%s\n",
	   radio.has_received_data()?"YES":"NO",
	   radio.has_data_sent()?"YES":"NO",
	   radio.is_at_max_retry_limit()?"YES":"NO",
	   radio.get_pipe_available(),
	   radio.is_transmit_full()?"YES":"NO"
	   );
    */
    scanf("%c", &status);
    pi.output(ce_pin, IHardwareGPIO::low) ;
    return 0 ;
    sleep(2);
  }
    
  return 0 ;
}
