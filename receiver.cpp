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

#include "rpinrf24.hpp"
#include "wpihardware.hpp"
#include "spihardware.hpp"
#include "radioutil.h"
#include "bufferedrf24.hpp"
#include <stdio.h>
#include <unistd.h>
#include <string.h>

int main(int argc, char **argv)
{
  BufferedRF24 radio ;
  
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

  const int payloadwidth = 8 ;
  uint8_t rx_address[5] = {0xC2,0xC2,0xC2,0xC2,0xC2} ;
  uint8_t tx_address[5] = {0xE7,0xE7,0xE7,0xE7,0xE7} ;
  uint8_t addr_len = 5 ;
  
  radio.set_spi(&spi) ;
  radio.auto_update(true);
  if (!radio.reset_rf24()) fprintf(stderr, "Cannot reset the radio\n") ;
  
  if (!radio.set_address_width(addr_len)) fprintf(stderr, "Cannot set_address_width\n") ;
  radio.set_retry(15,15);
  radio.set_channel(0x60) ;
  radio.set_pipe_ack(0,true) ;
  radio.set_power_level(RF24_0DBM) ;
  radio.crc_enabled(true) ;
  radio.set_2_byte_crc(true) ;

  radio.set_payload_width(0,payloadwidth) ;  // Must match senders data length
  radio.enable_pipe(1, false) ;
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

  print_state(&radio) ;

  bool block = false;
  if (argc > 1){
    if (strcmp("-b", argv[1]) == 0){
      block = true;
      printf("blocking\n");
    }
  }

  uint8_t buffer[33] ;
  try{
    for (;;){
      uint16_t bytes = radio.read(buffer, payloadwidth, 0, block) ;
      while(bytes){
	buffer[payloadwidth] = '\0' ;
	fprintf(stdout, "DATA: %s hex{", buffer) ;
	for (uint8_t i=0; i<payloadwidth;i++){
	  fprintf(stdout, " %X ", buffer[i]) ;
	}
	fprintf(stdout, "}\n") ;
	bytes = radio.read(buffer, payloadwidth, 0, false) ;	
      }
      if(!block) sleep(5);
    }

  }catch(BuffIOErr &e){
    fprintf(stderr, "%s\n", e.what()) ;
  }

  pi.output(ce_pin, IHardwareGPIO::low) ;
  
  return 0 ;
}
