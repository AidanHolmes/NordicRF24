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
#include "bufferedrf24.hpp"
#include "wpihardware.hpp"
#include "spihardware.hpp"
#include "radioutil.h"
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
  radio.set_spi(&spi) ;

  if (!radio.set_gpio(&pi, ce_pin, irq_pin)){
    fprintf(stderr, "Failed to initialise GPIO\n") ;
    return 1 ;
  }
  
  // Transmit and receive address must match receiver address to receive ACKs
  uint8_t rx_address[5] = {0xC2,0xC2,0xC2,0xC2,0xC2} ;
  uint8_t tx_address[5] = {0xC2,0xC2,0xC2,0xC2,0xC2} ;
  uint8_t addr_len = 5 ;
  
  radio.auto_update(true);
  if (!radio.set_address_width(addr_len)) fprintf(stderr, "Cannot set_address_width\n") ;
  radio.set_retry(15,15);
  radio.set_channel(0x60) ; // 2.496GHz
  //radio.set_pipe_ack(0,true) ;
  radio.set_power_level(RF24_0DBM) ;
  radio.crc_enabled(true) ;
  radio.set_2_byte_crc(true) ;

  //radio.set_payload_width(0,8) ;  // Must match receivers data length
  radio.enable_pipe(1, false) ; // disable unused pipes
  radio.set_data_rate(RF24_250KBPS) ; // slow data rate

  // Set addresses
  radio.set_tx_address(tx_address, &addr_len) ;
  radio.set_rx_address(0,rx_address, &addr_len) ;

  radio.set_transmit_width(8);
  
  radio.receiver(false) ;
  radio.power_up(true) ;
  nano_sleep(0,130000) ; // 130 micro seconds

  radio.flushrx();
  radio.flushtx();
  radio.clear_interrupts() ;

  print_state(&radio) ;

  uint8_t send_me[] = "Hello World" ;
  try{
    uint16_t sent = radio.write(send_me, strlen((char*)send_me), true) ;
    printf("Blocking write returned %d\n", sent) ;
  }catch(BuffIOErr &e){
    fprintf(stderr, "%s\n", e.what()) ;
  }catch(BuffMaxRetry &e){
    fprintf(stderr, "Message failed to deliver\n") ;
  }
  sleep(1);
  radio.reset_rf24();
    
  return 0 ;
}
