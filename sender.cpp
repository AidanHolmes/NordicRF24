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
#include <signal.h>
#include <stdlib.h>

#define ADDR_WIDTH 5
#define PAYLOAD_WIDTH 32

wPi pi ;
BufferedRF24 *pradio = NULL;

int opt_irq = 0,
  opt_ce = 0,
  opt_listen = 0,
  opt_channel = 0,
  opt_message = 0,
  opt_speed = 1;
bool opt_block = false ;


void siginterrupt(int sig)
{
  printf("\nExiting and resetting radio\n") ;
  pi.output(opt_ce, IHardwareGPIO::low) ;
  if (pradio) pradio->reset_rf24() ;
  exit(EXIT_SUCCESS) ;
}

int main(int argc, char **argv)
{
  const char usage[] = "Usage: %s -c ce -i irq [-o channel] [-a address] [-s 250|1|2] [-l] [-s message]\n" ;
  int opt = 0 ;
  uint8_t rf24address[ADDR_WIDTH] ;
  bool opt_addr_set = false ;
  struct sigaction siginthandle ;
  char szMessage[RF24_BUFFER_WRITE+1] ;

  BufferedRF24 radio ;

  pradio = &radio;

  siginthandle.sa_handler = siginterrupt ;
  sigemptyset(&siginthandle.sa_mask) ;
  siginthandle.sa_flags = 0 ;

  if (sigaction(SIGINT, &siginthandle, NULL) < 0){
    fprintf(stderr,"Failed to set signal handler\n") ;
    return EXIT_FAILURE ;
  }
  
  while ((opt = getopt(argc, argv, "s:i:c:o:a:lm:b")) != -1) {
    switch (opt) {
    case 'l': // listen
      opt_listen = 1;
      break;
    case 'm': // message
      opt_message = 1 ;
      strncpy(szMessage, optarg, RF24_BUFFER_WRITE) ;
      break;
    case 'i': // IRQ pin
      opt_irq = atoi(optarg) ;
      break ;
    case 'c': // CE pin
      opt_ce = atoi(optarg) ;
      break ;
    case 'o': // channel
      opt_channel = atoi(optarg) ;
      break;
    case 's': // speed
      opt_speed = atoi(optarg) ;
      break ;
    case 'b': // blocking
      opt_block = true ;
      break ;
    case 'a': // address
      if (!straddr_to_addr(optarg, rf24address, ADDR_WIDTH)){
	fprintf(stderr, "Invalid address\n") ;
	return EXIT_FAILURE ;
      }
      opt_addr_set = true;
      break;
    default: // ? opt
      fprintf(stderr, usage, argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  
  if (!opt_ce || !opt_irq){
    fprintf(stderr, usage, argv[0]);
    exit(EXIT_FAILURE);
  }

  switch (opt_speed){
  case 1:
    opt_speed = RF24_1MBPS ;
    break ;
  case 2:
    opt_speed = RF24_2MBPS ;
    break ;
  case 250:
    opt_speed = RF24_250KBPS ;
    break ;
  default:
    fprintf(stderr, "Invalid speed option. Use 250, 1 or 2\n") ;
    return EXIT_FAILURE ;
  }

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

  if (!radio.set_gpio(&pi, opt_ce, opt_irq)){
    fprintf(stderr, "Failed to initialise GPIO\n") ;
    return 1 ;
  }
  
  // Transmit and receive address must match receiver address to receive ACKs
  uint8_t def_address[ADDR_WIDTH] = {0xC2,0xC2,0xC2,0xC2,0xC2} ;
  uint8_t addr_len = ADDR_WIDTH ;
  uint16_t bytesread = 0 ;
  
  radio.auto_update(true);
  if (!radio.set_address_width(ADDR_WIDTH)) fprintf(stderr, "Cannot set_address_width\n") ;
  radio.set_retry(15,15);
  radio.set_channel(opt_channel) ; // 2.400GHz + channel MHz
  //radio.set_pipe_ack(0,true) ;
  radio.set_power_level(RF24_0DBM) ;
  radio.crc_enabled(true) ;
  radio.set_2_byte_crc(true) ;

  radio.set_payload_width(0,PAYLOAD_WIDTH) ;  // Must match receivers data length
  radio.enable_pipe(1, false) ; // disable pipe 1
  radio.set_data_rate(opt_speed) ; // slow data rate

  radio.set_transmit_width(PAYLOAD_WIDTH);

  radio.receiver(true) ;
  if (opt_message) radio.receiver(false) ;
  
  radio.power_up(true) ;
  nano_sleep(0,130000) ; // 130 micro seconds

  radio.flushrx();
  radio.flushtx();
  radio.clear_interrupts() ;

  uint8_t addr_width = ADDR_WIDTH ;
  if (opt_listen){ // Listen for messages
    if (!opt_addr_set) memcpy(rf24address, def_address, ADDR_WIDTH) ;
    // Set pipe 1 with the receiving address
    if (!radio.set_rx_address(0, rf24address, &addr_len)) return EXIT_FAILURE;
    print_state(&radio) ; // print the radio config before receiving data

    // Start listening
    if (!pi.output(opt_ce, IHardwareGPIO::high)) return EXIT_FAILURE ;

    uint8_t buffer[PAYLOAD_WIDTH+1] ;
    try{
      for ( ; ; ){
	// Read a byte buffer, which is the max payload width
	bytesread = radio.read(buffer, PAYLOAD_WIDTH, 0, opt_block) ;
	
	while(bytesread > 0){
	  buffer[PAYLOAD_WIDTH] = '\0' ;
	  fprintf(stdout, "%s", buffer) ;
	  fflush(stdout) ;
	  bytesread = radio.read(buffer, PAYLOAD_WIDTH, 0, opt_block) ;
	}
	if(!opt_block) sleep(1); // put in a sleep to stop overloading the CPU
      }
    }catch(BuffIOErr &e){
      fprintf(stderr, "\n%s\n", e.what()) ;
    }
  }else if (opt_message){ // Send a message
    // Use the default address if one isn't specified on the command line
    if (!opt_addr_set) memcpy(rf24address, def_address, ADDR_WIDTH) ;
    // RX and TX addresses have to match for the sender.
    if (!radio.set_tx_address(rf24address, &addr_width)){printf("failed to set tx address\n"); return EXIT_FAILURE ;}
    if (!radio.set_rx_address(0, rf24address, &addr_width)){printf("failed to set tx address\n"); return EXIT_FAILURE ;}

    try{
      radio.write((uint8_t*)szMessage, strlen(szMessage), opt_block) ;
      if (!opt_block) sleep(1) ; // sleep to give the transmit a chance
    }catch(BuffIOErr &e){
      fprintf(stderr, "%s\n", e.what()) ;
    }catch(BuffMaxRetry &e){
      fprintf(stderr, "Message failed to deliver\n") ;
    }
  }
  radio.reset_rf24();
  pi.output(opt_ce, IHardwareGPIO::low);

  return 0 ;
}
