//   Copyright 2020 Aidan Holmes

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

#include "wpihardware.hpp"
#include "spihardware.hpp"
#include "RF24Driver.hpp"
#include "radioutil.hpp"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>

RF24Driver radio;

int opt_irq = 0,
  opt_ce = 0,
  opt_channel = 0,
  opt_speed = 1;


void siginterrupt(int sig)
{
  printf("\nExiting and resetting radio\n") ;
  radio.shutdown() ;
  exit(EXIT_SUCCESS) ;
}

bool data_received(void* ctx, uint8_t* sender, uint8_t* packet)
{
  char szAddr[(PACKET_DRIVER_MAX_ADDRESS_LEN*2)+1] ;
  addr_to_straddr(sender, szAddr, PACKET_DRIVER_MAX_ADDRESS_LEN) ;
  printf("\nFrom {%s}: %s\n", szAddr, packet) ;
  return true ;
}

int main(int argc, char **argv)
{
  const char usage[] = "Usage: %s -c ce -i irq -a address [-o channel] [-s 250|1|2]\n" ;
  int opt = 0 ;
  uint8_t rf24address[PACKET_DRIVER_MAX_ADDRESS_LEN] ;
  bool opt_addr_set = false ;
  struct sigaction siginthandle ;
  char szMessage[PACKET_DRIVER_MAX_PAYLOAD] ;

  siginthandle.sa_handler = siginterrupt ;
  sigemptyset(&siginthandle.sa_mask) ;
  siginthandle.sa_flags = 0 ;

  if (sigaction(SIGINT, &siginthandle, NULL) < 0){
    fprintf(stderr,"Failed to set signal handler\n") ;
    return EXIT_FAILURE ;
  }
  
  while ((opt = getopt(argc, argv, "s:i:c:o:a:")) != -1) {
    switch (opt) {
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
    case 'a': // address
      if (!straddr_to_addr(optarg, rf24address, PACKET_DRIVER_MAX_ADDRESS_LEN)){
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
  
  if (!opt_ce || !opt_irq || !opt_addr_set){
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
  wPi pi ;

  // Pi has only one bus available on the user pins. 
  // Two devices 0,0 and 0,1 are available (CS0 & CS1). 
  if (!spi.spiopen(0,0)){ // init SPI
    fprintf(stderr, "Cannot Open SPI\n") ;
    return 1;
  }

  // 1 KHz = 1000 Hz
  // 1 MHz = 1000 KHz
  spi.setSpeed(6000000) ;
  radio.set_spi(&spi) ;
  radio.set_timer(&pi) ;
  
  if (!radio.set_gpio(&pi, opt_ce, opt_irq)){
    fprintf(stderr, "Failed to initialise GPIO\n") ;
    return 1 ;
  }
  
  uint8_t broadcast[PACKET_DRIVER_MAX_ADDRESS_LEN] = {0xC2,0xC2,0xC2,0xC2,0xC2} ;

  radio.set_data_received_callback(&data_received) ;
  if (!radio.initialise(rf24address, broadcast, PACKET_DRIVER_MAX_ADDRESS_LEN)){
    fprintf(stderr, "Failed to initialise driver\n") ;
    return 1 ;
  }
  radio.set_channel(opt_channel) ; // 2.400GHz + channel MHz
  radio.set_data_rate(opt_speed) ; // slow data rate
  print_state(&radio) ; // print the radio config before receiving data

  uint8_t i=0, j=0 ;
  char c ;
  bool readmsg = false ;
  uint8_t recipient[PACKET_DRIVER_MAX_ADDRESS_LEN] ;
  char recipient_addr[(PACKET_DRIVER_MAX_ADDRESS_LEN*2)+1] ;
  for ( ; ; ){
    read(STDIN_FILENO, &c, 1);
    switch (c){
    case '\n':
      if (readmsg){
	recipient_addr[i] = '\0';
	szMessage[j] = '\0';
	straddr_to_addr(recipient_addr, recipient, PACKET_DRIVER_MAX_ADDRESS_LEN) ;
	if (!radio.send(recipient, (uint8_t *)szMessage, j+1)){
	  fprintf(stderr, "Failed to send message: %s\n", szMessage) ;
	}
      }
      i = 0;
      j = 0;
      readmsg = false ;
      break;
    case ' ':
      if (!readmsg){
	readmsg = true ;
	break;
      }
      // else passthrough
    default:
      if (readmsg && j < PACKET_DRIVER_MAX_PAYLOAD) szMessage[j++] = c ;
      else{
	if (i < PACKET_DRIVER_MAX_ADDRESS_LEN*2){
	  recipient_addr[i] = c ;
	  i++;
	}
      }
    }
  }

  return 0 ;
}

