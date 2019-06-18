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

#include "clientmqtt.hpp"
#include "wpihardware.hpp"
#include "spihardware.hpp"
#include "radioutil.h"
#include "command.hpp"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/select.h>

#define ADDR_WIDTH 5

wPi pi ;
ClientMqttSnRF24 *pradio = NULL;

int opt_irq = 0,
  opt_ce = 0,
  opt_gw = 0,
  opt_channel = 0,
  opt_cname = 0,
  opt_speed = 1,
  opt_ack = 0;


bool inputAvailable()
{
  struct timeval tv;
  fd_set fds;
  tv.tv_sec = 0;
  tv.tv_usec = 0;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
  return (FD_ISSET(0, &fds));
}

void siginterrupt(int sig)
{
  printf("\nExiting and resetting radio\n") ;
  pi.output(opt_ce, IHardwareGPIO::low) ;
  if (pradio){
    pradio->shutdown();
    pradio->reset_rf24() ;
  }
  exit(EXIT_SUCCESS) ;
}

void connect()
{
  uint8_t gw = 0;
  
  if (!pradio->get_known_gateway(&gw)){
    printf("Cannot connect, no known gateway\n") ;
    return;
  }

  try{
    if (!pradio->connect(gw, true, true, 10)){
      printf("Connecting to gateway %u\n", gw) ;
    }
  }catch(MqttConnectErr &e){
    printf("Cannot connect: %s\n", e.what()) ;
  }
}

void search()
{
  if(pradio->searchgw(1)){
    printf("Sending gateway search...\n");
  }else{
    printf("Failed to send search!\n");
  }
}

int main(int argc, char **argv)
{
  const char usage[] = "Usage: %s -c ce -i irq -a address -b address [-n clientname] [-o channel] [-s 250|1|2] [-x]\n" ;
  const char optlist[] = "i:c:o:a:b:s:n:x" ;
  int opt = 0 ;
  uint8_t rf24address[ADDR_WIDTH] ;
  uint8_t rf24broadcast[ADDR_WIDTH] ;
  bool baddr = false;
  bool bbroad = false ;
  char szclientid[MAX_MQTT_CLIENTID+1] ;
  Command commands[] = {Command("connect",&connect,true),
			Command("disconnect",NULL,true),
			Command("topic",NULL,true),
			Command("search",&search,true)} ;
  
  struct sigaction siginthandle ;

  ClientMqttSnRF24 mqtt ;

  pradio = &mqtt;

  siginthandle.sa_handler = siginterrupt ;
  sigemptyset(&siginthandle.sa_mask) ;
  siginthandle.sa_flags = 0 ;

  if (sigaction(SIGINT, &siginthandle, NULL) < 0){
    fprintf(stderr,"Failed to set signal handler\n") ;
    return EXIT_FAILURE ;
  }

  while ((opt = getopt(argc, argv, optlist)) != -1) {
    switch (opt) {
    case 'x': //ack
      opt_ack = 1 ;
      break ;
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
    case 'a': // unicast address
      if (!straddr_to_addr(optarg, rf24address, ADDR_WIDTH)){
	fprintf(stderr, "Invalid address\n") ;
	return EXIT_FAILURE ;
      }
      baddr = true ;
      break;
    case 'b': // broadcast address
      if (!straddr_to_addr(optarg, rf24broadcast, ADDR_WIDTH)){
	fprintf(stderr, "Invalid address\n") ;
	return EXIT_FAILURE ;
      }
      bbroad = true ;
      break;
    case 'n': // client name
      strncpy(szclientid, optarg, MAX_MQTT_CLIENTID) ;
      opt_cname = 1 ;
      break;
    default: // ? opt
      fprintf(stderr, usage, argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  
  if (!opt_ce || !opt_irq || !bbroad || !baddr){
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
  mqtt.set_spi(&spi) ;

  if (!mqtt.set_gpio(&pi, opt_ce, opt_irq)){
    fprintf(stderr, "Failed to initialise GPIO\n") ;
    return 1 ;
  }
  
  mqtt.reset_rf24();
  mqtt.auto_update(true);

  // Link layer specific options
  mqtt.set_retry(15,15);
  mqtt.set_channel(opt_channel) ; // 2.400GHz + channel MHz
  mqtt.set_pipe_ack(0,opt_ack) ; // Turn off/on ACKs
  mqtt.set_pipe_ack(1,opt_ack) ;  // On/Off ACK for this pipe
  mqtt.set_power_level(RF24_0DBM) ;
  mqtt.crc_enabled(true) ;
  mqtt.set_2_byte_crc(true) ;
  mqtt.set_data_rate(opt_speed) ; 

  if (opt_cname)
    mqtt.set_client_id(szclientid);
  else
    mqtt.set_client_id("CL") ;

  mqtt.initialise(ADDR_WIDTH, rf24broadcast, rf24address) ;

  print_state(&mqtt) ;  
  
  time_t now = time(NULL) ;
  time_t last_search = 0, last_register = 0 ;
  const uint16_t search_interval = 5 ; // sec
  const uint16_t ping_interval = 30 ; // sec
  bool ret = false ;
  bool gateway_known = false ;
  uint8_t gwhandle =0;
  
  // Working loop
  for ( ; ; ){
    now = time(NULL) ;

    while (inputAvailable()){
      char c;
      read(STDIN_FILENO, &c, 1);
      switch(c){
      case ' ':
	break;
      default:
	for (int i=0; i < 4; i++){
	  if(commands[i].found(c)){
	    commands[i].cmd();
	    commands[i].reset();
	  }
	}
	break;
      }
      // write(STDOUT_FILENO, &c, 1);
    }
    
    mqtt.manage_connections() ;
    nano_sleep(0, 5000000) ; // 5ms wait
  }

  mqtt.reset_rf24();

  return 0 ;
}
