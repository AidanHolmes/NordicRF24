#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include "rpinrf24.hpp"
#include "wpihardware.hpp"
#include "spihardware.hpp"
#include "radioutil.hpp"
#include <string.h>

void set_channel(NordicRF24 *r, int channel)
{
  if (channel >= 128){
    fprintf(stderr, "Invalid channel\n") ;
    return ;
  }
  r->set_channel(channel) ;
}

void print_info(NordicRF24 *pRadio)
{
  uint8_t dr = pRadio->get_data_rate() ;
  pRadio->set_data_rate(RF24_250KBPS) ;
  if (pRadio->get_data_rate() == RF24_250KBPS){
    printf("RF24L01+ variant\n") ;
  }else{
    printf("RF24L01 standard variant\n") ;
  }
  // Reset the original data rate
  pRadio->set_data_rate(dr) ;

  printf("FIFO status-----------\n") ;
  printf("RX empty:\t%s\n", pRadio->is_rx_empty()?"yes":"no") ;
  printf("RX full:\t%s\n", pRadio->is_rx_full()?"yes":"no") ;
  printf("TX empty:\t%s\n", pRadio->is_tx_empty()?"yes":"no") ;
  printf("TX full:\t%s\n", pRadio->is_tx_full()?"yes":"no") ;
  printf("TX reuse:\t%s\n", pRadio->is_tx_reuse()?"yes":"no") ;
}

int main(int argc, char *argv[])
{
  const char usage[] = "Usage: %s -c ce [-r] [-o channel] [-p]\n" ;
  int opt = 0, reset = 0, print = 0, info = 0;
  int ce = 0, chan = -1 ;
  
  while ((opt = getopt(argc, argv, "o:pric:")) != -1) {
    switch (opt) {
    case 'r': // reset
      reset = 1;
      break;
    case 'o': // channel
      chan = atoi(optarg) ;
      break;
    case 'p': // print state
      print = 1 ;
      break ;
    case 'i':
      info = 1 ;
      break;
    case 'c': // CE pin
      ce = atoi(optarg) ;
      break ;
    default: // ? opt
      fprintf(stderr, usage, argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  
  if (!ce){
    fprintf(stderr, usage, argv[0]);
    exit(EXIT_FAILURE);
  }

  printf("Using pins CE %d\n", ce) ;

  NordicRF24 radio ;
  wPi pi ;
  spiHw spi ;

  if (!spi.spiopen(0,0)){ // init SPI
    fprintf(stderr, "Cannot Open SPI\n") ;
    return EXIT_FAILURE;
  }
  spi.setCSHigh(false) ;
  spi.setMode(0) ;
  spi.setSpeed(6000000) ;

  if (!radio.set_gpio(&pi, ce, 0)){
    fprintf(stderr, "Failed to initialise GPIO\n") ;
    return EXIT_FAILURE ;
  }
  
  radio.set_spi(&spi) ;
  radio.auto_update(true);
  
  if (reset){
    printf ("Resetting RF24...\n") ;
    pi.output(ce, IHardwareGPIO::low) ;
    radio.reset_rf24() ;
    radio.flushrx() ;
    radio.flushtx() ;
    radio.clear_interrupts() ;
  }
  if (chan >= 0){
    printf("Setting channel %d\n", chan) ;
    set_channel(&radio, chan);
  }
  if (print){
    print_state(&radio) ;
  }
  if (info){
    if (print) printf("\n\n") ;
    print_info(&radio);
  }
  
  return EXIT_SUCCESS;
}

