#include "radioutil.h"
#include <stdio.h>

void print_state(NordicRF24 *pRadio)
{
  int i=0 ;
  uint8_t addr_width = pRadio->get_address_width() ;
  uint8_t address[5], w=0 ;
  
  printf("Data Ready Interrupt: %s\n", pRadio->use_interrupt_data_ready()?"true":"false") ;
  printf("Data Sent Interrupt: %s\n", pRadio->use_interrupt_data_sent()?"true":"false") ;
  printf("Max Retry Interrupt: %s\n", pRadio->use_interrupt_max_retry()?"true":"false") ;
  printf("CRC Enabled: %s\n", pRadio->is_crc_enabled()?"true":"false") ;
  printf("Is Powered Up: %s\n", pRadio->is_powered_up()?"true":"false") ;
  printf("Is Receiver: %s\n", pRadio->is_receiver()?"true":"false") ;
  printf("2 byte CRC: %s\n", pRadio->is_2_byte_crc()?"true":"false") ;
  printf("Address Width: %d\n", addr_width);
  printf("Retry Delay: %d\n", pRadio->get_retry_delay()) ;
  printf("Retry Count: %d\n", pRadio->get_retry_count()) ;
  printf("Channel: %d\n", pRadio->get_channel()) ;
  printf("Power Level: %d\n",pRadio->get_power_level());
  printf("Data Rate: %d\n",pRadio->get_data_rate());
  printf("Continuous Carrier: %s\n", pRadio->is_continuous_carrier_transmit()?"true":"false") ;
  printf("Dynamic Payloads: %s\n", pRadio->dynamic_payloads_enabled()?"true":"false") ;
  printf("Payload ACK: %s\n", pRadio->payload_ack_enabled()?"true":"false") ;
  printf("TX No ACK: %s\n", pRadio->tx_noack_cmd_enabled()?"true":"false") ;
  
  for (i=0; i < RF24_PIPES;i++){
    printf("Pipe %d Enabled: %s\n", i, pRadio->is_pipe_enabled(i)?"true":"false") ;
    printf("Pipe %d ACK: %s\n", i, pRadio->is_pipe_ack(i)?"true":"false") ;
    pRadio->get_rx_address(i, address, &addr_width) ;
    printf("Pipe %d Address: [", i) ;
    for (int j=0; j < addr_width; j++){
      printf(" %X ",address[j]) ;
    }
    printf("]\n");
    pRadio->get_payload_width(i, &w) ;
    printf("Pipe %d Payload Width: %d\n", i,w) ;
    printf("Pipe %d Dynamic Payloads: %s\n\n", i, pRadio->is_dynamic_payload(i)?"true":"false") ;
  }

  pRadio->get_tx_address(address,&addr_width) ;
  printf("Transmit Address: [", i) ;
  for(int j=0; j < addr_width; j++){
    printf(" %X ",address[j]) ;
  }
  printf("]\n");
  
}
