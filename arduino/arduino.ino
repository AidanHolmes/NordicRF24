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

#include "arduinohardware.hpp"
#include "RF24Driver.hpp"
#include "radioutil.hpp"

RF24Driver radio;
ArduinoSpiHw spi ;
ArduinoGPIO gpio ;
ArduinoTimer timer ;

int opt_irq = 2, // SET PIN HERE
  opt_ce = 4, // SET PIN HERE
  opt_cs = 10, // SET PIN HERE
  opt_channel = 76, // CONFIRM CHANNEL
  opt_speed = RF24_1MBPS; // CONFIRM SPEED

char szBroadcast[] = "C0C0C0C0C0" ; // ADDRESS FOR ALL DEVICES
char szAddress[] = "A0A0A0A0A0" ; // ADDRESS OF THIS DEVICE
uint8_t rf24address[PACKET_DRIVER_MAX_ADDRESS_LEN] ;
uint8_t broadcast[PACKET_DRIVER_MAX_ADDRESS_LEN] ;
char szMessage[PACKET_DRIVER_MAX_PAYLOAD] ;

bool data_received(void* ctx, uint8_t* sender, uint8_t* packet)
{
  char szAddr[(PACKET_DRIVER_MAX_ADDRESS_LEN*2)+1] ;
  addr_to_straddr(sender, szAddr, PACKET_DRIVER_MAX_ADDRESS_LEN) ;
  Serial.print("\r\nFrom {");
  Serial.print(szAddr) ;
  Serial.print("}: ");
  Serial.println((char *)packet) ;
  return true ;
}

void print_radio_state(NordicRF24 *pRadio)
{
  int i=0 ;
  uint8_t addr_width = pRadio->get_address_width() ;
  uint8_t address[5];

  Serial.print("Data Ready Interrupt: ");
  Serial.print(pRadio->use_interrupt_data_ready()?"true\r\n":"false\r\n") ;
  Serial.print("Data Sent Interrupt: "); 
  Serial.print(pRadio->use_interrupt_data_sent()?"true\r\n":"false\r\n") ;
  Serial.print("Max Retry Interrupt: ");
  Serial.print(pRadio->use_interrupt_max_retry()?"true\r\n":"false\r\n") ;
  Serial.print("CRC Enabled: "); 
  Serial.print(pRadio->is_crc_enabled()?"true\r\n":"false\r\n") ;
  Serial.print("Is Powered Up: ");
  Serial.print(pRadio->is_powered_up()?"true\r\n":"false\r\n") ;
  Serial.print("Is Receiver: ");
  Serial.print(pRadio->is_receiver()?"true\r\n":"false\r\n") ;
  Serial.print("2 byte CRC: ");
  Serial.print(pRadio->is_2_byte_crc()?"true\r\n":"false\r\n") ;
  Serial.print("Address Width: ");
  Serial.print(addr_width);
  Serial.println();
  Serial.print("Retry Delay: ");
  Serial.print(pRadio->get_retry_delay()) ;
  Serial.println();
  Serial.print("Retry Count: ");
  Serial.print(pRadio->get_retry_count()) ;
  Serial.println();
  Serial.print("Channel: ");
  Serial.print(pRadio->get_channel()) ;
  Serial.println();
  Serial.print("Power Level: ");
  Serial.print(pRadio->get_power_level());
  Serial.println();
  Serial.print("Data Rate: ");
  Serial.print(pRadio->get_data_rate());
  Serial.println();
  Serial.print("Continuous Carrier: ");
  Serial.print(pRadio->is_continuous_carrier_transmit()?"true\r\n":"false\r\n") ;
  Serial.print("Dynamic Payloads: ");
  Serial.print(pRadio->dynamic_payloads_enabled()?"true\r\n":"false\r\n") ;
  Serial.print("Payload ACK: ");
  Serial.print(pRadio->payload_ack_enabled()?"true\r\n":"false\r\n") ;
  Serial.print("TX No ACK: ");
  Serial.print(pRadio->tx_noack_cmd_enabled()?"true\r\n":"false\r\n") ;
  
  for (i=0; i < RF24_PIPES;i++){
    Serial.print("Pipe ");
    Serial.print(i);
    Serial.print(" Enabled: ");
    Serial.print(pRadio->is_pipe_enabled(i)?"true\r\n":"false\r\n") ;
    Serial.print("Pipe ");
    Serial.print(i);
    Serial.print(" ACK: ");
    Serial.print(pRadio->is_pipe_ack(i)?"true\r\n":"false\r\n") ;
    pRadio->get_rx_address(i, address, &addr_width) ;
    Serial.print("Pipe ") ;
    Serial.print(i); 
    Serial.print(" Address: [") ;
    // Print backwards so MSB is printed first
    for (int j=addr_width-1; j >= 0; j--){
      Serial.print(address[j], HEX) ;
    }
    Serial.print("]\r\n");
  }

  pRadio->get_tx_address(address,&addr_width) ;
  Serial.print("Transmit Address: [") ;
  for (int j=addr_width-1; j >= 0; j--){
    Serial.print(address[j], HEX) ;
  }
  Serial.print("]\r\n");  
}

void setup()
{
  // Translate character string to radio address
  straddr_to_addr(szAddress, rf24address, PACKET_DRIVER_MAX_ADDRESS_LEN) ;
  straddr_to_addr(szBroadcast, broadcast, PACKET_DRIVER_MAX_ADDRESS_LEN) ;
  
  // Open SPI
  spi.spiopen(0,opt_cs) ; // init SPI

  // Configure SPI
  // 1 KHz = 1000 Hz
  // 1 MHz = 1000 KHz
  spi.setSpeed(1000000) ;

  // Setup radio
  radio.set_spi(&spi) ;
  radio.set_timer(&timer) ;
  radio.set_gpio(&gpio, opt_ce, opt_irq) ;

  // RF24 setup
  radio.set_data_received_callback(&data_received) ;
  radio.initialise(rf24address, broadcast, PACKET_DRIVER_MAX_ADDRESS_LEN);
  radio.set_channel(opt_channel) ; // 2.400GHz + channel MHz
  radio.set_data_rate(opt_speed) ; // data rate

  // Initialise and wait for Serial
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000) ; // STM32 serial delay
  print_radio_state(&radio) ;
  Serial.print("Usage: <recipient_address> <message>\nPress return (newline) to send\n") ;
}

// Basic state engine variables
uint8_t i=0, j=0 ;
bool readmsg = false ;
uint8_t recipient[PACKET_DRIVER_MAX_ADDRESS_LEN] ;
char recipient_addr[(PACKET_DRIVER_MAX_ADDRESS_LEN*2)+1] ;



void loop()
{
  int c ;

  c = Serial.read() ;
  if (c >= 0){
    switch ((char)c){
    case '\n':
      if (readmsg){
	      recipient_addr[i] = '\0';
	      szMessage[j] = '\0';
	      straddr_to_addr(recipient_addr, recipient, PACKET_DRIVER_MAX_ADDRESS_LEN) ;
	      if (!radio.send(recipient, (uint8_t *)szMessage, j+1)){
	        Serial.print("Failed to send message: ");
	        Serial.println(szMessage) ;
	      }
      }
      i = 0;
      j = 0;
      readmsg = false ;
      break;
    case '\r':
      break ; // ignore
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
}
