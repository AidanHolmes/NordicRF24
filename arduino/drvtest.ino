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

/*
 * Build notes
 * _gettimeofday required for STM32 library
 */

extern "C" {
  // This must exist to keep the linker happy but is never called.
  int _gettimeofday( struct timeval *tv, void *tzvp )
  {
    uint64_t t = 0;  // get uptime in nanoseconds
    tv->tv_sec = t / 1000000000;  // convert to seconds
    tv->tv_usec = ( t % 1000000000 ) / 1000;  // get remaining microseconds
    return 0;  // return non-zero for error
  } // end _gettimeofday()
}

RF24Driver radio;
ArduinoSpiHw spi ;
ArduinoGPIO gpio ;
ArduinoTimer timer ;

int opt_irq = 0, // SET PIN HERE
  opt_ce = 0, // SET PIN HERE
  opt_cs = 0, // SET PIN HERE
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
  Serial.print("\nFrom {%s}: %s\n", szAddr, packet) ;
  return true ;
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
  radio.set_data_rate(opt_speed) ; // slow data rate

  // Initialise and wait for Serial
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(1000) ; // STM32 serial delay
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

