#include "pingRF24.hpp"
#include "wpihardware.hpp"
#include "spihardware.hpp"
#include <iostream>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

#define ADDR_WIDTH 5

PingRF24::PingRF24()
{
  m_failed = 0;
  m_succeeded = 0;
  m_max_ping = 0;
  m_min_ping = 0;
  m_avg_ping = 0;
  m_remaining = 0 ;
  m_tick = 0 ;
  
  if (pthread_mutex_init(&m_rwlock, NULL) != 0)
    std::cerr << "Mutex creation failed\n" ;
}

PingRF24::~PingRF24()
{
  pthread_mutex_destroy(&m_rwlock) ;  
}

bool PingRF24::data_received_interrupt()
{
  uint8_t ping_buff[32] ;
  uint8_t pipe = get_pipe_available();
  if (pipe == RF24_PIPE_EMPTY) return true ; // no pipe

  while(!is_rx_empty()){
    // Read all pings. There's nothing to show for this just do nothing
    if (!read_payload(ping_buff, 32)) return false ; // SPI error
  }
  
  return true ;
}

bool PingRF24::max_retry_interrupt()
{
  pthread_mutex_lock(&m_rwlock) ;

  m_failed++ ;
  clock_t tick = clock() ;
  clock_t time_elapsed = tick - m_tick ;
  std::cout << "Ping timed out " << (time_elapsed / (CLOCKS_PER_SEC/1000)) << " ms (" << "remaining " << m_remaining << ")\n";

  m_tick = 0 ;
  flushtx() ; // Clear failed data in TX buffer

  if (m_remaining > 0){
    m_remaining-- ;
    m_tick = clock() ;
    // Pulse CE and send new packet
    if (!write_packet((uint8_t*)&m_tick)) return 0 ;
  }

  pthread_mutex_unlock(&m_rwlock) ;
  
  return true ;
}

bool PingRF24::data_sent_interrupt()
{
  pthread_mutex_lock(&m_rwlock) ;

  // ACK received. Record time as ping response
  m_succeeded++ ;
  clock_t tick = clock() ;

  uint32_t clock_ticks = tick - m_tick ;
  if (clock_ticks > m_max_ping) m_max_ping = clock_ticks ;
  if (m_min_ping == 0 || clock_ticks < m_min_ping) m_min_ping = clock_ticks ;
  m_avg_ping += clock_ticks ;
  
  float time_elapsed = (float)(tick - m_tick) / (float)(CLOCKS_PER_SEC / 1000) ;
  std::cout << "ACK received in " << time_elapsed << " ms (" << "remaining " << m_remaining << ")\n";

  if (m_remaining > 0){
    m_remaining-- ;
    m_tick = clock() ;
    // Pulse CE and send new packet
    if (!write_packet((uint8_t*)&m_tick)) return 0 ;
  }
  
  pthread_mutex_unlock(&m_rwlock) ;
  
  return true ;
}

bool PingRF24::initialise(uint8_t channel)
{
  // Need to setup GPIO and SPI before initialisation
  if (!m_pGPIO || !m_pSPI) return false ;

  if (!reset_rf24()) return false ;
  
  // Enable all interrupts
  set_use_interrupt_data_ready(true);
  set_use_interrupt_data_sent(true);
  set_use_interrupt_max_retry(true);

  // 16bit CRC enabled
  crc_enabled(true) ;
  set_2_byte_crc(true);

  // Configure pipe 0 as transmit ACK buffer
  set_pipe_ack(0, true);
  if (!set_payload_width(0,32)) return false ; // 32 byte pings

  // Configure pipe 1 to receive
  enable_pipe(1, true) ;
  set_pipe_ack(1, true);
  if (!set_payload_width(1,32)) return false ; // 32 byte pings 

  if (!set_channel(channel)) return false ; // use specified channel
  if (!set_retry(15,15)) return false ; // max retry and delay values
  if (!set_address_width(ADDR_WIDTH)) return false; // 5 byte addresses
  
  set_power_level(RF24_0DBM) ; // max power
  set_data_rate(RF24_1MBPS) ; // 1 MB per sec rate

  if (!clear_interrupts()) return false ;
  if (!set_transmit_width(32)) return false ; // 32 byte pings

  if (!flushtx()) return false ;
  if (!flushrx()) return false ;
  
  return true ;
}

bool PingRF24::listen(uint8_t *address)
{
  uint8_t addr_width = ADDR_WIDTH ;
  if (!set_rx_address(1, address, &addr_width)) return false ;

  receiver(true) ;
  power_up(true) ;
  // Raise CE
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::high)) return false ;

  return true ;
}

bool PingRF24::stop_listening()
{
  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)) return false ;
  power_up(false) ;
  return true ;
}

uint16_t PingRF24::ping(uint8_t *address, uint16_t count)
{
  uint8_t rx_addr_width = ADDR_WIDTH ;
  uint8_t tx_addr_width = ADDR_WIDTH ;

  // Check if a ping is running. Return remaining number of
  // pings if running.
  pthread_mutex_lock(&m_rwlock) ;
  if (m_remaining > 0){
    pthread_mutex_unlock(&m_rwlock) ;
    return m_remaining ;
  }
  pthread_mutex_unlock(&m_rwlock) ;

  if (!address){ return 0 ;}
  
  // Set addresses for new ping
  if (!set_tx_address(address, &tx_addr_width)){printf("failed to set tx address\n"); return 0 ;}
  if (!set_rx_address(0, address, &rx_addr_width)){printf("failed to set tx address\n"); return 0 ;}

  m_failed = 0;
  m_succeeded = 0;
  m_max_ping = 0;
  m_min_ping = 0;
  m_avg_ping = 0 ;
  m_remaining = count-1 ;

  if (count == 0){printf("Count is zero\n") ; return 0;} // nothing to do

  if (!m_pGPIO->output(m_ce, IHardwareGPIO::low)){fprintf(stderr, "Cannot reset GPIO CE pin\n") ; return 0 ;}
  receiver(false) ;
  power_up(true) ;

  clock_t tick = clock() ;
  m_tick = tick ;
  // Pulse CE and send packet
  if (!write_packet((uint8_t*)&tick)){fprintf(stderr, "Write packet failed\n"); return 0 ;}
  
  return m_remaining ;
}

void PingRF24::print_summary()
{
  uint32_t avg_ping = 0 ;
  avg_ping = m_avg_ping / m_succeeded ;

  printf("here as well\n") ;
  std::cout << "Packets failed: " << m_failed << ", succeeded: " << m_succeeded << ", max ping: " << (float)m_max_ping/1000.0 << ", min ping: " << (float)m_min_ping/1000.0 << std::endl ;
  std::cout << "Average ping: " << (float)avg_ping/1000.0 << std::endl;
  uint32_t throughput = (CLOCKS_PER_SEC * 32) / avg_ping ;
  std::cout << "Estimated throughput " << throughput/1024 << " kbytes per sec\n" ;
}

#include <string.h>
#include <time.h>
#include <signal.h>

bool straddr_to_addr(char *str, uint8_t *rf24addr)
{
  int shift = 4;
  memset(rf24addr, 0, ADDR_WIDTH) ;
  if (strlen(str) != ADDR_WIDTH * 2) return false ;
  uint8_t *p = rf24addr ;
  for (int i = 0; i <= ADDR_WIDTH * 2; i++){
    if (str[i] >= '0' && str[i] <= '9'){
      *p |= ((str[i] - '0') << shift) ;
    }else if(str[i] >= 'A' && str[i] <= 'F'){
      *p |= ((str[i] - 'A' + 10) << shift) ;
    }else if(str[i] >= 'a' && str[i] <= 'f'){
      *p |= ((str[i] - 'a' + 10) << shift) ;
    }
    
    if (shift == 0){
      p++ ;
    }
    shift = (shift == 0)?4:0 ;
  }
  return true ;
}

PingRF24 *pradio =NULL ;

void siginterrupt(int sig)
{
  printf("Exiting and resetting radio\n") ;
  if (pradio) pradio->reset_rf24() ;
  exit(EXIT_SUCCESS) ;
}

int main(int argc, char *argv[])
{
  const char usage[] = "Usage: %s -c ce -i irq [-o channel] [-a address] [-s 250|1|2] [-n count] [-l] [-p]\n" ;
  int opt = 0 ;
  int irq = 0, ce = 0, ping = 0, listen = 0, channel = 0, count = 10, speed = 1;
  uint8_t rf24address[ADDR_WIDTH] ;
  bool addr_set = false ;
  struct sigaction siginthandle ;

  siginthandle.sa_handler = siginterrupt ;
  sigemptyset(&siginthandle.sa_mask) ;
  siginthandle.sa_flags = 0 ;

  if (sigaction(SIGINT, &siginthandle, NULL) < 0){
    fprintf(stderr,"Failed to set signal handler\n") ;
    return EXIT_FAILURE ;
  }
  
  while ((opt = getopt(argc, argv, "s:i:c:o:a:n:pl")) != -1) {
    switch (opt) {
    case 'l': // listen
      listen = 1;
      break;
    case 'p': // ping
      ping = 1 ;
      break;
    case 'i': // IRQ pin
      irq = atoi(optarg) ;
      break ;
    case 'c': // CE pin
      ce = atoi(optarg) ;
      break ;
    case 'o': // channel
      channel = atoi(optarg) ;
      break;
    case 's': // speed
      speed = atoi(optarg) ;
      break ;
    case 'a': // address
      if (!straddr_to_addr(optarg, rf24address)) return EXIT_FAILURE ;
      addr_set = true;
      break;
    case 'n': // pings
      count = atoi(optarg) ;
      break ;
    default: // ? opt
      fprintf(stderr, usage, argv[0]);
      exit(EXIT_FAILURE);
    }
  }
  
  if (!ce || !irq){
    fprintf(stderr, usage, argv[0]);
    exit(EXIT_FAILURE);
  }

  switch (speed){
  case 1:
    speed = RF24_1MBPS ;
    break ;
  case 2:
    speed = RF24_2MBPS ;
    break ;
  case 250:
    speed = RF24_250KBPS ;
    break ;
  default:
    fprintf(stderr, "Invalid speed option. Use 250, 1 or 2\n") ;
    return EXIT_FAILURE ;
  }

  PingRF24 radio ;
  wPi pi ; // WiringPi
  spiHw spi ; // SPIDEV interface

  pradio = &radio ;
  
  if (!spi.spiopen(0,0)){ // init SPI
    fprintf(stderr, "Cannot Open SPI\n") ;
    return EXIT_FAILURE;
  }
  spi.setCSHigh(false) ;
  spi.setMode(0) ;
  spi.setSpeed(6000000) ;

  if (!radio.set_gpio(&pi, ce, irq)){
    fprintf(stderr, "Failed to initialise GPIO\n") ;
    return EXIT_FAILURE ;
  }
  
  radio.set_spi(&spi) ;
  radio.auto_update(true);

  if (!radio.initialise(channel)){
    fprintf(stderr, "Cannot initialise using channel %d\n", channel) ;
    return EXIT_FAILURE;
  }

  radio.set_data_rate(speed) ;

  // default addresses
  uint8_t rx_address[ADDR_WIDTH] = {0xC2,0xC2,0xC2,0xC2,0xC2} ;
  uint8_t tx_address[ADDR_WIDTH] = {0xE7,0xE7,0xE7,0xE7,0xE7} ;
  
  if (listen){
    if (!addr_set) memcpy(rf24address, rx_address, ADDR_WIDTH) ;
    if (!radio.listen(rf24address)) return EXIT_FAILURE ;
    for ( ; ; ){
      sleep(1000) ; // ZZZZzz
    }
  }else if (ping){
    if (!addr_set) memcpy(rf24address, tx_address, ADDR_WIDTH) ;
    radio.ping(rf24address,count) ;
    while (radio.ping(NULL,0)){ // non-blocking ping
      sleep(1) ; // poll for completed ping
    }
    radio.print_summary() ;
  }
  
  return EXIT_SUCCESS;
}
