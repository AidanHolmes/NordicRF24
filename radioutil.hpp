#ifndef __RADIO_UTILITY
#define __RADIO_UTILITY

#include "rpinrf24.hpp"
#include <time.h>


extern "C"
{

  /* Print to stdout the state of the radio */
  void print_state(NordicRF24 *pRadio);

  /* wraps call to nanosleep - shouldn't be required with use of IHardwareTimer */
  //void nano_sleep(time_t sec, long nano) ;

  /* 
     Convert a hex address string to a byte representation.
     str - hex string
     rf24addr - output address for use with radio
     len - address length (max 5 bytes for NRF24 radio, but will generate longer addresses)
     returns 1 on success and 0 on failure
  */
  int straddr_to_addr(const char *str, uint8_t *rf24addr, const unsigned int len);

  /* 
     Convert an address to a hex string. 
     szaddress must be allocated to at least address_len * 2 + 1
     This is a risky function as memory allocation is unchecked and can easily overflow memory
  */
  void addr_to_straddr(uint8_t *rf24addr, char *szaddress, const uint8_t address_len);

}


#endif
