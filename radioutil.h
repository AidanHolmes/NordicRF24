#ifndef __RADIO_UTILITY
#define __RADIO_UTILITY

#include "rpinrf24.hpp"
#include <time.h>

extern "C"
{
  /* Print to stdout the state of the radio */
  void print_state(NordicRF24 *pRadio);

  /* wraps call to nanosleep */
  void nano_sleep(time_t sec, long nano) ;

  /* 
     Convert a hex address string to a byte representation.
     str - hex string
     rf24addr - output address for use with radio
     len - address length (max 5 bytes for NRF24 radio, but will generate longer)
     returns 1 on success and 0 on failure
  */
  int straddr_to_addr(const char *str, uint8_t *rf24addr, const unsigned int len);
}

#endif
