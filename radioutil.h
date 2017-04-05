#include "rpinrf24.hpp"
#include <time.h>

extern "C"
{
  void print_state(NordicRF24 *pRadio);
  void nano_sleep(time_t sec, long nano) ;
}
