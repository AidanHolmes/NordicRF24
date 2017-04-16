# PingRF24 class

## Command line

### Required
-c
	specifies the CE pin
-i
	specifies the IRQ pin
-l
	listens for pings and sends an ACK to transmitter. Will wait forever until interrupted
-p
	sends 32 byte ping messages and waits for ACK. Status is sent to stdout

-l or -p should be specified to do something. Run a listener before the ping to see meaningful results.

### Optional
-o
	specify the channel to use. Valid ranges 0 to 125. Each channel is 1MHz from 2.4GHz
-a
	specify the address in hex. Addresses are 10 characters long. If not specified then receive address C2C2C2C2C2 and transmit address E7E7E7E7E7 are used
-s
	set the speed. Options are 1, 2 & 250. These relate to 1MBs, 2MBs and 250KBs speeds. Defaults to 1MBs
-n
	set the number of pings to send. Defaults to 10

## Operation

Enables pipe 0 as ACK receiver for tansmitter.
Pipe 1 acts as the receiver pipe for the listener mode.

* 16 bit CRC used with 32 byte fixed payloads.
* 15 retries with max timeout enabled.
* 5 byte addresses enabled
* Max power output used for all modes of operation

A dummy payload is used for the ping. The ping payload is not full duplex and the time is calculated based on the received ACK.

The ping client and server code demonstrate the use of the interrupt call back functions and code to setup a sender or receiver. 

