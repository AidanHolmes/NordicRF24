# rf24drvtest

## Command line
Usage: ./rf24drvtest -c ce -i irq -a address [-o channel] [-s 250|1|2]

### Required
-c
	specifies the CE pin
-i
	specifies the IRQ pin
-a
	address to use for the device (should be unique)

### Optional
-o
	specify the channel to use. Valid ranges 0 to 125. Each channel is 1MHz from 2.4GHz
-s
	set the speed. Options are 1, 2 & 250. These relate to 1MBs, 2MBs and 250KBs speeds. Defaults to 1MBs

## Operation

The driver test uses the RF24Driver class for bidirectional sending and receiving of data.
Pipe 0 and 1 are enabled with pipe 0 listening on a broadcast address of C0C0C0C0C0 and pipe 1 listening for direct comms to the address set using option -a.

Addresses are 5 bytes and set as hex values on the command line. Note that the Arduino code sets all parameters in the code itself and will need rebuilding and uploading to change.

Once the code is run then commands can be entered via the standard input (Linux) or over serial (Arduino).
Each message starts with the 5 byte destination address. A space is used to start the message which will be limited to 27 characters. Send a newline character to end the message and send to the destination.
The destination will print the address of the sender and the message if successfully received. Check your settings, distance between radios and wiring if communication isn't working. USB power to devices can cause some instability. Switch to a separate power source and try again.

e.g.
6060606060 hello world

will send to device with address 6060606060 and print 'hello world'.

Any messages sent to the broadcast should appear on all devices.

