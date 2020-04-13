
# NordicRF24
Raspberry Pi & Arduino RF24L01+ C++ library

A Raspberry Pi library for the plus version of the Nordic RF24L01 chip.
Note that this may work with the non-plus versions, but this is untested. 
The primary focus is a simple library to provide an easy to use library.

* IRQ interrupts impelmented and wrapped into base class
* All functionality exposed in hardware is available through the base class
* Basic helper functionality for common queries and data writes
* Buffered read/writes (BufferedRF24) for fixed payload widths

## Library functionality

### Base class
Documentation for the [NordicRF24](NordicRF24.md) details the public functionality available.
The base class offers all functionality to control the hardware. The class avoids automation such as switching from receiver to transmitter modes, dynamic or static payload management or sequencing calls correctly to the hardware.

The intended architecture is to use this class as a base instance, inherit any specialisation and provide suitable interfaces for the job required.

### Tools
rf24command.c provides a simple command line tool to manage the hardware. This allows resetting and printing the state of the registers.
Parameters:
* -c *(specify the CE pin in BCM format)*
* -p *(print the state of the RF24 hardware)*
* -r *(reset the RF24 hardware and pulls CE pin low)*

### Examples
The [BufferedRF24](BufferedRF24.md) class provides blocking and non-blocking read/write calls. This shows how the interrupt virtual calls work in an inherited class. It can be used as a library to support code in it's current state although the error handling and timing isn't optimal. 

Another example is the implementation of a simple ping class with a listener and sender in [PingRF24](PingRF24.md). This can be thought of as a class inheritance implementing a protocol on top of the link layer. Of course it's not a protocol as it will treat any garbage received as a ping message without checking content but the example can be built upon for other implementations of protocols.

### Dependencies
This library requires the hardware library:
https://github.com/AidanHolmes/PiDisplays
This is a bit of a misnomer as the original shared library was for driving displays, but contains interfaces for SPI, timers and GPIO connectivity.

Perform a git clone on this repository in a sibling directory. The hardware library is statically built and linked so doesn't require a shared libarary to be installed (this may change in the future). 

Wiring Pi library is required for this C++ build. For Raspian Jessie:
```
> sudo apt-get update
```
```
> sudo apt-get wiringpi
```

Enable SPIDEV with raspi-config. Enter Advanced and enable SPI.
```
> sudo raspi-config
```

## Why do this?
The primary example libraries are built for Arduino and don't back port too well. The polling implementation used in dedicated hardware, doesn't work well in *user land* code running on Linux.
This implementation uses the interrupt from the RF24 to best effect on all hardware devices. 
