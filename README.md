
# NordicRF24
Raspberry Pi & Arduino RF24L01+ C++ library

A Raspberry Pi library for the plus version of the Nordic RF24L01 chip.
Note that this may work with the non-plus versions, but this is untested. 
The primary focus is a simple library to provide an easy to use library.

* IRQ interrupts impelmented and wrapped into base class
* All functionality exposed in hardware is available through the base class
* Basic helper functionality for common queries and data writes
* Buffered read/writes (BufferedRF24) for fixed payload widths
* Packet driver interface

## Library functionality

### Base class
Documentation for the [NordicRF24](NordicRF24.md) details the public functionality available.
The base class offers all functionality to control the hardware. The class avoids automation such as switching from receiver to transmitter modes, dynamic or static payload management or sequencing calls correctly to the hardware.

The intended architecture is to use this class as a base instance, inherit any specialisation and provide suitable interfaces.

### Tools
rf24command.c provides a simple command line tool to manage the hardware. This allows resetting and printing the state of the registers.
Parameters:
* -c *(specify the CE pin in BCM format)*
* -p *(print the state of the RF24 hardware)*
* -r *(reset the RF24 hardware and pulls CE pin low)*
* -i *(additional information about the chip)*
* -s *(search the channel space for signals over -64dBM)

### Examples
The [BufferedRF24](BufferedRF24.md) class provides blocking and non-blocking read/write calls. This shows how the interrupt virtual calls work in an inherited class. It can be used as a library to support code in it's current state although the error handling and timing isn't optimal. 

rf24ping - a simple ping class with a listener and sender in [PingRF24](PingRF24.md). This can be thought of as a class inheritance implementing a protocol on top of the link layer. Of course it's not a protocol as it will treat any data received as a ping message without checking content but the example can be built upon for other implementations of protocols.

rf24send - creates a listener or one-time sender application that can send a string from the command line to the listener.

[rf24drvtest](DrvTest.md) - uses the RF24PacketDriver class to implement a bidirectional communication app. Simply add the destination address and a short string to send. 

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

## What is different compared to other RF24 libraries?
Many libraries are primarily built for Arduino. Raspberry Pi feels like an afterthought and compatibility with newer versions of the Raspberry Pi were not available at the time.

The driver code uses the IRQ instead of polling for the interrupt. Whilst polling isn't an issue with Arduino it can be a performance issue in a operating system environment.

Size and speed of this code will likely be larger and slower due to the nature of the interface layers and reliability on underlying APIs (WiringPi, SPIDEV or Arduino API). 

## RF24 application use-case
RF24 devices make sense when looking for the lowest price option at short to medium range (across a room or building using the PA+LNA devices).

Reliablity seems to be in the sweet spot at the slower protocol speeds of 1Mbs or 250Kbs. The devices are sensitive to delivered power levels and external power sources seem to work better than USB power even with separate capacitors and separate 3.3v regulators. 

The max speed of 2Mbs is more theoretical and although tests will suggest speeds close this (try rf24ping), the reality of needing some protocol to exist for streaming data will effectively shrink this throughput. Modern Bluetooth, Zigbee or WiFi hardware chips now fit this space much better than the old RF24s. 

32 bytes provides enough data to issue simple controls and readings for IoT projects. They are cheap enough that you can use to serve simple applications such as a temperature reading device in every room of your house.