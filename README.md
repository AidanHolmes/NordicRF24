# NordicRF24
Raspberry Pi RF24L01+ C++ library

Incomplete library (work-in-progress) to create a Raspberry Pi library for the plus version of the Nordic RF24L01 chip.
The primary focus is a simple library to provide an easy to use library and avoid the API of the commonly used RF24 Arduino library.

3 modes of connectivity
- no-ack broadcast comms (simple fire-and-forget transmissions)
- ack full duplex communication with fixed length payloads
- ack full duplex communication protocol using dynamic payloads across all pipes

## Current library support

See the rpinrf24.hpp for all calls to configure and run the RF24L01+ chip with an Raspberry Pi.
This library requires the hardware library
https://github.com/AidanHolmes/PiDisplays

This is a bit of a misnomer as the original shared library was for driving displays, but contains interfaces for SPI, timers and GPIO connectivity.
Wiring Pi library and SPIDEV required for this C++ build.

## Why do this?

The RF24 is there and has been modified to work with the RPi. It's quite possible I've not seen great examples, but there's not good IRQ handling and there seems to be some unnecessary work going on to actually drive this chip.
I don't get along with the way the supporting libraries imply the pipes are an open handle. The chip is either receiving or writing and it's a mode of operation. I think the API can be easier to work with as a receiver or sender, with the receiver in a multi-node environment and a sender in a point-to-point environment.

## Progress

Receives 8 byte data frames from sender (pynrf24).

## To do

Everything above. Needs a transmitter example and support for dynamic payloads (full enhanced mode available).
MQTT protocol to be supported as an inherited class.