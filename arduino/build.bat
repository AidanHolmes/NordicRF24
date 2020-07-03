@echo off
set HWFILES=arduinohardware.cpp arduinohardware.hpp hardware.cpp hardware.hpp PacketDriver.hpp
set RF24FILES=RF24Driver.cpp RF24Driver.hpp rpinrf24.cpp rpinrf24.hpp
set HW_DIR=../../hardware
set RF24_DIR=..
set ARDUINO_DIR=.
set DRVTEST=drvtest.ino

for %I in (%HWFILES%) do copy %HW_DIR%\%I %ARDUINO_DIR%\%I
for %I in (%RF24FILES%) do copy %RF24_DIR%\%I %ARDUINO_DIR%\%I

arduino --upload -v %DRVTEST%

