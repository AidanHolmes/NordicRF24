COMPILER=arduino
OPTIONS=--upload -v

ARDUINO_DIR = .
HW_DIR = ../../hardware
RF24_DIR = ..
HWFILES = arduinohardware.cpp arduinohardware.hpp hardware.cpp hardware.hpp PacketDriver.hpp
RF24FILES = RF24Driver.cpp RF24Driver.hpp rpinrf24.cpp rpinrf24.hpp

DRVTEST=drvtest.ino

.PHONY: all source build clean

all: source build

source: $(HWFILES) $(RF24FILES)

$(HWFILES): $(HW_DIR)/$@
	cp -f $(HW_DIR)/$@ $(ARDUINO_DIR)

$(RF24FILES): $(RF24_DIR)/$@
	cp -f $(RF24_DIR)/$@ $(ARDUINO_DIR)

build:
	$(COMPILER) $(OPTIONS) $(DRVTEST)

clean:
	rm -f $(HWFILES) $(RF24FILES)
