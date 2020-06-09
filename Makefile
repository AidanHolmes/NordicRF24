DEBUG=-DDEBUG
HWLIBS = ../hardware
CXX = g++
CC = g++
CXXFLAGS= -std=c++11 -Wall -I$(HWLIBS) $(DEBUG) -g
CFLAGS = $(CXXFLAGS)
LIBS = -lwiringPi -lpihw -lpthread
LDFLAGS = -L$(HWLIBS)

SRCS_LIB = bufferedrf24.cpp rpinrf24.cpp RF24Driver.cpp radioutil.cpp
H_LIB = $(SRCS_LIB:.cpp=.hpp)
OBJS_LIB = $(SRCS_LIB:.cpp=.o)

SRCS_DRV = packetdrivertest.cpp RF24Driver.cpp rpinrf24.cpp
OBJS_DRV = $(SRCS_DRV:.cpp=.o)

SRCS_CMD = radioutil.cpp
OBJS_CMD = $(SRCS_CMD:.cpp=.o)

SRCS_PING = pingRF24.cpp rpinrf24.cpp
OBJS_PING = $(SRCS_PING:.cpp=.o)

SRCS_SEND = sender.cpp bufferedrf24.cpp rpinrf24.cpp
OBJS_SEND = $(SRCS_SEND:.cpp=.o) 

SRCS_CMDUTIL = rf24command.cpp rpinrf24.cpp
OBJS_CMDUTIL = $(SRCS_CMDUTIL:.cpp=.o) 

PINGEXE = rf24ping
SENDEXE = rf24send
CMDEXE = rf24cmd
DRVEXE = rf24drvtest
ARCHIVE = librf24.a

.PHONY: all
all: $(PINGEXE) $(SENDEXE) $(CMDEXE) $(DRVEXE) $(ARCHIVE)

$(PINGEXE): $(OBJS_PING) libhw
	$(CXX) $(LDFLAGS) $(OBJS_PING) $(LIBS) -o $@

$(SENDEXE): $(OBJS_SEND) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_SEND) $(OBJS_CMD) $(LIBS) -o $@

$(CMDEXE): $(OBJS_CMDUTIL) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_CMDUTIL) $(OBJS_CMD) $(LIBS) -o $@

$(DRVEXE): $(OBJS_DRV) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_DRV) $(OBJS_CMD) $(LIBS) -o $@

$(ARCHIVE): $(OBJS_LIB)
	ar r $@ $?

$(OBJS_LIB): $(H_LIB)

.PHONY: libhw
libhw:
	$(MAKE) libpihw.a -C $(HWLIBS)

.PHONY: clean
clean:
	rm -f *.o $(PINGEXE) $(SENDEXE) $(CMDEXE) $(ARCHIVE)
