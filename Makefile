DEBUG=-DDEBUG
HWLIBS = ../hardware
CXX = g++
CC = g++
CXXFLAGS= -std=c++11 -Wall -I$(HWLIBS) -L$(HWLIBS) $(DEBUG) -g
CFLAGS = $(CXXFLAGS)
LIBS = -lwiringPi -lpihw -lpthread
LDFLAGS = -I$(HWLIBS) -L$(HWLIBS)

SRCS_MQTT = mqttapp.cpp mqttsnrf24.cpp bufferedrf24.cpp rpinrf24.cpp
OBJS_MQTT = $(SRCS_MQTT:.cpp=.o) 

SRCS_CMD = radioutil.c
OBJS_CMD = $(SRCS_CMD:.c=.o)

SRCS_PING = pingRF24.cpp rpinrf24.cpp
OBJS_PING = $(SRCS_PING:.cpp=.o)

SRCS_SEND = sender.cpp bufferedrf24.cpp rpinrf24.cpp
OBJS_SEND = $(SRCS_SEND:.cpp=.o) 

SRCS_CMDUTIL = rf24command.cpp rpinrf24.cpp
OBJS_CMDUTIL = $(SRCS_CMDUTIL:.cpp=.o) 

MQTTEXE = rf24mqtt
PINGEXE = rf24ping
SENDEXE = rf24send
CMDEXE = rf24cmd

.PHONY: all
all: $(PINGEXE) $(SENDEXE) $(MQTTEXE) $(CMDEXE)

$(PINGEXE): $(OBJS_PING) libhw
	$(CXX) $(LDFLAGS) $(OBJS_PING) $(LIBS) -o $@

$(MQTTEXE): $(OBJS_MQTT) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_MQTT) $(OBJS_CMD) -lmosquitto $(LIBS) -o $@

$(SENDEXE): $(OBJS_SEND) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_SEND) $(OBJS_CMD) $(LIBS) -o $@

$(CMDEXE): $(OBJS_CMDUTIL) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_CMDUTIL) $(OBJS_CMD) $(LIBS) -o $@

.PHONY: libhw
libhw:
	$(MAKE) libpihw.a -C $(HWLIBS)

.PHONY: clean
clean:
	rm -f *.o $(PINGEXE) $(MQTTEXE) $(SENDEXE) $(CMDEXE)
