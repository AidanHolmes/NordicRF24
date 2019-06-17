DEBUG=-DDEBUG
HWLIBS = ../hardware
CXX = g++
CC = g++
CXXFLAGS= -std=c++11 -Wall -I$(HWLIBS) -L$(HWLIBS) $(DEBUG) -g
CFLAGS = $(CXXFLAGS)
LIBS = -lwiringPi -lpihw -lpthread
LDFLAGS = -I$(HWLIBS) -L$(HWLIBS)

SRCS_MQTTCLIENT = mqttclientapp.cpp clientmqtt.cpp mqttsnrf24.cpp bufferedrf24.cpp rpinrf24.cpp mqttconnection.cpp mqtttopic.cpp
OBJS_MQTTCLIENT = $(SRCS_MQTTCLIENT:.cpp=.o) 

SRCS_MQTTSERVER = mqttserverapp.cpp servermqtt.cpp mqttsnrf24.cpp bufferedrf24.cpp rpinrf24.cpp mqttconnection.cpp mqtttopic.cpp
OBJS_MQTTSERVER = $(SRCS_MQTTSERVER:.cpp=.o) 

SRCS_CMD = radioutil.c
OBJS_CMD = $(SRCS_CMD:.c=.o)

SRCS_PING = pingRF24.cpp rpinrf24.cpp
OBJS_PING = $(SRCS_PING:.cpp=.o)

SRCS_SEND = sender.cpp bufferedrf24.cpp rpinrf24.cpp
OBJS_SEND = $(SRCS_SEND:.cpp=.o) 

SRCS_CMDUTIL = rf24command.cpp rpinrf24.cpp
OBJS_CMDUTIL = $(SRCS_CMDUTIL:.cpp=.o) 

MQTTSERVEREXE = mqttsnserver
PINGEXE = rf24ping
SENDEXE = rf24send
CMDEXE = rf24cmd
MQTTCLIENTEXE = mqttsnclient

.PHONY: all
all: $(PINGEXE) $(SENDEXE) $(MQTTSERVEREXE) $(CMDEXE) $(MQTTCLIENTEXE)

$(PINGEXE): $(OBJS_PING) libhw
	$(CXX) $(LDFLAGS) $(OBJS_PING) $(LIBS) -o $@

$(MQTTSERVEREXE): $(OBJS_MQTTSERVER) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_MQTTSERVER) $(OBJS_CMD) -lmosquitto $(LIBS) -o $@

$(MQTTCLIENTEXE): $(OBJS_MQTTCLIENT) $(OBJS_CMD) libhw
		$(CXX) $(LDFLAGS) $(OBJS_MQTTCLIENT) $(OBJS_CMD) $(LIBS) -o $@

$(SENDEXE): $(OBJS_SEND) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_SEND) $(OBJS_CMD) $(LIBS) -o $@

$(CMDEXE): $(OBJS_CMDUTIL) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_CMDUTIL) $(OBJS_CMD) $(LIBS) -o $@

.PHONY: libhw
libhw:
	$(MAKE) libpihw.a -C $(HWLIBS)

.PHONY: clean
clean:
	rm -f *.o $(PINGEXE) $(MQTTSERVEREXE) $(SENDEXE) $(CMDEXE) $(MQTTCLIENTEXE)
