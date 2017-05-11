HWLIBS = ../hardware
CXX = g++
CC = g++
CXXFLAGS= -Wall -I$(HWLIBS) -L$(HWLIBS) -g
CFLAGS = $(CXXFLAGS)
LIBS = -lwiringPi -lpihw
LDFLAGS = -I$(HWLIBS) -L$(HWLIBS)

SRCS_CMD = radioutil.c
OBJS_CMD = $(SRCS_CMD:.c=.o)

SRCS_PING = pingRF24.cpp rpinrf24.cpp
OBJS_PING = $(SRCS_PING:.cpp=.o)

SRCS_RECV = receiver.cpp bufferedrf24.cpp rpinrf24.cpp
OBJS_RECV = $(SRCS_RECV:.cpp=.o) 

SRCS_SEND = sender.cpp bufferedrf24.cpp rpinrf24.cpp
OBJS_SEND = $(SRCS_SEND:.cpp=.o) 

SRCS_CMDUTIL = rf24command.cpp rpinrf24.cpp
OBJS_CMDUTIL = $(SRCS_CMDUTIL:.cpp=.o) 

PINGEXE = rf24ping
SENDEXE = rf24send
RECVEXE = rf24recv
CMDEXE = rf24cmd

.PHONY: all
all: $(PINGEXE) $(SENDEXE) $(RECVEXE) $(CMDEXE)

$(PINGEXE): $(OBJS_PING) libhw
	$(CXX) $(LDFLAGS) $(OBJS_PING) $(LIBS) -o $@

$(RECVEXE): $(OBJS_RECV) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_RECV) $(OBJS_CMD) $(LIBS) -o $@

$(SENDEXE): $(OBJS_SEND) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_SEND) $(OBJS_CMD) $(LIBS) -o $@

$(CMDEXE): $(OBJS_CMDUTIL) $(OBJS_CMD) libhw
	$(CXX) $(LDFLAGS) $(OBJS_CMDUTIL) $(OBJS_CMD) $(LIBS) -o $@

.PHONY: libhw
libhw:
	$(MAKE) libpihw.a -C $(HWLIBS)

.PHONY: clean
clean:
	rm -f *.o $(PINGEXE) $(RECVEXE) $(SENDEXE) $(CMDEXE)
