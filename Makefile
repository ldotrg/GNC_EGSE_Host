BIN_IMAGE = readCan
BIN_IMAGE2 = sendCan
###### C flags #####
CC = gcc
CFLAGS = -Wall -g
CFLAGS += -I./
CFLAGS +=-pthread
##### C++ flags #####
CXX = g++

CXXFLAGS =-g \
	  -I./gpio_sync_timer \
	  -lbiodaq \
	  -I.

##### C Source #####
C_SOURCES += can2serialport.c \
	     rs422_serialport.c

##### C++ Source #####

CPP_SOURCES = gpio_sync_timer/DIInterrupt.cpp
##### OBJECTS #####
OBJECTS = $(patsubst %.cpp, %.o, $(CPP_SOURCES))
OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))

all: $(BIN_IMAGE) $(BIN_IMAGE2)

deps := $(OBJECTS:%.o=%.o.d)

%.o: %.cpp
	$(CXX) -c $< -o $@ -MMD -MF $@.d  $(CXXFLAGS) 
%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

$(BIN_IMAGE): $(OBJECTS)
	$(CXX) -o $@ $(OBJECTS) $(CXXFLAGS) $(CFLAGS)
#	$(CC) -o $@ $(OBJECTS) $(CFLAGS)
$(BIN_IMAGE2): sendCan.c
	$(CC) -Wall -g sendCan.c -o sendCan -lrt
.PHONY : clean
clean:
	rm -f $(BIN_IMAGE)
	rm -f $(BIN_IMAGE2)
	find . -name "*.o" -type f -delete
	find . -name "*.d" -type f -delete
