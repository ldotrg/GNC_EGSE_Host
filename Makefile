BIN_IMAGE = readCan
###### C flags #####
CC = gcc
CFLAGS = -Wall -g
CFLAGS += -I./
CFLAGS +=-pthread
##### C++ flags #####
CXX = g++

CXXFLAGS =-g \
	  -I./gpio_sync_timer \
	  -lbiodaq

##### C Source #####
C_SOURCES += can2serialport.c \
	     rs422_serialport.c

##### C++ Source #####

CPP_SOURCES = gpio_sync_timer/DIInterrupt.cpp
##### OBJECTS #####
OBJECTS = $(patsubst %.cpp, %.o, $(CPP_SOURCES))
OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))

##### Target malloc LIB#####

all: $(BIN_IMAGE)

deps := $(OBJECTS:%.o=%.o.d)

%.o: %.cpp
	$(CXX) -c $< -o $@ -MMD -MF $@.d  $(CXXFLAGS) 
%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

$(BIN_IMAGE): $(OBJECTS)
	$(CC) -o $@ $(OBJECTS) $(CFLAGS)
	$(CC) -Wall -g sendCan.c -o sendCan

clean:
	rm -f $(BIN_IMAGE)
	rm -f sendCan
	find . -name "*.o" -type f -delete
	find . -name "*.d" -type f -delete
