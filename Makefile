CC=gcc
BIN_IMAGE = readCan
C_SOURCES = can2serialport.c \
	    rs422_serialport.c

CFLAGS = -Wall -g
CFLAGS += -I./
LIB_PATH:=-pthread
all: 
	$(CC) $(CFLAGS) $(LIB_PATH) $(C_SOURCES) -o $(BIN_IMAGE)
	$(CC) -Wall -g sendCan.c -o sendCan
clean:
	rm -f $(BIN_IMAGE)
	rm -f sendCan