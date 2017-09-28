CC=gcc
BIN_IMAGE = readCan
C_SOURCES = can2serialport.c \
	    rs422_serialport.c

CFLAGS = -Wall -g
CFLAGS += -I./
LIB_PATH:=-pthread
all: 
	@$(CC) $(CFLAGS) $(LIB_PATH) $(C_SOURCES) -o $(BIN_IMAGE)
clean:
	rm -f $(BIN_IMAGE)
	rm -f sendCan