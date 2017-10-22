BIN_IMAGE = sendCan
###### C flags #####
CC = gcc
CFLAGS = -Wall -g
CFLAGS += -I./
CFLAGS +=-pthread

##### C Source #####
C_SOURCES += sendCan.c
##### OBJECTS #####
OBJECTS += $(patsubst %.c, %.o, $(C_SOURCES))

all: $(BIN_IMAGE)

deps := $(OBJECTS:%.o=%.o.d)

%.o: %.c
	$(CC) -c $< -o $@ $(CFLAGS)

$(BIN_IMAGE): $(OBJECTS)
	$(CC) -Wall -g $(C_SOURCES) -o $@ -lrt
.PHONY : clean
clean:
	rm -f $(BIN_IMAGE)
	find . -name "*.o" -type f -delete
	find . -name "*.d" -type f -delete
