# Automatically generated Makefile for arm
sensor_speed.arm : sensor_speed.cc 
	 arm-linux-gnueabi-g++ -ansi -Wall -g -I/usr/arm-unknown-linux-gnueabi/include -I/export/teach/1BRobot -L/usr/arm-unknown-linux-gnueabi/lib sensor_speed.cc -o sensor_speed.arm -lrobot
	 
	 
CC=arm-linux-gnueabi-g++
CFLAGS=-Wall -ansi -g -I artifacts/1BRobot -c
LDFLAGS=-L artifacts/1BRobot
SOURCES=#SOURCE FILES#
OBJECTS=$(SOURCES:.cc=.o)
EXECUTABLE=#OUTPUT FILE NAME#

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@ -lrobot

.cc.o:
	$(CC) $(CFLAGS) $< -o $@
	
clean :
	-rm $(OBJECTS)
