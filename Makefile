CC=g++
CFLAGS=-Wall -ansi -g -I artifacts/1BRobot -c
LDFLAGS=-L artifacts/1BRobot
SOURCES=main.cc line.cc
OBJECTS=$(SOURCES:.cc=.o)
EXECUTABLE=main

all: $(SOURCES) $(EXECUTABLE)
	
$(EXECUTABLE): $(OBJECTS)
	$(CC) $(LDFLAGS) $(OBJECTS) -o $@ -lrobot

.cc.o:
	$(CC) $(CFLAGS) $< -o $@
	
clean :
	-rm $(OBJECTS)
