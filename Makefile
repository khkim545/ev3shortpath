OBJS += main.o

SRCS = $(OBJS:.o=.cpp)
HEADERS = $(OBJS:.o=.h) lmstypes.h bytecodes.h
TARGETS = $(OBJS:.o=)
CC = g++ 
CPPFLAGS = -DDEBUG -g -O0
LDFLAGS = -lopencv_highgui -lopencv_core -lopencv_imgproc -lzbar
POSTCMD = ./control

.PHONY: clean

.SUFFIXES: .o
.SUFFIXES: .cpp.o 
	
.o:
	$(CC) $< -o $@ $(LDFLAGS)
.cpp.o:
	$(CC) -c $< -o $@ $(CPPFLAGS)

.dependfile:
	$(CC) -M $(SRCS) > .dependfile

all: $(TARGETS) 
#	$(POSTCMD)

include .dependfile

clean:
	rm -f $(TARGETS) $(OBJS)
	rm -f .dependfile

CPPFLAGS += 
