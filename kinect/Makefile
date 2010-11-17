# Makefile for Kinect driver
# Requires libusb-1.0, Player>=3.0, and libfreenect
# Rich Mattes

SRC = kinectdriver.cc
OBJLIBS = kinectdriver.so
OBJS = kinectdriver.o

CXX = g++

all: $(OBJLIBS)

$(OBJS): $(SRC)
	echo Building the KinectDriver plugin...
	$(CXX) -Wall -fpic -g3 -std=gnu++0x `pkg-config --cflags playercore libusb-1.0`  -c $(SRC)

$(OBJLIBS): $(OBJS)
	$(CXX) -shared `pkg-config --libs playercore libusb-1.0` -lfreenect -o $@ $^

clean:
	echo Cleaning up the KinectDriver plugin...
	rm -f $(OBJS) $(OBJLIBS)
