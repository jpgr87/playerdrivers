SRC = kinectdriver.cc
OBJLIBS = kinectdriver.so
OBJS = kinectdriver.o

CXX = g++

all: $(OBJLIBS)

$(OBJS): $(SRC)
	echo Building the PlayerDriver plugin...
	$(CXX) -Wall -fpic -g3 -std=gnu++0x `pkg-config --cflags playercore libusb-1.0`  -c $(SRC)

$(OBJLIBS): $(OBJS)
	$(CXX) -shared `pkg-config --libs playercore libusb-1.0` -lfreenect -o $@ $^

clean:
	echo Cleaning up the PlayerDriver plugin...
	rm -f $(OBJS) $(OBJLIBS)