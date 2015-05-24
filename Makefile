#!/usr/bin/make -f

.PHONY: clean all test

CPPFLAGS = -Wall -std=c++11
INCLUDES = -I./ -I/usr/local/include
LIBS = -L/usr/lib/ -L/usr/local/lib -lboost_thread-mt -lboost_system

all: sender receiver

sender: sender.c bno055.h imu_types.h
	gcc -Wall sender.c -o sender

receiver: bno055.h imu_types.h receiver.cpp imu_receiver.cpp imu_receiver.h
	g++ $(CPPFLAGS) imu_receiver.cpp -c $(INCLUDES)
	g++ $(CPPFLAGS) receiver.cpp -o receiver imu_receiver.o $(INCLUDES) $(LIBS)

gl_test: bno055.h imu_types.h gl_test.cpp imu_receiver.cpp imu_receiver.h
	g++  $(CPPFLAGS) imu_receiver.cpp -c $(INCLUDES)
	g++  -std=c++11 -w gl_test.cpp -o gl_test imu_receiver.o $(INCLUDES) $(LIBS) -framework GLUT -framework OpenGL

clean:
	rm -f *.o *~ sender receiver
