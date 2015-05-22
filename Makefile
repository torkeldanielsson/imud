#!/usr/bin/make -f

.PHONY: clean all test

all: sender receiver

sender: sender.c bno055.h imu_types.h
	gcc -Wall sender.c -o sender

receiver: bno055.h imu_types.h receiver.cpp imu_receiver.cpp imu_receiver.h
	g++ -Wall -std=c++11  imu_receiver.cpp -c
	g++ -Wall -std=c++11 receiver.cpp -o receiver imu_receiver.o

clean:
	rm -f *.o *~ sender receiver
