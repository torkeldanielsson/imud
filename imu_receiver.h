#ifndef IMU_RECEIVER_H
#define IMU_RECEIVER_H

/*
 * Receives UDP packets with IMU data, which can be queried with
 * getQuaternion().
 * Needs a call to run() to start reception thread.
 *
 * TODO: implement prober error handling for Oden! (See comments in .cpp)
 */

#include <thread>

#include "imu_types.h"


#define DEFAULT_PORT 5300
#define BUFLEN 100


class IMUReceiver 
{

public:
	IMUReceiver(int port = DEFAULT_PORT);
	void run();
	void stop();
	quaternion_t getQuaternion();

private:
	void loop();

	quaternion_t mqIMUData;
	std::mutex mIMUDataMutex;
	int mSocketFd; 
	int mPort; 
	char mBuffer[BUFLEN];
	std::thread mReceptionThread;
	bool mStopFlag;
};


#endif // IMU_RECEIVER_H