#ifndef IMU_RECEIVER_H
#define IMU_RECEIVER_H

/*
 * IMUReceiver
 *
 * Receives UDP packets with IMU data (as quaternions), which can be queried with
 * getQuaternion().
 *
 * Needs a call to run() to start the reception thread.
 *
 * Written 2015 by Torkel Danielsson
 */


#include <thread>

#include "imu_types.h"


#define DEFAULT_PORT 5300
#define BUFLEN 150


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
  char mBuffer[BUFLEN];
  std::thread mReceptionThread;
  bool mStopFlag;
  int mPort;
};


#endif // IMU_RECEIVER_H