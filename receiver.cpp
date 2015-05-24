/*
 * Example showing how to use IMUReceiver.
 */

#include <iostream>
#include <chrono>
#include <thread>

// Include file for IMUReceiver:
#include "imu_receiver.h"


int main(int argc, char *argv[]) {
  quaternion_t quaternion;
  
  // Create IMUReceiver object:
  IMUReceiver imuReceiver;

  // Start the reception thread:
  imuReceiver.run();

  for (;;) {
    // This is how rotation data is fetched:
    quaternion = imuReceiver.getQuaternion();
  
    std::cout << "quaternion: " << quaternion.w << " " << quaternion.x << " " << quaternion.y << " " << quaternion.z << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(80));
  }

  // Terminate the reception thread:
  imuReceiver.stop();

  return 0;
}