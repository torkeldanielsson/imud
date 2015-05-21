/*
 * Example showing how to use IMUReceiver.
 */

#include <iostream>
#include <unistd.h>

// Include file for IMUReceiver:
#include "imu_receiver.h"


int main(int argc, char *argv[]) 
{
	quaternion_t quaternion;
	
	// Create IMUReceiver object:
	IMUReceiver imuReceiver;
	// Start the reception thread:
	imuReceiver.run();

	while (1)
	{
		// This is how rotation data is fetched:
		quaternion = imuReceiver.getQuaternion();

    	printf("quaternion: %f %f %f %f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    	sleep(3);
    }

    // Terminate the reception thread:
    imuReceiver.stop();

	return 0;
}