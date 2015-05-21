#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <cstdlib>
#include <string.h>
#include <thread>
#include <iostream>
#include <sys/socket.h>

#include "imu_receiver.h"


IMUReceiver::IMUReceiver(int port) 
{
	mPort = port;
	mqIMUData.w = 1.0f;
	mqIMUData.x = 1.0f;
	mqIMUData.y = 0.0f;
	mqIMUData.z = 0.0f;
	mStopFlag = false;
}


void IMUReceiver::run() 
{
	struct sockaddr_in msiReceiver;

	if ((mSocketFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) 
	{
		/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
		 * TODO Implement proper error handling for Oden!
		 */
		perror("IMUReceiver: Failed to create UDP socket");
  		exit(1);
	}

	memset((char *) &msiReceiver, 0, sizeof(msiReceiver));
	msiReceiver.sin_family = AF_INET;
	msiReceiver.sin_port = htons(mPort);
	msiReceiver.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(mSocketFd, (struct sockaddr*) &msiReceiver, sizeof(msiReceiver)) == -1) 
	{
		/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
		 * TODO Implement proper error handling for Oden!
		 */
		perror("IMUReceiver: Failed to bind UDP socket");
  		exit(1);
	}

	struct timeval tv;
	tv.tv_sec = 0;
	tv.tv_usec = 500000;
	if (setsockopt(mSocketFd, SOL_SOCKET, SO_RCVTIMEO, (char *)&tv, sizeof(struct timeval)) == -1) 
	{
		/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
		 * TODO Implement proper error handling for Oden!
		 */
		perror("IMUReceiver: Failed to set socket options");
  		exit(1);
	}

	mReceptionThread = std::thread(&IMUReceiver::loop, this);
}

void IMUReceiver::stop()
{
	mStopFlag = true;

	if (mReceptionThread.joinable())
	{
		mReceptionThread.join();
	}
}


quaternion_t IMUReceiver::getQuaternion()
{
	mIMUDataMutex.lock();
	quaternion_t retVal = mqIMUData;
	mIMUDataMutex.unlock();

	return retVal;
}

	
void IMUReceiver::loop()
{
	struct sockaddr_in msiSender;
	socklen_t slen_sender = sizeof(msiSender);
	int n;
	float w, x, y, z;

	while (1) 
	{
		if (recvfrom(mSocketFd, mBuffer, BUFLEN, 0, (struct sockaddr*) &msiSender, &slen_sender) == -1) 
		{
			if (errno == EAGAIN || EWOULDBLOCK)
			{
				if (mStopFlag)
					break; // terminate thread
				else
					continue;
			}
			else
			{
				/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! 
				 * TODO Implement proper error handling for Oden!
				 */
				perror("IMUReceiver: Failed in recvfrom()");
  				exit(1);
  			}
		} 
		else 
		{
			//printf("IMUReceiver: Received packet from %s:%d\nData: %s\n", inet_ntoa(msiSender.sin_addr), ntohs(msiSender.sin_port), mBuffer);
			n = sscanf(mBuffer, "%a %a %a %a", &w, &x, &y, &z);
			if (n == 4)
			{
				mIMUDataMutex.lock();
				mqIMUData.w = w;
				mqIMUData.x = x;
				mqIMUData.y = y;
				mqIMUData.z = z;
				mIMUDataMutex.unlock();
			}
		}
	}

	close(mSocketFd);
}
