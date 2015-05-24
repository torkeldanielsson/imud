#include <iostream>
#include <thread>
#include <boost/asio.hpp>

#include "imu_receiver.h"


using boost::asio::ip::udp;


IMUReceiver::IMUReceiver(int port) 
{
  mqIMUData.w = 1.2f; // test data, to know if we're receiving something...
  mqIMUData.x = 3.4f;
  mqIMUData.y = 5.6f;
  mqIMUData.z = 7.8f;
  mStopFlag = false;
  mPort = port;
}


void IMUReceiver::run() 
{
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
  int n;
  float w, x, y, z;
  udp::endpoint sender_endpoint;

  try
  {
    boost::asio::io_service io_service;
    udp::socket sock(io_service, udp::endpoint(udp::v4(), mPort));

    for (;;)
    {
      sock.receive_from(boost::asio::buffer(mBuffer, BUFLEN), sender_endpoint);

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

      if (mStopFlag)
        break; // terminate thread
      else
        continue;
    }
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}
