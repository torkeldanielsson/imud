#include <arpa/inet.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPiI2C.h>
#include <inttypes.h>

#include "imu_types.h"
#include "bno055.h"


#define DEFAULT_PORT    5300
#define BUFLEN          128
#define SRV_IP          "127.0.0.1"
#define SLEEP_US        50

void die_with_error(char *s) {
  perror(s);
  exit(1);
}

/*
 * Functions etc for interfacing BNO055.
 * Requires WiringPi with I2C to be installed and set up.
 */

int imuHandle;

int bno055Init() {

  imuHandle = wiringPiI2CSetup(BNO055_ADDRESS_A);
  if (imuHandle == -1)
    die_with_error("Cannot connect to IMU");  

  if ((uint16_t)wiringPiI2CReadReg8(imuHandle, BNO055_CHIP_ID_ADDR) != BNO055_ID)
    die_with_error("Wrong IMU device ID");

  wiringPiI2CWriteReg8(imuHandle, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
  usleep(20);

  /* reset */
  wiringPiI2CWriteReg8(imuHandle, BNO055_SYS_TRIGGER_ADDR, 0x20);
  while (wiringPiI2CReadReg8(imuHandle, BNO055_CHIP_ID_ADDR) != BNO055_ID)
    usleep(50); 

  wiringPiI2CWriteReg8(imuHandle, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
  usleep(20);

  /*  */
  wiringPiI2CWriteReg8(imuHandle, BNO055_PAGE_ID_ADDR, 0x00);

  wiringPiI2CWriteReg8(imuHandle, BNO055_SYS_TRIGGER_ADDR, 0x00);
  usleep(20);
  wiringPiI2CWriteReg8(imuHandle, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
  usleep(20);

  return imuHandle;
}

quaternion_t bno055ReadQuaternion() {
  uint8_t buffer[8];
  quaternion_t quaternion;
  int16_t w, x, y, z;
  w = x = y = z = 0;
  int i;

  buffer[0] = wiringPiI2CReadReg8(imuHandle, BNO055_QUATERNION_DATA_W_LSB_ADDR);
  buffer[1] = wiringPiI2CReadReg8(imuHandle, BNO055_QUATERNION_DATA_W_MSB_ADDR);
  buffer[2] = wiringPiI2CReadReg8(imuHandle, BNO055_QUATERNION_DATA_X_LSB_ADDR);
  buffer[3] = wiringPiI2CReadReg8(imuHandle, BNO055_QUATERNION_DATA_X_MSB_ADDR);
  buffer[4] = wiringPiI2CReadReg8(imuHandle, BNO055_QUATERNION_DATA_Y_LSB_ADDR);
  buffer[5] = wiringPiI2CReadReg8(imuHandle, BNO055_QUATERNION_DATA_Y_MSB_ADDR);
  buffer[6] = wiringPiI2CReadReg8(imuHandle, BNO055_QUATERNION_DATA_Z_LSB_ADDR);
  buffer[7] = wiringPiI2CReadReg8(imuHandle, BNO055_QUATERNION_DATA_Z_MSB_ADDR);

  w = (((uint16_t)buffer[1]) << 8) | ((uint16_t)buffer[0]);
  x = (((uint16_t)buffer[3]) << 8) | ((uint16_t)buffer[2]);
  y = (((uint16_t)buffer[5]) << 8) | ((uint16_t)buffer[4]);
  z = (((uint16_t)buffer[7]) << 8) | ((uint16_t)buffer[6]);

  quaternion.w = (float)w;
  quaternion.x = (float)x;
  quaternion.y = (float)y;
  quaternion.z = (float)z;

  printf("raw data: ");
  for (i=0; i<8;++i)
    printf("0x%1x ", buffer[i]);
  printf("\n");

  return quaternion;
}


int main(int argc, char *argv[]) {
  struct sockaddr_in si_receiver;
  int socketFd; 
  socklen_t slen_receiver = sizeof(si_receiver);
  char buf[BUFLEN];
  quaternion_t quaternion;
  
  if ((socketFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    die_with_error("Failed to create UDP socket");
  }
  
  memset((char *) &si_receiver, 0, sizeof(si_receiver));
  si_receiver.sin_family = AF_INET;
  si_receiver.sin_port = htons(DEFAULT_PORT);
  if (inet_aton(SRV_IP, &si_receiver.sin_addr) == 0) {
    die_with_error("Failed inet_aton()");
  }
  
  if (sendto(socketFd, buf, BUFLEN, 0, (struct sockaddr*) &si_receiver, slen_receiver) == -1)
    die_with_error("Failed sendto()");

  while (1) {
    quaternion = readBNO055Quaternion(imuHandle);
    printf("Quaternion values: %f, %f, %f, %f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    sprintf(buf, "%a %a %a %a", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    if (sendto(socketFd, buf, BUFLEN, 0, (struct sockaddr*) &si_receiver, slen_receiver) == -1)
      die_with_error("Failed sendto()");
    sleep(1);
  }  

  /* Will never reach here... */
  close(s);
  return 0;
}