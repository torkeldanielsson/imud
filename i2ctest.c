#include <stdio.h>
#include <stdlib.h>
#include <wiringPiI2C.h>
#include <inttypes.h>

#include "bno055.h"

void die_with_error (char *s) {
  perror(s);
  exit(1);
}

#define BNO055_ADDRESS_A (0x28)

typedef struct {
  float w;
  float x;
  float y;
  float z;
} quaternion_t;

quaternion_t readBNO055Quaternion(int handle) {
  uint8_t buffer[8];
  quaternion_t quaternion;
  int16_t w, x, y, z;
  w = x = y = z = 0;
  int i;

  buffer[0] = wiringPiI2CReadReg8(handle, BNO055_QUATERNION_DATA_W_LSB_ADDR);
  buffer[1] = wiringPiI2CReadReg8(handle, BNO055_QUATERNION_DATA_W_MSB_ADDR);
  buffer[2] = wiringPiI2CReadReg8(handle, BNO055_QUATERNION_DATA_X_LSB_ADDR);
  buffer[3] = wiringPiI2CReadReg8(handle, BNO055_QUATERNION_DATA_X_MSB_ADDR);
  buffer[4] = wiringPiI2CReadReg8(handle, BNO055_QUATERNION_DATA_Y_LSB_ADDR);
  buffer[5] = wiringPiI2CReadReg8(handle, BNO055_QUATERNION_DATA_Y_MSB_ADDR);
  buffer[6] = wiringPiI2CReadReg8(handle, BNO055_QUATERNION_DATA_Z_LSB_ADDR);
  buffer[7] = wiringPiI2CReadReg8(handle, BNO055_QUATERNION_DATA_Z_MSB_ADDR);

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

int main() {
  int imuHandle;
  quaternion_t quaternion;
  int8_t temp;

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

  wiringPiI2CWriteReg8(imuHandle, BNO055_PAGE_ID_ADDR, 0);

  wiringPiI2CWriteReg8(imuHandle, BNO055_SYS_TRIGGER_ADDR, 0x00);
  usleep(20);
  wiringPiI2CWriteReg8(imuHandle, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
  usleep(20);


  temp = (int8_t)wiringPiI2CReadReg8(imuHandle, BNO055_TEMP_ADDR);
  printf("temp: %i\n", temp);


  while (1) {
    quaternion = readBNO055Quaternion(imuHandle);
    printf("Quaternion values: %f, %f, %f, %f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z); 
    sleep(1);
  }  
}
