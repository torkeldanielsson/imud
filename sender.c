/*
 * sender.c
 *
 * Connects to BNO055 IMU over the RPi UART.
 * Transmits quaternions in UDP packets to SRV_IP 
 * every SLEEP_US microseconds.
 *
 * Inspired by Adafruit library for the BNO055 and by WiringPi.
 *
 * Written 2015 by Torkel Danielsson
 */

#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "imu_types.h"
#include "bno055.h"


#define SERIAL_PORT          "/dev/ttyAMA0"
#define DEFAULT_PORT         5300
#define BUFLEN               128
#define SRV_IP               "10.0.1.8"
#define SLEEP_US             90000 /* 90 ms ~ 111 Hz (BNO055 fusion rate is 100 Hz) */
#define OVERRUN_PROTECT_US   500 /* The BNO055 has lousy UART buffer handling... */


void die_with_error(char *s) {
  perror(s);
  exit(1);
}

/*
 * Functions for interfacing the BNO055 over UART
 */

int fd;

int bno055Read(uint8_t reg, uint8_t *data) {
  uint8_t command[4];
  uint8_t response[3];
  int i;
  
  command[0] = 0xAA;
  command[1] = 0x01;
  command[2] = reg;
  command[3] = 0x01;
  for (i=0; i<4; ++i) {
    write(fd, &command[i], 1);
    usleep(OVERRUN_PROTECT_US);
  }
  
  response[0] = response[1] = response[2] = 0;
  for (i=0; i<2; ++i) {
    if (read(fd, &response[i], 1) != 1) {
      printf("Failure in bno055Read(): read() 1\n");
      printf("Response: 0x%1x %1x %1x\n", response[0], response[1], response[2]);
      return -1;
    }
  }
  if (response[0] != 0xBB || response[1] != 0x01) {
    printf("Failure in bno055Read()\n");
    printf("Reg: 0x%1x\n", reg);
    printf("Command: 0x%1x %1x %1x %1x\n", command[0], command[1], command[2], command[3]);
    printf("Response: 0x%1x %1x %1x\n", response[0], response[1], response[2]);
    return -1;
  }
  if (read(fd, &response[2], 1) != 1) {
    printf("Failure in bno055Read(): read() 2\n");
    printf("Response: 0x%1x %1x %1x\n", response[0], response[1], response[2]);
    return -1;
  }

  *data = response[2];

  return 0;
}

int bno055Write(uint8_t reg, uint8_t data) {
  uint8_t command[5];
  uint8_t response[2];
  int i;

  command[0] = 0xAA;
  command[1] = 0x00;
  command[2] = reg;
  command[3] = 0x01;
  command[4] = data;
  for (i=0; i<5; ++i) {
    write(fd, &command[i], 1);
    usleep(OVERRUN_PROTECT_US);
  }
  
  for (i=0; i<2; ++i) {
    if (read(fd, &response[i], 1) != 1) {
      printf("Failure in bno055Write(): read()\n");
      printf("Response: 0x%1x %1x\n", response[0], response[1]);
      return -1;
    }
  }
  if (response[0] != 0xEE || response[1] != 0x01) {
    printf("Failure in bno055Write()\n");
    printf("Reg: 0x%1x\n", reg);
    printf("Command: 0x%1x %1x %1x %1x %1x\n", command[0], command[1], command[2], command[3], command[4]);
    printf("Response: 0x%1x %1x\n", response[0], response[1]);
    return -1;
  }

  return 0;
}

int bno055Reset() {
  uint8_t command[5];
  uint8_t response;
  int i;

  command[0] = 0xAA;
  command[1] = 0x00;
  command[2] = BNO055_SYS_TRIGGER_ADDR;
  command[3] = 0x01;
  command[4] = 0x20;
  for (i=0; i<5; ++i) {
    write(fd, &command[i], 1);
    usleep(OVERRUN_PROTECT_US);
  }
 
  if (read(fd, &response, 1) != 1)
    return -1;
  if (response != 0xEE) {
    printf("Failure in bno055Reset()\n");
    return -1;
  }

  /* Time from reset to normal mode typ. 650 ms */
  usleep(650000);
  
  return 0;
}

int bno055PrintStatus() {
  uint8_t data;

  if (bno055Read(BNO055_CHIP_ID_ADDR, &data) == -1)
    return -1;
  printf("CHIP_ID 0x%1x\n", data);

  if (bno055Read(BNO055_PAGE_ID_ADDR, &data) == -1)
    return -1;
  printf("PAGE_ID 0x%1x\n", data);

  if (bno055Read(BNO055_TEMP_ADDR, &data) == -1)
    return -1;
  printf("TEMP %i\n", (int)data);

  if (bno055Read(BNO055_ST_RESULT_ADDR, &data) == -1)
    return -1;
  printf("ST_RESULT 0x%1x\n", data);

  if (bno055Read(BNO055_SYS_CLK_STATUS_ADDR, &data) == -1)
    return -1;
  printf("SYS_CLK_STATUS 0x%1x\n", data);

  if (bno055Read(BNO055_SYS_STATUS_ADDR, &data) == -1)
    return -1;
  printf("SYS_STATUS 0x%1x\n", data);

  if (bno055Read(BNO055_SYS_ERR_ADDR, &data) == -1)
    return -1;
  printf("SYS_ERR 0x%1x\n", data);

  if (bno055Read(BNO055_OPR_MODE_ADDR, &data) == -1)
    return -1;
  printf("OPR_MODE 0x%1x\n", data);

  if (bno055Read(BNO055_PWR_MODE_ADDR, &data) == -1)
    return -1;
  printf("PWR_MODE 0x%1x\n", data);

  if (bno055Read(BNO055_TEMP_SOURCE_ADDR, &data) == -1)
    return -1;
  printf("TEMP_SOURCE 0x%1x\n", data);

  return 0;
}

int bno055Init() {
  uint8_t data;
  struct termios options;
  speed_t baud = B115200;
  int status;

  if ((fd = open (SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    die_with_error("Cannot open serial port to device");  

  fcntl (fd, F_SETFL, O_RDWR);

  tcgetattr (fd, &options);

  cfmakeraw   (&options);
  cfsetispeed (&options, baud);
  cfsetospeed (&options, baud);

  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_oflag &= ~OPOST;

  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 10 ; // 1s read timeout

  tcsetattr (fd, TCSANOW | TCSAFLUSH, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  if (bno055Read(BNO055_CHIP_ID_ADDR, &data) == -1)
    die_with_error("Error reading from device");
  if (data != BNO055_ID) {
    printf("data: 0x%1x\n", data);
    die_with_error("Wrong IMU device ID");
  }

  bno055Reset();

  bno055Write(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);

  /* Register page 1 */
  bno055Write(BNO055_PAGE_ID_ADDR, 0x00);

  /* Use external xtal */
  bno055Write(BNO055_SYS_TRIGGER_ADDR, 0x80);
  usleep(20000);

  bno055Write(BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
  usleep(10000);

  do {
    usleep(1000);
    if (bno055Read(BNO055_OPR_MODE_ADDR, &data) == -1)
      die_with_error("Timeout when waiting for device to enter normal mode");
  } while (data != OPERATION_MODE_NDOF);

  return fd;
}

int bno055ReadQuaternion(quaternion_t *quaternion) {
  uint8_t command[4];
  uint8_t response[10];
  int i;
  
  command[0] = 0xAA;
  command[1] = 0x01;
  command[2] = BNO055_QUATERNION_DATA_W_LSB_ADDR;
  command[3] = 0x08;
  for (i=0; i<4; ++i) {
    write(fd, &command[i], 1);
    usleep(OVERRUN_PROTECT_US);
  }
  
  for (i=0; i<2; ++i) {
    if (read(fd, &response[i], 1) != 1) {
      printf("Failure in bno055Read(): read() 1\n");
      printf("Response: 0x%1x %1x %1x\n", response[0], response[1], response[2]);
      return -1;
    }
  }
  if (response[0] != 0xBB || response[1] != 0x08) {
    printf("Failure in bno055ReadQuaternion(): 1\n");
    printf("Command: 0x%1x %1x %1x %1x\n", command[0], command[1], command[2], command[3]);
    printf("Response: 0x%1x %1x -\n", response[0], response[1]);
    return -1;
  }
  for (i=2; i<10; ++i) {
    if (read(fd, &response[i], 1) != 1) {
      printf("Error in bno055ReadQuaternion(): %i\n", i);
      printf("Response: 0x%1x %1x %1x ...\n", response[0], response[1], response[2]);
      return -1;
    }
  }

  quaternion->w = (((uint16_t)response[3]) << 8) | ((uint16_t)response[2]);
  quaternion->x = (((uint16_t)response[5]) << 8) | ((uint16_t)response[4]);
  quaternion->y = (((uint16_t)response[7]) << 8) | ((uint16_t)response[6]);
  quaternion->z = (((uint16_t)response[9]) << 8) | ((uint16_t)response[8]);

  printf("bno055ReadQuaternion(): raw data: ");
  for (i=2; i<10; ++i)
    printf("0x%1x ", response[i]);
  printf("\n");

  return 0;
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

  bno055Init();

  while (1) {
    bno055ReadQuaternion(&quaternion);
    printf("Quaternion values: %f, %f, %f, %f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    sprintf(buf, "%a %a %a %a", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
    if (sendto(socketFd, buf, BUFLEN, 0, (struct sockaddr*) &si_receiver, slen_receiver) == -1)
      die_with_error("Failed sendto()");
    usleep(SLEEP_US);
  }  

  /* Will never reach here... */
  close(socketFd);
  return 0;
}
