#ifndef IMU_TYPES_H
#define IMU_TYPES_H

/*
 * Types for IMU data transfer.
 */

typedef struct {
  float w;
  float x;
  float y;
  float z;
} quaternion_t;


#endif // IMU_TYPES_H