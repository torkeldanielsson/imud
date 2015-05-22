#ifndef BNO_055_H
#define BNO_055_H

/* 
 * Defines for the BNO055 
 */

 /* I2C address etc */

#define BNO055_ADDRESS_A                  (0x28)
#define BNO055_ADDRESS_B                  (0x29)
#define BNO055_ID                         (0xA0)


/* Register addresses */

#define BNO055_CHIP_ID_ADDR               (0x00)
#define BNO055_PAGE_ID_ADDR               (0x07)
#define BNO055_QUATERNION_DATA_W_LSB_ADDR (0x20)
#define BNO055_QUATERNION_DATA_W_MSB_ADDR (0x21)
#define BNO055_QUATERNION_DATA_X_LSB_ADDR (0x22)
#define BNO055_QUATERNION_DATA_X_MSB_ADDR (0x23)
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR (0x24)
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR (0x25)
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR (0x26)
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR (0x27)
#define BNO055_TEMP_ADDR                  (0x34)
#define BNO055_ST_RESULT_ADDR             (0x36)
#define BNO055_SYS_CLK_STATUS_ADDR        (0x38)
#define BNO055_SYS_STATUS_ADDR            (0x39)
#define BNO055_SYS_ERR_ADDR               (0x3A)
#define BNO055_OPR_MODE_ADDR              (0x3D)
#define BNO055_PWR_MODE_ADDR              (0x3E)
#define BNO055_SYS_TRIGGER_ADDR           (0x3F)
#define BNO055_TEMP_SOURCE_ADDR           (0x40)

 /* Modes */

#define OPERATION_MODE_CONFIG             (0x00)
#define OPERATION_MODE_NDOF               (0x0C)
#define POWER_MODE_NORMAL                 (0x00)


#endif /* BNO_055_H */
