#ifndef ICM20608_HEADER
#define ICM20608_HEADER

#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//#include "i2c_helper.h"

#define ICM20608_DEFAULT_ADDR 0x68

#define WHOAMI_R 0x75

#define ACCEL_XOUT_H_R 0x3B
#define ACCEL_YOUT_H_R 0x3D
#define ACCEL_ZOUT_H_R 0x3F
#define GYRO_XOUT_H_R 0x43
#define GYRO_YOUT_H_R 0x45
#define GYRO_ZOUT_H_R 0x47

#define PWR_MGMT_1_R 0x6B
#define PWR_MGMT_2_R 0x6C

#define ACK_CHECK_EN                    0x1
#define ACK_CHECK_DIS                   0x0
#define ACK_VAL                         0x0
#define NACK_VAL                        0x1

#define DEFAULT_CALIB_ITERATIONS 100

typedef struct ICM_20608_DEV {
    i2c_port_t i2c_num;
    uint8_t address;
    TickType_t TIMEOUT_TICKS;
    uint8_t buffer[14];
}ICM_20608;


void ICM20608_init(ICM_20608* device, i2c_port_t i2c_num, uint8_t address, TickType_t TIMEOUT_TICKS);

esp_err_t ICM20608_Who_Am_I(ICM_20608* device, uint8_t* rx_data);

esp_err_t ICM20608_getMotion(ICM_20608* device, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz); 

esp_err_t ICM20608_getOffsets(ICM_20608* device, int16_t* ax_off, int16_t* ay_off, int16_t* az_off, int16_t* gx_off, int16_t* gy_off, int16_t* gz_off);

esp_err_t I2C_readBytes(ICM_20608* device, uint8_t reg_addr, uint8_t length, uint8_t* data);

esp_err_t I2C_writeBytes(ICM_20608* device, uint8_t reg_addr, uint8_t length, uint8_t data);

#endif