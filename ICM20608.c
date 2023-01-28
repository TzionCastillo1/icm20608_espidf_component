/** Library is heavily based on ElectronicCats MPU6050 Arduino library
 *  TODO: finish lol
 * 
*/

#include "ICM20608.h"
#include "stdio.h"

esp_err_t I2C_writeBytes(ICM_20608* device, uint8_t reg_addr, uint8_t length, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //shifting address and making LSB 0 to indicate a write
    i2c_master_write_byte(cmd, device->address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(device->i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t I2C_readBytes(ICM_20608* device, uint8_t reg_addr, uint8_t length, uint8_t* data)
{
    int count = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    //shifting address and making LSB 1 to indicate a read
    i2c_master_write_byte(cmd, device->address << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, 1);
    i2c_master_start(cmd);
    for(; count < length - 1; count++){
        i2c_master_write_byte(cmd, device->address << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, data + count, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + count, NACK_VAL); 
    
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(device->i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

void ICM20608_init(ICM_20608* device, i2c_port_t i2c_num, uint8_t address, TickType_t TIMEOUT_TICKS)
{
    device->i2c_num = i2c_num;
    device->address = address;
    device->TIMEOUT_TICKS = TIMEOUT_TICKS;


    I2C_writeBytes(device, PWR_MGMT_1_R, 1, 0b00000001);
    I2C_writeBytes(device, PWR_MGMT_2_R, 1, 0b00000000);

}

esp_err_t ICM20608_Who_Am_I(ICM_20608* device, uint8_t* rx_data)
{
    uint8_t tx_data = WHOAMI_R;
    return i2c_master_write_read_device(device->i2c_num, device->address, &tx_data, 1, rx_data, 1, device->TIMEOUT_TICKS);
}

esp_err_t ICM20608_getMotion(ICM_20608* device, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    esp_err_t ret = I2C_readBytes(device, ACCEL_XOUT_H_R, 2, device->buffer);
    I2C_readBytes(device, ACCEL_YOUT_H_R, 2, device->buffer + 2);
    I2C_readBytes(device, ACCEL_ZOUT_H_R, 2, device->buffer + 4);
    I2C_readBytes(device, GYRO_XOUT_H_R, 2, device->buffer + 8);
    I2C_readBytes(device, GYRO_YOUT_H_R, 2, device->buffer + 10);
    I2C_readBytes(device, GYRO_ZOUT_H_R, 2, device->buffer + 12);
    *ax = (((int16_t)device->buffer[0]) << 8) | device->buffer[1];
    *ay = (((int16_t)device->buffer[2]) << 8) | device->buffer[3];
    *az = (((int16_t)device->buffer[4]) << 8) | device->buffer[5];
    *gx = (((int16_t)device->buffer[8]) << 8) | device->buffer[9];
    *gy = (((int16_t)device->buffer[10]) << 8) | device->buffer[11];
    *gz = (((int16_t)device->buffer[12]) << 8) | device->buffer[13];
    return ret;
}

esp_err_t ICM20608_getOffsets(ICM_20608* device, int16_t* ax_off, int16_t* ay_off, int16_t* az_off, int16_t* gx_off, int16_t* gy_off, int16_t* gz_off)
{
    int16_t axbuf, aybuf, azbuf, gxbuf, gybuf, gzbuf;
    int32_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
    esp_err_t ret = ESP_OK;
    /** 
     * Discard the first 100 readings
     **/
    for(uint8_t i = 0; i < 100; i++){
        ICM20608_getMotion(device, &axbuf, &aybuf, &azbuf, &gxbuf, &gybuf, &gzbuf);  
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    ESP_LOGI("CALIBRATION", "INTERMEDIATE: \n ax: %d \n ay: %d\n az: %d\n gx: %d \n gy: %d \n gz: %d \n", axbuf, aybuf, azbuf, gxbuf, gybuf, gzbuf);
    vTaskDelay(2000/portTICK_PERIOD_MS);
    for(uint16_t i = 0; i < DEFAULT_CALIB_ITERATIONS; i++){
        /** TODO: Store values in someway, and calculate a mean
         *  Potentially automatically add offsets to getMotion()
         **/

        ret = ICM20608_getMotion(device, &axbuf, &aybuf, &azbuf, &gxbuf, &gybuf, &gzbuf);
        ax+=axbuf;
        ay+=aybuf;
        az+=azbuf;
        gx+=gxbuf;
        gy+=gybuf;
        gz+=gzbuf;  
        ESP_LOGI("CALIBRATION", "RUNNING: \n ax: %d \n ay: %d\n az: %d\n gx: %d \n gy: %d \n gz: %d \n", axbuf, aybuf, azbuf, gxbuf, gybuf, gzbuf);

        vTaskDelay(10/portTICK_PERIOD_MS);
    }

    *ax_off = ax / DEFAULT_CALIB_ITERATIONS;
    *ay_off = ay / DEFAULT_CALIB_ITERATIONS;
    *az_off = (az / DEFAULT_CALIB_ITERATIONS) - 16384; //1/2 of max int16_t value
    *gx_off = gx / DEFAULT_CALIB_ITERATIONS;
    *gy_off = gy / DEFAULT_CALIB_ITERATIONS;
    *gz_off = gz / DEFAULT_CALIB_ITERATIONS;

    return ret;
}
