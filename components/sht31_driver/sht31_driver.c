
#include "sht31_driver.h"
#include "i2c_interface.h"

struct i2c_struct sht31_s;

uint8_t SHT31_CRC(uint8_t *data) 
{
    uint8_t crc = 0xff;
    int i, j;
    for(i = 0; i < 2; i++) {
        crc ^= data[i];
        for(j = 0; j < 8; j++) {
            if(crc & 0x80) {
                crc <<= 1;
                crc ^= 0x131;
            }
            else
                crc <<= 1;
        }
    }
    return crc;
}

uint16_t SHT31_CalculateTemp(uint8_t *data_rd, uint8_t unit)
{
    float ret = 0;

    if(unit == 0)//c
    {
        ret = (-45 + (175 *(float)(data_rd[0] * 256 + data_rd[1])/ 65535.0)) * 10.0f;
    }
    else if(unit == 1)//f
    {
        ret = (-49 + (315 *(float)(data_rd[0] * 256 + data_rd[1])/ 65535.0)) * 10.0f;
    }
    
    return ret;
}

void SHT31_Init(void)
{
    sht31_s.port = SHT31_I2C;
    sht31_s.address = SHT31_I2C_ADDRESS;
    sht31_s.reg_size = SHT31_REG_SIZE;
    sht31_s.ack_en = ACK_CHECK_EN;
}

esp_err_t SHT31_ReadSerialNumber(uint32_t* serialNumber)
{
    unsigned char data_rd[6] = {};
  
    esp_err_t error = i2c_read(sht31_s, CMD_READ_SERIALNBR, data_rd, 6);

    // check error
    if(error == ESP_OK)
    {
        if(data_rd[2] != SHT31_CRC(data_rd) || 
        data_rd[5] != SHT31_CRC(data_rd + 3)) 
            return ESP_ERR_INVALID_CRC;

        printf("serialNumber %d %d %d %d", data_rd[0], data_rd[1], data_rd[3], data_rd[4]);

        *serialNumber = (((data_rd[0] << 8) | data_rd[1]) << 16) | ((data_rd[3] << 8) | data_rd[4]);
    }
    
    return error;
}

esp_err_t SHT31_ReadTempHumi(uint16_t *temp, float *humi, uint8_t unit)
{
    unsigned char data_rd[6] = {};
    
    esp_err_t error = i2c_read(sht31_s, CMD_MEAS_POLLING_H, data_rd, 6);

    // check error
    if(error == ESP_OK)
    {
        if(data_rd[2] != SHT31_CRC(data_rd) || 
        data_rd[5] != SHT31_CRC(data_rd + 3)) 
            return ESP_ERR_INVALID_CRC;
        
        *temp = SHT31_CalculateTemp(data_rd, unit);
        *humi = 100 * (float)(data_rd[3] * 256 + data_rd[4]) / 65535.0;
    }

    return error;
}

esp_err_t SHT31_ReadStatus(uint16_t* status)
{
    unsigned char data_rd[3] = {};
  
    esp_err_t error = i2c_read(sht31_s, CMD_READ_STATUS, data_rd, 6);

    // check error
    if(error == ESP_OK)
    {
        // combine the two bytes to a 16-bit value
        *status = (data_rd[0] << 8) | data_rd[1];
    }
    
    return error;
}

esp_err_t SHT31_ClearStatus(void)
{
    unsigned char data_wr[2] = {};

    return i2c_write(sht31_s, CMD_CLEAR_STATUS, data_wr, 1);
}

esp_err_t SHT31_EnableHeater(void)
{
    esp_err_t error; // error code
    unsigned char data_wr[2] = { CMD_HEATER_ENABLE >> 8, CMD_HEATER_ENABLE & 0xFF };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    error = i2c_start_write_access(cmd, SHT31_I2C_ADDRESS, ACK_CHECK_EN);

    // if no error, write heater enable command
    if(error == ESP_OK) error = i2c_master_write(cmd, data_wr, 2, ACK_CHECK_EN);

    error = i2c_stop_access(SHT31_I2C, cmd);

    return error;
}

esp_err_t SHT31_DisableHeater(void)
{
    esp_err_t error; // error code
    unsigned char data_wr[2] = { CMD_HEATER_DISABLE >> 8, CMD_HEATER_DISABLE & 0xFF };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    error = i2c_start_write_access(cmd, SHT31_I2C_ADDRESS, ACK_CHECK_EN);

    // if no error, write heater enable command
    if(error == ESP_OK) error = i2c_master_write(cmd, data_wr, 2, ACK_CHECK_EN);

    error = i2c_stop_access(SHT31_I2C, cmd);

    return error;
}

esp_err_t SHT31_SoftReset(void)
{
    esp_err_t error; // error code
    unsigned char data_wr[2] = { CMD_SOFT_RESET >> 8, CMD_SOFT_RESET & 0xFF };

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    error = i2c_start_write_access(cmd, SHT31_I2C_ADDRESS, ACK_CHECK_EN);

    // write reset command
    if(error == ESP_OK) error = i2c_master_write(cmd, data_wr, 2, ACK_CHECK_EN);

    error = i2c_stop_access(SHT31_I2C, cmd);
    
    // if no error, wait 50 ms after reset
    vTaskDelay(5);

    return error;
}

void SHT3X_HardReset(void)
{
    // set reset low
    // RESET_LOW();

    // wait 100 ms
    vTaskDelay(10);
    
    // release reset
    // RESET_HIGH();
    
    // wait 50 ms after reset
    vTaskDelay(5);
}
