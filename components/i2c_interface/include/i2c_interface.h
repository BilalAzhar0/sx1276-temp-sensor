#include "driver/i2c.h"


#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#define SDA_IO_NUM 19
#define SCL_IO_NUM 22

#define I2C_MASTER_TX_BUF_DISABLE 0 
#define I2C_MASTER_RX_BUF_DISABLE 0
#define WRITE_BIT I2C_MASTER_WRITE            
#define READ_BIT I2C_MASTER_READ
#define ACK_CHECK_EN 0x1
#define ACK_CHECK_DIS 0x0
#define ACK_VAL 0x0
#define NACK_VAL 0x1
typedef struct i2c_struct
{
   uint8_t port;
   uint8_t address;
   uint8_t reg_size;
   bool ack_en;
} i2c_t;

esp_err_t i2c_init(i2c_port_t i2c_num);
esp_err_t i2c_start_write_access(i2c_cmd_handle_t cmd, uint8_t address, bool ack_en);
esp_err_t i2c_start_read_access(i2c_cmd_handle_t cmd, uint8_t address, bool ack_en);
esp_err_t i2c_stop_access(i2c_port_t i2c_num, i2c_cmd_handle_t cmd);
esp_err_t i2c_read_bytes(i2c_cmd_handle_t cmd, unsigned char* data_rd, uint8_t size);
esp_err_t i2c_read(i2c_t i2c_s, uint16_t reg, unsigned char* data_rd, uint8_t size);
esp_err_t i2c_write(i2c_t i2c_s, uint16_t reg, unsigned char* data_wr, uint8_t size);

#endif