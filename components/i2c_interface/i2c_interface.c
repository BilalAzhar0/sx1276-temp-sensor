#include "i2c_interface.h"

esp_err_t i2c_init(i2c_port_t i2c_num)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_IO_NUM;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = SCL_IO_NUM;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
    i2c_param_config(i2c_num, &conf);
    return i2c_driver_install(i2c_num, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

esp_err_t i2c_start_write_access(i2c_cmd_handle_t cmd, uint8_t address, bool ack_en)
{
    i2c_master_start(cmd);
    return i2c_master_write_byte(cmd, (address << 1) | WRITE_BIT, ack_en);
}

esp_err_t i2c_start_read_access(i2c_cmd_handle_t cmd, uint8_t address, bool ack_en)
{
    i2c_master_start(cmd);
    return i2c_master_write_byte(cmd, (address << 1) | READ_BIT, ack_en);
}

esp_err_t i2c_stop_access(i2c_port_t i2c_num, i2c_cmd_handle_t cmd)
{
    i2c_master_stop(cmd);
    esp_err_t error = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_PERIOD_MS);
    
    if (error != ESP_OK) {
        return error;
    }
    
    i2c_cmd_link_delete(cmd);

    return ESP_OK;
}

esp_err_t i2c_read(i2c_t i2c_s, uint16_t reg, unsigned char* data_rd, uint8_t size)
{
    unsigned char reg_buf[2] = {0};

    if(i2c_s.reg_size == 1)
    {
        reg_buf[0] = reg;
    }
    else if(i2c_s.reg_size == 2)
    {
        reg_buf[0] = reg >> 8;
        reg_buf[1] = reg;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    esp_err_t error = i2c_master_start(cmd);
    error |= i2c_master_write_byte(cmd, (i2c_s.address << 1) | WRITE_BIT, i2c_s.ack_en);

    error |= i2c_master_write(cmd, reg_buf, i2c_s.reg_size, i2c_s.ack_en);
    error |= i2c_stop_access(i2c_s.port, cmd);

    // Delay 20 ms
    vTaskDelay(2);

    cmd = i2c_cmd_link_create();
    error |= i2c_master_start(cmd);

    error |= i2c_master_write_byte(cmd, (i2c_s.address << 1) | READ_BIT, i2c_s.ack_en);

    if (size > 1) {
        error |= i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }

    error |= i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_stop_access(i2c_s.port, cmd);

    return error;
}

esp_err_t i2c_write(i2c_t i2c_s, uint16_t reg, unsigned char* data_wr, uint8_t size)
{
    unsigned char reg_buf[2] = {0};

    if(i2c_s.reg_size == 1)
    {
        reg_buf[0] = reg;
    }
    else if(i2c_s.reg_size == 2)
    {
        reg_buf[0] = reg >> 8;
        reg_buf[1] = reg;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    esp_err_t error = i2c_master_start(cmd);
    error |= i2c_master_write_byte(cmd, (i2c_s.address << 1) | WRITE_BIT, i2c_s.ack_en);
    error |= i2c_master_write(cmd, reg_buf, i2c_s.reg_size, i2c_s.ack_en);
    
    if (size > 1) {
        error |= i2c_master_write(cmd, data_wr, size - 1, ACK_VAL);
        error |= i2c_master_write_byte(cmd, data_wr + size - 1, NACK_VAL);
    }
    else
    {
        error |= i2c_master_write_byte(cmd, data_wr[0], NACK_VAL);
    }
    
    i2c_stop_access(i2c_s.port, cmd);

    return error;
}