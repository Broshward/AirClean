#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "i2c.h"

#define I2C_MASTER_SCL_IO           9 //CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           2  //CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          10000 //CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */

i2c_master_dev_handle_t MCP9800_handle;
i2c_master_dev_handle_t RTC_handle;
i2c_master_bus_handle_t i2c_handle;

esp_err_t i2c_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_buffer_read(i2c_master_dev_handle_t dev_handle, uint8_t addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_buffer_write(i2c_master_dev_handle_t dev_handle, uint8_t *data, size_t len)
{
    return i2c_master_transmit(dev_handle, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t i2c_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
void i2c_master_init(i2c_master_bus_handle_t *bus_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
}

void i2c_dev_init(const i2c_master_bus_handle_t *bus_handle, const uint8_t dev_addr, i2c_master_dev_handle_t *dev_handle)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev_addr,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));

    ESP_LOGI("i2c", "I2C initialized successfully");

}

void i2c_init()
{
    i2c_master_init(&i2c_handle);
	i2c_dev_init(&i2c_handle, MCP9800_SENSOR_ADDR, &MCP9800_handle);
	i2c_dev_init(&i2c_handle, RTC_ADDRESS, &RTC_handle);
}

#define CONFIG_REGISTER_VALUE		(0b11<<5)
#define TEMPERATURE_RESOLUTION		0.0625 
void config_MCP9800()
{
	//Write configuration
    i2c_register_write_byte(MCP9800_handle, MCP9800_CONFIG_REG, CONFIG_REGISTER_VALUE); //12-bit resolution
}

float temperature_calc(uint8_t *data)
{
	float temp = ((int8_t)data[0]) + (data[1]>>4)*TEMPERATURE_RESOLUTION;
	return temp;
}

