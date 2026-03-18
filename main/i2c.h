#include "driver/i2c_types.h"
#include "driver/i2c_master.h"

#define MCP9800_TEMPERATURE_REG		0			// Temperature register address
#define MCP9800_CONFIG_REG			1			// Configureation register address
#define I2C_MASTER_TIMEOUT_MS       1000
#define MCP9800_SENSOR_ADDR         0b1001000        /*!< Address of the MCP9800 sensor */
#define RTC_ADDRESS					0x6F    

extern i2c_master_dev_handle_t MCP9800_handle;
extern i2c_master_dev_handle_t RTC_handle;

void i2c_init();
void config_MCP9800();
esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);
float temperature_calc(uint8_t *data);
//esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len);

