#include "tasks.h"
#include "blufi.h"
#include "sntp.h"
#include "ping.h"
#include <math.h>
#include "esp_attr.h"
#include "esp_blufi_api.h"
#include "esp_sntp.h"



#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
//#include "esp_console.h"
#include "esp_event.h"
#include "nvs_flash.h"
//#include "argtable3/argtable3.h"
//#include "protocol_examples_common.h"




#include "oneshot_read_adc_main.c"
/*ADC temperature sensor Task*/
#define CONFIG_LIGHT_ADC_PERIOD 1000*1			//Период измерения
#define CONFIG_LIGHT_TRANSMIT_PERIOD 1000*10	//Период передачи показаний
float gl_luminosity;

void LightTask(void *pvParameters)
{
	adc_config();
	// Переменная для хранения предыдущего значения
	static int filtered_value;
	adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &filtered_value); //Начальное значение 
	const int coeff = 4; // Коэффициент фильтрации
	int count=0;
    while (1) {
        if (do_calibration1_chan0) {

			int raw_value;
			int voltage;
			adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &raw_value);

			// Применяем фильтр: y = (y * (k-1) + x) / k 
			filtered_value = (filtered_value * (coeff - 1) + raw_value) / coeff;

            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, filtered_value, &voltage));

			gl_luminosity = pow(10,((float)voltage-250.0)/380); // 10**((V-Vdark)/S) V,Vdark[mV], S [V/decade] 
            //ESP_LOGI(ADC_TAG, "ADC Voltage = %d mV, Luminosity = %.2f Lux", voltage[0][0], gl_luminosity);
			
			if (count == CONFIG_LIGHT_TRANSMIT_PERIOD/CONFIG_LIGHT_ADC_PERIOD){
				// Отправка через BluFi
				char payload[40];
				snprintf(payload, sizeof(payload), "Lumin:%.2f", gl_luminosity);
				esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
				count=0;
			}
			count++;
        }

        //ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[0][1]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, adc_raw[0][1]);
        //if (do_calibration1_chan1) {
        //    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
        //    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, voltage[0][1]);
        //}
        //vTaskDelay(pdMS_TO_TICKS(1000));
        vTaskDelay(pdMS_TO_TICKS(CONFIG_LIGHT_ADC_PERIOD));
    }
}



#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"



#define I2C_MASTER_SCL_IO           7 //CONFIG_I2C_MASTER_SCL       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           10  //CONFIG_I2C_MASTER_SDA       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          100000 //CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MCP9800_SENSOR_ADDR         0b1001000        /*!< Address of the MCP9800 sensor */
#define MCP9800_TEMPERATURE_REG		0			// Temperature register address
#define MCP9800_CONFIG_REG			1			// Configureation register address

i2c_master_dev_handle_t dev_handle;
i2c_master_bus_handle_t bus_handle;
static const char *I2C_TAG = "i2c";
uint8_t gl_temperature[2];

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/**
 * @brief i2c master initialization
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
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

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MCP9800_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));

}

#define CONFIG_REGISTER_VALUE		(0b11<<5)
#define TEMPERATURE_RESOLUTION		0.0625 
void config_MCP9800()
{
	//Write configuration
    register_write_byte(dev_handle, MCP9800_CONFIG_REG, CONFIG_REGISTER_VALUE); //12-bit resolution
}
float temperature_calc(uint8_t *data)
{
	float temp = ((int8_t)data[0]) + (data[1]>>4)*TEMPERATURE_RESOLUTION;
	return temp;
}
#define CONFIG_TEMP_PERIOD 10000 //10 sec
void I2C_Task(void *pvParameters)
{
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(I2C_TAG, "I2C initialized successfully");

	config_MCP9800();
	//Read configuration
    register_read(dev_handle, MCP9800_CONFIG_REG, gl_temperature, 1);
    ESP_LOGI(I2C_TAG, "Configuration register = 0x%X", gl_temperature[0]);

	while(1){
		register_read(dev_handle, MCP9800_TEMPERATURE_REG, gl_temperature, 2);
		//ESP_LOGI(I2C_TAG, "Temperature = %.2f", temperature_calc(gl_temperature));

		// Отправка через BluFi
		char payload[40];
		snprintf(payload, sizeof(payload), "Amb_Temp:%.2f", temperature_calc(gl_temperature));
		esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
			
        vTaskDelay(pdMS_TO_TICKS(CONFIG_TEMP_PERIOD));
	}    /* Demonstrate writing by resetting the MPU9250 */

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(I2C_TAG, "I2C de-initialized successfully");
}

#include "driver/temperature_sensor.h"
void Temp_sensor_Task(void * pvParameters)
{
	temperature_sensor_handle_t temp_handle = NULL;
	temperature_sensor_config_t temp_sensor_config = TEMPERATURE_SENSOR_CONFIG_DEFAULT(20, 50);
	ESP_ERROR_CHECK(temperature_sensor_install(&temp_sensor_config, &temp_handle));
	while(1){
		// Enable temperature sensor
		ESP_ERROR_CHECK(temperature_sensor_enable(temp_handle));
		// Get converted sensor data
		float tsens_out;
		ESP_ERROR_CHECK(temperature_sensor_get_celsius(temp_handle, &tsens_out));
		// Disable the temperature sensor if it is not needed and save the power
		ESP_ERROR_CHECK(temperature_sensor_disable(temp_handle));

		// Отправка через BluFi
		char payload[20];
		snprintf(payload, sizeof(payload), "Chip_Temp:%.2f", tsens_out);
		esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
			
#define TEMP_PERIOD 1000*10
		vTaskDelay(pdMS_TO_TICKS(TEMP_PERIOD));
	}
}

/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include "sdkconfig.h"
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h>            // struct addrinfo
#include <arpa/inet.h>
#include "esp_netif.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#if defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
#include "addr_from_stdin.h"
#endif

static const char *TCP_TAG = "TCP_IP";

void create_data(char *data)
{
// Device MAC getting
	uint8_t mac[6];
	esp_base_mac_addr_get(mac);
// Add sensors MAC, NAME info
	sprintf(data,"#"MACSTR, MAC2STR(mac));
	sprintf(data+strlen(data),"#AirClean"); // The NAME field
	sprintf(data+strlen(data),"\n");
// Add BSSID and RSSI information
	wifi_ap_record_t ap_info;
	esp_err_t res = esp_wifi_sta_get_ap_info(&ap_info);
	if (res == ESP_OK) {
		ESP_LOGI(TCP_TAG, "RSSI: %d dBm", ap_info.rssi);
	} else if (res == ESP_ERR_WIFI_NOT_CONNECT) {
		ESP_LOGE(TCP_TAG, "Ошибка: Устройство не подключено к AP\n");
	} else {
		ESP_LOGE(TCP_TAG, "Ошибка получения данных: %d\n", res);
	}
	sprintf(data+strlen(data),"#AP:"MACSTR"#%d", MAC2STR(ap_info.bssid),ap_info.rssi);
	sprintf(data+strlen(data),"\n");
//Add temperature sensor info
	sprintf(data+strlen(data),"#T1#%.2f#MCP9800", temperature_calc(gl_temperature));
	sprintf(data+strlen(data),"\n");
//Add temperature sensor info
	sprintf(data+strlen(data),"#L1#%.2f#APDS-9007", gl_luminosity);
	sprintf(data+strlen(data),"\n");
// End of package
	sprintf(data+strlen(data),"##");
}

//#define HOST_IP_ADDR	"fd01::568d:5aff:fed3:c363"
#define HOST_IP_ADDR "192.168.1.75"						// Debug IP-address
#define PORT 8283			// narodmon.com TCP-port address
#define TIME_PERIOD 1000*60 // Perod between sends
#define TIME_PERIOD_CONN 1000*1 // Period between reconnects
RTC_DATA_ATTR time_t gl_last_send_time=0; // Last time in RTC SRAM part

void tcp_clientTask(void *pvParameters)
{
    configure_led();
	ulTaskNotifyTake(true, portMAX_DELAY); // Wait for time syncing
	gl_last_send_time = load_last_send_time(); //Load from nvs last sending time
	printf("Last sending time from after reset ESP32 is %d\n", (int)gl_last_send_time);

    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR; //gl_narodmon_addr;
    int addr_family = 0;
    int ip_protocol = 0;

// String for sending to server
	int sock=-1;
    while (1) {
		time_t now;
		time(&now);
		printf("Time NOW: %d\n", (int)now);
		printf("Time of last send: %d\n", (int)gl_last_send_time);
		if (now-gl_last_send_time < TIME_PERIOD/1000)
			vTaskDelay(pdMS_TO_TICKS(TIME_PERIOD-1000*(now-gl_last_send_time))); //
		do { 
			do_ping_cmd(HOST_IP_ADDR);
#define PING_PERIOD 1000*2
			vTaskDelay(pdMS_TO_TICKS(PING_PERIOD));
		} while(!gl_ping); //Ждём пинга от сервера

        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        if (sock != -1) {
            ESP_LOGE(TCP_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
		sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d", errno);
            continue;
        }
        ESP_LOGI(TCP_TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TCP_TAG, "Socket unable to connect: errno %d", errno);
            continue;
        }
        ESP_LOGI(TCP_TAG, "Successfully connected");

		char  data[4096];
		create_data(data);
		ESP_LOGI(TCP_TAG, "Data length = %d\n",strlen(data));

	// Send to server
			err = send(sock, data, strlen(data), 0);
            if (err < 0) {
                ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
				continue;
            }
// Recieve part for server ansvers
#define TIMEOUT 1000*1 
			vTaskDelay(pdMS_TO_TICKS(TIMEOUT)); // Timeout for server reply
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TCP_TAG, "recv failed: errno %d", errno);
				continue;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                //ESP_LOGI(TCP_TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TCP_TAG, "Answer: %s", rx_buffer);
				if (strcmp(rx_buffer,"OK")) 
					while(1){
						ESP_LOGE(TCP_TAG, "Server return error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: '%s' ", rx_buffer);
						vTaskDelay(pdMS_TO_TICKS(1000*100));  // If server return error need to debug
					}
				else{
					time(&gl_last_send_time);  //Set last send to server time
					save_last_send_time(gl_last_send_time); // Write last send time to flash
				}
				
            }
			printf("----------------------------------------------------------------------\n");

			vTaskDelay(pdMS_TO_TICKS(TIME_PERIOD));
    }
}

#define TIME_SEND_TIME 1000*10 //3600
void timeTask(void *pvParameters)
{
	bool tcp_notify_given=false; //Эта переменная нужна, чтобы задача не надоедала уведомлениями
	while(1){
		time_t now;
		struct tm timeinfo;
		time(&now);
		localtime_r(&now, &timeinfo);
		char time_str[10];
		strftime(time_str, sizeof(time_str), "%H_%M_%S", &timeinfo);
	
		char payload[15];
		if (timeinfo.tm_year > 2025-1900) {
			snprintf(payload, sizeof(payload), "Time:%s", time_str);
			esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
			if (tcp_notify_given==false){
				xTaskNotifyGive(tcptask);
				tcp_notify_given=true;
			}
		}
		else {
			snprintf(payload, sizeof(payload), "Time:Not sync");
			esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
			ESP_LOGI("Time", "Resync time");
    	    esp_sntp_stop();
    	    esp_sntp_init(); 
		}
		vTaskDelay(pdMS_TO_TICKS(TIME_SEND_TIME));
	}
}
