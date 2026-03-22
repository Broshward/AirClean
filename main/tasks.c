#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h>            // struct addrinfo
#include <arpa/inet.h>
#include "esp_mac.h"
#include "esp_wifi.h"
#include "driver/temperature_sensor.h"
#include "driver/gpio.h"

#include "tasks.h"
#include "blufi.h"
#include "times.h"
#include "ping.h"
#include "i2c.h"
#include "oneshot_read_adc_main.c"
#include "spi.h"
#include "ota.h"

uint8_t gl_temperature[2];


/*ADC Light sensor Task*/
#define CONFIG_LIGHT_ADC_PERIOD 10			//Период измерения
#define CONFIG_LIGHT_TRANSMIT_PERIOD 1000*5	//Период передачи показаний
float gl_luminosity;

void LightTask(void *pvParameters)
{
	adc_config();
	// Переменная для хранения предыдущего значения
	static int filtered_value;
	adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &filtered_value); //Начальное значение 
	const int coeff = 16; // Коэффициент фильтрации
	int count=0;
    while (1) {
        if (do_calibration1_chan0) {

			int raw_value;
			int voltage;
			adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &raw_value);

			// Применяем фильтр: y = (y * (k-1) + x) / k 
			filtered_value = (filtered_value * (coeff - 1) + raw_value) / coeff;

            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, filtered_value, &voltage));

			gl_luminosity = pow(10,((float)voltage-255.0)/380); // 10**((V-Vdark)/S) V,Vdark[mV], S [V/decade] 
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

#define CONFIG_TEMP_PERIOD 10000 //10 sec
void I2C_Task(void *pvParameters)
{
	config_MCP9800();
	//Read configuration
    i2c_register_read(MCP9800_handle, MCP9800_CONFIG_REG, gl_temperature, 1);
    ESP_LOGI("i2c", "Configuration register = 0x%X", gl_temperature[0]);

	while(1){
		i2c_register_read(MCP9800_handle, MCP9800_TEMPERATURE_REG, gl_temperature, 2);
		//ESP_LOGI(I2C_TAG, "Temperature = %.2f", temperature_calc(gl_temperature));

		// Отправка через BluFi
		char payload[40];
		snprintf(payload, sizeof(payload), "Amb_Temp:%.2f", temperature_calc(gl_temperature));
		esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
			
        vTaskDelay(pdMS_TO_TICKS(CONFIG_TEMP_PERIOD));
	}

}

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

void after_success()
{
//	gpio_set_level(BLINK_GPIO, 0); // LED is on
}
void after_failure()
{
//	gpio_set_level(BLINK_GPIO, 1); // LED is off
}

//#define HOST_IP_ADDR	"fd01::568d:5aff:fed3:c363"
#define HOST_IP_ADDR "192.168.43.105"						// Debug IP-address
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
		if (now-gl_last_send_time < TIME_PERIOD/1000) {
			printf("Time to wait : %d second\n", (int)(TIME_PERIOD-1000*(now-gl_last_send_time))/1000);
			vTaskDelay(pdMS_TO_TICKS(TIME_PERIOD-1000*(now-gl_last_send_time))); //
		}
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
			after_failure();
            continue;
        }
        ESP_LOGI(TCP_TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TCP_TAG, "Socket unable to connect: errno %d", errno);
			after_failure();
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
				after_failure();
				continue;
            }

// Recieve part for server ansvers
#define TIMEOUT 1000*1
			vTaskDelay(pdMS_TO_TICKS(TIMEOUT)); // Timeout for server reply
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TCP_TAG, "recv failed: errno %d", errno);
				after_failure();
				continue;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                //ESP_LOGI(TCP_TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TCP_TAG, "Answer: %s", rx_buffer);
				if (strcmp(rx_buffer,"OK")) 
					while(1){
						void after_failure();
						ESP_LOGE(TCP_TAG, "Server return error!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!: '%s' ", rx_buffer);
						vTaskDelay(pdMS_TO_TICKS(1000*100));  // If server return error need to debug
					}
				else{
					time(&gl_last_send_time);  //Set last send to server time
					save_last_send_time(gl_last_send_time); // Write last send time to flash
					after_success();
				}
				
            }
			//printf("----------------------------------------------------------------------\n");
			ESP_LOGI(TCP_TAG, "----------");

			//vTaskDelay(pdMS_TO_TICKS(TIME_PERIOD));
    }
}

#define TIME_TO_TIME 1000*60 //3600
void timeTask(void *pvParameters)
{
	bool tcp_notify_given=false; //Эта переменная нужна, чтобы задача не надоедала уведомлениями
	while(1){
		rtc_to_system_time(); // Синхронизируем с внутренними часами каждые TIME_TO_TIME
		time_t now;
		time(&now);
		printf("Time = %d\n", (int)now);

		char payload[35];
		char timestr[20];
		struct tm timeinfo;
		localtime_r(&now, &timeinfo);
		//Time
		strftime(timestr, sizeof(timestr), "%H_%M_%S", &timeinfo);
		snprintf(payload, sizeof(payload), "Time:%s", timestr);
		esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
		//Date
		strftime(timestr, sizeof(timestr), "%d - %m - %Y", &timeinfo);
		snprintf(payload, sizeof(payload), "Date:%s", timestr);
		esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
		if (timeinfo.tm_year > 2025-1900) {
			if (tcp_notify_given==false){
				xTaskNotifyGive(tcptask);
				tcp_notify_given=true;
			}
		}
		else {
			snprintf(payload, sizeof(payload), "Time_sync_sntp:Not sync");
			esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
			ESP_LOGI("Time", "Resync time");
			resync_time();
		}
		vTaskDelay(pdMS_TO_TICKS(TIME_TO_TIME));
	}
}

void spi_test(void *pvParameters)
{
	init_spi_eeprom();
	int i=0;
	//time_t now;
	while(1){
	//	rtc_to_system_time();
		vTaskDelay(pdMS_TO_TICKS(10000));
		i+=4;
	}
}
