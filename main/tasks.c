#include "tasks.h"
#include <math.h>



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
#include "ping/ping_sock.h"
#include "esp_check.h"
#include "esp_netif.h"
#include "driver/gpio.h"

const static char *PING_TAG = "PING:";
bool gl_ping=0;
char gl_narodmon_addr[44];   // Заменить на максимальную длину строки IP-адреса

static void cmd_ping_on_ping_success(esp_ping_handle_t hdl, void *args)
{
    uint8_t ttl;
    uint16_t seqno;
    uint32_t elapsed_time, recv_len;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TTL, &ttl, sizeof(ttl));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_SIZE, &recv_len, sizeof(recv_len));
    esp_ping_get_profile(hdl, ESP_PING_PROF_TIMEGAP, &elapsed_time, sizeof(elapsed_time));
    ESP_LOGI(PING_TAG,"%" PRIu32 " bytes from %s icmp_seq=%" PRIu16 " ttl=%" PRIu16 " time=%" PRIu32 " ms",
           recv_len, ipaddr_ntoa((ip_addr_t*)&target_addr), seqno, ttl, elapsed_time);
	strcpy(gl_narodmon_addr, ipaddr_ntoa((ip_addr_t*)&target_addr));
}

static void cmd_ping_on_ping_timeout(esp_ping_handle_t hdl, void *args)
{
    uint16_t seqno;
    ip_addr_t target_addr;
    esp_ping_get_profile(hdl, ESP_PING_PROF_SEQNO, &seqno, sizeof(seqno));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    printf("From %s icmp_seq=%d timeout\n",ipaddr_ntoa((ip_addr_t*)&target_addr), seqno);
}

static void cmd_ping_on_ping_end(esp_ping_handle_t hdl, void *args)
{
    ip_addr_t target_addr;
    uint32_t transmitted;
    uint32_t received;
    uint32_t total_time_ms;
    uint32_t loss;

    esp_ping_get_profile(hdl, ESP_PING_PROF_REQUEST, &transmitted, sizeof(transmitted));
    esp_ping_get_profile(hdl, ESP_PING_PROF_REPLY, &received, sizeof(received));
    esp_ping_get_profile(hdl, ESP_PING_PROF_IPADDR, &target_addr, sizeof(target_addr));
    esp_ping_get_profile(hdl, ESP_PING_PROF_DURATION, &total_time_ms, sizeof(total_time_ms));

    if (transmitted > 0) {
        loss = (uint32_t)((1 - ((float)received) / transmitted) * 100);
    } else {
        loss = 0;
    }
//#ifdef CONFIG_LWIP_IPV4
//    if (IP_IS_V4(&target_addr)) {
//        printf("\n--- %s ping statistics ---\n", inet_ntoa(*ip_2_ip4(&target_addr)));
//    }
//#endif
//#ifdef CONFIG_LWIP_IPV6
//    if (IP_IS_V6(&target_addr)) {
//        printf("\n--- %s ping statistics ---\n", inet6_ntoa(*ip_2_ip6(&target_addr)));
//    }
//#endif
    ESP_LOGI(PING_TAG, "%" PRIu32 " packets transmitted, %" PRIu32 " received, %" PRIu32 "%% packet loss, time %" PRIu32 "ms\n",
           transmitted, received, loss, total_time_ms);
#define BLINK_GPIO 8
	if(received==0) {
		gl_ping=false;
		gpio_set_level(BLINK_GPIO, 1);
	}
	else {
		gl_ping=true;
		gpio_set_level(BLINK_GPIO, 0);
	}
    // delete the ping sessions, so that we clean up all resources and can create a new ping session
    // we don't have to call delete function in the callback, instead we can call delete function from other tasks
    esp_ping_delete_session(hdl);
}

static int do_ping_cmd(char *addr)
{
    esp_ping_config_t config = ESP_PING_DEFAULT_CONFIG();
	config.count = 1;
	config.timeout_ms = 1000; // Ping timeout

    // parse IP address
    ip_addr_t target_addr;
    memset(&target_addr, 0, sizeof(target_addr));

//    if (inet_pton(AF_INET6, ping_args.host->sval[0], &sock_addr6.sin6_addr) == 1) {
//        /* convert ip6 string to ip6 address */
//        ipaddr_aton(ping_args.host->sval[0], &target_addr);
//    } else {
        struct addrinfo hint;
        struct addrinfo *res = NULL;
        memset(&hint, 0, sizeof(hint));
hint.ai_family = AF_UNSPEC; // Важно: разрешить оба протокола
hint.ai_socktype = SOCK_RAW; // Для пинга лучше использовать RAW или ANY
hint.ai_flags = AI_ADDRCONFIG; // Использовать только те протоколы, которые настроены на интерфейсах

        /* convert ip4 string or hostname to ip4 or ip6 address */
        if (getaddrinfo(addr, NULL, &hint, &res) != 0) {
            printf("ping: unknown host %s\n", addr);
			gl_ping=false;
            return 1;
        }
#ifdef CONFIG_LWIP_IPV4
        if (res->ai_family == AF_INET) {
            struct in_addr addr4 = ((struct sockaddr_in *) (res->ai_addr))->sin_addr;
            inet_addr_to_ip4addr(ip_2_ip4(&target_addr), &addr4);
        }
#endif
#ifdef CONFIG_LWIP_IPV6
        if (res->ai_family == AF_INET6) {
            struct in6_addr addr6 = ((struct sockaddr_in6 *) (res->ai_addr))->sin6_addr;
            inet6_addr_to_ip6addr(ip_2_ip6(&target_addr), &addr6);
        }
#endif
        freeaddrinfo(res);
    //}
    config.target_addr = target_addr;

    /* set callback functions */
    esp_ping_callbacks_t cbs = {
        .cb_args = NULL,
        .on_ping_success = cmd_ping_on_ping_success,
        .on_ping_timeout = cmd_ping_on_ping_timeout,
        .on_ping_end = cmd_ping_on_ping_end
    };
    esp_ping_handle_t ping;
    ESP_RETURN_ON_FALSE(esp_ping_new_session(&config, &cbs, &ping) == ESP_OK, -1, PING_TAG, "esp_ping_new_session failed");
    ESP_RETURN_ON_FALSE(esp_ping_start(ping) == ESP_OK, -1, PING_TAG, "esp_ping_start() failed");
    return 0;
}

#define CONFIG_PING_PERIOD 1000*10 // 10 sec
void configure_led(void)
{
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 1);
}

#include "oneshot_read_adc_main.c"
/*ADC temperature sensor Task*/
#define CONFIG_LIGHT_ADC_PERIOD 1000*10
float gl_luminosity;

void LightTask(void *pvParameters)
{
	adc_config();
    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN0, &adc_raw[0][0]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN0, adc_raw[0][0]);
        if (do_calibration1_chan0) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan0_handle, adc_raw[0][0], &voltage[0][0]));
			gl_luminosity = pow(10,((float)voltage[0][0]-250.0)/500); // 10**((V-Vdark)/S) V,Vdark[mV], S [V/decade] 
            ESP_LOGI(ADC_TAG, "ADC Voltage = %d mV, Luminosity = %.2f Lux", voltage[0][0], gl_luminosity);
        }
        vTaskDelay(pdMS_TO_TICKS(CONFIG_LIGHT_ADC_PERIOD));

        //ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, EXAMPLE_ADC1_CHAN1, &adc_raw[0][1]));
        //ESP_LOGI(TAG, "ADC%d Channel[%d] Raw Data: %d", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, adc_raw[0][1]);
        //if (do_calibration1_chan1) {
        //    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan1_handle, adc_raw[0][1], &voltage[0][1]));
        //    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, EXAMPLE_ADC1_CHAN1, voltage[0][1]);
        //}
        //vTaskDelay(pdMS_TO_TICKS(1000));
    }
    //Tear Down
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    if (do_calibration1_chan0) {
        example_adc_calibration_deinit(adc1_cali_chan0_handle);
    }
    //if (do_calibration1_chan1) {
    //    example_adc_calibration_deinit(adc1_cali_chan1_handle);
    //}
}



#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

#include "i2c_MCP9800.h"


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
    //ESP_LOGI(I2C_TAG, "Temperature = %X %X", data[0], data[1]);
    //ESP_LOGI(I2C_TAG, "Temperature = %f", temp); 
	return temp;
}
#define CONFIG_TEMP_PERIOD 10000 //10 sec
void TempTask(void *pvParameters)
{
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(I2C_TAG, "I2C initialized successfully");

	config_MCP9800();
	//Read configuration
    register_read(dev_handle, MCP9800_CONFIG_REG, gl_temperature, 1);
    ESP_LOGI(I2C_TAG, "Configuration register = 0x%X", gl_temperature[0]);

	while(1){
		register_read(dev_handle, MCP9800_TEMPERATURE_REG, gl_temperature, 2);
		ESP_LOGI(I2C_TAG, "Temperature = %f", temperature_calc(gl_temperature));

        vTaskDelay(pdMS_TO_TICKS(CONFIG_TEMP_PERIOD));
	}    /* Demonstrate writing by resetting the MPU9250 */

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(I2C_TAG, "I2C de-initialized successfully");
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

//#define HOST_IP_ADDR	"fd01::568d:5aff:fed3:c363"
#define HOST_IP_ADDR "192.168.1.43"						// Debug IP-address
#define PORT 8283			// narodmon.com TCP-port address
#define TIME_PERIOD 1000*60 // Perod between sends
#define TIME_PERIOD_CONN 1000*1 // Period between reconnects

static const char *TCP_TAG = "TCP_IP";

void tcp_clientTask(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR; //gl_narodmon_addr;
    int addr_family = 0;
    int ip_protocol = 0;

// Device MAC getting
	uint8_t mac[6];
	esp_base_mac_addr_get(mac);
// String for sending to server
	char  data[1024];
	int sock=-1;
    while (1) {
		do {
			do_ping_cmd("narodmon.com");
#define PING_PERIOD 1000*2
			vTaskDelay(pdMS_TO_TICKS(PING_PERIOD));
		} while(!gl_ping);
		printf("!!!!!!!!!!!!!!!!! ping: %d\n",gl_ping);

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

        //while (1) {
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
	// Send to server
            //int 
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
            }
//xTaskNotifyGive(tcptask);
//ulTaskNotifyTake(true, portMAX_DELAY);
			printf("----------------------------------------------------------------------\n");
        //}

			vTaskDelay(pdMS_TO_TICKS(TIME_PERIOD));
    }
}
