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

#define HOST_IP_ADDR "192.168.1.43"						// Debug IP-address
#define PORT 3344	// Test port for debug
#define TIME_PERIOD 10000*1 // Perod between sends
#define TIME_PERIOD_CONN 1000*1 // Period between reconnects

static const char *TCP_TAG = "TCP_IP";

void tcp_clientTask(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

// Device MAC getting
	uint8_t mac[6];
	esp_base_mac_addr_get(mac);
// String for sending to server
	char  data[1024];
    while (1) {
//#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
//#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
//        struct sockaddr_storage dest_addr = { 0 };
//        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_STREAM, &ip_protocol, &addr_family, &dest_addr));
//#endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TCP_TAG, "Unable to create socket: errno %d", errno);
//            break;
        }
        ESP_LOGI(TCP_TAG, "Socket created, connecting to %s:%d", host_ip, PORT);

        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TCP_TAG, "Socket unable to connect: errno %d", errno);
//            break;
        }
        ESP_LOGI(TCP_TAG, "Successfully connected");

        while (1) {
// Add sensors information
		sprintf(data,"#"MACSTR, MAC2STR(mac));
		sprintf(data+strlen(data),"#AirClean"); // The NAME field
// Add BSSID and RSSI information
		wifi_ap_record_t ap_info;
		esp_err_t res = esp_wifi_sta_get_ap_info(&ap_info);
		if (res == ESP_OK) {
			ESP_LOGI(TCP_TAG, "RSSI: %d dBm\n", ap_info.rssi);
		} else if (res == ESP_ERR_WIFI_NOT_CONNECT) {
			ESP_LOGE(TCP_TAG, "Ошибка: Устройство не подключено к AP\n");
		} else {
			ESP_LOGE(TCP_TAG, "Ошибка получения данных: %d\n", res);
		}
		sprintf(data+strlen(data),"#AP:"MACSTR"#%d", MAC2STR(ap_info.bssid),ap_info.rssi);
		sprintf(data+strlen(data),"\n");
// Send to server
            int err = send(sock, data, strlen(data), 0);
            if (err < 0) {
                ESP_LOGE(TCP_TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
// Recieve part for server ansvers
            //int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            //// Error occurred during receiving
            //if (len < 0) {
            //    ESP_LOGE(TAG, "recv failed: errno %d", errno);
            //    break;
            //}
            //// Data received
            //else {
            //    rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
            //    ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
            //    ESP_LOGI(TAG, "%s", rx_buffer);
            //}
			vTaskDelay(pdMS_TO_TICKS(TIME_PERIOD));
        }

        if (sock != -1) {
            ESP_LOGE(TCP_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
		vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
