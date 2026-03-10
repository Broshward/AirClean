/* LwIP SNTP example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
//#include "protocol_examples_common.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "sntp.h"

void sntp_init_and_sync() 
{
    ESP_LOGI("SNTP", "Инициализация SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org"); // Можно добавить несколько серверов
    esp_sntp_init();

    // Установка часового пояса (например, Москва UTC+3)
    setenv("TZ", "MSK-3", 1);
    tzset();
}

// Функция для получения текущего времени в строку
void get_current_time_str(char* buf, size_t size) 
{
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    strftime(buf, size, "%H:%M:%S", &timeinfo);
}

void save_last_send_time(uint32_t timestamp) 
{
    nvs_handle_t my_handle;
    // Открываем хранилище с именем "storage"
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u32(my_handle, "last_sync", timestamp);
        nvs_commit(my_handle); // Обязательно фиксируем изменения
        nvs_close(my_handle);
        ESP_LOGI("NVS", "Время сохранения: %lu", timestamp);
    }
}

uint32_t load_last_send_time() 
{
    nvs_handle_t my_handle;
    uint32_t timestamp = 0;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        nvs_get_u32(my_handle, "last_sync", &timestamp);
        nvs_close(my_handle);
    }
    return timestamp;
}
