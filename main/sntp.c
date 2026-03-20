#include <time.h>
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

#include "sntp.h"
#include "rtc.h"
#include "blufi.h"

time_t last_sync_sntp=0;

void save_last_sync_sntp(uint32_t timestamp) 
{
    nvs_handle_t my_handle;
    // Открываем хранилище с именем "storage"
    if (nvs_open("storage", NVS_READWRITE, &my_handle) == ESP_OK) {
        nvs_set_u32(my_handle, "last_sync", timestamp);
        nvs_commit(my_handle); // Обязательно фиксируем изменения
        nvs_close(my_handle);
    }
}

uint32_t load_last_sync_sntp() 
{
    nvs_handle_t my_handle;
    uint32_t timestamp = 0;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        nvs_get_u32(my_handle, "last_sync", &timestamp);
        nvs_close(my_handle);
    }
    return timestamp;
}

// Колбэк функция, которая сработает при успехе
void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI("SNTP", "Синхронизация с сервером времени прошла успешно!");
    
	time(&last_sync_sntp);
	save_last_sync_sntp(last_sync_sntp); //Сохраняем время синхронизации
	send_last_sync_sntp();
    // Как только время стало актуальным — сразу записываем его в наши часы RTC
    system_time_to_rtc(); 
}

void sntp_init_and_sync() 
{
    ESP_LOGI("SNTP", "Инициализация SNTP...");
	last_sync_sntp = load_last_sync_sntp(); // Загружаем время последней синхронизации
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org"); // Можно добавить несколько серверов
	
    // Регистрируем наш колбэк
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    esp_sntp_init();
}

void resync_time()
{
	esp_sntp_stop();
	esp_sntp_init(); 
}

void send_last_sync_sntp()
{
	struct tm timeinfo;
	localtime_r(&last_sync_sntp, &timeinfo);
	char time_str[20];
	strftime(time_str, sizeof(time_str), "%H_%M_%S %d-%m-%Y", &timeinfo);
	char payload[35];
	snprintf(payload, sizeof(payload), "Time_sync_sntp:%s", time_str);
	esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
}
