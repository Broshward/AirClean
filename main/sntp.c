#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_sntp.h"

#include "sntp.h"
#include "rtc.h"

// Сама функция, которая сработает при успехе
void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI("SNTP", "Синхронизация с сервером времени прошла успешно!");
    
    // Как только время стало актуальным — сразу записываем его в наши часы RTC
    system_time_to_rtc(); 
}

void sntp_init_and_sync() 
{
    ESP_LOGI("SNTP", "Инициализация SNTP...");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org"); // Можно добавить несколько серверов
	
    // Регистрируем наш колбэк
    esp_sntp_set_time_sync_notification_cb(time_sync_notification_cb);
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

void resync_time()
{
	esp_sntp_stop();
	esp_sntp_init(); 
}
