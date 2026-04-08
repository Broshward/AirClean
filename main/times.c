#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "nvs_flash.h"

#include "times.h"
#include "blufi.h"
#include "i2c.h"

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
	snprintf(payload, sizeof(payload), "TZ:%d", (int)load_tz_from_nvs());
	esp_blufi_send_custom_data((uint8_t *)payload, strlen(payload));
}

// Вспомогательные функции BCD
static uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0f); }
static uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

// Функция нужна для DS1307
void rtc_init_DS1307()
{
	uint8_t data;
	i2c_register_read(RTC_handle, 0, &data, 1);
	i2c_register_write_byte(RTC_handle, 0, data&0x7f);
	//i2c_register_write_byte(RTC_handle, 0, data);
}

// Чтение времени из RTC и установка в систему
void rtc_to_system_time() 
{
    uint8_t data[7]={0,0,0,0,0,0,0};
	i2c_register_read(RTC_handle, 0, data, 7);

    struct tm tm;
    tm.tm_sec = bcd2dec(data[0] & 0x7F);
    tm.tm_min = bcd2dec(data[1]);
    tm.tm_hour = bcd2dec(data[2] & 0x3F);
    tm.tm_mday = bcd2dec(data[4] & 0x3F);
    tm.tm_mon = bcd2dec(data[5] & 0x1F) - 1;
    tm.tm_year = bcd2dec(data[6]) + 100; // 2000 + год

	// Установка времени
    time_t t = mktime(&tm);
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, NULL);
    ESP_LOGI("RTC", "Время установлено из MCP79410");
	//printf("SEC=%x\n",data[0]);
}

// Запись системного (NTP) времени в RTC
void system_time_to_rtc() 
{
    time_t now;
    struct tm tm;
    time(&now);
    localtime_r(&now, &tm);
    uint8_t data[8] = {
        0x00, // Начальный регистр(адрес)
        dec2bcd(tm.tm_sec) | 0x80, // Бит ST (Start Oscillator) = 1
        dec2bcd(tm.tm_min),
        dec2bcd(tm.tm_hour),
        0x08, // Day + VBATEN = 1 (включить батарейку)
        dec2bcd(tm.tm_mday),
        dec2bcd(tm.tm_mon + 1),
        dec2bcd(tm.tm_year - 100)
    };
	i2c_buffer_write(RTC_handle, data, 8);

    ESP_LOGI("RTC", "Время синхронизировано в MCP79410");
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
        nvs_set_u32(my_handle, "last_send", timestamp);
        nvs_commit(my_handle); // Обязательно фиксируем изменения
        nvs_close(my_handle);
    }
}

uint32_t load_last_send_time() 
{
    nvs_handle_t my_handle;
    uint32_t timestamp = 0;
    if (nvs_open("storage", NVS_READONLY, &my_handle) == ESP_OK) {
        nvs_get_u32(my_handle, "last_send", &timestamp);
        nvs_close(my_handle);
    }
    return timestamp;
}

void set_timezone(int offset) 
{
    char tz_string[16];
    // Формат POSIX: UTC-3 означает "на 3 часа ВПЕРЕД от Гринвича" (да, знаки там инвертированы)
    // Но проще использовать формат: <ANY_NAME><-OFFSET>
    // Если offset = 3 (Москва), то TZ будет "MSK-3"
    snprintf(tz_string, sizeof(tz_string), "UTC%s%d", offset >= 0 ? "-" : "+", abs(offset));
    
    setenv("TZ", tz_string, 1);
    tzset();
    
    ESP_LOGI("TIME", "Часовой пояс установлен: %s", tz_string);
}

void save_tz_to_nvs(int8_t offset) 
{
    nvs_handle_t h;
    if (nvs_open("storage", NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_i8(h, "tz_offset", offset);
        nvs_commit(h);
        nvs_close(h);
    }
}

int8_t load_tz_from_nvs() 
{
    nvs_handle_t h;
    int8_t offset = 3; // По умолчанию Москва
    if (nvs_open("storage", NVS_READONLY, &h) == ESP_OK) {
        nvs_get_i8(h, "tz_offset", &offset);
        nvs_close(h);
    }
    return offset;
}

