#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include <time.h>
#include <sys/time.h>

#include "i2c.h"

// Вспомогательные функции BCD
static uint8_t bcd2dec(uint8_t val) { return ((val >> 4) * 10) + (val & 0x0f); }
static uint8_t dec2bcd(uint8_t val) { return ((val / 10) << 4) | (val % 10); }

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

    time_t t = mktime(&tm);
    struct timeval now = { .tv_sec = t };
    settimeofday(&now, NULL);
	time(&t);
    ESP_LOGI("RTC", "Время установлено из MCP79410");
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
