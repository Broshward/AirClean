#include <string.h>
#include <stdio.h>
#include "driver/temperature_sensor.h"
#include "esp_blufi.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "sensor.h"
#include "i2c.h"
#include "tasks.h"

const char *SENSOR_TAG = "Sensors";

// Counter struct
typedef struct {
    const char* type_prefix; // "T", "L", "H", "P"...
    int current_index;       // Текущий номер (1, 2, 3...)
} type_counter_t;

#define MAX_SENSORS 32 

// Это наш живой реестр в оперативной памяти
static sensor_t all_sensors[MAX_SENSORS]; 
static int sensor_count = 0;

void sensor_set_value(int id, value_type_t v_type, const char* name, const char* label, sensor_data_t new_val) 
{
    // 1. Сначала ищем: может такой датчик уже есть?
    for (int i = 0; i < sensor_count; i++) {
        if (all_sensors[i].id == id) {
            all_sensors[i].value = new_val; // Обновляем значение (union копируется целиком)
            return; 
        }
    }

    // 2. Если не нашли — АВТОРЕГИСТРАЦИЯ (Plug & Play)
    if (sensor_count < MAX_SENSORS) {
        all_sensors[sensor_count].id = id;
        all_sensors[sensor_count].val_type = v_type;
        all_sensors[sensor_count].type_name = name;
        all_sensors[sensor_count].label = label;
        all_sensors[sensor_count].value = new_val;
        
        sensor_count++;
        
        // Здесь можно отправить во Flutter флаг "Конфиг изменился!", 
        // чтобы приложение перезапросило список карточек.
    }
}

#define MAX_BLUFI_PACKET 128 // Оптимальный размер для стабильной передачи

void send_sensors_values(void) {
    if (sensor_count == 0) return;

    char buf[MAX_BLUFI_PACKET];
    char temp[32]; // Временный буфер для одной записи (ID:VAL;)
    
    snprintf(buf, sizeof(buf), "Values:"); // Наш префикс для значений

    for (int i = 0; i < sensor_count; i++) {
        // Форматируем текущее значение в temp
        if (all_sensors[i].val_type == VAL_TYPE_FLOAT) {
            snprintf(temp, sizeof(temp), "%d:%.1f;", all_sensors[i].id, all_sensors[i].value.f);
        } else if (all_sensors[i].val_type == VAL_TYPE_BOOL) {
            snprintf(temp, sizeof(temp), "%d:%d;", all_sensors[i].id, all_sensors[i].value.b ? 1 : 0);
        } else {
            snprintf(temp, sizeof(temp), "%d:%d;", all_sensors[i].id, all_sensors[i].value.i);
        }

        // ПРОВЕРКА: Поместится ли temp в текущий buf?
        if (strlen(buf) + strlen(temp) >= MAX_BLUFI_PACKET - 1) {
            // Если не влезает — отправляем то, что уже накопили
            esp_blufi_send_custom_data((uint8_t*)buf, strlen(buf));
            
            // Начинаем новый пакет с тем же префиксом
            snprintf(buf, sizeof(buf), "Values:");
        }
        
        strcat(buf, temp);
    }

    // Отправляем остаток (последний пакет)
    if (strlen(buf) > 7) { // 7 — это длина "Values|"
        esp_blufi_send_custom_data((uint8_t*)buf, strlen(buf));
    }
}

// Отправляет типы датчиков() в приложение для печати карточек
void send_sensors()
{
    char temp_pce[32];
	char out_buffer[sizeof(temp_pce)*sensor_count];
    strcpy(out_buffer, "Sensors:"); 
	temp_pce[0]='\0';

    for (int i = 0; i < sensor_count; i++) {
		sprintf(temp_pce, "%d:%d:%s:%s;", i, all_sensors[i].val_type,all_sensors[i].type_name,all_sensors[i].label);
		strcat(out_buffer,temp_pce);
	}
	esp_blufi_send_custom_data((uint8_t *)out_buffer, strlen(out_buffer));
}

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
		ESP_LOGI(SENSOR_TAG, "RSSI: %d dBm", ap_info.rssi);
	} else if (res == ESP_ERR_WIFI_NOT_CONNECT) {
		ESP_LOGE(SENSOR_TAG, "Ошибка: Устройство не подключено к AP\n");
	} else {
		ESP_LOGE(SENSOR_TAG, "Ошибка получения данных: %d\n", res);
	}
	sprintf(data+strlen(data),"#AP:"MACSTR"#%d\n", MAC2STR(ap_info.bssid),ap_info.rssi);

//Add temperature sensor info
	sprintf(data+strlen(data),"#T1#%.2f#MCP9800", temperature_calc(gl_temperature).f);
	sprintf(data+strlen(data),"\n");
//Add temperature sensor info
	sprintf(data+strlen(data),"#L1#%.2f#APDS-9007", gl_luminosity);
	sprintf(data+strlen(data),"\n");
// End of package
	sprintf(data+strlen(data),"##");
}

#define TEMPERATURE_RESOLUTION		0.0625 
sensor_data_t temperature_calc(uint8_t *data)
{
	sensor_data_t temp;
	temp.f = ((int8_t)data[0]) + (data[1]>>4)*TEMPERATURE_RESOLUTION;
	return temp;
}

