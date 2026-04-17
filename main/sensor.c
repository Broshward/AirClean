#include <string.h>
#include <stdio.h>
#include "driver/temperature_sensor.h"
#include "esp_blufi.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_log.h"

#include "sensor.h"
#include "tasks.h"
#include "blufi.h"
#include "spi.h"
#include "i2c.h"
#include "gatts.h"

const char *SENSOR_TAG = "Sensors";

#define MAX_SENSORS 32 

// Это наш живой реестр в оперативной памяти
static sensor_t all_sensors[MAX_SENSORS]; 
static int sensor_count = 0;

uint32_t prev_tail_addr = 0; // Это нужно, чтобы не перезаписывать одно и тоже значение

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

void send_sensors_values(void) 
{
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
            send_ble_data(buf);
            
            // Начинаем новый пакет с тем же префиксом
            snprintf(buf, sizeof(buf), "Values:");
        }
        
        strcat(buf, temp);
    }

    // Отправляем остаток (последний пакет)
    if (strlen(buf) > 7) { // 7 — это длина "Values|"
        send_ble_data(buf);
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
		sprintf(temp_pce, "%d:%d:%s:%s;", all_sensors[i].id, all_sensors[i].val_type,all_sensors[i].type_name,all_sensors[i].label);
		strcat(out_buffer,temp_pce);
	}
	send_ble_data(out_buffer);
}

void flash_log_all_sensors(uint32_t timestamp) 
{
    uint8_t buf[259]; // Временный буфер для сборки пакета
    uint8_t ptr = 0;

    buf[ptr++] = 0xAA;      // Маркер
    buf[ptr++] = 0;         // Резерв под длину (заполним позже)
    
    // Пишем время (4 байта)
    memcpy(&buf[ptr], &timestamp, 4);
    ptr += 4;

    // Пишем все активные датчики
    for (int i = 0; i < sensor_count; i++) {
		buf[ptr++] = all_sensors[i].id; // 1 байт - ID

		float temp_val;
		
		// Приводим все типы к float для унификации в логе
		switch (all_sensors[i].val_type) {
			case VAL_TYPE_FLOAT:
				temp_val = all_sensors[i].value.f;
				break;
			case VAL_TYPE_INT:
				temp_val = (float)all_sensors[i].value.i;
				break;
			case VAL_TYPE_BOOL:
				temp_val = all_sensors[i].value.b ? 1.0f : 0.0f;
				break;
			default:
				temp_val = 0.0f;
		}

		memcpy(&buf[ptr], &temp_val, 4);
		ptr += 4;
	}

    buf[1] = ptr - 2; // Записываем реальную длину данных (без маркера и самой длины)

    // Считаем CRC8 (простая сумма всех байт для примера)
    uint8_t crc = 0;
    for (int i = 0; i < ptr; i++) crc += buf[i];
    buf[ptr++] = crc;

    // Теперь пишем буфер на флешку по адресу head
     flash_write_data(buf, ptr);
    
    // Обновляем head в eeprom-памяти часов
	save_head_to_eeprom(current_head_addr+ptr);
}

const char* get_label_by_id(int id) 
{
    for (int i = 0; i < sensor_count; i++) {
        if (all_sensors[i].id == id) return all_sensors[i].label;
    }
    return "unknown";
}

const char* get_type_by_id(int id) 
{
    for (int i = 0; i < sensor_count; i++) {
        if (all_sensors[i].id == id) return all_sensors[i].type_name;
    }
    return "U"; // Unknown
}

bool get_one_flash_packet_string(char *out_str, size_t free_space) 
{
    uint8_t packet[259]; 
    uint16_t p_len;
    uint32_t temp_tail = current_tail_addr; // Используем локальный указатель для поиска

    while (temp_tail != current_head_addr) {
        uint8_t header[2];
        flash_read_data(temp_tail, header, 2);

        if (header[0] != 0xAA) {
            temp_tail = (temp_tail + 1) % FLASH_TOTAL_SIZE;
            continue;
        }

        p_len = header[1] + 3; 
        flash_read_data(temp_tail, packet, p_len);

        // Проверка CRC
        uint8_t sum = 0;
        for (int i = 0; i < p_len - 1; i++) sum += packet[i];
        if (sum != packet[p_len - 1]) {
            temp_tail = (temp_tail + 1) % FLASH_TOTAL_SIZE;
            continue;
        }

        // Пакет валиден. Теперь проверяем, влезет ли он в ТЕКСТОВОМ виде
        char temp_str[2048] = ""; // Временная строка для одного пакета
        uint32_t timestamp;
        memcpy(&timestamp, &packet[2], 4);
        
        int ptr = 6;
        while (ptr < p_len - 1) {
            uint8_t id = packet[ptr++];
            float val;
            memcpy(&val, &packet[ptr], 4);
            ptr += 4;

            char line[128];
            snprintf(line, sizeof(line), "#%s%d#%.2f#%lu#%s\n", 
                     get_type_by_id(id), id, val, timestamp, get_label_by_id(id));
            strcat(temp_str, line);
        }

        // ГЛАВНАЯ ПРОВЕРКА:
        if (strlen(temp_str) < free_space) {
            strcpy(out_str, temp_str);
            current_tail_addr = (temp_tail + p_len) % FLASH_TOTAL_SIZE; // Сдвигаем хвост
            return true;
        } else {
            // Не влезает в оставшееся место основного буфера.
            // Хвост НЕ двигаем, выходим.
            return false; 
        }
    }
    return false;
}

void get_narodmon_string(char *data, size_t max_len) 
{
    // 1. Текущие данные
    uint8_t mac[6];
    esp_base_mac_addr_get(mac); 
	//esp_read_mac(mac, ESP_MAC_BT); // Явно просим адрес блютуза	

    // 1. Заголовок: ID устройства (MAC) и имя
    snprintf(data, max_len, "#" MACSTR "#AirClean\n", MAC2STR(mac));

    // 2. Служебная информация: BSSID, RSSI
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
	    snprintf(data + strlen(data), max_len - strlen(data), 
             "#AP:" MACSTR "#%d\n", MAC2STR(ap_info.bssid), ap_info.rssi);
    }

    // 3. Динамический опрос всех датчиков из нашего массива
    for (int i = 0; i < sensor_count; i++) {
        char sensor_line[64];
        
        // Формируем уникальный ключ датчика (например, T1, T2 или по его ID)
        // Народмон любит короткие ключи, поэтому используем type_name + ID
        if (all_sensors[i].val_type == VAL_TYPE_FLOAT) {
            snprintf(sensor_line, sizeof(sensor_line), "#%s%d#%.2f#%s\n", 
                     all_sensors[i].type_name, all_sensors[i].id, 
                     all_sensors[i].value.f, all_sensors[i].label);
        } 
        else if (all_sensors[i].val_type == VAL_TYPE_BOOL) {
            snprintf(sensor_line, sizeof(sensor_line), "#%s%d#%d#%s\n", 
                     all_sensors[i].type_name, all_sensors[i].id, 
                     all_sensors[i].value.b ? 1 : 0, all_sensors[i].label);
        }
        else if (all_sensors[i].val_type == VAL_TYPE_INT) {
            snprintf(sensor_line, sizeof(sensor_line), "#%s%d#%d#%s\n", 
                     all_sensors[i].type_name, all_sensors[i].id, 
                     all_sensors[i].value.i, all_sensors[i].label);
        }
        // Добавляем строку в общий буфер, если есть место
        if (strlen(data) + strlen(sensor_line) < max_len - 3) {
            strcat(data, sensor_line);
        }
    }

	// Выгрузка истории (если она есть)
    char *one_packet_buf = malloc(2048);//[2048];

    // Добавляем из истории, пока get_one_flash_packet_string говорит "True"
    // Мы передаем (max_len - текущая длина - запас под "##")
    while (get_one_flash_packet_string(one_packet_buf, max_len - strlen(data) - 3)) {
        strcat(data, one_packet_buf);
    }
	free(one_packet_buf);

    strcat(data, "##");
}

#define EEPROM_ADDR_HEAD 0x00
#define EEPROM_ADDR_TAIL 0x04

void save_head_to_eeprom(uint32_t head) 
{
    mcp_eeprom_write_bytes(EEPROM_ADDR_HEAD, (uint8_t*)&head, 4);
}

void save_tail_to_eeprom(uint32_t tail) 
{
    // 1. Проверяем: изменился ли хвост по сравнению с тем, что уже в EEPROM?
    if (tail == prev_tail_addr) {
        // Данные не изменились (выгрузки логов не было), ничего не пишем
        return; 
    }

    // 2. Если изменился — пишем в "железо"
    esp_err_t err = mcp_eeprom_write_bytes(0x04, (uint8_t*)&tail, 4);
    if (err == ESP_OK) {
        prev_tail_addr = tail; // Запоминаем новое стабильное состояние
        ESP_LOGI("EEPROM", "Tail updated in EEPROM to: %u", tail);
    }
}

uint32_t read_head_from_eeprom() 
{
    uint32_t head = 0;
	mcp_eeprom_read_bytes(EEPROM_ADDR_HEAD, (uint8_t*)&head, 4);
    
    // Если EEPROM пустой (FF) или адрес больше размера флешки
    if (head >= FLASH_TOTAL_SIZE) {
        ESP_LOGW("EEPROM", "Invalid head in EEPROM, resetting to 0");
        return 0;
    }
    return head;
}

uint32_t read_tail_from_eeprom() 
{
    uint32_t tail = 0;
	mcp_eeprom_read_bytes(EEPROM_ADDR_TAIL, (uint8_t *)&tail, 4);
    
    if (tail >= FLASH_TOTAL_SIZE) {
        ESP_LOGW("EEPROM", "Invalid tail in EEPROM, resetting to 0");
        return 0;
    }
    return tail;
}

void init_flash_logger() 
{
    // 1. Инициализируем SPI для флешки
    init_external_flash_spi();
    
    // 2. Читаем последние координаты из EEPROM
    current_head_addr = read_head_from_eeprom();
    current_tail_addr = read_tail_from_eeprom();
	prev_tail_addr = current_tail_addr;
    
    ESP_LOGI("LOGGER", "Flash Logger Ready. Head: %u, Tail: %u", 
             current_head_addr, current_tail_addr);
}

