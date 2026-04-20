#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h" // Все UUID здесь

#include "gatts.h"
#include "sensor.h"
#include "blufi.h"
#include "times.h"
#include "ota.h"

// Глобальные переменные для нашего нового канала
uint16_t sensor_handle;
uint16_t command_handle;
esp_gatt_if_t sensor_gatt_if = ESP_GATT_IF_NONE;

uint16_t sensor_conn_id = 0xffff;

void sensor_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) 
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            if (param->reg.app_id == 1) { // Наш ID
                sensor_gatt_if = gatts_if;
                
                // Используем структуру для создания сервиса
                esp_gatt_srvc_id_t service_id = {
                    .is_primary = true,
                    .id.inst_id = 0x00,
                    .id.uuid.len = ESP_UUID_LEN_16,
                    .id.uuid.uuid.uuid16 = 0x00FF, // UUID сервиса
                };
                esp_ble_gatts_create_service(gatts_if, &service_id, 6); 
            }
            break;

        case ESP_GATTS_CREATE_EVT:
            // Сервис создан, запускаем его и добавляем характеристику
            esp_ble_gatts_start_service(param->create.service_handle);
            
            esp_bt_uuid_t char_uuid = {
                .len = ESP_UUID_LEN_16,
                .uuid.uuid16 = 0xFF01, // UUID характеристики
            };
            
            // Права: Чтение + Уведомление (Notify)
            esp_ble_gatts_add_char(param->create.service_handle, &char_uuid, 
                                   ESP_GATT_PERM_READ, 
                                   ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY, 
                                   NULL, NULL);
            break;

		case ESP_GATTS_ADD_CHAR_EVT:
			if (param->add_char.char_uuid.uuid.uuid16 == 0xFF01) {
				sensor_handle = param->add_char.attr_handle;
				// Только ТЕПЕРЬ добавляем ВТОРУЮ характеристику
				esp_bt_uuid_t char2_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = 0xFF02};
				esp_ble_gatts_add_char(param->add_char.service_handle, &char2_uuid,
									   ESP_GATT_PERM_WRITE,
									   ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR, NULL, NULL);
			} else {
				command_handle = param->add_char.attr_handle;
				ESP_LOGI("GATTS", "Обе трубы готовы! 01: %d, 02: %d", sensor_handle, command_handle);
			}
			break;
		
        case ESP_GATTS_CONNECT_EVT:
            sensor_conn_id = param->connect.conn_id;
            ESP_LOGI("GATTS", "Наш канал датчиков подключен!");
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            sensor_conn_id = 0xffff;
            break;

		case ESP_GATTS_WRITE_EVT:
		   if (param->write.need_rsp) {
				esp_gatt_rsp_t gatt_rsp;
				memset(&gatt_rsp, 0, sizeof(esp_gatt_rsp_t));
				gatt_rsp.attr_value.handle = param->write.handle;
				gatt_rsp.attr_value.len = param->write.len;
				gatt_rsp.attr_value.offset = param->write.offset;
				memcpy(gatt_rsp.attr_value.value, param->write.value, param->write.len);
				
				esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &gatt_rsp);
			}
			
			if (!param->write.is_prep) { // Простая запись (не длинная)
				// param->write.value - это массив байт от телефона
				// param->write.len - длина команды
				
				// Маленький лайфхак: завершим строку нулем для безопасности
				//char cmd_str[64];
				int len = param->write.len;
				char *cmd_str = (char *)malloc(len + 1);
				memcpy(cmd_str, param->write.value, len);
				cmd_str[len] = '\0';
				ESP_LOGI("GATTS", "Received command: %s", cmd_str);

				// Обрабатываем команду
				if (strcmp(cmd_str, "GET_SENSORS") == 0)
					send_sensors();
				else if (strcmp(cmd_str, "GET_NET") == 0)
					get_net();
				else if (strncmp(cmd_str, "SET_STATIC:", 11) == 0)
					set_static(cmd_str);
				else if (strcmp(cmd_str, "SET_DHCP") == 0)
					set_dhcp();
				else if (strcmp(cmd_str, "GET_SYNC_TIME") == 0) 
					send_last_sync_sntp();
				else if (strncmp(cmd_str, "SET_TZ:", 7) == 0) {
					int offset = atoi(cmd_str + 7);
					save_tz_to_nvs(offset);
					set_timezone(offset);
					// После смены пояса нужно обновить время в RTC, 
					// так как системное время изменилось!
					system_time_to_rtc();
				}
				if (strcmp(cmd_str, "START_OTA") == 0) {
					ESP_LOGI("OTA", " Запуск обновления...");
					// Вызываем функцию обновления
					run_ota_update_secure(); 
					//run_ota_update(); 
				}
				if (strcmp(cmd_str, "RESET") == 0) { // Program reset
					esp_restart();
				}


				else if (strcmp(cmd_str, "GET_LOGS") == 0) {
					 // Начинаем выгрузку флешки
				}
				free(cmd_str);
			}
			break;
		
        default:
            break;
    }
}

void send_ble_data(const char* data)
{
    if (sensor_gatt_if == ESP_GATT_IF_NONE || sensor_conn_id == 0xffff) return;

    if (xSemaphoreTake(ble_send_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        esp_ble_gatts_send_indicate(sensor_gatt_if, sensor_conn_id, sensor_handle, 
                                    strlen(data), (uint8_t *)data, false);
        xSemaphoreGive(ble_send_mutex);
    }
}

