#include <string.h>
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h" // Все UUID здесь

//#include <string.h>
//#include "esp_log.h"
//#include "esp_gatts_api.h"

// Глобальные переменные для нашего нового канала
uint16_t sensor_handle;
esp_gatt_if_t sensor_gatt_if = ESP_GATT_IF_NONE;
uint16_t sensor_conn_id = 0xffff;

SemaphoreHandle_t ble_send_mutex;

void sensor_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) 
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            if (param->reg.app_id == 1) { // Наш ID
                sensor_gatt_if = gatts_if;
                
                // Исправлено: используем правильную структуру для создания сервиса
                esp_gatt_srvc_id_t service_id = {
                    .is_primary = true,
                    .id.inst_id = 0x00,
                    .id.uuid.len = ESP_UUID_LEN_16,
                    .id.uuid.uuid.uuid16 = 0x00FF, // UUID сервиса
                };
                esp_ble_gatts_create_service(gatts_if, &service_id, 4); 
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
            sensor_handle = param->add_char.attr_handle;
            ESP_LOGI("GATTS", "Характеристика датчиков создана, handle: %d", sensor_handle);
            break;

        case ESP_GATTS_CONNECT_EVT:
            sensor_conn_id = param->connect.conn_id;
            ESP_LOGI("GATTS", "Наш канал датчиков подключен!");
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            sensor_conn_id = 0xffff;
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

