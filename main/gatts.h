#include "esp_gatts_api.h"

void sensor_gatts_cb(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
void send_ble_data(const char* data);

extern SemaphoreHandle_t ble_send_mutex;
