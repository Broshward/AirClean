#include "esp_https_ota.h"
#include "esp_log.h"

#define UPDATE_SERVER "http://192.168.1.75:8284/blufi_demo.bin"

// Эти переменные создаются автоматически благодаря target_add_binary_data
extern const uint8_t server_cert_pem_start[] asm("_binary_server_cert_pem_start");
extern const uint8_t server_cert_pem_end[]   asm("_binary_server_cert_pem_end");

void run_ota_update_secure() {
    esp_http_client_config_t config = {
        .url = "https://your-secure-server.com", // Теперь HTTPS
        .cert_pem = (const char *)server_cert_pem_start, // Указатель на вшитый сертификат
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };

    ESP_LOGI("OTA", "Начинаю обновление...");
    esp_err_t ret = esp_https_ota(&ota_config);
    
    if (ret == ESP_OK) {
        ESP_LOGI("OTA", "Успешно! Перезагрузка...");
        esp_restart();
    } else {
        ESP_LOGE("OTA", "Ошибка обновления!");
    }
}

void run_ota_update() 
{
    esp_http_client_config_t config = {
        .url = UPDATE_SERVER, 
	//	.port = 8284,
        .timeout_ms = 5000,
        .keep_alive_enable = true,		
    };
	esp_https_ota_config_t conf = {
		.http_config = &config,
	};

    ESP_LOGI("OTA", "Начинаю обновление...");
    esp_err_t ret = esp_https_ota(&conf);
    
    if (ret == ESP_OK) {
        ESP_LOGI("OTA", "Успешно! Перезагрузка...");
        esp_restart();
    } else {
        ESP_LOGE("OTA", "Ошибка обновления!");
    }
}
