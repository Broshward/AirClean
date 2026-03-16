#include "esp_https_ota.h"
#include "esp_log.h"
#include "esp_crt_bundle.h"

#define UPDATE_SERVER "https://192.168.1.75:8284/build/blufi_demo.bin"

// Эти переменные создаются автоматически благодаря target_add_binary_data
extern const uint8_t server_cert_pem_start[] asm("_binary_server_cert_pem_start");
extern const uint8_t server_cert_pem_end[]   asm("_binary_server_cert_pem_end");

const char* server_cert = "-----BEGIN CERTIFICATE-----\n"
"MIIBgzCCASmgAwIBAgIUb85Ye6tCio937t3ASl06i4h9KfwwCgYIKoZIzj0EAwIw\n"
"FzEVMBMGA1UEAwwMMTkyLjE2OC4xLjc1MB4XDTI2MDMxNjA5MjUyNVoXDTM2MDMx\n"
"MzA5MjUyNVowFzEVMBMGA1UEAwwMMTkyLjE2OC4xLjc1MFkwEwYHKoZIzj0CAQYI\n"
"KoZIzj0DAQcDQgAElOPUiHZNmq6BRUX0OfOew5gq+fw2Pijy/67Fk7Fpdym1Ul0l\n"
"PLKdGVqWOpzdQRMO+5eUOKHo8+VAHNUeuA8N6qNTMFEwHQYDVR0OBBYEFLIRjitb\n"
"WukFNgUF5+FC7N+xkNKlMB8GA1UdIwQYMBaAFLIRjitbWukFNgUF5+FC7N+xkNKl\n"
"MA8GA1UdEwEB/wQFMAMBAf8wCgYIKoZIzj0EAwIDSAAwRQIgJv2luXULT6xYeNTz\n"
"CB8MVQ7Is369/MnD3oLfcohNFZACIQCjBgOmgmE8Zq0LAzeVHIo6bGXCbVHUhSMz\n"
"4z2V8r1oVg==\n"
"-----END CERTIFICATE-----";

void run_ota_update_secure() 
{
    esp_http_client_config_t config = {
        .url = UPDATE_SERVER, // Теперь HTTPS
        .cert_pem = server_cert, // Указатель на вшитый сертификат
    .skip_cert_common_name_check = true, // Обязательно для самоподписанных!
    .transport_type = HTTP_TRANSPORT_OVER_SSL,

//		.crt_bundle_attach = esp_crt_bundle_attach,
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
