#include <stdint.h>
#include "esp_err.h"


void configure_led(void);
void sensorsTask(void *pvParameters); 
void tcp_clientTask(void *pvParameters); // prototype of task function from tcp_client_v4.c file
void timeTask(void *pvParameters);
void spi_test(void *pvParameters);
void blufi_sender_task(void *pvParameters);

esp_err_t queue_blufi_data(uint8_t *data, size_t len); //Instead esp_blufi_send_custom_data()

#include "freertos/FreeRTOS.h"
extern TaskHandle_t tcptask;

extern float gl_luminosity;
extern float gl_temp;
extern float gl_chip_temp;
extern uint8_t gl_temperature[2];

typedef struct {
    uint8_t *payload; // Динамический буфер для данных
    size_t length;    // Длина данных
} blufi_msg_t;


