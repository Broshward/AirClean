#include <stdint.h>


void configure_led(void);
void sensorsTask(void *pvParameters); 
void tcp_clientTask(void *pvParameters); // prototype of task function from tcp_client_v4.c file
void timeTask(void *pvParameters);
void spi_test(void *pvParameters);

#include "freertos/FreeRTOS.h"
extern TaskHandle_t tcptask;

extern float gl_luminosity;
extern float gl_temp;
extern float gl_chip_temp;
extern uint8_t gl_temperature[2];
